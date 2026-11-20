// 使用解析位姿链与合成几何检验图估计、降秩似然及回环共识。
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

#include "backend/loop_consistency.hpp"
#include "backend/pose_graph.hpp"
#include "backend/registration.hpp"
#include "backend/uwb_alignment.hpp"
#include "geometry.hpp"

namespace backend {
namespace {
/// 用解析位姿链检查里程计图估计及累计运动带来的不确定度增长。
TEST(PoseGraph, OdometryChainRecoversGeometryAndUncertaintyGrows) {
    PoseGraph graph;
    graph.addPrior(0, Eigen::Matrix4d::Identity(), PoseCovariance6d::Identity() * .01, 1);
    graph.update(false);
    for (int index = 1; index <= 4; ++index) {
        graph.addOdometryMeasurement(index - 1, index, testing::translation(1),
                                     testing::translation(index + .1),
                                     PoseCovariance6d::Identity() * .01, index + 1);
        graph.update(false);
    }
    const auto pose = graph.estimate(4);
    ASSERT_TRUE(pose);
    EXPECT_LT((*pose - testing::translation(4)).norm(), 1e-7);
    const auto first_covariance = graph.marginalCovariance(0);
    const auto last_covariance = graph.marginalCovariance(4);
    ASSERT_TRUE(first_covariance);
    ASSERT_TRUE(last_covariance);
    EXPECT_GT((*last_covariance)(0, 0), (*first_covariance)(0, 0));
    EXPECT_FALSE(graph.estimate(999));
    graph.reset({});
    EXPECT_FALSE(graph.hasEstimate(4));
}

/// 固定滞后窗口移除过期节点后，当前位姿及其边缘协方差仍可查询。
TEST(PoseGraph, FixedLagRetiresOldStatesWithoutLosingCurrentMotion) {
    PoseGraphConfig config;
    config.fixed_lag_seconds = 2;
    PoseGraph graph(config);
    graph.addPrior(0, Eigen::Matrix4d::Identity(), 1);
    graph.update(false);
    for (int index = 1; index <= 8; ++index) {
        graph.addOdometryMeasurement(index - 1, index, testing::translation(1),
                                     testing::translation(index),
                                     PoseCovariance6d::Identity() * .01, index + 1);
        graph.update(false);
    }
    EXPECT_FALSE(graph.hasEstimate(0));
    EXPECT_FALSE(graph.marginalCovariance(0));
    const auto current = graph.estimate(8);
    ASSERT_TRUE(current);
    EXPECT_LT((*current - testing::translation(8)).norm(), 1e-7);
    EXPECT_TRUE(graph.marginalCovariance(8));
}

/// 构造具有精确二进制坐标的窗口轨迹，避免观测残差引发额外重线性化。
Eigen::Matrix4d windowPose(int index) {
    return testing::translation(.25 * index, .125 * (index % 3), .0625 * (index % 5));
}

/// 按解析匀速局部轨迹生成同时关联空间、高度与时间标定量的 UWB 观测。
UwbMeasurement windowMeasurement(int index, double time_offset) {
    const Eigen::Vector3d velocity(.25, index % 2 ? .125 : -.125, .0625);
    UwbTimeOffsetSupport support;
    support.measurement_timestamp = index;
    support.antenna_position_sensor = Eigen::Vector3d(.5, -.25, .125);
    support.odometry = {
        {index - 1.0, testing::translation(-velocity.x(), -velocity.y(), -velocity.z())},
        {index + 1.0, testing::translation(velocity.x(), velocity.y(), velocity.z())}};
    UwbMeasurement measurement;
    measurement.with_alignment = true;
    measurement.time_support = support;
    measurement.covariance = Eigen::Matrix3d::Identity() * .0625;
    measurement.position = windowPose(index).block<3, 1>(0, 3) + support.antenna_position_sensor +
                           time_offset * velocity;
    return measurement;
}

/// 以固定线性化点的完整图为参照，验证迟到观测下窗口边缘化保留共享标定及交叉信息。
TEST(PoseGraph, FixedLagPreservesSharedCalibrationAndJointInnovationWithDelayedObservations) {
    constexpr int kWindowSeconds = 12;
    constexpr double kTimeOffset = .125;
    PoseGraphConfig config;
    config.fixed_lag_seconds = kWindowSeconds;
    config.use_robust_kernel = false;
    PoseGraph window(config);
    config.fixed_lag_seconds = 0;
    PoseGraph reference(config);
    const Eigen::Matrix4d alignment = Eigen::Matrix4d::Identity();
    for (auto* graph : {&window, &reference}) {
        graph->addPrior(0, windowPose(0), PoseCovariance6d::Identity() * .01, 0);
        graph->update(false);
    }

    std::mt19937 random(20261020);
    for (int index = 1; index <= 120; ++index) {
        SCOPED_TRACE(index);
        const auto pose = windowPose(index);
        std::vector<int> delayed;
        for (unsigned sample = 0, count = random() % 4; sample < count; ++sample)
            delayed.push_back(index -
                              static_cast<int>(random() % (std::min(index, kWindowSeconds) + 1)));

        // 两图使用同一批迟到目标；随机性只选择存活图键，不扰动测量和线性化点。
        for (auto* graph : {&window, &reference}) {
            graph->addOdometryMeasurement(index - 1, index, windowPose(index - 1).inverse() * pose,
                                          pose, PoseCovariance6d::Identity() * .01, index);
            if (index == 1) {
                graph->addUwbAlignmentPrior(alignment, PoseCovariance6d::Identity() * .04, true);
                graph->addUwbTimeOffsetPrior(kTimeOffset, .2);
                for (int old = 0; old <= index; ++old)
                    graph->addUwbPosition(old, windowMeasurement(old, kTimeOffset), true);
            }
            if (index % 12 < 4)
                graph->addUwbPosition(index, windowMeasurement(index, kTimeOffset), true);
            for (const int old : delayed)
                graph->addUwbPosition(old, windowMeasurement(old, kTimeOffset), true);
            graph->update(true);
        }
        if (index > kWindowSeconds) {
            EXPECT_FALSE(window.hasEstimate(index - kWindowSeconds - 1));
            EXPECT_TRUE(reference.hasEstimate(index - kWindowSeconds - 1));
        }

        const auto current = window.estimate(index);
        const auto covariance = window.marginalCovariance(index);
        const auto expected_covariance = reference.marginalCovariance(index);
        ASSERT_TRUE(current);
        ASSERT_TRUE(covariance);
        ASSERT_TRUE(expected_covariance);
        EXPECT_LT((*current - pose).norm(), 1e-9);
        EXPECT_LT((*covariance - *expected_covariance).norm(), 1e-8);

        const auto spatial = window.uwbAlignmentEstimate();
        const auto spatial_covariance = window.uwbAlignmentMarginalCovariance();
        const auto expected_spatial_covariance = reference.uwbAlignmentMarginalCovariance();
        const auto temporal = window.uwbTimeOffsetEstimate();
        const auto temporal_variance = window.uwbTimeOffsetMarginalVariance();
        const auto expected_temporal_variance = reference.uwbTimeOffsetMarginalVariance();
        ASSERT_TRUE(spatial);
        ASSERT_TRUE(spatial_covariance);
        ASSERT_TRUE(expected_spatial_covariance);
        ASSERT_TRUE(temporal);
        ASSERT_TRUE(temporal_variance);
        ASSERT_TRUE(expected_temporal_variance);
        EXPECT_LT((*spatial - alignment).norm(), 1e-9);
        EXPECT_NEAR(*temporal, kTimeOffset, 1e-9);
        EXPECT_LT((*spatial_covariance - *expected_spatial_covariance).norm(), 1e-8);
        EXPECT_NEAR(*temporal_variance, *expected_temporal_variance, 1e-8);

        // 查询探针不入图；联合位姿和 UWB 创新同时约束当前位姿与全部共享变量的相关性。
        auto measurement = windowMeasurement(index, kTimeOffset);
        measurement.position += Eigen::Vector3d(.0625, -.03125, .015625);
        const Eigen::Matrix4d probe = pose * testing::translation(-.125, .0625, -.03125);
        const PoseSqrtInformation information = PoseSqrtInformation::Identity(6, 6) * 4;
        const auto actual =
            window.assessInnovationBatch({window.linearizeAbsolutePose(index, probe, information),
                                          window.linearizeUwbPosition(index, measurement, true)});
        const auto expected = reference.assessInnovationBatch(
            {reference.linearizeAbsolutePose(index, probe, information),
             reference.linearizeUwbPosition(index, measurement, true)});
        ASSERT_TRUE(actual.joint.valid) << actual.joint.invalid_reason;
        ASSERT_TRUE(expected.joint.valid) << expected.joint.invalid_reason;
        ASSERT_EQ(actual.individual.size(), 2U);
        EXPECT_EQ(actual.joint.degrees_of_freedom, 9);
        EXPECT_NEAR(actual.joint.nis, expected.joint.nis, 1e-8);
        EXPECT_GT(std::abs(actual.joint.nis - actual.individual[0].nis - actual.individual[1].nis),
                  1e-6);
    }
}

/// 秩一位置观测仅约束对应平移方向，其余方向沿用图预测。
TEST(PoseGraph, RankOneObservationConstrainsOnlyMeasuredTranslation) {
    PoseGraph graph;
    graph.addPrior(0, Eigen::Matrix4d::Identity(), PoseCovariance6d::Identity(), 1);
    graph.update(false);
    PoseSqrtInformation information = PoseSqrtInformation::Zero(1, 6);
    information(0, 0) = 2;
    graph.addAbsolutePoseInformation(0, testing::translation(.2), information);
    graph.update(true);
    const auto pose = graph.estimate(0);
    const auto covariance = graph.marginalCovariance(0);
    ASSERT_TRUE(pose);
    ASSERT_TRUE(covariance);
    EXPECT_NEAR((*pose)(0, 3), .16, 1e-6);
    EXPECT_NEAR((*covariance)(0, 0), .2, 1e-6);
    EXPECT_NEAR((*covariance)(1, 1), 1, 1e-6);
}

/// 单平面配准提供法向约束，但无法通过完整位姿锚的验收。
TEST(Registration, PlaneConstrainsNormalMotionWithoutInventingFullPoseAnchor) {
    RegistrationConfig config;
    config.voxel_resolution = 0;
    config.min_correspondences = 20;
    config.plane_knn = 8;
    Registration registration(config);
    const auto cloud = testing::planeCloud();
    ASSERT_TRUE(registration.setTarget(cloud, 0));
    const auto result = registration.align(*cloud, testing::translation(0, 0, .04));
    ASSERT_TRUE(result.likelihoodValid());
    EXPECT_NEAR(result.source_to_target(2, 3), 0, 1e-5);
    EXPECT_LT(result.measurement_information.rank, 6);
    EXPECT_FALSE(result.anchorValid());
    const auto scaled = registration.align(*cloud, testing::translation(0, 0, .04), .25);
    ASSERT_TRUE(scaled.likelihoodValid());
    EXPECT_LT((scaled.source_to_target - result.source_to_target).norm(), 1e-9);
    const auto precision = result.measurement_information.sqrt_information.transpose() *
                           result.measurement_information.sqrt_information;
    const auto scaled_precision = scaled.measurement_information.sqrt_information.transpose() *
                                  scaled.measurement_information.sqrt_information;
    EXPECT_LT(scaled_precision.trace(), precision.trace());
}

/// 回环候选先按一致集合规模筛选，再在集合内按残差选择。
TEST(LoopConsensus, SelectsLargestGroupBeforeBestIndividualResidual) {
    LoopClosureConfig config;
    config.enable_pcm = true;
    config.pcm_translation_threshold = .2;
    LoopClosureVerifier verifier(config);
    std::vector<VerifiedLoop> candidates(5);
    for (int index = 0; index < 5; ++index) {
        candidates[index].target_keyframe.odom_pose = testing::translation(index);
        candidates[index].measured_between = testing::translation(index + (index < 3 ? 0 : 10));
        candidates[index].registration.rmse = index < 3 ? .2 : .01;
        candidates[index].candidate.descriptor_distance = .1;
    }
    auto selected = verifier.selectConsistentCandidateClique(candidates);
    std::sort(selected.begin(), selected.end());
    EXPECT_EQ(selected, (std::vector<int>{0, 1, 2}));
    candidates[2].target_keyframe.epoch = 1;
    EXPECT_FALSE(verifier.areSameSourceLoopsConsistent(candidates[0], candidates[2]));
}

/// 区分已有历史下的一致边数量要求与没有可比较历史时的启动放行。
TEST(LoopConsensus, HistoricalSupportThresholdAndMissingHistoryHaveSeparateMeaning) {
    LoopClosureConfig config;
    config.min_consistent_loops = 2;
    config.pcm_translation_threshold = .2;
    LoopClosureVerifier verifier(config);
    Keyframe latest, candidate;
    const auto lookup = [](int, Keyframe* out) {
        *out = Keyframe{};
        return true;
    };
    EXPECT_TRUE(verifier.passesAcceptedConsistency(latest, candidate, Eigen::Matrix4d::Identity(),
                                                   {}, lookup));
    std::vector<AcceptedLoop> history{{0, 1, Eigen::Matrix4d::Identity()}};
    EXPECT_FALSE(verifier.passesAcceptedConsistency(latest, candidate, Eigen::Matrix4d::Identity(),
                                                    history, lookup));
    history.push_back({2, 3, Eigen::Matrix4d::Identity()});
    EXPECT_TRUE(verifier.passesAcceptedConsistency(latest, candidate, Eigen::Matrix4d::Identity(),
                                                   history, lookup));
    EXPECT_FALSE(verifier.passesAcceptedConsistency(latest, candidate, testing::translation(3),
                                                    history, lookup));
}

/// 用已知平面变换验证 UWB 对齐，并检查内点保留原始测量时间戳。
TEST(UwbAlignment, RecoversKnownPlanarTransformAndRetainsOriginalTimestamps) {
    UwbAlignmentConfig config;
    UwbAlignmentBootstrapper bootstrap(config);
    const Eigen::Matrix3d rotation =
        Eigen::AngleAxisd(.4, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const Eigen::Vector3d translation(4, -2, 0);
    std::optional<UwbAlignmentEstimate> result;
    for (int index = 0; index < 20; ++index) {
        const Eigen::Vector3d odom(.4 * index, .3 * (index % 3), 0);
        result = bootstrap.addPair(10 + .5 * index, odom, rotation * odom + translation);
    }
    ASSERT_TRUE(result);
    EXPECT_TRUE(result->observable);
    EXPECT_TRUE(result->stable);
    EXPECT_LT((result->T_map_odom.block<3, 3>(0, 0) - rotation).norm(), 1e-6);
    EXPECT_LT((result->T_map_odom.block<2, 1>(0, 3) - translation.head<2>()).norm(), 1e-6);
    EXPECT_EQ(result->inlier_timestamps.size(), 20U);
    EXPECT_DOUBLE_EQ(result->inlier_timestamps.back(), 19.5);
}
}  // namespace
}  // namespace backend
