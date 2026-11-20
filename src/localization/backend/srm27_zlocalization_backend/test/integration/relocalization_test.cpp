// 地点召回与真实配准共享固定场景，验收的是已知地图到里程计变换。
#include <gtest/gtest.h>

#include <set>

#include "backend/scan_context.hpp"
#include "backend/search.hpp"
#include "scene.hpp"

namespace backend {
namespace {
/// 为地点检索与几何验证提供独立的地图、里程计和真实传感器位姿。
class RelocalizationSearch : public ::testing::Test {
protected:
    Cloud::Ptr world = testing::roomCloud();
    KeyframeMap prior;
    ScanContextDescriptor descriptor;
    RelocalizationTargetCache cache{8, 8U * 1024U * 1024U};
    RelocalizationConfig config;
    Keyframe runtime;
    std::vector<Eigen::Vector3d> positions;
    Eigen::Matrix4d truth = testing::scenePose(.2, -.15, .1, .12);

    /// 构造房间先验及具有已知全局偏移的运行扫描。
    void SetUp() override {
        ScanContextDescriptorConfig descriptor_config;
        descriptor_config.augment_count = 1;
        descriptor_config.downsample = .2;
        descriptor.configure(descriptor_config);
        const auto map_frame = testing::sceneFrame(*world, 1, Eigen::Matrix4d::Identity(),
                                                   Eigen::Matrix4d::Identity());
        prior.add(map_frame);
        rebuildMapDescriptors(descriptor, prior.snapshot());
        positions = prior.positions();
        config.relocalization_parallel_workers = 1;
        config.relocalization_yaw_sweep_count = 4;
        config.point_to_plane_config.max_iterations = 16;
        const auto source =
            testing::sceneFrame(*world, 10, truth, testing::scenePose(-1, 2, .3, -.1));
        runtime.index = 0;
        runtime.timestamp = source.timestamp;
        runtime.odom_pose = source.odom_pose;
        runtime.cloud = CloudBlock(source.cloud_local);
    }
    /// 检查恢复位姿与独立真值一致，并具备有限正方差。
    void expectCorrectPose(const RelocalizationResult& result) const {
        ASSERT_TRUE(result.success);
        EXPECT_LT((result.T_map_odom * runtime.odom_pose - truth).norm(), 2e-4);
        EXPECT_TRUE(result.covariance_valid);
        EXPECT_TRUE(result.covariance.allFinite());
        EXPECT_GT(result.covariance.diagonal().minCoeff(), 0);
    }
};

/// 通过描述子检索和几何验证恢复已知的 odom 到 map 变换。
TEST_F(RelocalizationSearch, DescriptorRetrievalProducesVerifiedMapToOdometryTransform) {
    const auto revision = prior.revision();
    const auto descriptor_revision = descriptor.revision();
    RelocalizationRequest request{runtime, config, positions};
    const auto result = runRelocalization(prior, descriptor, cache, request);
    expectCorrectPose(result);
    EXPECT_EQ(prior.revision(), revision);
    EXPECT_EQ(descriptor.revision(), descriptor_revision);
}

/// 局部范围没有关键帧时，仅在配置全局补充候选后继续检索。
TEST_F(RelocalizationSearch, EmptyLocalNeighborhoodRequiresExplicitGlobalFallback) {
    RelocalizationRequest request{runtime, config, positions};
    request.T_map_runtime_pred = testing::scenePose(40, 40, 0, 0);
    request.search_radius = 1;
    EXPECT_FALSE(runRelocalization(prior, descriptor, cache, request).success);
    request.global_fallback_candidates = 1;
    expectCorrectPose(runRelocalization(prior, descriptor, cache, request));
}

/// 二维位置先验按水平距离验收，并在初值与结果中计入天线杆臂。
TEST_F(RelocalizationSearch, HorizontalPositionPriorIgnoresHeightAndAccountsForLeverArm) {
    RelocalizationPositionPrior position;
    position.source = "antenna";
    position.use_z = false;
    position.runtime_sensor_point = Eigen::Vector3d(.3, -.2, .1);
    position.map_point =
        truth.block<3, 3>(0, 0) * position.runtime_sensor_point + truth.block<3, 1>(0, 3);
    position.map_point.z() = 1000;
    position.yaw_seed_rad = .1;
    position.search_radius = .1;
    position.max_candidates = 1;
    RelocalizationRequest request{runtime, config, positions};
    request.T_map_runtime_pred = truth;
    request.position_prior = &position;
    expectCorrectPose(runRelocalization(prior, descriptor, cache, request));
    position.map_point.x() += .3;
    position.search_radius = .05;
    EXPECT_FALSE(runRelocalization(prior, descriptor, cache, request).success);
}

/// 退化的单平面几何应使完整重定位位姿锚验收失败。
TEST_F(RelocalizationSearch, DegeneratePlaneCannotEstablishFullPoseAnchor) {
    auto plane = Cloud::Ptr(new Cloud);
    for (const auto& point : *world)
        if (point.z == -2.F)
            plane->push_back(point);
    prior.clear();
    auto input =
        testing::sceneFrame(*plane, 1, Eigen::Matrix4d::Identity(), Eigen::Matrix4d::Identity());
    prior.add(input);
    rebuildMapDescriptors(descriptor, prior.snapshot());
    positions = prior.positions();
    runtime.cloud = CloudBlock(input.cloud_local);
    runtime.odom_pose.setIdentity();
    RelocalizationPositionPrior position;
    position.source = "position";
    position.search_radius = 1;
    position.max_candidates = 1;
    position.yaw_seed_rad = 0;
    RelocalizationRequest request{runtime, config, positions};
    request.position_prior = &position;
    EXPECT_FALSE(runRelocalization(prior, descriptor, cache, request).success);
}

/// 同一候选集合的串行与并行验证应选出相同位姿和协方差。
TEST_F(RelocalizationSearch, ParallelCandidateVerificationAgreesWithSerialResult) {
    const auto second_pose = testing::scenePose(.3, 0, 0, 0);
    prior.add(testing::sceneFrame(*world, 2, second_pose, second_pose));
    rebuildMapDescriptors(descriptor, prior.snapshot());
    positions = prior.positions();
    RelocalizationRequest request{runtime, config, positions};
    const auto serial = runRelocalization(prior, descriptor, cache, request);
    config.relocalization_parallel_workers = 4;
    const auto parallel = runRelocalization(prior, descriptor, cache, request);
    expectCorrectPose(serial);
    expectCorrectPose(parallel);
    EXPECT_EQ(parallel.prior_index, serial.prior_index);
    EXPECT_LT((parallel.T_map_odom - serial.T_map_odom).norm(), 1e-10);
}

/// 检查有无航向初值两种调度均覆盖全部离散航向，且每个航向只验证一次。
TEST(RelocalizationScheduling, YawSchedulesVisitEachDiscreteHeadingOnce) {
    for (int count : {1, 4, 5, 12}) {
        const auto unseeded = makeYawSweepSchedule(count, false);
        const auto seeded = makeYawSweepSchedule(count, true);
        ASSERT_EQ(seeded.primary_offsets, (std::vector<int>{0}));
        auto combined = seeded.primary_offsets;
        combined.insert(combined.end(), seeded.fallback_offsets.begin(),
                        seeded.fallback_offsets.end());
        EXPECT_EQ(combined, unseeded.primary_offsets);
        std::set<int> headings;
        for (int offset : combined)
            headings.insert((offset % count + count) % count);
        EXPECT_EQ(headings.size(), static_cast<std::size_t>(count));
    }
}
}  // namespace
}  // namespace backend
