// UWB 的物理解、图消费水位与观测接纳拥有不同生命周期。
#include <gtest/gtest.h>

#include "backend/observation.hpp"
#include "backend/pose_graph.hpp"
#include "backend/uwb_fusion.hpp"
#include "geometry.hpp"

namespace backend {
namespace {
TEST(BackendCapabilities, DefaultsOffAndAllEightSelectionsRemainIndependent) {
    const Config defaults;
    EXPECT_FALSE(defaults.prior_map_enabled);
    EXPECT_FALSE(defaults.loop.enabled);
    EXPECT_FALSE(defaults.uwb.enabled);
    for (int mask = 0; mask < 8; ++mask) {
        SCOPED_TRACE(mask);
        Config requested;
        requested.prior_map_enabled = (mask & 1) != 0;
        requested.loop.enabled = (mask & 2) != 0;
        requested.uwb.enabled = (mask & 4) != 0;
        const auto effective = resolveConfig(requested, true).values;
        EXPECT_EQ(effective.prior_map_enabled, requested.prior_map_enabled);
        EXPECT_EQ(effective.loop.enabled, requested.loop.enabled);
        EXPECT_EQ(effective.uwb.enabled, requested.uwb.enabled);
        EXPECT_DOUBLE_EQ(effective.graph.fixed_lag_seconds,
                         requested.loop.enabled ? 0.0 : requested.graph.fixed_lag_seconds);
    }
    Config unavailable;
    unavailable.prior_map_enabled = true;
    unavailable.loop.enabled = true;
    EXPECT_FALSE(resolveConfig(unavailable, false).values.loop.enabled);
}

/// 同一 UWB 位置解的高频保持重发不推进有效观测时间。
TEST(UwbStream, HeldTransmissionsDoNotCreateNewPhysicalSamples) {
    UwbObservationStream stream({true, false, 1});
    const Eigen::Vector3d position(1, 2, 0);
    ASSERT_TRUE(stream.ingest(10, position));
    EXPECT_FALSE(stream.ingest(10.1, position));
    EXPECT_DOUBLE_EQ(stream.latest()->timestamp, 10);
    EXPECT_FALSE(stream.ingest(9, position));
    EXPECT_TRUE(stream.ingest(10.2, Eigen::Vector3d(1.1, 2, 0)));
    EXPECT_DOUBLE_EQ(stream.latest()->timestamp, 10.2);
}

/// 用恒速运动与固定杆臂验证时间偏置下的天线位置和速度。
TEST(UwbTimeSupport, OffsetAndVelocityMatchAnalyticConstantMotionWithLeverArm) {
    UwbTimeOffsetSupport support;
    support.measurement_timestamp = 2;
    support.keyframe_odom_pose = testing::translation(1);
    support.antenna_position_sensor = Eigen::Vector3d(.3, .2, 0);
    support.odometry = {{1, testing::translation(0)}, {3, testing::translation(4)}};
    Eigen::Vector3d point, velocity;
    ASSERT_TRUE(support.evaluate(.25, &point, &velocity));
    EXPECT_LT((point - Eigen::Vector3d(1.8, .2, 0)).norm(), 1e-12);
    EXPECT_LT((velocity - Eigen::Vector3d(2, 0, 0)).norm(), 1e-12);
    Eigen::Vector3d plus, minus;
    ASSERT_TRUE(support.evaluate(.25 + 1e-5, &plus));
    ASSERT_TRUE(support.evaluate(.25 - 1e-5, &minus));
    EXPECT_LT(((plus - minus) / 2e-5 - velocity).norm(), 1e-9);
}

/// 换代和图重置保留 UWB 消费水位，完整重置清空观测历史。
TEST(UwbLifecycle, EpochAndGraphResetPreserveConsumptionButFullResetClearsIt) {
    PoseGraph graph;
    TimedPoseBuffer odometry;
    UwbFusionConfig config;
    config.uwb.enabled = true;
    UwbFusion fusion(config, graph, odometry);
    ASSERT_TRUE(fusion.ingest(10, Eigen::Vector3d(1, 2, 0)));
    UwbFactorInput input;
    input.timestamp = 10;
    InnovationAssessment rejected;
    rejected.valid = true;
    rejected.degrees_of_freedom = 2;
    rejected.upper_tail_probability = 0;
    fusion.finishAssessment(input, rejected);
    EXPECT_DOUBLE_EQ(fusion.lastConsumed(), 10);
    fusion.resetEpoch();
    ASSERT_TRUE(fusion.observations().latest());
    EXPECT_DOUBLE_EQ(fusion.observations().latest()->timestamp, 10);
    EXPECT_DOUBLE_EQ(fusion.lastConsumed(), 10);
    fusion.resetGraph();
    EXPECT_DOUBLE_EQ(fusion.lastConsumed(), 10);
    EXPECT_TRUE(fusion.observations().latest());
    fusion.resetAll();
    EXPECT_FALSE(fusion.observations().latest());
    EXPECT_LT(fusion.lastConsumed(), 10);
}

/// 缺少时间或几何支撑的观测保持不可用状态，不累计冲突。
TEST(ObservationPolicy, UnavailableEvidenceIsNotAConflictVote) {
    ObservationBatch batch;
    batch.unavailable.push_back("no temporal support");
    EXPECT_EQ(batch.conclusion().outcome, ObservationOutcome::Unavailable);
    EXPECT_FALSE(batch.conclusion().had_conflict);
}

/// 检查多来源观测的联合接纳，以及逐项诊断数量不完整时的拒绝。
TEST(ObservationPolicy, JointAssessmentControlsAcceptanceAcrossSources) {
    ObservationBatch batch;
    batch.entries = {{ObservationSource::PriorMap, "map", {}, false},
                     {ObservationSource::Uwb, "uwb", {}, false}};
    InnovationBatchAssessment assessment;
    assessment.joint = {true, 3, .2, .9, {}};
    assessment.individual = {{true, 1, .1, .9, {}}, {true, 2, .1, .9, {}}};
    ASSERT_TRUE(batch.validate(assessment));
    batch.classify(assessment, .001);
    EXPECT_EQ(batch.conclusion().outcome, ObservationOutcome::Accepted);
    EXPECT_TRUE(batch.entries[0].accepted);
    EXPECT_TRUE(batch.entries[1].accepted);
    ObservationBatch inconsistent;
    inconsistent.entries = batch.entries;
    assessment.individual.pop_back();
    EXPECT_FALSE(inconsistent.validate(assessment));
}
}  // namespace
}  // namespace backend
