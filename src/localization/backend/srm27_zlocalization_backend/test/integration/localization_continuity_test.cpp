// 验证停车、暖重置与跨代运动的位姿和协方差连续性。
#include "localization_fixture.hpp"

namespace backend {
namespace {
using testing::LocalizationFlow;

/// 停车首帧因速度异常未入图时，随后暖重置仍继承全局不确定度而非新前端规范先验。
TEST_F(LocalizationFlow, ParkedReferenceSurvivesVelocitySpikeBeforeWarmReset) {
    config.uwb.enabled = true;
    config.uwb.use_z = false;
    config.uwb.antenna_position_sensor = Eigen::Vector3d(.3, .1, 0);
    reset.available = true;
    start(false);
    FrontendFrame current;
    feedUwbTrajectory(testing::scenePose(3, -2, 0, .3), current);
    ASSERT_EQ(controller->status(), LocalizationStatus::Normal);
    const auto before = output(current);
    const auto size = trajectory.size();
    ASSERT_TRUE(controller->requestCoordinatedReset(/*assume_stationary=*/true));
    reply(FrontendResetOutcome::Completed, 1);
    const auto parked = frame(22, before.T_map_sensor, Eigen::Matrix4d::Identity(), 1);
    ASSERT_TRUE(ingest(parked, Eigen::Vector3d(config.tracking.suspicious_linear_speed + 1, 0, 0)));
    ASSERT_EQ(controller->status(), LocalizationStatus::Suspect);
    ASSERT_EQ(trajectory.size(), size);
    ASSERT_TRUE(controller->requestCoordinatedReset(/*assume_stationary=*/true));
    reply(FrontendResetOutcome::Completed, 2,
          FrontendResetBoundary{1, Timestamp::fromSeconds(22.1), Eigen::Matrix4d::Identity()});
    auto next = frame(22.2, before.T_map_sensor, Eigen::Matrix4d::Identity(), 2);
    next.pose_covariance = PoseCovariance6d::Identity() * 1e-4;
    ASSERT_TRUE(ingest(next));
    ASSERT_EQ(controller->status(), LocalizationStatus::Normal);
    EXPECT_EQ(trajectory.size(), size + 1);
    EXPECT_GT(output(next).pose_covariance(0, 0), before.pose_covariance(0, 0));
    EXPECT_LT((output(next).T_map_sensor - before.T_map_sensor).norm(), 1e-9);
}

/// 停车参考经过连续暖重置后，以独立三节点图验证旋转和平移对应的完整协方差。
TEST_F(LocalizationFlow, ParkedReferenceComposesWarmMotionAndMatchesIndependentGraph) {
    config.uwb.enabled = true;
    config.uwb.use_z = false;
    config.uwb.antenna_position_sensor = Eigen::Vector3d(.3, .1, 0);
    reset.available = true;
    start(false);
    FrontendFrame current;
    feedUwbTrajectory(testing::scenePose(3, -2, 0, .3), current);
    ASSERT_EQ(controller->status(), LocalizationStatus::Normal);
    const auto before = output(current);
    const auto size = trajectory.size();
    ASSERT_TRUE(controller->requestCoordinatedReset(/*assume_stationary=*/true));
    reply(FrontendResetOutcome::Completed, 1);
    const auto reference_odom = testing::scenePose(.15, -.1, 0, .2);
    const auto parked = frame(22, before.T_map_sensor, reference_odom, 1);
    ASSERT_TRUE(ingest(parked, Eigen::Vector3d(config.tracking.suspicious_linear_speed + 1, 0, 0)));
    ASSERT_EQ(controller->status(), LocalizationStatus::Suspect);
    const auto first_boundary = testing::scenePose(.3, .1, 0, .4);
    const auto second_boundary = testing::scenePose(.1, .1, 0, -.25);
    ASSERT_TRUE(controller->requestCoordinatedReset(/*assume_stationary=*/true));
    reply(FrontendResetOutcome::Completed, 2,
          FrontendResetBoundary{1, Timestamp::fromSeconds(22.2), first_boundary});
    ASSERT_TRUE(controller->requestCoordinatedReset(/*assume_stationary=*/true));
    reply(FrontendResetOutcome::Completed, 3,
          FrontendResetBoundary{2, Timestamp::fromSeconds(22.4), second_boundary});
    EXPECT_EQ(trajectory.size(), size);
    const auto local = testing::scenePose(.4, .2, .1, .2);
    const Eigen::Matrix4d motion =
        reference_odom.inverse() * first_boundary * second_boundary * local;
    const Eigen::Matrix4d expected_pose = before.T_map_sensor * motion;
    auto next = frame(23.2, expected_pose, local, 3);
    next.pose_covariance = PoseCovariance6d::Identity() * 1e-4;
    ASSERT_TRUE(ingest(next));
    ASSERT_EQ(trajectory.size(), size + 1);
    EXPECT_LT((output(next).T_map_sensor - expected_pose).norm(), 1e-8);

    // 显式保留停车参考节点，独立检验消去该节点后的均值和交叉协方差。
    PoseGraph reference(config.graph);
    reference.addPrior(
        0, before.T_map_sensor,
        worldAdditiveToLocalPoseCovariance(before.T_map_sensor, before.pose_covariance),
        current.timestamp);
    reference.update(false);
    PoseCovariance6d parked_covariance = config.graph.odom_noise.covariance(0, 0, 0);
    parked_covariance.diagonal().head<3>().array() +=
        config.reset.stationary_translation_std * config.reset.stationary_translation_std;
    parked_covariance.diagonal().tail<3>().array() +=
        config.reset.stationary_rotation_std_rad * config.reset.stationary_rotation_std_rad;
    reference.addResetContinuityMeasurement(0, 1, Eigen::Matrix4d::Identity(), before.T_map_sensor,
                                            parked_covariance, parked.timestamp);
    reference.update(false);
    const auto motion_covariance = config.graph.odom_noise.covariance(
        next.timestamp - parked.timestamp, motion.block<3, 1>(0, 3).norm(),
        rotationAngleFromMatrix(motion));
    reference.addOdometryMeasurement(1, 2, motion, expected_pose, motion_covariance,
                                     next.timestamp);
    reference.addGravityAlignment(2, next.odom_pose,
                                  config.uwb.bootstrap.gravity_alignment_std_rad);
    reference.update(false);
    const auto covariance = reference.marginalCovariance(2);
    ASSERT_TRUE(covariance);
    EXPECT_LT((output(next).pose_covariance -
               localToWorldAdditivePoseCovariance(expected_pose, *covariance))
                  .norm(),
              1e-7);
}

/// 首个新代次观测保留边界之后的实际位移，不把重置后的运动吸收到地图变换中。
TEST_F(LocalizationFlow, WarmBoundaryPreservesMotionBeforeTheFirstNewFrame) {
    reset.available = true;
    start(false);
    const auto initial_pose = testing::scenePose(2, 0, 0, 0);
    const auto first = frame(10, initial_pose, initial_pose);
    ASSERT_TRUE(ingest(first, Eigen::Vector3d::UnitX()));
    ASSERT_TRUE(controller->requestCoordinatedReset());
    reply(FrontendResetOutcome::Completed, 1,
          FrontendResetBoundary{0, Timestamp::fromSeconds(10), initial_pose});
    for (double elapsed : {.1, .2}) {
        const auto local = testing::scenePose(elapsed, 0, 0, 0);
        const Eigen::Matrix4d expected = initial_pose * local;
        const auto current = frame(10 + elapsed, expected, local, 1);
        ASSERT_TRUE(ingest(current, Eigen::Vector3d::UnitX()));
        const auto projected = output(current);
        ASSERT_TRUE(projected.valid);
        EXPECT_TRUE(projected.pose_trusted);
        EXPECT_NEAR(projected.T_map_sensor(0, 3), 2 + elapsed, 1e-10);
    }
}

/// 连续暖重置复合旋转规范，并在延迟首关键帧入图时覆盖整个物理运动区间。
TEST_F(LocalizationFlow, ConsecutiveWarmBoundariesPreserveDelayedGraphMotionAndUncertainty) {
    config.loop.enabled = true;
    config.keyframes.min_interval = .5;
    reset.available = true;
    const auto map_from_old = testing::scenePose(4, -3, 1, .6);
    const auto old_pose = testing::scenePose(1, .5, .2, -.3);
    const Eigen::Matrix4d initial_pose = map_from_old * old_pose;
    config.startup_hint.T_map_sensor = initial_pose;
    start(false);
    ASSERT_TRUE(ingest(frame(10, initial_pose, old_pose)));
    ASSERT_EQ(trajectory.size(), 1U);
    const auto newer_odom = testing::scenePose(1.1, .5, .2, -.2);
    ASSERT_TRUE(ingest(frame(10.1, map_from_old * newer_odom, newer_odom)));
    EXPECT_DOUBLE_EQ(trajectory.latest().timestamp, 10);

    const auto old_from_middle = testing::scenePose(1.2, .4, .2, .4);
    const auto middle_from_new = testing::scenePose(.2, -.1, 0, -.25);
    ASSERT_TRUE(controller->requestCoordinatedReset());
    reply(FrontendResetOutcome::Completed, 1,
          FrontendResetBoundary{0, Timestamp::fromSeconds(10.2), old_from_middle});
    ASSERT_TRUE(controller->requestCoordinatedReset());
    reply(FrontendResetOutcome::Completed, 2,
          FrontendResetBoundary{1, Timestamp::fromSeconds(10.4), middle_from_new});

    const auto new_pose = testing::scenePose(.4, .1, 0, .2);
    const Eigen::Matrix4d expected = map_from_old * old_from_middle * middle_from_new * new_pose;
    const auto current = frame(10.8, expected, new_pose, 2);
    ASSERT_TRUE(ingest(current));
    ASSERT_EQ(trajectory.size(), 2U);
    EXPECT_EQ(trajectory.latest().epoch, 2U);
    EXPECT_LT((trajectory.latest().optimized_pose - expected).norm(), 1e-9);
    const auto projected = output(current);
    ASSERT_TRUE(projected.pose_covariance_valid);
    EXPECT_LT((projected.T_map_sensor - expected).norm(), 1e-9);

    // 独立两节点图使用真实首尾位姿和完整 0.8 秒过程噪声，不加入停车重置噪声。
    PoseGraph reference(config.graph);
    reference.addPrior(0, initial_pose, 10);
    reference.update(false);
    const Eigen::Matrix4d measured_between = initial_pose.inverse() * expected;
    const auto relative_covariance = config.graph.odom_noise.covariance(
        .8, measured_between.block<3, 1>(0, 3).norm(), rotationAngleFromMatrix(measured_between));
    reference.addResetContinuityMeasurement(0, 1, measured_between, expected, relative_covariance,
                                            10.8);
    reference.update(false);
    const auto covariance = reference.marginalCovariance(1);
    ASSERT_TRUE(covariance);
    const auto expected_covariance = localToWorldAdditivePoseCovariance(expected, *covariance);
    EXPECT_LT((projected.pose_covariance - expected_covariance).norm(), 1e-9);
    const auto next_local = testing::scenePose(.9, .2, 0, .3);
    const Eigen::Matrix4d next_expected =
        map_from_old * old_from_middle * middle_from_new * next_local;
    const auto next = frame(11.5, next_expected, next_local, 2);
    ASSERT_TRUE(ingest(next));
    ASSERT_EQ(trajectory.size(), 3U);
    EXPECT_LT((trajectory.latest().optimized_pose - next_expected).norm(), 1e-9);
}

/// 非法边界时间不能改变坐标规范；终态仍换代并采用无运动边界的停车合同。
TEST_F(LocalizationFlow, StaleWarmBoundaryCannotSupplyANewCoordinateTransform) {
    reset.available = true;
    start(false);
    const auto initial_pose = testing::scenePose(2, 0, 0, 0);
    ASSERT_TRUE(ingest(frame(10, initial_pose, initial_pose)));
    ASSERT_TRUE(controller->requestCoordinatedReset(/*assume_stationary=*/true));
    reply(FrontendResetOutcome::Completed, 1,
          FrontendResetBoundary{0, Timestamp::fromSeconds(9), testing::scenePose(100, 0, 0, 0)});
    EXPECT_EQ(controller->activeEpoch(), 1U);
    const auto current = frame(10.1, initial_pose, testing::scenePose(.1, 0, 0, 0), 1);
    ASSERT_TRUE(ingest(current));
    EXPECT_LT((output(current).T_map_sensor - initial_pose).norm(), 1e-10);
}

/// 边界之前及同刻的新代次帧均不得消费种子，后续合法观测仍可正常入图。
TEST_F(LocalizationFlow, WarmBoundaryRejectsNoncausalFramesWithoutConsumingTheTransform) {
    reset.available = true;
    start(false);
    ASSERT_TRUE(ingest(frame(10, testing::scenePose(2, 0, 0, 0), testing::scenePose(2, 0, 0, 0))));
    const auto boundary_pose = testing::scenePose(2.2, 0, 0, 0);
    ASSERT_TRUE(controller->requestCoordinatedReset());
    reply(FrontendResetOutcome::Completed, 1,
          FrontendResetBoundary{0, Timestamp::fromSeconds(10.2), boundary_pose});
    for (double time : {10.1, 10.2})
        EXPECT_FALSE(ingest(frame(time, boundary_pose, testing::scenePose(100, 0, 0, 0), 1)));
    EXPECT_EQ(trajectory.size(), 1U);
    const auto local = testing::scenePose(.1, 0, 0, 0);
    const Eigen::Matrix4d expected = boundary_pose * local;
    const auto current = frame(10.3, expected, local, 1);
    ASSERT_TRUE(ingest(current));
    EXPECT_LT((output(current).T_map_sensor - expected).norm(), 1e-10);
    EXPECT_EQ(trajectory.size(), 2U);
}

/// 坐标边界只转换里程计规范，不为先验定位中已经丢失的状态恢复可信度。
TEST_F(LocalizationFlow, WarmBoundaryCannotMakeLostPriorLocalizationTrusted) {
    config.startup_hint.T_map_sensor = Eigen::Matrix4d::Identity();
    config.recovery.suspect_failures_before_lost = 1;
    config.recovery.automatic_when_lost = false;
    reset.available = true;
    start(true);
    const auto first = frame(10, Eigen::Matrix4d::Identity(), Eigen::Matrix4d::Identity());
    ASSERT_TRUE(ingest(first));
    maintain();
    ASSERT_EQ(controller->status(), LocalizationStatus::Normal);
    const auto trusted = output(first);
    auto sparse = frame(11, Eigen::Matrix4d::Identity(), Eigen::Matrix4d::Identity());
    sparse.cloud_local->resize(1);
    ASSERT_TRUE(ingest(sparse, Eigen::Vector3d(10, 0, 0)));
    maintain();
    ASSERT_EQ(controller->status(), LocalizationStatus::Lost);
    ASSERT_TRUE(controller->requestCoordinatedReset());
    const auto boundary_pose = testing::scenePose(.2, 0, 0, .1);
    reply(FrontendResetOutcome::Completed, 1,
          FrontendResetBoundary{0, Timestamp::fromSeconds(11.1), boundary_pose});
    const auto local = testing::scenePose(.1, 0, 0, 0);
    const auto current = frame(11.2, boundary_pose * local, local, 1);
    ASSERT_TRUE(ingest(current));
    maintain();
    EXPECT_EQ(controller->status(), LocalizationStatus::Lost);
    const auto projected = output(current);
    EXPECT_FALSE(projected.pose_trusted);
    EXPECT_LT((projected.T_map_sensor - trusted.T_map_sensor).norm(), 1e-10);
}
}  // namespace
}  // namespace backend
