// 验证输入中断、处理失败与冷启动后的恢复边界。
#include "localization_fixture.hpp"

namespace backend {
namespace {
using testing::LocalizationFlow;

/// 服务不可用时输入故障也立即冻结输出，重复通知不清图或重记故障代次。
TEST_F(LocalizationFlow, InputDiscontinuityFreezesBeforeResetServiceIsAvailable) {
    config.loop.enabled = true;
    start(false);
    const auto first = frame(10, testing::scenePose(2, 0, 0, 0), testing::scenePose(2, 0, 0, 0));
    ASSERT_TRUE(ingest(first));
    const auto before = output(first);
    const auto size = trajectory.size();
    controller->notifyFrontendDiscontinuity(0);
    ASSERT_EQ(controller->status(), LocalizationStatus::Lost);
    const auto generation = controller->snapshot().localization.generation;
    for (int attempt = 0; attempt < 3; ++attempt)
        controller->notifyFrontendDiscontinuity(0);
    EXPECT_EQ(controller->snapshot().localization.generation, generation);
    EXPECT_EQ(controller->snapshot().localization.estimator_failures.count, 0U);
    EXPECT_EQ(trajectory.size(), size);
    EXPECT_TRUE(reset.requests.empty());
    const auto later = frame(11, testing::scenePose(3, 0, 0, 0), testing::scenePose(3, 0, 0, 0));
    EXPECT_LT((output(later).T_map_sensor - before.T_map_sensor).norm(), 1e-12);
    reset.available = true;
    controller->notifyFrontendDiscontinuity(0);
    ASSERT_EQ(reset.requests.size(), 1U);
    controller->notifyFrontendDiscontinuity(0);
    EXPECT_EQ(reset.requests.size(), 1U);
    EXPECT_EQ(controller->snapshot().localization.generation, generation);
    reply(FrontendResetOutcome::Completed, 1);
    ASSERT_TRUE(ingest(frame(12, first.odom_pose, Eigen::Matrix4d::Identity(), 1)));
    ASSERT_EQ(controller->status(), LocalizationStatus::Lost);
    const auto resumed_generation = controller->snapshot().localization.generation;
    controller->notifyFrontendDiscontinuity(0);
    EXPECT_EQ(controller->status(), LocalizationStatus::Lost);
    EXPECT_EQ(controller->snapshot().localization.generation, resumed_generation);
    EXPECT_EQ(reset.requests.size(), 1U);
}

/// 在途手动重置被拒绝时，后续发生的输入故障仍保持冻结状态。
TEST_F(LocalizationFlow, InputDiscontinuityInvalidatesManualResetRollback) {
    reset.available = true;
    start(false);
    ASSERT_TRUE(ingest(frame(10, Eigen::Matrix4d::Identity(), Eigen::Matrix4d::Identity())));
    ASSERT_TRUE(controller->requestCoordinatedReset(/*assume_stationary=*/true));
    controller->notifyFrontendDiscontinuity(0);
    reply(FrontendResetOutcome::Rejected, 0);
    EXPECT_EQ(controller->status(), LocalizationStatus::Lost);
    EXPECT_EQ(controller->snapshot().localization.estimator_failures.count, 0U);
    EXPECT_EQ(trajectory.size(), 1U);
}

/// 未观测的跨代运动不能被自动冷启动或后续暖边界解释为停车连续性。
TEST_F(LocalizationFlow, AutomaticColdResetDoesNotReconnectAnExistingMap) {
    config.loop.enabled = true;
    reset.available = true;
    start(false);
    const auto first_pose = testing::scenePose(2, 0, 0, 0);
    const auto first = frame(10, first_pose, first_pose);
    ASSERT_TRUE(ingest(first));
    const auto before = output(first);
    controller->notifyFrontendDiscontinuity(0);
    reply(FrontendResetOutcome::Completed, 1);
    const auto moved = frame(20, testing::scenePose(5, 0, 0, 0), Eigen::Matrix4d::Identity(), 1);
    ASSERT_TRUE(ingest(moved));
    EXPECT_EQ(controller->status(), LocalizationStatus::Lost);
    EXPECT_FALSE(output(moved).pose_trusted);
    EXPECT_EQ(trajectory.size(), 1U);
    EXPECT_LT((output(moved).T_map_sensor - before.T_map_sensor).norm(), 1e-12);
    for (int cycle = 0; cycle < 3; ++cycle)
        maintain();
    EXPECT_EQ(reset.requests.size(), 1U);

    ASSERT_TRUE(controller->requestCoordinatedReset());
    const auto boundary = testing::scenePose(.3, .1, 0, .2);
    reply(FrontendResetOutcome::Completed, 2,
          FrontendResetBoundary{1, Timestamp::fromSeconds(20.1), boundary});
    const auto next =
        frame(21, testing::scenePose(5.5, .1, 0, .3), testing::scenePose(.2, 0, 0, .1), 2);
    ASSERT_TRUE(ingest(next));
    EXPECT_EQ(controller->status(), LocalizationStatus::Lost);
    EXPECT_FALSE(output(next).pose_trusted);
    EXPECT_EQ(trajectory.size(), 1U);
}

/// 自动冷启动后的 UWB 系统必须用新代次证据重新建立对齐，不能沿用旧位置立即报可信。
TEST_F(LocalizationFlow, AutomaticColdResetRequiresFreshUwbAlignment) {
    config.uwb.enabled = true;
    config.uwb.use_z = false;
    config.uwb.antenna_position_sensor = Eigen::Vector3d(.3, .1, 0);
    reset.available = true;
    start(false);
    FrontendFrame current;
    feedUwbTrajectory(testing::scenePose(3, -2, 0, .3), current);
    ASSERT_EQ(controller->status(), LocalizationStatus::Normal);
    const auto size = trajectory.size();
    controller->notifyFrontendDiscontinuity(0);
    reply(FrontendResetOutcome::Completed, 1);
    const auto alignment = testing::scenePose(8, -4, 0, -.4);
    const auto first = frame(21.5, alignment, Eigen::Matrix4d::Identity(), 1);
    ASSERT_TRUE(ingest(first));
    EXPECT_EQ(controller->status(), LocalizationStatus::Initializing);
    EXPECT_FALSE(output(first).pose_trusted);
    EXPECT_EQ(trajectory.size(), size);
    feedUwbTrajectory(alignment, current, 1, 22);
    ASSERT_EQ(controller->status(), LocalizationStatus::Normal);
    EXPECT_EQ(trajectory.latest().epoch, 1U);
    EXPECT_LT((output(current).T_map_sensor - alignment * current.odom_pose).norm(), 1e-4);
}

/// 普通前端失败锁存同代次输入和恢复，不确定重置仍保持失败直至成功显式换代。
TEST_F(LocalizationFlow, FrontendFailureRequiresAnExplicitCompletedReset) {
    config.loop.enabled = true;
    reset.available = true;
    start(false);
    const auto pose = testing::scenePose(2, 0, 0, .1);
    ASSERT_TRUE(ingest(frame(10, pose, pose)));
    controller->notifyFrontendFailure(0);
    const auto generation = controller->snapshot().localization.generation;
    for (int cycle = 0; cycle < 3; ++cycle) {
        controller->notifyFrontendFailure(0);
        controller->notifyFrontendDiscontinuity(0);
        maintain();
    }
    EXPECT_EQ(controller->status(), LocalizationStatus::Lost);
    EXPECT_EQ(controller->snapshot().localization.generation, generation);
    EXPECT_EQ(controller->snapshot().localization.estimator_failures.count, 0U);
    EXPECT_TRUE(reset.requests.empty());
    EXPECT_FALSE(ingest(frame(11, pose, pose)));
    ASSERT_TRUE(controller->requestCoordinatedReset(/*assume_stationary=*/true));
    reply(FrontendResetOutcome::Indeterminate, 1);
    EXPECT_EQ(controller->status(), LocalizationStatus::Lost);
    EXPECT_FALSE(ingest(frame(12, pose, Eigen::Matrix4d::Identity(), 1)));
    ASSERT_TRUE(controller->requestCoordinatedReset(/*assume_stationary=*/true));
    reply(FrontendResetOutcome::Completed, 2);
    ASSERT_TRUE(ingest(frame(13, pose, Eigen::Matrix4d::Identity(), 2)));
    EXPECT_EQ(controller->status(), LocalizationStatus::Normal);
    const auto resumed_generation = controller->snapshot().localization.generation;
    controller->notifyFrontendFailure(0);
    EXPECT_EQ(controller->status(), LocalizationStatus::Normal);
    EXPECT_EQ(controller->snapshot().localization.generation, resumed_generation);
    EXPECT_EQ(reset.requests.size(), 2U);
}

/// 未建立可信位置的 LOST 不得产生全局 TF，已验收位置的冻结输出仍允许发布。
TEST_F(LocalizationFlow, LostWithoutTrustedPoseCannotPublishGlobalTransforms) {
    reset.available = true;
    start(true);
    ASSERT_TRUE(controller->requestCoordinatedReset());
    reply(FrontendResetOutcome::Completed, 1);
    const auto first = frame(10, Eigen::Matrix4d::Identity(), Eigen::Matrix4d::Identity(), 1);
    ASSERT_TRUE(ingest(first));
    const auto unknown = controller->project(first, Timestamp::fromSeconds(10));
    EXPECT_EQ(unknown.status, LocalizationStatus::Lost);
    EXPECT_FALSE(unknown.global_pose_available);
    EXPECT_FALSE(unknown.should_publish_tf);
    EXPECT_FALSE(output(first).global_pose_available);
    maintain();
    ASSERT_EQ(controller->status(), LocalizationStatus::Normal);
    const auto known = controller->project(first, Timestamp::fromSeconds(10));
    EXPECT_TRUE(known.global_pose_available);
    EXPECT_TRUE(known.should_publish_tf);
    controller->notifyFrontendDiscontinuity(1);
    reply(FrontendResetOutcome::Completed, 2);
    const auto next = frame(11, Eigen::Matrix4d::Identity(), testing::scenePose(.2, 0, 0, 0), 2);
    ASSERT_TRUE(ingest(next));
    const auto frozen = controller->project(next, Timestamp::fromSeconds(11));
    EXPECT_EQ(frozen.status, LocalizationStatus::Lost);
    EXPECT_FALSE(frozen.pose_trusted);
    EXPECT_TRUE(frozen.global_pose_available);
    EXPECT_TRUE(frozen.should_publish_tf);
    EXPECT_LT((frozen.T_map_sensor - known.T_map_sensor).norm(), 1e-12);
}

/// 在输入失效后显式确认停车，连续边仍以最后可信非关键帧的时间和运动计算噪声。
TEST_F(LocalizationFlow, InputDiscontinuityRetainsTrustedReferenceNoiseForColdReset) {
    config.loop.enabled = true;
    config.keyframes.min_interval = 5;
    config.keyframes.max_interval = 10;
    config.graph.odom_noise.translation_variance_per_meter = .03;
    config.graph.odom_noise.rotation_variance_per_radian = .04;
    reset.available = true;
    std::optional<PoseCovariance6d> manual_covariance;
    for (const bool interrupted : {false, true}) {
        SCOPED_TRACE(interrupted);
        controller.reset();
        trajectory.clear();
        descriptor.clear();
        reset.requests.clear();
        start(false);
        const auto first_pose = testing::scenePose(2, 1, .3, .2);
        ASSERT_TRUE(ingest(frame(10, first_pose, first_pose)));
        const auto motion = testing::scenePose(.4, -.1, .1, .2);
        const Eigen::Matrix4d trusted_pose = first_pose * motion;
        const auto trusted = frame(10.2, trusted_pose, trusted_pose);
        ASSERT_TRUE(ingest(trusted));
        ASSERT_EQ(trajectory.size(), 1U);
        controller->project(trusted, Timestamp::fromSeconds(trusted.timestamp));
        if (interrupted) {
            controller->notifyFrontendDiscontinuity(0);
            reply(FrontendResetOutcome::Rejected, 0);
        }
        ASSERT_TRUE(controller->requestCoordinatedReset(/*assume_stationary=*/true));
        ASSERT_EQ(reset.requests.size(), interrupted ? 2U : 1U);
        if (interrupted) {
            const auto generation = controller->snapshot().localization.generation;
            controller->notifyFrontendDiscontinuity(0);
            EXPECT_EQ(controller->snapshot().localization.generation, generation);
            EXPECT_EQ(reset.requests.size(), 2U);
        }
        reply(FrontendResetOutcome::Completed, 1);
        const auto resumed = frame(11, trusted_pose, Eigen::Matrix4d::Identity(), 1);
        ASSERT_TRUE(ingest(resumed));
        const auto actual = output(resumed);
        ASSERT_TRUE(actual.pose_covariance_valid);
        EXPECT_LT((actual.T_map_sensor - trusted_pose).norm(), 1e-9);

        PoseGraph reference(config.graph);
        reference.addPrior(0, first_pose, 10);
        reference.update(false);
        PoseCovariance6d covariance = config.graph.odom_noise.covariance(
            .2, motion.block<3, 1>(0, 3).norm(), rotationAngleFromMatrix(motion));
        covariance.diagonal().head<3>().array() +=
            config.reset.stationary_translation_std * config.reset.stationary_translation_std;
        covariance.diagonal().tail<3>().array() +=
            config.reset.stationary_rotation_std_rad * config.reset.stationary_rotation_std_rad;
        reference.addResetContinuityMeasurement(0, 1, motion, trusted_pose, covariance, 11);
        reference.update(false);
        const auto expected = reference.marginalCovariance(1);
        ASSERT_TRUE(expected);
        EXPECT_LT(
            (actual.pose_covariance - localToWorldAdditivePoseCovariance(trusted_pose, *expected))
                .norm(),
            1e-9);
        if (manual_covariance)
            EXPECT_LT((actual.pose_covariance - *manual_covariance).norm(), 1e-12);
        else
            manual_covariance = actual.pose_covariance;
    }
}
}  // namespace
}  // namespace backend
