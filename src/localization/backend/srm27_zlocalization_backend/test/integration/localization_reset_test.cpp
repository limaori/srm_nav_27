// 验证协调重置的事务重试、拒绝回退与终态处理。
#include "localization_fixture.hpp"

namespace backend {
namespace {
using testing::LocalizationFlow;

/// 恢复计算期间发起并拒绝重置后，旧恢复结果仍因工作身份失效而被丢弃。
TEST_F(LocalizationFlow, ResetRejectedDuringRecoveryStillInvalidatesOldWork) {
    config.startup_hint.T_map_sensor = Eigen::Matrix4d::Identity();
    config.registration_debug_enabled = true;
    reset.available = true;
    start(true);
    ASSERT_TRUE(ingest(frame(10, Eigen::Matrix4d::Identity(), Eigen::Matrix4d::Identity())));
    const auto generation = controller->snapshot().localization.generation;
    bool sent = false;
    visualization.before_commit = [&] {
        sent = controller->requestCoordinatedReset();
        reply(FrontendResetOutcome::Rejected, 0);
    };
    maintain();
    ASSERT_TRUE(sent);
    EXPECT_TRUE(trajectory.empty());
    EXPECT_EQ(controller->status(), LocalizationStatus::Initializing);
    EXPECT_GT(controller->snapshot().localization.generation, generation);
    maintain();
    EXPECT_EQ(controller->status(), LocalizationStatus::Normal);
    EXPECT_EQ(trajectory.size(), 1U);
}

/// 覆盖重置发送失败、超时重试与成功换代，检查可信地图位姿的接续。
TEST_F(LocalizationFlow, ResetFailureRetryAndNewEpochPreservePhysicalContinuity) {
    start(false);
    auto first = frame(10, testing::scenePose(2, 0, 0, 0), testing::scenePose(2, 0, 0, 0));
    ASSERT_TRUE(ingest(first));
    const auto previous = output(first);
    ASSERT_TRUE(previous.pose_trusted);
    EXPECT_FALSE(controller->requestCoordinatedReset(/*assume_stationary=*/true));
    EXPECT_TRUE(controller->projectionIsCurrent(previous));
    reset.available = true;
    reset.accepts = false;
    EXPECT_FALSE(controller->requestCoordinatedReset(/*assume_stationary=*/true));
    EXPECT_EQ(controller->status(), LocalizationStatus::Normal);
    reset.accepts = true;
    ASSERT_TRUE(controller->requestCoordinatedReset(/*assume_stationary=*/true));
    ASSERT_TRUE(controller->requestCoordinatedReset(/*assume_stationary=*/true));
    EXPECT_EQ(reset.requests.size(), 1U);
    EXPECT_FALSE(controller->projectionIsCurrent(previous));
    clock.advance(6);
    controller->runMaintenanceCycle();
    ASSERT_EQ(reset.requests.size(), 2U);
    EXPECT_EQ(reset.requests[0].transaction_id, reset.requests[1].transaction_id);
    reply(FrontendResetOutcome::Rejected, 0);
    EXPECT_EQ(controller->status(), LocalizationStatus::Normal);
    EXPECT_EQ(controller->activeEpoch(), 0U);
    ASSERT_TRUE(controller->requestCoordinatedReset(/*assume_stationary=*/true));
    reply(FrontendResetOutcome::Completed);
    EXPECT_EQ(controller->activeEpoch(), 1U);
    EXPECT_EQ(reset.boundaries, 1);
    EXPECT_FALSE(ingest(first));
    auto restarted = frame(11, first.odom_pose, Eigen::Matrix4d::Identity(), 1);
    ASSERT_TRUE(ingest(restarted));
    EXPECT_EQ(controller->status(), LocalizationStatus::Normal);
    EXPECT_LT((output(restarted).T_map_sensor - previous.T_map_sensor).norm(), 1e-8);
    EXPECT_FALSE(controller->projectionIsCurrent(previous));
    reply(FrontendResetOutcome::Completed);
    EXPECT_EQ(reset.boundaries, 1);
}

/// 尚无任何全局轨迹时的身份协调保留新地图启动能力。
TEST_F(LocalizationFlow, ColdIdentityResetBeforeFirstFrameCanStartANewMap) {
    reset.available = true;
    start(false);
    ASSERT_TRUE(controller->requestCoordinatedReset());
    reply(FrontendResetOutcome::Completed, 1);
    const auto first = frame(10, Eigen::Matrix4d::Identity(), Eigen::Matrix4d::Identity(), 1);
    ASSERT_TRUE(ingest(first));
    EXPECT_EQ(controller->status(), LocalizationStatus::Normal);
    EXPECT_EQ(trajectory.size(), 1U);
}

/// 不确定终态不确认停车衔接，即使请求曾获得显式停车保证。
TEST_F(LocalizationFlow, IndeterminateResetCannotConfirmStationaryContinuity) {
    config.loop.enabled = true;
    reset.available = true;
    start(false);
    ASSERT_TRUE(ingest(frame(10, Eigen::Matrix4d::Identity(), Eigen::Matrix4d::Identity())));
    ASSERT_TRUE(controller->requestCoordinatedReset(/*assume_stationary=*/true));
    reply(FrontendResetOutcome::Indeterminate, 1);
    const auto next = frame(11, Eigen::Matrix4d::Identity(), Eigen::Matrix4d::Identity(), 1);
    ASSERT_TRUE(ingest(next));
    EXPECT_EQ(controller->status(), LocalizationStatus::Lost);
    EXPECT_FALSE(output(next).pose_trusted);
    EXPECT_EQ(trajectory.size(), 1U);
}

/// 迟到的输入中断或普通失败会撤销在途人工重置的停车资格。
TEST_F(LocalizationFlow, FrontendFaultRevokesAnInFlightStationaryAssumption) {
    config.loop.enabled = true;
    reset.available = true;
    for (const bool terminal_failure : {false, true}) {
        SCOPED_TRACE(terminal_failure);
        controller.reset();
        trajectory.clear();
        descriptor.clear();
        reset.requests.clear();
        start(false);
        ASSERT_TRUE(ingest(frame(10, Eigen::Matrix4d::Identity(), Eigen::Matrix4d::Identity())));
        ASSERT_TRUE(controller->requestCoordinatedReset(/*assume_stationary=*/true));
        if (terminal_failure)
            controller->notifyFrontendFailure(0);
        else
            controller->notifyFrontendDiscontinuity(0);
        ASSERT_EQ(reset.requests.size(), 1U);
        reply(FrontendResetOutcome::Completed, 1);
        ASSERT_TRUE(
            ingest(frame(20, testing::scenePose(3, 0, 0, 0), Eigen::Matrix4d::Identity(), 1)));
        EXPECT_EQ(controller->status(), LocalizationStatus::Lost);
        EXPECT_EQ(trajectory.size(), 1U);
    }
}
}  // namespace
}  // namespace backend
