// 验证定位状态、重置事务、因果历史及候选任务执行。
#include <gtest/gtest.h>

#include <array>
#include <stdexcept>

#include "backend/frame_history.hpp"
#include "backend/localization_state.hpp"
#include "backend/reset_coordinator.hpp"
#include "geometry.hpp"
#include "parallel.hpp"

namespace backend {
namespace {
/// 推进冲突、恢复与故障事件，检查状态切换和累计故障记录各自的生命周期。
TEST(LocalizationLifecycle, ConflictRecoveryAndFaultHistoryHaveDistinctLifetimes) {
    LocalizationStateMachine state;
    state.onTrustedCorrection("anchor", TrustedCorrectionSource::Relocalization);
    EXPECT_EQ(state.status(), LocalizationStatus::Normal);
    const auto generation = state.generation();
    state.onCorrectionConflict("conflict", 2);
    EXPECT_EQ(state.status(), LocalizationStatus::ObsDegraded);
    EXPECT_TRUE(LocalizationStateMachine::poseTrusted(state.status()));
    state.onCorrectionConflict("conflict", 2);
    EXPECT_EQ(state.status(), LocalizationStatus::Suspect);
    EXPECT_TRUE(LocalizationStateMachine::tfFrozen(state.status()));
    state.onRecoveryRelocalizationFailure(1, "geometry");
    EXPECT_EQ(state.status(), LocalizationStatus::Lost);
    state.onEstimatorFailure("numerical failure");
    state.onFrontendResetCompleted();
    state.onTrustedCorrection("new anchor", TrustedCorrectionSource::Relocalization);
    EXPECT_EQ(state.status(), LocalizationStatus::Normal);
    EXPECT_GT(state.generation(), generation);
    EXPECT_EQ(state.snapshot().estimator_failures.count, 1U);
    EXPECT_EQ(state.snapshot().estimator_failures.last_reason, "numerical failure");
}

/// 普通跟踪观测不能解除 LOST，经过独立验收的恢复观测可以恢复正常状态。
TEST(LocalizationLifecycle, RegularObservationCannotUnfreezeLostState) {
    LocalizationStateMachine state;
    state.forceLost("recovery required");
    const auto generation = state.generation();
    state.onTrustedCorrection("late tracking result", TrustedCorrectionSource::RegularObservation);
    EXPECT_EQ(state.status(), LocalizationStatus::Lost);
    EXPECT_EQ(state.generation(), generation);
    state.onTrustedCorrection("verified recovery", TrustedCorrectionSource::Relocalization);
    EXPECT_EQ(state.status(), LocalizationStatus::Normal);
    EXPECT_GT(state.generation(), generation);
}

/// 超时重试沿用事务身份，终态回复只完成一次事务并启动冷却。
TEST(ResetCoordinator, TimeoutRetriesSameIdentityAndTerminalReplyClosesOnce) {
    ResetCoordinator coordinator;
    const auto transaction = coordinator.begin(Timestamp::fromSeconds(1), 10, 4);
    ASSERT_TRUE(transaction);
    EXPECT_EQ(transaction->target_epoch, 5U);
    coordinator.markSent(Timestamp::fromSeconds(1));
    EXPECT_FALSE(coordinator.retryIfExpired(Timestamp::fromSeconds(2), 2).transaction);
    const auto retry = coordinator.retryIfExpired(Timestamp::fromSeconds(3), 2);
    ASSERT_TRUE(retry.transaction);
    EXPECT_TRUE(retry.report_timeout);
    EXPECT_EQ(retry.transaction->transaction_id, transaction->transaction_id);
    EXPECT_EQ(retry.transaction->requester_instance_id, transaction->requester_instance_id);
    EXPECT_FALSE(coordinator.retryIfExpired(Timestamp::fromSeconds(5), 2).report_timeout);
    FrontendResetReply reply{*transaction, FrontendResetOutcome::Completed, 51, 1};
    EXPECT_EQ(coordinator.acceptReply(reply, Timestamp::fromSeconds(6)),
              ResetReplyAction::Completed);
    EXPECT_FALSE(coordinator.pending());
    EXPECT_EQ(coordinator.acceptReply(reply, Timestamp::fromSeconds(6)),
              ResetReplyAction::IgnoreMismatch);
    EXPECT_FALSE(coordinator.canBegin(Timestamp::fromSeconds(10), 10));
    EXPECT_TRUE(coordinator.canBegin(Timestamp::fromSeconds(11), 10));
}

/// 身份不匹配的重置回复应保留当前在途事务。
TEST(ResetCoordinator, MismatchedReplyCannotReleasePendingTransaction) {
    ResetCoordinator coordinator;
    const auto transaction = coordinator.begin(Timestamp::fromSeconds(1), 0, 0);
    ASSERT_TRUE(transaction);
    FrontendResetReply reply{*transaction, FrontendResetOutcome::Completed, 1, 1};
    ++reply.transaction.target_epoch;
    EXPECT_EQ(coordinator.acceptReply(reply, Timestamp::fromSeconds(2)),
              ResetReplyAction::IgnoreMismatch);
    EXPECT_TRUE(coordinator.pending());
    coordinator.cancelUnsent();
    EXPECT_TRUE(coordinator.canBegin(Timestamp::fromSeconds(2), 100));
}

/// 检查里程计插值、查询范围，以及历史失效后的时间和代次准入。
TEST(FrameHistory, InterpolationAndInvalidationKeepCausalTimeBoundary) {
    TimedPoseBuffer poses;
    poses.append(1, testing::translation(0), 10);
    poses.append(3, testing::translation(2), 10);
    const auto middle = poses.poseAt(2, 2);
    ASSERT_TRUE(middle);
    EXPECT_NEAR((*middle)(0, 3), 1, 1e-12);
    EXPECT_FALSE(poses.poseAt(4, .1));
    UwbConfig timing;
    SourceAccumulationConfig source;
    FrameHistory history(timing, source, source);
    auto frame = testing::frame(3);
    ASSERT_TRUE(history.accepts(frame, 0));
    history.append(frame);
    history.invalidate();
    EXPECT_FALSE(history.hasLatest());
    EXPECT_FALSE(history.accepts(frame, 0));
    frame.timestamp = 4;
    EXPECT_TRUE(history.accepts(frame, 0));
    EXPECT_FALSE(history.accepts(frame, 1));
}

/// 候选任务全部汇合后，统一报告输入序号最小的异常。
TEST(CandidateExecution, AllWorkCompletesAndLowestInputExceptionWins) {
    for (std::size_t workers : {0U, 1U, 4U, 16U}) {
        SCOPED_TRACE(workers);
        std::array<int, 5> visits{};
        try {
            detail::parallelFor(visits.size(), workers, [&](std::size_t index) {
                ++visits[index];
                if (index == 1)
                    throw std::invalid_argument("first");
                if (index == 3)
                    throw std::runtime_error("later");
            });
            FAIL() << "task exception was lost";
        } catch (const std::invalid_argument&) {
            EXPECT_EQ(visits, (std::array<int, 5>{1, 1, 1, 1, 1}));
        }
    }
}
}  // namespace
}  // namespace backend
