// 验证前端进程身份、重置回复及输入故障的会话准入。
#include <gtest/gtest.h>

#include "apps/frontend_session.hpp"
#include "geometry.hpp"

namespace backend {
namespace {
/// 源代次与目标代次分别翻译，坐标边界和零时间戳保持原值。
TEST(FrontendSession, FrontendAndBackendEpochsRemainSeparateAcrossResetTranslation) {
    ros::FrontendSession session;
    ASSERT_EQ(session.observe(10, 7, 1), ros::FrontendSession::Admission::Accept);
    const FrontendResetTransaction request{42, 1, 1};
    const auto attempt = session.prepareReset(request);
    ASSERT_TRUE(attempt);
    EXPECT_EQ(attempt->target_epoch, 8U);
    session.markSent(*attempt);
    FrontendResetReply reply{{42, 1, 8}, FrontendResetOutcome::Completed, 10, 1};
    reply.boundary = FrontendResetBoundary{7, Timestamp(), testing::translation(2)};
    const FrontendResetReply wire_reply = reply;
    ASSERT_TRUE(session.translateReply(reply, *attempt));
    EXPECT_EQ(reply.transaction.target_epoch, 1U);
    ASSERT_TRUE(reply.boundary);
    EXPECT_EQ(reply.boundary->source_epoch, 0U);
    EXPECT_EQ(reply.boundary->stamp.nanoseconds(), 0);
    EXPECT_EQ(reply.boundary->old_from_new, testing::translation(2));
    session.commitBoundary(1);
    EXPECT_EQ(session.observe(10, 8, 1), ros::FrontendSession::Admission::Accept);
    EXPECT_EQ(session.observe(10, 7, 99), ros::FrontendSession::Admission::Drop);
    FrontendResetReply duplicate = wire_reply;
    EXPECT_FALSE(session.translateReply(duplicate, *attempt));

    const auto next = session.prepareReset({42, 2, 2});
    ASSERT_TRUE(next);
    EXPECT_EQ(next->target_epoch, 9U);
    session.markSent(*next);
    FrontendResetReply second{{42, 2, 9}, FrontendResetOutcome::Completed, 10, 2};
    second.boundary = FrontendResetBoundary{8, Timestamp::fromSeconds(10), testing::translation(3)};
    ASSERT_TRUE(session.translateReply(second, *next));
    ASSERT_TRUE(second.boundary);
    EXPECT_EQ(second.boundary->source_epoch, 1U);
    EXPECT_EQ(second.boundary->stamp.nanoseconds(), 10000000000LL);
    EXPECT_EQ(second.boundary->old_from_new, testing::translation(3));
    session.commitBoundary(2);
    EXPECT_EQ(session.observe(10, 9, 1), ros::FrontendSession::Admission::Accept);
}

/// 同一业务事务重试后，仅最新传输尝试可以确认前端身份。
TEST(FrontendSession, OnlyLatestRpcAttemptCanConfirmTheBusinessTransaction) {
    ros::FrontendSession session;
    session.observe(10, 0, 1);
    const FrontendResetTransaction request{42, 1, 1};
    const auto first = session.prepareReset(request);
    ASSERT_TRUE(first);
    session.markSent(*first);
    const auto retry = session.prepareReset(request);
    ASSERT_TRUE(retry);
    session.markSent(*retry);
    FrontendResetReply reply{{42, 1, 1}, FrontendResetOutcome::Completed, 10, 1};
    EXPECT_FALSE(session.translateReply(reply, *first));
    EXPECT_EQ(session.observe(10, 1, 1), ros::FrontendSession::Admission::Drop);
    EXPECT_TRUE(session.translateReply(reply, *retry));
    session.commitBoundary(1);
    EXPECT_EQ(session.observe(10, 1, 1), ros::FrontendSession::Admission::Accept);
}

/// 新候选身份使旧 RPC 回复失效，已退休进程无法重新接入活动会话。
TEST(FrontendSession, NewCandidateInvalidatesOldReplyAndRetiredProcessesCannotReturn) {
    ros::FrontendSession session;
    session.observe(10, 0, 1);
    EXPECT_EQ(session.observe(20, 0), ros::FrontendSession::Admission::ResetRequired);
    const FrontendResetTransaction request{42, 1, 1};
    const auto first = session.prepareReset(request);
    ASSERT_TRUE(first);
    session.markSent(*first);
    EXPECT_EQ(session.observe(30, 0), ros::FrontendSession::Admission::Drop);
    FrontendResetReply old_reply{{42, 1, 1}, FrontendResetOutcome::Completed, 20, 1};
    EXPECT_FALSE(session.translateReply(old_reply, *first));
    const auto retry = session.prepareReset(request);
    ASSERT_TRUE(retry);
    session.markSent(*retry);
    FrontendResetReply current_reply{{42, 1, 1}, FrontendResetOutcome::Completed, 30, 1};
    current_reply.boundary =
        FrontendResetBoundary{0, Timestamp::fromSeconds(10), testing::translation(2)};
    ASSERT_TRUE(session.translateReply(current_reply, *retry));
    EXPECT_FALSE(current_reply.boundary);
    session.commitBoundary(1);
    EXPECT_EQ(session.observe(10, 99, 99), ros::FrontendSession::Admission::Drop);
    EXPECT_EQ(session.observe(20, 99, 99), ros::FrontendSession::Admission::Drop);
    EXPECT_EQ(session.observe(30, 1, 1), ros::FrontendSession::Admission::Accept);
    EXPECT_EQ(session.pendingAction(), ros::FrontendSession::Admission::Drop);
}

/// 同一进程的过期源代次不能借成功事务连接到当前活动坐标。
TEST(FrontendSession, StaleSourceEpochDiscardsContinuityWithoutRejectingCompletedReset) {
    ros::FrontendSession session;
    ASSERT_EQ(session.observe(10, 7, 1), ros::FrontendSession::Admission::Accept);
    const auto attempt = session.prepareReset({42, 1, 1});
    ASSERT_TRUE(attempt);
    session.markSent(*attempt);
    FrontendResetReply reply{{42, 1, 8}, FrontendResetOutcome::Completed, 10, 1};
    reply.boundary = FrontendResetBoundary{6, Timestamp::fromSeconds(10), testing::translation(2)};
    ASSERT_TRUE(session.translateReply(reply, *attempt));
    EXPECT_FALSE(reply.boundary);
    EXPECT_EQ(reply.outcome, FrontendResetOutcome::Completed);
    session.commitBoundary(1);
    EXPECT_EQ(session.observe(10, 8, 1), ros::FrontendSession::Admission::Accept);
}

/// 冷启动和非成功回复不能生成运动连续性，即使非成功回复错误携带了几何字段。
TEST(FrontendSession, ColdAndNoncompletedRepliesDoNotCarryContinuity) {
    for (const auto outcome : {FrontendResetOutcome::Completed, FrontendResetOutcome::Indeterminate,
                               FrontendResetOutcome::Rejected, FrontendResetOutcome::InProgress}) {
        SCOPED_TRACE(static_cast<int>(outcome));
        ros::FrontendSession session;
        ASSERT_EQ(session.observe(10, 0, 1), ros::FrontendSession::Admission::Accept);
        const auto attempt = session.prepareReset({42, 1, 1});
        ASSERT_TRUE(attempt);
        session.markSent(*attempt);
        FrontendResetReply reply{{42, 1, 1}, outcome, 10, 1};
        if (outcome != FrontendResetOutcome::Completed)
            reply.boundary =
                FrontendResetBoundary{0, Timestamp::fromSeconds(10), testing::translation(2)};
        ASSERT_TRUE(session.translateReply(reply, *attempt));
        EXPECT_FALSE(reply.boundary);
        EXPECT_EQ(reply.outcome, outcome);
        EXPECT_EQ(reply.transaction.target_epoch, 1U);
    }
}

/// 未发送撤销及在途回复保持原会话，只有有效终态可以提交代次边界。
TEST(FrontendSession, UnsentCancellationAndNonterminalRepliesCannotCommitBoundary) {
    ros::FrontendSession session;
    session.observe(10, 0, 1);
    const FrontendResetTransaction request{42, 1, 1};
    ASSERT_TRUE(session.prepareReset(request));
    session.cancelUnsent(request);
    EXPECT_EQ(session.observe(10, 0, 2), ros::FrontendSession::Admission::Accept);
    const auto attempt = session.prepareReset(request);
    ASSERT_TRUE(attempt);
    session.markSent(*attempt);
    session.cancelUnsent(request);
    FrontendResetReply pending{{42, 1, 1}, FrontendResetOutcome::InProgress, 10, 0};
    ASSERT_TRUE(session.translateReply(pending, *attempt));
    session.commitBoundary(1);
    EXPECT_EQ(session.observe(10, 1, 1), ros::FrontendSession::Admission::Drop);
    FrontendResetReply complete{{42, 1, 1}, FrontendResetOutcome::Completed, 10, 1};
    ASSERT_TRUE(session.translateReply(complete, *attempt));
    session.commitBoundary(1);
    EXPECT_EQ(session.observe(10, 1, 1), ros::FrontendSession::Admission::Accept);
}

/// 重复使用旧重置代数的回复不能完成另一个业务事务。
TEST(FrontendSession, ReplayedGenerationCannotFinishAnotherTransaction) {
    ros::FrontendSession session;
    session.observe(10, 0, 1);
    const auto first = session.prepareReset({42, 1, 1});
    ASSERT_TRUE(first);
    session.markSent(*first);
    FrontendResetReply reply{{42, 1, 1}, FrontendResetOutcome::Completed, 10, 1};
    ASSERT_TRUE(session.translateReply(reply, *first));
    session.commitBoundary(1);
    const auto second = session.prepareReset({42, 2, 2});
    ASSERT_TRUE(second);
    session.markSent(*second);
    reply.transaction = {42, 2, 2};
    reply.boundary = FrontendResetBoundary{1, Timestamp::fromSeconds(10), testing::translation(2)};
    EXPECT_FALSE(session.translateReply(reply, *second));
    reply.reset_generation = 2;
    ASSERT_TRUE(session.translateReply(reply, *second));
    ASSERT_TRUE(reply.boundary);
    EXPECT_EQ(reply.boundary->source_epoch, 1U);
    session.commitBoundary(2);
    EXPECT_EQ(session.observe(10, 2, 1), ros::FrontendSession::Admission::Accept);
}

/// 活动会话拒绝重复帧序号，并要求变更进程先完成身份确认。
TEST(FrontendSession, DuplicateSequenceAndChangedProcessCannotBeAdmitted) {
    ros::FrontendSession session;
    EXPECT_EQ(session.observe(10, 0, 1), ros::FrontendSession::Admission::Accept);
    EXPECT_EQ(session.observe(10, 0, 1), ros::FrontendSession::Admission::Drop);
    EXPECT_EQ(session.observe(10, 0, 2), ros::FrontendSession::Admission::Accept);
    EXPECT_EQ(session.observe(11, 0, 1), ros::FrontendSession::Admission::ResetRequired);
    EXPECT_EQ(session.pendingAction(), ros::FrontendSession::Admission::ResetRequired);
}

/// 未出首帧也可锁存明确输入故障，迟到的正常通知不能解除它。
TEST(FrontendSession, InputDiscontinuityIsLatchedUntilCommittedBoundary) {
    using Admission = ros::FrontendSession::Admission;
    ros::FrontendSession session;
    ASSERT_EQ(session.observe(10, 7, std::nullopt, true), Admission::InputDiscontinuity);
    EXPECT_EQ(session.observe(10, 7), Admission::InputDiscontinuity);
    EXPECT_EQ(session.observe(10, 7, 1), Admission::InputDiscontinuity);
    EXPECT_EQ(session.pendingAction(), Admission::InputDiscontinuity);
    const auto attempt = session.prepareReset({42, 1, 1});
    ASSERT_TRUE(attempt);
    session.markSent(*attempt);
    EXPECT_EQ(session.pendingAction(), Admission::InputDiscontinuity);
    FrontendResetReply reply{{42, 1, 8}, FrontendResetOutcome::Completed, 10, 1};
    ASSERT_TRUE(session.translateReply(reply, *attempt));
    session.commitBoundary(1);
    EXPECT_EQ(session.pendingAction(), Admission::Drop);
    EXPECT_EQ(session.observe(10, 7, std::nullopt, true), Admission::Drop);
    EXPECT_EQ(session.observe(10, 8, 1), Admission::Accept);
}

/// 在途手动重置期间收到故障，拒绝回复不得丢失后续自动恢复请求。
TEST(FrontendSession, RejectedResetDoesNotClearInputDiscontinuity) {
    using Admission = ros::FrontendSession::Admission;
    ros::FrontendSession session;
    ASSERT_EQ(session.observe(10, 0), Admission::Accept);
    const auto attempt = session.prepareReset({42, 1, 1});
    ASSERT_TRUE(attempt);
    session.markSent(*attempt);
    EXPECT_EQ(session.observe(10, 0, std::nullopt, true), Admission::InputDiscontinuity);
    FrontendResetReply reply{{42, 1, 1}, FrontendResetOutcome::Rejected, 10, 0};
    ASSERT_TRUE(session.translateReply(reply, *attempt));
    EXPECT_EQ(session.pendingAction(), Admission::InputDiscontinuity);
    EXPECT_EQ(session.observe(10, 0), Admission::InputDiscontinuity);
}

/// 陌生实例的通知参与身份仲裁，不能冒充活动会话的输入故障。
TEST(FrontendSession, ForeignFailureDoesNotReplaceActiveHealth) {
    using Admission = ros::FrontendSession::Admission;
    ros::FrontendSession session;
    ASSERT_EQ(session.observe(10, 4, 1), Admission::Accept);
    EXPECT_EQ(session.observe(20, 4, std::nullopt, true), Admission::ResetRequired);
    EXPECT_EQ(session.pendingAction(), Admission::ResetRequired);
    const auto attempt = session.prepareReset({42, 1, 1});
    ASSERT_TRUE(attempt);
    session.markSent(*attempt);
    FrontendResetReply reply{{42, 1, 5}, FrontendResetOutcome::Completed, 10, 1};
    ASSERT_TRUE(session.translateReply(reply, *attempt));
    session.commitBoundary(1);
    EXPECT_EQ(session.observe(20, 6, std::nullopt, true), Admission::Drop);
    EXPECT_EQ(session.pendingAction(), Admission::Drop);
    EXPECT_EQ(session.observe(10, 5, 1), Admission::Accept);
}

/// 普通失败优先于可恢复中断，正常消息和拒绝回复都不能解除故障。
TEST(FrontendSession, ProcessingFailureRequiresAnExplicitBoundary) {
    using Admission = ros::FrontendSession::Admission;
    ros::FrontendSession session;
    ASSERT_EQ(session.observe(10, 4, 1), Admission::Accept);
    EXPECT_EQ(session.observe(10, 4, std::nullopt, true, true), Admission::InputFailure);
    EXPECT_EQ(session.observe(10, 4, 2), Admission::InputFailure);
    EXPECT_EQ(session.pendingAction(), Admission::InputFailure);
    const auto rejected = session.prepareReset({42, 1, 1});
    ASSERT_TRUE(rejected);
    session.markSent(*rejected);
    FrontendResetReply reply{{42, 1, 5}, FrontendResetOutcome::Rejected, 10, 0};
    ASSERT_TRUE(session.translateReply(reply, *rejected));
    EXPECT_EQ(session.pendingAction(), Admission::InputFailure);

    const auto completed = session.prepareReset({42, 2, 1});
    ASSERT_TRUE(completed);
    session.markSent(*completed);
    reply = {{42, 2, 5}, FrontendResetOutcome::Completed, 10, 1};
    ASSERT_TRUE(session.translateReply(reply, *completed));
    session.commitBoundary(1);
    EXPECT_EQ(session.observe(10, 5, 1), Admission::Accept);
    EXPECT_EQ(session.observe(10, 4, std::nullopt, false, true), Admission::Drop);
    EXPECT_EQ(session.pendingAction(), Admission::Drop);
}

/// 目标代次的故障无论先于还是晚于终态回复到达，都必须保留到边界提交之后。
TEST(FrontendSession, TargetEpochFaultSurvivesReplyAndBoundaryCommitOrdering) {
    using Admission = ros::FrontendSession::Admission;
    for (const auto outcome :
         {FrontendResetOutcome::Completed, FrontendResetOutcome::Indeterminate}) {
        for (const bool failed : {false, true}) {
            for (const bool before_reply : {false, true}) {
                SCOPED_TRACE(::testing::Message()
                             << "outcome=" << static_cast<int>(outcome) << " failed=" << failed
                             << " before_reply=" << before_reply);
                ros::FrontendSession session;
                ASSERT_EQ(session.observe(10, 7, 1), Admission::Accept);
                const auto attempt = session.prepareReset({42, 1, 1});
                ASSERT_TRUE(attempt);
                session.markSent(*attempt);
                EXPECT_EQ(session.observe(10, 8, std::nullopt, before_reply && !failed,
                                          before_reply && failed),
                          Admission::Drop);
                FrontendResetReply reply{{42, 1, 8}, outcome, 10, 1};
                ASSERT_TRUE(session.translateReply(reply, *attempt));
                if (!before_reply) {
                    EXPECT_EQ(session.observe(10, 8, std::nullopt, !failed, failed),
                              Admission::Drop);
                }
                EXPECT_EQ(session.observe(10, 8), Admission::Drop);
                session.commitBoundary(1);
                const auto expected =
                    failed ? Admission::InputFailure : Admission::InputDiscontinuity;
                EXPECT_EQ(session.pendingAction(), expected);
                EXPECT_EQ(session.observe(10, 8, 1), expected);
                EXPECT_EQ(session.observe(10, 7, std::nullopt, true, true), Admission::Drop);
                EXPECT_EQ(session.pendingAction(), expected);
            }
        }
    }
}

/// 未被选中的新实例故障不能污染已确认的健康目标会话。
TEST(FrontendSession, ForeignTargetFaultDoesNotContaminateConfirmedHealthyInstance) {
    using Admission = ros::FrontendSession::Admission;
    ros::FrontendSession session;
    ASSERT_EQ(session.observe(10, 0, 1), Admission::Accept);
    EXPECT_EQ(session.observe(20, 0), Admission::ResetRequired);
    const FrontendResetTransaction request{42, 1, 1};
    const auto attempt = session.prepareReset(request);
    ASSERT_TRUE(attempt);
    session.markSent(*attempt);
    EXPECT_EQ(session.observe(20, 1, std::nullopt, false, true), Admission::Drop);
    FrontendResetReply reply{{42, 1, 1}, FrontendResetOutcome::Completed, 10, 1};
    ASSERT_TRUE(session.translateReply(reply, *attempt));
    session.commitBoundary(1);
    EXPECT_EQ(session.pendingAction(), Admission::Drop);
    EXPECT_EQ(session.observe(10, 1, 1), Admission::Accept);
    EXPECT_EQ(session.observe(20, 1, std::nullopt, false, true), Admission::Drop);
}

/// 前端进程更换仍可协调身份，旧进程故障不能永久阻塞新的实例。
TEST(FrontendSession, ReplacementProcessCanRecoverFromProcessingFailure) {
    using Admission = ros::FrontendSession::Admission;
    ros::FrontendSession session;
    ASSERT_EQ(session.observe(10, 0, std::nullopt, false, true), Admission::InputFailure);
    EXPECT_EQ(session.observe(20, 0), Admission::ResetRequired);
    EXPECT_EQ(session.pendingAction(), Admission::ResetRequired);
    const auto attempt = session.prepareReset({42, 1, 1});
    ASSERT_TRUE(attempt);
    session.markSent(*attempt);
    FrontendResetReply reply{{42, 1, 1}, FrontendResetOutcome::Completed, 20, 1};
    ASSERT_TRUE(session.translateReply(reply, *attempt));
    session.commitBoundary(1);
    EXPECT_EQ(session.observe(20, 1, 1), Admission::Accept);
    EXPECT_EQ(session.observe(10, 0, std::nullopt, false, true), Admission::Drop);
}
}  // namespace
}  // namespace backend
