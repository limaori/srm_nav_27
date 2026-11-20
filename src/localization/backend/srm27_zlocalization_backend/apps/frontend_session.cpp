/**
 * @file frontend_session.cpp
 * @brief 仲裁前端身份和重置响应资格，保持业务与传输身份的独立生命周期。
 */
#include "apps/frontend_session.hpp"

#include <algorithm>
#include <limits>

namespace backend::ros {
/// 返回身份确认、可恢复中断或禁止自动重试的会话失败。
FrontendSession::Admission FrontendSession::pendingAction() {
    std::lock_guard<std::mutex> lock(mutex_);
    return pendingActionLocked();
}

/// 在持锁状态下按身份确认、普通失败及输入中断的优先级选择动作。
FrontendSession::Admission FrontendSession::pendingActionLocked() const {
    if (!pending_ && needsConfirmation())
        return Admission::ResetRequired;
    if (input_failed_)
        return Admission::InputFailure;
    if (input_interrupted_)
        return Admission::InputDiscontinuity;
    return Admission::Drop;
}

/// 按进程和代次锁存已观察故障，迟到的正常通知不能解除同代次失败。
FrontendSession::Admission FrontendSession::observe(std::uint64_t instance, std::uint64_t epoch,
                                                    std::optional<std::uint64_t> sequence,
                                                    bool reset_required, bool failed) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!instance || retired_.count(instance) || (sequence && *sequence == 0))
        return Admission::Drop;
    const Identity incoming{instance, epoch};
    auto [observed, inserted] =
        observed_.try_emplace(instance, ObservedSession{epoch, reset_required, failed});
    if (inserted) {
        // 身份不携带跨进程顺序；只有新候选使此前发出的 RPC 失去仲裁资格。
        ++identity_revision_;
    } else {
        if (epoch < observed->second.epoch)
            return Admission::Drop;
        if (epoch > observed->second.epoch)
            observed->second = ObservedSession{epoch, reset_required, failed};
        else {
            observed->second.interrupted = observed->second.interrupted || reset_required;
            observed->second.failed = observed->second.failed || failed;
        }
    }
    if (!active_ && !pending_ && observed_.size() == 1)
        active_ = incoming;
    if (active_ && active_->instance == instance && active_->epoch == epoch) {
        input_interrupted_ = input_interrupted_ || observed->second.interrupted;
        input_failed_ = input_failed_ || observed->second.failed;
    }
    const auto action = pendingActionLocked();
    if (action != Admission::Drop)
        return action;
    if (pending_)
        return Admission::Drop;
    if (sequence) {
        if (*sequence <= sequence_)
            return Admission::Drop;
        sequence_ = *sequence;
    }
    return Admission::Accept;
}

/// 选定前端目标代次，并为本次传输捕获独立的尝试编号与身份版本。
std::optional<FrontendSession::ResetAttempt> FrontendSession::prepareReset(
    const FrontendResetTransaction& request) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!pending_) {
        auto target = request.target_epoch;
        // 只取可递增 epoch 的上界作为请求目标，不据此推断哪个进程更新。
        for (const auto& peer : observed_)
            if (peer.second.epoch != std::numeric_limits<std::uint64_t>::max())
                target = std::max(target, peer.second.epoch + 1);
        pending_ = Pending{request, target, std::nullopt, std::nullopt};
    }
    if (!same(pending_->request, request))
        return std::nullopt;
    return ResetAttempt{request, pending_->target, identity_revision_, ++next_attempt_};
}

/// 登记当前业务事务实际发送的传输尝试。
void FrontendSession::markSent(const ResetAttempt& attempt) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (pending_ && same(pending_->request, attempt.request) &&
        pending_->target == attempt.target_epoch)
        pending_->sent_attempt = attempt.rpc_attempt;
}

/// 验收回复资格，翻译前后端代次并保存待安装的会话身份。
bool FrontendSession::translateReply(FrontendResetReply& reply, const ResetAttempt& attempt) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!pending_ || !same(pending_->request, attempt.request) ||
        attempt.target_epoch != pending_->target || pending_->sent_attempt != attempt.rpc_attempt ||
        attempt.identity_revision != identity_revision_ || !reply.frontend_instance_id ||
        retired_.count(reply.frontend_instance_id) ||
        reply.transaction.requester_instance_id != pending_->request.requester_instance_id ||
        reply.transaction.transaction_id != pending_->request.transaction_id ||
        reply.transaction.target_epoch != pending_->target)
        return false;
    const auto found = generations_.find(reply.frontend_instance_id);
    const auto observed = observed_.find(reply.frontend_instance_id);
    const bool terminal_boundary = reply.outcome == FrontendResetOutcome::Completed ||
                                   reply.outcome == FrontendResetOutcome::Indeterminate;
    // REJECTED 不携带实际 epoch；未见过的响应者需要等其会话/观测给出该信息。
    if (reply.outcome == FrontendResetOutcome::Rejected && observed == observed_.end())
        return false;
    if (found != generations_.end() &&
        (reply.reset_generation < found->second ||
         (terminal_boundary && reply.reset_generation == found->second)))
        return false;
    // 只有当前活动前端的旧坐标能连接后端旧代次，其他终态只承担会话隔离。
    if (reply.boundary) {
        if (reply.outcome == FrontendResetOutcome::Completed && active_ &&
            active_->instance == reply.frontend_instance_id &&
            active_->epoch == reply.boundary->source_epoch && pending_->request.target_epoch > 0)
            reply.boundary->source_epoch = pending_->request.target_epoch - 1;
        else
            reply.boundary.reset();
    }
    generations_[reply.frontend_instance_id] = reply.reset_generation;
    reply.transaction = pending_->request;
    if (reply.outcome != FrontendResetOutcome::InProgress) {
        const auto epoch = terminal_boundary
                               ? std::max(pending_->target,
                                          observed == observed_.end() ? 0 : observed->second.epoch)
                               : observed->second.epoch;
        const ObservedSession confirmed =
            observed != observed_.end() && observed->second.epoch == epoch ? observed->second
                                                                           : ObservedSession{epoch};
        for (const auto& peer : observed_)
            if (peer.first != reply.frontend_instance_id)
                retired_.insert(peer.first);
        observed_.clear();
        observed_.emplace(reply.frontend_instance_id, confirmed);
    }
    if (terminal_boundary) {
        pending_->applied = Identity{reply.frontend_instance_id, pending_->target};
    } else if (reply.outcome == FrontendResetOutcome::Rejected)
        pending_.reset();
    return true;
}

/// 安装已确认身份及其已观察故障，清除旧代次状态和帧序号水位。
void FrontendSession::commitBoundary(std::uint64_t backend_epoch) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!pending_ || !pending_->applied || pending_->request.target_epoch != backend_epoch)
        return;
    active_ = pending_->applied;
    const auto observed = observed_.find(active_->instance);
    const bool same_epoch = observed != observed_.end() && observed->second.epoch == active_->epoch;
    input_interrupted_ = same_epoch && observed->second.interrupted;
    input_failed_ = same_epoch && observed->second.failed;
    sequence_ = 0;
    pending_.reset();
}

/// 释放尚未发送到前端的业务事务。
void FrontendSession::cancelUnsent(const FrontendResetTransaction& request) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (pending_ && !pending_->sent_attempt && same(pending_->request, request))
        pending_.reset();
}

/// 检查观察到的身份集合是否与唯一活动会话一致。
bool FrontendSession::needsConfirmation() const {
    return !observed_.empty() &&
           (!active_ || observed_.size() != 1 || observed_.begin()->first != active_->instance ||
            observed_.begin()->second.epoch != active_->epoch);
}

/// 比较两条请求是否属于同一业务事务及目标代次。
bool FrontendSession::same(const FrontendResetTransaction& a, const FrontendResetTransaction& b) {
    return a.requester_instance_id == b.requester_instance_id &&
           a.transaction_id == b.transaction_id && a.target_epoch == b.target_epoch;
}
}  // namespace backend::ros
