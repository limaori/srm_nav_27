/**
 * @file reset_coordinator.hpp
 * @brief 维护前端重置的幂等事务身份、超时及响应水位。
 */
#pragma once

#include <cstdint>
#include <optional>

#include "backend/ports.hpp"
#include "backend/time.hpp"

namespace backend {

/// reset 响应对当前事务产生的确定动作。
enum class ResetReplyAction {
    IgnoreMismatch,
    IgnoreMissingInstance,
    IgnoreRegressedGeneration,
    IgnoreReplayedTerminal,
    InProgress,
    Rejected,
    Completed,
    Indeterminate,
};

/// 超时后需要以同一幂等键重发的请求。
struct ResetRetry {
    std::optional<FrontendResetTransaction> transaction;
    bool report_timeout = false;
};

/// 维护单个前端 reset 事务，不直接执行 ROS 调用或修改定位状态。
class ResetCoordinator {
public:
    /// 生成本进程的非零重置请求者身份。
    ResetCoordinator();

    /// 清除运行中的事务和前端响应高水位，保留本进程 requester 身份及事务序列。
    void clearRuntimeState();

    /// 返回是否存在尚未终结的前端重置事务。
    bool pending() const { return active_.has_value(); }
    /// 最近已交给传输层的事务；终结响应和重试不改变该工作边界。
    std::uint64_t lastSentTransactionId() const { return last_sent_transaction_id_; }

    /// 判断当前时刻是否允许预检一个新事务。
    bool canBegin(const Timestamp& now, double minimum_gap_seconds) const;

    /// 占用下一个事务身份；调用方随后必须标记发送成功或撤销占用。
    std::optional<FrontendResetTransaction> begin(const Timestamp& now, double minimum_gap_seconds,
                                                  std::uint64_t active_epoch);

    /// 记录请求已交给传输层并开始计算最小事务间隔。
    void markSent(const Timestamp& now);

    /// 撤销尚未进入传输层的事务，不消耗最小事务间隔。
    void cancelUnsent();

    /// 响应超时后返回同一事务；未到期或无事务时返回空请求。
    ResetRetry retryIfExpired(const Timestamp& now, double response_timeout_seconds);

    /// 校验事务身份和前端 generation，并推进内部事务状态。
    ResetReplyAction acceptReply(const FrontendResetReply& reply, const Timestamp& now);

private:
    /// 结合系统熵和单调时钟生成非零请求者身份。
    static std::uint64_t makeRequesterInstanceId() noexcept;
    /// 释放当前事务及其超时状态。
    void clearActive();

    std::uint64_t requester_instance_id_ = 0;
    std::uint64_t next_transaction_id_ = 0;
    std::uint64_t last_sent_transaction_id_ = 0;
    std::optional<FrontendResetTransaction> active_;
    bool timeout_reported_ = false;
    Timestamp last_attempt_at_;
    Timestamp last_request_at_;
    std::uint64_t frontend_instance_id_ = 0;
    std::uint64_t frontend_generation_ = 0;
};

}  // namespace backend
