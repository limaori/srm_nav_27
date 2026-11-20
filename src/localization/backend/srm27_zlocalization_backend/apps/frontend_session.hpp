/**
 * @file frontend_session.hpp
 * @brief 仲裁前端实例、代次与序号，并协调重置回复的身份确认。
 */
#pragma once

#include <cstdint>
#include <map>
#include <mutex>
#include <optional>
#include <set>

#include "backend/ports.hpp"

namespace backend::ros {
/// 保存活动前端、输入故障与待确认事务，统一决定输入准入。
class FrontendSession {
public:
    /// 描述输入准入或需要执行的协调动作。
    enum class Admission { Accept, Drop, ResetRequired, InputDiscontinuity, InputFailure };
    /// 固定一次传输尝试的业务身份、前端目标及身份观察版本。
    struct ResetAttempt {
        FrontendResetTransaction request;
        std::uint64_t target_epoch;
        std::uint64_t identity_revision;
        std::uint64_t rpc_attempt;
    };
    /// 返回待处理的身份确认、输入中断或普通失败，无待处理事件时返回 Drop。
    Admission pendingAction();
    /// 记录身份、代次及中断通知，并按确认状态和帧序号决定准入。
    Admission observe(std::uint64_t instance, std::uint64_t epoch,
                      std::optional<std::uint64_t> sequence = std::nullopt,
                      bool reset_required = false, bool failed = false);

    /// 每次 RPC 单独捕获身份版本；业务幂等键及已发送目标不因重试改变。
    std::optional<ResetAttempt> prepareReset(const FrontendResetTransaction& request);

    /// 记录当前业务事务实际发出的传输尝试编号。
    void markSent(const ResetAttempt& attempt);

    /// 使用当前回复对应的 RPC 尝试确认身份，仅接纳最近一次候选身份变化后发出的尝试。
    /// 成功暖重置的源坐标须属于活动前端，其代次同步转换为后端坐标身份。
    bool translateReply(FrontendResetReply& reply, const ResetAttempt& attempt);

    /// 将已确认的前端身份安装为活动会话并重置帧序号水位。
    void commitBoundary(std::uint64_t backend_epoch);

    /// 撤销尚未发出传输请求的待确认事务。
    void cancelUnsent(const FrontendResetTransaction& request);

private:
    struct Identity {
        std::uint64_t instance, epoch;
    };
    /// 保存每个进程最新已观察代次及该代次锁存的输入故障。
    struct ObservedSession {
        std::uint64_t epoch;
        bool interrupted = false;
        bool failed = false;
    };
    struct Pending {
        FrontendResetTransaction request;
        std::uint64_t target;
        std::optional<Identity> applied;
        std::optional<std::uint64_t> sent_attempt;
    };
    /// 在持锁状态下按身份确认、普通失败及输入中断的优先级选择动作。
    Admission pendingActionLocked() const;
    /// 判断已观察前端是否与唯一活动身份及代次完全一致。
    bool needsConfirmation() const;
    /// 比较两个重置请求的请求者、业务编号和目标代次。
    static bool same(const FrontendResetTransaction& a, const FrontendResetTransaction& b);
    std::mutex mutex_;
    std::optional<Identity> active_;
    std::map<std::uint64_t, ObservedSession> observed_;
    std::uint64_t identity_revision_ = 0;
    std::uint64_t next_attempt_ = 0;
    std::optional<Pending> pending_;
    std::set<std::uint64_t> retired_;
    std::map<std::uint64_t, std::uint64_t> generations_;
    std::uint64_t sequence_ = 0;
    bool input_interrupted_ = false;
    bool input_failed_ = false;
};
}  // namespace backend::ros
