/**
 * @file localization_state.hpp
 * @brief 定位状态机和状态计数器。
 */
#pragma once

#include <cstdint>
#include <string>

namespace backend {

/// 定位状态机。
enum class LocalizationStatus {
    Initializing,  // 等待首次成功定位
    Normal,        // 正常几何跟踪
    ObsDegraded,   // 位姿仍可信；系统外部观测连续冲突但尚未达到 SUSPECT 阈值
    Suspect,       // 位姿可疑，重定位恢复中；不重置前端
    Lost,          // 定位丢失，等待独立恢复或协调重置
};

/// 保存当前进程累计的估计器故障次数及最近一次原因。
struct EstimatorFailureHistory {
    std::uint64_t count = 0;
    std::string last_reason{};
};

struct LocalizationStateSnapshot {
    LocalizationStatus status = LocalizationStatus::Initializing;
    std::string reason = "waiting for first correction";
    /// 当前定位状态阶段的单调身份标识。
    /// 状态转移或 reset 前捕获的工作不得修改新阶段，即使枚举值后来恰好相同。
    std::uint64_t generation = 0;
    EstimatorFailureHistory estimator_failures{};
};

struct LocalizationTransition {
    LocalizationStatus previous = LocalizationStatus::Initializing;
    LocalizationStatus current = LocalizationStatus::Initializing;
    std::string reason;
    bool changed = false;
};

enum class TrustedCorrectionSource {
    RegularObservation,
    Relocalization,
};

class LocalizationStateMachine {
public:
    /// 返回定位状态对应的稳定名称。
    static const char* statusName(LocalizationStatus status);
    /// 判断当前状态是否允许使用可信全局位姿。
    static bool poseTrusted(LocalizationStatus status);
    /// 判断当前状态是否应冻结最后可信的全局位姿。
    static bool tfFrozen(LocalizationStatus status);
    /// 判断当前状态是否应执行全局恢复流程。
    static bool recoveryPath(LocalizationStatus status);

    /// 捕获状态、原因、工作代次及累计估计器故障诊断。
    LocalizationStateSnapshot snapshot() const;
    /// 返回当前定位状态。
    LocalizationStatus status() const { return status_; }
    /// 返回当前状态工作代次。
    std::uint64_t generation() const { return generation_; }

    /// 进入初始化状态并清空连续观测证据，保留累计故障诊断。
    void initialize(const std::string& reason);
    /// 清空连续观测证据并推进工作代次，保留累计故障诊断。
    void onFrontendResetCompleted();
    /// reset 端点证明尚未触碰前端且已拒绝操作时，恢复请求前的定位状态。
    /// 累计故障诊断保留最新值。
    /// 工作代次继续递增，重置请求期间 LOST 阶段的快照保持失效。
    void restoreAfterResetRejected(const LocalizationStateMachine& saved);

    /// 将可信状态转为可疑状态并记录速度异常原因。
    LocalizationTransition onVelocitySpike(double linear_speed, double angular_speed);
    /// 跟踪假设超出局部观测模型的有效范围时，立即进入 SUSPECT 并冻结可信输出。
    LocalizationTransition onObservationModelInvalid(const std::string& reason);
    /// 按观测来源验收可信校正并进入正常状态。
    LocalizationTransition onTrustedCorrection(const std::string& reason,
                                               TrustedCorrectionSource source);
    /// 一个图更新事务至少拥有一条合法外部观测，但所有观测都与更新前系统预测
    /// 冲突。每个事务只累计一次；冲突因子不会写入图。
    LocalizationTransition onCorrectionConflict(const std::string& reason,
                                                int conflicts_before_suspect);
    /// 强制建立新的 LOST 工作代次，由调用方按可用能力执行恢复或协调重置。
    LocalizationTransition forceLost(const std::string& reason);
    /// 累计估计器故障诊断并建立新的 LOST 工作代次。
    LocalizationTransition onEstimatorFailure(const std::string& reason);
    /// 记录重定位失败；SUSPECT 连续失败达阈值后进入 LOST，INITIALIZING 保持等待。
    /// LOST 状态保持不变，由调用方决定是否发起协调重置。
    LocalizationTransition onRecoveryRelocalizationFailure(int suspect_failures_before_lost,
                                                           const std::string& source_name);

private:
    /// 清空连续观测冲突和恢复失败计数。
    void clearEvidence();
    /// 推进非零状态代次以使旧工作身份失效。
    void advanceGeneration();

    int consecutive_correction_conflicts_ = 0;
    int consecutive_suspect_relocalization_failures_ = 0;
    LocalizationStatus status_ = LocalizationStatus::Initializing;
    std::string reason_ = "waiting for first correction";
    std::uint64_t generation_ = 0;
    EstimatorFailureHistory estimator_failures_;
};

}  // namespace backend
