/**
 * @file localization_state.cpp
 * @brief 定位状态机和状态计数器实现。
 */

#include "backend/localization_state.hpp"

#include <algorithm>
#include <limits>
#include <sstream>
#include <utility>

namespace backend {

/// 返回定位状态对应的稳定名称。
const char* LocalizationStateMachine::statusName(const LocalizationStatus status) {
    switch (status) {
        case LocalizationStatus::Initializing:
            return "INITIALIZING";
        case LocalizationStatus::Normal:
            return "NORMAL";
        case LocalizationStatus::ObsDegraded:
            return "OBS_DEGRADED";
        case LocalizationStatus::Suspect:
            return "SUSPECT";
        case LocalizationStatus::Lost:
            return "LOST";
    }
    return "UNKNOWN";
}

/// 判断当前状态是否允许使用可信全局位姿。
bool LocalizationStateMachine::poseTrusted(const LocalizationStatus status) {
    return status == LocalizationStatus::Normal || status == LocalizationStatus::ObsDegraded;
}

/// 判断当前状态是否应冻结最后可信的全局位姿。
bool LocalizationStateMachine::tfFrozen(const LocalizationStatus status) {
    return status == LocalizationStatus::Suspect || status == LocalizationStatus::Lost;
}

/// 判断当前状态是否应执行全局恢复流程。
bool LocalizationStateMachine::recoveryPath(const LocalizationStatus status) {
    return status == LocalizationStatus::Initializing || status == LocalizationStatus::Suspect ||
           status == LocalizationStatus::Lost;
}

/// 捕获状态、原因、工作代次及累计估计器故障诊断。
LocalizationStateSnapshot LocalizationStateMachine::snapshot() const {
    return {status_, reason_, generation_, estimator_failures_};
}

/// 推进非零状态代次以使旧工作身份失效。
void LocalizationStateMachine::advanceGeneration() {
    ++generation_;
    if (generation_ == 0) {
        ++generation_;
    }
}

/// 进入初始化状态并清空连续观测证据，保留累计故障诊断。
void LocalizationStateMachine::initialize(const std::string& reason) {
    status_ = LocalizationStatus::Initializing;
    reason_ = reason;
    clearEvidence();
    advanceGeneration();
}

/// 清空连续观测证据并推进工作代次，保留累计故障诊断。
void LocalizationStateMachine::onFrontendResetCompleted() {
    clearEvidence();
    // reset 即使保持 LOST，也必须创建新的工作代次。
    advanceGeneration();
}

/// 恢复被拒绝重置前的定位状态，并保留最新故障诊断及工作身份失效语义。
void LocalizationStateMachine::restoreAfterResetRejected(const LocalizationStateMachine& saved) {
    const std::uint64_t aborted_generation = generation_;
    auto estimator_failures = std::move(estimator_failures_);
    *this = saved;
    estimator_failures_ = std::move(estimator_failures);
    generation_ = aborted_generation;
    advanceGeneration();
}

/// 清空连续观测冲突和恢复失败计数。
void LocalizationStateMachine::clearEvidence() {
    consecutive_correction_conflicts_ = 0;
    consecutive_suspect_relocalization_failures_ = 0;
}

/// 将可信状态转为可疑状态并记录速度异常原因。
LocalizationTransition LocalizationStateMachine::onVelocitySpike(const double linear_speed,
                                                                 const double angular_speed) {
    LocalizationTransition transition;
    transition.previous = status_;
    transition.current = status_;
    if (!poseTrusted(status_)) {
        return transition;
    }

    std::ostringstream oss;
    oss << "velocity spike: linear=" << linear_speed << " m/s, angular=" << angular_speed
        << " rad/s";
    status_ = LocalizationStatus::Suspect;
    reason_ = oss.str();
    consecutive_correction_conflicts_ = 0;
    consecutive_suspect_relocalization_failures_ = 0;

    transition.current = status_;
    transition.reason = reason_;
    transition.changed = transition.previous != transition.current;
    if (transition.changed) {
        advanceGeneration();
    }
    return transition;
}

/// 因观测模型失效进入可疑状态并清空旧证据。
LocalizationTransition LocalizationStateMachine::onObservationModelInvalid(
    const std::string& reason) {
    LocalizationTransition transition;
    transition.previous = status_;
    transition.current = status_;
    if (!poseTrusted(status_)) {
        return transition;
    }

    status_ = LocalizationStatus::Suspect;
    reason_ = "observation model invalid: " + reason;
    clearEvidence();

    transition.current = status_;
    transition.reason = reason_;
    transition.changed = transition.previous != transition.current;
    if (transition.changed) {
        advanceGeneration();
    }
    return transition;
}

/// 按观测来源验收可信校正并进入正常状态。
LocalizationTransition LocalizationStateMachine::onTrustedCorrection(
    const std::string& reason, const TrustedCorrectionSource source) {
    LocalizationTransition transition;
    transition.previous = status_;
    transition.current = status_;
    transition.reason = reason_;
    // 普通全局观测只在可信状态提交；恢复观测可将初始化、可疑或丢失状态转为正常。
    if (source == TrustedCorrectionSource::RegularObservation && !poseTrusted(status_)) {
        return transition;
    }
    consecutive_correction_conflicts_ = 0;
    consecutive_suspect_relocalization_failures_ = 0;
    status_ = LocalizationStatus::Normal;
    reason_ = reason;

    transition.current = status_;
    transition.reason = reason_;
    transition.changed = transition.previous != transition.current;
    if (transition.changed) {
        advanceGeneration();
    }
    return transition;
}

/// 累计校正冲突，按阈值进入观测降级或可疑状态。
LocalizationTransition LocalizationStateMachine::onCorrectionConflict(
    const std::string& reason, const int conflicts_before_suspect) {
    LocalizationTransition transition;
    transition.previous = status_;
    transition.current = status_;
    transition.reason = reason_;
    if (!poseTrusted(status_)) {
        return transition;
    }

    ++consecutive_correction_conflicts_;
    consecutive_suspect_relocalization_failures_ = 0;
    const int threshold = std::max(1, conflicts_before_suspect);
    std::ostringstream oss;
    oss << "correction conflict " << consecutive_correction_conflicts_ << "/" << threshold << ": "
        << reason;
    reason_ = oss.str();
    status_ = consecutive_correction_conflicts_ >= threshold ? LocalizationStatus::Suspect
                                                             : LocalizationStatus::ObsDegraded;

    transition.current = status_;
    transition.reason = reason_;
    transition.changed = transition.previous != transition.current;
    if (transition.changed) {
        advanceGeneration();
    }
    return transition;
}

/// 强制进入丢失状态并清空旧证据。
LocalizationTransition LocalizationStateMachine::forceLost(const std::string& reason) {
    LocalizationTransition transition;
    transition.previous = status_;
    consecutive_correction_conflicts_ = 0;
    consecutive_suspect_relocalization_failures_ = 0;
    status_ = LocalizationStatus::Lost;
    reason_ = reason;

    transition.current = status_;
    transition.reason = reason_;
    transition.changed = transition.previous != transition.current;
    advanceGeneration();
    return transition;
}

/// 累计估计器故障诊断并建立新的 LOST 工作代次。
LocalizationTransition LocalizationStateMachine::onEstimatorFailure(const std::string& reason) {
    estimator_failures_.last_reason = reason;
    if (estimator_failures_.count < std::numeric_limits<std::uint64_t>::max())
        ++estimator_failures_.count;
    return forceLost(reason);
}

/// 累计恢复失败并按当前状态决定继续等待或转为丢失。
LocalizationTransition LocalizationStateMachine::onRecoveryRelocalizationFailure(
    const int suspect_failures_before_lost, const std::string& source_name) {
    LocalizationTransition transition;
    transition.previous = status_;

    if (status_ == LocalizationStatus::Suspect) {
        ++consecutive_suspect_relocalization_failures_;
        const int limit = std::max(1, suspect_failures_before_lost);
        std::ostringstream oss;
        oss << source_name << " local recovery failed "
            << consecutive_suspect_relocalization_failures_ << "/" << limit << " in SUSPECT";
        if (consecutive_suspect_relocalization_failures_ >= limit) {
            status_ = LocalizationStatus::Lost;
            oss << "; escalating to LOST";
        }
        reason_ = oss.str();
    } else if (status_ == LocalizationStatus::Lost) {
        reason_ = source_name + " global recovery failed in LOST; front-end reset pending";
    } else {
        reason_ = "initial localization via " + source_name + " pending";
    }

    transition.current = status_;
    transition.reason = reason_;
    transition.changed = transition.previous != transition.current;
    if (transition.changed) {
        advanceGeneration();
    }
    return transition;
}

}  // namespace backend
