/**
 * @file observation.cpp
 * @brief 捕获与联合检验全局观测，提交可信图更新。
 */

#include "backend/observation.hpp"

#include <algorithm>
#include <sstream>
#include <utility>

#include "backend/localization.hpp"
#include "backend/logger.hpp"
#include "backend/measurement.hpp"

namespace backend {
/// 冻结当前可信关键帧及其全局观测所需的因果输入。
std::optional<GlobalObservationWork> Localization::captureTracking() {
    std::lock_guard<std::mutex> lock(mutex_);
    GlobalObservationWork work;

    if (!LocalizationStateMachine::poseTrusted(state_.status()) || trajectory_.empty()) {
        return std::nullopt;
    }
    work.state_generation = state_.generation();
    work.runtime = trajectory_.latest();
    work.prior_input.configured =
        config_.prior_map_enabled && work.runtime.index != last_lidar_observation_runtime_index_;
    work.uwb = uwb_.captureFresh(work.runtime);
    if ((!work.prior_input.configured && !work.uwb) ||
        !tryStartCycle(clock_.nowSteadyTimestamp(), config_.tracking.correction_period,
                       last_global_observation_cycle_time_)) {
        return std::nullopt;
    }
    work.predicted_map_pose = poses_.transform() * work.runtime.odom_pose;
    if (!work.prior_input.configured)
        return work;

    if (frames_.trackingSource().has_value() &&
        frames_.trackingSource()->runtime_index == work.runtime.index &&
        frames_.trackingSource()->reference.epoch == work.runtime.epoch) {
        work.source_snapshot = *frames_.trackingSource();
    } else {
        // 兼容外部预填充轨迹：缺少冻结快照时退化为单帧输入。
        work.source_snapshot_missing = true;
        work.source_snapshot.runtime_index = work.runtime.index;
        work.source_snapshot.reference.epoch = work.runtime.epoch;
        work.source_snapshot.reference.timestamp = work.runtime.timestamp;
        work.source_snapshot.reference.odom_pose = work.runtime.odom_pose;
        work.fallback_source = work.runtime.cloud;
    }
    work.previous_reference_index = last_tracking_reference_index_;
    return work;
}
/// NORMAL 全局观测从线性化到状态提交期间共享的临时事务数据。
struct GlobalObservationTransaction {
    PoseSqrtInformation prior_information_right;
    ObservationBatch observations;
    bool publish_state = false;
};

namespace {
/// 将同周期的观测诊断原因连接成状态说明。
std::string joinEvidence(const std::vector<std::string>& items) {
    std::ostringstream out;
    for (std::size_t i = 0; i < items.size(); ++i) {
        if (i)
            out << "; ";
        out << items[i];
    }
    return out.str();
}
/// 按统计有效性及统一卡方上尾概率阈值验收创新结果。
bool passesInnovation(const InnovationAssessment& value, double rejection_probability) {
    return value.valid && value.upper_tail_probability >= rejection_probability;
}

/// 报告身份失效或图节点过期的全局观测，返回该事务是否应停止后续发布。
bool reportDiscardedObservation(const GlobalObservationWork& work,
                                const GlobalObservationCommit& commit) {
    if (commit.stale) {
        BACKEND_INFO_STREAM("discarding stale global-observation work: runtime="
                            << work.runtime.index << ", epoch=" << work.runtime.epoch
                            << ", generation=" << work.state_generation);
        return true;
    }
    if (commit.graph_node_unavailable) {
        BACKEND_WARN_STREAM(
            "discarding expired global-observation work without health "
            "evidence: runtime="
            << work.runtime.index << " graph node unavailable");
        return true;
    }
    return false;
}

}  // namespace

/// 按输入顺序复制图接口所需的线性化项，保持统计结果与条目一一对应。
std::vector<InnovationTerm> ObservationBatch::terms() const {
    std::vector<InnovationTerm> result;
    result.reserve(entries.size());
    for (const auto& entry : entries)
        result.push_back(entry.innovation);
    return result;
}

/// 检查联合统计及逐项数量；不可用诊断不产生观测冲突。
bool ObservationBatch::validate(const InnovationBatchAssessment& assessment) {
    if (assessment.joint.valid && assessment.individual.size() == entries.size())
        return true;
    unavailable.push_back("joint system innovation unavailable: " +
                          (assessment.joint.invalid_reason.empty()
                               ? "incomplete per-source diagnostics"
                               : assessment.joint.invalid_reason));
    return false;
}

/// 联合一致时接受全部来源，否则仅保留唯一通过单项检验的来源。
void ObservationBatch::classify(const InnovationBatchAssessment& assessment,
                                double rejection_probability) {
    const bool joint_accepted = passesInnovation(assessment.joint, rejection_probability);
    const auto passing_count = std::count_if(
        assessment.individual.begin(), assessment.individual.end(),
        [&](const auto& item) { return passesInnovation(item, rejection_probability); });
    for (std::size_t index = 0; index < entries.size(); ++index) {
        auto& entry = entries[index];
        const auto& individual = assessment.individual[index];
        // 联合冲突仅隔离唯一通过的来源；多个单项合理时不任意挑选其中一个。
        entry.accepted = joint_accepted || (entries.size() > 1U && passing_count == 1 &&
                                            passesInnovation(individual, rejection_probability));
        std::ostringstream evidence;
        evidence << entry.label << " NIS=" << individual.nis << "/" << individual.degrees_of_freedom
                 << ", p=" << individual.upper_tail_probability;
        if (entry.accepted)
            accepted_evidence.push_back(evidence.str());
        else
            conflicts.push_back(evidence.str());
    }
    if (!joint_accepted) {
        std::ostringstream evidence;
        evidence << "joint system NIS=" << assessment.joint.nis << "/"
                 << assessment.joint.degrees_of_freedom
                 << ", p=" << assessment.joint.upper_tail_probability;
        conflicts.push_back(evidence.str());
    }
}

/// 汇总接受、冲突和不可用证据，供唯一提交端推进状态机。
ObservationConclusion ObservationBatch::conclusion() const {
    ObservationConclusion result;
    result.had_conflict = !conflicts.empty();
    if (!accepted_evidence.empty()) {
        result.outcome = ObservationOutcome::Accepted;
        result.reason = "system global observation accepted: " + joinEvidence(accepted_evidence);
        if (result.had_conflict)
            result.reason += "; rejected sibling evidence: " + joinEvidence(conflicts);
    } else if (result.had_conflict) {
        result.outcome = ObservationOutcome::Conflict;
        result.reason = "all available global observations conflicted: " + joinEvidence(conflicts);
    }
    return result;
}

/// 收集尚未消费且可线性化的全局观测及不可用原因。
void Localization::collectGlobalInnovationsLocked(const GlobalObservationWork& work,
                                                  const PriorMapObservation& prior_observation,
                                                  GlobalObservationTransaction& transaction) const {
    const ObservationEvidence& prior = prior_observation.evidence;
    if (prior.configured && prior.disposition == ObservationDisposition::Conflict) {
        transaction.observations.conflicts.push_back(prior.reason);
    } else if (prior.configured && prior.disposition == ObservationDisposition::Measurement) {
        transaction.prior_information_right = leftToRightPoseSqrtInformation(
            prior_observation.registration.source_to_target,
            prior_observation.registration.measurement_information.sqrt_information);
        InnovationTerm term =
            graph_.linearizeAbsolutePose(work.runtime.index, prior_observation.measured_map_pose,
                                         transaction.prior_information_right);
        if (term.valid) {
            transaction.observations.entries.push_back(
                {ObservationSource::PriorMap, "prior scan-to-map", std::move(term)});
        } else {
            transaction.observations.unavailable.push_back(
                "prior scan-to-map innovation unavailable: " + term.invalid_reason);
        }
    } else if (prior.configured && !prior.reason.empty()) {
        transaction.observations.unavailable.push_back(prior.reason);
    }

    const bool fresh_measurement = work.uwb && work.uwb->timestamp > uwb_.lastConsumed();
    if (!fresh_measurement) {
        return;
    }
    InnovationTerm term = uwb_.linearize(work.runtime.index, *work.uwb);
    if (term.valid) {
        transaction.observations.entries.push_back(
            {ObservationSource::Uwb, "UWB", std::move(term)});
    } else {
        transaction.observations.unavailable.push_back("UWB innovation unavailable: " +
                                                       term.invalid_reason);
    }
}

/// 执行共同预测下的联合创新检验并完成有效 UWB 观测消费。
void Localization::assessGlobalInnovationsLocked(const GlobalObservationWork& work,
                                                 GlobalObservationTransaction& transaction,
                                                 GlobalObservationCommit& commit) {
    auto& observations = transaction.observations;
    if (observations.entries.empty())
        return;
    const InnovationBatchAssessment batch = graph_.assessInnovationBatch(observations.terms());
    commit.joint_innovation = batch.joint;
    if (!observations.validate(batch))
        return;

    // 完整统计成立后先消费物理解，再执行接纳政策和任何图更新。
    for (std::size_t index = 0; index < observations.entries.size(); ++index) {
        switch (observations.entries[index].source) {
            case ObservationSource::PriorMap:
                commit.prior_innovation = batch.individual[index];
                break;
            case ObservationSource::Uwb:
                uwb_.finishAssessment(*work.uwb, batch.joint);
                commit.uwb_innovation = batch.individual[index];
                break;
        }
    }
    observations.classify(batch, config_.innovation.rejection_probability);
    for (const auto& entry : observations.entries) {
        switch (entry.source) {
            case ObservationSource::PriorMap:
                commit.prior_accepted |= entry.accepted;
                break;
            case ObservationSource::Uwb:
                commit.uwb_accepted |= entry.accepted;
                break;
        }
    }
}

/// 插入已通过创新检验的因子并更新图及在线标定。
void Localization::applyGlobalObservationFactorsLocked(const GlobalObservationWork& work,
                                                       const PriorMapObservation& prior_observation,
                                                       GlobalObservationTransaction& transaction,
                                                       GlobalObservationCommit& commit) {
    if (commit.prior_accepted) {
        // 在线 UWB 对齐只使用完整锚；部分秩点云观测仍可正常进入图。
        if (prior_observation.registration.anchorValid()) {
            uwb_.observeAnchor(work.runtime, prior_observation.measured_map_pose);
        }
        graph_.addAbsolutePoseInformation(work.runtime.index, prior_observation.measured_map_pose,
                                          transaction.prior_information_right);
    }
    if (commit.uwb_accepted) {
        uwb_.insert(work.runtime.index, *work.uwb);
    }
    if (!transaction.observations.accepted_evidence.empty()) {
        commit.applied_correction = updateGraphLocked(/*global_factor_added=*/true);
    }
}

/// 根据同周期接受、冲突及不可用证据推进定位状态。
void Localization::updateGlobalObservationStateLocked(GlobalObservationTransaction& transaction,
                                                      GlobalObservationCommit& commit) {
    const auto conclusion = transaction.observations.conclusion();
    commit.had_conflict = conclusion.had_conflict;
    if (conclusion.outcome == ObservationOutcome::Unavailable)
        return;
    if (conclusion.outcome == ObservationOutcome::Accepted) {
        commit.transition = state_.onTrustedCorrection(conclusion.reason,
                                                       TrustedCorrectionSource::RegularObservation);
        transaction.publish_state = true;
        return;
    }
    commit.transition =
        state_.onCorrectionConflict(conclusion.reason, config_.innovation.conflicts_before_suspect);
    transaction.publish_state = true;
    if (commit.transition.changed && commit.transition.current == LocalizationStatus::Suspect) {
        beginSuspectRecoveryLocked();
    }
}

/// 原子提交全局观测的联合创新结论、图因子和定位状态。
GlobalObservationCommit Localization::commit(const GlobalObservationWork& work,
                                             const PriorMapObservation& prior_observation) {
    GlobalObservationCommit commit;
    GlobalObservationTransaction transaction;
    withOutputCommit([&] {
        const ObservationEvidence* invalid_evidence = nullptr;
        if (prior_observation.evidence.configured &&
            prior_observation.evidence.disposition == ObservationDisposition::ModelInvalid) {
            invalid_evidence = &prior_observation.evidence;
        }

        if (work.runtime.epoch != active_epoch_ || work.state_generation != state_.generation() ||
            !LocalizationStateMachine::poseTrusted(state_.status())) {
            commit.stale = true;
        } else if (!graph_.hasEstimate(work.runtime.index)) {
            commit.graph_node_unavailable = true;
        } else {
            if (prior_observation.evidence.configured)
                last_lidar_observation_runtime_index_ = work.runtime.index;
            if (invalid_evidence != nullptr) {
                // 局部模型失效是系统事件，不能被同周期的兄弟传感器掩盖。
                commit.model_invalid = true;
                commit.transition = state_.onObservationModelInvalid(
                    invalid_evidence->reason.empty() ? "global observation model invalid"
                                                     : invalid_evidence->reason);
                if (commit.transition.changed &&
                    commit.transition.current == LocalizationStatus::Suspect) {
                    beginSuspectRecoveryLocked();
                }
                transaction.publish_state = commit.transition.changed;
            } else {
                collectGlobalInnovationsLocked(work, prior_observation, transaction);
                assessGlobalInnovationsLocked(work, transaction, commit);
                applyGlobalObservationFactorsLocked(work, prior_observation, transaction, commit);
                updateGlobalObservationStateLocked(transaction, commit);
            }
        }
    });

    if (reportDiscardedObservation(work, commit))
        return commit;
    for (const std::string& reason : transaction.observations.unavailable) {
        BACKEND_WARN_STREAM("global observation unavailable (state unchanged): " << reason);
    }
    if (transaction.publish_state) {
        publishState();
    }
    if (commit.transition.changed) {
        BACKEND_WARN_STREAM("localization state -> "
                            << LocalizationStateMachine::statusName(commit.transition.current)
                            << " (" << commit.transition.reason << ")");
    } else if (commit.had_conflict && !commit.prior_accepted && !commit.uwb_accepted) {
        BACKEND_WARN_STREAM("system global observation rejected: " << commit.transition.reason);
    }
    return commit;
}
/// 复核跟踪任务的前端与状态代次，并记录所选参考关键帧。
bool Localization::claim(GlobalObservationWork& work) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (work.runtime.epoch != active_epoch_ || work.state_generation != state_.generation())
        return false;
    if (work.reference.index >= 0)
        last_tracking_reference_index_ = work.reference.index;
    return true;
}

}  // namespace backend
