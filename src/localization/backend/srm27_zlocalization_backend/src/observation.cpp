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
/// 以最近 0.5 s 里程计的**路径长度**估计车体角速度，并维护带迟滞的自转闸门。
///
/// 2026-11-27 实车（real_zloc_1127_161502）复盘：旧实现只看 `t-0.1 s` 的**净转角**。
/// 云台扫到 ±180° 反向时，0.1 s 内正反转抵消、净转角≈0，但实际仍以 ~200°/s 在摆；
/// 同时 9.3 Hz 的前端帧让 `poseAt(t-0.1, 0.05)` 约 13% 返回 nullopt（样本间隔
/// 0.107 s，最近样本最大差 0.054 s > 0.05）。两个洞叠加，自转 57 s 里仍有约 22% 的
/// 观测被放行 → 连续冲突 → 冻结 → 冻结投影让 map→odom 跟着云台转。
///
/// 这里改成：
///   * 0.5 s 窗口内逐段求 |Δθ| 再求和（路径长度），反向不会把已转过的角度抵消；
///   * 采样容差放宽到 0.2 s，稀疏/抖动的前端帧不再让闸门静默失效；
///   * 迟滞：超过阈值后至少锁 0.5 s，速度降到阈值一半以下且锁定到期才重新开闸。
///
/// 调用方须持有 mutex_；读的是里程计历史（至少保留 2 s），不是会被裁剪的 frame_history。
bool Localization::bodyRateGateLocked() {
    const double high = config_.tracking.max_body_angular_rate_rad;
    if (high <= 0.0 || !frames_.hasLatest()) {
        return false;
    }
    const FrontendFrame& newest = frames_.latest();
    // 前端换代时时间轴可能重启：旧的截止时间在新时间轴里可能是"很久以后"，
    // 不清零会把闸门永久锁死。
    if (newest.epoch != body_rate_gate_epoch_) {
        body_rate_gate_epoch_ = newest.epoch;
        body_rate_gate_until_ = 0.0;
    }
    constexpr double kWindowSeconds = 0.5;
    constexpr int kIntervals = 5;
    constexpr double kStepSeconds = kWindowSeconds / kIntervals;
    constexpr double kSampleToleranceSeconds = 0.2;
    constexpr double kHoldoffSeconds = 0.5;

    const TimedPoseBuffer& odom = frames_.odometry();
    double path_rad = 0.0;
    double elapsed_seconds = 0.0;
    bool have_previous = false;
    Eigen::Matrix4d previous = Eigen::Matrix4d::Identity();
    for (int index = kIntervals; index >= 0; --index) {
        const auto pose =
            odom.poseAt(newest.timestamp - index * kStepSeconds, kSampleToleranceSeconds);
        if (!pose.has_value()) {
            continue;
        }
        if (have_previous) {
            const Eigen::Matrix4d relative = previous.inverse() * *pose;
            if (relative.allFinite()) {
                path_rad += rotationAngleFromMatrix(relative);
                elapsed_seconds += kStepSeconds;
            }
        }
        previous = *pose;
        have_previous = true;
    }
    if (elapsed_seconds <= 1.0e-6) {
        return false;
    }

    const double rate = path_rad / elapsed_seconds;
    if (rate > high) {
        body_rate_gate_until_ = newest.timestamp + kHoldoffSeconds;
        return true;
    }
    // 迟滞：还在锁定期内且速度没有降到阈值一半以下，继续锁；否则按到期时间自然开闸。
    if (newest.timestamp < body_rate_gate_until_ && rate > 0.5 * high) {
        body_rate_gate_until_ = newest.timestamp + kHoldoffSeconds;
        return true;
    }
    return newest.timestamp < body_rate_gate_until_;
}

/// 冻结当前可信关键帧及其全局观测所需的因果输入。
std::optional<GlobalObservationWork> Localization::captureTracking() {
    std::lock_guard<std::mutex> lock(mutex_);
    GlobalObservationWork work;

    if (!LocalizationStateMachine::poseTrusted(state_.status()) || trajectory_.empty()) {
        return std::nullopt;
    }
    work.state_generation = state_.generation();
    work.runtime = trajectory_.latest();
    // 快速自转期间跳过先验观测（见 TrackingConfig::max_body_angular_rate_rad）：
    // 这时预测位姿本身对不齐，scan-to-map 会以 NIS 88~326 全线冲突，连 7 次就把状态
    // 推到 SUSPECT 并冻结。跳过只是"这一周期不产生观测"，不计冲突、不推进状态机，
    // 定位继续按 odom 递推；转速降下来后自动恢复纠正。
    const bool spinning_too_fast = bodyRateGateLocked();
    // initialization_only：首次先验匹配验收后，先验不再参与任何全局观测；
    // 若同时启用了 UWB 等独立来源，跟踪周期仍可为它们运行。
    const bool prior_observations_allowed =
        config_.prior_map_enabled && !spinning_too_fast &&
        !(config_.prior_map_initialization_only && prior_initialization_only_done_);
    work.prior_input.configured =
        prior_observations_allowed && work.runtime.index != last_lidar_observation_runtime_index_;
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
    /// 本次先验观测是"累计锚点一致证据放行"换来的：提交端要重落锚，证据也会写进 reason。
    bool prior_released = false;
    /// 放行时的越界幅度（相对锚点），只用于日志。
    double released_translation = 0.0;
    double released_rotation_rad = 0.0;
    int released_pending_frames = 0;
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
        // 一致证据放行的先验观测：越界到超过 NIS 门不是"配准错了"，而是"预测错了"
        // （累计锚点已经用连续 5 帧一致性证明过这一点）。这里强制接纳一次，
        // 让位姿被拉回测量值，提交后再重落锚；否则只会把 conflict 从锚点换到 NIS，
        // 状态机照样冻结 —— 2026-11-27 19:39 包就是这么死的。
        std::string release_note;
        if (entry.forced_accept && !entry.accepted) {
            entry.accepted = true;
            release_note = ", accepted by consistent-evidence release";
        }
        std::ostringstream evidence;
        evidence << entry.label << " NIS=" << individual.nis << "/" << individual.degrees_of_freedom
                 << ", p=" << individual.upper_tail_probability << release_note;
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
        // 累计预算：逐次上限只约束"这一步"，挡不住"每帧 15°、2 Hz 就 30°/s"的稳态旋转
        // （闸门比的是当前预测，上一次吃进去的错误已经变成新基准）。这里用不随错误移动的
        // 锚点判**累计**偏差，越界即当作冲突证据，让状态机走 conflict → SUSPECT 冻结，
        // 而不是一路把 map->odom 转过去。判据见 backend/correction_anchor.hpp。
        //
        // 只有绝对预算会死锁（真实漂移一旦超过 0.8 m 就再没有"安静帧"可以重落锚，
        // 2026-11-27 19:39 包冻结 70 s 就是这么来的），所以再看一道"一致证据放行"：
        // 连续 release_frames 帧彼此一致且幅度有界时放行一次，提交后重落锚。
        const Eigen::Matrix4d candidate_T_map_odom =
            prior_observation.measured_map_pose * work.runtime.odom_pose.inverse();
        const AnchorDecision decision = poses_.judgeCorrection(candidate_T_map_odom);
        if (decision.verdict == AnchorVerdict::Reject) {
            const CorrectionAnchor& anchor = poses_.correctionAnchor();
            std::ostringstream reason;
            reason << "prior scan-to-map over cumulative correction anchor: delta_r_deg="
                   << decision.rotation_rad * 180.0 / M_PI << " (limit "
                   << anchor.config().max_rotation_rad * 180.0 / M_PI << "), delta_t="
                   << decision.translation << "m (limit " << anchor.config().max_translation
                   << "m)";
            if (anchor.config().release_enabled) {
                reason << ", consistent " << decision.pending_frames << "/"
                       << std::max(1, anchor.config().release_frames) << " frames";
                if (!decision.within_release_cap) {
                    reason << " (outside release cap " << anchor.config().release_max_translation
                           << "m / " << anchor.config().release_max_rotation_rad * 180.0 / M_PI
                           << "deg)";
                }
            }
            transaction.observations.conflicts.push_back(reason.str());
        } else {
            const bool released = decision.verdict == AnchorVerdict::Release;
            if (released) {
                transaction.prior_released = true;
                transaction.released_translation = decision.translation;
                transaction.released_rotation_rad = decision.rotation_rad;
                transaction.released_pending_frames = decision.pending_frames;
            }
            transaction.prior_information_right = leftToRightPoseSqrtInformation(
                prior_observation.registration.source_to_target,
                prior_observation.registration.measurement_information.sqrt_information);
            InnovationTerm term =
                graph_.linearizeAbsolutePose(work.runtime.index,
                                             prior_observation.measured_map_pose,
                                             transaction.prior_information_right);
            if (term.valid) {
                ObservationEntry entry{ObservationSource::PriorMap, "prior scan-to-map",
                                       std::move(term)};
                // 放行是一次性的：越过 NIS 门也要让它进图，否则 conflict 只是换个理由而已。
                entry.forced_accept = released;
                transaction.observations.entries.push_back(std::move(entry));
            } else {
                transaction.observations.unavailable.push_back(
                    "prior scan-to-map innovation unavailable: " + term.invalid_reason);
            }
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
        // 锚点只在"安静"的修正后前移：正常小抖动/缓慢真实漂移会跟随，
        // 连续同向的大步修正不追认，累计到上限就停。
        // 例外是"一致证据放行"的那一次：不跟着挪，下一帧就会被同一道闸再拦一次。
        const bool released = transaction.prior_released && commit.prior_accepted;
        poses_.noteCorrection(poses_.transform(), commit.applied_correction.rotation_rad,
                              commit.applied_correction.translation, released);
        if (released) {
            BACKEND_WARN_STREAM(
                "prior scan-to-map accepted by consistent-evidence release after "
                << transaction.released_pending_frames << " agreeing frames: delta_t="
                << transaction.released_translation << "m, delta_r_deg="
                << transaction.released_rotation_rad * 180.0 / M_PI
                << " (anchor limits " << poses_.correctionAnchor().config().max_translation
                << "m / " << poses_.correctionAnchor().config().max_rotation_rad * 180.0 / M_PI
                << "deg); applied_t=" << commit.applied_correction.translation << "m, applied_r_deg="
                << commit.applied_correction.rotation_rad * 180.0 / M_PI
                << "; anchor re-seated, release count=" << poses_.correctionAnchor().releaseCount());
        }
    }
    // 一次性先验：若初始位姿来自 UWB 等其它来源，首次被验收的先验观测同样会锁存，
    // 此后跟踪周期不再产生先验观测。
    if (config_.prior_map_enabled && config_.prior_map_initialization_only &&
        commit.prior_accepted && !prior_initialization_only_done_) {
        prior_initialization_only_done_ = true;
        BACKEND_INFO_STREAM(
            "prior_map.initialization_only: first prior observation accepted; "
            "map->odom frozen, localization continues on frontend odometry");
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
