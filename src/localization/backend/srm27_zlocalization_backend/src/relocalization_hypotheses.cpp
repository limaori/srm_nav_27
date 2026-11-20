/**
 * @file relocalization_hypotheses.cpp
 * @brief 编排位置先验、局部描述子和全局描述子的确定性回退顺序。
 */

#include <algorithm>
#include <cmath>
#include <utility>

#include "backend/logger.hpp"
#include "search.hpp"

namespace backend::relocalization_detail {

/// 首批航向假设全部失败后才执行回退扫描。
bool Search::runSeedThenFallbackSweep(const std::vector<CandidateVerificationInput>& seed_inputs,
                                      const std::vector<CandidateVerificationInput>& sweep_inputs,
                                      const char* seed_suffix, const char* sweep_suffix) {
    if (seed_inputs.empty()) {
        return false;
    }
    const bool seed_found = runCandidateSet(seed_inputs);
    if (seed_found) {
        acceptBestRelocalization(seed_suffix);
        return true;
    }

    const bool ran_sweep = !sweep_inputs.empty();
    if (ran_sweep) {
        runCandidateSet(sweep_inputs);
    }
    if (!best_.found) {
        return false;
    }
    acceptBestRelocalization(ran_sweep ? sweep_suffix : seed_suffix);
    return true;
}

/// 根据候选是否具有可信航向选择首批和回退验证批次。
bool Search::runSeedThenSweepBatches(const std::vector<CandidateVerificationInput>& base_inputs,
                                     const char* seed_suffix, const char* sweep_suffix) {
    if (base_inputs.empty()) {
        return false;
    }

    const bool all_have_yaw_seed = std::all_of(
        base_inputs.begin(), base_inputs.end(),
        [](const CandidateVerificationInput& input) { return input.hit.yaw_seed_rad.has_value(); });
    if (!all_have_yaw_seed) {
        const YawSweepSchedule schedule =
            makeYawSweepSchedule(config_.relocalization_yaw_sweep_count,
                                 /*has_trusted_seed=*/false);
        runCandidateSet(buildDescriptorBatchInputs(base_inputs, schedule.primary_offsets));
        if (!best_.found) {
            return false;
        }
        acceptBestRelocalization(sweep_suffix);
        return true;
    }

    const YawSweepSchedule schedule = makeYawSweepSchedule(config_.relocalization_yaw_sweep_count,
                                                           /*has_trusted_seed=*/true);
    return runSeedThenFallbackSweep(
        buildDescriptorBatchInputs(base_inputs, schedule.primary_offsets),
        buildDescriptorBatchInputs(base_inputs, schedule.fallback_offsets), seed_suffix,
        sweep_suffix);
}

/// 召回位置邻近候选并执行位置先验约束下的航向验证。
bool Search::tryPositionPrior() {
    const RelocalizationPositionPrior& prior = *request_.position_prior;
    const std::vector<int> indices = nearestPriorKeyframeIndices(
        prior_kf_positions_, prior.map_point, prior.use_z, prior.max_candidates);
    if (indices.empty()) {
        BACKEND_WARN_STREAM("tryRelocalization: prior map has no keyframes for "
                            << prior.source << " position-prior verification");
        return false;
    }

    std::vector<CandidateVerificationInput> base_inputs;
    base_inputs.reserve(indices.size());
    for (std::size_t order = 0; order < indices.size(); ++order) {
        CandidateVerificationInput input;
        if (buildPositionCandidateInput(indices[order], &input)) {
            base_inputs.push_back(std::move(input));
        }
    }
    if (base_inputs.empty()) {
        return false;
    }

    const YawSweepSchedule schedule = makeYawSweepSchedule(config_.relocalization_yaw_sweep_count,
                                                           prior.yaw_seed_rad.has_value());
    if (!prior.yaw_seed_rad.has_value()) {
        runCandidateSet(buildPositionBatchInputs(base_inputs, schedule.primary_offsets));
        if (!best_.found) {
            return false;
        }
        acceptBestRelocalization(", matched via position-prior full yaw sweep batch");
        return true;
    }

    if (!prior.allow_yaw_sweep_fallback) {
        runCandidateSet(buildPositionBatchInputs(base_inputs, schedule.primary_offsets));
        if (!best_.found) {
            return false;
        }
        acceptBestRelocalization(", matched via position-prior yaw seed batch");
        return true;
    }

    return runSeedThenFallbackSweep(
        buildPositionBatchInputs(base_inputs, schedule.primary_offsets),
        buildPositionBatchInputs(base_inputs, schedule.fallback_offsets),
        ", matched via position-prior yaw seed batch",
        ", matched via position-prior fallback yaw sweep batch");
}

/// 验证局部描述子召回候选并保持确定性顺序。
bool Search::tryLocalCandidates() {
    std::vector<CandidateVerificationInput> inputs;
    inputs.reserve(candidates_.size());
    for (std::size_t i = 0; i < candidates_.size(); ++i) {
        CandidateVerificationInput input;
        if (buildCandidateInput(candidates_[i], &input)) {
            inputs.push_back(std::move(input));
        }
    }

    if (inputs.empty()) {
        return false;
    }

    return runSeedThenSweepBatches(inputs, ", matched via local descriptor yaw seed batch",
                                   ", matched via local full yaw sweep batch");
}

/// 在预算内验证尚未尝试的全局描述子候选。
bool Search::tryGlobalFallback() {
    if (!can_global_supplement_) {
        return false;
    }

    std::vector<int> seen_indices;
    seen_indices.reserve(candidates_.size() + static_cast<std::size_t>(global_fallback_budget_));
    for (const auto& candidate : candidates_) {
        const int index = candidate.index;
        if (index >= 0 &&
            std::find(seen_indices.begin(), seen_indices.end(), index) == seen_indices.end()) {
            seen_indices.push_back(index);
        }
    }

    const int request_size = std::max(
        global_fallback_budget_, static_cast<int>(seen_indices.size()) + global_fallback_budget_);
    const std::vector<LoopCandidate> global_candidates = descriptor_.queryLocalizationCandidates(
        *level_query_cloud_, /*allowed_indices=*/{}, request_size);

    BACKEND_WARN_STREAM("tryRelocalization: local candidates did not pass; trying global "
                        << backend_name_ << " fallback top " << global_fallback_budget_);

    std::vector<CandidateVerificationInput> inputs;
    inputs.reserve(static_cast<std::size_t>(global_fallback_budget_));
    for (const auto& candidate : global_candidates) {
        if (global_fallback_considered_ >= global_fallback_budget_) {
            break;
        }
        const LoopCandidate& hit = candidate;
        if (hit.index < 0) {
            continue;
        }
        if (std::find(seen_indices.begin(), seen_indices.end(), hit.index) != seen_indices.end()) {
            continue;
        }
        seen_indices.push_back(hit.index);
        ++global_fallback_considered_;
        if (!descriptorGatePassed(hit)) {
            BACKEND_WARN_STREAM("tryRelocalization: global fallback KF "
                                << hit.index
                                << " skipped (descriptor_distance=" << hit.descriptor_distance
                                << " > limit " << descriptor_distance_limit_ << ")");
            continue;
        }
        CandidateVerificationInput input;
        if (buildCandidateInput(candidate, &input)) {
            inputs.push_back(std::move(input));
        }
    }

    return runSeedThenSweepBatches(inputs, ", matched via global descriptor yaw seed batch",
                                   ", matched via global full yaw sweep batch");
}

/// 记录已接受恢复结果的来源、航向和质量诊断。
void Search::acceptBestRelocalization(const char* suffix) const {
    if (request_.position_prior != nullptr) {
        BACKEND_INFO_STREAM("tryRelocalization: matched KF "
                            << best_.index << " source=" << best_.source
                            << " hypothesis=" << best_.hypothesis
                            << " (yaw_seed=" << best_.descriptor_yaw * 180.0 / M_PI
                            << "deg, residual_score=" << best_.residual_score << ", quality={"
                            << qualitySummary(best_.quality) << "}"
                            << ", tried_verifications=" << tried_verifications_ << suffix << ")");
        return;
    }
    BACKEND_INFO_STREAM("tryRelocalization: matched KF "
                        << best_.index << " hypothesis=" << best_.hypothesis
                        << " (descriptor_distance=" << best_.descriptor_distance
                        << ", descriptor_score=" << best_.descriptor_score
                        << ", descriptor_yaw=" << best_.descriptor_yaw * 180.0 / M_PI
                        << "deg, residual_score=" << best_.residual_score << ", quality={"
                        << qualitySummary(best_.quality) << "}"
                        << ", tried_verifications=" << tried_verifications_ << suffix << ")");
}

/// 将最佳候选转换为可提交的完整恢复结果。
RelocalizationResult Search::makeResult() const {
    RelocalizationResult result;
    result.success = best_.found;
    result.T_map_odom = best_.T_map_odom;
    result.prior_index = best_.index;
    result.source = best_.source;
    result.covariance = best_.covariance;
    result.covariance_valid = best_.covariance_valid;
    return result;
}

}  // namespace backend::relocalization_detail
