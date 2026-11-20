/**
 * @file relocalization_verification.cpp
 * @brief 构造航向假设并并行执行确定性的 scan-to-map 验证。
 */

#include <algorithm>
#include <cmath>
#include <sstream>
#include <thread>
#include <utility>

#include "backend/geometry.hpp"
#include "backend/logger.hpp"
#include "backend/math.hpp"
#include "parallel.hpp"
#include "search.hpp"

namespace backend::relocalization_detail {

/// 按确定性质量排序更新最佳恢复结果并转换协方差坐标。
void Search::recordIfBest(RelocalizationBest& best, const CandidateVerificationInput& input,
                          const VerificationEntry& entry,
                          const VerificationRunResult& result) const {
    if (best.found && !betterScanToMapRegistration(result.residual_score, result.quality,
                                                   best.residual_score, best.quality)) {
        return;
    }
    const Eigen::Matrix4d T_map_runtime_corrected = entry.T_map_prior * result.measured;
    best.found = true;
    best.index = input.hit.index;
    best.source = input.source;
    best.descriptor_distance = input.hit.descriptor_distance;
    best.descriptor_score = input.hit.descriptor_score;
    best.descriptor_yaw = entry.yaw_seed;
    best.residual_score = result.residual_score;
    best.covariance = result.covariance_valid
                          ? leftToRightPoseCovariance(result.measured, result.covariance)
                          : PoseCovariance6d::Zero();
    best.covariance_valid = result.covariance_valid;
    best.quality = result.quality;
    best.T_map_odom = T_map_runtime_corrected * request_.runtime_kf.odom_pose.inverse();
    best.hypothesis = entry.label;
}

/// 格式化航向初值及相对角度偏移的诊断标签。
std::string Search::yawSampleLabel(const std::string& prefix, const int offset,
                                   const double step_deg) {
    if (offset == 0) {
        return prefix;
    }
    std::ostringstream oss;
    oss << prefix;
    if (offset > 0) {
        oss << "+";
    }
    oss << offset * step_deg << "deg";
    return oss.str();
}

/// 将描述子整平坐标的初值转换到原始源与目标坐标。
Eigen::Matrix4d Search::descriptorSeedToRawSourceToTarget(
    const Eigen::Matrix4d& descriptor_seed, const Eigen::Matrix4d& T_map_prior,
    const bool level_with_odom_roll_pitch) const {
    return descriptorYawToRawSourceTarget(
        yawFromMatrix(descriptor_seed), descriptor_seed.block<3, 1>(0, 3),
        request_.runtime_kf.odom_pose, T_map_prior, level_with_odom_roll_pitch);
}

/// 根据描述子航向初值及偏移生成单个几何验证假设。
VerificationEntry Search::makeVerificationEntry(const CandidateVerificationInput& input,
                                                const int yaw_offset, const double yaw_step_rad,
                                                const double yaw_step_deg) const {
    VerificationEntry entry;
    entry.T_map_prior = input.T_map_prior;
    entry.label =
        yawSampleLabel(input.hit.yaw_seed_rad.has_value() ? "descriptor_yaw" : "yaw_sweep",
                       yaw_offset, yaw_step_deg);
    const double sample_yaw =
        normalizeAngle(input.hit.yaw_seed_rad.value_or(0.0) + yaw_offset * yaw_step_rad);
    const Eigen::Matrix4d raw_seed =
        descriptorSeedToRawSourceToTarget(yawGuess(sample_yaw, Eigen::Vector3d::Zero()),
                                          input.T_map_prior, descriptor_.levelWithOdomRollPitch());
    entry.source_to_target_seed = raw_seed;
    entry.yaw_seed = yawFromMatrix(raw_seed);
    return entry;
}

/// 由物理位置先验及航向采样构造原始传感器位姿假设。
VerificationEntry Search::makePositionVerificationEntry(const CandidateVerificationInput& input,
                                                        const int yaw_offset,
                                                        const double yaw_step_rad,
                                                        const double yaw_step_deg) const {
    const RelocalizationPositionPrior& prior = *request_.position_prior;
    const double sample_yaw =
        normalizeAngle(prior.yaw_seed_rad.value_or(0.0) + yaw_offset * yaw_step_rad);
    const Eigen::Matrix3d R_map_sensor =
        yawGuess(sample_yaw, Eigen::Vector3d::Zero()).block<3, 3>(0, 0) * prior.R_yawless_sensor;

    Eigen::Matrix4d T_map_runtime = Eigen::Matrix4d::Identity();
    T_map_runtime.block<3, 3>(0, 0) = R_map_sensor;
    Eigen::Vector3d translation = prior.map_point - R_map_sensor * prior.runtime_sensor_point;
    if (!prior.use_z) {
        // UWB 只提供水平位置，竖直方向沿前端初值交给 scan-to-map 求解，
        // 不注入 UWB 高度。
        translation.z() = request_.T_map_runtime_pred(2, 3);
    }
    T_map_runtime.block<3, 1>(0, 3) = translation;

    VerificationEntry entry;
    entry.T_map_prior = input.T_map_prior;
    entry.label = yawSampleLabel(
        prior.yaw_seed_rad.has_value() ? prior.source + "_yaw" : prior.source + "_yaw_sweep",
        yaw_offset, yaw_step_deg);
    entry.source_to_target_seed = input.T_map_prior.inverse() * T_map_runtime;
    entry.yaw_seed = sample_yaw;
    return entry;
}

/// 执行单次配准并验收完整位姿可观性和位置先验一致性。
VerificationRunResult Search::evaluatePreparedEntry(const VerificationEntry& entry,
                                                    Registration& corrector) const {
    VerificationRunResult out;
    if (!source_ds_ || source_ds_->empty()) {
        return out;
    }

    ++out.tried_alignments;
    const RegistrationResult result =
        corrector.align(*source_ds_, entry.source_to_target_seed, request_.source_information_scale,
                        request_.coarse_source);
    const ObservabilityQuality& quality = result.quality;
    out.best_observed_residual_score = quality.rmse;
    out.best_observed_quality = quality;
    if (!result.anchorValid()) {
        return out;
    }

    if (request_.position_prior != nullptr) {
        const RelocalizationPositionPrior& prior = *request_.position_prior;
        const Eigen::Matrix4d T_map_runtime = entry.T_map_prior * result.source_to_target;
        const Eigen::Vector3d corrected_map_point =
            T_map_runtime.block<3, 3>(0, 0) * prior.runtime_sensor_point +
            T_map_runtime.block<3, 1>(0, 3);
        const Eigen::Vector3d delta = corrected_map_point - prior.map_point;
        const double distance = prior.use_z ? delta.norm() : delta.head<2>().norm();
        if (distance > prior.search_radius) {
            return out;
        }
    }

    out.success = true;
    out.measured = result.source_to_target;
    out.residual_score = quality.rmse;
    out.covariance = result.covariance;
    out.covariance_valid = result.covariance_valid;
    out.quality = quality;
    return out;
}

/// 验收描述子候选并准备其先验位姿和目标点云租约。
bool Search::buildCandidateInput(const LoopCandidate& hit, CandidateVerificationInput* out) {
    if (out == nullptr) {
        return false;
    }
    if (!descriptorGatePassed(hit)) {
        return false;
    }
    Keyframe prior_kf;
    if (!keyframes_.get(hit.index, &prior_kf)) {
        return false;
    }
    const RelocalizationPreparedTarget direct_target = directTargetFor(hit.index);
    if (!direct_target.ready()) {
        BACKEND_DEBUG_STREAM("tryRelocalization: candidate KF "
                             << hit.index << " skipped: empty verification target");
        return false;
    }

    out->source = descriptor_.name();
    out->hit = hit;
    out->T_map_prior = prior_kf.optimized_pose;
    out->entries.clear();
    out->target = direct_target.target;
    return true;
}

/// 按位置召回关键帧准备几何验证输入及航向来源。
bool Search::buildPositionCandidateInput(const int prior_index, CandidateVerificationInput* out) {
    if (out == nullptr || request_.position_prior == nullptr) {
        return false;
    }
    Keyframe prior_kf;
    if (!keyframes_.get(prior_index, &prior_kf)) {
        return false;
    }
    const RelocalizationPreparedTarget direct_target = directTargetFor(prior_index);
    if (!direct_target.ready()) {
        return false;
    }

    out->source = request_.position_prior->source;
    out->hit.index = prior_index;
    out->hit.yaw_seed_rad = request_.position_prior->yaw_seed_rad;
    out->T_map_prior = prior_kf.optimized_pose;
    out->entries.clear();
    out->target = direct_target.target;
    return true;
}

/// 按指定偏航偏移序列展开描述子候选的验证假设。
std::vector<CandidateVerificationInput> Search::buildDescriptorBatchInputs(
    const std::vector<CandidateVerificationInput>& base_inputs,
    const std::vector<int>& offsets) const {
    std::vector<CandidateVerificationInput> out;
    out.reserve(base_inputs.size());
    const int yaw_samples = std::max(1, config_.relocalization_yaw_sweep_count);
    const double yaw_step_rad = 2.0 * M_PI / yaw_samples;
    const double yaw_step_deg = 360.0 / yaw_samples;
    for (const auto& base : base_inputs) {
        CandidateVerificationInput input = base;
        input.entries.clear();
        input.entries.reserve(offsets.size());
        for (const int offset : offsets) {
            input.entries.push_back(
                makeVerificationEntry(base, offset, yaw_step_rad, yaw_step_deg));
        }
        if (!input.entries.empty()) {
            out.push_back(std::move(input));
        }
    }
    return out;
}

/// 按指定偏航偏移序列展开位置先验候选的验证假设。
std::vector<CandidateVerificationInput> Search::buildPositionBatchInputs(
    const std::vector<CandidateVerificationInput>& base_inputs,
    const std::vector<int>& offsets) const {
    std::vector<CandidateVerificationInput> out;
    const int yaw_samples = std::max(1, config_.relocalization_yaw_sweep_count);
    const double yaw_step_rad = 2.0 * M_PI / yaw_samples;
    const double yaw_step_deg = 360.0 / yaw_samples;
    out.reserve(base_inputs.size());
    for (const auto& base : base_inputs) {
        CandidateVerificationInput input = base;
        input.entries.clear();
        input.entries.reserve(offsets.size());
        for (const int offset : offsets) {
            input.entries.push_back(
                makePositionVerificationEntry(base, offset, yaw_step_rad, yaw_step_deg));
        }
        if (!input.entries.empty()) {
            out.push_back(std::move(input));
        }
    }
    return out;
}

/// 在单个候选目标上顺序验证航向假设并保留最佳结果。
CandidateVerificationResult Search::runCandidateVerification(
    const CandidateVerificationInput& input) const {
    CandidateVerificationResult out;
    if (!input.target || input.target->empty() || input.entries.empty()) {
        return out;
    }

    Registration corrector(config_.point_to_plane_config, &target_cache_.preparedTargets());
    if (!corrector.setTarget(input.target, input.hit.index)) {
        return out;
    }

    for (const auto& entry : input.entries) {
        const VerificationRunResult result = evaluatePreparedEntry(entry, corrector);
        out.tried += result.tried_alignments;
        if (result.best_observed_residual_score < out.best_failed_residual_score) {
            out.best_failed_residual_score = result.best_observed_residual_score;
            out.best_failed_quality = result.best_observed_quality;
        }
        if (!result.success) {
            continue;
        }
        recordIfBest(out.best, input, entry, result);
    }

    return out;
}

/// 将配置工作数限制在任务数和可用硬件并发数之内。
std::size_t Search::parallelWorkerCount(const std::size_t task_count) const {
    const unsigned int hw_threads = std::max(1u, std::thread::hardware_concurrency());
    const std::size_t configured =
        config_.relocalization_parallel_workers > 0
            ? static_cast<std::size_t>(config_.relocalization_parallel_workers)
            : static_cast<std::size_t>(hw_threads);
    return std::max<std::size_t>(
        1, std::min(task_count, std::min(configured, static_cast<std::size_t>(hw_threads))));
}

/// 并行验证候选并按原始序号归约诊断及最佳结果。
bool Search::runCandidateSet(const std::vector<CandidateVerificationInput>& inputs) {
    bool batch_found = false;
    if (inputs.empty()) {
        return batch_found;
    }

    std::vector<CandidateVerificationResult> results(inputs.size());
    // 结果按召回序号保存，后续归约顺序不受工作线程调度影响。
    detail::parallelFor(inputs.size(), parallelWorkerCount(inputs.size()), [&](std::size_t index) {
        results[index] = runCandidateVerification(inputs[index]);
    });

    for (const auto& result : results) {
        tried_verifications_ += result.tried;
        if (result.best_failed_residual_score < best_failed_residual_score_) {
            best_failed_residual_score_ = result.best_failed_residual_score;
            best_failed_quality_ = result.best_failed_quality;
        }
        if (!result.best.found) {
            continue;
        }
        batch_found = true;
        if (!best_.found ||
            betterScanToMapRegistration(result.best.residual_score, result.best.quality,
                                        best_.residual_score, best_.quality)) {
            best_ = result.best;
        }
    }
    return batch_found;
}

}  // namespace backend::relocalization_detail
