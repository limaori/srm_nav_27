/**
 * @file loop_consistency.cpp
 * @brief 回环闭合验证、PCM 一致性和候选选择实现。
 */

#include "backend/loop_consistency.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <sstream>

#include "backend/geometry.hpp"
#include "backend/logger.hpp"
#include "backend/math.hpp"

namespace backend {

/// 构造回环几何和成对一致性验证器。
LoopClosureVerifier::LoopClosureVerifier(const LoopClosureConfig& config) : config_(config) {
}

/// 更新回环几何门限和成对一致性配置。
void LoopClosureVerifier::reconfigure(const LoopClosureConfig& config) {
    config_ = config;
}

/// 在同代里程计或跨代地图坐标下验证回环几何。
bool LoopClosureVerifier::validateWorldGeometry(const Keyframe& latest, const Keyframe& candidate,
                                                const LoopCandidate& loop_candidate,
                                                const Eigen::Matrix4d& measured_between,
                                                std::string* reject_reason) const {
    // 同 epoch 校验使用原始重力对齐里程计，避免已接纳回环借优化结果自证。
    // reset 两侧的原始里程计规范无关，此时唯一公共坐标是图中的 map 估计。
    const bool same_epoch = latest.epoch == candidate.epoch;
    const Eigen::Matrix4d& latest_world_pose =
        same_epoch ? latest.odom_pose : latest.optimized_pose;
    const Eigen::Matrix4d& candidate_world_pose =
        same_epoch ? candidate.odom_pose : candidate.optimized_pose;
    const Eigen::Matrix4d predicted = latest_world_pose * measured_between;

    const double world_z_diff = predicted(2, 3) - candidate_world_pose(2, 3);
    if (config_.z_max > 0.0 && std::abs(world_z_diff) > config_.z_max) {
        if (reject_reason != nullptr) {
            std::ostringstream oss;
            oss << "world z diff " << std::abs(world_z_diff) << " > " << config_.z_max;
            *reject_reason = oss.str();
        }
        BACKEND_INFO_STREAM("Loop rejected by world z diff: " << std::abs(world_z_diff) << " > "
                                                              << config_.z_max);
        return false;
    }

    const Eigen::Matrix3d rotation_residual =
        predicted.block<3, 3>(0, 0) * candidate_world_pose.block<3, 3>(0, 0).transpose();
    const double world_roll_pitch = maxRollPitchFromRotation(rotation_residual);
    if (config_.roll_pitch_max_rad > 0.0 && world_roll_pitch > config_.roll_pitch_max_rad) {
        if (reject_reason != nullptr) {
            std::ostringstream oss;
            oss << "world roll/pitch residual " << world_roll_pitch * 180.0 / kPi << " deg > "
                << config_.roll_pitch_max_rad * 180.0 / kPi << " deg";
            *reject_reason = oss.str();
        }
        BACKEND_INFO_STREAM("Loop rejected by world roll/pitch residual: "
                            << world_roll_pitch * 180.0 / kPi << " deg > "
                            << config_.roll_pitch_max_rad * 180.0 / kPi << " deg");
        return false;
    }

    return validateDescriptorYaw(latest_world_pose, predicted, loop_candidate, reject_reason);
}

/// 在公共世界系中比较几何源到目标偏航与可信描述子偏航。
bool LoopClosureVerifier::validateDescriptorYaw(const Eigen::Matrix4d& latest_world_pose,
                                                const Eigen::Matrix4d& predicted,
                                                const LoopCandidate& loop_candidate,
                                                std::string* reject_reason) const {
    if (config_.yaw_residual_max_rad <= 0.0 || !loop_candidate.yaw_seed_rad.has_value()) {
        return true;
    }
    const double latest_world_yaw = yawFromRotation(latest_world_pose.block<3, 3>(0, 0));
    const double predicted_candidate_world_yaw = yawFromRotation(predicted.block<3, 3>(0, 0));
    // 描述子变换约定为 source(最新点云)->target(候选点云)，与候选相对
    // 最新帧的位姿方向相反。
    const double source_to_target_world_yaw =
        normalizeAngle(latest_world_yaw - predicted_candidate_world_yaw);
    const double descriptor_yaw = *loop_candidate.yaw_seed_rad;
    const double yaw_residual =
        std::abs(normalizeAngle(source_to_target_world_yaw - descriptor_yaw));
    if (yaw_residual <= config_.yaw_residual_max_rad) {
        return true;
    }
    if (reject_reason != nullptr) {
        std::ostringstream oss;
        oss << "descriptor yaw residual " << yaw_residual * 180.0 / kPi << " deg > "
            << config_.yaw_residual_max_rad * 180.0 / kPi
            << " deg (source_to_target_world_yaw=" << source_to_target_world_yaw * 180.0 / kPi
            << " deg, descriptor_yaw=" << descriptor_yaw * 180.0 / kPi << " deg)";
        *reject_reason = oss.str();
    }
    BACKEND_INFO_STREAM(
        "Loop rejected by descriptor yaw residual: "
        << yaw_residual * 180.0 / kPi << " deg > " << config_.yaw_residual_max_rad * 180.0 / kPi
        << " deg (source_to_target_world_yaw=" << source_to_target_world_yaw * 180.0 / kPi
        << " deg, descriptor_yaw=" << descriptor_yaw * 180.0 / kPi << " deg)");
    return false;
}

/// 验证回环配准质量、位移、姿态和描述子航向一致性。
bool LoopClosureVerifier::validateMeasurement(const Keyframe& latest, const Keyframe& candidate,
                                              const LoopCandidate& loop_candidate,
                                              const Eigen::Matrix4d& measured_between,
                                              const LoopRegistrationQuality& registration,
                                              std::string* reject_reason) const {
    if (!registration.converged || !registration.valid) {
        if (reject_reason != nullptr) {
            std::ostringstream oss;
            oss << "registration invalid: converged=" << std::boolalpha << registration.converged
                << ", rmse=" << registration.rmse;
            *reject_reason = oss.str();
        }
        return false;
    }

    const Eigen::Vector3d body_translation = measured_between.block<3, 1>(0, 3);
    const double translation_norm = body_translation.norm();
    if (config_.translation_max > 0.0 && translation_norm > config_.translation_max) {
        if (reject_reason != nullptr) {
            std::ostringstream oss;
            oss << "measured translation " << translation_norm << " > " << config_.translation_max;
            *reject_reason = oss.str();
        }
        BACKEND_INFO_STREAM("Loop rejected by measured translation: " << translation_norm << " > "
                                                                      << config_.translation_max);
        return false;
    }

    if (!validateWorldGeometry(latest, candidate, loop_candidate, measured_between,
                               reject_reason)) {
        return false;
    }

    if (config_.min_overlap_ratio > 0.0 && registration.overlap_ratio < config_.min_overlap_ratio) {
        if (reject_reason != nullptr) {
            std::ostringstream oss;
            oss << "overlap " << registration.overlap_ratio << " < " << config_.min_overlap_ratio;
            *reject_reason = oss.str();
        }
        BACKEND_INFO_STREAM("Loop rejected by overlap: " << registration.overlap_ratio << " < "
                                                         << config_.min_overlap_ratio);
        return false;
    }

    return true;
}

/// 历史支持只要求达到下限；后续不一致边不会撤销已满足的支持条件。
bool LoopClosureVerifier::passesAcceptedConsistency(const Keyframe& latest,
                                                    const Keyframe& candidate,
                                                    const Eigen::Matrix4d& measured_between,
                                                    const std::vector<AcceptedLoop>& accepted_loops,
                                                    const KeyframeLookup& keyframe_lookup) const {
    if (!config_.enable_pcm || !keyframe_lookup || config_.min_consistent_loops <= 0) {
        return true;
    }

    bool applicable = false;
    int consistent = 0;
    for (const auto& accepted : accepted_loops) {
        Keyframe old_from;
        Keyframe old_to;
        if (!keyframe_lookup(accepted.from, &old_from) || !keyframe_lookup(accepted.to, &old_to)) {
            continue;
        }
        // PCM 环中的每段非回环边必须处于同一原始里程计规范；reset
        // 会切断该关系，因而这类历史回环既不能支持也不能反对当前候选。
        if (latest.epoch != old_from.epoch || candidate.epoch != old_to.epoch) {
            continue;
        }
        applicable = true;

        // PCM 一致性链：用原始里程计计算非回环段（独立于 iSAM2 估计）
        const Eigen::Matrix4d latest_to_old_from = latest.odom_pose.inverse() * old_from.odom_pose;
        const Eigen::Matrix4d old_to_to_candidate =
            old_to.odom_pose.inverse() * candidate.odom_pose;
        const Eigen::Matrix4d expected =
            latest_to_old_from * accepted.measured_between * old_to_to_candidate;
        const Eigen::Matrix4d residual = expected.inverse() * measured_between;
        const double translation_error = residual.block<3, 1>(0, 3).norm();
        const double rotation_error = rotationAngleFromMatrix(residual);
        if (translation_error <= config_.pcm_translation_threshold &&
            rotation_error <= config_.pcm_rotation_threshold_rad) {
            if (++consistent >= config_.min_consistent_loops)
                return true;
        }
    }

    // 没有可比较历史时允许启动；有历史但支持不足时拒绝。
    return !applicable;
}

/// 检查同一源帧的两个候选是否满足成对位姿一致性。
bool LoopClosureVerifier::areSameSourceLoopsConsistent(const VerifiedLoop& lhs,
                                                       const VerifiedLoop& rhs) const {
    if (!config_.enable_pcm) {
        return true;
    }
    if (lhs.target_keyframe.epoch != rhs.target_keyframe.epoch) {
        // 非回环路径由同 epoch 原始里程计计算；同源一致团中的目标须属于同一代次。
        return false;
    }

    // 两条同源回环推出的相对位姿，应与目标关键帧间的同代原始里程计一致。
    const Eigen::Matrix4d measured_lhs_to_rhs =
        lhs.measured_between.inverse() * rhs.measured_between;
    const Eigen::Matrix4d odom_lhs_to_rhs =
        lhs.target_keyframe.odom_pose.inverse() * rhs.target_keyframe.odom_pose;
    const Eigen::Matrix4d residual = odom_lhs_to_rhs.inverse() * measured_lhs_to_rhs;
    const double translation_error = residual.block<3, 1>(0, 3).norm();
    const double rotation_error = rotationAngleFromMatrix(residual);
    return translation_error <= config_.pcm_translation_threshold &&
           rotation_error <= config_.pcm_rotation_threshold_rad;
}

/// 在候选一致性图上确定性选择最大一致团。
std::vector<int> LoopClosureVerifier::selectConsistentCandidateClique(
    const std::vector<VerifiedLoop>& candidates) const {
    const int count = static_cast<int>(candidates.size());
    if (!config_.enable_pcm || count <= 1) {
        std::vector<int> all;
        all.reserve(count);
        for (int i = 0; i < count; ++i) {
            all.push_back(i);
        }
        return all;
    }

    constexpr int kMaxExhaustiveCliqueCandidates = 20;
    if (count > kMaxExhaustiveCliqueCandidates) {
        BACKEND_WARN_STREAM("Candidate PCM clique limited from "
                            << count << " to " << kMaxExhaustiveCliqueCandidates << " candidates");
    }
    const int search_count = std::min(count, kMaxExhaustiveCliqueCandidates);

    std::vector<std::vector<bool>> consistent(search_count, std::vector<bool>(search_count, true));
    for (int i = 0; i < search_count; ++i) {
        for (int j = i + 1; j < search_count; ++j) {
            consistent[i][j] = areSameSourceLoopsConsistent(candidates[i], candidates[j]);
            consistent[j][i] = consistent[i][j];
        }
    }

    std::vector<int> best;
    double best_score_sum = std::numeric_limits<double>::max();
    double best_descriptor_distance_sum = std::numeric_limits<double>::max();
    /// 依次按团大小、配准残差和描述子距离更新最佳一致团。
    const auto consider = [&](const std::vector<int>& clique, const double score_sum,
                              const double descriptor_distance_sum) {
        if (clique.size() > best.size() ||
            (clique.size() == best.size() &&
             (score_sum < best_score_sum ||
              (score_sum == best_score_sum &&
               descriptor_distance_sum < best_descriptor_distance_sum)))) {
            best = clique;
            best_score_sum = score_sum;
            best_descriptor_distance_sum = descriptor_distance_sum;
        }
    };

    // 用分支限界精确求最大团；候选交集保持规模、RMSE、描述子距离的平局规则，
    // 并在分支不可能超过当前最优解时立即剪枝。
    std::vector<int> initial_candidates;
    initial_candidates.reserve(search_count);
    for (int index = 0; index < search_count; ++index) {
        initial_candidates.push_back(index);
    }
    std::vector<int> clique;
    std::function<void(std::vector<int>, double, double)> expand;
    /// 对剩余候选执行分支限界，并保持候选顺序与平局规则。
    expand = [&](std::vector<int> available, const double score_sum,
                 const double descriptor_distance_sum) {
        if (clique.size() + available.size() < best.size()) {
            return;
        }
        while (!available.empty()) {
            if (clique.size() + available.size() < best.size()) {
                return;
            }
            const int vertex = available.back();
            available.pop_back();
            clique.push_back(vertex);
            const double next_score = score_sum + candidates[vertex].registration.rmse;
            const double next_descriptor =
                descriptor_distance_sum + candidates[vertex].candidate.descriptor_distance;

            std::vector<int> next_available;
            next_available.reserve(available.size());
            for (const int candidate : available) {
                if (consistent[vertex][candidate]) {
                    next_available.push_back(candidate);
                }
            }
            consider(clique, next_score, next_descriptor);
            if (!next_available.empty()) {
                expand(std::move(next_available), next_score, next_descriptor);
            }
            clique.pop_back();
        }
    };
    expand(std::move(initial_candidates), 0.0, 0.0);

    BACKEND_INFO_STREAM("Candidate PCM clique selected: " << best.size() << " / " << count);
    return best;
}

}  // namespace backend
