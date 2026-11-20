/**
 * @file loop.cpp
 * @brief 检索与验证历史回环，并提交通过联合创新检验的约束。
 */

#include "backend/loop.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <thread>
#include <utility>

#include "backend/frame_history.hpp"
#include "backend/geometry.hpp"
#include "backend/localization.hpp"
#include "backend/logger.hpp"
#include "backend/sampling.hpp"
#include "parallel.hpp"

namespace backend {

/// 绑定回环几何、配准及一致性验收参数。
LoopMatcher::LoopMatcher(const LoopMatcherConfig& config)
    : config_(config), verifier_(config.loop.verification) {
}

/// 在按索引排列的冻结轨迹中查询指定关键帧。
bool LoopMatcher::getSnapshotKeyframe(const std::vector<Keyframe>& keyframes, const int index,
                                      Keyframe* out) {
    if (out == nullptr || index < 0) {
        return false;
    }
    const auto it = std::lower_bound(keyframes.begin(), keyframes.end(), index,
                                     [](const Keyframe& keyframe, const int requested_index) {
                                         return keyframe.index < requested_index;
                                     });
    if (it == keyframes.end() || it->index != index) {
        return false;
    }
    *out = *it;
    return true;
}

/// 在同代次历史前缀中选择空间邻居并组装中心坐标系子图。
Cloud::Ptr LoopMatcher::buildSpatialSubmapFromSnapshot(const std::vector<Keyframe>& keyframes,
                                                       const int center_index,
                                                       const std::size_t max_keyframes,
                                                       const double max_radius,
                                                       const int max_inclusive_index) {
    Keyframe center;
    if (center_index > max_inclusive_index ||
        !getSnapshotKeyframe(keyframes, center_index, &center)) {
        return Cloud::Ptr(new Cloud);
    }

    struct Neighbor {
        double squared_distance = 0.0;
        std::size_t offset = 0;
        int index = -1;
    };
    std::vector<Neighbor> neighbors;
    const double radius_squared =
        max_radius > 0.0 ? max_radius * max_radius : std::numeric_limits<double>::infinity();
    const Eigen::Vector3d center_position = center.optimized_pose.block<3, 1>(0, 3);
    for (std::size_t offset = 0; offset < keyframes.size(); ++offset) {
        const Keyframe& keyframe = keyframes[offset];
        if (keyframe.index > max_inclusive_index) {
            break;
        }
        if (keyframe.epoch != center.epoch) {
            continue;
        }
        const double distance =
            (keyframe.optimized_pose.block<3, 1>(0, 3) - center_position).squaredNorm();
        if (std::isfinite(distance) && distance <= radius_squared) {
            neighbors.push_back({distance, offset, keyframe.index});
        }
    }
    // 按空间距离和关键帧索引确定邻居的稳定顺序。
    std::sort(neighbors.begin(), neighbors.end(), [](const Neighbor& lhs, const Neighbor& rhs) {
        if (lhs.squared_distance != rhs.squared_distance) {
            return lhs.squared_distance < rhs.squared_distance;
        }
        return lhs.index < rhs.index;
    });
    if (neighbors.size() > std::max<std::size_t>(1, max_keyframes)) {
        neighbors.resize(std::max<std::size_t>(1, max_keyframes));
    }

    const Eigen::Matrix4d world_to_center = center.optimized_pose.inverse();
    Cloud::Ptr submap(new Cloud);
    for (const Neighbor& neighbor : neighbors) {
        const Keyframe& keyframe = keyframes[neighbor.offset];
        const auto geometry_cloud = keyframe.cloud.load();
        if (!geometry_cloud || geometry_cloud->empty()) {
            continue;
        }
        *submap += *transformCloud(*geometry_cloud, world_to_center * keyframe.optimized_pose);
    }
    return submap;
}

/// 累计历史因果源并按回环分辨率降采样。
Cloud::Ptr LoopMatcher::prepareSource(const LoopWork& work, int& source_keyframes) const {
    auto source = accumulateSource(work.latest, work.trajectory, config_.loop.source);
    source_keyframes = source.frame_count;
    const Cloud::Ptr source_raw = std::move(source.cloud);
    if (source_keyframes < config_.loop.source.min_frames) {
        return Cloud::Ptr(new Cloud);
    }
    return source_raw ? voxelize(*source_raw, config_.registration.voxel_resolution)
                      : Cloud::Ptr(new Cloud);
}

/// 对单个回环候选执行配准、几何和历史一致性验证。
std::optional<VerifiedLoop> LoopMatcher::verifyCandidate(const LoopWork& work,
                                                         const LoopCandidate& candidate,
                                                         const Cloud::Ptr& source,
                                                         const int source_keyframes) const {
    Keyframe target_keyframe;
    if (!getSnapshotKeyframe(work.trajectory, candidate.index, &target_keyframe)) {
        return std::nullopt;
    }
    const Cloud::Ptr target_raw = buildSpatialSubmapFromSnapshot(
        work.trajectory, candidate.index, config_.loop.target_keyframes, config_.loop.target_radius,
        candidate.index);
    const Cloud::Ptr target = target_raw
                                  ? voxelize(*target_raw, config_.registration.voxel_resolution)
                                  : Cloud::Ptr(new Cloud);
    if (!target || target->empty()) {
        return std::nullopt;
    }

    Registration matcher(config_.registration);
    if (!matcher.setTarget(target, candidate.index)) {
        return std::nullopt;
    }
    const Eigen::Matrix4d predicted =
        target_keyframe.optimized_pose.inverse() * work.latest.optimized_pose;
    Eigen::Matrix4d initial = predicted;
    if (candidate.yaw_seed_rad.has_value()) {
        initial = descriptorYawToRawSourceTarget(
            *candidate.yaw_seed_rad, predicted.block<3, 1>(0, 3), work.latest.odom_pose,
            target_keyframe.odom_pose, work.descriptor_level_with_odom_roll_pitch);
    }
    const RegistrationResult result =
        matcher.align(*source, initial, 1.0 / static_cast<double>(std::max(1, source_keyframes)));
    if (!result.geometryVerified()) {
        return std::nullopt;
    }

    VerifiedLoop outcome;
    outcome.candidate = candidate;
    outcome.target_keyframe = target_keyframe;
    outcome.registration.valid = true;
    outcome.registration.converged = true;
    outcome.registration.rmse = result.quality.rmse;
    outcome.aligned_source = transformCloud(*source, result.source_to_target);
    outcome.registration.overlap_ratio =
        matcher.overlap(*outcome.aligned_source, config_.loop.verification.overlap_distance);
    outcome.target_raw = target_raw;
    outcome.measured_between = result.source_to_target.inverse();
    // M=target<-source 的左误差取逆后成为右误差；同时取反残差行和 Jacobian
    // 不改变似然。
    outcome.sqrt_information = -result.measurement_information.sqrt_information;

    std::string reject_reason;
    if (!verifier_.validateMeasurement(work.latest, target_keyframe, candidate,
                                       outcome.measured_between, outcome.registration,
                                       &reject_reason)) {
        return std::nullopt;
    }
    if (!verifier_.passesAcceptedConsistency(
            work.latest, target_keyframe, outcome.measured_between, work.accepted_loops,
            [&](const int index, Keyframe* keyframe) {
                return getSnapshotKeyframe(work.trajectory, index, keyframe);
            })) {
        return std::nullopt;
    }
    return outcome;
}

/// 对候选一致性团按配准质量、描述子距离和索引确定性择优。
std::optional<VerifiedLoop> LoopMatcher::selectBest(std::vector<VerifiedLoop> valid) const {
    if (valid.empty()) {
        return std::nullopt;
    }
    std::vector<int> selected = verifier_.selectConsistentCandidateClique(valid);
    if (selected.empty() || static_cast<int>(selected.size()) <
                                std::max(1, config_.loop.verification.min_candidate_clique_size)) {
        return std::nullopt;
    }
    // 共享同一源的候选按配准质量、描述子距离和索引择优。
    std::sort(selected.begin(), selected.end(), [&](const int lhs, const int rhs) {
        const VerifiedLoop& left = valid[static_cast<std::size_t>(lhs)];
        const VerifiedLoop& right = valid[static_cast<std::size_t>(rhs)];
        if (left.registration.rmse != right.registration.rmse) {
            return left.registration.rmse < right.registration.rmse;
        }
        if (left.candidate.descriptor_distance != right.candidate.descriptor_distance) {
            return left.candidate.descriptor_distance < right.candidate.descriptor_distance;
        }
        return left.candidate.index < right.candidate.index;
    });
    // clique 可由多个候选建立，但它们共享 source；只提交最佳成员避免重复计数。
    return std::move(valid[static_cast<std::size_t>(selected.front())]);
}

/// 并行验证候选后按一致性团与同源信息预算返回最佳回环。
LoopVerificationBatch LoopMatcher::compute(const LoopWork& work) const {
    LoopVerificationBatch batch;
    if (config_.loop.descriptor_ratio_threshold > 0.0 &&
        config_.loop.descriptor_ratio_threshold < 1.0 && work.candidates.size() > 1U) {
        const double ratio = work.candidates[0].descriptor_distance /
                             std::max(1.0e-9, work.candidates[1].descriptor_distance);
        if (ratio > config_.loop.descriptor_ratio_threshold) {
            return batch;
        }
    }
    const Cloud::Ptr source = prepareSource(work, batch.source_keyframes);
    if (!source || source->empty()) {
        return batch;
    }

    const unsigned int hardware = std::max(1U, std::thread::hardware_concurrency());
    const std::size_t workers = std::max<std::size_t>(
        1, std::min(
               work.candidates.size(),
               std::min<std::size_t>(hardware, static_cast<std::size_t>(std::max(
                                                   1, config_.loop.max_parallel_verifications)))));
    std::vector<std::optional<VerifiedLoop>> outcomes(work.candidates.size());
    // 固定输出序号使并行完成次序不影响候选择优。
    detail::parallelFor(work.candidates.size(), workers, [&](std::size_t order) {
        outcomes[order] =
            verifyCandidate(work, work.candidates[order], source, batch.source_keyframes);
    });

    std::vector<VerifiedLoop> valid;
    for (auto& outcome : outcomes) {
        if (outcome.has_value()) {
            valid.push_back(std::move(*outcome));
        }
    }
    batch.selected = selectBest(std::move(valid));
    return batch;
}

/// 按冷却与缓存身份捕获回环候选及完整轨迹快照。
std::optional<LoopWork> Localization::captureLoop() {
    std::lock_guard<std::mutex> lock(mutex_);
    LoopWork work;

    if (!LocalizationStateMachine::poseTrusted(state_.status()) || trajectory_.empty()) {
        return std::nullopt;
    }

    work.latest = trajectory_.latest();
    work.state_generation = state_.generation();
    if (work.latest.index == last_loop_checked_index_) {
        return std::nullopt;
    }
    if (last_accepted_loop_index_ >= 0 &&
        work.latest.index - last_accepted_loop_index_ < config_.loop.cooldown_keyframes) {
        return std::nullopt;
    }
    const auto trajectory_revision = trajectory_.revision();
    const auto descriptor_revision = trajectory_descriptor_.revision();
    if (descriptor_revision != 0 && empty_loop_ && work.latest.index == empty_loop_->index &&
        trajectory_revision == empty_loop_->trajectory_revision &&
        descriptor_revision == empty_loop_->descriptor_revision)
        return std::nullopt;
    work.trajectory = trajectory_.snapshot();

    if (config_.loop.candidate_max_distance > 0.0) {
        std::vector<int> allowed;
        for (const Keyframe& candidate : work.trajectory) {
            if (candidate.index >= work.latest.index) {
                continue;
            }
            const double distance = (candidate.optimized_pose.block<3, 1>(0, 3) -
                                     work.latest.optimized_pose.block<3, 1>(0, 3))
                                        .norm();
            if (distance <= config_.loop.candidate_max_distance) {
                allowed.push_back(candidate.index);
            }
        }
        work.candidates = trajectory_descriptor_.detectLatestCandidatesWithin(allowed);
    } else {
        work.candidates = trajectory_descriptor_.detectLatestCandidates();
    }
    // 描述子仍会由同步帧线程增长；同锁冻结其姿态语义，工作线程不再访问可变对象。
    work.descriptor_level_with_odom_roll_pitch = trajectory_descriptor_.levelWithOdomRollPitch();
    work.accepted_loops = accepted_loops_;
    // 只有真实候选批次才消费该关键帧；冷却或无候选不会造成永久跳过。
    if (!work.candidates.empty()) {
        last_loop_checked_index_ = work.latest.index;
    }
    if (work.candidates.empty()) {
        empty_loop_ =
            EmptyLoopIdentity{work.latest.index, trajectory_revision, descriptor_revision};
        return std::nullopt;
    }
    return work;
}

/// 在当前图预测处计算回环创新，并按联合概率阈值验收该约束。
bool Localization::assessLoopLocked(const LoopWork& work, const VerifiedLoop& verified,
                                    LoopCommitSummary& summary) const {
    InnovationTerm term =
        graph_.linearizeRelativePose(work.latest.index, verified.target_keyframe.index,
                                     verified.measured_between, verified.sqrt_information);
    if (!term.valid) {
        BACKEND_WARN_STREAM("loop innovation unavailable: latest="
                            << work.latest.index << ", target=" << verified.target_keyframe.index
                            << ", reason=" << term.invalid_reason);
        return false;
    }
    const InnovationBatchAssessment batch = graph_.assessInnovationBatch({term});
    if (!batch.joint.valid || batch.individual.size() != 1U) {
        BACKEND_WARN_STREAM("joint loop innovation unavailable: latest="
                            << work.latest.index << ", reason="
                            << (batch.joint.invalid_reason.empty()
                                    ? "incomplete per-loop diagnostics"
                                    : batch.joint.invalid_reason));
        return false;
    }
    summary.innovation = batch.individual.front();
    summary.joint_innovation = batch.joint;
    if (batch.joint.upper_tail_probability < config_.innovation.rejection_probability) {
        BACKEND_WARN_STREAM("loop rejected by joint pre-update NIS: latest="
                            << work.latest.index << ", target=" << verified.target_keyframe.index
                            << ", individual_NIS=" << summary.innovation.nis << "/"
                            << summary.innovation.degrees_of_freedom
                            << ", individual_p=" << summary.innovation.upper_tail_probability
                            << ", joint_NIS=" << batch.joint.nis << "/"
                            << batch.joint.degrees_of_freedom
                            << ", joint_p=" << batch.joint.upper_tail_probability);
        return false;
    }

    return true;
}

/// 将通过联合创新检验的回环写入图及可视化历史。
LoopCommitSummary Localization::commit(const LoopWork& work,
                                       const LoopVerificationBatch& verification) {
    LoopCommitSummary summary;
    if (!verification.selected) {
        return summary;
    }
    const VerifiedLoop& verified = *verification.selected;
    VisualizationSnapshot visualization;
    if (!withOutputCommit([&] {
            if (!LocalizationStateMachine::poseTrusted(state_.status()) ||
                state_.generation() != work.state_generation ||
                !graph_.hasEstimate(work.latest.index)) {
                return false;
            }

            if (!assessLoopLocked(work, verified, summary))
                return false;

            graph_.addLoopInformation(work.latest.index, verified.target_keyframe.index,
                                      verified.measured_between, verified.sqrt_information);
            accepted_loops_.push_back(
                {work.latest.index, verified.target_keyframe.index, verified.measured_between});
            loop_edges_.push_back({work.latest.index, verified.target_keyframe.index});
            if (config_.loop.history_size > 0) {
                const std::size_t history_size =
                    static_cast<std::size_t>(config_.loop.history_size);
                if (accepted_loops_.size() > history_size) {
                    accepted_loops_.erase(
                        accepted_loops_.begin(),
                        accepted_loops_.end() - static_cast<std::ptrdiff_t>(history_size));
                }
                if (loop_edges_.size() > history_size) {
                    loop_edges_.erase(
                        loop_edges_.begin(),
                        loop_edges_.end() - static_cast<std::ptrdiff_t>(history_size));
                }
            }
            summary.applied_correction = updateGraphLocked(/*global_factor_added=*/true);
            last_accepted_loop_index_ = work.latest.index;
            visualization.loop_edges = loop_edges_;
            if (config_.registration_debug_enabled) {
                visualization.debug_aligned_source = transformCloud(
                    *verified.aligned_source, verified.target_keyframe.optimized_pose);
                visualization.debug_target =
                    transformCloud(*verified.target_raw, verified.target_keyframe.optimized_pose);
                visualization.include_debug_clouds = true;
            }
            summary.accepted = true;
            summary.target_index = verified.target_keyframe.index;
            return true;
        }))
        return summary;
    visualization_sink_.publishVisualization(visualization);
    return summary;
}
/// 报告已接受回环的创新统计和全局校正幅度。
void reportLoop(const LoopWork& work, const LoopVerificationBatch& verification,
                const LoopCommitSummary& commit) {
    if (!commit.accepted) {
        return;
    }
    BACKEND_INFO_STREAM(
        "loop scan-to-map accepted: latest="
        << work.latest.index << ", target=" << commit.target_index
        << ", source_keyframes=" << verification.source_keyframes << ", factors=1"
        << ", nis=" << commit.innovation.nis << "/" << commit.innovation.degrees_of_freedom
        << ", p=" << commit.innovation.upper_tail_probability << ", joint_nis="
        << commit.joint_innovation.nis << "/" << commit.joint_innovation.degrees_of_freedom
        << ", joint_p=" << commit.joint_innovation.upper_tail_probability
        << ", delta_t=" << commit.applied_correction.translation
        << "m, delta_r_deg=" << commit.applied_correction.rotation_rad * 180.0 / M_PI);
}

}  // namespace backend
