/**
 * @file tracking_matcher.cpp
 * @brief 准备先验地图跟踪观测并报告提交结果。
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>
#include <utility>

#include "backend/frame_history.hpp"
#include "backend/geometry.hpp"
#include "backend/logger.hpp"
#include "backend/matching.hpp"
#include "backend/sampling.hpp"

namespace backend {
namespace {
/// 格式化点面配准的几何及信息质量摘要。
std::string qualitySummary(const ObservabilityQuality& quality) {
    std::ostringstream text;
    text << "rmse=" << quality.rmse << ", support=" << quality.correspondences << "/"
         << quality.source_points << ", ratio=" << quality.correspondence_ratio
         << ", sectors=" << quality.coverage_sectors << ", min_eig=" << quality.min_eigenvalue
         << ", rank=" << quality.observable_rank << "/6"
         << ", observable=" << (quality.observable ? "true" : "false")
         << ", schur_t=" << quality.translation_condition
         << ", schur_r=" << quality.rotation_condition << ", weak=";
    for (bool weak : quality.weak_directions)
        text << (weak ? '1' : '0');
    return text.str();
}
}  // namespace

/// 绑定只读先验地图、目标缓存及点面配准器。
TrackingMatcher::TrackingMatcher(TrackingMatcherConfig config, const KeyframeMap& prior,
                                 RelocalizationTargetCache& targets)
    : config_(std::move(config)),
      prior_map_(prior),
      target_cache_(targets),
      corrector_(config_.registration, &targets.preparedTargets()) {
}
/// 准备因果配准源、参考关键帧及只读目标几何。
void TrackingMatcher::prepare(GlobalObservationWork& work) const {
    if (work.prior_input.configured && prepareSource(work))
        selectReference(work);
}
/// 计算先验地图点面观测并按预测增量判定几何冲突。
PriorMapObservation TrackingMatcher::compute(GlobalObservationWork& work) {
    if (work.prior_input.configured && work.reference.index >= 0) {
        const auto target = target_cache_.getOrPrepare(
            work.reference.index, static_cast<int>(config_.tracking.target_keyframes),
            config_.registration.voxel_resolution, prior_map_, config_.tracking.target_radius);
        work.target = target.target;
        if (!target.ready()) {
            work.prior_input.reason = "empty spatial prior submap";
            work.prior_input.disposition = ObservationDisposition::ModelInvalid;
        }
    }

    PriorMapObservation observation;
    observation.evidence = work.prior_input;
    if (!observation.evidence.configured) {
        return observation;
    }
    if (!work.target || work.target->empty()) {
        observation.evidence.reason = work.prior_input.reason.empty()
                                          ? "prior scan-to-map input unavailable"
                                          : work.prior_input.reason;
        return observation;
    }

    const Cloud::Ptr source = voxelize(*work.source, config_.registration.voxel_resolution);
    if (source->empty() || !corrector_.setTarget(work.target, work.reference.index)) {
        observation.evidence.disposition = ObservationDisposition::ModelInvalid;
        observation.evidence.reason = "prior scan-to-map preparation failed";
        return observation;
    }

    const Eigen::Matrix4d initial_source_to_target =
        work.reference.optimized_pose.inverse() * work.predicted_map_pose;
    const auto start = std::chrono::steady_clock::now();
    observation.registration =
        corrector_.align(*source, initial_source_to_target,
                         1.0 / static_cast<double>(std::max(1, work.accumulated_frames)));
    observation.elapsed_ms = std::chrono::duration_cast<std::chrono::microseconds>(
                                 std::chrono::steady_clock::now() - start)
                                 .count() /
                             1000.0;

    if (config_.debug_enabled && observation.registration.source_to_target.allFinite()) {
        observation.debug_aligned_source = transformCloud(
            *source, work.reference.optimized_pose * observation.registration.source_to_target);
        observation.debug_target = transformCloud(*work.target, work.reference.optimized_pose);
    }

    classifyObservation(work, observation);
    return observation;
}
/// 根据似然有效性和物理校正幅度分类先验图观测，并保存地图系测量位姿。
void TrackingMatcher::classifyObservation(const GlobalObservationWork& work,
                                          PriorMapObservation& observation) const {
    if (!observation.registration.modelValid()) {
        std::ostringstream reason;
        if (observation.registration.likelihoodValid()) {
            reason << "prior scan-to-map residual exceeds local-model basin: "
                   << "rmse=" << observation.registration.quality.rmse << " > "
                   << observation.registration.model_validity_rmse_limit << ", ";
        } else {
            reason << "prior scan-to-map cannot construct a pose likelihood: ";
        }
        reason << qualitySummary(observation.registration.quality);
        const std::string& invalid_reason =
            observation.registration.measurement_information.invalid_reason;
        if (!invalid_reason.empty()) {
            reason << ", information=" << invalid_reason;
        }
        observation.evidence.disposition = ObservationDisposition::ModelInvalid;
        observation.evidence.reason = reason.str();
        return;
    }

    observation.measured_map_pose =
        work.reference.optimized_pose * observation.registration.source_to_target;
    const Eigen::Matrix4d correction =
        work.predicted_map_pose.inverse() * observation.measured_map_pose;
    observation.predicted_translation_delta = correction.block<3, 1>(0, 3).norm();
    observation.predicted_rotation_delta_rad = rotationAngleFromMatrix(correction);
    if ((config_.tracking.max_translation_correction > 0.0 &&
         observation.predicted_translation_delta > config_.tracking.max_translation_correction) ||
        (config_.tracking.max_rotation_correction_rad > 0.0 &&
         observation.predicted_rotation_delta_rad > config_.tracking.max_rotation_correction_rad)) {
        std::ostringstream reason;
        reason << "prior scan-to-map physical correction cap: delta_t="
               << observation.predicted_translation_delta
               << "m, delta_r_deg=" << observation.predicted_rotation_delta_rad * 180.0 / M_PI
               << ", " << qualitySummary(observation.registration.quality);
        observation.evidence.disposition = ObservationDisposition::Conflict;
        observation.evidence.reason = reason.str();
        return;
    }

    observation.evidence.disposition = ObservationDisposition::Measurement;
}
/// 从关键帧创建时冻结的因果历史生成跟踪源点云。
bool TrackingMatcher::prepareSource(GlobalObservationWork& work) const {
    if (work.source_snapshot_missing) {
        const auto cloud = work.fallback_source.load();
        if (cloud)
            work.source_snapshot.reference.cloud_local.reset(new Cloud(*cloud));
        work.source_snapshot.causal_history = {work.source_snapshot.reference};
        BACKEND_WARN_STREAM("runtime keyframe "
                            << work.runtime.index
                            << " has no frozen tracking-source snapshot; using one frame");
    }
    auto source = accumulateSource(work.source_snapshot.reference,
                                   work.source_snapshot.causal_history, config_.tracking.source);
    work.accumulated_frames = source.frame_count;
    work.source_span_seconds = source.span_seconds;
    work.source = std::move(source.cloud);
    if (work.accumulated_frames < config_.tracking.source.min_frames) {
        work.prior_input.reason = "prior scan-to-map source has too few causal frames";
        return false;
    }
    if (!work.source || work.source->empty()) {
        work.prior_input.reason = "prior scan-to-map source is empty";
        work.prior_input.disposition = ObservationDisposition::ModelInvalid;
        return false;
    }
    return true;
}
/// 按距离与切换滞回选择先验地图参考关键帧。
void TrackingMatcher::selectReference(GlobalObservationWork& work) const {
    const Eigen::Vector3d reference_query = work.predicted_map_pose.block<3, 1>(0, 3);
    const std::vector<int> nearest = prior_map_.nearestKeyframeIndices(
        reference_query, 1, config_.tracking.nearest_reference_radius);
    if (nearest.empty()) {
        work.prior_input.reason = "no spatial prior-map neighbor";
        work.prior_input.disposition = ObservationDisposition::ModelInvalid;
        return;
    }
    int selected_reference = nearest.front();
    if (!prior_map_.get(selected_reference, &work.reference)) {
        work.prior_input.reason = "invalid spatial prior-map neighbor";
        work.prior_input.disposition = ObservationDisposition::ModelInvalid;
        return;
    }
    if (work.previous_reference_index >= 0 && work.previous_reference_index != selected_reference &&
        config_.tracking.target_hysteresis_distance > 0.0) {
        Keyframe previous_reference;
        if (prior_map_.get(work.previous_reference_index, &previous_reference)) {
            const double nearest_distance =
                (work.reference.optimized_pose.block<3, 1>(0, 3) - reference_query).norm();
            const double previous_distance =
                (previous_reference.optimized_pose.block<3, 1>(0, 3) - reference_query).norm();
            const bool inside_search_radius =
                config_.tracking.nearest_reference_radius <= 0.0 ||
                previous_distance <= config_.tracking.nearest_reference_radius;
            if (inside_search_radius &&
                previous_distance <=
                    nearest_distance + config_.tracking.target_hysteresis_distance) {
                selected_reference = work.previous_reference_index;
                work.reference = previous_reference;
            }
        }
    }
}
namespace {
/// 报告先验地图观测的接受、模型失效或冲突诊断。
void reportPriorTracking(const GlobalObservationWork& work,
                         const PriorMapObservation& prior_observation,
                         const GlobalObservationCommit& commit, const Config& config,
                         double measurement_elapsed_ms) {
    if (commit.prior_accepted) {
        BACKEND_INFO_STREAM(
            "prior scan-to-map factor accepted: runtime="
            << work.runtime.index << ", reference=" << work.reference.index
            << ", source_frames=" << work.accumulated_frames << "/"
            << config.tracking.source.max_frames << ", source_span=" << work.source_span_seconds
            << "s, " << qualitySummary(prior_observation.registration.quality)
            << ", iterations=" << prior_observation.registration.iterations
            << ", coarse=" << std::boolalpha << prior_observation.registration.used_coarse_level
            << ", NIS=" << commit.prior_innovation.nis << "/"
            << commit.prior_innovation.degrees_of_freedom
            << ", p=" << commit.prior_innovation.upper_tail_probability << ", elapsed_ms="
            << prior_observation.elapsed_ms << ", measurement_ms=" << measurement_elapsed_ms);
    } else if (commit.model_invalid) {
        BACKEND_WARN_STREAM("prior scan-to-map model invalid: runtime="
                            << work.runtime.index << ", " << prior_observation.evidence.reason
                            << ", source_frames=" << work.accumulated_frames << "/"
                            << config.tracking.source.max_frames
                            << ", source_span=" << work.source_span_seconds
                            << "s, elapsed_ms=" << prior_observation.elapsed_ms);
    } else if (prior_observation.evidence.disposition == ObservationDisposition::Conflict ||
               (commit.prior_innovation.valid && commit.prior_innovation.upper_tail_probability <
                                                     config.innovation.rejection_probability)) {
        BACKEND_WARN_STREAM(
            "prior scan-to-map factor conflicted: runtime="
            << work.runtime.index << ", reference=" << work.reference.index
            << ", source_frames=" << work.accumulated_frames << "/"
            << config.tracking.source.max_frames << ", source_span=" << work.source_span_seconds
            << "s, " << qualitySummary(prior_observation.registration.quality)
            << ", iterations=" << prior_observation.registration.iterations
            << ", coarse=" << std::boolalpha << prior_observation.registration.used_coarse_level
            << ", delta_t=" << prior_observation.predicted_translation_delta << "m, delta_r_deg="
            << prior_observation.predicted_rotation_delta_rad * 180.0 / M_PI << ", "
            << (prior_observation.evidence.reason.empty()
                    ? "NIS=" + std::to_string(commit.prior_innovation.nis) + "/" +
                          std::to_string(commit.prior_innovation.degrees_of_freedom)
                    : prior_observation.evidence.reason));
    }
}
}  // namespace

/// 报告跟踪观测、创新检验和最终图提交的诊断结果。
void reportTracking(const GlobalObservationWork& work, const PriorMapObservation& prior_observation,
                    const GlobalObservationCommit& commit, const Config& config,
                    double measurement_elapsed_ms) {
    if (commit.stale || commit.graph_node_unavailable) {
        return;
    }

    if (prior_observation.evidence.configured)
        reportPriorTracking(work, prior_observation, commit, config, measurement_elapsed_ms);
    if (work.uwb) {
        if (commit.uwb_accepted) {
            BACKEND_INFO_STREAM("UWB factor accepted in system transaction: runtime="
                                << work.runtime.index << ", NIS=" << commit.uwb_innovation.nis
                                << "/" << commit.uwb_innovation.degrees_of_freedom
                                << ", p=" << commit.uwb_innovation.upper_tail_probability);
        } else if (commit.uwb_innovation.valid) {
            BACKEND_WARN_STREAM("UWB factor conflicted in system transaction: runtime="
                                << work.runtime.index << ", NIS=" << commit.uwb_innovation.nis
                                << "/" << commit.uwb_innovation.degrees_of_freedom
                                << ", p=" << commit.uwb_innovation.upper_tail_probability);
        }
    }
    if (commit.prior_accepted || commit.uwb_accepted) {
        BACKEND_INFO_STREAM(
            "global correction committed: runtime="
            << work.runtime.index
            << ", sources=" << (commit.prior_accepted ? "prior_scan_to_map" : "")
            << (commit.prior_accepted && commit.uwb_accepted ? "+" : "")
            << (commit.uwb_accepted ? "uwb" : "")
            << (commit.joint_innovation.valid
                    ? ", joint_NIS=" + std::to_string(commit.joint_innovation.nis) + "/" +
                          std::to_string(commit.joint_innovation.degrees_of_freedom)
                    : "")
            << ", delta_t=" << commit.applied_correction.translation
            << "m, delta_r_deg=" << commit.applied_correction.rotation_rad * 180.0 / M_PI);
    }
}
}  // namespace backend
