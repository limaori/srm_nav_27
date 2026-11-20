/**
 * @file recovery_matcher.cpp
 * @brief 按启动提示、UWB 位置与描述子次序求解恢复位姿。
 */

#include <algorithm>
#include <utility>

#include "backend/frame_history.hpp"
#include "backend/geometry.hpp"
#include "backend/logger.hpp"
#include "backend/matching.hpp"
#include "backend/sampling.hpp"

namespace backend {
/// 累计恢复源历史并构造粗细分辨率点云及先验位置快照。
void RecoveryMatcher::prepare(RecoveryWork& work) const {
    auto source = accumulateSource(work.frame, work.source_history, config_.recovery.source);
    work.accumulated_frames = source.frame_count;
    work.source_span_seconds = source.span_seconds;
    const Cloud::Ptr source_cloud = std::move(source.cloud);
    work.runtime.cloud = CloudBlock(source_cloud);
    const bool source_ready = source_cloud && !source_cloud->empty() &&
                              work.accumulated_frames >= config_.recovery.source.min_frames;
    work.prior_positions = prior_map_.positions();
    if (source_cloud && !source_cloud->empty()) {
        work.fine_source =
            voxelize(*source_cloud, config_.relocalization.point_to_plane_config.voxel_resolution);
        if (config_.relocalization.point_to_plane_config.voxel_resolution > 0)
            work.coarse_source =
                voxelize(*work.fine_source,
                         2 * config_.relocalization.point_to_plane_config.voxel_resolution);
    }
    if (source_ready) {
        work.source_evidence.disposition = ObservationDisposition::Measurement;
    } else if (work.source_history.size() >=
               static_cast<std::size_t>(std::max(1, config_.recovery.source.min_frames))) {
        work.source_evidence.disposition = ObservationDisposition::ModelInvalid;
        work.source_evidence.reason =
            "recovery source continuity gate rejected available causal "
            "frames";
    } else {
        work.source_evidence.disposition = ObservationDisposition::Unavailable;
        work.source_evidence.reason = "waiting for post-boundary recovery source frames";
    }
}
/// 执行单个位置先验或描述子恢复分支并保留完整可观位姿结果。
bool RecoveryMatcher::match(const RecoveryWork& work,
                            const RelocalizationPositionPrior* position_prior,
                            RecoveryResult& out) {
    if (work.prior_positions.empty())
        return false;

    RelocalizationRequest request{work.runtime, config_.relocalization, work.prior_positions};
    request.status = work.identity.status;
    request.T_map_runtime_pred = work.predicted_map_pose;
    request.fine_source = work.fine_source;
    request.coarse_source = work.coarse_source;
    request.position_prior = position_prior;
    request.source_information_scale =
        1.0 / static_cast<double>(std::max(1, work.accumulated_frames));
    if (position_prior == nullptr) {
        if (work.identity.status == LocalizationStatus::Suspect) {
            request.search_radius = config_.recovery.suspect_search_radius;
            request.global_fallback_candidates = 0;
        } else {
            request.search_radius = config_.recovery.lost_search_radius;
            request.global_fallback_candidates = config_.recovery.global_fallback_candidates;
        }
    }

    const RelocalizationResult result =
        runRelocalization(prior_map_, descriptor_, target_cache_, request);
    if (!result.success || !result.covariance_valid) {
        if (result.success) {
            BACKEND_WARN_STREAM(
                "relocalization produced no full-rank pose likelihood; "
                "keeping recovery active");
        }
        return false;
    }
    out.pose = result.T_map_odom * work.runtime.odom_pose;
    out.covariance = result.covariance;
    out.source = result.source;
    if (config_.debug_enabled && result.prior_index >= 0) {
        Keyframe reference;
        const RelocalizationPreparedTarget target = target_cache_.getOrPrepare(
            result.prior_index, config_.relocalization.relocalization_submap_keyframes,
            config_.relocalization.point_to_plane_config.voxel_resolution, prior_map_,
            config_.relocalization.relocalization_submap_radius);
        const auto& source_debug = work.fine_source;
        if (prior_map_.get(result.prior_index, &reference) && target.ready() && source_debug) {
            out.debug_aligned_source = transformCloud(*source_debug, out.pose);
            out.debug_target = transformCloud(*target.target, reference.optimized_pose);
        }
    }
    return true;
}

/// 按启动提示、UWB 和描述子优先级求解完整可观恢复位姿。
RecoveryResult RecoveryMatcher::compute(const RecoveryWork& work) {
    RecoveryResult result;
    // 显式责任链：完整启动提示 → UWB 位置先验 → 描述子；每个分支仍调用同一匹配器。
    if (work.startup_hint_prior) {
        if (match(work, &*work.startup_hint_prior, result)) {
            result.outcome = RecoveryOutcome::Matched;
            result.source = "startup hint + scan-to-map";
            return result;
        }
        if (work.force_startup_hint) {
            BACKEND_DEBUG_STREAM(
                "forced startup-hint scan-to-map failed; retrying on the next synchronized frame");
            return result;
        }
        if (work.source_evidence.disposition == ObservationDisposition::Unavailable) {
            BACKEND_INFO_STREAM("startup-hint scan-to-map failed; waiting for recovery source ("
                                << work.accumulated_frames << "/"
                                << config_.recovery.source.min_frames
                                << " frames, span=" << work.source_span_seconds << "s)");
            return result;
        }
    }
    if (work.source_evidence.disposition == ObservationDisposition::ModelInvalid) {
        result.outcome = RecoveryOutcome::Failed;
        result.source = work.source_evidence.reason;
        return result;
    }
    if (work.source_evidence.disposition != ObservationDisposition::Measurement)
        return result;
    if (work.uwb_prior && match(work, &*work.uwb_prior, result)) {
        result.outcome = RecoveryOutcome::Matched;
        result.source = "UWB + scan-to-map";
        return result;
    }
    if (match(work, nullptr, result)) {
        result.outcome = RecoveryOutcome::Matched;
        result.source = (result.source.empty() ? "descriptor" : result.source) + " + scan-to-map";
        return result;
    }
    result.outcome = RecoveryOutcome::Failed;
    result.source = work.uwb_prior ? "UWB preference and descriptor fallback" : "descriptor";
    return result;
}
}  // namespace backend
