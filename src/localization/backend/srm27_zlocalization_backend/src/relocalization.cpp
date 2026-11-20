/**
 * @file relocalization.cpp
 * @brief 生成恢复假设并编排确定性地点搜索。
 */

#include <algorithm>
#include <iterator>
#include <numeric>
#include <sstream>
#include <utility>

#include "backend/geometry.hpp"
#include "backend/logger.hpp"
#include "backend/sampling.hpp"
#include "backend/search.hpp"
#include "search.hpp"

namespace backend {

/// 生成对称偏航序列，可信初值先验证零偏移再解锁其余角度。
YawSweepSchedule makeYawSweepSchedule(const int samples, const bool has_trusted_seed) {
    const int count = std::max(1, samples);
    std::vector<int> symmetric_offsets;
    symmetric_offsets.reserve(static_cast<std::size_t>(count));
    symmetric_offsets.push_back(0);
    for (int step = 1; static_cast<int>(symmetric_offsets.size()) < count; ++step) {
        symmetric_offsets.push_back(step);
        if (static_cast<int>(symmetric_offsets.size()) >= count) {
            break;
        }
        if (count % 2 == 0 && step == count / 2) {
            continue;
        }
        symmetric_offsets.push_back(-step);
    }

    YawSweepSchedule schedule;
    if (!has_trusted_seed) {
        schedule.primary_offsets = std::move(symmetric_offsets);
        return schedule;
    }
    schedule.primary_offsets = {0};
    schedule.fallback_offsets.assign(std::next(symmetric_offsets.begin()), symmetric_offsets.end());
    return schedule;
}

/// 按二维或三维位置距离召回先验关键帧，等距离按编号排序。
std::vector<int> nearestPriorKeyframeIndices(const std::vector<Eigen::Vector3d>& prior_positions,
                                             const Eigen::Vector3d& map_point, const bool use_z,
                                             const int max_candidates) {
    std::vector<int> indices(prior_positions.size());
    std::iota(indices.begin(), indices.end(), 0);
    /// 按位置先验使用的维度计算关键帧距离平方。
    const auto squared_distance = [&](const int index) {
        const Eigen::Vector3d delta = prior_positions[static_cast<std::size_t>(index)] - map_point;
        return use_z ? delta.squaredNorm() : delta.head<2>().squaredNorm();
    };
    const auto limit =
        std::min(indices.size(), static_cast<std::size_t>(std::max(1, max_candidates)));
    std::partial_sort(indices.begin(), indices.begin() + limit, indices.end(),
                      [&](const int lhs, const int rhs) {
                          const double lhs_distance = squared_distance(lhs);
                          const double rhs_distance = squared_distance(rhs);
                          if (lhs_distance != rhs_distance) {
                              return lhs_distance < rhs_distance;
                          }
                          return lhs < rhs;
                      });
    indices.resize(limit);
    return indices;
}

/// 创建仅在本次计算中存活的检索工作区并返回恢复结果。
RelocalizationResult runRelocalization(const KeyframeMap& keyframes,
                                       const PlaceDescriptor& descriptor,
                                       RelocalizationTargetCache& target_cache,
                                       const RelocalizationRequest& request) {
    relocalization_detail::Search search(keyframes, descriptor, target_cache, request);
    return search.run();
}

}  // namespace backend

namespace backend::relocalization_detail {

/// 绑定一次恢复计算必需的地图、描述子、配置和请求引用。
Search::Search(const KeyframeMap& keyframes, const PlaceDescriptor& descriptor,
               RelocalizationTargetCache& target_cache, const RelocalizationRequest& request)
    : keyframes_(keyframes),
      descriptor_(descriptor),
      target_cache_(target_cache),
      request_(request),
      config_(request.config),
      prior_kf_positions_(request.prior_kf_positions) {
}

/// 执行当前恢复请求的位置来源或描述子回退链。
RelocalizationResult Search::run() {
    RelocalizationResult result;
    const bool descriptor_source = request_.position_prior == nullptr;
    if (descriptor_source && (!descriptor_.enabled() || descriptor_.size() == 0)) {
        return result;
    }
    backend_name_ = descriptor_source ? descriptor_.name() : request_.position_prior->source;
    initializing_ = request_.status == LocalizationStatus::Initializing;

    if (descriptor_source) {
        if (!queryCandidates()) {
            return result;
        }
    }
    if (!prepareScanToMapInputs()) {
        return result;
    }

    if (descriptor_source) {
        if (tryLocalCandidates()) {
            return makeResult();
        }
        if (tryGlobalFallback()) {
            return makeResult();
        }

        BACKEND_WARN_STREAM("tryRelocalization: scan-to-map verification failed after "
                            << tried_verifications_ << " alignment attempts"
                            << " (best_descriptor_distance=" << best_descriptor_distance_
                            << ", best_residual_score=" << best_failed_residual_score_
                            << ", best_quality=" << qualitySummary(best_failed_quality_)
                            << ", global_fallback_considered=" << global_fallback_considered_
                            << ")");
        return result;
    }

    if (tryPositionPrior()) {
        return makeResult();
    }
    BACKEND_WARN_STREAM("tryRelocalization: "
                        << backend_name_ << " position-prior verification failed after "
                        << tried_verifications_ << " alignment attempts"
                        << " (best_residual_score=" << best_failed_residual_score_
                        << ", best_quality=" << qualitySummary(best_failed_quality_) << ")");
    return result;
}

/// 按三维空间半径筛选描述子召回允许的先验关键帧。
std::vector<int> Search::findPriorKeyframesWithinRadius(const Eigen::Vector3d& center,
                                                        const double radius) const {
    std::vector<int> out;
    for (std::size_t i = 0; i < prior_kf_positions_.size(); ++i) {
        if (radius <= 0.0) {
            out.push_back(static_cast<int>(i));
            continue;
        }
        const Eigen::Vector3d delta = prior_kf_positions_[i] - center;
        const double distance = delta.norm();
        if (distance <= radius) {
            out.push_back(static_cast<int>(i));
        }
    }
    return out;
}

/// 返回候选中的最小描述子距离，空集合返回最大值。
double Search::bestRetrievalDistance(const std::vector<LoopCandidate>& items) {
    if (items.empty()) {
        return std::numeric_limits<double>::max();
    }
    const auto it = std::min_element(items.begin(), items.end(),
                                     [](const LoopCandidate& lhs, const LoopCandidate& rhs) {
                                         return lhs.descriptor_distance < rhs.descriptor_distance;
                                     });
    return it->descriptor_distance;
}

/// 准备局部描述子召回并确定全局补充预算和距离门限。
bool Search::queryCandidates() {
    const Eigen::Vector3d anchor = request_.T_map_runtime_pred.block<3, 1>(0, 3);
    local_candidate_count_ = std::max(1, config_.relocalization_max_candidates);
    global_fallback_budget_ = std::max(0, request_.global_fallback_candidates);

    subset_ = findPriorKeyframesWithinRadius(anchor, request_.search_radius);
    if (subset_.empty()) {
        if (global_fallback_budget_ <= 0) {
            BACKEND_WARN_STREAM("tryRelocalization: no prior KFs within radius "
                                << request_.search_radius << "m of pose prior");
            return false;
        }
        BACKEND_WARN_STREAM("tryRelocalization: no prior KFs within radius "
                            << request_.search_radius << "m of pose prior; falling back to global "
                            << backend_name_ << " retrieval");
    }

    const bool initial_query_is_global = subset_.empty();
    const bool local_search_is_global =
        initial_query_is_global || (!subset_.empty() && subset_.size() >= descriptor_.size());
    can_global_supplement_ = global_fallback_budget_ > 0 && !local_search_is_global;

    levelQueryCloud();
    const int initial_candidate_count =
        initial_query_is_global
            ? std::max(local_candidate_count_, std::max(1, global_fallback_budget_))
            : local_candidate_count_;
    candidates_ = descriptor_.queryLocalizationCandidates(*level_query_cloud_, subset_,
                                                          initial_candidate_count);

    if (candidates_.empty()) {
        BACKEND_WARN_STREAM("tryRelocalization: " << backend_name_
                                                  << " search returned no candidates");
        if (!can_global_supplement_) {
            return false;
        }
    }

    best_descriptor_distance_ = bestRetrievalDistance(candidates_);
    descriptor_distance_limit_ = config_.relocalization_descriptor_distance_limit;
    if (!candidates_.empty() && best_descriptor_distance_ > descriptor_distance_limit_) {
        BACKEND_WARN_STREAM("tryRelocalization: "
                            << backend_name_ << " search found no candidate worth scan-to-map"
                            << " (best_descriptor_distance=" << best_descriptor_distance_
                            << ", limit=" << descriptor_distance_limit_
                            << ", radius=" << request_.search_radius << "m)");
    }

    return true;
}

/// 按描述子约定整平当前源云，保留原始匹配源坐标。
void Search::levelQueryCloud() {
    const auto cloud = request_.runtime_kf.cloud.load();
    if (!cloud) {
        level_query_cloud_.reset(new Cloud);
        return;
    }
    if (descriptor_.levelWithOdomRollPitch()) {
        level_query_cloud_ =
            transformCloud(*cloud, levelFromBodyTransform(request_.runtime_kf.odom_pose));
    } else {
        level_query_cloud_.reset(new Cloud(*cloud));
    }
}

/// 准备细层配准源并记录目标子图的关键帧预算。
bool Search::prepareScanToMapInputs() {
    const auto cloud = request_.runtime_kf.cloud.load();
    if (!cloud)
        return false;
    source_ds_ = request_.fine_source
                     ? request_.fine_source
                     : voxelize(*cloud, config_.point_to_plane_config.voxel_resolution);
    if (source_ds_->empty()) {
        return false;
    }

    target_keyframe_limit_ = std::max(0, config_.relocalization_submap_keyframes);
    return true;
}

/// 按先验关键帧和子图配置取得可复用的几何验证目标。
RelocalizationPreparedTarget Search::directTargetFor(const int center_index) {
    return target_cache_.getOrPrepare(center_index, target_keyframe_limit_,
                                      config_.point_to_plane_config.voxel_resolution, keyframes_,
                                      config_.relocalization_submap_radius);
}

/// 将配准支撑、残差和可观性诊断格式化为日志文本。
std::string Search::qualitySummary(const ObservabilityQuality& quality) {
    std::ostringstream oss;
    oss << "residual_ok=" << (quality.residual_ok ? "true" : "false")
        << ", support_ok=" << (quality.correspondences_ok ? "true" : "false")
        << ", observable=" << (quality.observable ? "true" : "false")
        << ", corr_ratio=" << quality.correspondence_ratio
        << ", sectors=" << quality.coverage_sectors << ", rmse=" << quality.rmse
        << ", min_eig=" << quality.min_eigenvalue << ", cond=" << quality.condition_number;
    return oss.str();
}

/// 检查候选编号、通用距离门限和启动专用距离门限。
bool Search::descriptorGatePassed(const LoopCandidate& hit) const {
    if (hit.index < 0 || hit.descriptor_distance > descriptor_distance_limit_) {
        return false;
    }
    return !initializing_ || config_.initial_relocalization_max_descriptor_distance <= 0.0 ||
           hit.descriptor_distance <= config_.initial_relocalization_max_descriptor_distance;
}

}  // namespace backend::relocalization_detail
