/**
 * @file retrieval.cpp
 * @brief 实现 G-PROBE 的粗提名、航向验证和确定性候选排序。
 */

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

#include "backend/math.hpp"
#include "gprobe.hpp"

namespace backend {

using namespace gprobe_detail;

/// 根据查询传感器视场配置生成角度掩码。
Mask GProbeDescriptor::Impl::queryMask() const {
    return angleMask(config.num_sectors, {config.query_fov_center_deg}, config.query_fov_deg);
}

/// 构造地图分支、查询分支和对应的关键帧位置。
FrameDescriptor GProbeDescriptor::Impl::makeFrame(const Cloud& cloud,
                                                  const Eigen::Vector3d& position) const {
    FrameDescriptor frame;
    frame.position = position;
    const auto pairs = makeMapPairs(config.num_sectors);
    const auto sampled = downsampleFirstPoint(cloud, config.voxel_size);
    for (int pair = 0; pair < kMapBranchCount; ++pair) {
        frame.map_branches[static_cast<std::size_t>(pair)] =
            makeSampledBranch(sampled, pairs[static_cast<std::size_t>(pair)].mask, config);
    }
    frame.query_branch = makeSampledBranch(sampled, queryMask(), config);
    return frame;
}

/// 规范化候选索引，空集合按完整索引展开。
std::vector<int> GProbeDescriptor::Impl::candidateIndices(
    const std::vector<int>& allowed_indices) const {
    std::vector<int> indices;
    if (allowed_indices.empty()) {
        indices.resize(frames.size());
        std::iota(indices.begin(), indices.end(), 0);
        return indices;
    }
    indices.reserve(allowed_indices.size());
    for (const int index : allowed_indices) {
        if (index >= 0 && index < static_cast<int>(frames.size())) {
            indices.push_back(index);
        }
    }
    std::sort(indices.begin(), indices.end());
    indices.erase(std::unique(indices.begin(), indices.end()), indices.end());
    return indices;
}

/// 对已排序候选执行基于地图距离的非极大值抑制。
std::vector<int> GProbeDescriptor::Impl::selectDiverse(
    const std::vector<std::pair<int, double>>& ranked, const int count) const {
    std::vector<int> selected;
    selected.reserve(static_cast<std::size_t>(count));
    const double radius_squared = config.spatial_nms_radius * config.spatial_nms_radius;
    for (const auto& [index, score] : ranked) {
        (void)score;
        if (radius_squared > 0.0 &&
            std::any_of(selected.begin(), selected.end(), [&](const int other) {
                return (frames[static_cast<std::size_t>(index)].position.head<2>() -
                        frames[static_cast<std::size_t>(other)].position.head<2>())
                           .squaredNorm() < radius_squared;
            })) {
            continue;
        }
        selected.push_back(index);
        if (static_cast<int>(selected.size()) >= count) {
            break;
        }
    }
    return selected;
}

/// 按地图视场分支粗检索并提名空间分散的候选。
std::vector<std::vector<Nomination>> GProbeDescriptor::Impl::buildNominations(
    const Branch& query, const std::vector<int>& indices, const Mask& query_mask,
    const PairSet& pairs) const {
    std::vector<std::vector<Nomination>> nominations(frames.size());
    for (int pair_index = 0; pair_index < kMapBranchCount; ++pair_index) {
        const PairGeometry& pair = pairs[static_cast<std::size_t>(pair_index)];
        const double hint_deg = wrapDegrees(config.query_fov_center_deg - pair.center_deg);
        const int hint_shift =
            static_cast<int>(std::round(hint_deg / (360.0 / config.num_sectors)));
        Mask overlap(static_cast<std::size_t>(config.num_sectors), 0U);
        for (int sector = 0; sector < config.num_sectors; ++sector) {
            const int database_sector = rolledSourceIndex(sector, hint_shift, config.num_sectors);
            overlap[static_cast<std::size_t>(sector)] =
                query_mask[static_cast<std::size_t>(sector)] != 0U &&
                        pair.mask[static_cast<std::size_t>(database_sector)] != 0U
                    ? 1U
                    : 0U;
        }

        const std::vector<double> query_key = normalizedKey(query, overlap, 0, config);
        std::vector<std::pair<int, double>> similarities;
        similarities.reserve(indices.size());
        for (const int index : indices) {
            const Branch& database = frames[static_cast<std::size_t>(index)]
                                         .map_branches[static_cast<std::size_t>(pair_index)];
            const std::vector<double> database_key =
                normalizedKey(database, overlap, hint_shift, config);
            similarities.emplace_back(index, dotProduct(query_key, database_key));
        }
        std::sort(similarities.begin(), similarities.end(), [](const auto& lhs, const auto& rhs) {
            if (lhs.second != rhs.second) {
                return lhs.second > rhs.second;
            }
            return lhs.first < rhs.first;
        });

        const int branch_count =
            std::min(std::max(1, config.branch_top_k), static_cast<int>(similarities.size()));
        const std::vector<int> selected = selectDiverse(similarities, branch_count);
        const double fov_weight =
            static_cast<double>(countMask(overlap)) /
            std::max(1, std::min(countMask(query_mask), countMask(pair.mask)));
        for (const int index : selected) {
            const auto found =
                std::find_if(similarities.begin(), similarities.end(),
                             [index](const auto& entry) { return entry.first == index; });
            nominations[static_cast<std::size_t>(index)].push_back(
                {pair_index, found != similarities.end() ? found->second : 0.0, fov_weight});
        }
    }
    return nominations;
}

/// 结合粗键相似度和视场支持度选择精验证候选。
std::vector<int> GProbeDescriptor::Impl::prefilterCandidates(
    const std::vector<int>& indices,
    const std::vector<std::vector<Nomination>>& nominations) const {
    std::vector<int> prefilter;
    prefilter.reserve(indices.size());
    for (const int index : indices) {
        if (!nominations[static_cast<std::size_t>(index)].empty()) {
            prefilter.push_back(index);
        }
    }
    /// 分别累加候选的视场权重和粗匹配相似度。
    const auto totals = [&](const int index) {
        double support = 0.0;
        double similarity = 0.0;
        for (const Nomination& nomination : nominations[static_cast<std::size_t>(index)]) {
            support += nomination.fov_weight;
            similarity += nomination.similarity;
        }
        return std::pair<double, double>{support, similarity};
    };
    std::sort(prefilter.begin(), prefilter.end(), [&](const int lhs, const int rhs) {
        const auto left = totals(lhs);
        const auto right = totals(rhs);
        if (left.first != right.first) {
            return left.first > right.first;
        }
        if (left.second != right.second) {
            return left.second > right.second;
        }
        return lhs < rhs;
    });
    if (prefilter.size() > static_cast<std::size_t>(config.verification_top_k)) {
        prefilter.resize(static_cast<std::size_t>(config.verification_top_k));
    }
    return prefilter;
}

/// 将六个地图分支归并为两个正交航向轴假设。
GProbeDescriptor::Impl::HeadingModel GProbeDescriptor::Impl::makeHeadingModel(
    const PairSet& pairs) const {
    HeadingModel model;
    for (int pair_index = 0; pair_index < kMapBranchCount; ++pair_index) {
        const double hint = wrapDegrees(config.query_fov_center_deg -
                                        pairs[static_cast<std::size_t>(pair_index)].center_deg);
        int hypothesis = static_cast<int>(std::round(hint / 90.0)) * 90;
        hypothesis = static_cast<int>(wrapDegrees(hypothesis));
        model.hypotheses[static_cast<std::size_t>(pair_index)] = hypothesis;
        for (int other = 0; other < kMapBranchCount; ++other) {
            const double other_hint = wrapDegrees(
                config.query_fov_center_deg - pairs[static_cast<std::size_t>(other)].center_deg);
            int other_hypothesis = static_cast<int>(std::round(other_hint / 90.0)) * 90;
            other_hypothesis = static_cast<int>(wrapDegrees(other_hypothesis));
            if (other_hypothesis == hypothesis) {
                ++model.hypothesis_sizes[static_cast<std::size_t>(pair_index)];
            }
        }
    }
    return model;
}

/// 在提名分支的航向窗口内搜索并计算概率得分。
GProbeDescriptor::Impl::PairEvaluation GProbeDescriptor::Impl::evaluatePairs(
    const int index, const Branch& query, const Mask& query_mask, const PairSet& pairs,
    const std::vector<std::vector<Nomination>>& nominations) const {
    PairEvaluation evaluation;
    const int window_bins =
        static_cast<int>(std::round(config.heading_window_deg / (360.0 / config.num_sectors)));
    for (const Nomination& nomination : nominations[static_cast<std::size_t>(index)]) {
        const int pair_index = nomination.pair;
        const PairGeometry& pair = pairs[static_cast<std::size_t>(pair_index)];
        const double hint_deg = wrapDegrees(config.query_fov_center_deg - pair.center_deg);
        const int hint_shift =
            static_cast<int>(std::round(hint_deg / (360.0 / config.num_sectors)));
        Mask query_overlap(static_cast<std::size_t>(config.num_sectors), 0U);
        Mask database_overlap(static_cast<std::size_t>(config.num_sectors), 0U);
        for (int sector = 0; sector < config.num_sectors; ++sector) {
            query_overlap[static_cast<std::size_t>(sector)] =
                query_mask[static_cast<std::size_t>(sector)] != 0U &&
                        pair.mask[static_cast<std::size_t>(
                            rolledSourceIndex(sector, hint_shift, config.num_sectors))] != 0U
                    ? 1U
                    : 0U;
            database_overlap[static_cast<std::size_t>(sector)] =
                pair.mask[static_cast<std::size_t>(sector)] != 0U &&
                        query_mask[static_cast<std::size_t>(
                            rolledSourceIndex(sector, -hint_shift, config.num_sectors))] != 0U
                    ? 1U
                    : 0U;
        }

        const Branch& database = frames[static_cast<std::size_t>(index)]
                                     .map_branches[static_cast<std::size_t>(pair_index)];
        int best_shift = hint_shift;
        double best_correlation = -std::numeric_limits<double>::infinity();
        for (int shift = hint_shift - window_bins; shift <= hint_shift + window_bins; ++shift) {
            const double correlation = correlationForShift(query, database, query_overlap,
                                                           database_overlap, shift, config);
            if (correlation > best_correlation) {
                best_correlation = correlation;
                best_shift = shift;
            }
        }
        evaluation.scores[static_cast<std::size_t>(pair_index)] =
            branchScore(query, database, query_mask, pair.mask, best_shift, config);
        evaluation.nominated[static_cast<std::size_t>(pair_index)] = true;
    }
    return evaluation;
}

/// 计算两个正交航向轴中主导轴的 S-GRT 概率。
double GProbeDescriptor::Impl::dominantAxisProbability(const PairEvaluation& evaluation,
                                                       const HeadingModel& model) const {
    std::array<double, 2> axis_scores{};
    std::array<bool, 2> axis_present{};
    for (int pair_index = 0; pair_index < kMapBranchCount; ++pair_index) {
        const int hypothesis = model.hypotheses[static_cast<std::size_t>(pair_index)];
        bool first_in_hypothesis = true;
        for (int previous = 0; previous < pair_index; ++previous) {
            if (model.hypotheses[static_cast<std::size_t>(previous)] == hypothesis) {
                first_in_hypothesis = false;
                break;
            }
        }
        if (!first_in_hypothesis) {
            continue;
        }

        double sum = 0.0;
        for (int other = 0; other < kMapBranchCount; ++other) {
            if (model.hypotheses[static_cast<std::size_t>(other)] == hypothesis &&
                evaluation.nominated[static_cast<std::size_t>(other)]) {
                sum += evaluation.scores[static_cast<std::size_t>(other)];
            }
        }
        const double mean =
            sum / std::max(1, model.hypothesis_sizes[static_cast<std::size_t>(pair_index)]);
        const int axis = wrapIndex(hypothesis, 180) == 0 ? 0 : 1;
        axis_scores[static_cast<std::size_t>(axis)] =
            std::max(axis_scores[static_cast<std::size_t>(axis)], mean);
        axis_present[static_cast<std::size_t>(axis)] = true;
    }

    const int best_axis = axis_scores[0] >= axis_scores[1] ? 0 : 1;
    const double best_axis_score = axis_scores[static_cast<std::size_t>(best_axis)];
    if (best_axis_score <= 0.0) {
        return 1.0;
    }
    double denominator = 1.0;
    for (int axis = 0; axis < 2; ++axis) {
        if (axis == best_axis || !axis_present[static_cast<std::size_t>(axis)]) {
            continue;
        }
        denominator +=
            std::exp(config.sgrt_kappa *
                     (axis_scores[static_cast<std::size_t>(axis)] / best_axis_score - 1.0));
    }
    return 1.0 / denominator;
}

/// 将分支精验证结果归约为候选的地点一致性得分。
std::optional<ScoredCandidate> GProbeDescriptor::Impl::verifyCandidate(
    const int index, const Branch& query, const Mask& query_mask, const PairSet& pairs,
    const std::vector<std::vector<Nomination>>& nominations,
    const HeadingModel& heading_model) const {
    const PairEvaluation evaluation = evaluatePairs(index, query, query_mask, pairs, nominations);
    const double p_dominant = dominantAxisProbability(evaluation, heading_model);

    int best_pair = -1;
    double raw_score = -1.0;
    for (int pair_index = 0; pair_index < kMapBranchCount; ++pair_index) {
        if (evaluation.nominated[static_cast<std::size_t>(pair_index)] &&
            evaluation.scores[static_cast<std::size_t>(pair_index)] > raw_score) {
            raw_score = evaluation.scores[static_cast<std::size_t>(pair_index)];
            best_pair = pair_index;
        }
    }
    if (best_pair < 0) {
        return std::nullopt;
    }

    const double gamma = std::min(config.query_fov_deg, 360.0) / 360.0;
    return ScoredCandidate{index, std::max(0.0, raw_score) * std::pow(p_dominant, 1.0 - gamma)};
}

/// 执行粗提名、精验证和稳定的候选排序。
std::vector<ScoredCandidate> GProbeDescriptor::Impl::scoreQuery(
    const Branch& query, const std::vector<int>& allowed_indices) const {
    const std::vector<int> indices = candidateIndices(allowed_indices);
    if (indices.empty()) {
        return {};
    }

    const Mask query_mask = queryMask();
    const PairSet pairs = makeMapPairs(config.num_sectors);
    const auto nominations = buildNominations(query, indices, query_mask, pairs);
    const std::vector<int> prefilter = prefilterCandidates(indices, nominations);
    const HeadingModel heading_model = makeHeadingModel(pairs);

    std::vector<ScoredCandidate> scored;
    scored.reserve(prefilter.size());
    for (const int index : prefilter) {
        const auto candidate =
            verifyCandidate(index, query, query_mask, pairs, nominations, heading_model);
        if (candidate.has_value()) {
            scored.push_back(*candidate);
        }
    }
    std::sort(scored.begin(), scored.end(),
              [](const ScoredCandidate& lhs, const ScoredCandidate& rhs) {
                  if (lhs.score != rhs.score) {
                      return lhs.score > rhs.score;
                  }
                  return lhs.index < rhs.index;
              });
    return scored;
}

}  // namespace backend
