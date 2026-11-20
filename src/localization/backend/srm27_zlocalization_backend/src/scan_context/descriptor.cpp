/**
 * @file descriptor.cpp
 * @brief 实现 Scan Context 描述子构造、航向估计和候选检索。
 */

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include "backend/math.hpp"
#include "backend/sampling.hpp"
#include "backend/scan_context.hpp"

namespace backend {
namespace {

/// 生成配置要求的虚拟平移增广序列。
std::vector<Eigen::Vector2d> augmentationShifts(const ScanContextDescriptorConfig& config) {
    const double shift = std::max(0.0, config.lateral_shift);
    std::vector<Eigen::Vector2d> shifts{{0.0, 0.0},       {0.0, -shift},   {0.0, shift},
                                        {-shift, -shift}, {shift, -shift}, {-shift, shift},
                                        {shift, shift}};
    const int keep = std::clamp(config.augment_count, 1, static_cast<int>(shifts.size()));
    shifts.resize(keep);
    return shifts;
}

/// 使用扇区键确定最佳循环列对齐。
int bestSectorAlignment(const Eigen::VectorXf& query, const Eigen::VectorXf& candidate) {
    const int sectors = static_cast<int>(query.size());
    if (sectors <= 0 || candidate.size() != query.size()) {
        return 0;
    }
    int best_shift = 0;
    double best_score = -std::numeric_limits<double>::infinity();
    for (int shift = 0; shift < sectors; ++shift) {
        double dot = 0.0;
        double q_norm = 0.0;
        double c_norm = 0.0;
        for (int i = 0; i < sectors; ++i) {
            const double q = query((i + shift) % sectors);
            const double c = candidate(i);
            dot += q * c;
            q_norm += q * q;
            c_norm += c * c;
        }
        const double score =
            (q_norm > 0.0 && c_norm > 0.0) ? dot / (std::sqrt(q_norm) * std::sqrt(c_norm)) : 0.0;
        if (score > best_score) {
            best_score = score;
            best_shift = shift;
        }
    }
    return best_shift;
}

}  // namespace

/// 规范化描述子几何与检索配置，并推进内容修订号。
void ScanContextDescriptor::configure(const ScanContextDescriptorConfig& config) {
    revision_ = nextRevision();
    config_ = config;
    config_.num_rings = std::max(1, config_.num_rings);
    config_.num_sectors = std::max(1, config_.num_sectors);
    config_.max_radius = std::max(1.0, config_.max_radius);
    config_.downsample = std::max(0.0, config_.downsample);
    config_.min_bin_points = std::max(1, config_.min_bin_points);
    config_.augment_count = std::clamp(config_.augment_count, 1, 7);
    config_.lateral_shift = std::max(0.0, config_.lateral_shift);
    config_.search_ratio = std::clamp(config_.search_ratio, 0.0, 1.0);
    config_.exclude_recent = std::max(1, config_.exclude_recent);
    config_.candidates = std::max(1, config_.candidates);
    config_.distance_threshold = std::max(0.0, config_.distance_threshold);
}

/// 创建保留当前配置且没有关键帧的描述子实例。
std::unique_ptr<PlaceDescriptor> ScanContextDescriptor::createEmpty() const {
    auto descriptor = std::make_unique<ScanContextDescriptor>();
    descriptor->configure(config_);
    return descriptor;
}

/// 清空关键帧描述子并推进内容修订号。
void ScanContextDescriptor::clear() {
    descriptors_.clear();
    revision_ = nextRevision();
}

/// 将点云按极坐标网格编码为高度描述子。
ScanContextDescriptor::Descriptor ScanContextDescriptor::makeDescriptor(
    const Cloud& cloud_local, const Eigen::Vector2d& root_shift) const {
    return makeSampledDescriptor(*voxelize(cloud_local, config_.downsample), root_shift);
}

/// 将首点采样后的点集编码为指定平移下的高度描述子。
ScanContextDescriptor::Descriptor ScanContextDescriptor::makeSampledDescriptor(
    const Cloud& sampled, const Eigen::Vector2d& root_shift) const {
    Descriptor descriptor;
    descriptor.image = Eigen::MatrixXf::Zero(config_.num_rings, config_.num_sectors);
    Eigen::MatrixXi counts = Eigen::MatrixXi::Zero(config_.num_rings, config_.num_sectors);
    Eigen::MatrixXf max_z = Eigen::MatrixXf::Constant(config_.num_rings, config_.num_sectors,
                                                      -std::numeric_limits<float>::max());

    const double ring_gap = config_.max_radius / config_.num_rings;
    const double sector_gap = 2.0 * kPi / config_.num_sectors;
    for (const auto& point : sampled.points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
            continue;
        }
        const double x = static_cast<double>(point.x) - root_shift.x();
        const double y = static_cast<double>(point.y) - root_shift.y();
        const double z = static_cast<double>(point.z) + config_.lidar_height;
        const double radius = std::hypot(x, y);
        if (radius <= 0.0 || radius > config_.max_radius) {
            continue;
        }
        int ring = static_cast<int>(std::floor(radius / ring_gap));
        ring = std::clamp(ring, 0, config_.num_rings - 1);
        double theta = std::atan2(y, x);
        if (theta < 0.0) {
            theta += 2.0 * kPi;
        }
        int sector = static_cast<int>(std::floor(theta / sector_gap));
        sector = std::clamp(sector, 0, config_.num_sectors - 1);
        counts(ring, sector) += 1;
        max_z(ring, sector) = std::max(max_z(ring, sector), static_cast<float>(z));
    }

    for (int r = 0; r < config_.num_rings; ++r) {
        for (int s = 0; s < config_.num_sectors; ++s) {
            if (counts(r, s) >= config_.min_bin_points) {
                descriptor.image(r, s) = max_z(r, s);
            }
        }
    }
    descriptor.sector_key = descriptor.image.colwise().mean().transpose();
    return descriptor;
}

/// 构造原始及平移增广的描述子集合。
ScanContextDescriptor::DescriptorVariants ScanContextDescriptor::makeDescriptorVariants(
    const Cloud& cloud_local) const {
    DescriptorVariants variants;
    if (!config_.enabled) {
        return variants;
    }
    const auto shifts = augmentationShifts(config_);
    variants.reserve(shifts.size());
    const auto sampled = voxelize(cloud_local, config_.downsample);
    for (const auto& shift : shifts) {
        variants.push_back(makeSampledDescriptor(*sampled, shift));
    }
    return variants;
}

/// 构造并保存关键帧描述子，更新内容修订号。
void ScanContextDescriptor::addKeyframe(const Cloud& cloud_local) {
    descriptors_.push_back(makeDescriptorVariants(cloud_local));
    revision_ = nextRevision();
}

/// 校验关键帧索引和位置；Scan Context 编码不依赖地图位置。
void ScanContextDescriptor::updateKeyframePosition(const int index,
                                                   const Eigen::Vector3d& position) {
    if (index < 0 || static_cast<std::size_t>(index) >= descriptors_.size())
        throw std::out_of_range("Scan Context keyframe index is out of range");
    if (!position.allFinite())
        throw std::invalid_argument("Scan Context keyframe position must be finite");
}

/// 构造满足时间资格及指定索引范围的检索集合。
std::vector<int> ScanContextDescriptor::candidateIndexSet(
    const std::vector<int>& allowed_indices) const {
    if (!allowed_indices.empty()) {
        return allowed_indices;
    }
    std::vector<int> indices;
    indices.reserve(descriptors_.size());
    for (std::size_t i = 0; i < descriptors_.size(); ++i) {
        indices.push_back(static_cast<int>(i));
    }
    return indices;
}

/// 召回最新关键帧的历史回环候选并排除近期帧。
std::vector<LoopCandidate> ScanContextDescriptor::detectLatestCandidates() const {
    return detectLatestCandidatesWithin(candidateIndexSet({}));
}

/// 在符合资格的历史索引中召回最新关键帧的候选。
std::vector<LoopCandidate> ScanContextDescriptor::detectLatestCandidatesWithin(
    const std::vector<int>& eligible_indices) const {
    std::vector<LoopCandidate> out;
    if (!config_.enabled || eligible_indices.empty() || descriptors_.size() < 2 ||
        descriptors_.back().empty()) {
        return out;
    }

    const int latest_index = static_cast<int>(descriptors_.size()) - 1;
    const int recent_cutoff = std::max(0, latest_index - config_.exclude_recent + 1);
    const Descriptor& query = descriptors_.back().front();

    out.reserve(eligible_indices.size());
    for (const int index : eligible_indices) {
        if (index < 0 || index >= latest_index || index >= recent_cutoff) {
            continue;
        }
        const auto estimate = estimateYawFromDescriptor(query, index);
        if (!estimate.has_value() || estimate->descriptor_distance >= config_.distance_threshold) {
            continue;
        }
        LoopCandidate candidate;
        candidate.index = index;
        candidate.yaw_seed_rad = estimate->yaw_rad;
        candidate.descriptor_distance = estimate->descriptor_distance;
        candidate.descriptor_score = 1.0 - estimate->descriptor_distance;
        out.push_back(candidate);
    }

    std::sort(out.begin(), out.end(), [](const LoopCandidate& lhs, const LoopCandidate& rhs) {
        if (lhs.descriptor_distance == rhs.descriptor_distance) {
            return lhs.index < rhs.index;
        }
        return lhs.descriptor_distance < rhs.descriptor_distance;
    });
    if (static_cast<int>(out.size()) > config_.candidates) {
        out.resize(config_.candidates);
    }
    return out;
}

/// 返回当前保存的数据条目数。
std::size_t ScanContextDescriptor::size() const {
    return descriptors_.size();
}

/// 判断当前索引是否没有条目。
bool ScanContextDescriptor::empty() const {
    return descriptors_.empty();
}

/// 返回描述子检索是否启用。
bool ScanContextDescriptor::enabled() const {
    return config_.enabled;
}

/// 返回当前生效的算法配置。
const ScanContextDescriptorConfig& ScanContextDescriptor::config() const {
    return config_;
}

/// 返回描述子是否使用前端横滚和俯仰整平点云。
bool ScanContextDescriptor::levelWithOdomRollPitch() const {
    return config_.level_with_odom_roll_pitch;
}

/// 按有效列计算两个高度描述子的距离。
std::pair<double, int> ScanContextDescriptor::descriptorDistance(
    const Descriptor& query, const Descriptor& candidate) const {
    const int sectors = config_.num_sectors;
    const int rings = config_.num_rings;
    if (query.image.rows() != rings || candidate.image.rows() != rings ||
        query.image.cols() != sectors || candidate.image.cols() != sectors) {
        return {std::numeric_limits<double>::max(), 0};
    }

    const int init = bestSectorAlignment(query.sector_key, candidate.sector_key);
    const int radius = std::max(
        0, static_cast<int>(std::ceil(0.5 * config_.search_ratio * static_cast<double>(sectors))));
    Eigen::VectorXd query_norm(sectors), candidate_norm(sectors);
    for (int sector = 0; sector < sectors; ++sector) {
        double q = 0, c = 0;
        for (int ring = 0; ring < rings; ++ring) {
            const double x = query.image(ring, sector), y = candidate.image(ring, sector);
            q += x * x;
            c += y * y;
        }
        query_norm[sector] = std::sqrt(q);
        candidate_norm[sector] = std::sqrt(c);
    }
    // 宽航向搜索批量计算点积；窄搜索及超大配置仅复用范数，限制临时内存。
    const bool batch = 2 * radius + 1 >= sectors / 2 && sectors <= 1024;
    Eigen::MatrixXd dots;
    if (batch)
        dots.noalias() = query.image.cast<double>().transpose() * candidate.image.cast<double>();
    /// 按指定循环列移比较有效列，精算模式保留原始浮点内积顺序。
    const auto similarity = [&](int shift, bool exact) {
        double sum = 0;
        int engaged = 0;
        for (int sector = 0; sector < sectors; ++sector) {
            const int q = (sector + shift) % sectors;
            if (query_norm[q] <= 0 || candidate_norm[sector] <= 0)
                continue;
            double dot = 0;
            if (exact) {
                for (int ring = 0; ring < rings; ++ring)
                    dot +=
                        static_cast<double>(query.image(ring, q)) * candidate.image(ring, sector);
            } else
                dot = dots(q, sector);
            sum += dot / (query_norm[q] * candidate_norm[sector]);
            ++engaged;
        }
        return engaged ? sum / engaged : 0.0;
    };
    /// 将搜索偏移映射到以粗航向为中心的环形列索引。
    const auto shiftAt = [&](int offset) { return (init + offset + sectors) % sectors; };
    double best_sim = -std::numeric_limits<double>::infinity();
    int best_shift = init;
    std::vector<double> scores;
    double approximate_best = best_sim;
    if (batch) {
        for (int offset = -radius; offset <= radius; ++offset) {
            scores.push_back(similarity(shiftAt(offset), false));
            approximate_best = std::max(approximate_best, scores.back());
        }
    }
    // 高对称和近并列候选用原累加顺序复核，不能让矩阵乘法的舍入改变 yaw 选择。
    const double tie_bound = 64 * std::numeric_limits<double>::epsilon() * (rings + sectors);
    for (int offset = -radius; offset <= radius; ++offset) {
        if (batch && scores[offset + radius] < approximate_best - tie_bound)
            continue;
        const int shift = shiftAt(offset);
        const double sim = similarity(shift, true);
        if (sim > best_sim) {
            best_sim = sim;
            best_shift = shift;
        }
    }
    return {1.0 - best_sim, best_shift};
}

/// 从最佳循环列对齐估计源到目标的有符号偏航角。
std::optional<ScanContextYawEstimate> ScanContextDescriptor::estimateYawFromDescriptor(
    const Descriptor& query, const int keyframe_index) const {
    if (!config_.enabled || keyframe_index < 0 ||
        keyframe_index >= static_cast<int>(descriptors_.size())) {
        return std::nullopt;
    }
    const DescriptorVariants& variants = descriptors_[static_cast<std::size_t>(keyframe_index)];
    if (variants.empty()) {
        return std::nullopt;
    }

    ScanContextYawEstimate best;
    for (int i = 0; i < static_cast<int>(variants.size()); ++i) {
        const auto [distance, shift] = descriptorDistance(query, variants[i]);
        if (!std::isfinite(distance) || distance >= best.descriptor_distance) {
            continue;
        }
        best.descriptor_distance = distance;
        best.yaw_shift = shift;
        best.variant_index = i;
        // descriptorDistance 平移 query 列以对齐存储的 target；正向列移表示
        // query 点云自身正向旋转，因此几何 source(query)->target
        // 变换取相反偏航。
        best.yaw_rad = normalizeAngle(-static_cast<double>(shift) * 2.0 * kPi /
                                      static_cast<double>(config_.num_sectors));
    }

    if (best.variant_index < 0) {
        return std::nullopt;
    }
    return best;
}

/// 查询两帧或外部点云之间的有符号偏航初值。
std::optional<ScanContextYawEstimate> ScanContextDescriptor::estimateYaw(
    const Cloud& query_cloud, const int keyframe_index) const {
    if (query_cloud.empty()) {
        return std::nullopt;
    }
    const Descriptor query = makeDescriptor(query_cloud, Eigen::Vector2d::Zero());
    return estimateYawFromDescriptor(query, keyframe_index);
}

/// 在指定存储索引中检索外部点云并按距离与编号排序。
std::vector<LoopCandidate> ScanContextDescriptor::queryAgainstStoredCandidates(
    const Cloud& query_cloud, const std::vector<int>& allowed_indices,
    const int max_results) const {
    std::vector<LoopCandidate> out;
    if (!config_.enabled || query_cloud.empty() || descriptors_.empty()) {
        return out;
    }

    const Descriptor query = makeDescriptor(query_cloud, Eigen::Vector2d::Zero());
    const std::vector<int> indices = candidateIndexSet(allowed_indices);

    out.reserve(indices.size());
    for (const int index : indices) {
        const auto estimate = estimateYawFromDescriptor(query, index);
        if (!estimate.has_value()) {
            continue;
        }
        LoopCandidate candidate;
        candidate.index = index;
        candidate.descriptor_distance = estimate->descriptor_distance;
        candidate.descriptor_score = 1.0 - estimate->descriptor_distance;
        candidate.yaw_seed_rad = estimate->yaw_rad;
        out.push_back(candidate);
    }

    std::sort(out.begin(), out.end(), [](const LoopCandidate& lhs, const LoopCandidate& rhs) {
        if (lhs.descriptor_distance == rhs.descriptor_distance) {
            return lhs.index < rhs.index;
        }
        return lhs.descriptor_distance < rhs.descriptor_distance;
    });
    if (max_results > 0 && static_cast<int>(out.size()) > max_results) {
        out.resize(max_results);
    }
    return out;
}

/// 召回外部查询点云的恢复候选，保留描述子自身排序。
std::vector<LoopCandidate> ScanContextDescriptor::queryLocalizationCandidates(
    const Cloud& query_cloud, const std::vector<int>& allowed_indices,
    const int max_results) const {
    return queryAgainstStoredCandidates(query_cloud, allowed_indices, max_results);
}

}  // namespace backend
