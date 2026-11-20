/**
 * @file descriptor.cpp
 * @brief 实现 G-PROBE 描述子的公开生命周期和候选接口。
 */

#include <algorithm>
#include <numeric>
#include <stdexcept>
#include <utility>

#include "backend/gprobe.hpp"
#include "gprobe.hpp"

namespace backend {

using namespace gprobe_detail;

/// 分配 G-PROBE 描述子索引的独占实现。
GProbeDescriptor::GProbeDescriptor() : impl_(std::make_unique<Impl>()) {
}
/// 释放 G-PROBE 描述子索引及其存储。
GProbeDescriptor::~GProbeDescriptor() = default;
/// 转移 G-PROBE 描述子索引的独占所有权。
GProbeDescriptor::GProbeDescriptor(GProbeDescriptor&&) noexcept = default;
/// 转移描述子索引的独占所有权。
GProbeDescriptor& GProbeDescriptor::operator=(GProbeDescriptor&&) noexcept = default;

/// 规范化描述子几何与检索配置，并推进内容修订号。
void GProbeDescriptor::configure(const GProbeDescriptorConfig& config) {
    impl_->revision = nextRevision();
    impl_->config = config;
    impl_->config.num_rings = std::clamp(config.num_rings, 1, 512);
    impl_->config.num_sectors = std::clamp(config.num_sectors, 4, 720);
    impl_->config.max_range = std::max(1.0, config.max_range);
    impl_->config.voxel_size = std::max(0.0, config.voxel_size);
    impl_->config.sigma_t = std::max(0.0, config.sigma_t);
    impl_->config.query_fov_deg = std::clamp(config.query_fov_deg, 1.0, 360.0);
    impl_->config.query_fov_center_deg = wrapDegrees(config.query_fov_center_deg);
    impl_->config.branch_top_k = std::max(1, config.branch_top_k);
    impl_->config.verification_top_k = std::max(1, config.verification_top_k);
    impl_->config.spatial_nms_radius = std::max(0.0, config.spatial_nms_radius);
    impl_->config.heading_window_deg = std::clamp(config.heading_window_deg, 0.0, 180.0);
    impl_->config.column_trim_ratio = std::clamp(config.column_trim_ratio, 0.0, 0.95);
    impl_->config.sgrt_kappa = std::max(0.0, config.sgrt_kappa);
    impl_->config.exclude_recent = std::max(1, config.exclude_recent);
    impl_->config.candidates = std::max(1, config.candidates);
}

/// 返回当前生效的算法配置。
const GProbeDescriptorConfig& GProbeDescriptor::config() const {
    return impl_->config;
}

/// 创建保留当前配置且没有关键帧的描述子实例。
std::unique_ptr<PlaceDescriptor> GProbeDescriptor::createEmpty() const {
    auto descriptor = std::make_unique<GProbeDescriptor>();
    descriptor->configure(impl_->config);
    return descriptor;
}

/// 返回当前内容修订号，用于检索缓存身份判断。
std::uint64_t GProbeDescriptor::revision() const {
    return impl_->revision;
}

/// 清空关键帧描述子并推进内容修订号。
void GProbeDescriptor::clear() {
    impl_->frames.clear();
    impl_->revision = nextRevision();
}

/// 构造并保存关键帧描述子，更新内容修订号。
void GProbeDescriptor::addKeyframe(const Cloud& cloud_local) {
    addKeyframe(cloud_local, Eigen::Vector3d::Zero());
}

/// 构造并保存关键帧描述子，更新内容修订号。
void GProbeDescriptor::addKeyframe(const Cloud& cloud_local, const Eigen::Vector3d& position) {
    if (!impl_->config.enabled) {
        return;
    }
    impl_->frames.push_back(impl_->makeFrame(cloud_local, position));
    impl_->revision = nextRevision();
}

/// 更新检索所用的地图位置并推进内容修订号。
void GProbeDescriptor::updateKeyframePosition(const int index, const Eigen::Vector3d& position) {
    if (index < 0 || static_cast<std::size_t>(index) >= impl_->frames.size())
        throw std::out_of_range("G-PROBE keyframe index is out of range");
    if (!position.allFinite())
        throw std::invalid_argument("G-PROBE keyframe position must be finite");
    auto& previous = impl_->frames[static_cast<std::size_t>(index)].position;
    if ((previous.array() != position.array()).any())
        impl_->revision = nextRevision();
    previous = position;
}

/// 召回最新关键帧的历史回环候选并排除近期帧。
std::vector<LoopCandidate> GProbeDescriptor::detectLatestCandidates() const {
    if (!impl_->config.enabled || impl_->frames.size() <= 1U) {
        return {};
    }
    const int latest_index = static_cast<int>(impl_->frames.size()) - 1;
    const int recent_cutoff = std::max(0, latest_index - impl_->config.exclude_recent + 1);
    std::vector<int> eligible(static_cast<std::size_t>(recent_cutoff));
    std::iota(eligible.begin(), eligible.end(), 0);
    return detectLatestCandidatesWithin(eligible);
}

/// 在符合资格的历史索引中召回最新关键帧的候选。
std::vector<LoopCandidate> GProbeDescriptor::detectLatestCandidatesWithin(
    const std::vector<int>& eligible_indices) const {
    if (!impl_->config.enabled || impl_->frames.size() <= 1U || eligible_indices.empty()) {
        return {};
    }
    const int latest_index = static_cast<int>(impl_->frames.size()) - 1;
    const int recent_cutoff = std::max(0, latest_index - impl_->config.exclude_recent + 1);
    std::vector<int> allowed;
    allowed.reserve(eligible_indices.size());
    for (const int index : eligible_indices) {
        if (index >= 0 && index < latest_index && index < recent_cutoff) {
            allowed.push_back(index);
        }
    }
    std::sort(allowed.begin(), allowed.end());
    allowed.erase(std::unique(allowed.begin(), allowed.end()), allowed.end());
    if (allowed.empty()) {
        return {};
    }
    const auto scored = impl_->scoreQuery(impl_->frames.back().query_branch, allowed);
    std::vector<LoopCandidate> out;
    const std::size_t count =
        std::min(scored.size(), static_cast<std::size_t>(impl_->config.candidates));
    out.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
        const ScoredCandidate& candidate = scored[i];
        LoopCandidate hit;
        hit.index = candidate.index;
        hit.yaw_seed_rad.reset();
        hit.descriptor_score = candidate.score;
        hit.descriptor_distance = 1.0 - candidate.score;
        out.push_back(hit);
    }
    return out;
}

/// 召回外部查询点云的恢复候选，保留描述子自身排序。
std::vector<LoopCandidate> GProbeDescriptor::queryLocalizationCandidates(
    const Cloud& query_cloud, const std::vector<int>& allowed_indices,
    const int max_results) const {
    if (!impl_->config.enabled || impl_->frames.empty() || query_cloud.empty()) {
        return {};
    }
    const Branch query = makeBranch(query_cloud, impl_->queryMask(), impl_->config);
    const auto scored = impl_->scoreQuery(query, allowed_indices);
    const int requested = max_results > 0 ? max_results : impl_->config.candidates;
    const std::size_t count =
        std::min(scored.size(), static_cast<std::size_t>(std::max(1, requested)));
    std::vector<LoopCandidate> out;
    out.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
        const ScoredCandidate& candidate = scored[i];
        LoopCandidate hit;
        hit.index = candidate.index;
        hit.yaw_seed_rad.reset();
        hit.descriptor_score = candidate.score;
        hit.descriptor_distance = 1.0 - candidate.score;
        out.push_back(hit);
    }
    return out;
}

/// 返回当前保存的数据条目数。
std::size_t GProbeDescriptor::size() const {
    return impl_->frames.size();
}
/// 判断当前索引是否没有条目。
bool GProbeDescriptor::empty() const {
    return impl_->frames.empty();
}
/// 返回描述子检索是否启用。
bool GProbeDescriptor::enabled() const {
    return impl_->config.enabled;
}
/// 返回描述子是否使用前端横滚和俯仰整平点云。
bool GProbeDescriptor::levelWithOdomRollPitch() const {
    return impl_->config.level_with_odom_roll_pitch;
}

/// 将当前描述子集合写入版本化二进制文件。
bool GProbeDescriptor::save(const std::string& directory) const {
    return saveBundle(directory, impl_->config, impl_->frames);
}

/// 从版本化二进制文件加载与当前几何配置兼容的描述子。
bool GProbeDescriptor::load(const std::string& directory) {
    if (!loadBundle(directory, impl_->config, &impl_->frames))
        return false;
    impl_->revision = nextRevision();
    return true;
}

/// 仅读取持久化描述子的配置，不加载帧数据。
bool GProbeDescriptor::loadConfig(const std::string& directory, GProbeDescriptorConfig* config) {
    return loadBundleConfig(directory, config);
}

}  // namespace backend
