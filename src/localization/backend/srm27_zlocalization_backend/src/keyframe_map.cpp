/**
 * @file keyframe_map.cpp
 * @brief 管理关键帧、空间索引和全局地图视图。
 */

#include "backend/keyframe_map.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <limits>
#include <nanoflann.hpp>
#include <stdexcept>
#include <unordered_map>

#include "backend/cloud_storage.hpp"
#include "backend/geometry.hpp"
#include "backend/sampling.hpp"

namespace backend {
/// 清空运行关键帧及空间索引，并从零开始新轨迹编号。
void KeyframeMap::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    keyframes_.clear();
    first_index_ = 0;
    next_index_ = 0;
    storage_.reset();
    changedLocked();
}

/// 捕获轨迹快照并构建地图坐标系下的全局点云。
Cloud::Ptr KeyframeMap::buildOptimizedMap(double resolution) const {
    return buildOptimizedMapFromSnapshot(snapshot(), resolution);
}

/// 逐块投影几何并按指定体素质心规则生成确定排序的全局点云。
Cloud::Ptr KeyframeMap::buildOptimizedMapFromSnapshot(const std::vector<Keyframe>& frames,
                                                      double resolution) {
    if (!std::isfinite(resolution))
        throw std::invalid_argument("non-finite map voxel resolution");
    const float inverse_leaf = resolution > 0.0 ? 1.0F / static_cast<float>(resolution) : 0.0F;
    if (!std::isfinite(inverse_leaf))
        throw std::invalid_argument("map voxel resolution too small");
    Cloud::Ptr result(new Cloud);
    struct Moment {
        Eigen::Vector4d sum = Eigen::Vector4d::Zero();
        std::size_t count = 0;
    };
    // 体素按 Z/Y/X 整数坐标排序，以双精度累计并输出单精度 XYZ/I 质心。
    using Key = std::array<std::int64_t, 3>;
    struct Hash {
        /// 组合三轴整数体素坐标的哈希值。
        std::size_t operator()(const Key& key) const {
            std::size_t hash = 0;
            for (auto value : key)
                hash ^=
                    std::hash<std::int64_t>{}(value) + 0x9e3779b9U + (hash << 6U) + (hash >> 2U);
            return hash;
        }
    };
    std::unordered_map<Key, Moment, Hash> voxels;
    for (const auto& frame : frames) {
        if (!frame.optimized_pose.allFinite())
            continue;
        const auto local = frame.cloud.load();
        if (!local || local->empty())
            continue;
        const auto world = transformCloud(*local, frame.optimized_pose);
        if (resolution <= 0.0) {
            *result += *world;
            continue;
        }
        for (const auto& p : *world) {
            if (!pointIsFinite(p))
                continue;
            // 与 PCL VoxelGrid 使用相同的 float 乘法确定体素归属。
            const Eigen::Vector3d cell =
                (Eigen::Vector3f(p.x, p.y, p.z) * inverse_leaf).cast<double>();
            if (cell.cwiseAbs().maxCoeff() >=
                static_cast<double>(std::numeric_limits<std::int64_t>::max()))
                throw std::runtime_error("map voxel index overflow");
            const std::array<std::int64_t, 3> key{static_cast<std::int64_t>(std::floor(cell.z())),
                                                  static_cast<std::int64_t>(std::floor(cell.y())),
                                                  static_cast<std::int64_t>(std::floor(cell.x()))};
            auto& moment = voxels[key];
            moment.sum += Eigen::Vector4d(p.x, p.y, p.z, p.intensity);
            ++moment.count;
        }
    }
    result->reserve(result->size() + voxels.size());
    std::vector<const decltype(voxels)::value_type*> ordered;
    ordered.reserve(voxels.size());
    for (const auto& item : voxels)
        ordered.push_back(&item);
    std::sort(ordered.begin(), ordered.end(), [](auto a, auto b) { return a->first < b->first; });
    for (const auto* item : ordered) {
        const auto mean = (item->second.sum / item->second.count).eval();
        PointType p;
        p.x = mean.x();
        p.y = mean.y();
        p.z = mean.z();
        p.intensity = mean.w();
        result->push_back(p);
    }
    return result;
}
namespace {
std::atomic<std::uint64_t> next_revision{1};
}

/// 返回同一轨迹锁内捕获的全部优化位置。
std::vector<Eigen::Vector3d> KeyframeMap::positions() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<Eigen::Vector3d> result;
    result.reserve(keyframes_.size());
    for (const auto& keyframe : keyframes_)
        result.push_back(keyframe.optimized_pose.block<3, 1>(0, 3));
    return result;
}

struct KeyframeMap::SpatialIndex {
    struct View {
        const std::vector<Keyframe>& frames;
        std::vector<std::size_t> offsets;
        /// 向空间索引提供有限关键帧位置的数量。
        std::size_t kdtree_get_point_count() const { return offsets.size(); }
        /// 向空间索引提供指定关键帧位置的单轴坐标。
        double kdtree_get_pt(std::size_t i, std::size_t axis) const {
            return frames[offsets[i]].optimized_pose(axis, 3);
        }
        /// 通知空间索引自行计算包围盒。
        template <class Box>
        bool kdtree_get_bbox(Box&) const {
            return false;
        }
    };
    using Tree = nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<double, View>,
                                                     View, 3, std::size_t>;
    std::vector<Keyframe> frames;
    std::uint64_t revision;
    View view;
    std::unique_ptr<Tree> tree;
    /// 冻结关键帧与版本，并为有限优化位置建立只读空间索引。
    SpatialIndex(std::vector<Keyframe> input, std::uint64_t version)
        : frames(std::move(input)), revision(version), view{frames, {}} {
        for (std::size_t i = 0; i < frames.size(); ++i)
            if (frames[i].optimized_pose.block<3, 1>(0, 3).allFinite())
                view.offsets.push_back(i);
        if (!view.offsets.empty())
            tree = std::make_unique<Tree>(3, view, nanoflann::KDTreeSingleIndexAdaptorParams(10));
    }
    /// 检索满足数量及半径限制的近邻，并按真实距离和索引稳定排序。
    std::vector<std::size_t> nearest(const Eigen::Vector3d& p, std::size_t count,
                                     double radius) const {
        if (!tree || !p.allFinite())
            return {};
        double bound = radius > 0 ? radius * radius : std::numeric_limits<double>::infinity();
        // 先用最近邻查询收紧搜索界限，再保留边界上的等距候选。
        if (count > 0 && count < view.offsets.size()) {
            std::vector<std::size_t> indices(count);
            std::vector<double> distances(count);
            const auto found = tree->knnSearch(p.data(), count, indices.data(), distances.data());
            if (found)
                bound = std::min(bound, distances[found - 1]);
        }
        std::vector<std::pair<std::size_t, double>> hits;
        const double margin = std::isfinite(bound) ? 64 * std::numeric_limits<double>::epsilon() *
                                                         std::max(1.0, bound)
                                                   : 0;
        tree->radiusSearch(p.data(), bound + margin, hits, nanoflann::SearchParams());
        std::vector<std::pair<double, std::size_t>> exact;
        const double radius2 =
            radius > 0 ? radius * radius : std::numeric_limits<double>::infinity();
        for (const auto& hit : hits) {
            const auto offset = view.offsets[hit.first];
            const double d = (frames[offset].optimized_pose.block<3, 1>(0, 3) - p).squaredNorm();
            if (std::isfinite(d) && d <= radius2)
                exact.emplace_back(d, offset);
        }
        // 以原始双精度距离和关键帧编号确定稳定顺序。
        std::sort(exact.begin(), exact.end(), [&](const auto& a, const auto& b) {
            return a.first != b.first ? a.first < b.first
                                      : frames[a.second].index < frames[b.second].index;
        });
        if (count > 0 && exact.size() > count)
            exact.resize(count);
        std::vector<std::size_t> result;
        result.reserve(exact.size());
        for (const auto& hit : exact)
            result.push_back(hit.second);
        return result;
    }
};

/// 创建具有独立全局版本身份的空关键帧序列。
KeyframeMap::KeyframeMap() : revision_(next_revision.fetch_add(1)) {
}
/// 在持锁状态推进轨迹版本并使旧空间索引失效。
void KeyframeMap::changedLocked() {
    revision_ = next_revision.fetch_add(1);
    spatial_index_.reset();
}
/// 返回当前关键帧元数据及优化位姿的版本身份。
std::uint64_t KeyframeMap::revision() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return revision_;
}
/// 捕获磁盘后备所有权后，在轨迹锁外读取其缓存统计。
CloudCacheStats KeyframeMap::storageStats() const {
    std::shared_ptr<CloudStorage> storage;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        storage = storage_;
    }
    return storage ? storage->stats() : CloudCacheStats{};
}
/// 在锁外构建位置索引，并仅在轨迹版本未变时缓存结果。
std::shared_ptr<const KeyframeMap::SpatialIndex> KeyframeMap::spatialIndex() const {
    std::vector<Keyframe> frames;
    std::uint64_t revision;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (spatial_index_)
            return spatial_index_;
        frames = keyframes_;
        revision = revision_;
    }
    auto index = std::make_shared<SpatialIndex>(std::move(frames), revision);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (revision_ == revision)
            spatial_index_ = index;
    }
    return index;
}

/// 查询当前关键帧序列是否为空。
bool KeyframeMap::empty() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return keyframes_.empty();
}

/// 返回当前保留的关键帧记录数量。
std::size_t KeyframeMap::size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return keyframes_.size();
}

/// 首帧和新代次直接建帧；同代次满足最小间隔后由平移、转角或最大间隔触发。
bool KeyframeMap::shouldCreateKeyframe(const FrontendFrame& frame,
                                       const KeyframePolicy& policy) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (keyframes_.empty()) {
        return true;
    }
    if (frame.epoch != keyframes_.back().epoch) {
        return true;
    }

    const double dt = frame.timestamp - keyframes_.back().timestamp;
    if (policy.min_interval > 0.0 && dt < policy.min_interval) {
        return false;
    }

    const Eigen::Vector3d last_t = keyframes_.back().odom_pose.block<3, 1>(0, 3);
    const Eigen::Vector3d curr_t = frame.odom_pose.block<3, 1>(0, 3);
    if (policy.distance > 0.0 && (curr_t - last_t).norm() >= policy.distance) {
        return true;
    }

    const Eigen::Matrix3d relative_rot = keyframes_.back().odom_pose.block<3, 3>(0, 0).transpose() *
                                         frame.odom_pose.block<3, 3>(0, 0);
    const double cos_angle = std::max(-1.0, std::min(1.0, (relative_rot.trace() - 1.0) * 0.5));
    if (policy.angle_rad > 0.0 && std::acos(cos_angle) >= policy.angle_rad) {
        return true;
    }

    return policy.max_interval > 0.0 && dt >= policy.max_interval;
}

/// 追加共享输入几何的关键帧并推进轨迹和空间索引版本。
Keyframe KeyframeMap::add(const FrontendFrame& frame) {
    std::lock_guard<std::mutex> lock(mutex_);
    Keyframe keyframe;
    keyframe.index = next_index_++;
    if (keyframes_.empty()) {
        first_index_ = keyframe.index;
    }
    keyframe.epoch = frame.epoch;
    keyframe.timestamp = frame.timestamp;
    keyframe.odom_pose = frame.odom_pose;
    keyframe.optimized_pose = frame.odom_pose;
    // FrontendFrame 点云在摄入后不可变，关键帧共享其所有权而不重复复制数据。
    keyframe.cloud = CloudBlock(frame.cloud_local);
    keyframes_.push_back(keyframe);
    changedLocked();
    return keyframe;
}

/// 在轨迹锁内复制关键帧元数据及几何租约句柄。
std::vector<Keyframe> KeyframeMap::snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return keyframes_;
}

/// 按单调图索引复制当前仍保留的关键帧，越界或空目标返回失败。
bool KeyframeMap::get(const int index, Keyframe* out) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (out == nullptr || index < first_index_ || index >= next_index_) {
        return false;
    }
    const std::size_t offset = static_cast<std::size_t>(index - first_index_);
    if (offset >= keyframes_.size() || keyframes_[offset].index != index) {
        return false;
    }
    *out = keyframes_[offset];
    return true;
}

/// 返回最新关键帧副本；调用方须确认序列非空。
Keyframe KeyframeMap::latest() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (keyframes_.empty())
        throw std::logic_error("cannot read the latest keyframe of an empty map");
    return keyframes_.back();
}

/// 返回下次追加使用的单调图索引。
int KeyframeMap::nextIndex() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return next_index_;
}

/// 裁剪旧记录并保留最新指定数量的关键帧，不复用图索引。
void KeyframeMap::retainLatest(const std::size_t max_count) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (keyframes_.size() <= max_count) {
        return;
    }
    changedLocked();
    if (max_count == 0U) {
        keyframes_.clear();
        first_index_ = next_index_;
        return;
    }
    keyframes_.erase(keyframes_.begin(), keyframes_.end() - static_cast<std::ptrdiff_t>(max_count));
    first_index_ = keyframes_.front().index;
}

/// 更新仍被保留的有限图位姿，并在实际变化时推进版本。
void KeyframeMap::updateOptimizedPoses(const std::vector<std::pair<int, Eigen::Matrix4d>>& poses) {
    std::lock_guard<std::mutex> lock(mutex_);
    bool changed = false;
    for (const auto& [index, pose] : poses) {
        if (index < first_index_ || index >= next_index_ || !pose.allFinite()) {
            continue;
        }
        const std::size_t offset = static_cast<std::size_t>(index - first_index_);
        if (offset < keyframes_.size() && keyframes_[offset].index == index) {
            changed = changed || !keyframes_[offset].optimized_pose.isApprox(pose, 0.0);
            keyframes_[offset].optimized_pose = pose;
        }
    }
    if (changed)
        changedLocked();
}

/// 在冻结位置索引中查询近邻并返回对应的图关键帧索引。
std::vector<int> KeyframeMap::nearestKeyframeIndices(const Eigen::Vector3d& position,
                                                     std::size_t count, double radius) const {
    const auto index = spatialIndex();
    const auto selected = index->nearest(position, count, radius);
    std::vector<int> result;
    result.reserve(selected.size());
    for (auto offset : selected)
        result.push_back(index->frames[offset].index);
    return result;
}

/// 选择空间邻居并在状态锁外加载几何，拼接到中心传感器坐标系。
Cloud::Ptr KeyframeMap::buildSpatialSubmap(int center_index, std::size_t count,
                                           double radius) const {
    const auto index = spatialIndex();
    const auto center =
        std::lower_bound(index->frames.begin(), index->frames.end(), center_index,
                         [](const Keyframe& frame, int value) { return frame.index < value; });
    if (center == index->frames.end() || center->index != center_index)
        return Cloud::Ptr(new Cloud);
    const auto center_offset = static_cast<std::size_t>(center - index->frames.begin());
    auto selected = index->nearest(center->optimized_pose.block<3, 1>(0, 3),
                                   std::max<std::size_t>(1, count), radius);
    if (std::find(selected.begin(), selected.end(), center_offset) == selected.end()) {
        if (selected.size() >= std::max<std::size_t>(1, count))
            selected.pop_back();
        selected.push_back(center_offset);
    }
    const Eigen::Matrix4d inverse = center->optimized_pose.inverse();
    Cloud::Ptr cloud(new Cloud);
    std::size_t points = 0;
    for (auto offset : selected)
        points += index->frames[offset].cloud.pointCount();
    cloud->reserve(points);
    for (auto offset : selected) {
        const auto& frame = index->frames[offset];
        const auto local = frame.cloud.load();
        if (local && !local->empty())
            *cloud += *transformCloud(*local, inverse * frame.optimized_pose);
    }
    return cloud;
}
}  // namespace backend
