/**
 * @file target_cache.hpp
 * @brief 定义恢复子图租约与分层几何准备缓存。
 */
#pragma once

#include <tuple>

#include "backend/cache.hpp"
#include "backend/keyframe_map.hpp"
#include "backend/registration.hpp"

namespace backend {
struct RelocalizationPreparedTarget {
    Cloud::ConstPtr target;
    std::uint64_t revision = 0;
    /// 返回租约是否持有可用于配准的非空目标。
    bool ready() const { return target && !target->empty(); }
};

/// 与几何块使用同一字节预算缓存；键包含地图版本及全部子图生成参数。
class RelocalizationTargetCache {
public:
    /// 按固定比例分配子图点云与几何准备缓存的总字节预算。
    explicit RelocalizationTargetCache(std::size_t max_entries = 32,
                                       std::size_t max_bytes = 64U * 1024U * 1024U)
        : cache_(max_bytes / 4, max_entries), prepared_(max_bytes - max_bytes / 4, max_entries) {}
    /// 同时清空目标子图租约及细、粗层几何准备缓存。
    void clear() {
        cache_.clear();
        prepared_.clear();
    }
    /// 返回用于共享细、粗层几何准备结果的缓存。
    Registration::TargetCache& preparedTargets() { return prepared_; }
    /// 返回当前缓存占用、命中及逐出统计。
    CloudCacheStats stats() const { return cache_.stats(); }
    /// 按地图版本和子图配置取得或准备恢复目标租约。
    RelocalizationPreparedTarget getOrPrepare(int center_index, int max_keyframes,
                                              double voxel_resolution, const KeyframeMap& keyframes,
                                              double max_radius = 0.0);

private:
    using Key = std::tuple<std::uint64_t, int, int, double, double>;
    CloudCache<Key> cache_;
    Registration::TargetCache prepared_;
};
}  // namespace backend
