/**
 * @file target_cache.cpp
 * @brief 按地图版本和子图参数缓存恢复目标点云及其几何准备结果。
 */

#include "backend/target_cache.hpp"

#include "backend/logger.hpp"
#include "backend/sampling.hpp"

namespace backend {
/// 按地图版本和子图配置取得或准备恢复目标租约。
RelocalizationPreparedTarget RelocalizationTargetCache::getOrPrepare(int center_index,
                                                                     int max_keyframes,
                                                                     double voxel_resolution,
                                                                     const KeyframeMap& keyframes,
                                                                     double max_radius) {
    if (center_index < 0 || !std::isfinite(voxel_resolution) || !std::isfinite(max_radius))
        return {};
    max_keyframes = std::max(1, max_keyframes);
    // 修改版本后绝不返回旧视图。持续变化的地图交由下一次维护周期重试。
    for (int attempt = 0; attempt < 3; ++attempt) {
        const auto revision = keyframes.revision();
        const Key key{revision, center_index, max_keyframes, voxel_resolution, max_radius};
        /// 缓存未命中时构建空间子图，并在版本稳定后执行体素采样。
        const auto target = cache_.get(key, [&]() -> Cloud::ConstPtr {
            Cloud::Ptr raw;
            try {
                raw = keyframes.buildSpatialSubmap(center_index, max_keyframes, max_radius);
            } catch (const std::runtime_error& error) {
                BACKEND_ERROR_STREAM("prior submap unavailable: " << error.what());
                return {};  // 磁盘后备失效属于不可用观测，不提交残缺子图。
            }
            if (keyframes.revision() != revision)
                return {};
            return voxelize(*raw, voxel_resolution);
        });
        if (keyframes.revision() == revision)
            return {target, revision};
    }
    return {};
}
}  // namespace backend
