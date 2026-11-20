/**
 * @file map_bundle.hpp
 * @brief 从一致轨迹快照构建描述子、全局点云并提交地图目录。
 */
#pragma once

#include <string>
#include <vector>

namespace backend {
struct Keyframe;
class KeyframeMap;
class PlaceDescriptor;

/// 地图包派生点云的保存选项。
struct BundleOptions {
    bool global_cloud = true;
    double voxel_resolution = 0.3;
};
/// 按同一关键帧快照重建并写出描述子。
bool saveMapDescriptor(const std::string& directory, const std::vector<Keyframe>& frames,
                       PlaceDescriptor& descriptor);
/// 按具体描述子的配置构造实例并写出地图描述子。
template <class Descriptor, class Config>
bool saveMapDescriptor(const std::string& directory, const std::vector<Keyframe>& frames,
                       const Config& config) {
    Descriptor descriptor;
    descriptor.configure(config);
    return saveMapDescriptor(directory, frames, descriptor);
}
/// 将同一关键帧快照导出为全局点云。
bool saveMapCloud(const std::string& path, const std::vector<Keyframe>& frames, double voxel);
/// 以一致轨迹快照生成地图包，并通过目录事务替换目标。
bool saveMapBundle(const std::string& directory, const KeyframeMap& trajectory,
                   const PlaceDescriptor& descriptor, const BundleOptions& options);
}  // namespace backend
