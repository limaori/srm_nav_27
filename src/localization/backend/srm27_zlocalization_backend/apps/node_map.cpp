/**
 * @file node_map.cpp
 * @brief 加载不可变先验地图并通过地图包接口保存运行轨迹。
 */

#include <vector>

#include "apps/node.hpp"
#include "backend/directory.hpp"
#include "backend/map_bundle.hpp"

namespace backend::ros {

/// 加载并核对先验关键帧与地点描述子。
bool LocalizationNode::loadPriorMap(const std::string& directory) {
    if (directory.empty() ||
        !prior_map_.load(directory, parameters_.runtime.storage.cloud_cache_bytes) ||
        prior_map_.empty()) {
        RCLCPP_ERROR_STREAM(get_logger(), "failed to load prior keyframes from " << directory);
        return false;
    }

    if (!descriptor_->enabled()) {
        RCLCPP_WARN(get_logger(),
                    "place descriptor is disabled; prior tracking remains available "
                    "but arbitrary-point recovery requires fresh UWB");
        return true;
    }
    if (!descriptor_->load(directory)) {
        RCLCPP_ERROR_STREAM(get_logger(), "failed to load " << descriptor_->name()
                                                            << " descriptors from " << directory);
        // 具体差异由 storage.cpp 打在 stderr（"saved SC config does not match ..."），
        // 这里补一条可操作的提示：最常见的原因是建包参数与 backend.yaml 不一致。
        RCLCPP_ERROR_STREAM(get_logger(),
                            "  -> 核对建包时的 --sc-* 与 scan_context.* 是否逐位一致"
                            "（尤其中继/建包的 --sc-lidar-height 必须等于 scan_context.lidar_height，"
                            "URDF 值 0.346388 -> 统一取 0.3464）；不一致就用相同参数重建地图包。");
        return false;
    }
    const std::vector<Keyframe> keyframes = prior_map_.snapshot();
    if (descriptor_->size() != keyframes.size()) {
        RCLCPP_ERROR_STREAM(get_logger(), "prior descriptor/keyframe count mismatch: "
                                              << descriptor_->size() << " vs " << keyframes.size());
        return false;
    }
    for (const Keyframe& keyframe : keyframes) {
        descriptor_->updateKeyframePosition(keyframe.index,
                                            keyframe.optimized_pose.block<3, 1>(0, 3));
    }
    RCLCPP_INFO_STREAM(get_logger(), "loaded immutable prior: keyframes=" << keyframes.size()
                                                                          << ", descriptor="
                                                                          << descriptor_->name());
    return true;
}

/// 按统一地图包接口保存当前轨迹快照。
bool LocalizationNode::saveTrajectoryMap(const std::string& directory) {
    if (directory.empty() || parameters_.runtime.prior_map_enabled)
        return false;
    if (pathsReferToSameLocation(directory, parameters_.maps.prior_directory)) {
        RCLCPP_ERROR(get_logger(), "runtime save directory aliases the immutable prior map");
        return false;
    }
    return saveMapBundle(directory, trajectory_, *descriptor_,
                         {parameters_.maps.save_global_pcd, parameters_.maps.runtime_voxel});
}
}  // namespace backend::ros
