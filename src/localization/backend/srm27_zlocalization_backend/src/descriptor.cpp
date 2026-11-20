/**
 * @file descriptor.cpp
 * @brief 使用在线或持久化整平参考构建关键帧地点描述子。
 */

#include "backend/descriptor.hpp"

#include "backend/geometry.hpp"

namespace backend {
namespace {
/// 依据指定整平参考构造并添加关键帧描述子。
void addDescriptor(PlaceDescriptor& descriptor, const Keyframe& keyframe,
                   const Eigen::Matrix4d& leveling_pose) {
    const auto geometry = keyframe.cloud.load();
    const Cloud empty;
    const Cloud& source = geometry ? *geometry : empty;
    const auto leveled = descriptor.levelWithOdomRollPitch()
                             ? transformCloud(source, levelFromBodyTransform(leveling_pose))
                             : Cloud::Ptr{};
    descriptor.addKeyframe(leveled ? *leveled : source, keyframe.optimized_pose.block<3, 1>(0, 3));
}
}  // namespace

/// 使用前端里程计整平参考添加在线关键帧描述子。
void addKeyframeDescriptor(PlaceDescriptor& descriptor, const Keyframe& keyframe) {
    addDescriptor(descriptor, keyframe, keyframe.odom_pose);
}

/// 使用持久化优化位姿重建完整地图描述子集合。
void rebuildMapDescriptors(PlaceDescriptor& descriptor, const std::vector<Keyframe>& keyframes) {
    descriptor.clear();
    for (const auto& keyframe : keyframes)
        addDescriptor(descriptor, keyframe, keyframe.optimized_pose);
}
}  // namespace backend
