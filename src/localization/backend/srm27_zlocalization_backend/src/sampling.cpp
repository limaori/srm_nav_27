/**
 * @file sampling.cpp
 * @brief 点云几何工具：体素下采样、重叠率、非拥有指针包装。
 */

#include "backend/sampling.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <Eigen/Geometry>
#include <cmath>

#include "backend/cloud.hpp"
#include "backend/geometry.hpp"

namespace backend {

/// 检查点坐标与强度是否均为有限值。
bool pointIsFinite(const PointType& point) {
    return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z) &&
           std::isfinite(point.intensity);
}

/// 复制点云；标记为密集时直接复制，否则只保留坐标和强度均有限的点。
Cloud::Ptr finiteCloudCopy(const Cloud& cloud) {
    Cloud::Ptr out(new Cloud);
    if (cloud.is_dense) {
        *out = cloud;
        return out;
    }
    out->reserve(cloud.size());
    for (const PointType& point : cloud) {
        if (pointIsFinite(point)) {
            out->push_back(point);
        }
    }
    out->is_dense = true;
    return out;
}

/// 构造一个不负责释放底层对象的 Cloud::ConstPtr。
/// 用于把栈上或他处拥有的点云传入以 shared_ptr 为接口的 PCL 算法，避免拷贝。
Cloud::ConstPtr nonOwningCloudPtr(const Cloud& cloud) {
    return Cloud::ConstPtr(&cloud, [](const Cloud*) {});
}

/// 对有限输入执行体素采样；非正分辨率或空输入直接返回有限点副本。
Cloud::Ptr voxelize(const Cloud& cloud, const double resolution) {
    Cloud::Ptr out(new Cloud);
    const Cloud::Ptr finite = cloud.is_dense ? Cloud::Ptr() : finiteCloudCopy(cloud);
    const Cloud& input = finite ? *finite : cloud;
    if (input.empty() || resolution <= 0.0) {
        *out = input;
        return out;
    }

    pcl::VoxelGrid<PointType> voxel;
    voxel.setInputCloud(nonOwningCloudPtr(input));
    voxel.setLeafSize(resolution, resolution, resolution);
    voxel.filter(*out);
    return out;
}

/// 重叠率：source 按 source_to_target 变换后，落在 target 的 max_distance
/// 邻域内的点占比。空云或非正距离阈值返回 0。
double overlapRatio(const Cloud& source, const Cloud& target,
                    const Eigen::Matrix4d& source_to_target, const double max_distance) {
    if (source.empty() || target.empty() || max_distance <= 0.0) {
        return 0.0;
    }

    Cloud::Ptr aligned = transformCloud(source, source_to_target);
    pcl::KdTreeFLANN<PointType> tree;
    tree.setInputCloud(nonOwningCloudPtr(target));

    const double max_sq = max_distance * max_distance;
    int overlapping = 0;
    std::vector<int> indices(1);
    std::vector<float> distances(1);
    for (const auto& point : aligned->points) {
        if (tree.nearestKSearch(point, 1, indices, distances) > 0 &&
            static_cast<double>(distances[0]) <= max_sq) {
            ++overlapping;
        }
    }
    return static_cast<double>(overlapping) / static_cast<double>(aligned->size());
}

}  // namespace backend
