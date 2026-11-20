/**
 * @file sampling.hpp
 * @brief 定义有限点复制、体素采样与点云重叠率接口。
 */
#pragma once

#include <Eigen/Core>

#include "backend/cloud.hpp"

namespace backend {

/// 检查点坐标与强度是否均为有限值。
bool pointIsFinite(const PointType& point);

/// 复制点云；标记为密集时直接复制，否则只保留坐标和强度均有限的点。
Cloud::Ptr finiteCloudCopy(const Cloud& cloud);

/// 构造不释放底层对象的借用指针；调用方必须保证点云覆盖 PCL 算法的使用期。
Cloud::ConstPtr nonOwningCloudPtr(const Cloud& cloud);

/// 对有限输入执行体素采样；非正分辨率或空输入直接返回有限点副本。
Cloud::Ptr voxelize(const Cloud& cloud, double resolution);

/// 返回变换后的源点落入目标距离邻域的比例；空云或非正距离阈值返回零。
double overlapRatio(const Cloud& source, const Cloud& target,
                    const Eigen::Matrix4d& source_to_target, double max_distance);

}  // namespace backend
