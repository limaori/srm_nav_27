/**
 * @file geometry.hpp
 * @brief 构造解析平移、平面点云和前端帧。
 */
#pragma once

#include "backend/types.hpp"

namespace backend::testing {
/// 构造给定平移的刚体位姿。
inline Eigen::Matrix4d translation(double x, double y = 0, double z = 0) {
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    pose.block<3, 1>(0, 3) = Eigen::Vector3d(x, y, z);
    return pose;
}

/// 生成具有固定点身份的均匀水平平面点云。
inline Cloud::Ptr planeCloud() {
    Cloud::Ptr cloud(new Cloud);
    for (int x = 0; x < 12; ++x)
        for (int y = 0; y < 12; ++y) {
            PointType point;
            point.x = 1.F + .1F * x;
            point.y = -.5F + .1F * y;
            point.z = 1.F;
            point.intensity = static_cast<float>(x * 12 + y);
            cloud->push_back(point);
        }
    return cloud;
}

/// 构造携带位姿协方差和平面点云的前端帧。
inline FrontendFrame frame(double time, double x = 0) {
    FrontendFrame result;
    result.timestamp = time;
    result.odom_pose = translation(x);
    result.pose_covariance = PoseCovariance6d::Identity() * .01;
    result.pose_covariance_valid = true;
    result.cloud_local = planeCloud();
    return result;
}
}  // namespace backend::testing
