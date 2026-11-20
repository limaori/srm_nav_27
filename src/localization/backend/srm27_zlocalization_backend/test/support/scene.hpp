/**
 * @file scene.hpp
 * @brief 构造非对称房间及已知位姿下的局部扫描。
 */
#pragma once

#include "backend/types.hpp"

namespace backend::testing {
/// 构造给定平移和偏航的地图位姿。
inline Eigen::Matrix4d scenePose(double x, double y, double z, double yaw) {
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    pose.block<3, 3>(0, 0) = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    pose.block<3, 1>(0, 3) = Eigen::Vector3d(x, y, z);
    return pose;
}

/// 以六个不同距离的平面构造具有三维约束的房间。
inline Cloud::Ptr roomCloud() {
    Cloud::Ptr cloud(new Cloud);
    for (int i = -5; i <= 5; ++i)
        for (int j = -5; j <= 5; ++j) {
            const float u = .4F * i, v = .4F * j;
            for (const Eigen::Vector3f& xyz :
                 {Eigen::Vector3f(-3, u, v), Eigen::Vector3f(5, u, v), Eigen::Vector3f(u, -4, v),
                  Eigen::Vector3f(u, 6, v), Eigen::Vector3f(u, v, -2), Eigen::Vector3f(u, v, 3)}) {
                PointType point;
                point.getVector3fMap() = xyz;
                point.intensity = static_cast<float>(cloud->size());
                cloud->push_back(point);
            }
        }
    return cloud;
}

/// 按真实位姿生成局部点云，并附加独立指定的里程计位姿。
inline FrontendFrame sceneFrame(const Cloud& world, double time,
                                const Eigen::Matrix4d& T_world_sensor,
                                const Eigen::Matrix4d& T_odom_sensor, std::uint64_t epoch = 0) {
    FrontendFrame frame;
    frame.timestamp = time;
    frame.epoch = epoch;
    frame.odom_pose = T_odom_sensor;
    frame.pose_covariance = PoseCovariance6d::Identity() * .001;
    frame.pose_covariance_valid = true;
    const Eigen::Matrix3d rotation = T_world_sensor.block<3, 3>(0, 0);
    const Eigen::Vector3d position = T_world_sensor.block<3, 1>(0, 3);
    for (const auto& point : world) {
        const Eigen::Vector3d local =
            rotation.transpose() * (point.getVector3fMap().cast<double>() - position);
        PointType copy = point;
        copy.getVector3fMap() = local.cast<float>();
        frame.cloud_local->push_back(copy);
    }
    return frame;
}
}  // namespace backend::testing
