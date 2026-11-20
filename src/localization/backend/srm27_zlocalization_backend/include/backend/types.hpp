/**
 * @file types.hpp
 * @brief 定义后端模块共享的点云、前端帧、关键帧和召回候选类型。
 */
#pragma once

#include <Eigen/Dense>
#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

#include "backend/cloud.hpp"
#include "backend/covariance.hpp"

namespace backend {

/// 前端帧：前端 LIO 输出的一帧数据。
struct FrontendFrame {
    /// 前端 reset 代次；不同 epoch 的帧不得共享里程计边。
    std::uint64_t epoch = 0;
    double timestamp = 0.0;
    Eigen::Matrix4d odom_pose = Eigen::Matrix4d::Identity();  // T_odom_sensor
    /// ROS 顺序 [x,y,z,rotX,rotY,rotZ] 的位姿边缘协方差。
    PoseCovariance6d pose_covariance = PoseCovariance6d::Zero();
    bool pose_covariance_valid = false;
    Cloud::Ptr cloud_local{new Cloud};  // sensor 坐标系下的点云
};

/// 关键帧：从前端帧中选出的稀疏帧，参与回环和位姿图优化。
struct Keyframe {
    int index = -1;
    std::uint64_t epoch = 0;
    double timestamp = 0.0;
    Eigen::Matrix4d odom_pose = Eigen::Matrix4d::Identity();       // 原始 T_odom_sensor
    Eigen::Matrix4d optimized_pose = Eigen::Matrix4d::Identity();  // 图优化后的 T_map_sensor
    CloudBlock cloud;  // sensor 坐标系不可变几何；元数据快照不加载点云。
};

/// 回环/重定位候选：全局描述子检索到的一组可能匹配。
struct LoopCandidate {
    int index = -1;  // 匹配关键帧索引
    // 有符号 source-cloud 到 target-cloud
    // yaw；描述子边界负责一次性转换原生列移约定。
    std::optional<double> yaw_seed_rad;
    double descriptor_distance = std::numeric_limits<double>::max();  // 描述子距离，越小越好
    double descriptor_score = 0.0;  // 描述子验证分数，越大越好
};

}  // namespace backend
