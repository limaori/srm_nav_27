/**
 * @file geometry.hpp
 * @brief 位姿几何工具：点云刚体变换、重力对齐（水平化）等纯数学操作。
 */
#pragma once

#include <pcl/common/transforms.h>

#include <Eigen/Dense>

#include "backend/cloud.hpp"
#include "backend/math.hpp"

namespace backend {

/// 用齐次变换矩阵对点云做刚体变换，返回新点云。
inline Cloud::Ptr transformCloud(const Cloud& cloud, const Eigen::Matrix4d& transform) {
    Cloud::Ptr out(new Cloud);
    pcl::transformPointCloud(cloud, *out, transform.cast<float>());
    return out;
}

/// 计算 sensor/body 到水平坐标系的变换。全局描述子要求 Z 轴朝上；函数从
/// world_from_body 提取 yaw，并返回 T_level_body = R_yaw^T * R_wb，消除
/// roll/pitch 而保留 yaw。
inline Eigen::Matrix4d levelFromBodyTransform(const Eigen::Matrix4d& world_from_body) {
    const Eigen::Matrix3d R_wb = world_from_body.block<3, 3>(0, 0);
    const double yaw = yawFromRotation(R_wb);
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    Eigen::Matrix3d R_yaw;
    R_yaw << c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0;

    Eigen::Matrix4d T_level_body = Eigen::Matrix4d::Identity();
    T_level_body.block<3, 3>(0, 0) = R_yaw.transpose() * R_wb;
    return T_level_body;
}

/// 把整平描述子给出的有符号 yaw 转换为扫描匹配使用的原始 source-cloud 到
/// target-cloud 变换；平移已在原始 target 坐标系表达，保持不变。
inline Eigen::Matrix4d descriptorYawToRawSourceTarget(const double source_to_target_yaw,
                                                      const Eigen::Vector3d& raw_target_translation,
                                                      const Eigen::Matrix4d& source_world_pose,
                                                      const Eigen::Matrix4d& target_world_pose,
                                                      const bool clouds_were_leveled) {
    Eigen::Matrix4d guess = yawGuess(source_to_target_yaw, raw_target_translation);
    if (!clouds_were_leveled) {
        return guess;
    }
    const Eigen::Matrix3d R_level_source =
        levelFromBodyTransform(source_world_pose).block<3, 3>(0, 0);
    const Eigen::Matrix3d R_level_target =
        levelFromBodyTransform(target_world_pose).block<3, 3>(0, 0);
    guess.block<3, 3>(0, 0) = R_level_target.transpose() * guess.block<3, 3>(0, 0) * R_level_source;
    return guess;
}

}  // namespace backend
