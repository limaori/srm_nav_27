/**
 * @file ros.hpp
 * @brief 转换后端值类型与 ROS 时间、位姿及点云消息。
 */
#pragma once

#include <Eigen/Geometry>
#include <builtin_interfaces/msg/time.hpp>
#include <cstdint>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>

#include "backend/cloud.hpp"
#include "backend/time.hpp"

namespace backend::ros {

/// 校验后端输入时间戳的非负秒数和规范纳秒范围。
bool validStamp(const builtin_interfaces::msg::Time& time);

/// 在 Eigen 四元数归一化前校验完整位姿边界，避免零模或非有限四元数引入 NaN。
bool hasUsableOdometryPose(const nav_msgs::msg::Odometry& odom);

/// 将整数纳秒原样转为领域时间戳；时间基准由调用方保持一致。
Timestamp toTimestamp(const rclcpp::Time& time);

/// 将已验收位姿转换为 child_frame_id → header.frame_id 的齐次变换。
/// 调用方先通过 hasUsableOdometryPose 完成位置和四元数检查。
Eigen::Matrix4d odomToMatrix(const nav_msgs::msg::Odometry& odom);

/// 将 frame_id 系中的齐次位姿转换为消息，并沿用调用方提供的观测时刻。
geometry_msgs::msg::PoseStamped matrixToPoseStamped(const Eigen::Matrix4d& pose,
                                                    const std::string& frame_id,
                                                    const builtin_interfaces::msg::Time& stamp);

/// 秒值取整到最近纳秒后生成时间消息；非有限值返回零，负纳秒按借位规整。
builtin_interfaces::msg::Time stampFromSec(double sec);

/// 将 ROS 时间的整数纳秒原样转换为时间消息。
builtin_interfaces::msg::Time timeToMsg(const rclcpp::Time& time);

/// 将内部点云编码为 PointCloud2；输入点须已在 frame_id 系中表达。
sensor_msgs::msg::PointCloud2 toRosCloud(const Cloud& cloud, const std::string& frame_id,
                                         const builtin_interfaces::msg::Time& stamp);

/// 将 child_frame → parent_frame 的齐次变换编码为 TF 消息，输出四元数归一化。
geometry_msgs::msg::TransformStamped toRosTransform(const Eigen::Matrix4d& transform,
                                                    const std::string& parent_frame,
                                                    const std::string& child_frame,
                                                    const builtin_interfaces::msg::Time& stamp);

/// ROS Vector3 → Eigen 向量。
Eigen::Vector3d vectorFromRos(const geometry_msgs::msg::Vector3& vector);

}  // namespace backend::ros
