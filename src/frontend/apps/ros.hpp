/**
 * @file ros.hpp
 * @brief 声明 ROS 点云解码和里程计消息构造接口。
 */
#pragma once

#include <builtin_interfaces/msg/time.hpp>
#include <cstddef>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <string>
#include <vector>

#include "zlio/types.hpp"

namespace zlio::ros {

/// 保存点云逐点时间格式与输入容量。
struct PointCloudFormat {
    std::string time_field = "offset_time";
    double time_scale = 1e-9;
    bool absolute_time = false;
    std::size_t maximum_points = 1000000;
};

/// 保存里程计输出的父坐标系和机体坐标系名称。
struct OutputFrames {
    std::string world = "odom", body = "imu";
};

/// 按明确字段格式解码 PointCloud2 回波。
class PointCloudDecoder {
public:
    /// 设置时间字段、单位和容量。
    explicit PointCloudDecoder(PointCloudFormat format);
    /// 将点云消息转换为具有获取时间的原始回波。
    Scan decode(const sensor_msgs::msg::PointCloud2& cloud) const;

private:
    PointCloudFormat format_;
};

/// 将 ROS 时间转换为整数纳秒。
Time messageTime(const builtin_interfaces::msg::Time& time);

/// 将整数纳秒规范为 ROS 时间，秒字段越界时拒绝。
builtin_interfaces::msg::Time makeTimeMessage(Time time);

/// 根据完整状态和物理协方差接口构造里程计消息。
nav_msgs::msg::Odometry makeOdometryMessage(const Estimate& estimate, const Vec3& angular_velocity,
                                            const OutputFrames& frames, double gyro_variance);

/// 根据已去畸变的机体系点构造 XYZ 点云，沿用后验的时间戳。
sensor_msgs::msg::PointCloud2 makePointCloudMessage(const std::vector<Vec3>& points,
                                                    const std_msgs::msg::Header& header,
                                                    const std::string& body_frame);

}  // namespace zlio::ros
