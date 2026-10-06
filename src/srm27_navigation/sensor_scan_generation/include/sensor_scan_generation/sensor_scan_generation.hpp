// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SENSOR_SCAN_GENERATION__SENSOR_SCAN_GENERATION_HPP_
#define SENSOR_SCAN_GENERATION__SENSOR_SCAN_GENERATION_HPP_

#include <memory>
#include <string>

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace sensor_scan_generation
{

class SensorScanGenerationNode : public rclcpp::Node
{
public:
  explicit SensorScanGenerationNode(const rclcpp::NodeOptions & options);

private:
  void laserCloudAndOdometryHandler(
    const nav_msgs::msg::Odometry::ConstSharedPtr & odometry,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & laserCloud2);

  /// \brief 查询 TF 得到 target <- source 的变换。
  ///
  /// 与旧实现的区别（方案 §2.3 修正项 4）：**不使用“返回单位变换”的静默回退**。
  /// 精确时刻查询失败时先尝试最新可用变换（Gazebo 的传感器消息会略早于
  /// robot_state_publisher），并把该情况通过 `_used_latest` 明确暴露给调用方；
  /// 仍然失败则返回 false，调用方必须放弃本次输出，而不是发布一个错误的位姿。
  bool getTransform(
    const std::string & target_frame, const std::string & source_frame, const rclcpp::Time & time,
    tf2::Transform & transform, bool & used_latest, std::string & error);

  void publishTransform(
    const tf2::Transform & transform, const std::string & parent_frame,
    const std::string & child_frame, const rclcpp::Time & stamp);

  /// \brief 发布底盘里程计。
  ///
  /// 语义（方案 §2.3）：
  ///  * `pose` 在 `header.frame_id`（父系）下；`twist` 表达在 `child_frame_id` 下，
  ///    因此父系差分速度必须显式旋转到车体系后再写入，不能直接把父系分量塞进
  ///    `twist.twist.linear`。
  ///  * 差分分母使用**消息采样时间差**；`steady_clock` 只用于看门狗与耗时统计，
  ///    仿真倍速/暂停/回放时两者不能混用。
  ///  * 重复时间戳、时间倒退、过大间隔与异常位姿跳变都不产生速度尖峰：前者沿用
  ///    上一次有效速度，后两者清零并标记不可用。
  void publishOdometry(
    const tf2::Transform & transform, const std::string & parent_frame,
    const std::string & child_frame, const rclcpp::Time & stamp);

  std::string lidar_frame_;
  std::string base_frame_;
  std::string robot_base_frame_;
  bool publish_tf_;
  /// \brief 差分速度可用的最大采样间隔（s）。
  double max_sample_gap_{0.20};
  /// \brief 单次采样允许的最大平移跳变（m）。
  double max_position_jump_{0.75};
  /// \brief 单次采样允许的最大航向跳变（rad）。
  double max_yaw_jump_{1.20};
  /// \brief 上一次 TF 查询是否退化为“最新变换”。
  bool last_transform_used_latest_{false};

  std::unique_ptr<tf2_ros::TransformBroadcaster> br_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_laser_cloud_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_chassis_odometry_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  message_filters::Subscriber<nav_msgs::msg::Odometry> odometry_sub_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> laser_cloud_sub_;

  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    nav_msgs::msg::Odometry, sensor_msgs::msg::PointCloud2>;
  std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  tf2::Transform tf_lidar_to_robot_base_;

  // 位姿差分状态：分别记录“上次发布出去的位姿”和“上次的采样时间戳”。
  tf2::Transform previous_odometry_transform_;
  rclcpp::Time previous_odometry_stamp_{0, 0, RCL_ROS_TIME};
  bool has_previous_odometry_{false};
  double last_valid_linear_x_{0.0};
  double last_valid_linear_y_{0.0};
  double last_valid_angular_z_{0.0};
  /// \brief 上一次里程计发布是否带着有效速度。
  bool last_odometry_velocity_valid_{false};
};

}  // namespace sensor_scan_generation

#endif  // SENSOR_SCAN_GENERATION__SENSOR_SCAN_GENERATION_HPP_
