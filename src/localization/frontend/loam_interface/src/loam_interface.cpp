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

#include "loam_interface/loam_interface.hpp"

#include <cmath>

#include "pcl_ros/transforms.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace loam_interface
{

namespace
{

tf2::Transform planarizeBasePose(const tf2::Transform & transform)
{
  tf2::Transform planar = tf2::Transform::getIdentity();
  const auto & origin = transform.getOrigin();
  planar.setOrigin(tf2::Vector3(origin.x(), origin.y(), 0.0));
  tf2::Quaternion rotation;
  rotation.setRPY(0.0, 0.0, tf2::getYaw(transform.getRotation()));
  planar.setRotation(rotation);
  return planar;
}

}  // namespace

LoamInterfaceNode::LoamInterfaceNode(const rclcpp::NodeOptions & options)
: Node("loam_interface", options)
{
  this->declare_parameter<std::string>("state_estimation_topic", "");
  this->declare_parameter<std::string>("registered_scan_topic", "");
  this->declare_parameter<std::string>("odom_frame", "odom");
  this->declare_parameter<std::string>("base_frame", "");
  this->declare_parameter<std::string>("lidar_frame", "");
  // 方案 §4.1（2026-11-08 云台系）：base_frame 改成 gimbal_yaw 之后，odom 的原点会跟着
  // 抬到云台平面（gimbal_yaw 在 base_link 上方 0.268 m），地面的 odom z 变成 -0.268。
  // 而代价地图/体素层的 min_obstacle_height / max_obstacle_height / origin_z 都是**绝对 z**，
  // 抬高之后车身底部（轮子、低矮台阶）会被当成"低于 min_obstacle_height"而丢掉。
  // ground_frame 给一个**原点在地面**的系（实车填 base_link），本节点在拍快照时顺手把
  // base_frame 相对它的 z 反向补偿掉，odom z=0 仍在地面，下游阈值一个都不用改。
  // 留空 = 不补偿（仿真/旧链行为不变）。
  this->declare_parameter<std::string>("ground_frame", "");
  this->declare_parameter<bool>("planarize_pose", false);

  this->get_parameter("state_estimation_topic", state_estimation_topic_);
  this->get_parameter("registered_scan_topic", registered_scan_topic_);
  this->get_parameter("odom_frame", odom_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("lidar_frame", lidar_frame_);
  this->get_parameter("ground_frame", ground_frame_);
  this->get_parameter("planarize_pose", planarize_pose_);

  base_frame_to_lidar_initialized_ = false;

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  pcd_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("registered_scan", 5);
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("lidar_odometry", 5);

  if (planarize_pose_) {
    synchronized_odom_sub_.subscribe(this, state_estimation_topic_, rmw_qos_profile_sensor_data);
    synchronized_pcd_sub_.subscribe(this, registered_scan_topic_, rmw_qos_profile_sensor_data);
    sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(20), synchronized_odom_sub_, synchronized_pcd_sub_);
    sync_->registerCallback(std::bind(
      [this](
        const nav_msgs::msg::Odometry::ConstSharedPtr & odometry,
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud) {
        odometryCallback(odometry);
        pointCloudCallback(cloud);
      },
      std::placeholders::_1, std::placeholders::_2));
  } else {
    pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      registered_scan_topic_, 5,
      std::bind(&LoamInterfaceNode::pointCloudCallback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      state_estimation_topic_, 5,
      std::bind(&LoamInterfaceNode::odometryCallback, this, std::placeholders::_1));
  }
}

void LoamInterfaceNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  // Point-LIO has already transformed cloud_registered from the lidar into
  // its camera_init world frame. Only align that fixed world frame with odom;
  // applying the current lidar pose again would double the robot motion.
  if (!base_frame_to_lidar_initialized_) {
    return;
  }
  // The registered cloud is expressed in Point-LIO's `camera_init` world frame.
  // Project it through the SAME planar odom->lidar pose used for
  // odom -> base_footprint, instead of the static lidar extrinsic, so the map
  // does not keep Point-LIO's unobservable z/roll/pitch drift.  Using the
  // full (non-planar) pose here is what previously left the map tilted/flying
  // while the base footprint was planarized.
  tf2::Transform tf_odom_to_camera_init = tf_odom_to_lidar_odom_;
  if (planarize_pose_ && planar_pose_available_) {
    // p_odom = T_odom<-camera_init * p_camera_init
    //        = T_odom<-lidar(planar) * inverse(T_camera_init<-lidar)
    tf_odom_to_camera_init = tf_odom_to_lidar_planar_ * tf_camera_init_to_lidar_.inverse();
  }
  auto out = std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl_ros::transformPointCloud(odom_frame_, tf_odom_to_camera_init, *msg, *out);
  pcd_pub_->publish(*out);
}

void LoamInterfaceNode::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  // NOTE: Input odometry message is based on the `lidar_odom`
  // Here we transform it to the `odom` frame
  if (!base_frame_to_lidar_initialized_) {
    try {
      auto tf_stamped = tf_buffer_->lookupTransform(
        base_frame_, lidar_frame_, msg->header.stamp, rclcpp::Duration::from_seconds(0.5));
      tf2::Transform tf_base_frame_to_lidar;
      tf2::fromMsg(tf_stamped.transform, tf_base_frame_to_lidar);
      tf2::Transform tf_ground_offset = tf2::Transform::getIdentity();
      if (!ground_frame_.empty()) {
        // T(base_frame <- ground_frame) 的 z 是"地面在 base_frame 下方多少"，取负即
        // "base_frame 在地面上方多少"。把它加到 odom 的 z 上，odom z=0 就仍在**地面**。
        const auto ground_stamped = tf_buffer_->lookupTransform(
          base_frame_, ground_frame_, msg->header.stamp, rclcpp::Duration::from_seconds(0.5));
        const double dz = -ground_stamped.transform.translation.z;
        tf_ground_offset.setOrigin(tf2::Vector3(0.0, 0.0, dz));
        RCLCPP_INFO(
          this->get_logger(),
          "odom z 按 ground_frame=%s 拉回地面: base_frame=%s 离地 %.4f m（补偿 %+.4f m）",
          ground_frame_.c_str(), base_frame_.c_str(), dz, dz);
      }
      // ⚠ 补偿必须作用在**快照**上：点云与位姿都走同一个 tf_odom_to_lidar_odom_，
      // 两者一起平移，几何关系不变，但 odom 的 z 原点回到地面。
      tf_odom_to_lidar_odom_ = tf_ground_offset * tf_base_frame_to_lidar;
      base_frame_to_lidar_initialized_ = true;
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s Retrying...", ex.what());
      return;
    }
  }

  // Transform the odometry_msg (based lidar_odom) to the odom frame
  tf2::Transform tf_lidar_odom_to_lidar;
  tf2::fromMsg(msg->pose.pose, tf_lidar_odom_to_lidar);
  tf2::Transform tf_odom_to_lidar = tf_odom_to_lidar_odom_ * tf_lidar_odom_to_lidar;

  // In 2D simulation, reject Point-LIO's unobservable z/roll/pitch drift while
  // preserving the physical lidar mounting angle in the reconstructed pose.
  if (planarize_pose_) {
    const tf2::Transform tf_odom_to_base = tf_odom_to_lidar * tf_odom_to_lidar_odom_.inverse();
    tf_odom_to_lidar = planarizeBasePose(tf_odom_to_base) * tf_odom_to_lidar_odom_;
  }

  // Cache the latest Point-LIO state so the registered cloud can be projected
  // through the same planar pose (see pointCloudCallback).
  tf_camera_init_to_lidar_ = tf_lidar_odom_to_lidar;
  tf_odom_to_lidar_planar_ = tf_odom_to_lidar;
  planar_pose_available_ = true;

  nav_msgs::msg::Odometry out;
  out.header.stamp = msg->header.stamp;
  out.header.frame_id = odom_frame_;
  out.child_frame_id = lidar_frame_;

  const auto & origin = tf_odom_to_lidar.getOrigin();
  out.pose.pose.position.x = origin.x();
  out.pose.pose.position.y = origin.y();
  out.pose.pose.position.z = origin.z();
  out.pose.pose.orientation = tf2::toMsg(tf_odom_to_lidar.getRotation());

  odom_pub_->publish(out);
}

}  // namespace loam_interface

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(loam_interface::LoamInterfaceNode)
