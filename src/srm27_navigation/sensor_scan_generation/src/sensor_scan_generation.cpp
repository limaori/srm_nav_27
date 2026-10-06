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

#include "sensor_scan_generation/sensor_scan_generation.hpp"

#include <cmath>

#include "pcl_ros/transforms.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace sensor_scan_generation
{

namespace
{

constexpr double kPositionJitterThreshold = 0.002;
constexpr double kYawJitterThreshold = 0.002;

double normalizeAngle(double angle) { return std::atan2(std::sin(angle), std::cos(angle)); }

// Nav2's base_footprint is a planar frame.  Point-LIO estimates a full 6DoF
// pose for the tilted lidar, but passing its z/roll/pitch noise to the planar
// robot frame makes the robot model bob relative to a 2D map.
tf2::Transform planarize(const tf2::Transform & transform)
{
  tf2::Transform planar = tf2::Transform::getIdentity();
  const auto & origin = transform.getOrigin();
  planar.setOrigin(tf2::Vector3(origin.x(), origin.y(), 0.0));
  tf2::Quaternion rotation;
  rotation.setRPY(0.0, 0.0, tf2::getYaw(transform.getRotation()));
  planar.setRotation(rotation);
  return planar;
}

// Point-LIO still has millimetre-level XY and yaw noise after planarization.
// Keep that noise from moving the RViz model while the robot is stationary,
// while allowing any meaningful motion to pass through unchanged.
tf2::Transform suppressJitter(const tf2::Transform & transform, bool robot_frame)
{
  static tf2::Transform last_chassis_transform;
  static tf2::Transform last_robot_transform;
  static bool chassis_initialized = false;
  static bool robot_initialized = false;
  auto & last_transform = robot_frame ? last_robot_transform : last_chassis_transform;
  auto & initialized = robot_frame ? robot_initialized : chassis_initialized;

  if (!initialized) {
    last_transform = transform;
    initialized = true;
    return transform;
  }

  const auto & origin = transform.getOrigin();
  const auto & last_origin = last_transform.getOrigin();
  const double dx = origin.x() - last_origin.x();
  const double dy = origin.y() - last_origin.y();
  const double yaw_delta = normalizeAngle(
    tf2::getYaw(transform.getRotation()) - tf2::getYaw(last_transform.getRotation()));

  if (std::hypot(dx, dy) < kPositionJitterThreshold && std::abs(yaw_delta) < kYawJitterThreshold) {
    return last_transform;
  }

  last_transform = transform;
  return transform;
}

}  // namespace

SensorScanGenerationNode::SensorScanGenerationNode(const rclcpp::NodeOptions & options)
: Node("sensor_scan_generation", options)
{
  this->declare_parameter<std::string>("lidar_frame", "");
  this->declare_parameter<std::string>("base_frame", "");
  this->declare_parameter<std::string>("robot_base_frame", "");
  this->declare_parameter<bool>("publish_tf", true);
  this->declare_parameter<double>("max_sample_gap", 0.20);
  this->declare_parameter<double>("max_position_jump", 0.75);
  this->declare_parameter<double>("max_yaw_jump", 1.20);

  this->get_parameter("lidar_frame", lidar_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("publish_tf", publish_tf_);
  this->get_parameter("max_sample_gap", max_sample_gap_);
  this->get_parameter("max_position_jump", max_position_jump_);
  this->get_parameter("max_yaw_jump", max_yaw_jump_);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  br_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  pub_laser_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sensor_scan", 2);
  pub_chassis_odometry_ = this->create_publisher<nav_msgs::msg::Odometry>("odometry", 2);

  rmw_qos_profile_t qos_profile = {
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    1,
    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false};

  odometry_sub_.subscribe(this, "lidar_odometry", qos_profile);
  laser_cloud_sub_.subscribe(this, "registered_scan", qos_profile);

  sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(100), odometry_sub_, laser_cloud_sub_);
  sync_->registerCallback(std::bind(
    &SensorScanGenerationNode::laserCloudAndOdometryHandler, this, std::placeholders::_1,
    std::placeholders::_2));
}

void SensorScanGenerationNode::laserCloudAndOdometryHandler(
  const nav_msgs::msg::Odometry::ConstSharedPtr & odometry_msg,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pcd_msg)
{
  tf2::Transform tf_lidar_to_chassis;
  tf2::Transform tf_odom_to_chassis;
  tf2::Transform tf_odom_to_robot_base;
  tf2::Transform tf_odom_to_lidar;

  tf2::fromMsg(odometry_msg->pose.pose, tf_odom_to_lidar);

  // TF 查询失败时明确放弃本次输出：既不用单位变换伪装成有效定位，也不发布错误的
  // odom->base_link，避免规划器把失效状态继续用于控制（方案 §2.3 修正项 4）。
  bool used_latest = false;
  std::string tf_error;
  if (!getTransform(
        lidar_frame_, robot_base_frame_, pcd_msg->header.stamp, tf_lidar_to_robot_base_,
        used_latest, tf_error)) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "跳过本次里程计/TF 输出：%s <- %s 查询失败 (%s)", lidar_frame_.c_str(),
      robot_base_frame_.c_str(), tf_error.c_str());
    last_transform_used_latest_ = false;
    return;
  }
  bool chassis_used_latest = false;
  if (!getTransform(
        lidar_frame_, base_frame_, pcd_msg->header.stamp, tf_lidar_to_chassis,
        chassis_used_latest, tf_error)) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "跳过本次里程计/TF 输出：%s <- %s 查询失败 (%s)", lidar_frame_.c_str(), base_frame_.c_str(),
      tf_error.c_str());
    last_transform_used_latest_ = false;
    return;
  }
  last_transform_used_latest_ = used_latest || chassis_used_latest;

  tf_odom_to_chassis = tf_odom_to_lidar * tf_lidar_to_chassis;
  tf_odom_to_robot_base = tf_odom_to_lidar * tf_lidar_to_robot_base_;

  const auto chassis_transform = suppressJitter(planarize(tf_odom_to_chassis), false);
  const auto robot_base_transform = suppressJitter(planarize(tf_odom_to_robot_base), true);

  if (publish_tf_) {
    publishTransform(
      chassis_transform, odometry_msg->header.frame_id, base_frame_, pcd_msg->header.stamp);
  }
  publishOdometry(
    robot_base_transform, odometry_msg->header.frame_id, robot_base_frame_, pcd_msg->header.stamp);

  // Use the same filtered planar pose as the TF published above.  Transforming
  // the cloud with the raw Point-LIO pose while publishing a filtered base TF
  // makes a static scene appear to slide by a few millimetres in RViz.
  sensor_msgs::msg::PointCloud2 out;
  // Ground-truth simulation and loam_interface both publish registered_scan
  // directly in odom.  Applying the lidar<-odom transform again would move
  // every point twice and appears as a drifting/flying map in RViz.  Raw
  // lidar-frame clouds still use the normal conversion path.
  if (
    pcd_msg->header.frame_id == lidar_frame_ ||
    pcd_msg->header.frame_id == odometry_msg->header.frame_id ||
    pcd_msg->header.frame_id == "odom") {
    // Gazebo point clouds are already expressed in the lidar frame.  Point-LIO
    // and loam_interface clouds are expressed in odom.  In both cases the
    // message is ready for RViz; applying the inverse pose again would move the
    // cloud twice and make it appear to fly or drift.
    out = *pcd_msg;
  } else {
    const auto filtered_odom_to_lidar = chassis_transform * tf_lidar_to_chassis.inverse();
    pcl_ros::transformPointCloud(lidar_frame_, filtered_odom_to_lidar.inverse(), *pcd_msg, out);
  }
  pub_laser_cloud_->publish(out);
}

bool SensorScanGenerationNode::getTransform(
  const std::string & target_frame, const std::string & source_frame, const rclcpp::Time & time,
  tf2::Transform & transform, bool & used_latest, std::string & error)
{
  used_latest = false;
  error.clear();
  try {
    auto transform_stamped = tf_buffer_->lookupTransform(
      target_frame, source_frame, time, rclcpp::Duration::from_seconds(0.5));
    tf2::fromMsg(transform_stamped.transform, transform);
    return true;
  } catch (tf2::TransformException & ex) {
    // Gazebo 的传感器消息会略早于 robot_state_publisher，精确时刻查询可能因为
    // “向未来外推”而失败。此时退化为最新可用变换是**显式**的降级，不是静默回退：
    // used_latest 会向上暴露，且失败时不再返回单位变换。
    try {
      auto transform_stamped = tf_buffer_->lookupTransform(
        target_frame, source_frame, tf2::TimePointZero, tf2::durationFromSec(0.5));
      tf2::fromMsg(transform_stamped.transform, transform);
      used_latest = true;
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "TF at %.3f unavailable (%s); using the latest transform for %s <- %s", time.seconds(),
        ex.what(), target_frame.c_str(), source_frame.c_str());
      return true;
    } catch (tf2::TransformException & latest_ex) {
      error = std::string(ex.what()) + "; latest lookup also failed: " + latest_ex.what();
      return false;
    }
  }
}

void SensorScanGenerationNode::publishTransform(
  const tf2::Transform & transform, const std::string & parent_frame,
  const std::string & child_frame, const rclcpp::Time & stamp)
{
  geometry_msgs::msg::TransformStamped transform_msg;
  transform_msg.header.stamp = stamp;
  transform_msg.header.frame_id = parent_frame;
  transform_msg.child_frame_id = child_frame;
  transform_msg.transform = tf2::toMsg(transform);
  br_->sendTransform(transform_msg);
}

void SensorScanGenerationNode::publishOdometry(
  const tf2::Transform & transform, const std::string & parent_frame,
  const std::string & child_frame, const rclcpp::Time & stamp)
{
  nav_msgs::msg::Odometry out;
  out.header.stamp = stamp;
  out.header.frame_id = parent_frame;
  out.child_frame_id = child_frame;

  const auto & origin = transform.getOrigin();
  out.pose.pose.position.x = origin.x();
  out.pose.pose.position.y = origin.y();
  out.pose.pose.position.z = origin.z();
  out.pose.pose.orientation = tf2::toMsg(transform.getRotation());

  // 差分分母使用消息采样时间差（不是回调到达时间差），因此仿真倍速/暂停/回放时
  // 速度不会因为“到达节奏”变化而产生错误量级（方案 §2.3 修正项 1）。
  const double dt =
    has_previous_odometry_ ? (stamp - previous_odometry_stamp_).seconds() : 0.0;

  bool velocity_valid = false;
  if (has_previous_odometry_ && dt > 1.0e-6) {
    const auto delta = transform.getOrigin() - previous_odometry_transform_.getOrigin();
    const double previous_yaw = tf2::getYaw(previous_odometry_transform_.getRotation());
    const double current_yaw = tf2::getYaw(transform.getRotation());
    const double dyaw = normalizeAngle(current_yaw - previous_yaw);
    const double translation = std::hypot(delta.x(), delta.y());

    if (dt > max_sample_gap_) {
      // 间隔过大（暂停、丢包、回放跳段）：不给速度，避免把长间隔噪声放大成尖峰。
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "里程计采样间隔 %.3f s 超过 max_sample_gap %.3f s，本次不输出差分速度", dt,
        max_sample_gap_);
    } else if (translation > max_position_jump_ || std::abs(dyaw) > max_yaw_jump_) {
      // 位姿跳变（重定位/坐标系跳变）：清零并标记不可用，让下游明确看到失效。
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "里程计位姿跳变 (%.3f m, %.3f rad)，本次速度置零", translation, dyaw);
    } else {
      // twist 表达在 child_frame_id 下：父系差分速度必须旋转到车体系。
      // 二维平面刚体的关系为 v_child = R(yaw)^T * (dp/dt)_parent，旋转不改变角速度。
      const double c = std::cos(previous_yaw);
      const double s = std::sin(previous_yaw);
      const double vx_parent = delta.x() / dt;
      const double vy_parent = delta.y() / dt;
      last_valid_linear_x_ = c * vx_parent + s * vy_parent;
      last_valid_linear_y_ = -s * vx_parent + c * vy_parent;
      last_valid_angular_z_ = dyaw / dt;
      velocity_valid = true;
    }
  }

  if (has_previous_odometry_ && !velocity_valid) {
    // 重复时间戳/时间倒退/跳变/长间隔：沿用上一次有效速度，而不是清零造成
    // “速度瞬间归零”的假象，也不是把异常差分当速度输出。
    last_odometry_velocity_valid_ = false;
  } else {
    last_odometry_velocity_valid_ = velocity_valid;
  }
  out.twist.twist.linear.x = last_valid_linear_x_;
  out.twist.twist.linear.y = last_valid_linear_y_;
  out.twist.twist.linear.z = 0.0;
  out.twist.twist.angular.x = 0.0;
  out.twist.twist.angular.y = 0.0;
  out.twist.twist.angular.z = last_valid_angular_z_;

  if (stamp > previous_odometry_stamp_ || !has_previous_odometry_) {
    previous_odometry_transform_ = transform;
    previous_odometry_stamp_ = stamp;
    has_previous_odometry_ = true;
  }

  pub_chassis_odometry_->publish(out);
}

}  // namespace sensor_scan_generation

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(sensor_scan_generation::SensorScanGenerationNode)
