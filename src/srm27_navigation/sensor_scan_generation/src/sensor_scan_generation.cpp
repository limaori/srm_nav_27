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

  this->get_parameter("lidar_frame", lidar_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("publish_tf", publish_tf_);

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
  tf_lidar_to_robot_base_ = getTransform(lidar_frame_, robot_base_frame_, pcd_msg->header.stamp);
  tf_lidar_to_chassis = getTransform(lidar_frame_, base_frame_, pcd_msg->header.stamp);

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

tf2::Transform SensorScanGenerationNode::getTransform(
  const std::string & target_frame, const std::string & source_frame, const rclcpp::Time & time)
{
  try {
    auto transform_stamped = tf_buffer_->lookupTransform(
      target_frame, source_frame, time, rclcpp::Duration::from_seconds(0.5));
    tf2::Transform transform;
    tf2::fromMsg(transform_stamped.transform, transform);
    return transform;
  } catch (tf2::TransformException & ex) {
    // Gazebo publishes sensor messages slightly ahead of robot_state_publisher.
    // A lookup at the exact cloud timestamp can therefore fail with a future
    // extrapolation even though a valid latest transform is available.  Falling
    // back to the latest transform avoids publishing a frame with an identity
    // transform, which makes the point cloud jump/fly in RViz.
    try {
      auto transform_stamped = tf_buffer_->lookupTransform(
        target_frame, source_frame, tf2::TimePointZero, tf2::durationFromSec(0.5));
      tf2::Transform transform;
      tf2::fromMsg(transform_stamped.transform, transform);
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "TF at %.3f unavailable (%s); using latest transform for %s <- %s", time.seconds(),
        ex.what(), target_frame.c_str(), source_frame.c_str());
      return transform;
    } catch (tf2::TransformException & latest_ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "TF lookup failed for %s <- %s (%s); latest lookup also failed (%s). Returning identity.",
        target_frame.c_str(), source_frame.c_str(), ex.what(), latest_ex.what());
      return tf2::Transform::getIdentity();
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
  const tf2::Transform & transform, std::string parent_frame, const std::string & child_frame,
  const rclcpp::Time & stamp)
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

  static tf2::Transform previous_transform;
  static auto previous_time = std::chrono::steady_clock::now();
  static bool previous_initialized = false;
  const auto current_time = std::chrono::steady_clock::now();

  const double dt =
    std::chrono::duration_cast<std::chrono::nanoseconds>(current_time - previous_time).count() *
    1e-9;

  if (previous_initialized && dt > 0) {
    const auto linear_velocity = (transform.getOrigin() - previous_transform.getOrigin()) / dt;

    const tf2::Quaternion q_diff =
      transform.getRotation() * previous_transform.getRotation().inverse();
    const auto angular_velocity = q_diff.getAxis() * q_diff.getAngle() / dt;

    out.twist.twist.linear.x = linear_velocity.x();
    out.twist.twist.linear.y = linear_velocity.y();
    out.twist.twist.linear.z = linear_velocity.z();
    out.twist.twist.angular.x = angular_velocity.x();
    out.twist.twist.angular.y = angular_velocity.y();
    out.twist.twist.angular.z = angular_velocity.z();
  }

  previous_transform = transform;
  previous_time = current_time;
  previous_initialized = true;

  pub_chassis_odometry_->publish(out);
}

}  // namespace sensor_scan_generation

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(sensor_scan_generation::SensorScanGenerationNode)
