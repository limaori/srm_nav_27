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

#include "small_gicp_relocalization/small_gicp_relocalization.hpp"

#include <cmath>
#include <stdexcept>

#include "pcl/common/transforms.h"
#include "pcl/filters/filter.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/create_timer.hpp"
#include "small_gicp/pcl/pcl_registration.hpp"
#include "small_gicp/util/downsampling_omp.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

namespace small_gicp_relocalization
{

SmallGicpRelocalizationNode::SmallGicpRelocalizationNode(const rclcpp::NodeOptions & options)
: Node("small_gicp_relocalization", options),
  result_t_(Eigen::Isometry3d::Identity()),
  previous_result_t_(Eigen::Isometry3d::Identity())
{
  this->declare_parameter("num_threads", 4);
  this->declare_parameter("num_neighbors", 20);
  this->declare_parameter("global_leaf_size", 0.25);
  this->declare_parameter("registered_leaf_size", 0.25);
  this->declare_parameter("max_dist_sq", 1.0);
  this->declare_parameter("registration_interval", 0.5);
  this->declare_parameter("max_translation_step", 2.0);
  this->declare_parameter("max_rotation_step", 0.5);
  this->declare_parameter("min_inlier_ratio", 0.3);
  this->declare_parameter("max_registration_rmse", 0.3);
  this->declare_parameter("map_frame", "map");
  this->declare_parameter("odom_frame", "odom");
  this->declare_parameter("base_frame", "");
  this->declare_parameter("robot_base_frame", "");
  this->declare_parameter("lidar_frame", "");
  this->declare_parameter("prior_pcd_file", "");
  this->declare_parameter("init_pose", std::vector<double>{0., 0., 0., 0., 0., 0.});
  this->declare_parameter("input_cloud_topic", "registered_scan");

  this->get_parameter("num_threads", num_threads_);
  this->get_parameter("num_neighbors", num_neighbors_);
  this->get_parameter("global_leaf_size", global_leaf_size_);
  this->get_parameter("registered_leaf_size", registered_leaf_size_);
  this->get_parameter("max_dist_sq", max_dist_sq_);
  this->get_parameter("registration_interval", registration_interval_);
  this->get_parameter("max_translation_step", max_translation_step_);
  this->get_parameter("max_rotation_step", max_rotation_step_);
  this->get_parameter("min_inlier_ratio", min_inlier_ratio_);
  this->get_parameter("max_registration_rmse", max_registration_rmse_);
  this->get_parameter("map_frame", map_frame_);
  this->get_parameter("odom_frame", odom_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("lidar_frame", lidar_frame_);
  this->get_parameter("prior_pcd_file", prior_pcd_file_);
  this->get_parameter("init_pose", init_pose_);
  this->get_parameter("input_cloud_topic", input_cloud_topic_);

  if (
    num_threads_ < 1 || num_neighbors_ < 3 || !(global_leaf_size_ > 0.0) ||
    !(registered_leaf_size_ > 0.0) || !(max_dist_sq_ > 0.0) || !(registration_interval_ > 0.0) ||
    !(max_translation_step_ > 0.0) || !(max_rotation_step_ > 0.0) ||
    !(min_inlier_ratio_ > 0.0 && min_inlier_ratio_ <= 1.0) || !(max_registration_rmse_ > 0.0)) {
    throw std::invalid_argument("Invalid GICP sampling, timing or acceptance parameters");
  }

  // [x, y, z, roll, pitch, yaw] - init_pose parameters
  if (!init_pose_.empty() && init_pose_.size() >= 6) {
    result_t_.translation() << init_pose_[0], init_pose_[1], init_pose_[2];
    result_t_.linear() =
      Eigen::AngleAxisd(init_pose_[5], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(init_pose_[4], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(init_pose_[3], Eigen::Vector3d::UnitX()).toRotationMatrix();
  }
  previous_result_t_ = result_t_;

  registered_scan_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  global_map_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  register_ = std::make_shared<
    small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  loadGlobalMap(prior_pcd_file_);

  // Downsample points and convert them into pcl::PointCloud<pcl::PointCovariance>
  target_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *global_map_, global_leaf_size_);

  if (target_->size() < static_cast<size_t>(num_neighbors_)) {
    throw std::runtime_error("Prior PCD has too few valid points after downsampling");
  }

  // Estimate covariances of points
  small_gicp::estimate_covariances_omp(*target_, num_neighbors_, num_threads_);

  // Create KdTree for target
  target_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    target_, small_gicp::KdTreeBuilderOMP(num_threads_));

  pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_cloud_topic_, rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&SmallGicpRelocalizationNode::registeredPcdCallback, this, std::placeholders::_1));

  initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 10,
    std::bind(&SmallGicpRelocalizationNode::initialPoseCallback, this, std::placeholders::_1));

  register_timer_ = rclcpp::create_timer(
    this, this->get_clock(), rclcpp::Duration::from_seconds(registration_interval_),
    std::bind(&SmallGicpRelocalizationNode::performRegistration, this));

  transform_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),  // 20 Hz
    std::bind(&SmallGicpRelocalizationNode::publishTransform, this));
}

void SmallGicpRelocalizationNode::loadGlobalMap(const std::string & file_name)
{
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *global_map_) == -1) {
    throw std::runtime_error("Couldn't read PCD file: " + file_name);
  }
  std::vector<int> valid_indices;
  global_map_->is_dense = false;
  pcl::removeNaNFromPointCloud(*global_map_, *global_map_, valid_indices);
  RCLCPP_INFO(this->get_logger(), "Loaded global map with %zu points", global_map_->points.size());

  // NOTE: Transform global pcd_map (based on `lidar_odom` frame) to the `odom` frame
  Eigen::Affine3d odom_to_lidar_odom;
  while (rclcpp::ok()) {
    try {
      auto tf_stamped = tf_buffer_->lookupTransform(
        base_frame_, lidar_frame_, tf2::TimePointZero, tf2::durationFromSec(1.0));
      odom_to_lidar_odom = tf2::transformToEigen(tf_stamped.transform);
      RCLCPP_INFO_STREAM(
        this->get_logger(), "odom_to_lidar_odom: translation = "
                              << odom_to_lidar_odom.translation().transpose() << ", rpy = "
                              << odom_to_lidar_odom.rotation().eulerAngles(0, 1, 2).transpose());
      break;
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s Retrying...", ex.what());
      rclcpp::sleep_for(std::chrono::seconds(1));
    }
  }
  if (!rclcpp::ok()) {
    throw std::runtime_error("Shutdown while waiting for the prior-map frame transform");
  }
  pcl::transformPointCloud(*global_map_, *global_map_, odom_to_lidar_odom);
}

void SmallGicpRelocalizationNode::registeredPcdCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (msg->header.frame_id != odom_frame_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000, "Expected cloud in %s, received %s",
      odom_frame_.c_str(), msg->header.frame_id.c_str());
    return;
  }

  last_scan_time_ = msg->header.stamp;
  pcl::fromROSMsg(*msg, *registered_scan_);
  std::vector<int> valid_indices;
  registered_scan_->is_dense = false;
  pcl::removeNaNFromPointCloud(*registered_scan_, *registered_scan_, valid_indices);
}

void SmallGicpRelocalizationNode::performRegistration()
{
  if (registered_scan_->empty()) {
    return;
  }

  source_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *registered_scan_, registered_leaf_size_);
  registered_scan_->clear();

  if (source_->size() < static_cast<size_t>(num_neighbors_)) {
    return;
  }
  small_gicp::estimate_covariances_omp(*source_, num_neighbors_, num_threads_);

  register_->reduction.num_threads = num_threads_;
  register_->rejector.max_dist_sq = max_dist_sq_;
  register_->optimizer.max_iterations = 80;

  auto result = register_->align(*target_, *source_, *target_tree_, previous_result_t_);

  if (result.converged && result.T_target_source.matrix().allFinite()) {
    const Eigen::Matrix3d rotation = result.T_target_source.rotation();
    const double yaw = std::atan2(rotation(1, 0), rotation(0, 0));
    Eigen::Isometry3d planar = Eigen::Isometry3d::Identity();
    planar.translation() << result.T_target_source.translation().x(),
      result.T_target_source.translation().y(), 0.0;
    planar.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    size_t inliers = 0;
    double squared_error = 0.0;
    for (const auto & point : source_->points) {
      const Eigen::Vector4d transformed = planar * point.getVector4fMap().cast<double>();
      size_t neighbor_index;
      double squared_distance;
      if (
        target_tree_->nearest_neighbor_search(transformed, &neighbor_index, &squared_distance) &&
        squared_distance <= max_dist_sq_) {
        ++inliers;
        squared_error += squared_distance;
      }
    }
    const double inlier_ratio = static_cast<double>(inliers) / source_->size();
    const double rmse = inliers ? std::sqrt(squared_error / inliers) : INFINITY;
    const double dpos = (planar.translation() - previous_result_t_.translation()).norm();
    const double dangle =
      Eigen::AngleAxisd(planar.rotation() * previous_result_t_.rotation().inverse()).angle();
    if (
      dpos < max_translation_step_ && dangle < max_rotation_step_ &&
      inlier_ratio >= min_inlier_ratio_ && rmse <= max_registration_rmse_) {
      result_t_ = previous_result_t_ = planar;
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "GICP accepted: inliers=%.3f, rmse=%.3f m, correction=%.3f m / %.3f rad", inlier_ratio,
        rmse, dpos, dangle);
    } else {
      RCLCPP_WARN_STREAM(
        this->get_logger(), "GICP result rejected (delta pos=" << dpos << "m, angle=" << dangle
                                                               << "rad, inliers=" << inlier_ratio
                                                               << ", rmse=" << rmse << "m)");
    }
  } else {
    RCLCPP_WARN(this->get_logger(), "GICP did not converge.");
  }
}

void SmallGicpRelocalizationNode::publishTransform()
{
  if (last_scan_time_.nanoseconds() == 0) {
    return;
  }

  geometry_msgs::msg::TransformStamped transform_stamped;
  // `+ 0.1` means transform into future. according to https://robotics.stackexchange.com/a/96615
  transform_stamped.header.stamp = last_scan_time_ + rclcpp::Duration::from_seconds(0.1);
  transform_stamped.header.frame_id = map_frame_;
  transform_stamped.child_frame_id = odom_frame_;

  const Eigen::Vector3d translation = result_t_.translation();
  const Eigen::Quaterniond rotation(result_t_.rotation());

  transform_stamped.transform.translation.x = translation.x();
  transform_stamped.transform.translation.y = translation.y();
  transform_stamped.transform.translation.z = translation.z();
  transform_stamped.transform.rotation.x = rotation.x();
  transform_stamped.transform.rotation.y = rotation.y();
  transform_stamped.transform.rotation.z = rotation.z();
  transform_stamped.transform.rotation.w = rotation.w();

  tf_broadcaster_->sendTransform(transform_stamped);
}

void SmallGicpRelocalizationNode::initialPoseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  RCLCPP_INFO(
    this->get_logger(), "Received initial pose: [x: %f, y: %f, z: %f]", msg->pose.pose.position.x,
    msg->pose.pose.position.y, msg->pose.pose.position.z);

  Eigen::Isometry3d map_to_robot_base = Eigen::Isometry3d::Identity();
  map_to_robot_base.translation() << msg->pose.pose.position.x, msg->pose.pose.position.y,
    msg->pose.pose.position.z;
  Eigen::Quaterniond orientation(
    msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z);
  if (
    !orientation.coeffs().allFinite() || orientation.norm() < 1e-6 ||
    !map_to_robot_base.translation().allFinite()) {
    RCLCPP_WARN(this->get_logger(), "Ignoring invalid initial pose");
    return;
  }
  map_to_robot_base.linear() = orientation.normalized().toRotationMatrix();

  try {
    if (msg->header.frame_id != map_frame_) {
      auto map_transform =
        tf_buffer_->lookupTransform(map_frame_, msg->header.frame_id, msg->header.stamp);
      map_to_robot_base = tf2::transformToEigen(map_transform.transform) * map_to_robot_base;
    }
    auto transform =
      tf_buffer_->lookupTransform(robot_base_frame_, odom_frame_, tf2::TimePointZero);
    Eigen::Isometry3d robot_base_to_odom = tf2::transformToEigen(transform.transform);
    Eigen::Isometry3d map_to_odom = map_to_robot_base * robot_base_to_odom;

    previous_result_t_ = result_t_ = map_to_odom;
    registered_scan_->clear();
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(
      this->get_logger(), "Could not transform initial pose from %s to %s: %s",
      robot_base_frame_.c_str(), odom_frame_.c_str(), ex.what());
  }
}

}  // namespace small_gicp_relocalization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(small_gicp_relocalization::SmallGicpRelocalizationNode)
