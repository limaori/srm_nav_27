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

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>
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
namespace
{

constexpr char kLogPrefix[] = "[gicp_diag] ";

/// JSON 字符串转义：诊断输出是机器可读的逐次记录，必须保证可直接解析。
std::string jsonEscape(const std::string & input)
{
  std::string output;
  output.reserve(input.size() + 8);
  for (const char c : input) {
    switch (c) {
      case '"':
        output += "\\\"";
        break;
      case '\\':
        output += "\\\\";
        break;
      case '\n':
        output += "\\n";
        break;
      case '\r':
        output += "\\r";
        break;
      case '\t':
        output += "\\t";
        break;
      default:
        if (static_cast<unsigned char>(c) < 0x20) {
          std::ostringstream escaped;
          escaped << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                  << static_cast<int>(static_cast<unsigned char>(c));
          output += escaped.str();
        } else {
          output += c;
        }
    }
  }
  return output;
}

double wrapToPi(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

/// 只保留 yaw 的平面化：与验收口径（x/y/yaw）一致。
Eigen::Isometry3d toPlanar(const Eigen::Isometry3d & transform)
{
  const Eigen::Matrix3d rotation = transform.rotation();
  const double yaw = std::atan2(rotation(1, 0), rotation(0, 0));
  Eigen::Isometry3d planar = Eigen::Isometry3d::Identity();
  planar.translation() << transform.translation().x(), transform.translation().y(), 0.0;
  planar.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return planar;
}

double planarTranslationNorm(const Eigen::Isometry3d & a, const Eigen::Isometry3d & b)
{
  return (a.translation() - b.translation()).head<2>().norm();
}

double planarYawDelta(const Eigen::Isometry3d & a, const Eigen::Isometry3d & b)
{
  const double yaw_a = std::atan2(a.rotation()(1, 0), a.rotation()(0, 0));
  const double yaw_b = std::atan2(b.rotation()(1, 0), b.rotation()(0, 0));
  return std::abs(wrapToPi(yaw_a - yaw_b));
}

}  // namespace

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

  // 初始化专用策略：默认全部关闭，行为与既有版本一致。
  this->declare_parameter("init_acceptance_enable", false);
  this->declare_parameter("init_max_translation_step", 2.0);
  this->declare_parameter("init_max_rotation_step", 1.75);
  this->declare_parameter("init_accept_count", 3);
  this->declare_parameter("init_tracking_hold_count", 2);
  this->declare_parameter("init_hypothesis_enable", false);
  this->declare_parameter(
    "init_hypothesis_angles",
    std::vector<double>{-1.5707963267948966, -1.0471975511965976, -0.5235987755982988, 0.0,
                        0.5235987755982988, 1.0471975511965976, 1.5707963267948966});
  this->declare_parameter("init_hypothesis_quality_gap", 0.10);
  this->declare_parameter("init_hypothesis_rmse_margin", 0.05);

  // 逐次诊断与数据新鲜度
  this->declare_parameter("diagnostic_topic", "gicp_diagnostics");
  this->declare_parameter("run_id", "");
  this->declare_parameter("max_scan_age", 3.0);

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

  this->get_parameter("init_acceptance_enable", init_acceptance_enable_);
  this->get_parameter("init_max_translation_step", init_max_translation_step_);
  this->get_parameter("init_max_rotation_step", init_max_rotation_step_);
  this->get_parameter("init_accept_count", init_accept_count_);
  this->get_parameter("init_tracking_hold_count", init_tracking_hold_count_);
  this->get_parameter("init_hypothesis_enable", init_hypothesis_enable_);
  this->get_parameter("init_hypothesis_angles", init_hypothesis_angles_);
  this->get_parameter("init_hypothesis_quality_gap", init_hypothesis_quality_gap_);
  this->get_parameter("init_hypothesis_rmse_margin", init_hypothesis_rmse_margin_);

  this->get_parameter("diagnostic_topic", diagnostic_topic_);
  this->get_parameter("run_id", run_id_);
  this->get_parameter("max_scan_age", max_scan_age_);

  if (
    num_threads_ < 1 || num_neighbors_ < 3 || !(global_leaf_size_ > 0.0) ||
    !(registered_leaf_size_ > 0.0) || !(max_dist_sq_ > 0.0) || !(registration_interval_ > 0.0) ||
    !(max_translation_step_ > 0.0) || !(max_rotation_step_ > 0.0) ||
    !(min_inlier_ratio_ > 0.0 && min_inlier_ratio_ <= 1.0) || !(max_registration_rmse_ > 0.0)) {
    throw std::invalid_argument("Invalid GICP sampling, timing or acceptance parameters");
  }
  if (
    init_acceptance_enable_ &&
    (!(init_max_translation_step_ > 0.0) || !(init_max_rotation_step_ > 0.0))) {
    throw std::invalid_argument("Invalid GICP initialization acceptance parameters");
  }
  if (
    init_hypothesis_enable_ &&
    (!(init_hypothesis_quality_gap_ >= 0.0) || init_hypothesis_angles_.empty())) {
    throw std::invalid_argument("Invalid GICP initialization hypothesis parameters");
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
  applied_init_t_ = result_t_;

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

  diagnostic_pub_ = this->create_publisher<std_msgs::msg::String>(diagnostic_topic_, 200);

  register_timer_ = rclcpp::create_timer(
    this, this->get_clock(), rclcpp::Duration::from_seconds(registration_interval_),
    std::bind(&SmallGicpRelocalizationNode::performRegistration, this));

  transform_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),  // 20 Hz
    std::bind(&SmallGicpRelocalizationNode::publishTransform, this));

  RCLCPP_INFO(
    this->get_logger(),
    "[gicp_diag] node ready: run_id='%s' map_points=%zu target_points=%zu "
    "init_acceptance=%s init_hypothesis=%s max_rotation_step=%.4f init_max_rotation_step=%.4f",
    run_id_.c_str(), global_map_->size(), target_->size(), init_acceptance_enable_ ? "on" : "off",
    init_hypothesis_enable_ ? "on" : "off", max_rotation_step_, init_max_rotation_step_);
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

  last_scan_time_ = rclcpp::Time(msg->header.stamp, this->get_clock()->get_clock_type());
  last_scan_stamp_ns_ = last_scan_time_.nanoseconds();
  last_scan_recv_time_ = this->now();
  pcl::fromROSMsg(*msg, *registered_scan_);
  std::vector<int> valid_indices;
  registered_scan_->is_dense = false;
  pcl::removeNaNFromPointCloud(*registered_scan_, *registered_scan_, valid_indices);
}

void SmallGicpRelocalizationNode::evaluateTransform(
  const Eigen::Isometry3d & raw, bool converged, size_t iterations, double optimizer_error,
  Attempt & attempt) const
{
  attempt.converged = converged;
  attempt.iterations = iterations;
  attempt.optimizer_error = optimizer_error;

  if (!converged || !raw.matrix().allFinite()) {
    return;
  }

  attempt.planar = toPlanar(raw);

  // 质量评估：与 final TF 口径一致（平面化位姿 + max_dist_sq 内的最近邻）
  double squared_error = 0.0;
  attempt.inliers = 0;
  for (const auto & point : source_->points) {
    const Eigen::Vector4d transformed = attempt.planar * point.getVector4fMap().cast<double>();
    size_t neighbor_index;
    double squared_distance;
    if (
      target_tree_->nearest_neighbor_search(transformed, &neighbor_index, &squared_distance) &&
      squared_distance <= max_dist_sq_) {
      ++attempt.inliers;
      squared_error += squared_distance;
    } else {
      ++attempt.neighbors_in_range;
    }
  }
  attempt.inlier_ratio =
    source_->empty() ? 0.0 : static_cast<double>(attempt.inliers) / source_->size();
  attempt.rmse = attempt.inliers ? std::sqrt(squared_error / attempt.inliers) : INFINITY;

  attempt.dpos = (attempt.planar.translation() - previous_result_t_.translation()).norm();
  attempt.dangle =
    Eigen::AngleAxisd(attempt.planar.rotation() * previous_result_t_.rotation().inverse()).angle();

  // 与「本轮实际生效的初值」比较：/initialpose 覆盖后这里反映的是消息给定的初值，
  // 而不是启动参数 init_pose。
  attempt.init_dpos = (attempt.planar.translation() - applied_init_t_.translation()).norm();
  attempt.init_dangle =
    Eigen::AngleAxisd(attempt.planar.rotation() * applied_init_t_.rotation().inverse()).angle();
}

std::vector<SmallGicpRelocalizationNode::Candidate>
SmallGicpRelocalizationNode::evaluateHypotheses()
{
  std::vector<Candidate> candidates;
  candidates.reserve(init_hypothesis_angles_.size());

  for (const double yaw_offset : init_hypothesis_angles_) {
    Candidate candidate;
    candidate.yaw_offset = yaw_offset;

    Eigen::Isometry3d guess = previous_result_t_;
    guess.linear() = guess.rotation() * Eigen::AngleAxisd(yaw_offset, Eigen::Vector3d::UnitZ())
                                          .toRotationMatrix();

    const auto result = register_->align(*target_, *source_, *target_tree_, guess);
    candidate.converged = result.converged;
    candidate.iterations = result.iterations;
    candidate.optimizer_error = result.error;

    if (!result.converged || !result.T_target_source.matrix().allFinite()) {
      candidate.reason = "not_converged";
      candidates.push_back(candidate);
      continue;
    }

    double squared_error = 0.0;
    for (const auto & point : source_->points) {
      const Eigen::Vector4d transformed =
        candidate.planar * point.getVector4fMap().cast<double>();
      size_t neighbor_index;
      double squared_distance;
      if (
        target_tree_->nearest_neighbor_search(transformed, &neighbor_index, &squared_distance) &&
        squared_distance <= max_dist_sq_) {
        ++candidate.inliers;
        squared_error += squared_distance;
      } else {
        ++candidate.neighbors_in_range;
      }
    }
    candidate.inlier_ratio =
      source_->empty() ? 0.0 : static_cast<double>(candidate.inliers) / source_->size();
    candidate.rmse = candidate.inliers ? std::sqrt(squared_error / candidate.inliers) : INFINITY;
    candidate.distance_to_previous =
      (candidate.planar.translation() - previous_result_t_.translation()).norm();
    candidate.yaw_to_previous = planarYawDelta(candidate.planar, previous_result_t_);

    std::vector<std::string> reasons;
    // 候选搜索阶段允许大航向修正，因此这里只检查平移步长与质量门限。
    if (candidate.distance_to_previous >= init_max_translation_step_) {
      reasons.push_back("step_translation");
    }
    if (candidate.inlier_ratio < min_inlier_ratio_) {
      reasons.push_back("inlier_ratio");
    }
    if (candidate.rmse > max_registration_rmse_) {
      reasons.push_back("rmse");
    }
    candidate.step_ok = reasons.empty();
    candidate.feasible = reasons.empty();
    for (size_t i = 0; i < reasons.size(); ++i) {
      if (i) {
        candidate.reason += "|";
      }
      candidate.reason += reasons[i];
    }
    candidates.push_back(candidate);
  }
  return candidates;
}

void SmallGicpRelocalizationNode::performRegistration()
{
  const auto start_time = std::chrono::steady_clock::now();
  Attempt attempt;
  std::vector<Candidate> candidates;
  bool initialized_by_this_attempt = false;
  double best_quality_gap = 0.0;

  attempt.has_scan = !registered_scan_->empty();
  attempt.scan_stamp_ns = last_scan_stamp_ns_;
  if (attempt.has_scan && last_scan_stamp_ns_ != 0) {
    attempt.scan_age = (this->now() - last_scan_time_).seconds();
  }
  attempt.raw_points = registered_scan_->size();

  const auto finish = [&](const std::string & branch, const std::string & reason) {
    attempt.branch = branch;
    attempt.reason = reason;
    attempt.duration_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start_time)
        .count();
    publishDiagnostic(attempt, candidates, initialized_by_this_attempt, best_quality_gap);
  };

  if (!attempt.has_scan) {
    // 方案 §6/§7（S6）：没有新点云时不产生新 accepted，也不能刷新 TF 有效性。
    finish("no_scan", "no_point_cloud");
    return;
  }

  source_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *registered_scan_, registered_leaf_size_);
  registered_scan_->clear();
  attempt.source_points = source_->size();

  if (source_->size() < static_cast<size_t>(num_neighbors_)) {
    finish("too_few_points", "source_points_below_num_neighbors");
    return;
  }
  small_gicp::estimate_covariances_omp(*source_, num_neighbors_, num_threads_);

  register_->reduction.num_threads = num_threads_;
  register_->rejector.max_dist_sq = max_dist_sq_;
  register_->optimizer.max_iterations = 80;

  // 冷启动阶段（尚未稳定跟踪）且开启候选搜索时，先在粗初值周围生成多个航向候选。
  const bool in_init_phase = !initialized_ || tracking_hold_count_ > 0;
  if (init_hypothesis_enable_ && in_init_phase) {
    candidates = evaluateHypotheses();

    const Candidate * best = nullptr;
    const Candidate * second = nullptr;
    for (const auto & candidate : candidates) {
      if (!candidate.feasible) {
        continue;
      }
      if (!best || candidate.inlier_ratio > best->inlier_ratio ||
          (candidate.inlier_ratio == best->inlier_ratio && candidate.rmse < best->rmse)) {
        second = best;
        best = &candidate;
      } else if (!second || candidate.inlier_ratio > second->inlier_ratio) {
        second = &candidate;
      }
    }

    if (best) {
      attempt = Attempt();
      attempt.has_scan = true;
      attempt.scan_stamp_ns = last_scan_stamp_ns_;
      attempt.source_points = source_->size();
      attempt.planar = best->planar;
      attempt.converged = best->converged;
      attempt.iterations = best->iterations;
      attempt.optimizer_error = best->optimizer_error;
      attempt.inliers = best->inliers;
      attempt.neighbors_in_range = best->neighbors_in_range;
      attempt.inlier_ratio = best->inlier_ratio;
      attempt.rmse = best->rmse;
      attempt.dpos = (attempt.planar.translation() - previous_result_t_.translation()).norm();
      attempt.dangle = Eigen::AngleAxisd(
                         attempt.planar.rotation() * previous_result_t_.rotation().inverse())
                         .angle();

      if (second) {
        best_quality_gap = best->inlier_ratio - second->inlier_ratio;
      } else {
        best_quality_gap = best->inlier_ratio;  // 只有一个候选可行
      }

      // 歧义判定：次优候选质量接近时不接受，继续观察（方案 §10）。
      const bool ambiguous = second && best_quality_gap < init_hypothesis_quality_gap_ &&
                             second->rmse <= best->rmse + init_hypothesis_rmse_margin_;
      if (ambiguous) {
        finish("hypothesis_ambiguous", "candidate_quality_gap_too_small");
        return;
      }
      attempt.accepted = true;
      attempt.branch = "accepted_hypothesis";
      result_t_ = previous_result_t_ = attempt.planar;
      ++accepted_count_;
      ++init_accepted_count_;
      initialized_by_this_attempt = !initialized_;
      initialized_ = true;
      tracking_hold_count_ = init_tracking_hold_count_;
      finish(attempt.branch, "");
      return;
    }
    // 没有可行候选：仍然用基准初值做一次常规配准，把真实原因记录下来。
  }

  const auto result = register_->align(*target_, *source_, *target_tree_, previous_result_t_);
  attempt.raw_points = registered_scan_->size();
  evaluateTransform(result.T_target_source, result.converged, result.iterations, result.error,
                    attempt);

  if (!attempt.converged) {
    finish("not_converged", "optimizer_not_converged");
    return;
  }
  if (!attempt.planar.matrix().allFinite() || !result.T_target_source.matrix().allFinite()) {
    finish("non_finite", "non_finite_result");
    return;
  }

  const bool step_ok = attempt.dpos < max_translation_step_ && attempt.dangle < max_rotation_step_;
  const bool quality_ok =
    attempt.inlier_ratio >= min_inlier_ratio_ && attempt.rmse <= max_registration_rmse_;
  const bool init_step_ok = init_acceptance_enable_ && in_init_phase &&
                            attempt.dpos < init_max_translation_step_ &&
                            attempt.dangle < init_max_rotation_step_;

  attempt.reason.clear();
  if (!step_ok && !init_step_ok) {
    if (attempt.dpos >= max_translation_step_) {
      attempt.reason += "step_translation";
    }
    if (attempt.dangle >= max_rotation_step_) {
      if (!attempt.reason.empty()) {
        attempt.reason += "|";
      }
      attempt.reason += "step_rotation";
    }
  }
  if (attempt.inlier_ratio < min_inlier_ratio_) {
    if (!attempt.reason.empty()) {
      attempt.reason += "|";
    }
    attempt.reason += "inlier_ratio";
  }
  if (attempt.rmse > max_registration_rmse_) {
    if (!attempt.reason.empty()) {
      attempt.reason += "|";
    }
    attempt.reason += "rmse";
  }

  if (attempt.reason.empty()) {
    attempt.accepted = true;
    attempt.branch = step_ok ? "accepted" : "accepted_init_step";
    result_t_ = previous_result_t_ = attempt.planar;
    ++accepted_count_;
    if (in_init_phase) {
      ++init_accepted_count_;
      initialized_by_this_attempt = !initialized_;
      initialized_ = true;
      tracking_hold_count_ = init_tracking_hold_count_;
    }
    if (tracking_hold_count_ > 0) {
      --tracking_hold_count_;
    }
    finish(attempt.branch, "");
    return;
  }

  finish("rejected", attempt.reason);
}

void SmallGicpRelocalizationNode::publishDiagnostic(
  const Attempt & attempt, const std::vector<Candidate> & candidates,
  bool initialized_by_this_attempt, double best_quality_gap)
{
  const int64_t now_ns = this->now().nanoseconds();
  const double yaw = std::atan2(attempt.planar.rotation()(1, 0), attempt.planar.rotation()(0, 0));

  std::ostringstream json;
  json << std::fixed << std::setprecision(6);
  json << "{\"seq\":" << attempt_seq_ << ",\"run_id\":\"" << jsonEscape(run_id_) << "\""
       << ",\"node_wall_ns\":" << now_ns << ",\"scan_stamp_ns\":" << attempt.scan_stamp_ns
       << ",\"scan_age\":" << attempt.scan_age << ",\"interval\":" << registration_interval_
       << ",\"has_scan\":" << (attempt.has_scan ? "true" : "false")
       << ",\"raw_points\":" << attempt.raw_points << ",\"source_points\":" << attempt.source_points
       << ",\"converged\":" << (attempt.converged ? "true" : "false")
       << ",\"iterations\":" << attempt.iterations
       << ",\"optimizer_error\":" << attempt.optimizer_error << ",\"inliers\":" << attempt.inliers
       << ",\"neighbors_out_of_range\":" << attempt.neighbors_in_range
       << ",\"inlier_ratio\":" << attempt.inlier_ratio << ",\"rmse\":" << attempt.rmse
       << ",\"dpos\":" << attempt.dpos << ",\"dangle\":" << attempt.dangle
       << ",\"init_dpos\":" << attempt.init_dpos << ",\"init_dangle\":" << attempt.init_dangle
       << ",\"result_x\":" << attempt.planar.translation().x()
       << ",\"result_y\":" << attempt.planar.translation().y() << ",\"result_yaw\":" << yaw
       << ",\"accepted\":" << (attempt.accepted ? "true" : "false")
       << ",\"accepted_count\":" << accepted_count_
       << ",\"init_accepted_count\":" << init_accepted_count_
       << ",\"initialized\":" << (initialized_ ? "true" : "false")
       << ",\"initialized_by_this_attempt\":"
       << (initialized_by_this_attempt ? "true" : "false")
       << ",\"branch\":\"" << jsonEscape(attempt.branch) << "\""
       << ",\"reason\":\"" << jsonEscape(attempt.reason) << "\""
       << ",\"planarized\":true,\"duration_ms\":" << attempt.duration_ms
       << ",\"best_quality_gap\":" << best_quality_gap;

  if (!candidates.empty()) {
    json << ",\"candidates\":[";
    for (size_t i = 0; i < candidates.size(); ++i) {
      const auto & candidate = candidates[i];
      if (i) {
        json << ",";
      }
      json << "{\"yaw_offset\":" << candidate.yaw_offset
           << ",\"converged\":" << (candidate.converged ? "true" : "false")
           << ",\"iterations\":" << candidate.iterations
           << ",\"optimizer_error\":" << candidate.optimizer_error
           << ",\"inlier_ratio\":" << candidate.inlier_ratio << ",\"rmse\":" << candidate.rmse
           << ",\"distance_to_previous\":" << candidate.distance_to_previous
           << ",\"yaw_to_previous\":" << candidate.yaw_to_previous
           << ",\"feasible\":" << (candidate.feasible ? "true" : "false")
           << ",\"reason\":\"" << jsonEscape(candidate.reason) << "\"}";
    }
    json << "]";
  }
  json << "}";

  std_msgs::msg::String message;
  message.data = json.str();
  diagnostic_pub_->publish(message);

  // 逐次诊断同时进入日志，便于没有订阅者时事后归档。accepted 日志不再节流。
  RCLCPP_INFO(this->get_logger(), "%s%s", kLogPrefix, message.data.c_str());

  ++attempt_seq_;
}

void SmallGicpRelocalizationNode::fillTransform(
  geometry_msgs::msg::TransformStamped & transform_stamped) const
{
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
}

void SmallGicpRelocalizationNode::publishTransform()
{
  if (last_scan_time_.nanoseconds() == 0) {
    return;
  }

  // 方案 §2/§7（S6）：点云停发后不得继续用旧扫描时间戳刷新 map -> odom。
  if (max_scan_age_ > 0.0) {
    const double scan_age = (this->now() - last_scan_time_).seconds();
    if (scan_age > max_scan_age_) {
      if (tf_published_) {
        RCLCPP_WARN(
          this->get_logger(),
          "[gicp_diag] input stale (%.3f s > %.3f s): stop publishing map -> %s",
          scan_age, max_scan_age_, odom_frame_.c_str());
        tf_published_ = false;
      }
      return;
    }
  }

  geometry_msgs::msg::TransformStamped transform_stamped;
  // `+ 0.1` means transform into future. according to https://robotics.stackexchange.com/a/96615
  transform_stamped.header.stamp = last_scan_time_ + rclcpp::Duration::from_seconds(0.1);
  fillTransform(transform_stamped);

  tf_broadcaster_->sendTransform(transform_stamped);
  tf_published_ = true;
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
    applied_init_t_ = map_to_odom;
    registered_scan_->clear();
    // 重新给定初值等同于重新冷启动：重置初始化阶段计数与诊断序号。
    initialized_ = false;
    init_accepted_count_ = 0;
    tracking_hold_count_ = 0;
    const int64_t now_ns = this->now().nanoseconds();
    std::ostringstream json;
    json << std::fixed << std::setprecision(6);
    json << "{\"seq\":" << attempt_seq_ << ",\"run_id\":\"" << jsonEscape(run_id_) << "\""
         << ",\"event\":\"initial_pose_applied\",\"node_wall_ns\":" << now_ns
         << ",\"stamp_ns\":" << rclcpp::Time(msg->header.stamp).nanoseconds()
         << ",\"frame_id\":\"" << jsonEscape(msg->header.frame_id) << "\""
         << ",\"result_x\":" << map_to_odom.translation().x()
         << ",\"result_y\":" << map_to_odom.translation().y()
         << ",\"result_yaw\":"
         << std::atan2(map_to_odom.rotation()(1, 0), map_to_odom.rotation()(0, 0))
         << ",\"branch\":\"initial_pose_applied\",\"reason\":\"\""
         << ",\"accepted\":false,\"converged\":false,\"scan_stamp_ns\":0,\"source_points\":0"
         << ",\"raw_points\":0,\"duration_ms\":0.0}";
    std_msgs::msg::String message;
    message.data = json.str();
    diagnostic_pub_->publish(message);
    RCLCPP_INFO(this->get_logger(), "%s%s", kLogPrefix, message.data.c_str());
    ++attempt_seq_;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(
      this->get_logger(), "Could not transform initial pose from %s to %s: %s",
      robot_base_frame_.c_str(), odom_frame_.c_str(), ex.what());
  }
}

}  // namespace small_gicp_relocalization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(small_gicp_relocalization::SmallGicpRelocalizationNode)
