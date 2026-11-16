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

#ifndef SMALL_GICP_RELOCALIZATION__SMALL_GICP_RELOCALIZATION_HPP_
#define SMALL_GICP_RELOCALIZATION__SMALL_GICP_RELOCALIZATION_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "pcl/io/pcd_io.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "small_gicp/ann/kdtree_omp.hpp"
#include "small_gicp/factors/gicp_factor.hpp"
#include "small_gicp/pcl/pcl_point.hpp"
#include "small_gicp/registration/reduction_omp.hpp"
#include "small_gicp/registration/registration.hpp"
#include "small_gicp/registration/registration_result.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace small_gicp_relocalization
{

class SmallGicpRelocalizationNode : public rclcpp::Node
{
public:
  explicit SmallGicpRelocalizationNode(const rclcpp::NodeOptions & options);

private:
  /// 单次配准尝试的完整结果，用于逐次诊断输出。
  struct Attempt
  {
    // 触发本次尝试的输入状态
    bool has_scan = false;
    int64_t scan_stamp_ns = 0;
    double scan_age = -1.0;
    size_t raw_points = 0;
    size_t source_points = 0;

    // 优化结果
    bool converged = false;
    size_t iterations = 0;
    double optimizer_error = 0.0;

    // 平面化后的位姿（map -> odom）
    Eigen::Isometry3d planar = Eigen::Isometry3d::Identity();

    // 质量与步长
    size_t inliers = 0;
    size_t neighbors_in_range = 0;
    double inlier_ratio = 0.0;
    double rmse = 0.0;
    double dpos = 0.0;
    double dangle = 0.0;
    double init_dpos = 0.0;
    double init_dangle = 0.0;

    bool accepted = false;
    std::string branch;   // 判定分支：no_scan / too_few_points / not_converged / non_finite / accepted ...
    std::string reason;   // 拒绝原因（可多个，`|` 分隔）
    double duration_ms = 0.0;
  };

  /// 多航向候选的一次评估结果。
  struct Candidate
  {
    double yaw_offset = 0.0;
    Eigen::Isometry3d planar = Eigen::Isometry3d::Identity();
    bool converged = false;
    size_t iterations = 0;
    double optimizer_error = 0.0;
    size_t inliers = 0;
    size_t neighbors_in_range = 0;
    double inlier_ratio = 0.0;
    double rmse = 0.0;
    double distance_to_previous = 0.0;
    double yaw_to_previous = 0.0;
    bool step_ok = false;
    bool feasible = false;  // 收敛、有限、步长与质量门限全部通过
    std::string reason;
  };

  void registeredPcdCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void loadGlobalMap(const std::string & file_name);
  void performRegistration();
  void publishTransform();
  void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  /// 单次配准后按当前 `/tf` 计算平面化结果、内点率、RMSE 与相对上一步的步长。
  void evaluateTransform(
    const Eigen::Isometry3d & raw, bool converged, size_t iterations, double optimizer_error,
    Attempt & attempt) const;

  /// 围绕上一次结果按 `init_hypothesis_angles` 生成候选并逐一带初值局部配准。
  std::vector<Candidate> evaluateHypotheses();

  /// 追加一条逐次诊断 JSON（覆盖 accepted 与全部失败分支）。
  void publishDiagnostic(const Attempt & attempt, const std::vector<Candidate> & candidates,
                         bool initialized_by_this_attempt, double best_quality_gap);

  /// 把当前位姿写进 TF 消息。
  void fillTransform(geometry_msgs::msg::TransformStamped & transform_stamped) const;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostic_pub_;

  int num_threads_;
  int num_neighbors_;
  float global_leaf_size_;
  float registered_leaf_size_;
  float max_dist_sq_;
  double registration_interval_;
  double max_translation_step_;
  double max_rotation_step_;
  double min_inlier_ratio_;
  double max_registration_rmse_;
  std::vector<double> init_pose_;

  // ---- 初始化专用策略（方案 §10）----
  // 默认关闭：保持既有行为不变。开启后冷启动阶段允许更大的单次修正，
  // 并/或在上一次结果周围做多航向候选搜索。
  bool init_acceptance_enable_;
  double init_max_translation_step_;
  double init_max_rotation_step_;
  int init_accept_count_;
  int init_tracking_hold_count_;
  bool init_hypothesis_enable_;
  std::vector<double> init_hypothesis_angles_;
  double init_hypothesis_quality_gap_;
  double init_hypothesis_rmse_margin_;

  std::string map_frame_;
  std::string odom_frame_;
  std::string prior_pcd_file_;
  std::string base_frame_;
  std::string robot_base_frame_;
  std::string lidar_frame_;
  std::string input_cloud_topic_;
  std::string diagnostic_topic_;
  std::string run_id_;
  double max_scan_age_;

  rclcpp::Time last_scan_time_;
  rclcpp::Time last_scan_recv_time_;
  int64_t last_scan_stamp_ns_ = 0;
  uint64_t attempt_seq_ = 0;
  uint64_t accepted_count_ = 0;
  int init_accepted_count_ = 0;  // 冷启动阶段已接受的配准次数
  int tracking_hold_count_ = 0;  // 进入连续跟踪后仍然允许初始化门限的剩余次数
  bool initialized_ = false;
  bool tf_published_ = false;
  Eigen::Isometry3d result_t_;
  Eigen::Isometry3d previous_result_t_;
  Eigen::Isometry3d applied_init_t_;  // 本轮实际生效的初值（init_pose 参数或 /initialpose）

  pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr registered_scan_;
  pcl::PointCloud<pcl::PointCovariance>::Ptr target_;
  pcl::PointCloud<pcl::PointCovariance>::Ptr source_;

  std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> target_tree_;
  std::shared_ptr<
    small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>
    register_;

  rclcpp::TimerBase::SharedPtr transform_timer_;
  rclcpp::TimerBase::SharedPtr register_timer_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace small_gicp_relocalization

#endif  // SMALL_GICP_RELOCALIZATION__SMALL_GICP_RELOCALIZATION_HPP_
