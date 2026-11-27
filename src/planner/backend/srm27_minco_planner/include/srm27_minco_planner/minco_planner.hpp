// Copyright 2026 SRM
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

#ifndef SRM27_MINCO_PLANNER__MINCO_PLANNER_HPP_
#define SRM27_MINCO_PLANNER__MINCO_PLANNER_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Core>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "srm27_map_adapter/costmap_adapter.hpp"
#include "srm27_map_core/esdf_2d.hpp"
#include "srm27_minco_core/minco_optimizer.hpp"
#include "srm27_minco_core/trajectory_2d.hpp"
#include "srm27_minco_core/trajectory_initializer.hpp"
#include "srm27_minco_core/trajectory_validator.hpp"
#include "srm27_minco_core/types.hpp"
#include "srm27_minco_interfaces/msg/minco_trajectory.hpp"
#include "srm27_minco_interfaces/msg/minco_path_request.hpp"
#include "srm27_minco_pipeline/planning_worker.hpp"
#include "srm27_nav_planner/yastar_global_planner.hpp"
#include "tf2_ros/buffer.h"

namespace srm27_minco_planner
{

/// \brief Nav2 全局规划器插件：调 YAstar 出全局路径，内部再用 MINCO 把它变成
/// 可执行的**时间参数化轨迹**并发布。
///
/// 为什么是"规划器 + 内部线程"（对齐 `navi_minco_bit` 的 `MincoPlanner`）：
///  * `createPlan()` 是同步接口，MINCO 单次 3~15 ms、还要按 `opt_freq` 连续重规划，
///    塞进 action 调用会把 BT 的 compute_path 一起拖慢；
///  * 所以 `createPlan()` 只做全局搜索并记录目标，真正的 MINCO 在内部线程里按
///    `opt_freq` 跑，结果通过 `MincoTrajectory` 话题直接发给控制器；
///  * 控制器因此退化为纯 MPC 跟踪（`trajectory_source: topic`），与参考工程一致。
///
/// 输入路径：默认用 `smoother_server` 输出的 `/plan_smoothed`（`smooth_path_topic`），
/// 拿不到时退回 `createPlan()` 的原始路径。这样保住了本仓库"走廊居中"的效果
/// —— 直接吃 YAstar 的稀疏折线会切内道。
class MincoPlanner : public nav2_core::GlobalPlanner
{
public:
  MincoPlanner() = default;
  ~MincoPlanner() override;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  /// \brief 调 YAstar 生成全局路径（map），同时记录目标供内部线程使用。
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  // ---- 参数 ----
  void loadParameters();
  bool validateParameters(std::string * _reason) const;

  // ---- 内部线程 ----
  void runLoop();
  void stopLoop();
  void tick();
  bool currentState(srm27_minco_core::State2D & _state, std::string * _reason);
  bool updateOdometryVelocity();
  std::vector<Eigen::Vector2d> selectInputPath(std::uint64_t & _path_version) const;
  void publishTrajectory(
    const srm27_minco_core::Trajectory2D & _trajectory,
    const srm27_minco_core::VersionSet & _versions, double _stamp, double _terminal_speed,
    const std::string & _terminal_reason);
  void publishDiagnostics();

  // ---- 组件 ----
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::shared_ptr<rclcpp::Clock> clock_;
  std::string plugin_name_;

  /// 复用既有 YAstar 插件做全局搜索（它自己的参数在 `GridBased:` 段里）。
  srm27_nav_planner::YAstarGlobalPlanner yastar_;
  /// 内部 YAstar 实例的参数前缀（`planner_server.<yastar_plugin_name>.*`）。
  std::string yastar_plugin_name_{"yastar"};

  srm27_map_adapter::CostmapAdapter costmap_adapter_;
  srm27_map_core::Esdf2D::Config esdf_config_{};

  // ---- 参数值 ----
  std::string map_frame_{"map"};
  std::string base_frame_{"gimbal_yaw"};
  std::string output_frame_{"odom"};
  std::string follow_path_topic_;
  std::string smoothed_topic_{"plan_smoothed"};
  std::string odom_topic_{"odometry"};
  std::string trajectory_topic_{"planning/minco_trajectory"};
  std::string path_topic_{"planning/minco_path"};
  std::string diagnostics_topic_{"planning/diagnostics"};
  double opt_frequency_{10.0};
  double planning_horizon_{6.0};
  double path_centering_max_displacement_{0.3};
  double validity_window_{1.0};
  double terminal_reached_radius_{0.2};
  double goal_change_tolerance_{0.1};
  double diagnostics_period_{0.5};
  double state_timeout_{0.5};
  bool publish_visualization_{true};
  bool use_warm_start_{true};

  srm27_minco_core::Limits2D limits_{};
  srm27_minco_core::MincoOptimizerConfig minco_config_{};
  srm27_minco_core::TrajectoryInitializer::Config initializer_config_{};
  srm27_minco_core::TrajectoryValidatorConfig validator_config_{};

  // ---- 运行期状态 ----
  mutable std::mutex mutex_;
  srm27_minco_interfaces::msg::MincoPathRequest::ConstSharedPtr follow_path_request_;
  double follow_path_received_{-1.0};
  rclcpp::Subscription<srm27_minco_interfaces::msg::MincoPathRequest>::SharedPtr follow_path_sub_;
  nav_msgs::msg::Path latest_plan_{};    ///< createPlan 的结果（map）
  double latest_plan_stamp_{0.0};
  nav_msgs::msg::Path smoothed_path_{};  ///< smoother_server 的输出（map）
  double smoothed_stamp_{0.0};
  /// `/plan_smoothed` 是否在**当前这份 plan 之后**收到过；没收到就退回原始路径。
  bool smoothed_fresh_{false};
  bool has_goal_{false};
  Eigen::Vector2d goal_{Eigen::Vector2d::Zero()};
  std::uint64_t goal_epoch_{0};
  std::uint64_t path_version_{0};
  std::uint64_t map_version_{0};

  // 里程计缓存：只用来算速度（位姿一律走 TF，避免两套位姿不一致）
  nav_msgs::msg::Odometry last_odom_{};
  bool has_odom_{false};
  Eigen::Vector2d odom_velocity_{Eigen::Vector2d::Zero()};
  Eigen::Vector2d previous_odom_position_{Eigen::Vector2d::Zero()};
  double last_odom_stamp_{0.0};

  // 热启动缓存（可选）
  bool has_warm_start_{false};
  std::vector<Eigen::Vector2d> warm_inner_points_{};
  std::vector<double> warm_durations_{};
  double warm_start_stamp_{0.0};

  // 最近一次结果（诊断用）
  srm27_minco_pipeline::PlanningResult last_result_{};
  bool has_result_{false};
  double last_result_stamp_{0.0};
  double last_diagnostics_stamp_{0.0};

  // ---- ROS 接口 ----
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr smoothed_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp_lifecycle::LifecyclePublisher<srm27_minco_interfaces::msg::MincoTrajectory>::SharedPtr
    trajectory_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    diagnostics_pub_;

  // ---- 线程 ----
  std::thread loop_;
  std::atomic<bool> running_{false};
  std::atomic<bool> active_{false};
};

}  // namespace srm27_minco_planner

#endif  // SRM27_MINCO_PLANNER__MINCO_PLANNER_HPP_
