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

#ifndef SRM27_MINCO_CONTROLLER__MINCO_MPC_CONTROLLER_HPP_
#define SRM27_MINCO_CONTROLLER__MINCO_MPC_CONTROLLER_HPP_

#include <Eigen/Core>
#include <atomic>
#include <cstdint>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <memory>
#include <mutex>
#include <nav2_core/controller.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <string>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>

#include "srm27_minco_controller/costmap_adapter.hpp"
#include "srm27_minco_controller/diagnostics.hpp"
#include "srm27_minco_controller/planning_worker.hpp"
#include "srm27_minco_controller/state_adapter.hpp"
#include "srm27_minco_controller/yaw_policy.hpp"
#include "srm27_minco_core/esdf_2d.hpp"
#include "srm27_minco_core/minco_optimizer.hpp"
#include "srm27_minco_core/mpc_solver.hpp"
#include "srm27_minco_core/replan_manager.hpp"
#include "srm27_minco_core/tracking_reference.hpp"
#include "srm27_minco_core/trajectory_validator.hpp"
#include "srm27_minco_core/types.hpp"

namespace srm27_minco_controller
{

/// \brief MINCO + MPC 的 Nav2 `FollowPath` 控制器插件。
///
/// 结构（方案 §4.1、§4.3）：
///  * 控制线程（`computeVelocityCommands`）：取一致状态、维护地图快照与距离场、做
///    轨迹复验、构造跟踪参考、求解 MPC、校验并返回车体系 `TwistStamped`。
///  * 后台线程（`PlanningWorker`）：前端初值 + 两阶段 MINCO + 独立轨迹验证，只产出
///    不可变轨迹快照，不发布任何速度。
///  * 唯一速度出口是 Nav2 标准返回路径；插件不直连串口、Gazebo 或 mux。
class MincoMpcController : public nav2_core::Controller
{
public:
  MincoMpcController() = default;
  ~MincoMpcController() override;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & _parent, std::string _name,
    std::shared_ptr<tf2_ros::Buffer> _tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> _costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  void setPlan(const nav_msgs::msg::Path & _path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & _pose, const geometry_msgs::msg::Twist & _velocity,
    nav2_core::GoalChecker * _goal_checker) override;

  void setSpeedLimit(const double & _speed_limit, const bool & _percentage) override;

private:
  /// \brief 声明并读取参数；非法配置直接失败并报清楚原因。
  bool loadParameters(std::string * _reason);

  /// \brief 由配置建立 MPC 模型与求解器。
  bool configureSolver(std::string * _reason);

  /// \brief 构造轨迹校验器配置。
  ///
  /// 控制周期内的复验（`validator_`）与后台工作线程使用**同一份**配置，
  /// 只有速度/加速度上限按当前有效约束覆盖。
  srm27_minco_core::TrajectoryValidatorConfig makeValidatorConfig() const;

  /// \brief 每个控制周期刷新地图快照与距离场。
  void refreshMap(double _stamp, const ControllerDiagnostics & _diagnostics);

  /// \brief 依据重规划策略提交一次后台规划请求。
  void requestReplan(
    const srm27_minco_core::State2D & _state, double _now_stamp, double _now_steady);

  /// \brief 取回并提交后台规划结果。
  void harvestPlanningResult(double _now_stamp);

  /// \brief 把里程计坐标系下的路径转换到规划坐标系。
  bool updatePathInPlanningFrame(double _now_stamp, std::string * _reason);

  /// \brief 轨迹作废（新目标、取消、限速收紧、定位重置、失活）。
  void invalidateTrajectory(const std::string & _reason, double _now_stamp);

  /// \brief 组装一条受控零速度命令并记录停止原因。
  geometry_msgs::msg::TwistStamped stopCommand(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & _node, const std::string & _reason,
    double _now_stamp);

  /// \brief 发布规划器实际使用的局部路径（可视化，不参与控制）。
  void publishPlanningInputPath(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & _node, double _now_stamp);

  /// \brief 发布 MINCO 轨迹与 MPC 预测轨迹（可视化，不参与控制）。
  void publishVisualization(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & _node,
    const srm27_minco_core::Trajectory2D & _trajectory,
    const srm27_minco_core::MpcSolution & _solution, double _now_stamp);

  /// \brief 发布诊断。
  void publishDiagnostics(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & _node, const ControllerDiagnostics & _diag,
    double _now_stamp);

  /// \brief 从已提交轨迹提取热启动初值。
  void updateWarmStart(const srm27_minco_core::Trajectory2D & _trajectory);

  // 插件与生命周期
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_{};
  std::string plugin_name_{"FollowPath"};
  std::shared_ptr<tf2_ros::Buffer> tf_{};
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_{};
  bool configured_{false};
  bool active_{false};

  // 参数
  std::string planning_frame_{"odom"};
  std::string base_frame_{"base_link"};
  std::string odom_topic_{"odometry"};
  double replan_frequency_{10.0};
  double planning_horizon_{2.0};
  double planning_deadline_ms_{30.0};
  double state_timeout_{0.10};
  double map_timeout_{0.30};
  double trajectory_max_age_{0.30};
  int polynomial_order_{5};
  bool two_stage_optimization_{true};
  bool two_pass_reference_{false};
  bool stop_rotation_on_goal_{true};
  double command_lookahead_{0.0};
  double qp_deadline_ms_{5.0};
  int prediction_steps_{30};
  double prediction_dt_{0.02};
  std::string mpc_model_name_{"velocity_integrator"};
  YawMode yaw_mode_{YawMode::kXyOnly};
  double yaw_narrow_clearance_threshold_{0.60};
  bool yaw_allow_reverse_{true};
  double state_max_sample_gap_{0.20};
  double state_max_position_jump_{0.75};
  double state_max_yaw_jump_{1.20};
  double state_velocity_filter_alpha_{0.35};
  double diagnostics_period_{0.5};
  double build_grace_period_{1.0};
  /// \brief 有效制动减速度（m/s^2）；<=0 表示退化为使用 `limits.max_linear_accel`。
  ///
  /// 这个量直接决定"轨迹有效前缀必须覆盖多长才能停车"：取小了会把所有候选轨迹判死
  /// （0.5 m/s 时要求 1.77 s 还能过，1.5 m/s 时要求 5.1 s 就必然失败），
  /// 所以必须显式配置并来自实测，而不是沿用优化器里的加速度上限（方案 §6.1、§10 P5）。
  double braking_deceleration_{0.0};
  /// \brief 判定"已到达路径终点"的距离半径（m）。
  double terminal_reached_radius_{0.20};
  double release_timeout_{0.20};
  bool publish_visualization_{true};

  // 有效约束
  srm27_minco_core::Limits2D base_limits_{};
  srm27_minco_core::Limits2D limits_{};
  std::uint64_t limits_version_{1};
  double applied_speed_scale_{1.0};

  // 子模块
  StateAdapter state_adapter_{};
  CostmapAdapter costmap_adapter_{};
  YawPolicy yaw_policy_{};
  PlanningWorker worker_{};
  srm27_minco_core::MpcModel mpc_model_{};
  srm27_minco_core::MpcSolver mpc_solver_{};
  srm27_minco_core::MpcSolverConfig mpc_config_{};
  srm27_minco_core::TrackingReferenceBuilder reference_builder_{};
  srm27_minco_core::TrajectoryValidator validator_{};
  srm27_minco_core::ReplanManager replan_manager_{};
  srm27_minco_core::MincoOptimizerConfig minco_config_{};
  srm27_minco_core::TrajectoryInitializer::Config initializer_config_{};
  srm27_minco_core::Esdf2D::Config esdf_config_{};
  srm27_minco_core::TrackingReferenceConfig reference_config_{};

  // 运行状态
  std::mutex path_mutex_{};
  nav_msgs::msg::Path raw_path_{};
  bool has_raw_path_{false};
  std::vector<Eigen::Vector2d> planning_path_{};
  bool has_planning_path_{false};
  std::string path_source_frame_{};
  std::uint64_t path_version_{0};
  std::uint64_t goal_epoch_{0};
  std::uint64_t trajectory_id_{0};

  std::shared_ptr<const srm27_minco_core::Trajectory2D> trajectory_{};
  std::shared_ptr<srm27_minco_core::Esdf2D> esdf_{};
  double esdf_received_steady_{0.0};
  std::uint64_t map_version_{0};
  double map_stamp_{0.0};
  double last_full_revalidation_stamp_{0.0};
  /// \brief 上次复验之后地图是否变化过（只用于诊断与节流判断）。
  bool map_changed_since_validation_{false};

  std::vector<Eigen::Vector2d> warm_inner_points_{};
  std::vector<double> warm_durations_{};
  bool has_warm_start_{false};

  Eigen::VectorXd previous_applied_input_{};
  bool has_previous_input_{false};
  double last_control_stamp_{0.0};
  bool has_last_control_stamp_{false};
  std::uint64_t last_odom_reset_count_{0};

  std::uint64_t switched_trajectory_count_{0};
  double build_start_stamp_{-1.0};
  bool stopping_{false};
  /// \brief 上一次规划判定"已在终点"，用于受控停车（而非当成控制失败）。
  bool at_goal_stop_{false};
  std::string stop_reason_{"none"};
  double stop_request_stamp_{0.0};
  double output_zero_stamp_{0.0};
  double actual_stop_stamp_{0.0};

  ControllerDiagnostics diagnostics_{};

  // 发布者
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_{};
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr prediction_pub_{};
  /// \brief 规划器输入的局部路径（诊断用）。
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr planning_input_pub_{};
  std::vector<Eigen::Vector2d> last_local_path_{};
  double last_local_path_stamp_{0.0};
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    diagnostics_pub_{};
  double last_diagnostics_stamp_{0.0};
};

}  // namespace srm27_minco_controller

#endif  // SRM27_MINCO_CONTROLLER__MINCO_MPC_CONTROLLER_HPP_
