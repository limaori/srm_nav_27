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

#include "srm27_minco_controller/minco_mpc_controller.hpp"

#include <tf2/utils.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <nav2_core/exceptions.hpp>
#include <nav2_util/node_utils.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <memory>
#include <stdexcept>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>

#include "srm27_minco_core/kinematics.hpp"

namespace srm27_minco_controller
{

using nav2_util::declare_parameter_if_not_declared;
using srm27_minco_core::Limits2D;
using srm27_minco_core::MpcInitialState;
using srm27_minco_core::MpcReference;
using srm27_minco_core::MpcSolution;
using srm27_minco_core::SolveStatus;
using srm27_minco_core::State2D;
using srm27_minco_core::TrackingReferenceResult;
using srm27_minco_core::Trajectory2D;

namespace
{

/// \brief 插件级异常。
///
/// Humble 的 `nav2_core` 只定义了 `PlannerException`，没有给控制器插件提供专用的
/// 异常类型；controller_server 会把插件抛出的 `std::runtime_error` 当作控制器失败
/// 处理（`computeVelocityCommands` 之外由 server 的 try/catch 统一接管）。这里定义
/// 一个明确的派生类型，便于后续按类型区分“插件配置错误”和“运行时控制失败”。
class MincoMpcControllerError : public std::runtime_error
{
public:
  explicit MincoMpcControllerError(const std::string & _message) : std::runtime_error(_message) {}
};

/// \brief 单调时钟（秒），只用于耗时与看门狗，不参与状态时间戳。
double steadyNow()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
           std::chrono::steady_clock::now().time_since_epoch())
           .count() *
         1.0e-9;
}

/// \brief 诊断严重级别。
constexpr int kDiagnosticOk = 0;
constexpr int kDiagnosticWarn = 1;

/// \brief 判定“新目标”的终点位置阈值（m）的默认值。
///
/// 小于该值视为同一目标的路径刷新；大于该值说明导航目标变了，必须开启新会话。
/// 实际取值来自参数 `goal_change_tolerance`（默认 0.10 m）。
/// 原来的硬编码值是 **1 mm**，比全局规划器每次刷新的末点抖动还小，导致同一个任务在
/// 接近终点时被反复判成“新目标”，把轨迹、热启动与重规划状态一起清空
/// （2026-10-09 实车日志：两次 “Reached the goal” 之前各多出一次会话重置）。
/// **已知限制**：取消后重新下发**完全相同坐标**的目标会得到同一个会话编号，
/// 无法与普通路径刷新区分；真正的 Action 级会话守护属于方案 §8.3 的 P4 工作。
constexpr double kGoalChangeThresholdMeters = 0.10;

}  // namespace

MincoMpcController::~MincoMpcController()
{
  worker_.stop();
  state_adapter_.reset();
}

void MincoMpcController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & _parent, std::string _name,
  std::shared_ptr<tf2_ros::Buffer> _tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> _costmap_ros)
{
  parent_ = _parent;
  plugin_name_ = _name;
  tf_ = std::move(_tf);
  costmap_ros_ = std::move(_costmap_ros);

  auto node = parent_.lock();
  if (!node) {
    throw MincoMpcControllerError("MincoMpcController: parent node is not available");
  }

  std::string reason;
  if (!loadParameters(&reason)) {
    throw MincoMpcControllerError("MincoMpcController: " + reason);
  }
  if (!configureSolver(&reason)) {
    throw MincoMpcControllerError("MincoMpcController: " + reason);
  }

  StateAdapter::Config state_config;
  state_config.odometry_topic = odom_topic_;
  state_config.base_frame = base_frame_;
  state_config.planning_frame = planning_frame_;
  state_config.timeout = state_timeout_;
  state_config.max_sample_gap = state_max_sample_gap_;
  state_config.max_position_jump = state_max_position_jump_;
  state_config.max_yaw_jump = state_max_yaw_jump_;
  state_config.velocity_filter_alpha = state_velocity_filter_alpha_;
  if (!state_adapter_.configure(node, tf_, state_config, &reason)) {
    throw MincoMpcControllerError("MincoMpcController: state adapter: " + reason);
  }

  CostmapAdapter::Config costmap_config;
  costmap_config.unknown_is_obstacle = esdf_config_.unknown_is_seed;
  if (!costmap_adapter_.configure(costmap_ros_, costmap_config, &reason)) {
    throw MincoMpcControllerError("MincoMpcController: costmap adapter: " + reason);
  }

  YawPolicy::Config yaw_config;
  yaw_config.mode = yaw_mode_;
  yaw_config.stop_rotation_on_goal = stop_rotation_on_goal_;
  yaw_config.max_angular_speed = base_limits_.max_angular_speed;
  yaw_config.max_angular_accel = base_limits_.max_angular_accel;
  yaw_config.narrow_clearance_threshold = yaw_narrow_clearance_threshold_;
  yaw_config.allow_reverse = yaw_allow_reverse_;
  if (!yaw_policy_.configure(yaw_config, &reason)) {
    throw MincoMpcControllerError("MincoMpcController: yaw policy: " + reason);
  }
  // 航向策略决定 MPC 的角速度权限：`xy_only` 下必须为 0，否则求解器仍然可以产生
  // 自转，就与“阶段一 MPC 不控自转、自转由独立链路负责”的契约不一致（方案 §8.1）。
  limits_.max_angular_speed = yaw_policy_.angularSpeedLimit();
  limits_.max_angular_accel = yaw_policy_.angularAccelLimit();
  if (!mpc_solver_.setLimits(limits_, &reason)) {
    throw MincoMpcControllerError("MincoMpcController: effective limits: " + reason);
  }

  if (!reference_builder_.configure(reference_config_, &reason)) {
    throw MincoMpcControllerError("MincoMpcController: tracking reference: " + reason);
  }

  const srm27_minco_core::TrajectoryValidatorConfig validator_config = makeValidatorConfig();
  if (!validator_.configure(validator_config, &reason)) {
    throw MincoMpcControllerError("MincoMpcController: trajectory validator: " + reason);
  }

  srm27_minco_core::ReplanManager::Config replan_config;
  replan_config.replan_period = 1.0 / replan_frequency_;
  replan_config.map_change_min_period = 1.0 / (2.0 * replan_frequency_);
  replan_config.trajectory_max_age = trajectory_max_age_;
  replan_config.prefix_reuse_min_duration = initializer_config_.nominal_piece_duration;
  if (!replan_manager_.configure(replan_config, &reason)) {
    throw MincoMpcControllerError("MincoMpcController: replan manager: " + reason);
  }

  trajectory_pub_ =
    node->create_publisher<nav_msgs::msg::Path>(plugin_name_ + "/minco_trajectory", rclcpp::QoS(1));
  prediction_pub_ =
    node->create_publisher<nav_msgs::msg::Path>(plugin_name_ + "/mpc_prediction", rclcpp::QoS(1));
  diagnostics_pub_ = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    plugin_name_ + "/diagnostics", rclcpp::QoS(1));
  planning_input_pub_ = node->create_publisher<nav_msgs::msg::Path>(
    plugin_name_ + "/planning_input_path", rclcpp::QoS(1));
  // 脱困可视化：逃逸方向箭头 + 当前净空球。与轨迹/预测分开成独立话题，
  // 现场可以直接只开这一条来确认"卡死时分诊是否成立、推的方向是否合理"。
  recovery_marker_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
    plugin_name_ + "/recovery_markers", rclcpp::QoS(1));

  // 自转请求进线：发布端（rotation_controller）是 RELIABLE，订阅端必须同为
  // RELIABLE 才能匹配（尽力订阅匹配不上可靠发布）。回调只缓存数值与到达时刻
  // （steady 时钟），新鲜度判定在控制循环里做，避免回调里锁生命周期节点。
  rclcpp::QoS spin_request_qos(rclcpp::KeepLast(1));
  spin_request_qos.reliable();
  spin_request_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
    spin_request_topic_, spin_request_qos,
    [this](geometry_msgs::msg::Twist::SharedPtr _msg) {
      const double wz = _msg ? _msg->angular.z : 0.0;
      std::lock_guard<std::mutex> lock(spin_request_mutex_);
      spin_request_value_ = std::isfinite(wz) ? wz : 0.0;
      spin_request_stamp_ = steadyNow();
    });

  configured_ = true;
  RCLCPP_INFO(
    node->get_logger(),
    "MincoMpcController 已配置: planning_frame=%s base_frame=%s command_frame=%s(%s) "
    "horizon=%.2f m replan=%.1f Hz v_max=%.2f m/s a_max=%.2f m/s^2 yaw_mode=%s "
    "wz 权限=%.3f rad/s spin_request=%s(超时 %.2f s)",
    planning_frame_.c_str(), base_frame_.c_str(), command_frame_.c_str(),
    toString(command_frame_mode_), planning_horizon_, replan_frequency_,
    limits_.max_linear_speed, limits_.max_linear_accel, yaw_policy_.modeName(),
    limits_.max_angular_speed, spin_request_topic_.c_str(), spin_request_timeout_);
  if (command_frame_mode_ == CommandFrameMode::kOdomAlignedFake) {
    RCLCPP_INFO(
      node->get_logger(),
      "MincoMpcController: command_frame=%s 与 odom 对齐 —— 控制器**不做** R(-yaw)，"
      "平移速度原样输出，唯一的坐标转换由 fake_vel_transform 负责（方案 §3.2）。",
      command_frame_.c_str());
  }
  RCLCPP_INFO(
    node->get_logger(),
    "MincoMpcController 脱困参数: fail_threshold=%d cooldown=%.2f s window=%.2f s "
    "escape_speed=%.2f m/s stuck_clearance=%.2f m escaped_clearance=%.2f m "
    "search=radius %.2f m/%d samples",
    recovery_fail_threshold_, recovery_cooldown_sec_, recovery_window_sec_,
    recovery_escape_speed_, recovery_stuck_clearance_, recovery_escaped_clearance_,
    recovery_search_radius_, recovery_search_samples_);
}

bool MincoMpcController::loadParameters(std::string * _reason)
{
  const auto fail = [&_reason](const std::string & _message) {
    if (_reason != nullptr) {
      *_reason = _message;
    }
    return false;
  };

  auto node = parent_.lock();
  if (!node) {
    return fail("parent node is not available");
  }
  const std::string & name = plugin_name_;
  const auto declare = [&node, &name](const std::string & _suffix, const auto & _default_value) {
    declare_parameter_if_not_declared(
      node, name + "." + _suffix, rclcpp::ParameterValue(_default_value));
  };

  declare("planning_frame", std::string("odom"));
  declare("base_frame", std::string("base_link"));
  // 速度命令的坐标系（方案 §3.2）：
  //   command_frame == base_frame      -> 输出真实车体系速度（现状；由本控制器做 R(-psi)）
  //   command_frame == fake_base_frame -> 输出 odom 对齐的平移速度（fakeyaw 迁移；
  //                                        唯一的转换在 fake_vel_transform 里做）
  declare("command_frame", std::string("base_link"));
  declare("fake_base_frame", std::string("base_link_fake"));
  declare("odom_topic", std::string("odometry"));
  declare("replan_frequency", 10.0);
  declare("planning_horizon", 2.0);
  declare("planning_deadline_ms", 30.0);
  declare("state_timeout", 0.10);
  declare("map_timeout", 0.30);
  declare("trajectory_max_age", 0.30);
  declare("build_grace_period", 1.0);
  declare("diagnostics_period", 0.5);
  declare("publish_visualization", true);
  declare("terminal_reached_radius", 0.20);
  // 同一导航任务内的路径刷新与"真正的换目标"必须区分：末点抖动超过这个距离才算新目标。
  declare("goal_change_tolerance", kGoalChangeThresholdMeters);

  // 终点急停（见 terminal_stop.hpp）：一进成功区域就立即给零速，不再追踪末点。
  declare("terminal.enabled", true);
  declare("terminal.tolerance", 0.0);
  declare("terminal.exit_margin", 0.15);

  declare("minco.polynomial_order", 5);
  declare("minco.two_stage_optimization", true);
  declare("minco.enable_report_fine_heuristic", false);
  declare("minco.nominal_piece_duration", 0.30);
  declare("minco.min_piece_duration", 0.05);
  declare("minco.max_piece_duration", 1.00);
  declare("minco.pre_time_ratio_min", 0.90);
  declare("minco.pre_time_ratio_max", 1.10);
  declare("minco.samples_per_piece", 8);
  declare("minco.max_iterations", 150);
  declare("minco.gradient_tolerance", 1.0e-4);
  declare("minco.lbfgs_past", 3);
  declare("minco.w_jerk", 1.0);
  declare("minco.w_time", 10.0);
  declare("minco.w_obstacle", 100.0);
  declare("minco.w_velocity", 20.0);
  declare("minco.w_acceleration", 2.0);
  declare("minco.w_time_ratio", 1.0);
  declare("minco.soft_hinge_beta", 20.0);
  declare("minco.optimization_clearance_margin", 0.05);
  declare("minco.resample_step", 0.10);
  declare("minco.corner_speed_ratio", 0.35);
  declare("minco.min_pieces", 2);
  declare("minco.max_pieces", 12);
  declare("minco.terminal_speed", 0.0);

  declare("limits.max_linear_speed", 0.50);
  declare("limits.max_linear_accel", 0.30);
  declare("limits.max_angular_speed", 0.30);
  declare("limits.max_angular_accel", 0.50);

  declare("mpc.model", "velocity_integrator");
  declare("mpc.prediction_dt", 0.02);
  declare("mpc.prediction_steps", 30);
  declare("mpc.two_pass_reference", false);
  declare("mpc.command_lookahead", 0.20);
  declare("mpc.qp_deadline_ms", 12.0);
  declare("mpc.use_hot_start", true);
  declare("mpc.q_position", 20.0);
  declare("mpc.q_yaw", 5.0);
  declare("mpc.q_velocity", 1.0);
  declare("mpc.q_omega", 0.5);
  declare("mpc.r_translation", 1.0);
  declare("mpc.r_angular", 1.0);
  declare("mpc.r_input_change", 0.0);
  declare("mpc.use_tangent_normal_weight", false);
  declare("mpc.q_along_track", 20.0);
  declare("mpc.q_cross_track", 40.0);
  declare("mpc.speed_polygon_sides", 8);
  declare("mpc.enforce_input_change", true);

  declare("yaw_policy.mode", std::string("xy_only"));
  declare("yaw_policy.stop_rotation_on_goal", true);
  declare("yaw_policy.narrow_clearance_threshold", 0.60);
  declare("yaw_policy.allow_reverse", true);

  // 自转请求进线（阶段二小陀螺，方案 §4.1.2）：`kSpin` 模式下把外部请求喂给
  // 航向策略；话题默认接旧链路自转速度链（已含 wz_max / 加速度限幅）。
  declare("spin_request.topic", std::string("rotation_velocity"));
  declare("spin_request.timeout", 0.25);

  declare("safety.robot_radius", 0.33);
  declare("safety.clearance_margin", 0.05);
  declare("safety.unknown_is_obstacle", true);
  declare("safety.braking_deceleration", 0.0);
  // 有效反应延迟（s）。<=0 表示退化为使用 state_timeout。
  declare("safety.reaction_latency", 0.0);

  declare("state.max_sample_gap", 0.20);
  declare("state.max_position_jump", 0.75);
  declare("state.max_yaw_jump", 1.20);
  declare("state.velocity_filter_alpha", 0.35);

  // 脱困（迁移自 navi_minco_bit 的 `MincoPlanner.recovery_server.*` +
  // MincoFsm 里硬编码的 0.25 / 0.40 分诊与成功阈值）。
  declare("recovery.fail_threshold", 5);
  declare("recovery.cooldown_sec", 1.0);
  declare("recovery.window_sec", 3.0);
  declare("recovery.escape_speed", 0.5);
  declare("recovery.stuck_clearance", 0.25);
  declare("recovery.escaped_clearance", 0.40);
  declare("recovery.search_radius", 0.4);
  declare("recovery.search_samples", 36);
  declare("recovery.publish_visualization", true);

  node->get_parameter(name + ".planning_frame", planning_frame_);
  node->get_parameter(name + ".base_frame", base_frame_);
  node->get_parameter(name + ".command_frame", command_frame_);
  node->get_parameter(name + ".fake_base_frame", fake_base_frame_);
  node->get_parameter(name + ".odom_topic", odom_topic_);
  node->get_parameter(name + ".replan_frequency", replan_frequency_);
  node->get_parameter(name + ".planning_horizon", planning_horizon_);
  node->get_parameter(name + ".planning_deadline_ms", planning_deadline_ms_);
  node->get_parameter(name + ".state_timeout", state_timeout_);
  node->get_parameter(name + ".map_timeout", map_timeout_);
  node->get_parameter(name + ".trajectory_max_age", trajectory_max_age_);
  node->get_parameter(name + ".build_grace_period", build_grace_period_);
  node->get_parameter(name + ".diagnostics_period", diagnostics_period_);
  node->get_parameter(name + ".publish_visualization", publish_visualization_);
  node->get_parameter(name + ".terminal_reached_radius", terminal_reached_radius_);
  node->get_parameter(name + ".goal_change_tolerance", goal_change_tolerance_);
  node->get_parameter(name + ".terminal.enabled", terminal_stop_config_.enabled);
  node->get_parameter(name + ".terminal.tolerance", terminal_stop_config_.tolerance);
  node->get_parameter(name + ".terminal.exit_margin", terminal_stop_config_.exit_margin);
  // 目标检查器容差不可用时的兜底：沿用既有的到点半径语义。
  terminal_stop_config_.fallback_tolerance = terminal_reached_radius_;
  terminal_stop_.configure(terminal_stop_config_);

  node->get_parameter(name + ".minco.polynomial_order", polynomial_order_);
  node->get_parameter(name + ".minco.two_stage_optimization", two_stage_optimization_);
  node->get_parameter(
    name + ".minco.enable_report_fine_heuristic", minco_config_.enable_report_fine_heuristic);
  node->get_parameter(
    name + ".minco.nominal_piece_duration", initializer_config_.nominal_piece_duration);
  node->get_parameter(name + ".minco.min_piece_duration", minco_config_.min_piece_duration);
  node->get_parameter(name + ".minco.max_piece_duration", minco_config_.max_piece_duration);
  node->get_parameter(name + ".minco.pre_time_ratio_min", minco_config_.pre_time_ratio_min);
  node->get_parameter(name + ".minco.pre_time_ratio_max", minco_config_.pre_time_ratio_max);
  node->get_parameter(name + ".minco.samples_per_piece", minco_config_.samples_per_piece);
  node->get_parameter(name + ".minco.max_iterations", minco_config_.max_iterations);
  node->get_parameter(name + ".minco.gradient_tolerance", minco_config_.gradient_tolerance);
  node->get_parameter(name + ".minco.lbfgs_past", minco_config_.lbfgs_past);
  node->get_parameter(name + ".minco.w_jerk", minco_config_.w_jerk);
  node->get_parameter(name + ".minco.w_time", minco_config_.w_time);
  node->get_parameter(name + ".minco.w_obstacle", minco_config_.w_obstacle);
  node->get_parameter(name + ".minco.w_velocity", minco_config_.w_velocity);
  node->get_parameter(name + ".minco.w_acceleration", minco_config_.w_acceleration);
  node->get_parameter(name + ".minco.w_time_ratio", minco_config_.w_time_ratio);
  node->get_parameter(name + ".minco.soft_hinge_beta", minco_config_.soft_hinge_beta);
  node->get_parameter(
    name + ".minco.optimization_clearance_margin", minco_config_.optimization_clearance_margin);
  node->get_parameter(name + ".minco.resample_step", initializer_config_.resample_step);
  node->get_parameter(name + ".minco.corner_speed_ratio", initializer_config_.corner_speed_ratio);
  node->get_parameter(name + ".minco.min_pieces", initializer_config_.min_pieces);
  node->get_parameter(name + ".minco.max_pieces", initializer_config_.max_pieces);
  node->get_parameter(name + ".minco.terminal_speed", initializer_config_.terminal_speed);

  node->get_parameter(name + ".limits.max_linear_speed", base_limits_.max_linear_speed);
  node->get_parameter(name + ".limits.max_linear_accel", base_limits_.max_linear_accel);
  node->get_parameter(name + ".limits.max_angular_speed", base_limits_.max_angular_speed);
  node->get_parameter(name + ".limits.max_angular_accel", base_limits_.max_angular_accel);

  node->get_parameter(name + ".mpc.model", mpc_model_name_);
  node->get_parameter(name + ".mpc.prediction_dt", prediction_dt_);
  node->get_parameter(name + ".mpc.prediction_steps", prediction_steps_);
  node->get_parameter(name + ".mpc.two_pass_reference", two_pass_reference_);
  node->get_parameter(name + ".mpc.command_lookahead", command_lookahead_);
  node->get_parameter(name + ".mpc.qp_deadline_ms", qp_deadline_ms_);
  node->get_parameter(name + ".mpc.use_hot_start", mpc_config_.use_hot_start);
  node->get_parameter(name + ".mpc.q_position", mpc_config_.q_position);
  node->get_parameter(name + ".mpc.q_yaw", mpc_config_.q_yaw);
  node->get_parameter(name + ".mpc.q_velocity", mpc_config_.q_velocity);
  node->get_parameter(name + ".mpc.q_omega", mpc_config_.q_omega);
  node->get_parameter(name + ".mpc.r_translation", mpc_config_.r_translation);
  node->get_parameter(name + ".mpc.r_angular", mpc_config_.r_angular);
  node->get_parameter(name + ".mpc.r_input_change", mpc_config_.r_input_change);
  node->get_parameter(
    name + ".mpc.use_tangent_normal_weight", mpc_config_.use_tangent_normal_weight);
  node->get_parameter(name + ".mpc.q_along_track", mpc_config_.q_along_track);
  node->get_parameter(name + ".mpc.q_cross_track", mpc_config_.q_cross_track);
  node->get_parameter(name + ".mpc.speed_polygon_sides", mpc_config_.speed_polygon_sides);
  node->get_parameter(name + ".mpc.enforce_input_change", mpc_config_.enforce_input_change);

  std::string yaw_mode_text;
  node->get_parameter(name + ".yaw_policy.mode", yaw_mode_text);
  node->get_parameter(name + ".yaw_policy.stop_rotation_on_goal", stop_rotation_on_goal_);
  node->get_parameter(
    name + ".yaw_policy.narrow_clearance_threshold", yaw_narrow_clearance_threshold_);
  node->get_parameter(name + ".yaw_policy.allow_reverse", yaw_allow_reverse_);

  node->get_parameter(name + ".spin_request.topic", spin_request_topic_);
  node->get_parameter(name + ".spin_request.timeout", spin_request_timeout_);
  if (!(spin_request_timeout_ > 0.0) || !std::isfinite(spin_request_timeout_)) {
    return fail("spin_request.timeout must be positive and finite");
  }
  if (spin_request_topic_.empty()) {
    return fail("spin_request.topic must not be empty");
  }

  node->get_parameter(name + ".safety.robot_radius", minco_config_.robot_radius);
  node->get_parameter(name + ".safety.clearance_margin", minco_config_.clearance_margin);
  node->get_parameter(name + ".safety.unknown_is_obstacle", esdf_config_.unknown_is_seed);
  node->get_parameter(name + ".safety.braking_deceleration", braking_deceleration_);
  node->get_parameter(name + ".safety.reaction_latency", reaction_latency_);

  node->get_parameter(name + ".state.max_sample_gap", state_max_sample_gap_);
  node->get_parameter(name + ".state.max_position_jump", state_max_position_jump_);
  node->get_parameter(name + ".state.max_yaw_jump", state_max_yaw_jump_);
  node->get_parameter(name + ".state.velocity_filter_alpha", state_velocity_filter_alpha_);

  // 脱困参数。`configure` 内部会把非法值夹到安全默认并强制
  // escaped_clearance > stuck_clearance，所以这里不做额外校验、不因参数失败而拒绝启动。
  {
    srm27_minco_core::RecoveryConfig recovery_config;
    node->get_parameter(name + ".recovery.fail_threshold", recovery_config.fail_threshold);
    node->get_parameter(name + ".recovery.cooldown_sec", recovery_config.cooldown_sec);
    node->get_parameter(name + ".recovery.window_sec", recovery_config.window_sec);
    node->get_parameter(name + ".recovery.escape_speed", recovery_config.escape_speed);
    node->get_parameter(name + ".recovery.stuck_clearance", recovery_config.stuck_clearance);
    node->get_parameter(name + ".recovery.escaped_clearance", recovery_config.escaped_clearance);
    node->get_parameter(name + ".recovery.search_radius", recovery_config.search_radius);
    node->get_parameter(name + ".recovery.search_samples", recovery_config.search_samples);
    recovery_decider_.configure(recovery_config);
    // 把夹取后的值回写到成员，供 `handlePlanningFailure` 的分诊判定与日志使用，
    // 避免"日志里打印的阈值"和"实际生效的阈值"不是同一个数。
    const srm27_minco_core::RecoveryConfig & effective = recovery_decider_.config();
    recovery_fail_threshold_ = effective.fail_threshold;
    recovery_cooldown_sec_ = effective.cooldown_sec;
    recovery_window_sec_ = effective.window_sec;
    recovery_escape_speed_ = effective.escape_speed;
    recovery_stuck_clearance_ = effective.stuck_clearance;
    recovery_escaped_clearance_ = effective.escaped_clearance;
    recovery_search_radius_ = effective.search_radius;
    recovery_search_samples_ = effective.search_samples;
  }
  node->get_parameter(name + ".recovery.publish_visualization", recovery_publish_visualization_);

  if (!parseYawMode(yaw_mode_text, yaw_mode_)) {
    return fail("unsupported yaw_policy.mode: " + yaw_mode_text);
  }
  if (!resolveCommandFrame(
        command_frame_, base_frame_, fake_base_frame_, command_frame_mode_, _reason)) {
    return false;
  }
  if (polynomial_order_ != Trajectory2D::kPolynomialOrder) {
    return fail("only the quintic MINCO representation (polynomial_order: 5) is supported");
  }
  if (!(replan_frequency_ > 0.0) || !std::isfinite(replan_frequency_)) {
    return fail("replan_frequency must be positive");
  }
  if (!(planning_horizon_ > 0.0) || !std::isfinite(planning_horizon_)) {
    return fail("planning_horizon must be positive");
  }
  if (!(planning_deadline_ms_ > 0.0) || !std::isfinite(planning_deadline_ms_)) {
    return fail("planning_deadline_ms must be positive");
  }
  if (!(state_timeout_ > 0.0) || !(map_timeout_ > 0.0) || !(trajectory_max_age_ > 0.0)) {
    return fail("state_timeout / map_timeout / trajectory_max_age must be positive");
  }
  if (!(build_grace_period_ >= 0.0) || !std::isfinite(build_grace_period_)) {
    return fail("build_grace_period must be non-negative");
  }
  if (!(command_lookahead_ >= 0.0) || !std::isfinite(command_lookahead_)) {
    return fail("command_lookahead must be non-negative");
  }
  if (!prediction_steps_ || prediction_steps_ < 1) {
    return fail("mpc.prediction_steps must be at least 1");
  }
  if (!(prediction_dt_ > 0.0) || !std::isfinite(prediction_dt_)) {
    return fail("mpc.prediction_dt must be positive");
  }
  if (!(qp_deadline_ms_ > 0.0) || !std::isfinite(qp_deadline_ms_)) {
    return fail("mpc.qp_deadline_ms must be positive");
  }
  if (
    mpc_model_name_ != "velocity_integrator" &&
    mpc_model_name_ != "acceleration_double_integrator") {
    return fail("unsupported mpc.model: " + mpc_model_name_);
  }
  if (!base_limits_.valid()) {
    return fail("limits.* must be finite and positive");
  }
  if (!std::isfinite(braking_deceleration_) || braking_deceleration_ < 0.0) {
    return fail(
      "safety.braking_deceleration must be non-negative (0 = use limits.max_linear_accel)");
  }
  if (!std::isfinite(reaction_latency_) || reaction_latency_ < 0.0) {
    return fail("safety.reaction_latency must be non-negative (0 = use state_timeout)");
  }
  if (!(goal_change_tolerance_ > 0.0) || !std::isfinite(goal_change_tolerance_)) {
    return fail("goal_change_tolerance must be positive");
  }
  // 终点急停：tolerance 允许为 0（表示"用目标检查器容差"），只拒绝非有限与负数 ——
  // 负值会被静默改写，宁可让配置错误在启动时就暴露。
  const TerminalStop::Config & terminal = terminal_stop_config_;
  if (
    !std::isfinite(terminal.tolerance) || terminal.tolerance < 0.0 ||
    !std::isfinite(terminal.exit_margin) || terminal.exit_margin < 0.0) {
    return fail("terminal.tolerance / terminal.exit_margin must be non-negative");
  }

  // MINCO 与前端共用段时长/权重配置。
  minco_config_.two_stage = two_stage_optimization_;
  initializer_config_.min_piece_duration = minco_config_.min_piece_duration;
  initializer_config_.max_piece_duration = minco_config_.max_piece_duration;
  initializer_config_.duplicate_epsilon = 1.0e-3;
  // 前端与验证器统一使用同一套有效约束（方案 §7.2 的“约束一致性”）。
  initializer_config_.max_speed = base_limits_.max_linear_speed;
  initializer_config_.max_accel = base_limits_.max_linear_accel;
  initializer_config_.max_brake = base_limits_.max_linear_accel;

  return true;
}

srm27_minco_core::TrajectoryValidatorConfig MincoMpcController::makeValidatorConfig() const
{
  srm27_minco_core::TrajectoryValidatorConfig config;
  config.robot_radius = minco_config_.robot_radius;
  config.clearance_margin = minco_config_.clearance_margin;
  config.max_linear_speed = limits_.max_linear_speed;
  config.max_linear_accel = limits_.max_linear_accel;
  config.min_piece_duration = minco_config_.min_piece_duration;
  config.unknown_is_obstacle = esdf_config_.unknown_is_seed;
  config.map_timeout = map_timeout_;
  config.trajectory_max_age = trajectory_max_age_;
  config.required_prefix_duration = prediction_steps_ * prediction_dt_;
  // 制动能力必须取"实测可保证"的值（方案 §6.1）：未单独标定时才退化为使用加速度上限。
  config.braking_deceleration =
    (braking_deceleration_ > 0.0) ? braking_deceleration_ : base_limits_.max_linear_accel;
  // reaction_latency 与 state_timeout 在早期实现里是同一个参数。实车把两者拆开了：
  // state_timeout 必须不小于里程计到达周期（实车约 10 Hz），而 reaction_latency 是
  // "从下发命令到车真正动起来"的延迟，量级只有 0.1 s 左右。混用会让终点短轨迹被
  // 一律拒绝：2026-10-09 实车日志里 "有效前缀=0.2611s 需要=0.6s" 就是这个后果 ——
  // 短停车轨迹能否通过取决于 `totalDuration >= reaction_latency + v/a_brake`，
  // reaction_latency 取 0.25 时 0.26 s 的终点轨迹只允许 v <= 0.033 m/s。
  config.reaction_latency = (reaction_latency_ > 0.0) ? reaction_latency_ : state_timeout_;
  return config;
}

bool MincoMpcController::configureSolver(std::string * _reason)
{
  srm27_minco_core::MpcModel::Config model_config;
  model_config.type = (mpc_model_name_ == "acceleration_double_integrator")
                        ? srm27_minco_core::MpcModelType::kAccelerationDoubleIntegrator
                        : srm27_minco_core::MpcModelType::kVelocityIntegrator;
  model_config.prediction_steps = prediction_steps_;
  model_config.prediction_dt = prediction_dt_;

  mpc_config_.qp_deadline_ms = qp_deadline_ms_;
  mpc_config_.max_linear_accel = base_limits_.max_linear_accel;
  mpc_config_.max_angular_accel = base_limits_.max_angular_accel;

  limits_ = base_limits_;
  if (!mpc_solver_.configure(model_config, mpc_config_, _reason)) {
    return false;
  }
  mpc_model_ = mpc_solver_.model();

  reference_config_.two_pass_reference = two_pass_reference_;
  return true;
}

void MincoMpcController::activate()
{
  active_ = true;
  // 生命周期重新激活后必须由新的 setPlan 开启会话，避免沿用旧的 goal_epoch_。
  goal_epoch_ = 0;
  build_start_stamp_ = -1.0;
  stopping_ = false;
  stop_reason_ = "none";
  terminal_stop_.reset();
  // 生命周期重新激活等于全新的会话：脱困计数与冷却起点都必须清零，
  // 否则一次旧的脱困会把新任务的首次卡死判定挡在冷却窗口之外。
  recovery_decider_.reset();
  recovery_direction_.setZero();
  recovery_escape_velocity_.setZero();
  recovery_entry_clearance_ = -1.0;
  recovery_command_count_ = 0;
  if (trajectory_pub_) {
    trajectory_pub_->on_activate();
  }
  if (prediction_pub_) {
    prediction_pub_->on_activate();
  }
  if (planning_input_pub_) {
    planning_input_pub_->on_activate();
  }
  if (diagnostics_pub_) {
    diagnostics_pub_->on_activate();
  }
  if (recovery_marker_pub_) {
    recovery_marker_pub_->on_activate();
  }
  // 预热 QP：把冷启动（约 8~15 ms）挪出 50 Hz 控制回调。
  std::string warm_reason;
  if (mpc_solver_.warmUp()) {
    RCLCPP_INFO((parent_.lock())->get_logger(), "MincoMpcController: QP 求解器预热完成");
  } else {
    RCLCPP_WARN(
      (parent_.lock())->get_logger(),
      "MincoMpcController: QP 预热未成功 (%s)，将在控制周期内冷启动", warm_reason.c_str());
  }
  worker_.start();
}

void MincoMpcController::deactivate()
{
  active_ = false;
  // 先停掉工作线程，保证退出后不再回写结果，再作废授权与轨迹。
  worker_.stop();
  worker_.clear();
  invalidateTrajectory("deactivated", 0.0);
  yaw_policy_.clearSpinRequest();
  reference_builder_.reset();
  replan_manager_.reset();
  state_adapter_.reset();
  terminal_stop_.reset();
  recovery_decider_.reset();
  has_previous_input_ = false;
  has_last_control_stamp_ = false;
  if (trajectory_pub_) {
    trajectory_pub_->on_deactivate();
  }
  if (prediction_pub_) {
    prediction_pub_->on_deactivate();
  }
  if (planning_input_pub_) {
    planning_input_pub_->on_deactivate();
  }
  if (diagnostics_pub_) {
    diagnostics_pub_->on_deactivate();
  }
  if (recovery_marker_pub_) {
    recovery_marker_pub_->on_deactivate();
  }
}

void MincoMpcController::cleanup()
{
  deactivate();
  worker_.clear();
  trajectory_pub_.reset();
  prediction_pub_.reset();
  planning_input_pub_.reset();
  diagnostics_pub_.reset();
  {
    std::lock_guard<std::mutex> lock(path_mutex_);
    raw_path_ = nav_msgs::msg::Path();
    has_raw_path_ = false;
  }
  planning_path_.clear();
  has_planning_path_ = false;
  esdf_.reset();
  configured_ = false;
}

void MincoMpcController::setPlan(const nav_msgs::msg::Path & _path)
{
  if (!configured_) {
    throw MincoMpcControllerError("MincoMpcController::setPlan called before configure()");
  }
  if (_path.poses.empty()) {
    RCLCPP_WARN((parent_.lock())->get_logger(), "MincoMpcController: 收到空路径，忽略本次 setPlan");
    return;
  }

  const auto node = parent_.lock();
  const double now_stamp = node ? node->now().seconds() : 0.0;
  const geometry_msgs::msg::Point & new_goal_point = _path.poses.back().pose.position;

  bool new_goal = false;
  std::string change_reason;
  double previous_distance = 0.0;
  {
    std::lock_guard<std::mutex> lock(path_mutex_);
    const bool same_source_frame = path_source_frame_ == _path.header.frame_id && has_raw_path_;
    // “同一目标 3 Hz 刷新”与“新目标”必须区分：前者保留可用轨迹，后者必须有新的会话编号，
    // 否则迟到的旧会话结果会重新激活运动（方案 §4.4）。
    if (!has_raw_path_) {
      new_goal = true;
      change_reason = "first path";
    } else if (!same_source_frame) {
      new_goal = true;
      change_reason =
        "path frame changed (" + path_source_frame_ + " -> " + _path.header.frame_id + ")";
    } else if (!raw_path_.poses.empty()) {
      const geometry_msgs::msg::Point & previous_goal_point = raw_path_.poses.back().pose.position;
      previous_distance = std::hypot(
        new_goal_point.x - previous_goal_point.x, new_goal_point.y - previous_goal_point.y);
      new_goal = previous_distance > goal_change_tolerance_;
      // 阈值必须明显大于全局规划器每次刷新的末点抖动（Theta* 的栅格量化本身就有厘米级），
      // 否则同一个任务会被反复判成"新目标"、清空轨迹/热启动/重规划状态。
      // 2026-10-09 实车日志里两次"Reached the goal"之前各多出一次会话重置，
      // 用的就是原来的 1 mm 阈值。
      if (new_goal) {
        change_reason = "path endpoint moved " + std::to_string(previous_distance) + " m";
      }
    }

    raw_path_ = _path;
    path_source_frame_ = _path.header.frame_id;
    has_raw_path_ = true;
    ++path_version_;
    if (!same_source_frame) {
      // 首帧或坐标系变化：清空已经转换好的局部路径。
      has_planning_path_ = false;
    }
  }

  if (new_goal) {
    // 新会话：作废轨迹、热启动、重规划状态、终点急停与参考进度。
    ++goal_epoch_;
    replan_manager_.reset();
    terminal_stop_.reset();
    // 脱困状态属于"上一个目标"：换目标时必须整体复位，对应参考实现里
    // `MincoFsm` 在收到新目标时调用 `recovery_server_->setMissionGoal()`（内部 reset）。
    // 否则一次旧目标的脱困会把新目标的失败计数与冷却窗口一起继承过来。
    recovery_decider_.reset();
    recovery_direction_.setZero();
    recovery_escape_velocity_.setZero();
    recovery_entry_clearance_ = -1.0;
    recovery_command_count_ = 0;
    invalidateTrajectory("new navigation goal", now_stamp);
    if (node) {
      // 记录"为什么"是新会话：只报一句"新会话 #N"在现场无法区分
      // "用户换了目标"、"规划器末点抖动"还是"坐标系变了"。
      RCLCPP_INFO(
        node->get_logger(),
        "MincoMpcController: 新导航会话 #%lu (路径版本 %lu, 原因: %s; 末点 %.3f, %.3f, frame=%s)",
        static_cast<unsigned long>(goal_epoch_), static_cast<unsigned long>(path_version_),
        change_reason.c_str(), new_goal_point.x, new_goal_point.y, _path.header.frame_id.c_str());
    }
  } else {
    // 同一目标的路径刷新：保留当前有效轨迹与控制进度，只递增路径版本。
    if (node) {
      RCLCPP_DEBUG(
        node->get_logger(),
        "MincoMpcController: 路径刷新 (版本 %lu)，保留当前轨迹 (末点移动 %.4f m, 阈值 %.3f m)",
        static_cast<unsigned long>(path_version_), previous_distance, goal_change_tolerance_);
    }
  }

  // 每条到达控制器的路径都计数。脱困看门狗（abortStaleRecovery）靠它区分
  // "同一会话里两次 setPlan 之间" 与 "中间目标被取消、控制器一度没被调用"。
  // 放在最后：上面的分支有早退，计数必须只在真的收下这条路径时递增。
  ++plan_delivery_count_;
}

bool MincoMpcController::updatePathInPlanningFrame(double _now_stamp, std::string * _reason)
{
  nav_msgs::msg::Path path;
  std::string source_frame;
  {
    std::lock_guard<std::mutex> lock(path_mutex_);
    if (!has_raw_path_) {
      if (_reason != nullptr) {
        *_reason = "no path has been set";
      }
      return false;
    }
    path = raw_path_;
    source_frame = path_source_frame_;
  }
  if (path.poses.empty()) {
    if (_reason != nullptr) {
      *_reason = "path is empty";
    }
    return false;
  }

  const auto fail = [&_reason](const std::string & _message) {
    if (_reason != nullptr) {
      *_reason = _message;
    }
    return false;
  };

  std::vector<Eigen::Vector2d> points;
  points.reserve(path.poses.size());

  const bool needs_transform = !source_frame.empty() && source_frame != planning_frame_;
  if (needs_transform) {
    if (!tf_) {
      return fail("TF buffer unavailable for path frame conversion");
    }
    try {
      // 每次都用最新 TF：重定位导致 map->odom 跳变时需要重新转换（方案 §5.1）。
      const geometry_msgs::msg::TransformStamped transform =
        tf_->lookupTransform(planning_frame_, source_frame, tf2::TimePointZero);
      tf2::Transform conversion;
      tf2::fromMsg(transform.transform, conversion);
      for (const geometry_msgs::msg::PoseStamped & pose : path.poses) {
        tf2::Vector3 point(pose.pose.position.x, pose.pose.position.y, 0.0);
        const tf2::Vector3 converted = conversion * point;
        points.emplace_back(converted.x(), converted.y());
      }
    } catch (const tf2::TransformException & exception) {
      // 不做“退化成单位变换”的静默回退。
      return fail(std::string("TF lookup failed for path conversion: ") + exception.what());
    }
  } else {
    for (const geometry_msgs::msg::PoseStamped & pose : path.poses) {
      points.emplace_back(pose.pose.position.x, pose.pose.position.y);
    }
  }

  // 去掉重复点，避免前端初值出现零长线段。
  std::vector<Eigen::Vector2d> cleaned;
  cleaned.reserve(points.size());
  for (const Eigen::Vector2d & point : points) {
    if (!point.allFinite()) {
      continue;
    }
    if (!cleaned.empty() && (point - cleaned.back()).norm() < 1.0e-4) {
      continue;
    }
    cleaned.push_back(point);
  }
  if (cleaned.size() < 2) {
    return fail("path degenerates to a single point");
  }
  planning_path_ = std::move(cleaned);
  has_planning_path_ = true;
  (void)_now_stamp;
  return true;
}

void MincoMpcController::refreshMap(double _stamp, const ControllerDiagnostics & _diagnostics)
{
  (void)_diagnostics;
  std::string reason;
  srm27_minco_core::GridSnapshot snapshot;
  std::uint64_t version = 0;
  if (!costmap_adapter_.snapshot(_stamp, snapshot, version, &reason)) {
    RCLCPP_WARN_THROTTLE(
      (parent_.lock())->get_logger(), *(parent_.lock())->get_clock(), 2000,
      "MincoMpcController: 无法取得代价地图快照: %s", reason.c_str());
    return;
  }

  const bool version_changed = version != map_version_;
  auto esdf = std::make_shared<srm27_minco_core::Esdf2D>();
  if (!esdf->build(snapshot, esdf_config_)) {
    RCLCPP_WARN_THROTTLE(
      (parent_.lock())->get_logger(), *(parent_.lock())->get_clock(), 2000,
      "MincoMpcController: 距离场建立失败");
    return;
  }
  esdf_ = std::move(esdf);
  esdf_received_steady_ = steadyNow();
  map_version_ = version;
  map_stamp_ = _stamp;
  // 地图变化只**标记**需要在最近一次节流窗口到期时复验，不强制立刻复验。
  // 原先这里直接把 last_full_revalidation_stamp_ 置 -1，等于让下面那个 0.2 s 的节流失效：
  // 滚动局部地图 + 地形层的版本号每周期都在变，于是每个控制周期都跑一次全轨迹复验
  // （含多项式求根与全段扫掠），叠加 QP 后出现 15.6 ms 的控制超时。
  if (version_changed) {
    map_changed_since_validation_ = true;
  }
}

void MincoMpcController::invalidateTrajectory(const std::string & _reason, double _now_stamp)
{
  std::atomic_store(&trajectory_, std::shared_ptr<const Trajectory2D>());
  has_warm_start_ = false;
  warm_inner_points_.clear();
  warm_durations_.clear();
  reference_builder_.reset();
  has_previous_input_ = false;
  stopping_ = true;
  at_goal_stop_ = false;
  stop_reason_ = _reason;
  stop_request_stamp_ = _now_stamp;
  output_zero_stamp_ = _now_stamp;
  if (worker_.running()) {
    worker_.clear();
  }
  // 本函数代表"一次新的建轨尝试"（新目标、定位重置、限速变化、复验失败、失活）。
  // 必须把建轨宽限计时清零：否则一次失败留下的旧起点会让下一次尝试在几毫秒内就
  // 超出 build_grace_period 而立刻抛异常——现场现象就是"脱困成功后 FollowPath 约 1 ms
  // 就再次失败"，BT 因此陷入 recovery 循环。
  build_start_stamp_ = -1.0;
}

void MincoMpcController::updateWarmStart(const Trajectory2D & _trajectory)
{
  const int pieces = _trajectory.pieceCount();
  if (pieces < 2) {
    has_warm_start_ = false;
    return;
  }
  warm_inner_points_.clear();
  warm_inner_points_.reserve(static_cast<std::size_t>(pieces - 1));
  double accumulated = 0.0;
  for (int i = 0; i + 1 < pieces; ++i) {
    accumulated += _trajectory.durations()[static_cast<std::size_t>(i)];
    warm_inner_points_.push_back(_trajectory.positionAt(accumulated));
  }
  warm_durations_ = _trajectory.durations();
  has_warm_start_ = true;
}

// -----------------------------------------------------------------------------
// 脱困（迁移自 navi_minco_bit 的 RecoverServer + MincoFsm::RECOVERING）
//
// 参考实现的链路是：
//   FSM 重规划失败 -> RecoverServer::handleReplanFailure（判定 + 算方向）
//   -> FSM 切 RECOVERING -> 每周期 publishEscapeCommand（伪轨迹 /opt_path）
//   -> 自家 MPC 跟踪伪轨迹 -> cmd_vel
//   -> RECOVERING 每周期判 ESDF > 0.40 成功 / 3 s 超时
//
// 本工作区没有 FSM、也没有 /opt_path：控制器的唯一速度出口是 computeVelocityCommands
// 的返回值。因此三类职责的落点是：
//   * **判定**：`harvestPlanningResult` 里每次规划失败调用 `handlePlanningFailure`；
//   * **脱困路径规划**：`RecoveryDecider` 用 ESDF 解析梯度（退化时环形搜索）算方向；
//   * **执行**：`updateRecovery` 在控制周期里直接返回逃逸 Twist，**旁路 MPC**——
//     等价于参考实现"伪轨迹持续覆盖 MPC 参考"的效果，但少了一层跟踪误差；
//   * **成功判定**：同一个 `updateRecovery`。
// -----------------------------------------------------------------------------

bool MincoMpcController::handlePlanningFailure(const State2D & _state, double _now_stamp)
{
  if (!_state.valid || !esdf_) {
    return false;
  }

  // 诊断必须**先于**任何可能提前返回的分支刷新。
  //
  // `computeVelocityCommands` 在 step 3 重置 `diagnostics_`、step 5 调本函数、
  // step 5.5 才由 `updateRecovery` 改写脱困字段。所以凡是本函数直接返回的分支，
  // 发布出去的就是 step 5.5 之后的值；只有在这里把"当前真实状态"写全，
  // "本周期刚脱困成功但重规划又失败一次"这种情形才不会报出错的 recovery_active。
  diagnostics_.recovery_active = recovery_decider_.isRecovering();
  diagnostics_.recovery_failures = recovery_decider_.consecutiveFailures();
  diagnostics_.recovery_exit = recovery_decider_.lastExit();

  // 已经在脱困里：本周期该由 `updateRecovery` 推进，不能再累加失败或重新决策。
  if (recovery_decider_.isRecovering()) {
    return true;
  }

  const srm27_minco_core::EsdfQueryResult here = esdf_->query(_state.x, _state.y);
  const double clearance = here.distance;
  diagnostics_.recovery_clearance = clearance;

  // 卡死分诊（参考实现里硬编码在 MincoFsm 的 `dist < 0.25`）：
  // 净空充足却规划失败，属于"前方路障"，应该让重规划绕路，而不是把车推离原地。
  // 只有自身净空不足才算"被卡住"，这时绕路本身也无解，才需要脱困。
  // ESDF 查询失败（在地图外 / 距离场未建立）时同样不计失败：那种情况下
  // `RecoveryDecider` 也一定算不出方向，计数只会白白把门限耗掉。
  if (!here.valid || !std::isfinite(clearance) || clearance >= recovery_stuck_clearance_) {
    return false;
  }

  Eigen::Vector2d escape_velocity = Eigen::Vector2d::Zero();
  const srm27_minco_core::RecoveryDecision decision =
    recovery_decider_.decide(_now_stamp, _state.position(), *esdf_, &escape_velocity);

  diagnostics_.recovery_failures = recovery_decider_.consecutiveFailures();
  if (decision == srm27_minco_core::RecoveryDecision::kNone) {
    return false;
  }
  if (decision == srm27_minco_core::RecoveryDecision::kNoEscapeDirection) {
    // 没方向就不进脱困：没有方向的脱困等于原地不动，只会把可恢复的卡死变成永久卡死。
    RCLCPP_WARN_THROTTLE(
      (parent_.lock())->get_logger(), *(parent_.lock())->get_clock(), 2000,
      "MincoMpcController: 卡死 (净空 %.3f m < %.2f m) 但距离场给不出逃逸方向，"
      "保持原状态继续重试（不作脱困、不抛异常）",
      clearance, recovery_stuck_clearance_);
    return false;
  }

  // kEscape：进入脱困。
  recovery_escape_velocity_ = escape_velocity;
  recovery_direction_ = escape_velocity;
  if (recovery_direction_.norm() > 1e-9) {
    recovery_direction_.normalize();
  }
  recovery_entry_clearance_ = clearance;
  recovery_command_count_ = 0;
  ++recovery_enter_count_;
  // 建轨宽限必须清零：脱困期间不会有新轨迹，宽限计时留着会让"脱困窗口"
  // 还没走完就先把 FollowPath 判失败抛给 BT，两条 recovery 链路会互相打架。
  build_start_stamp_ = -1.0;

  diagnostics_.recovery_active = true;
  diagnostics_.recovery_trigger = recovery_decider_.lastTrigger();
  diagnostics_.recovery_entry_clearance = recovery_entry_clearance_;
  diagnostics_.recovery_enter_count = recovery_enter_count_;
  diagnostics_.recovery_direction_x = recovery_direction_.x();
  diagnostics_.recovery_direction_y = recovery_direction_.y();
  diagnostics_.recovery_escape_vx = recovery_escape_velocity_.x();
  diagnostics_.recovery_escape_vy = recovery_escape_velocity_.y();
  RCLCPP_WARN(
    (parent_.lock())->get_logger(),
    "MincoMpcController: 进入脱困（触发=%s，进入净空 %.3f m < %.2f m，方向 %.1f deg，速度 %.2f m/s，"
    "窗口 %.2f s，成功阈值 %.2f m）",
    recovery_decider_.lastTrigger().c_str(), clearance, recovery_stuck_clearance_,
    std::atan2(recovery_direction_.y(), recovery_direction_.x()) * 180.0 / M_PI,
    recovery_escape_speed_, recovery_window_sec_, recovery_escaped_clearance_);
  return true;
}

geometry_msgs::msg::TwistStamped MincoMpcController::makeEscapeCommand(
  const State2D & _state, const Eigen::Vector2d & _escape_velocity,
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & _node)
{
  // 逃逸速度是规划坐标系（odom）下的世界速度，必须转成车体系 —— Nav2 的
  // TwistStamped 语义是车体系，直接塞世界速度会让车在非零航向下走错方向。
  Eigen::Vector2d velocity_odom = _escape_velocity;
  diagnostics_.applied_speed_scale = 1.0;
  const double speed = velocity_odom.norm();
  if (speed > limits_.max_linear_speed) {
    diagnostics_.applied_speed_scale = limits_.max_linear_speed / speed;
    velocity_odom *= diagnostics_.applied_speed_scale;
  }
  const Eigen::Vector2d velocity_body = srm27_minco_core::odomToBodyVelocity(
    _state.yaw, velocity_odom);

  geometry_msgs::msg::TwistStamped command;
  command.header.stamp = _node->now();
  command.header.frame_id = base_frame_;
  command.twist.linear.x = velocity_body.x();
  command.twist.linear.y = velocity_body.y();
  command.twist.linear.z = 0.0;
  command.twist.angular.x = 0.0;
  command.twist.angular.y = 0.0;
  // 脱困只平移、不自转：与参考实现"yaw 伪轨迹锁存当前 yaw"语义一致。
  command.twist.angular.z = 0.0;

  // 把实际下发的命令记成"上一条命令"，退出脱困后 MPC 的首步差分约束
  // 才有正确的基准（否则会拿脱困前的旧命令去算加速度变化量）。
  if (!has_previous_input_ || previous_applied_input_.size() != mpc_model_.inputDim()) {
    previous_applied_input_ = Eigen::VectorXd::Zero(mpc_model_.inputDim());
  }
  previous_applied_input_(0) = velocity_odom.x();
  previous_applied_input_(1) = velocity_odom.y();
  previous_applied_input_(2) = 0.0;
  has_previous_input_ = true;
  last_control_stamp_ = _node->now().seconds();
  has_last_control_stamp_ = true;

  diagnostics_.requested_vx = command.twist.linear.x;
  diagnostics_.requested_vy = command.twist.linear.y;
  diagnostics_.requested_wz = 0.0;
  diagnostics_.command_owner = "recovery";
  diagnostics_.stop_reason = "none";
  stopping_ = false;
  return command;
}

bool MincoMpcController::updateRecovery(
  const State2D & _state, double _now_stamp,
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & _node,
  geometry_msgs::msg::TwistStamped * _command)
{
  if (!recovery_decider_.isRecovering()) {
    return false;
  }
  if (_command == nullptr) {
    return false;
  }

  // 成功判据用的自身净空：单帧、单点、位置判据，与参考实现一致。
  // 查询失败（NaN / 地图外）不得判成功 —— 那是"不知道"，不是"脱困了"。
  double clearance = std::numeric_limits<double>::quiet_NaN();
  if (esdf_) {
    const srm27_minco_core::EsdfQueryResult here = esdf_->query(_state.x, _state.y);
    if (here.valid && std::isfinite(here.distance)) {
      clearance = here.distance;
    }
  }
  diagnostics_.recovery_clearance = clearance;

  Eigen::Vector2d escape_velocity = Eigen::Vector2d::Zero();
  // 逃逸方向由 decider 按**当前位置**每周期重算（对应参考实现 FSM 在 RECOVERING 里
  // 每次 tick 都调用 calculateEscapeVelocity），所以必须把当前位置与距离场传进去。
  // 距离场暂时不可用时传一个空对象：decider 会保留上一周期的方向，而不是输出零速。
  const srm27_minco_core::Esdf2D empty_esdf;
  const srm27_minco_core::RecoveryExit exit = recovery_decider_.updateRecovery(
    _now_stamp, _state.position(), esdf_ ? *esdf_ : empty_esdf, clearance, &escape_velocity,
    /*_log_transitions=*/recovery_command_count_ == 0);

  if (exit != srm27_minco_core::RecoveryExit::kNone) {
    leaveRecovery(srm27_minco_core::toString(exit), _now_stamp);
    if (exit == srm27_minco_core::RecoveryExit::kEscaped) {
      RCLCPP_INFO(
        _node->get_logger(),
        "MincoMpcController: 脱困成功（净空 %.3f m > %.2f m，用时 %.2f s，输出 %lu 个周期）",
        clearance, recovery_escaped_clearance_,
        _now_stamp - recovery_decider_.lastRecoveryStartSec(),
        static_cast<unsigned long>(recovery_command_count_));
    } else {
      RCLCPP_WARN(
        _node->get_logger(),
        "MincoMpcController: 脱困超时（%.2f s 窗口内净空只到 %.3f m，成功阈值 %.2f m，"
        "输出 %lu 个周期），返回重规划；若仍失败将由建轨宽限抛给行为树兜底",
        recovery_window_sec_, clearance, recovery_escaped_clearance_,
        static_cast<unsigned long>(recovery_command_count_));
    }
    // 本周期不接管：让正常链路（重规划请求 / 建轨宽限 / BT）立刻接手，
    // 避免"脱困已结束但本周期还在推车"的多推一个周期。
    return false;
  }

  recovery_escape_velocity_ = escape_velocity;
  if (recovery_escape_velocity_.norm() > 1e-9) {
    recovery_direction_ = recovery_escape_velocity_.normalized();
  }
  ++recovery_command_count_;
  *_command = makeEscapeCommand(_state, recovery_escape_velocity_, _node);

  diagnostics_.recovery_active = true;
  diagnostics_.recovery_trigger = recovery_decider_.lastTrigger();
  diagnostics_.recovery_entry_clearance = recovery_entry_clearance_;
  diagnostics_.recovery_command_count = recovery_command_count_;
  diagnostics_.recovery_enter_count = recovery_enter_count_;
  diagnostics_.recovery_stale_abort_count = recovery_stale_abort_count_;
  diagnostics_.recovery_direction_x = recovery_direction_.x();
  diagnostics_.recovery_direction_y = recovery_direction_.y();
  diagnostics_.recovery_escape_vx = recovery_escape_velocity_.x();
  diagnostics_.recovery_escape_vy = recovery_escape_velocity_.y();
  diagnostics_.command_age = 0.0;

  if (recovery_publish_visualization_) {
    publishRecoveryDebug(_node, _state, clearance);
  }
  return true;
}

void MincoMpcController::leaveRecovery(const char * _exit_reason, double _now_stamp)
{
  // 刻意**不**在这里调用 `finishRecovery`：脱困会话已经由 `RecoveryDecider::updateRecovery`
  // 用同一个 `_now_stamp` 结束并打好了冷却参考点，这里再调一次只会把同一个时刻写第二遍，
  // 且会让"谁结束了这次脱困"这件事在两个地方各有一份真相。
  (void)_now_stamp;
  recovery_escape_velocity_.setZero();
  recovery_direction_.setZero();
  // 脱困期间的命令与 MPC 的命令不连续，退出时必须让 MPC 重新建立差分基准。
  has_previous_input_ = false;
  has_last_control_stamp_ = false;
  // "脱困后必须重新获得完整建轨宽限"：否则一次脱困留下的旧计时会让重规划
  // 在毫秒内就超出宽限抛给 BT，现场就是"脱困成功但 FollowPath 立刻又失败"。
  build_start_stamp_ = -1.0;
  diagnostics_.recovery_active = false;
  diagnostics_.recovery_exit = (_exit_reason != nullptr) ? _exit_reason : "unknown";
  diagnostics_.recovery_command_count = recovery_command_count_;
  diagnostics_.recovery_enter_count = recovery_enter_count_;
  recovery_command_count_ = 0;
  recovery_entry_clearance_ = -1.0;
}

bool MincoMpcController::abortStaleRecovery(double _now_stamp, double _call_gap_sec)
{
  // 只认一个判据：**两帧 computeVelocityCommands 之间的间隔异常大**。
  //
  // 为什么不能用"还没输出过指令"（`recovery_command_count_ == 0`）当判据 —— 那是错的，
  // 而且已经实测踩过：进入脱困的**那一周期本身**必然还没输出（本函数在 updateRecovery
  // 之前跑），于是看门狗会在正常脱困里每个周期都把会话掐掉，脱困永远发不出速度。
  // 2026-10-17 实测现象：0.5 s 内反复「进入脱困 / 脱困停滞复位」共 17 次，
  // 最后靠人把车挪走才报"脱困成功（输出 0 个周期）"。
  //
  // 间隔为什么可靠：控制器只在导航进行中被 Nav2 调用；目标一取消就不再被调用。
  // 所以"间隔 >> 控制周期"是"中间目标被取消过"的唯一可靠证据。
  if (!recovery_decider_.isRecovering()) {
    return false;
  }
  if (!std::isfinite(_call_gap_sec) || _call_gap_sec <= 0.0) {
    return false;   // 没有上一帧记录，无从判断
  }
  // 50 Hz 控制周期是 0.02 s，正常抖动远小于它；但**不能取得太小**：
  // 行为树 recovery（清代价地图 + BackUpFreeSpace）会把 FollowPath 整个停掉，
  // 那一整段控制器都不被调用，而目标其实还活着。实测那次 backup 单次约 5 s、
  // 连续 6 次约 30 s。阈值太小会把"BT recovery 期间"误判成"目标被取消"，
  // 从而把一次本来有效的脱困会话复位掉。
  // 取 2.0 s：既能覆盖真实的目标取消（实测那次取消空档 6.13 s），
  // 又不会被 BT recovery 的正常停顿误触发。
  constexpr double kStaleGapSec = 2.0;
  if (_call_gap_sec < kStaleGapSec) {
    return false;
  }

  ++recovery_stale_abort_count_;
  RCLCPP_WARN(
    (parent_.lock())->get_logger(),
    "MincoMpcController: 脱困停滞复位 —— 两帧控制间隔 %.2f s 远超控制周期"
    "（阈值 %.2f s），说明中间导航目标被取消过（Nav2 在无目标时不调用控制器），"
    "而上一次脱困会话还留在状态里。本会话作废并整体复位，避免它在下一个目标到来时"
    "凭空报「脱困成功」。本次会话已输出 %lu 个周期（累计第 %lu 次）",
    _call_gap_sec, kStaleGapSec,
    static_cast<unsigned long>(recovery_command_count_),
    static_cast<unsigned long>(recovery_stale_abort_count_));
  leaveRecovery("stale_gap", _now_stamp);
  // leaveRecovery 只清控制器侧运行期状态；decider 的会话（窗口起点、失败计数、冷却）
  // 必须一起复位，否则下一个目标会继承上一段的失败计数与冷却窗口。
  recovery_decider_.reset();
  diagnostics_.recovery_failures = 0;
  return true;
}

void MincoMpcController::publishRecoveryDebug(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & _node, const State2D & _state,
  double _clearance)
{
  if (!recovery_marker_pub_ || !recovery_marker_pub_->is_activated()) {
    return;
  }
  visualization_msgs::msg::MarkerArray markers;
  const rclcpp::Time stamp = _node->now();

  // 当前净空球：绿=已过成功阈值，黄=仍在卡死分诊区内，红=更危险。
  visualization_msgs::msg::Marker clearance_marker;
  clearance_marker.header.frame_id = planning_frame_;
  clearance_marker.header.stamp = stamp;
  clearance_marker.ns = "recovery_clearance";
  clearance_marker.id = 0;
  clearance_marker.type = visualization_msgs::msg::Marker::SPHERE;
  clearance_marker.action = visualization_msgs::msg::Marker::ADD;
  clearance_marker.pose.position.x = _state.x;
  clearance_marker.pose.position.y = _state.y;
  clearance_marker.pose.position.z = 0.0;
  clearance_marker.pose.orientation.w = 1.0;
  const double clamped_clearance =
    (std::isfinite(_clearance) && _clearance > 0.0) ? std::min(_clearance, 2.0) : 0.05;
  clearance_marker.scale.x = 2.0 * clamped_clearance;
  clearance_marker.scale.y = 2.0 * clamped_clearance;
  clearance_marker.scale.z = 0.05;
  if (std::isfinite(_clearance) && _clearance > recovery_escaped_clearance_) {
    clearance_marker.color.r = 0.0f;
    clearance_marker.color.g = 1.0f;
  } else if (std::isfinite(_clearance) && _clearance >= recovery_stuck_clearance_) {
    clearance_marker.color.r = 1.0f;
    clearance_marker.color.g = 1.0f;
  } else {
    clearance_marker.color.r = 1.0f;
    clearance_marker.color.g = 0.0f;
  }
  clearance_marker.color.b = 0.0f;
  clearance_marker.color.a = 0.35f;
  markers.markers.push_back(clearance_marker);

  // 逃逸方向箭头：长度按"逃逸速度 × 脱困窗口"画，现场一眼看出这一轮能推多远。
  visualization_msgs::msg::Marker arrow;
  arrow.header.frame_id = planning_frame_;
  arrow.header.stamp = stamp;
  arrow.ns = "recovery_escape";
  arrow.id = 1;
  arrow.type = visualization_msgs::msg::Marker::ARROW;
  arrow.action = visualization_msgs::msg::Marker::ADD;
  arrow.scale.x = 0.06;
  arrow.scale.y = 0.12;
  arrow.scale.z = 0.12;
  arrow.color.r = 0.0f;
  arrow.color.g = 0.6f;
  arrow.color.b = 1.0f;
  arrow.color.a = 1.0f;
  geometry_msgs::msg::Point start;
  start.x = _state.x;
  start.y = _state.y;
  start.z = 0.0;
  geometry_msgs::msg::Point end = start;
  const double reach = recovery_escape_speed_ * recovery_window_sec_;
  end.x += recovery_direction_.x() * reach;
  end.y += recovery_direction_.y() * reach;
  arrow.points.push_back(start);
  arrow.points.push_back(end);
  markers.markers.push_back(arrow);

  // 脱困路径（绿线）：把"沿脱困方向场一路前推"的轨迹画出来 —— 这就是脱困路径规划的结果。
  //
  // 与上面的箭头不是一回事，两者都要留着：
  //  * 箭头 = 本周期实际要执行的那一步（方向 × 逃逸速度 × 脱困窗口），是"命令"；
  //  * 绿线 = 如果一直按每周期重算的方向走会走出什么轨迹，是"路径规划的结果"。
  // 因为方向随位置重算，绿线通常不是直线。现场判读方式：
  //   绿线绕开障碍 -> 梯度/环形搜索在工作，脱困方向合理；
  //   绿线也是一条撞墙的直线 -> 距离场里确实没有出口，该去查地图而不是查本逻辑。
  //
  // 本函数只在 `updateRecovery` 判定本周期继续脱困时被调用，所以这里必然处于
  // 脱困中；只额外护一下距离场（理论上非空，但不要靠"理论上"去解引用）。
  if (esdf_) {
    std::vector<Eigen::Vector2d> escape_path;
    // 步长取逃逸速度在一个控制周期里走过的距离量级（0.1 m ≈ 0.5 m/s @ 200 ms）；
    // 步数上限按"窗口内理论可达距离"再留一倍余量，避免画出窗口内根本走不完的线。
    const double reach = recovery_escape_speed_ * recovery_window_sec_;
    constexpr double kPathStep = 0.1;
    const int max_steps = static_cast<int>(std::ceil(2.0 * reach / kPathStep));
    srm27_minco_core::RecoveryDecider::traceEscapePath(
      *esdf_, Eigen::Vector2d(_state.x, _state.y), recovery_decider_.config(), kPathStep,
      max_steps, &escape_path);
    if (escape_path.size() >= 2) {
      visualization_msgs::msg::Marker path_marker;
      path_marker.header.frame_id = planning_frame_;
      path_marker.header.stamp = stamp;
      path_marker.ns = "recovery_escape_path";
      path_marker.id = 2;
      path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      path_marker.action = visualization_msgs::msg::Marker::ADD;
      // scale.x 是线宽（m）。0.06 是 RViz 里常见可见宽度：再细在缩小的视野下会看不清。
      path_marker.scale.x = 0.06;
      path_marker.color.r = 0.0f;
      path_marker.color.g = 1.0f;
      path_marker.color.b = 0.0f;
      path_marker.color.a = 1.0f;
      path_marker.pose.orientation.w = 1.0;
      path_marker.points.reserve(escape_path.size());
      for (const Eigen::Vector2d & point : escape_path) {
        geometry_msgs::msg::Point p;
        p.x = point.x();
        p.y = point.y();
        p.z = 0.0;
        path_marker.points.push_back(p);
      }
      markers.markers.push_back(path_marker);
    }
  }

  recovery_marker_pub_->publish(markers);
}

void MincoMpcController::requestReplan(
  const State2D & _state, double _now_stamp, double _now_steady)
{
  srm27_minco_core::ReplanRequest request;
  request.versions.goal_epoch = goal_epoch_;
  request.versions.path_version = path_version_;
  request.versions.map_version = map_version_;
  request.versions.limits_version = limits_version_;
  request.state_stamp = _state.sample_stamp;
  request.now_stamp = _now_stamp;
  request.has_goal = goal_epoch_ != 0;
  request.has_path = has_planning_path_;
  const auto trajectory = std::atomic_load(&trajectory_);
  request.previous_trajectory_reusable = static_cast<bool>(trajectory) && !stopping_;
  request.projection_reliable = _state.valid;
  request.state_position = _state.position();
  request.has_state_position = _state.valid;
  request.trajectory_progress = reference_builder_.progress();

  const srm27_minco_core::ReplanDecision decision = replan_manager_.evaluate(request);
  diagnostics_.replan_type = srm27_minco_core::toString(decision.type);
  if (!decision.allowed) {
    return;
  }
  if (!esdf_) {
    return;
  }
  if (worker_.busy()) {
    // 忙时只保留最新请求：仍提交，工作线程会替换旧任务。
    diagnostics_.missed_deadline_count += 1;
  }

  PlanningRequest planning_request;
  planning_request.versions = request.versions;
  planning_request.path = planning_path_;
  planning_request.state = _state;
  planning_request.limits = limits_;
  planning_request.minco_config = minco_config_;
  planning_request.initializer_config = initializer_config_;
  // 本插件持有的路径是 Nav2 下发的全局路径，其末点即当前 FollowPath 任务的目标。
  planning_request.terminal_is_global_goal = true;
  planning_request.request_stamp = _now_stamp;
  planning_request.request_steady = _now_steady;
  planning_request.local_path_horizon = planning_horizon_;
  planning_request.validity_window = trajectory_max_age_;
  planning_request.validator_config = makeValidatorConfig();
  planning_request.terminal_reached_radius = terminal_reached_radius_;
  // 距离场以只读快照交给工作线程；工作线程不持有 costmap 锁。
  planning_request.esdf = std::const_pointer_cast<const srm27_minco_core::Esdf2D>(esdf_);
  planning_request.use_warm_start = decision.use_warm_start && has_warm_start_;
  planning_request.warm_inner_points = warm_inner_points_;
  planning_request.warm_durations = warm_durations_;
  planning_request.replan_type = static_cast<int>(decision.type);
  worker_.submit(planning_request);
}

void MincoMpcController::harvestPlanningResult(const State2D & _state, double _now_stamp)
{
  PlanningResult result;
  if (!worker_.takeResult(result)) {
    return;
  }
  diagnostics_.esdf_time_ms = result.esdf_time_ms;
  diagnostics_.frontend_time_ms = result.frontend_time_ms;
  diagnostics_.pre_time_ms = result.pre_time_ms;
  diagnostics_.finely_time_ms = result.finely_time_ms;
  diagnostics_.validation_time_ms = result.validation_time_ms;
  diagnostics_.planning_time_ms = result.total_time_ms;
  diagnostics_.planning_result = result.status;
  diagnostics_.terminal_reason = result.terminal_reason;
  diagnostics_.terminal_speed = result.terminal_speed;
  diagnostics_.minimum_clearance = result.min_clearance;
  diagnostics_.maximum_speed = result.max_speed;
  diagnostics_.maximum_acceleration = result.max_acceleration;

  if (!result.success) {
    if (result.status == "already_at_goal") {
      // 良性停车：机器人已经落在路径终点附近，局部路径为空是正常的。
      // 这里既不算控制失败（不计入建轨宽限），也不需要重规划；由 StoppedGoalChecker
      // 判定成功。之前的实现把它报成 no_path 并计入失败，会把"到点"变成控制器异常。
      at_goal_stop_ = true;
      RCLCPP_INFO_THROTTLE(
        (parent_.lock())->get_logger(), *(parent_.lock())->get_clock(), 2000,
        "MincoMpcController: 已到达路径终点，受控停车等待目标检查器判定");
      return;
    }
    RCLCPP_WARN_THROTTLE(
      (parent_.lock())->get_logger(), *(parent_.lock())->get_clock(), 2000,
      "MincoMpcController: 规划失败 (%s): %s", result.status.c_str(), result.reason.c_str());
    // 脱困判定：这是一次真实的规划失败。`handlePlanningFailure` 内部做"卡死 / 前方路障"
    // 分诊与门限判定，达标即进入脱困（下一周期由 `updateRecovery` 接管输出）。
    if (handlePlanningFailure(_state, _now_stamp)) {
      RCLCPP_WARN(
        (parent_.lock())->get_logger(),
        "MincoMpcController: 连续规划失败已达门限（%d 次），本周期起由脱困接管输出",
        recovery_fail_threshold_);
    }
    return;
  }
  // 版本校验：迟到的旧会话结果不得重新激活运动。
  //
  // 注意顺序：先校验版本，再改任何控制状态。原实现把 `at_goal_stop_ = false` 放在校验之前，
  // 于是**任何**成功结果（包括已经过期的旧会话结果）都会解除"到点停车"，
  // 使停车状态无法可靠保持（2026-10-09 实车终点往返诊断 §1）。
  if (
    result.versions.goal_epoch != goal_epoch_ || result.versions.path_version != path_version_ ||
    result.versions.limits_version != limits_version_) {
    diagnostics_.rejected_stale_result_count += 1;
    return;
  }
  if (
    replan_manager_.hasCommittedTrajectory() &&
    result.versions.goal_epoch != replan_manager_.committedVersions().goal_epoch) {
    diagnostics_.rejected_stale_result_count += 1;
    return;
  }
  // 通过版本校验的成功结果到达：清除"到点停车"状态。
  // 终点急停（terminal_stop_）不在这里清：它是位置状态，只由离开成功区域或换目标解除。
  at_goal_stop_ = false;
  std::string reason;
  if (!replan_manager_.canCommit(result.trajectory, result.versions, _now_stamp, &reason)) {
    diagnostics_.rejected_stale_result_count += 1;
    RCLCPP_WARN_THROTTLE(
      (parent_.lock())->get_logger(), *(parent_.lock())->get_clock(), 2000,
      "MincoMpcController: 丢弃候选轨迹: %s", reason.c_str());
    return;
  }

  if (!result.local_path.empty()) {
    last_local_path_ = result.local_path;
    last_local_path_stamp_ = _now_stamp;
  }
  auto trajectory = std::make_shared<Trajectory2D>(result.trajectory);
  std::atomic_store(&trajectory_, std::static_pointer_cast<const Trajectory2D>(trajectory));
  updateWarmStart(*trajectory);
  replan_manager_.onPlanCommitted(result.versions, *trajectory, _now_stamp);
  reference_builder_.reset();
  // 注意：这里**不**清 `has_previous_input_`。命令连续性约束的对象是“上一条实际执行的
  // 命令”，与轨迹编号无关；每次刷新路径都清掉会让首步差分约束忽松忽紧，既让 QP 工作集
  // 大幅跳变（实测可到 ~10 ms），也与方案 §3.5 修正项 3 的本意相反。只有新目标、取消、
  // 限速变化、定位重置与失活才需要清空。
  stopping_ = false;
  stop_reason_ = "none";
  build_start_stamp_ = -1.0;
  // 一条真正提交的轨迹是"规划成功"的唯一证据：清零脱困失败计数。
  // 只在这里清、不在"收到结果"时清，因为被拒的候选轨迹（版本过期 / 提交被拒）
  // 并不代表重规划能力恢复，拿它清计数会让门限永远跨不过去。
  // 脱困会话本身不在这里结束：它由 `updateRecovery` 的成功 / 超时判定结束。
  recovery_decider_.onPlanningSuccess();
  diagnostics_.recovery_failures = recovery_decider_.consecutiveFailures();
  ++trajectory_id_;
  ++switched_trajectory_count_;
  diagnostics_.trajectory_id = trajectory_id_;
  diagnostics_.switched_trajectory_count = switched_trajectory_count_;
  diagnostics_.map_version = map_version_;
  diagnostics_.path_version = path_version_;
  diagnostics_.limits_version = limits_version_;
  diagnostics_.goal_epoch = goal_epoch_;
  RCLCPP_DEBUG(
    (parent_.lock())->get_logger(),
    "MincoMpcController: 提交轨迹 #%lu (段数 %d, 总时长 %.2f s, 净空 %.3f m, 规划 %.2f ms)",
    static_cast<unsigned long>(trajectory_id_), trajectory->pieceCount(),
    trajectory->totalDuration(), result.min_clearance, result.total_time_ms);
}

geometry_msgs::msg::TwistStamped MincoMpcController::stopCommand(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & _node, const std::string & _reason,
  double _now_stamp)
{
  geometry_msgs::msg::TwistStamped command;
  command.header.stamp = _node->now();
  command.header.frame_id = command_frame_;
  command.twist.linear.x = 0.0;
  command.twist.linear.y = 0.0;
  command.twist.linear.z = 0.0;
  command.twist.angular.x = 0.0;
  command.twist.angular.y = 0.0;
  command.twist.angular.z = 0.0;

  stopping_ = true;
  stop_reason_ = _reason;
  output_zero_stamp_ = _now_stamp;
  if (stop_request_stamp_ <= 0.0) {
    stop_request_stamp_ = _now_stamp;
  }
  diagnostics_.stop_reason = _reason;
  diagnostics_.stop_request_time = stop_request_stamp_;
  diagnostics_.output_zero_time = output_zero_stamp_;
  diagnostics_.command_owner = "stop";
  diagnostics_.requested_vx = 0.0;
  diagnostics_.requested_vy = 0.0;
  diagnostics_.requested_wz = 0.0;
  diagnostics_.applied_wz = 0.0;
  diagnostics_.mpc_requested_wz = 0.0;
  diagnostics_.yaw_reference = 0.0;
  diagnostics_.wz_clamp_reason = "stop";
  diagnostics_.command_frame = command_frame_;
  has_previous_input_ = false;
  return command;
}

geometry_msgs::msg::TwistStamped MincoMpcController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & _pose, const geometry_msgs::msg::Twist & _velocity,
  nav2_core::GoalChecker * _goal_checker)
{
  (void)_pose;

  const double control_begin = steadyNow();
  auto node = parent_.lock();
  if (!node) {
    throw MincoMpcControllerError("MincoMpcController: parent node is not available");
  }
  if (!configured_) {
    throw MincoMpcControllerError("MincoMpcController: not configured");
  }
  if (!active_) {
    throw MincoMpcControllerError("MincoMpcController: not active");
  }

  const double now_stamp = node->now().seconds();
  diagnostics_ = ControllerDiagnostics();
  diagnostics_.yaw_mode = yaw_policy_.modeName();
  diagnostics_.command_frame = command_frame_;
  diagnostics_.rejected_spin_request_count = yaw_policy_.rejectedSpinRequestCount();
  diagnostics_.map_version = map_version_;
  diagnostics_.path_version = path_version_;
  diagnostics_.limits_version = limits_version_;
  diagnostics_.goal_epoch = goal_epoch_;
  diagnostics_.trajectory_id = trajectory_id_;

  const auto fail = [this, &node, &now_stamp](const std::string & _reason) {
    diagnostics_.total_control_time_ms = 0.0;
    diagnostics_.planning_result = _reason;
    diagnostics_.stop_reason = _reason;
    const geometry_msgs::msg::TwistStamped command = stopCommand(node, _reason, now_stamp);
    // 失败路径同样要发布诊断：否则现场只能去翻 controller_server 日志才能知道原因，
    // 而 `ros2 topic echo .../diagnostics` 是最顺手的观测入口。
    publishDiagnostics(node, diagnostics_, now_stamp);
    if (build_start_stamp_ < 0.0) {
      build_start_stamp_ = now_stamp;
    }
    if ((now_stamp - build_start_stamp_) > build_grace_period_) {
      // 脱困期间不允许把 FollowPath 判失败：本控制器的脱困（RECOVERING 窗口）与
      // 行为树的 recovery（清代价地图 + BackUpFreeSpace）是两条独立的脱困链路，
      // 同时放出去会让两套速度指令互相打架。这里让宽限在脱困期间持续重置，
      // 等价于"脱困窗口 + 建轨宽限"都走完才交给 BT 兜底。
      if (recovery_decider_.isRecovering()) {
        build_start_stamp_ = now_stamp;
        return command;
      }
      // 超过建轨宽限：交给 BT 处理，而不是永远返回零并假装正常。
      // 本次 FollowPath 也就此结束，下一次调用属于"新的尝试"，必须重新获得完整宽限，
      // 否则恢复行为刚结束就会因为沿用旧计时而在毫秒级再次失败。
      build_start_stamp_ = -1.0;
      throw MincoMpcControllerError("MincoMpcController: " + _reason);
    }
    return command;
  };

  if (goal_epoch_ == 0) {
    return fail("no active navigation session");
  }

  // 1) 一致状态。
  State2D state;
  std::string reason;
  if (!state_adapter_.stateAt(now_stamp, steadyNow(), state, &reason)) {
    return fail("state unavailable: " + reason);
  }
  diagnostics_.state_age = std::max(0.0, now_stamp - state.sample_stamp);

  // 2) 定位重置：作废旧轨迹与热启动。
  if (state_adapter_.resetCount() != last_odom_reset_count_) {
    last_odom_reset_count_ = state_adapter_.resetCount();
    invalidateTrajectory("odometry reset", now_stamp);
    yaw_policy_.reset(state.yaw);
    replan_manager_.reset();
    // 定位跳变后"距离终点多远"这件事本身不再可信，终点急停必须重新判定。
    terminal_stop_.reset();
    RCLCPP_WARN(node->get_logger(), "MincoMpcController: 检测到里程计重置，已作废轨迹");
  }

  // 2.5) 自转请求（阶段二小陀螺）：新鲜请求喂给航向策略，超时视为撤回。
  refreshSpinRequest();

  // 3) 路径转换到规划坐标系。
  if (!updatePathInPlanningFrame(now_stamp, &reason)) {
    return fail("path unavailable: " + reason);
  }

  // 4) 地图与距离场。
  refreshMap(now_stamp, diagnostics_);
  if (!esdf_ || !esdf_->valid()) {
    return fail("distance field unavailable");
  }
  diagnostics_.map_age = std::max(0.0, now_stamp - map_stamp_);
  if (diagnostics_.map_age > map_timeout_) {
    return fail("distance field is older than map_timeout");
  }

  // 5) 取回后台结果（失败会触发脱困判定，所以需要当前状态）。
  harvestPlanningResult(state, now_stamp);

  // 5.5) 脱困接管：判成功 / 判超时；未退出则本周期输出逃逸速度并直接返回。
  //
  // 位置刻意放在"取回结果"之后，让本周期拿到的成功结果有机会先把失败计数清零；
  // 又放在"轨迹复验"与"重规划请求"之前，因为脱困期间运行这两件事没有意义
  // （复验会把已经在脱困的轨迹判死、重规划会和工作线程抢算力）。
  {
    // 5.5.0) 停滞看门狗：收拾"目标被取消、脱困状态却留在那儿"的残局。
    // 必须在 updateRecovery 之前跑 —— 否则冻结的会话会被误判成"成功"（实测 6.13 s
    // 后凭空报成功）。判据是"两帧控制间隔异常大"，细节见 abortStaleRecovery 的注释。
    {
      const double call_gap =
        (last_compute_call_stamp_ >= 0.0) ? (now_stamp - last_compute_call_stamp_) : -1.0;
      last_compute_call_stamp_ = now_stamp;
      abortStaleRecovery(now_stamp, call_gap);
    }

    geometry_msgs::msg::TwistStamped escape_command;
    if (updateRecovery(state, now_stamp, node, &escape_command)) {
      diagnostics_.total_control_time_ms = (steadyNow() - control_begin) * 1.0e3;
      publishDiagnostics(node, diagnostics_, now_stamp);
      return escape_command;
    }
  }

  // 6) 已提交轨迹在最新地图上的复验（版本变化或按固定周期）。
  auto trajectory = std::atomic_load(&trajectory_);
  if (trajectory) {
    diagnostics_.terminal_reason = trajectory->terminal_reason;
    diagnostics_.terminal_speed = trajectory->endVelocity().norm();
    diagnostics_.trajectory_age = std::max(0.0, now_stamp - trajectory->generated_stamp);
    // 复验节流：最多每 0.2 s（且不慢于重规划周期）在最新地图上完整复验一次。
    // 地图在这期间变化过也照样按该节流执行 —— 新轨迹由工作线程在最新地图上验证，
    // 已在执行的轨迹每 0.2 s 复验一次（1.5 m/s 下约 0.3 m 行程）是安全与耗时的折中。
    const double revalidation_period = std::max(0.2, 1.0 / replan_frequency_);
    const bool need_revalidation =
      (last_full_revalidation_stamp_ < 0.0) ||
      ((now_stamp - last_full_revalidation_stamp_) > revalidation_period);
    if (need_revalidation) {
      last_full_revalidation_stamp_ = now_stamp;
      srm27_minco_core::TrajectoryValidationReport report;
      if (!validator_.validate(*trajectory, *esdf_, state, now_stamp, report)) {
        invalidateTrajectory("trajectory revalidation failed: " + report.reason, now_stamp);
        trajectory.reset();
      }
    }
  }

  // 7) 重规划请求。
  requestReplan(state, now_stamp, control_begin);

  // 8) 终点急停：一进入目标检查器的成功区域就立即给零速，不再追踪精确末点。
  //
  // 这一步必须优先于跟踪已提交轨迹。MPC 的代价里一直带着"到精确末点的位置误差"
  // （mpc.q_position），即使车已经进入目标检查器的 xy 容差，控制器也会继续把它往末点上修；
  // 一旦冲过末点就反向修正，再冲、再修正 —— 现场就是终点附近来回走
  // （2026-10-09 实车日志）。
  //
  // 逻辑刻意做得最简单：**进容差 → 立即零速**，没有任何后段控制。
  //  * 进入只看位置、不看速度：轨迹本来就是停在末点的停车剖面，车进容差圈时的速度是
  //    sqrt(2*a*tol)（实车约 1.5 m/s，几乎满速）；任何"先慢下来再接管"的门槛等于永不接管。
  //  * 接管后**不再输出任何朝末点或背向末点的速度**，所以控制器再也不可能主动把车
  //    带出容差 —— 振荡在逻辑上不可能出现。
  //  * 停车距离完全由底盘自己决定（急刹），"停在哪里"取决于底盘减速能力，见 §5 制动辨识。
  //    容差对末点是对称的：在距末点 tol 处给零，滑行 v²/(2*a_chassis) 后停下，
  //    只要 v²/(2*a_chassis) <= 2·tol 就仍落在末点 ±tol 内（实车条件 a >= 1.41 m/s²）。
  //
  // 安全校验全部保留：已提交轨迹的复验（步骤 6）照常执行；规划请求（步骤 7）继续提交，
  // 解除急停时立刻有新鲜轨迹可用。
  {
    double distance_to_goal = std::numeric_limits<double>::infinity();
    if (!planning_path_.empty()) {
      const Eigen::Vector2d position(state.x, state.y);
      distance_to_goal = (planning_path_.back() - position).norm();
    }
    // 进入阈值与目标检查器同源：优先读它的 xy 容差，读不到才退回 terminal_reached_radius。
    double goal_tolerance = 0.0;
    if (_goal_checker != nullptr) {
      geometry_msgs::msg::Pose pose_tolerance;
      geometry_msgs::msg::Twist vel_tolerance;
      if (_goal_checker->getTolerances(pose_tolerance, vel_tolerance)) {
        goal_tolerance = pose_tolerance.position.x;
      }
    }
    const bool terminal_active = terminal_stop_.update(distance_to_goal, goal_tolerance);
    diagnostics_.terminal_distance = distance_to_goal;
    diagnostics_.terminal_active = terminal_active;
    diagnostics_.terminal_tolerance = terminal_stop_.entryTolerance();
    if (terminal_active) {
      RCLCPP_INFO_THROTTLE(
        (parent_.lock())->get_logger(), *(parent_.lock())->get_clock(), 2000,
        "MincoMpcController: 终点急停中 (距末点 %.3f m <= %.3f m)，等待目标检查器判定停稳",
        distance_to_goal, terminal_stop_.entryTolerance());
      geometry_msgs::msg::TwistStamped command = stopCommand(node, "terminal_stop", now_stamp);
      // 小陀螺（kSpin + 新鲜自转请求）: 终点急停只刹平移、保留自转 —— 持续自转是
      // 操作手要的驻车状态（方案 §4.1.2 阶段二）。导航会话结束后 wz 自然停止
      // （mux 零速兜底），不影响安全性。
      if (yaw_mode_ == YawMode::kSpin && diagnostics_.spin_request_fresh) {
        command.twist.angular.z = diagnostics_.spin_request;
      }
      // stopCommand 的 command_owner 统一是 "stop"；这里改成更具体的原因，
      // 现场才能从 diagnostics 直接区分"终点急停"与其它停车。
      diagnostics_.command_owner = "terminal_stop";
      diagnostics_.total_control_time_ms = (steadyNow() - control_begin) * 1.0e3;
      publishDiagnostics(node, diagnostics_, now_stamp);
      return command;
    }
  }

  // 9) 没有可用轨迹：先看是否属于"已经到达终点"的良性情况。
  if (!trajectory) {
    if (at_goal_stop_) {
      // 受控停车：不算控制失败，不计入建轨宽限，也不会抛异常把成功判成失败。
      return stopCommand(node, "at_goal", now_stamp);
    }
    return fail("no validated trajectory available yet");
  }
  if (diagnostics_.trajectory_age > trajectory_max_age_) {
    invalidateTrajectory("trajectory is older than trajectory_max_age", now_stamp);
    return fail("trajectory expired before it could be executed");
  }

  // 9) 跟踪参考。
  yaw_policy_.setCurrentYaw(state.yaw);
  // 到点保持（`se2` 的 stop_rotation_on_goal）：已进入终点急停或已判定到达时，航向参考
  // 冻结，避免"还在追切线 → 车一直在转 → 停稳条件永远不满足"（方案 §4.1.5）。
  if (stop_rotation_on_goal_) {
    yaw_policy_.setGoalHold(terminal_stop_.active() || at_goal_stop_);
  }
  TrackingReferenceResult reference;
  if (!reference_builder_.build(*trajectory, state, mpc_model_, yaw_policy_, reference)) {
    return fail("tracking reference failed: " + reference.message);
  }
  diagnostics_.projection_progress = reference.progress;

  // 10) MPC。
  MpcInitialState initial_state;
  if (mpc_model_.type() == srm27_minco_core::MpcModelType::kVelocityIntegrator) {
    initial_state.z0 = Eigen::VectorXd::Zero(3);
    initial_state.z0 << state.x, state.y,
      srm27_minco_core::unwrapAngleNear(reference.z_ref(2, 0), state.yaw);
  } else {
    initial_state.z0 = Eigen::VectorXd::Zero(6);
    initial_state.z0 << state.x, state.y,
      srm27_minco_core::unwrapAngleNear(reference.z_ref(2, 0), state.yaw), state.velocity.x(),
      state.velocity.y(), state.omega;
  }
  if (has_previous_input_ && previous_applied_input_.size() == mpc_model_.inputDim()) {
    initial_state.previous_applied_input = previous_applied_input_;
    initial_state.has_previous_input = true;
  }
  // 首步差分约束使用真实控制间隔，而不是模型步长（方案 §3.5 修正项 3）。
  initial_state.control_interval =
    has_last_control_stamp_ ? std::max(1.0e-3, now_stamp - last_control_stamp_) : prediction_dt_;

  MpcReference mpc_reference;
  mpc_reference.z_ref = reference.z_ref;
  mpc_reference.u_ref = reference.u_ref;
  mpc_reference.tangent_angle = reference.tangent_angle;

  MpcSolution solution;
  if (!mpc_solver_.solve(initial_state, mpc_reference, solution)) {
    diagnostics_.qp_status = static_cast<int>(solution.status);
    return fail("MPC solve failed: " + solution.message);
  }
  if (solution.deadline_exceeded) {
    diagnostics_.missed_deadline_count += 1;
    RCLCPP_WARN_THROTTLE(
      node->get_logger(), *node->get_clock(), 2000,
      "MincoMpcController: QP 求解 %.2f ms 超过预算 %.2f ms（解仍通过硬约束校验，本周期继续执行）",
      solution.solve_time_ms, qp_deadline_ms_);
  }
  diagnostics_.mpc_solve_time_pass1_ms = solution.solve_time_ms;
  diagnostics_.qp_status = 0;
  diagnostics_.qp_iterations = solution.iterations;
  diagnostics_.max_constraint_violation = solution.max_constraint_violation;

  // 11) 报告式第二次采样与求解（可选）。
  if (two_pass_reference_) {
    Eigen::MatrixXd predicted_velocity(2, mpc_model_.steps());
    if (mpc_model_.type() == srm27_minco_core::MpcModelType::kVelocityIntegrator) {
      predicted_velocity = solution.u.topRows<2>();
    } else {
      predicted_velocity = solution.z.block(3, 1, 2, mpc_model_.steps());
    }
    TrackingReferenceResult second_reference = reference;
    if (reference_builder_.buildSecondPass(
          *trajectory, predicted_velocity, mpc_model_, yaw_policy_, second_reference)) {
      MpcReference second_mpc_reference;
      second_mpc_reference.z_ref = second_reference.z_ref;
      second_mpc_reference.u_ref = second_reference.u_ref;
      second_mpc_reference.tangent_angle = second_reference.tangent_angle;
      MpcSolution second_solution;
      if (
        mpc_solver_.solve(initial_state, second_mpc_reference, second_solution) &&
        second_solution.usable()) {
        // 只有第二次解通过全部硬约束时才采用；否则有界降级为第一次解。
        solution = second_solution;
        reference = second_reference;
        diagnostics_.mpc_solve_time_pass2_ms = second_solution.solve_time_ms;
        if (second_solution.deadline_exceeded) {
          diagnostics_.missed_deadline_count += 1;
        }
      }
    }
  }

  // 12) 取执行时刻的预测速度并转换到命令坐标系（方案 §3.2、§4.1.3）。
  Eigen::VectorXd command;
  if (!mpc_solver_.commandAt(solution, command_lookahead_, command)) {
    return fail("failed to extract the predicted command");
  }
  const int lookahead_index = std::min(
    static_cast<int>(std::floor(command_lookahead_ / mpc_model_.dt())) + 1,
    static_cast<int>(solution.z.cols()) - 1);
  // 两种模型的 yaw 都直接是预测状态的第 3 维；命令生效时刻的 yaw 与命令时间一致。
  const double yaw_at_actuation = solution.z(2, lookahead_index);
  Eigen::Vector2d velocity_odom(command(0), command(1));
  // `base_link` 命令系：在这里做唯一的 R(-psi)；`base_link_fake`（odom 对齐）命令系：
  // 原样输出 odom 平移速度，转换全部留给 fake_vel_transform —— 两边都做就是重复旋转。
  Eigen::Vector2d velocity_command =
    velocityInCommandFrame(velocity_odom, yaw_at_actuation, command_frame_mode_);
  double omega_command = command(2);
  // 方案 §4.1.2 要求记录"请求的 wz"与"限幅原因"，这里先把 MPC 原始请求存下来。
  const double requested_omega = omega_command;
  std::string wz_clamp_reason = "none";

  if (!velocity_command.allFinite() || !std::isfinite(omega_command)) {
    return fail("MPC produced a non-finite command");
  }

  // 13) 输出前检查：合速度、自转与相对上一条命令的变化量。
  //     ⚠ 模长限幅在 odom 系与车体系下等价（纯 yaw 旋转），所以用命令系的速度即可。
  diagnostics_.applied_speed_scale = 1.0;
  const double speed = velocity_command.norm();
  if (speed > limits_.max_linear_speed) {
    const double scale = limits_.max_linear_speed / speed;
    diagnostics_.applied_speed_scale = scale;
    velocity_odom *= scale;
    velocity_command *= scale;
  }
  if (std::abs(omega_command) > limits_.max_angular_speed) {
    omega_command = srm27_minco_core::clampToRange(
      omega_command, -limits_.max_angular_speed, limits_.max_angular_speed);
    // 区分两种限幅原因：策略本身把权限设成 0（xy_only），还是真的撞到速度上限。
    wz_clamp_reason = (limits_.max_angular_speed <= 0.0) ? "yaw_policy_zero_angular_limit"
                                                         : "angular_speed_limit";
  }
  if (has_previous_input_ && previous_applied_input_.size() == mpc_model_.inputDim()) {
    // 平移与自转的变化量不得突破有效加速度上限（下游还有 mux 的末端限幅）。
    const double interval = std::max(1.0e-3, now_stamp - last_control_stamp_);
    const Eigen::Vector2d previous_odom(previous_applied_input_(0), previous_applied_input_(1));
    const Eigen::Vector2d delta = velocity_odom - previous_odom;
    const double max_delta = limits_.max_linear_accel * interval;
    if (delta.norm() > max_delta) {
      velocity_odom = previous_odom + delta.normalized() * max_delta;
      velocity_command = velocityInCommandFrame(velocity_odom, yaw_at_actuation, command_frame_mode_);
    }
    // 下周期的差分基准必须是本周期限幅后的输出，而不是 MPC 未执行的原始请求。
    previous_applied_input_(0) = velocity_odom.x();
    previous_applied_input_(1) = velocity_odom.y();
    previous_applied_input_(2) = omega_command;
  } else {
    previous_applied_input_ = Eigen::VectorXd::Zero(mpc_model_.inputDim());
    previous_applied_input_(0) = velocity_odom.x();
    previous_applied_input_(1) = velocity_odom.y();
    previous_applied_input_(2) = omega_command;
  }
  has_previous_input_ = true;
  last_control_stamp_ = now_stamp;
  has_last_control_stamp_ = true;

  geometry_msgs::msg::TwistStamped command_msg;
  command_msg.header.stamp = node->now();
  command_msg.header.frame_id = command_frame_;
  command_msg.twist.linear.x = velocity_command.x();
  command_msg.twist.linear.y = velocity_command.y();
  command_msg.twist.linear.z = 0.0;
  command_msg.twist.angular.x = 0.0;
  command_msg.twist.angular.y = 0.0;
  command_msg.twist.angular.z = omega_command;

  diagnostics_.requested_vx = command_msg.twist.linear.x;
  diagnostics_.requested_vy = command_msg.twist.linear.y;
  diagnostics_.requested_wz = command_msg.twist.angular.z;
  // 方案 §4.1.2：requested/applied wz、yaw 参考与限幅原因都要能被"事后查账"。
  diagnostics_.applied_wz = omega_command;
  diagnostics_.mpc_requested_wz = requested_omega;
  diagnostics_.yaw_reference = reference.z_ref(2, 0);
  diagnostics_.wz_clamp_reason = wz_clamp_reason;
  diagnostics_.command_frame = command_frame_;
  diagnostics_.command_owner = "navigation";
  diagnostics_.command_age = std::max(0.0, now_stamp - trajectory->generated_stamp);
  diagnostics_.stop_reason = "none";
  if (stopping_) {
    stopping_ = false;
    actual_stop_stamp_ = 0.0;
    diagnostics_.actual_stop_time = actual_stop_stamp_;
  }
  diagnostics_.total_control_time_ms = (steadyNow() - control_begin) * 1.0e3;

  if (publish_visualization_) {
    publishVisualization(node, *trajectory, solution, now_stamp);
  }
  publishDiagnostics(node, diagnostics_, now_stamp);
  return command_msg;
}

void MincoMpcController::setSpeedLimit(const double & _speed_limit, const bool & _percentage)
{
  if (!configured_) {
    return;
  }
  // Nav2 语义：百分比是相对配置最大速度；绝对值是绝对上限；0 表示解除限制。
  if (_percentage) {
    const double scale = srm27_minco_core::clampToRange(_speed_limit / 100.0, 0.0, 1.0);
    limits_ = base_limits_.scaled(scale);
    applied_speed_scale_ = scale;
  } else if (!(_speed_limit > 0.0)) {
    limits_ = base_limits_;
    applied_speed_scale_ = 1.0;
  } else {
    const double scale =
      srm27_minco_core::clampToRange(_speed_limit / base_limits_.max_linear_speed, 0.0, 1.0);
    limits_ = base_limits_.scaled(scale);
    applied_speed_scale_ = scale;
  }

  // 收紧平移时按同一比例缩放自转；`xy_only` 下自转权限仍必须是 0。
  limits_.max_angular_speed = std::min(limits_.max_angular_speed, yaw_policy_.angularSpeedLimit());
  limits_.max_angular_accel = std::min(limits_.max_angular_accel, yaw_policy_.angularAccelLimit());

  ++limits_version_;
  std::string reason;
  if (!mpc_solver_.setLimits(limits_, &reason)) {
    RCLCPP_WARN(
      (parent_.lock())->get_logger(), "MincoMpcController: 设置限速失败: %s", reason.c_str());
  }

  // 已有轨迹若超出新的有效上限，必须立即失效并重规划（不允许把它继续执行下去）。
  auto trajectory = std::atomic_load(&trajectory_);
  if (trajectory) {
    double max_speed = 0.0;
    double max_accel = 0.0;
    if (
      validator_.computeExtrema(*trajectory, max_speed, max_accel) &&
      (max_speed > limits_.max_linear_speed * 1.02 ||
       max_accel > limits_.max_linear_accel * 1.02)) {
      invalidateTrajectory("speed limit tightened below the committed trajectory", 0.0);
    }
  }
  RCLCPP_INFO(
    (parent_.lock())->get_logger(),
    "MincoMpcController: 有效限速更新为 v_max=%.2f m/s, a_max=%.2f m/s^2 (scale=%.2f)",
    limits_.max_linear_speed, limits_.max_linear_accel, applied_speed_scale_);
}

void MincoMpcController::publishVisualization(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & _node, const Trajectory2D & _trajectory,
  const MpcSolution & _solution, double _now_stamp)
{
  (void)_now_stamp;
  if (trajectory_pub_ && trajectory_pub_->is_activated()) {
    nav_msgs::msg::Path path;
    path.header.stamp = _node->now();
    path.header.frame_id = planning_frame_;
    const double sample_dt = 0.05;
    for (double t = 0.0; t <= _trajectory.totalDuration(); t += sample_dt) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      const Eigen::Vector2d position = _trajectory.positionAt(t);
      pose.pose.position.x = position.x();
      pose.pose.position.y = position.y();
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
    trajectory_pub_->publish(path);
  }

  if (planning_input_pub_ && planning_input_pub_->is_activated() && !last_local_path_.empty()) {
    nav_msgs::msg::Path path;
    path.header.stamp = _node->now();
    path.header.frame_id = planning_frame_;
    for (const Eigen::Vector2d & point : last_local_path_) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = point.x();
      pose.pose.position.y = point.y();
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
    planning_input_pub_->publish(path);
  }

  if (prediction_pub_ && prediction_pub_->is_activated() && _solution.usable()) {
    nav_msgs::msg::Path path;
    path.header.stamp = _node->now();
    path.header.frame_id = planning_frame_;
    for (int i = 0; i < _solution.z.cols(); ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = _solution.z(0, i);
      pose.pose.position.y = _solution.z(1, i);
      pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), _solution.z(2, i)));
      path.poses.push_back(pose);
    }
    prediction_pub_->publish(path);
  }
}

void MincoMpcController::refreshSpinRequest()
{
  double value = 0.0;
  double stamp = -1.0;
  {
    std::lock_guard<std::mutex> lock(spin_request_mutex_);
    value = spin_request_value_;
    stamp = spin_request_stamp_;
  }
  const bool fresh = (stamp >= 0.0) && (spin_request_timeout_ > 0.0) &&
                     ((steadyNow() - stamp) <= spin_request_timeout_);
  if (yaw_mode_ != YawMode::kSpin) {
    // 非 SPIN 模式不进线（方案 §4.1.2 的迁移契约）：自转请求只有 `kSpin` 消费；
    // `se2` 模式下旧链路常驻的 0 速流不应刷满"拒绝自转请求"计数（那个计数
    // 留给直接调用 setSpinRequest 的调用方）。
    diagnostics_.spin_request = 0.0;
    diagnostics_.spin_request_fresh = false;
    return;
  }
  if (fresh) {
    yaw_policy_.setSpinRequest(value);
  } else {
    // 请求撤回（发送端停发超时）：按"无请求"处理，自转停。
    yaw_policy_.clearSpinRequest();
  }
  diagnostics_.spin_request = fresh ? value : 0.0;
  diagnostics_.spin_request_fresh = fresh;
}

void MincoMpcController::publishDiagnostics(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & _node, const ControllerDiagnostics & _diag,
  double _now_stamp)
{
  if (!diagnostics_pub_ || !diagnostics_pub_->is_activated()) {
    return;
  }
  if ((_now_stamp - last_diagnostics_stamp_) < diagnostics_period_) {
    return;
  }
  last_diagnostics_stamp_ = _now_stamp;
  const bool healthy = _diag.planning_result == "success" || _diag.planning_result == "idle";
  diagnostic_msgs::msg::DiagnosticArray array =
    toDiagnosticArray(_diag, planning_frame_, healthy ? kDiagnosticOk : kDiagnosticWarn);
  array.header.stamp = _node->now();
  diagnostics_pub_->publish(array);
}

}  // namespace srm27_minco_controller

PLUGINLIB_EXPORT_CLASS(srm27_minco_controller::MincoMpcController, nav2_core::Controller)
