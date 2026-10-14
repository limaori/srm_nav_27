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
#include <stdexcept>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

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
  // 故障注入开关（演练重规划阶梯）：用默认 VOLATILE+RELIABLE，与
  // `ros2 topic pub --once` 的默认 QoS 直接匹配（TRANSIENT_LOCAL 订阅端会
  // 拒绝 VOLATILE 发布者，现场表现为"开关发不进去"）。
  failure_injection_sub_ = node->create_subscription<std_msgs::msg::Bool>(
    plugin_name_ + "/failure_injection", rclcpp::QoS(1),
    [this](const std_msgs::msg::Bool::SharedPtr _msg) {
      failure_injection_.store(_msg->data);
      if (auto locked = parent_.lock()) {
        RCLCPP_WARN(
          locked->get_logger(), "MincoMpcController: 故障注入已%s（重规划阶梯演练）",
          _msg->data ? "开启" : "关闭");
      }
    });
  planning_input_pub_ = node->create_publisher<nav_msgs::msg::Path>(
    plugin_name_ + "/planning_input_path", rclcpp::QoS(1));

  configured_ = true;
  RCLCPP_INFO(
    node->get_logger(),
    "MincoMpcController 已配置: planning_frame=%s base_frame=%s horizon=%.2f m "
    "replan=%.1f Hz v_max=%.2f m/s a_max=%.2f m/s^2 yaw_mode=%s",
    planning_frame_.c_str(), base_frame_.c_str(), planning_horizon_, replan_frequency_,
    limits_.max_linear_speed, limits_.max_linear_accel, yaw_policy_.modeName());
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

  // 重规划失败处理阶梯（默认值即行为开关：参数文件不写这些键时阶梯按默认启用）。
  declare("failure_ladder.cold_start_after", 3);        // 连续失败 N 次后改冷启动+降速
  declare("failure_ladder.speed_backoff_after", 6);     // 连续失败 N 次后再降一档速度
  declare("failure_ladder.backoff_factor", 0.5);        // 每档速度缩放系数
  declare("failure_patch.radius", 0.15);                // 失败位置临时膨胀补丁半径（m）
  declare("failure_patch.ttl", 3.0);                    // 补丁存活时间（s）
  declare("failure_patch.min_distance", 0.5);           // 距车太近的位置不打补丁（m）
  declare("failure_patch.max_count", 4);                // 同时生效的补丁上限

  node->get_parameter(name + ".planning_frame", planning_frame_);
  node->get_parameter(name + ".base_frame", base_frame_);
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
  node->get_parameter(name + ".failure_ladder.cold_start_after", failure_ladder_cold_start_after_);
  node->get_parameter(
    name + ".failure_ladder.speed_backoff_after", failure_ladder_speed_backoff_after_);
  node->get_parameter(name + ".failure_ladder.backoff_factor", failure_ladder_backoff_factor_);
  node->get_parameter(name + ".failure_patch.radius", failure_patch_radius_);
  node->get_parameter(name + ".failure_patch.ttl", failure_patch_ttl_);
  node->get_parameter(name + ".failure_patch.min_distance", failure_patch_min_distance_);
  node->get_parameter(name + ".failure_patch.max_count", failure_patch_max_count_);
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

  node->get_parameter(name + ".safety.robot_radius", minco_config_.robot_radius);
  node->get_parameter(name + ".safety.clearance_margin", minco_config_.clearance_margin);
  node->get_parameter(name + ".safety.unknown_is_obstacle", esdf_config_.unknown_is_seed);
  node->get_parameter(name + ".safety.braking_deceleration", braking_deceleration_);
  node->get_parameter(name + ".safety.reaction_latency", reaction_latency_);

  node->get_parameter(name + ".state.max_sample_gap", state_max_sample_gap_);
  node->get_parameter(name + ".state.max_position_jump", state_max_position_jump_);
  node->get_parameter(name + ".state.max_yaw_jump", state_max_yaw_jump_);
  node->get_parameter(name + ".state.velocity_filter_alpha", state_velocity_filter_alpha_);

  if (!parseYawMode(yaw_mode_text, yaw_mode_)) {
    return fail("unsupported yaw_policy.mode: " + yaw_mode_text);
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
  resetFailureLadder();
  yaw_policy_.clearSpinRequest();
  reference_builder_.reset();
  replan_manager_.reset();
  state_adapter_.reset();
  terminal_stop_.reset();
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
    invalidateTrajectory("new navigation goal", now_stamp);
    // 新会话 = 新的建轨尝试：失败阶梯复位（失败位置补丁保留，由 TTL 自然过期）。
    resetFailureLadder();
    last_violation_pos_text_ = "none";
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
  // 第 3 层：把"失败位置临时膨胀补丁"盖到快照上再建距离场。补丁只影响**候选生成
  // 与校验**（优化器/校验器/局部路径裁剪读同一份 ESDF），不改 costmap、不改参数；
  // TTL 过期自动消失，距车过近的位置在添加时已被护栏拦下。
  {
    std::lock_guard<std::mutex> lock(failure_patches_mutex_);
    if (!failure_patches_.empty()) {
      failure_patches_.erase(
        std::remove_if(
          failure_patches_.begin(), failure_patches_.end(),
          [_stamp](const FailurePatch & _patch) { return _patch.expire_stamp <= _stamp; }),
        failure_patches_.end());
    }
    if (!failure_patches_.empty()) {
      std::vector<srm27_minco_core::CellState> cells;
      cells.reserve(static_cast<std::size_t>(snapshot.sizeX()) * static_cast<std::size_t>(snapshot.sizeY()));
      for (int my = 0; my < snapshot.sizeY(); ++my) {
        for (int mx = 0; mx < snapshot.sizeX(); ++mx) {
          cells.push_back(snapshot.cell(mx, my));
        }
      }
      int stamped = 0;
      for (const FailurePatch & patch : failure_patches_) {
        int cx = 0;
        int cy = 0;
        snapshot.worldToCell(patch.position.x(), patch.position.y(), cx, cy);
        const int radius_cells =
          std::max(1, static_cast<int>(std::ceil(patch.radius / snapshot.resolution())));
        for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
          for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
            if (dx * dx + dy * dy > radius_cells * radius_cells) {
              continue;
            }
            const int mx = cx + dx;
            const int my = cy + dy;
            if (
              mx >= 0 && mx < snapshot.sizeX() && my >= 0 && my < snapshot.sizeY() &&
              cells[static_cast<std::size_t>(my) * static_cast<std::size_t>(snapshot.sizeX()) +
                    static_cast<std::size_t>(mx)] == srm27_minco_core::CellState::kFree) {
              cells[static_cast<std::size_t>(my) * static_cast<std::size_t>(snapshot.sizeX()) +
                    static_cast<std::size_t>(mx)] = srm27_minco_core::CellState::kOccupied;
              ++stamped;
            }
          }
        }
      }
      if (stamped > 0) {
        snapshot.reset(
          snapshot.originX(), snapshot.originY(), snapshot.resolution(), snapshot.sizeX(),
          snapshot.sizeY(), std::move(cells), version, snapshot.stamp());
      }
    }
  }
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
  // 阶梯的运行时速度缩放：只作用于**候选生成与本次校验上限**，不改参数文件、
  // 不动 base_limits_（下次成功后自动恢复）。速度是这里唯一动的杠杆：
  // 转弯半径 ∝ v²/a，降速直接把"切墙角"的解空间找回来。
  if (failure_speed_scale_ < 1.0 && failure_ladder_stage_ > 0) {
    planning_request.limits.max_linear_speed *= failure_speed_scale_;
  }
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
  // 阶梯强制冷启动：热启动复用的是上一条**被拒轨迹**的前缀，初值本身在错误盆地
  // 里，继续热启动只会在原地打转（2026-10-13 拐角切墙案例的 26 次同配方重试）。
  planning_request.use_warm_start =
    decision.use_warm_start && has_warm_start_ && !failure_force_cold_;
  planning_request.warm_inner_points = warm_inner_points_;
  planning_request.warm_durations = warm_durations_;
  planning_request.replan_type = static_cast<int>(decision.type);
  planning_request.inject_failure = failure_injection_.load();
  worker_.submit(planning_request);
}

void MincoMpcController::updateFailureLadder(const PlanningResult & _result, double _now_stamp)
{
  ++consecutive_plan_failures_;
  if (_result.has_violation_position) {
    char buffer[64];
    std::snprintf(
      buffer, sizeof(buffer), "(%.2f, %.2f)", _result.violation_position.x(),
      _result.violation_position.y());
    last_violation_pos_text_ = buffer;
  }
  consecutive_plan_successes_ = 0;  // 失败打断成功连击
  const int previous_stage = failure_ladder_stage_;
  if (
    failure_ladder_speed_backoff_after_ > 0 &&
    consecutive_plan_failures_ >=
      static_cast<std::uint64_t>(failure_ladder_speed_backoff_after_)) {
    failure_ladder_stage_ = 2;  // 冷启动 + 二次降速（factor²，默认 0.25）
  } else if (
    failure_ladder_cold_start_after_ > 0 &&
    consecutive_plan_failures_ >=
      static_cast<std::uint64_t>(failure_ladder_cold_start_after_)) {
    failure_ladder_stage_ = std::max(failure_ladder_stage_, 1);  // 冷启动 + 半速
  }
  applyLadderStage();
  if (failure_ladder_stage_ != previous_stage) {
    RCLCPP_WARN(
      (parent_.lock())->get_logger(),
      "MincoMpcController: 重规划阶梯 %d → %d（连续失败 #%lu，%s，速度 ×%.2f，越界点 %s）",
      previous_stage, failure_ladder_stage_,
      static_cast<unsigned long>(consecutive_plan_failures_),
      failure_ladder_stage_ == 1 ? "冷启动+半速" : (failure_ladder_stage_ == 2 ? "冷启动+1/4速" : "正常"),
      failure_speed_scale_, last_violation_pos_text_.c_str());
  }
  maybeAddFailurePatch(_result, _now_stamp);
}

void MincoMpcController::maybeAddFailurePatch(const PlanningResult & _result, double _now_stamp)
{
  if (failure_patch_radius_ <= 0.0 || failure_patch_ttl_ <= 0.0) {
    return;
  }
  // 从第 cold_start_after+1 次连续失败开始打补丁（先让阶梯的降速试一轮）。
  if (
    consecutive_plan_failures_ <
    static_cast<std::uint64_t>(std::max(1, failure_ladder_cold_start_after_ + 1))) {
    return;
  }
  if (!_result.has_violation_position) {
    return;
  }
  // 护栏：越界点离车太近时不打（补丁会把车自己困在障碍里，连安全停车都规划不出）。
  if (
    _result.has_request_position &&
    (_result.violation_position - _result.request_position).norm() < failure_patch_min_distance_) {
    return;
  }
  std::lock_guard<std::mutex> lock(failure_patches_mutex_);
  // 聚类：已有补丁覆盖的越界点不重复添加。
  for (const FailurePatch & patch : failure_patches_) {
    if ((patch.position - _result.violation_position).norm() <= failure_patch_radius_) {
      return;
    }
  }
  if (static_cast<int>(failure_patches_.size()) >= std::max(1, failure_patch_max_count_)) {
    // 上限：淘汰最老的补丁。
    failure_patches_.erase(failure_patches_.begin());
  }
  FailurePatch patch;
  patch.position = _result.violation_position;
  patch.radius = failure_patch_radius_;
  patch.expire_stamp = _now_stamp + failure_patch_ttl_;
  failure_patches_.push_back(patch);
  // 立即让在执行的轨迹对补丁复验：轨迹若穿过补丁区会被重新规划绕开。
  invalidateTrajectory("failure patch added", _now_stamp);
  RCLCPP_WARN(
    (parent_.lock())->get_logger(),
    "MincoMpcController: 越界点 %s 打临时膨胀补丁（半径 %.2f m，%.1f s 后过期，当前 %d 个）",
    last_violation_pos_text_.c_str(), patch.radius, failure_patch_ttl_,
    static_cast<int>(failure_patches_.size()));
}

void MincoMpcController::resetFailureLadder()
{
  consecutive_plan_failures_ = 0;
  consecutive_plan_successes_ = 0;
  failure_ladder_stage_ = 0;
  applyLadderStage();
}

void MincoMpcController::applyLadderStage()
{
  switch (failure_ladder_stage_) {
    case 2:
      failure_force_cold_ = true;
      failure_speed_scale_ =
        srm27_minco_core::clampToRange(
          failure_ladder_backoff_factor_ * failure_ladder_backoff_factor_, 0.05, 1.0);
      break;
    case 1:
      failure_force_cold_ = true;
      failure_speed_scale_ =
        srm27_minco_core::clampToRange(failure_ladder_backoff_factor_, 0.05, 1.0);
      break;
    default:
      failure_ladder_stage_ = 0;
      failure_force_cold_ = false;
      failure_speed_scale_ = 1.0;
      break;
  }
}

void MincoMpcController::harvestPlanningResult(double _now_stamp)
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
    // 只统计**当前会话**的失败：迟到/过期会话的结果不进阶梯（否则换目标后旧结果
    // 会把失败计数灌水，阶梯卡在高档位下不来）。
    if (result.status == "validation_failed" && result.versions.goal_epoch == goal_epoch_) {
      updateFailureLadder(result, _now_stamp);
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
  // 阶梯迟滞：一次成功**不**回到全速——连续 3 次成功才退一档（2→1→0）。
  // 否则窄弯里会震荡：降速成功→立刻全速→再失败→恢复行为循环（2026-10-14 实测
  // 22 次复位 / 90+ 次恢复的爬行）。失败计数清零；档位由成功连击驱动衰减。
  consecutive_plan_failures_ = 0;
  if (failure_ladder_stage_ > 0) {
    ++consecutive_plan_successes_;
    if (consecutive_plan_successes_ >= 3) {
      const int previous_stage = failure_ladder_stage_;
      --failure_ladder_stage_;
      consecutive_plan_successes_ = 0;
      applyLadderStage();
      RCLCPP_INFO(
        (parent_.lock())->get_logger(),
        "MincoMpcController: 规划连续 3 次成功，阶梯降档 %d → %d（速度 ×%.2f）",
        previous_stage, failure_ladder_stage_, failure_speed_scale_);
    }
  }
  if (result.degraded) {
    ++degraded_candidate_count_;
    RCLCPP_WARN(
      (parent_.lock())->get_logger(),
      "MincoMpcController: 提交降级候选（有效前缀+制动延伸，原候选在 %.2f s 越界）: %s",
      result.validation.first_violation_time, result.reason.c_str());
  }
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
  command.header.frame_id = base_frame_;
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
  diagnostics_.map_version = map_version_;
  diagnostics_.path_version = path_version_;
  diagnostics_.limits_version = limits_version_;
  diagnostics_.goal_epoch = goal_epoch_;
  diagnostics_.trajectory_id = trajectory_id_;
  // 重规划阶梯状态（跨周期成员 → 每周期诊断）。
  diagnostics_.consecutive_plan_failures = consecutive_plan_failures_;
  diagnostics_.failure_ladder_stage = failure_ladder_stage_;
  diagnostics_.failure_speed_scale = failure_speed_scale_;
  diagnostics_.degraded_candidate_count = degraded_candidate_count_;
  diagnostics_.failure_injection = failure_injection_.load() ? 1 : 0;
  diagnostics_.last_violation_pos = last_violation_pos_text_;
  {
    std::lock_guard<std::mutex> lock(failure_patches_mutex_);
    diagnostics_.active_failure_patches = static_cast<int>(failure_patches_.size());
  }

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

  // 5) 取回后台结果。
  harvestPlanningResult(now_stamp);

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

  // 12) 取执行时刻的预测速度并转换到车体系。
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
  Eigen::Vector2d velocity_body =
    srm27_minco_core::odomToBodyVelocity(yaw_at_actuation, velocity_odom);
  double omega_command = command(2);

  if (!velocity_body.allFinite() || !std::isfinite(omega_command)) {
    return fail("MPC produced a non-finite command");
  }

  // 13) 输出前检查：合速度、自转与相对上一条命令的变化量。
  diagnostics_.applied_speed_scale = 1.0;
  const double speed = velocity_body.norm();
  if (speed > limits_.max_linear_speed) {
    const double scale = limits_.max_linear_speed / speed;
    diagnostics_.applied_speed_scale = scale;
    velocity_odom *= scale;
    velocity_body *= scale;
  }
  if (std::abs(omega_command) > limits_.max_angular_speed) {
    omega_command = srm27_minco_core::clampToRange(
      omega_command, -limits_.max_angular_speed, limits_.max_angular_speed);
  }
  if (has_previous_input_ && previous_applied_input_.size() == mpc_model_.inputDim()) {
    // 平移与自转的变化量不得突破有效加速度上限（下游还有 mux 的末端限幅）。
    const double interval = std::max(1.0e-3, now_stamp - last_control_stamp_);
    const Eigen::Vector2d previous_odom(previous_applied_input_(0), previous_applied_input_(1));
    const Eigen::Vector2d delta = velocity_odom - previous_odom;
    const double max_delta = limits_.max_linear_accel * interval;
    if (delta.norm() > max_delta) {
      velocity_odom = previous_odom + delta.normalized() * max_delta;
      velocity_body = srm27_minco_core::odomToBodyVelocity(yaw_at_actuation, velocity_odom);
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
  command_msg.header.frame_id = base_frame_;
  command_msg.twist.linear.x = velocity_body.x();
  command_msg.twist.linear.y = velocity_body.y();
  command_msg.twist.linear.z = 0.0;
  command_msg.twist.angular.x = 0.0;
  command_msg.twist.angular.y = 0.0;
  command_msg.twist.angular.z = omega_command;

  diagnostics_.requested_vx = command_msg.twist.linear.x;
  diagnostics_.requested_vy = command_msg.twist.linear.y;
  diagnostics_.requested_wz = command_msg.twist.angular.z;
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
