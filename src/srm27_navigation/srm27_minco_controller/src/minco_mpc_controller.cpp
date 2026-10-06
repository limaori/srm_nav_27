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

/// \brief 判定“新目标”的终点位置阈值（m）。
///
/// 小于该值视为同一目标的路径刷新；大于该值说明导航目标变了，必须开启新会话。
/// **已知限制**：取消后重新下发**完全相同坐标**的目标会得到同一个会话编号，
/// 无法与普通路径刷新区分；真正的 Action 级会话守护属于方案 §8.3 的 P4 工作。
constexpr double kGoalChangeThresholdMeters = 1.0e-3;

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

  srm27_minco_core::TrajectoryValidatorConfig validator_config;
  validator_config.robot_radius = minco_config_.robot_radius;
  validator_config.clearance_margin = minco_config_.clearance_margin;
  validator_config.max_linear_speed = limits_.max_linear_speed;
  validator_config.max_linear_accel = limits_.max_linear_accel;
  validator_config.min_piece_duration = minco_config_.min_piece_duration;
  validator_config.unknown_is_obstacle = esdf_config_.unknown_is_seed;
  validator_config.map_timeout = map_timeout_;
  validator_config.trajectory_max_age = trajectory_max_age_;
  validator_config.required_prefix_duration = prediction_steps_ * prediction_dt_;
  validator_config.braking_deceleration = initializer_config_.max_brake;
  validator_config.reaction_latency = state_timeout_;
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
  declare("minco.w_jerk", 1.0);
  declare("minco.w_time", 10.0);
  declare("minco.w_obstacle", 100.0);
  declare("minco.w_velocity", 20.0);
  declare("minco.w_acceleration", 2.0);
  declare("minco.w_time_ratio", 1.0);
  declare("minco.soft_hinge_beta", 20.0);
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
  declare("mpc.command_lookahead", 0.0);
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

  declare("state.max_sample_gap", 0.20);
  declare("state.max_position_jump", 0.75);
  declare("state.max_yaw_jump", 1.20);
  declare("state.velocity_filter_alpha", 0.35);

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
  node->get_parameter(name + ".minco.w_jerk", minco_config_.w_jerk);
  node->get_parameter(name + ".minco.w_time", minco_config_.w_time);
  node->get_parameter(name + ".minco.w_obstacle", minco_config_.w_obstacle);
  node->get_parameter(name + ".minco.w_velocity", minco_config_.w_velocity);
  node->get_parameter(name + ".minco.w_acceleration", minco_config_.w_acceleration);
  node->get_parameter(name + ".minco.w_time_ratio", minco_config_.w_time_ratio);
  node->get_parameter(name + ".minco.soft_hinge_beta", minco_config_.soft_hinge_beta);
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
  if (trajectory_pub_) {
    trajectory_pub_->on_activate();
  }
  if (prediction_pub_) {
    prediction_pub_->on_activate();
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
  yaw_policy_.clearSpinRequest();
  reference_builder_.reset();
  replan_manager_.reset();
  state_adapter_.reset();
  has_previous_input_ = false;
  has_last_control_stamp_ = false;
  if (trajectory_pub_) {
    trajectory_pub_->on_deactivate();
  }
  if (prediction_pub_) {
    prediction_pub_->on_deactivate();
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
  {
    std::lock_guard<std::mutex> lock(path_mutex_);
    const bool same_source_frame = path_source_frame_ == _path.header.frame_id && has_raw_path_;
    // “同一目标 3 Hz 刷新”与“新目标”必须区分：前者保留可用轨迹，后者必须有新的会话编号，
    // 否则迟到的旧会话结果会重新激活运动（方案 §4.4）。
    if (!has_raw_path_ || !same_source_frame) {
      new_goal = true;
    } else if (!raw_path_.poses.empty()) {
      const geometry_msgs::msg::Point & previous_goal_point = raw_path_.poses.back().pose.position;
      new_goal = std::hypot(
                   new_goal_point.x - previous_goal_point.x,
                   new_goal_point.y - previous_goal_point.y) > kGoalChangeThresholdMeters;
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
    // 新会话：作废轨迹、热启动、重规划状态与参考进度。
    ++goal_epoch_;
    replan_manager_.reset();
    invalidateTrajectory("new navigation goal", now_stamp);
    if (node) {
      RCLCPP_INFO(
        node->get_logger(), "MincoMpcController: 新导航会话 #%lu (路径版本 %lu)",
        static_cast<unsigned long>(goal_epoch_), static_cast<unsigned long>(path_version_));
    }
  } else {
    // 同一目标的路径刷新：保留当前有效轨迹与控制进度，只递增路径版本。
    if (node) {
      RCLCPP_DEBUG(
        node->get_logger(), "MincoMpcController: 路径刷新 (版本 %lu)，保留当前轨迹",
        static_cast<unsigned long>(path_version_));
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
  if (version_changed) {
    // 地图变化后已提交轨迹必须在最新地图上复验（方案 §6.6）。
    last_full_revalidation_stamp_ = -1.0;
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
  stop_reason_ = _reason;
  stop_request_stamp_ = _now_stamp;
  output_zero_stamp_ = _now_stamp;
  if (worker_.running()) {
    worker_.clear();
  }
  if (map_version_ == 0) {
    build_start_stamp_ = -1.0;
  } else if (build_start_stamp_ < 0.0) {
    build_start_stamp_ = _now_stamp;
  }
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
  planning_request.minco_config = minco_config_;
  planning_request.initializer_config = initializer_config_;
  // 本插件持有的路径是 Nav2 下发的全局路径，其末点即当前 FollowPath 任务的目标。
  planning_request.terminal_is_global_goal = true;
  planning_request.request_stamp = _now_stamp;
  planning_request.request_steady = _now_steady;
  planning_request.local_path_horizon = planning_horizon_;
  // 距离场以只读快照交给工作线程；工作线程不持有 costmap 锁。
  planning_request.esdf = std::const_pointer_cast<const srm27_minco_core::Esdf2D>(esdf_);
  planning_request.use_warm_start = decision.use_warm_start && has_warm_start_;
  planning_request.warm_inner_points = warm_inner_points_;
  planning_request.warm_durations = warm_durations_;
  planning_request.replan_type = static_cast<int>(decision.type);
  worker_.submit(planning_request);
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
  diagnostics_.minimum_clearance = result.min_clearance;
  diagnostics_.maximum_speed = result.max_speed;
  diagnostics_.maximum_acceleration = result.max_acceleration;

  if (!result.success) {
    RCLCPP_WARN_THROTTLE(
      (parent_.lock())->get_logger(), *(parent_.lock())->get_clock(), 2000,
      "MincoMpcController: 规划失败 (%s): %s", result.status.c_str(), result.reason.c_str());
    return;
  }

  // 版本校验：迟到的旧会话结果不得重新激活运动。
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
  std::string reason;
  if (!replan_manager_.canCommit(result.trajectory, result.versions, _now_stamp, &reason)) {
    diagnostics_.rejected_stale_result_count += 1;
    RCLCPP_WARN_THROTTLE(
      (parent_.lock())->get_logger(), *(parent_.lock())->get_clock(), 2000,
      "MincoMpcController: 丢弃候选轨迹: %s", reason.c_str());
    return;
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
  (void)_velocity;
  (void)_goal_checker;

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

  const auto fail = [this, &node, &now_stamp](const std::string & _reason) {
    diagnostics_.total_control_time_ms = 0.0;
    const geometry_msgs::msg::TwistStamped command = stopCommand(node, _reason, now_stamp);
    if (build_start_stamp_ < 0.0) {
      build_start_stamp_ = now_stamp;
    }
    if ((now_stamp - build_start_stamp_) > build_grace_period_) {
      // 超过建轨宽限：交给 BT 处理，而不是永远返回零并假装正常。
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
    diagnostics_.trajectory_age = std::max(0.0, now_stamp - trajectory->generated_stamp);
    const bool need_revalidation =
      (last_full_revalidation_stamp_ < 0.0) ||
      ((now_stamp - last_full_revalidation_stamp_) > std::max(0.2, 1.0 / replan_frequency_));
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

  // 8) 没有可用轨迹：受控零输出 + 宽限计时。
  if (!trajectory) {
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
  const Eigen::Vector2d velocity_odom(command(0), command(1));
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
      const Eigen::Vector2d bounded = previous_odom + delta.normalized() * max_delta;
      velocity_body = srm27_minco_core::odomToBodyVelocity(yaw_at_actuation, bounded);
    }
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
