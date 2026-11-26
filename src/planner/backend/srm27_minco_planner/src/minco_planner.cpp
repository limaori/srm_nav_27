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

#include "srm27_minco_planner/minco_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <utility>

#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"

namespace srm27_minco_planner
{

namespace
{
using nav2_util::declare_parameter_if_not_declared;

/// \brief 稳态时钟（秒），只用于节流与耗时统计。
double steadyNow()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

/// \brief 采样步长（s）：控制器按时间线性插值，0.05 s 在 2.6 m/s 下约 13 cm。
constexpr double kSampleStep = 0.05;
/// \brief 弧度转角度（避免依赖 M_PI 的宏）。
constexpr double kRadToDeg = 57.29577951308232;

/// \brief nav_msgs::Path -> 二维折线（丢非有限点）。
std::vector<Eigen::Vector2d> pathToPoints(const nav_msgs::msg::Path & _path)
{
  std::vector<Eigen::Vector2d> points;
  points.reserve(_path.poses.size());
  for (const auto & pose : _path.poses) {
    const Eigen::Vector2d point(pose.pose.position.x, pose.pose.position.y);
    if (point.allFinite()) {
      points.push_back(point);
    }
  }
  return points;
}
}  // namespace

MincoPlanner::~MincoPlanner()
{
  stopLoop();
}

void MincoPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("MincoPlanner: 无法获得父节点");
  }
  plugin_name_ = name;
  tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);
  clock_ = node->get_clock();
  map_frame_ = costmap_ros_->getGlobalFrameID();
  base_frame_ = costmap_ros_->getBaseFrameID();

  loadParameters();
  // 内部 YAstar 的参数前缀在 loadParameters 里读，必须早于它自己的 configure。
  declare_parameter_if_not_declared(
    node, name + ".yastar_plugin_name", rclcpp::ParameterValue(yastar_plugin_name_));
  node->get_parameter(name + ".yastar_plugin_name", yastar_plugin_name_);

  std::string reason;
  if (!validateParameters(&reason)) {
    throw std::runtime_error("MincoPlanner: " + reason);
  }

  srm27_map_adapter::CostmapConfig costmap_config;
  costmap_config.occupied_cost_threshold = 253;
  if (!costmap_adapter_.configure(costmap_ros_, costmap_config, &reason)) {
    throw std::runtime_error("MincoPlanner: costmap adapter: " + reason);
  }

  // 全局搜索复用既有 YAstar 插件；它自己的参数在 `planner_server.<yastar_plugin_name_>` 段。
  yastar_.configure(parent, yastar_plugin_name_, tf_, costmap_ros_);

  RCLCPP_INFO(
    node->get_logger(),
    "MincoPlanner 已配置: name=%s map_frame=%s base_frame=%s output=%s opt_freq=%.1f Hz "
    "horizon=%.2f m v_max=%.2f a_max=%.2f 输入路径=%s（缺省回退 createPlan）",
    plugin_name_.c_str(), map_frame_.c_str(), base_frame_.c_str(), output_frame_.c_str(),
    opt_frequency_, planning_horizon_, limits_.max_linear_speed, limits_.max_linear_accel,
    smoothed_topic_.c_str());
  RCLCPP_INFO(
    node->get_logger(),
    "MincoPlanner 权重: w_ref=%.1f w_time=%.1f w_obstacle=%.1f w_vel=%.1f w_acc=%.1f "
    "转角: dz=%.2f sat=%.2f min_vel=%.2f ratio=%.2f decay=%.2f",
    minco_config_.w_reference, minco_config_.w_time, minco_config_.w_obstacle,
    minco_config_.w_velocity, minco_config_.w_acceleration,
    initializer_config_.turn_angle_deadzone, initializer_config_.turn_angle_saturation,
    initializer_config_.min_turn_vel, initializer_config_.min_turn_vel_ratio,
    initializer_config_.decay_power);
}

void MincoPlanner::loadParameters()
{
  auto node = node_.lock();
  const std::string & name = plugin_name_;
  const auto declare = [&node, &name](const std::string & _suffix, const auto & _default) {
    declare_parameter_if_not_declared(
      node, name + "." + _suffix, rclcpp::ParameterValue(_default));
  };
  const auto get = [&node, &name](const std::string & _suffix, auto & _value) {
    node->get_parameter(name + "." + _suffix, _value);
  };

  declare("base_frame", base_frame_);
  declare("output_frame", std::string("odom"));
  declare("smooth_path_topic", smoothed_topic_);
  declare("odom_topic", odom_topic_);
  declare("trajectory_topic", trajectory_topic_);
  declare("path_topic", path_topic_);
  declare("diagnostics_topic", diagnostics_topic_);
  declare("opt_frequency", 10.0);
  declare("planning_horizon", 3.0);
  declare("path_centering_max_displacement", 0.01);
  declare("validity_window", 1.0);
  declare("terminal_reached_radius", 0.20);
  declare("goal_change_tolerance", 0.10);
  declare("diagnostics_period", 0.5);
  declare("state_timeout", 0.50);
  declare("publish_visualization", true);
  declare("use_warm_start", true);

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
  declare("minco.w_reference", 0.0);
  declare("minco.w_time_ratio", 1.0);
  declare("minco.soft_hinge_beta", 20.0);
  declare("minco.optimization_clearance_margin", 0.05);
  declare("minco.resample_step", 0.10);
  declare("minco.corner_speed_ratio", 0.35);
  declare("minco.turn_angle_deadzone", 0.10);
  declare("minco.turn_angle_saturation", 0.80);
  declare("minco.min_turn_vel", 0.12);
  declare("minco.min_turn_vel_ratio", 0.0);
  declare("minco.decay_power", 1.6);
  declare("minco.corner_chord_window", 0.40);
  declare("minco.min_pieces", 2);
  declare("minco.max_pieces", 12);
  declare("minco.terminal_speed", 0.0);

  declare("limits.max_linear_speed", 0.50);
  declare("limits.max_linear_accel", 0.30);
  declare("limits.max_angular_speed", 0.30);
  declare("limits.max_angular_accel", 0.50);

  declare("safety.robot_radius", 0.33);
  declare("safety.clearance_margin", 0.05);
  declare("safety.unknown_is_obstacle", true);
  declare("safety.braking_deceleration", 0.0);
  declare("safety.reaction_latency", 0.0);

  declare("validator.sample_dt", 0.02);
  declare("validator.max_sample_spacing", 0.02);
  declare("validator.speed_tolerance_ratio", 0.02);
  declare("validator.accel_tolerance_ratio", 0.02);
  declare("validator.required_prefix_duration", 0.60);

  get("base_frame", base_frame_);
  get("output_frame", output_frame_);
  get("smooth_path_topic", smoothed_topic_);
  get("odom_topic", odom_topic_);
  get("trajectory_topic", trajectory_topic_);
  get("path_topic", path_topic_);
  get("diagnostics_topic", diagnostics_topic_);
  get("opt_frequency", opt_frequency_);
  get("planning_horizon", planning_horizon_);
  get("path_centering_max_displacement", path_centering_max_displacement_);
  get("validity_window", validity_window_);
  get("terminal_reached_radius", terminal_reached_radius_);
  get("goal_change_tolerance", goal_change_tolerance_);
  get("diagnostics_period", diagnostics_period_);
  get("state_timeout", state_timeout_);
  get("publish_visualization", publish_visualization_);
  get("use_warm_start", use_warm_start_);

  get("minco.two_stage_optimization", minco_config_.two_stage);
  get("minco.enable_report_fine_heuristic", minco_config_.enable_report_fine_heuristic);
  get("minco.nominal_piece_duration", initializer_config_.nominal_piece_duration);
  get("minco.min_piece_duration", minco_config_.min_piece_duration);
  get("minco.max_piece_duration", minco_config_.max_piece_duration);
  get("minco.pre_time_ratio_min", minco_config_.pre_time_ratio_min);
  get("minco.pre_time_ratio_max", minco_config_.pre_time_ratio_max);
  get("minco.samples_per_piece", minco_config_.samples_per_piece);
  get("minco.max_iterations", minco_config_.max_iterations);
  get("minco.gradient_tolerance", minco_config_.gradient_tolerance);
  get("minco.lbfgs_past", minco_config_.lbfgs_past);
  get("minco.w_jerk", minco_config_.w_jerk);
  get("minco.w_time", minco_config_.w_time);
  get("minco.w_obstacle", minco_config_.w_obstacle);
  get("minco.w_velocity", minco_config_.w_velocity);
  get("minco.w_acceleration", minco_config_.w_acceleration);
  get("minco.w_reference", minco_config_.w_reference);
  get("minco.w_time_ratio", minco_config_.w_time_ratio);
  get("minco.soft_hinge_beta", minco_config_.soft_hinge_beta);
  get("minco.optimization_clearance_margin", minco_config_.optimization_clearance_margin);
  get("minco.resample_step", initializer_config_.resample_step);
  get("minco.corner_speed_ratio", initializer_config_.corner_speed_ratio);
  get("minco.turn_angle_deadzone", initializer_config_.turn_angle_deadzone);
  get("minco.turn_angle_saturation", initializer_config_.turn_angle_saturation);
  get("minco.min_turn_vel", initializer_config_.min_turn_vel);
  get("minco.min_turn_vel_ratio", initializer_config_.min_turn_vel_ratio);
  get("minco.decay_power", initializer_config_.decay_power);
  get("minco.corner_chord_window", initializer_config_.corner_chord_window);
  get("minco.min_pieces", initializer_config_.min_pieces);
  get("minco.max_pieces", initializer_config_.max_pieces);
  get("minco.terminal_speed", initializer_config_.terminal_speed);

  get("limits.max_linear_speed", limits_.max_linear_speed);
  get("limits.max_linear_accel", limits_.max_linear_accel);
  get("limits.max_angular_speed", limits_.max_angular_speed);
  get("limits.max_angular_accel", limits_.max_angular_accel);

  get("safety.robot_radius", minco_config_.robot_radius);
  get("safety.clearance_margin", minco_config_.clearance_margin);
  get("safety.unknown_is_obstacle", esdf_config_.unknown_is_seed);

  double braking = 0.0;
  double reaction = 0.0;
  get("safety.braking_deceleration", braking);
  get("safety.reaction_latency", reaction);

  // 校验器配置：与控制器内部模式完全同构（阈值取同一份有效约束）。
  validator_config_.robot_radius = minco_config_.robot_radius;
  validator_config_.clearance_margin = minco_config_.clearance_margin;
  validator_config_.max_linear_speed = limits_.max_linear_speed;
  validator_config_.max_linear_accel = limits_.max_linear_accel;
  validator_config_.min_piece_duration = minco_config_.min_piece_duration;
  validator_config_.unknown_is_obstacle = esdf_config_.unknown_is_seed;
  validator_config_.braking_deceleration = braking > 0.0 ? braking : limits_.max_linear_accel;
  validator_config_.reaction_latency = reaction > 0.0 ? reaction : state_timeout_;
  get("validator.sample_dt", validator_config_.sample_dt);
  get("validator.max_sample_spacing", validator_config_.max_sample_spacing);
  get("validator.speed_tolerance_ratio", validator_config_.speed_tolerance_ratio);
  get("validator.accel_tolerance_ratio", validator_config_.accel_tolerance_ratio);
  get("validator.required_prefix_duration", validator_config_.required_prefix_duration);

  // 前端与验证器共用同一套有效约束。
  initializer_config_.min_piece_duration = minco_config_.min_piece_duration;
  initializer_config_.max_piece_duration = minco_config_.max_piece_duration;
  initializer_config_.duplicate_epsilon = 1.0e-3;
  initializer_config_.max_speed = limits_.max_linear_speed;
  initializer_config_.max_accel = limits_.max_linear_accel;
  initializer_config_.max_brake = limits_.max_linear_accel;
}

bool MincoPlanner::validateParameters(std::string * _reason) const
{
  const auto fail = [&_reason](const std::string & _message) {
    if (_reason != nullptr) {
      *_reason = _message;
    }
    return false;
  };
  if (!(opt_frequency_ > 0.0) || !std::isfinite(opt_frequency_)) {
    return fail("opt_frequency must be positive");
  }
  if (!(planning_horizon_ > 0.0) || !std::isfinite(planning_horizon_)) {
    return fail("planning_horizon must be positive");
  }
  if (!limits_.valid()) {
    return fail("limits.* must be finite and positive");
  }
  std::string minco_reason;
  if (!minco_config_.valid(&minco_reason)) {
    return fail("minco config: " + minco_reason);
  }
  if (!validator_config_.valid(&minco_reason)) {
    return fail("validator config: " + minco_reason);
  }
  return true;
}

void MincoPlanner::activate()
{
  auto node = node_.lock();
  if (!node) {
    return;
  }
  trajectory_pub_ =
    node->create_publisher<srm27_minco_interfaces::msg::MincoTrajectory>(trajectory_topic_, 1);
  path_pub_ = node->create_publisher<nav_msgs::msg::Path>(path_topic_, 1);
  diagnostics_pub_ =
    node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(diagnostics_topic_, 1);

  // LifecyclePublisher 在 on_activate() 之前 publish() 是**静默丢弃**的：
  // 少了这三行，日志显示“发布成功”，话题上却一条消息都没有。
  trajectory_pub_->on_activate();
  path_pub_->on_activate();
  diagnostics_pub_->on_activate();

  smoothed_sub_ = node->create_subscription<nav_msgs::msg::Path>(
    smoothed_topic_, rclcpp::QoS(1),
    [this](nav_msgs::msg::Path::ConstSharedPtr _msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      smoothed_path_ = *_msg;
      smoothed_stamp_ = rclcpp::Time(_msg->header.stamp).seconds();
      smoothed_fresh_ = true;
    });
  odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, rclcpp::QoS(10).best_effort(),
    [this](nav_msgs::msg::Odometry::ConstSharedPtr _msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      last_odom_ = *_msg;
      has_odom_ = true;
    });

  yastar_.activate();
  active_.store(true);
  running_.store(true);
  loop_ = std::thread(&MincoPlanner::runLoop, this);
}

void MincoPlanner::deactivate()
{
  stopLoop();
  active_.store(false);
  yastar_.deactivate();
  if (trajectory_pub_) {
    trajectory_pub_->on_deactivate();
  }
  if (path_pub_) {
    path_pub_->on_deactivate();
  }
  if (diagnostics_pub_) {
    diagnostics_pub_->on_deactivate();
  }
  trajectory_pub_.reset();
  path_pub_.reset();
  diagnostics_pub_.reset();
  smoothed_sub_.reset();
  odom_sub_.reset();
}

void MincoPlanner::cleanup()
{
  stopLoop();
  active_.store(false);
  yastar_.cleanup();
  costmap_adapter_ = srm27_map_adapter::CostmapAdapter();
  costmap_ros_.reset();
  tf_.reset();
  node_.reset();
}

void MincoPlanner::stopLoop()
{
  running_.store(false);
  if (loop_.joinable()) {
    loop_.join();
  }
}

nav_msgs::msg::Path MincoPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path plan = yastar_.createPlan(start, goal);
  if (plan.poses.empty()) {
    return plan;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  const Eigen::Vector2d goal_xy(goal.pose.position.x, goal.pose.position.y);
  const bool new_goal =
    !has_goal_ || (goal_xy - goal_).norm() > goal_change_tolerance_ ||
    std::abs(goal.pose.position.z) > 0.0;
  if (new_goal) {
    has_goal_ = true;
    goal_ = goal_xy;
    ++goal_epoch_;
    has_warm_start_ = false;
    smoothed_path_ = nav_msgs::msg::Path();
    smoothed_stamp_ = 0.0;
    smoothed_fresh_ = false;
  }
  latest_plan_ = plan;
  latest_plan_stamp_ = rclcpp::Time(plan.header.stamp).seconds();
  ++path_version_;
  return plan;
}

std::vector<Eigen::Vector2d> MincoPlanner::selectInputPath(std::uint64_t & _path_version) const
{
  // 优先用 smoother 的输出（走廊居中）；它的 stamp 比当前这份 plan 新才算数。
  _path_version = path_version_;
  // 只接受"本次目标之后收到的"平滑路径；顺带要求它确实通向当前目标，
  // 避免 smoother 失败时沿用上一条路线。
  if (smoothed_fresh_ && !smoothed_path_.poses.empty()) {
    const auto & last = smoothed_path_.poses.back().pose.position;
    const Eigen::Vector2d end(last.x, last.y);
    if ((end - goal_).norm() <= std::max(0.5, 2.0 * goal_change_tolerance_)) {
      return pathToPoints(smoothed_path_);
    }
  }
  return pathToPoints(latest_plan_);
}

bool MincoPlanner::updateOdometryVelocity()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_odom_) {
    return false;
  }
  const double stamp = rclcpp::Time(last_odom_.header.stamp).seconds();
  const Eigen::Vector2d position(
    last_odom_.pose.pose.position.x, last_odom_.pose.pose.position.y);
  if (last_odom_stamp_ > 0.0 && stamp > last_odom_stamp_) {
    const double dt = stamp - last_odom_stamp_;
    Eigen::Vector2d velocity = (position - previous_odom_position_) / dt;
    // 里程计速度在 odom 系，旋到 map 系（map->odom 只有旋转影响方向）。
    try {
      const auto tf_map_odom = tf_->lookupTransform(
        map_frame_, output_frame_, tf2::TimePointZero, tf2::durationFromSec(0.1));
      const double yaw = std::atan2(
        2.0 * (tf_map_odom.transform.rotation.w * tf_map_odom.transform.rotation.z +
               tf_map_odom.transform.rotation.x * tf_map_odom.transform.rotation.y),
        1.0 - 2.0 * (tf_map_odom.transform.rotation.y * tf_map_odom.transform.rotation.y +
                     tf_map_odom.transform.rotation.z * tf_map_odom.transform.rotation.z));
      const Eigen::Vector2d rotated(
        std::cos(yaw) * velocity.x() - std::sin(yaw) * velocity.y(),
        std::sin(yaw) * velocity.x() + std::cos(yaw) * velocity.y());
      odom_velocity_ = rotated;
    } catch (const tf2::TransformException &) {
      odom_velocity_ = velocity;
    }
  }
  previous_odom_position_ = position;
  last_odom_stamp_ = stamp;
  return true;
}

bool MincoPlanner::currentState(srm27_minco_core::State2D & _state, std::string * _reason)
{
  const auto fail = [&_reason](const std::string & _message) {
    if (_reason != nullptr) {
      *_reason = _message;
    }
    return false;
  };
  geometry_msgs::msg::TransformStamped map_base;
  try {
    map_base = tf_->lookupTransform(
      map_frame_, base_frame_, tf2::TimePointZero,
      tf2::durationFromSec(std::max(0.05, state_timeout_)));
  } catch (const tf2::TransformException & exception) {
    return fail(std::string("tf ") + exception.what());
  }

  const auto & q = map_base.transform.rotation;
  const double yaw = std::atan2(
    2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));

  if (!updateOdometryVelocity()) {
    return fail("odometry is not available");
  }

  _state.valid = true;
  _state.x = map_base.transform.translation.x;
  _state.y = map_base.transform.translation.y;
  _state.velocity = odom_velocity_;
  _state.yaw = yaw;
  _state.omega = 0.0;
  _state.acceleration_valid = false;
  _state.sample_stamp = clock_->now().seconds();
  _state.received_stamp = steadyNow();
  _state.source = "tf+odom";
  return true;
}

void MincoPlanner::publishTrajectory(
  const srm27_minco_core::Trajectory2D & _trajectory,
  const srm27_minco_core::VersionSet & _versions, double _stamp, double _terminal_speed,
  const std::string & _terminal_reason)
{
  // 1) 采样（规划系 = map）。
  std::vector<double> times;
  std::vector<Eigen::Vector2d> positions;
  std::vector<Eigen::Vector2d> velocities;
  const double duration = _trajectory.totalDuration();
  for (double t = 0.0; t < duration; t += kSampleStep) {
    times.push_back(t);
    positions.push_back(_trajectory.positionAt(t));
    velocities.push_back(_trajectory.velocityAt(t));
  }
  times.push_back(duration);
  positions.push_back(_trajectory.endPosition());
  velocities.push_back(_trajectory.endVelocity());

  // 2) map -> odom（控制器只认 odom 系的轨迹，契约不变）。
  Eigen::Vector2d translation = Eigen::Vector2d::Zero();
  double yaw = 0.0;
  try {
    const auto tf_out = tf_->lookupTransform(output_frame_, map_frame_, tf2::TimePointZero);
    const auto & q = tf_out.transform.rotation;
    yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    translation = Eigen::Vector2d(tf_out.transform.translation.x, tf_out.transform.translation.y);
  } catch (const tf2::TransformException & exception) {
    RCLCPP_WARN_THROTTLE(
      node_.lock()->get_logger(), *clock_, 2000,
      "MincoPlanner: 拿不到 %s->%s，轨迹不发布: %s", map_frame_.c_str(), output_frame_.c_str(),
      exception.what());
    return;
  }
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const Eigen::Matrix2d rotation = (Eigen::Matrix2d() << c, -s, s, c).finished();

  srm27_minco_interfaces::msg::MincoTrajectory message;
  message.header.stamp = rclcpp::Time(static_cast<int64_t>(_stamp * 1e9), clock_->get_clock_type());
  message.header.frame_id = output_frame_;
  message.path_version = _versions.path_version;
  message.goal_epoch = _versions.goal_epoch;
  message.duration = duration;
  message.terminal_is_global_goal = _trajectory.terminal_is_global_goal;
  message.terminal_speed = _terminal_speed;
  message.terminal_reason = _terminal_reason;
  message.t.resize(times.size());
  message.x.resize(times.size());
  message.y.resize(times.size());
  message.yaw.resize(times.size());
  message.vx.resize(times.size());
  message.vy.resize(times.size());
  for (std::size_t i = 0; i < times.size(); ++i) {
    const Eigen::Vector2d position = rotation * positions[i] + translation;
    const Eigen::Vector2d velocity = rotation * velocities[i];
    message.t[i] = times[i];
    message.x[i] = position.x();
    message.y[i] = position.y();
    message.vx[i] = velocity.x();
    message.vy[i] = velocity.y();
    message.yaw[i] = std::atan2(velocity.y(), velocity.x());
  }
  if (trajectory_pub_) {
    trajectory_pub_->publish(message);
  }

  // 3) RViz 用路径（同一份采样，odom 系）。
  if (publish_visualization_ && path_pub_) {
    nav_msgs::msg::Path path;
    path.header = message.header;
    path.poses.reserve(times.size());
    for (std::size_t i = 0; i < times.size(); ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = message.x[i];
      pose.pose.position.y = message.y[i];
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
    path_pub_->publish(path);
  }
}

void MincoPlanner::runLoop()
{
  const double period = 1.0 / std::max(1.0, opt_frequency_);
  double next = steadyNow();
  while (running_.load()) {
    next += period;
    tick();
    const double now = steadyNow();
    const double sleep_for = next - now;
    if (sleep_for > 0.0) {
      std::this_thread::sleep_for(std::chrono::duration<double>(sleep_for));
    } else {
      next = now;  // 落后太多就重新对齐，避免补偿式狂跑
    }
  }
}

void MincoPlanner::tick()
{
  auto node = node_.lock();
  if (!node || !active_.load()) {
    return;
  }

  nav_msgs::msg::Path plan;
  std::vector<Eigen::Vector2d> path;
  std::uint64_t goal_epoch = 0;
  std::uint64_t path_version = 0;
  srm27_minco_core::VersionSet versions;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_goal_ || latest_plan_.poses.size() < 2) {
      return;
    }
    path = selectInputPath(path_version);
    goal_epoch = goal_epoch_;
    versions.goal_epoch = goal_epoch_;
    versions.path_version = path_version;
    versions.limits_version = 0;
    versions.map_version = map_version_;
  }
  if (path.size() < 2) {
    return;
  }

  const double begin = steadyNow();
  std::string reason;
  srm27_minco_core::State2D state;
  if (!currentState(state, &reason)) {
    RCLCPP_WARN_THROTTLE(
      node->get_logger(), *clock_, 2000, "MincoPlanner: 状态不可用: %s", reason.c_str());
    return;
  }

  // 距离场快照（放到请求里给工作线程只读使用）。
  srm27_map_core::GridSnapshot snapshot;
  std::uint64_t map_version = 0;
  if (!costmap_adapter_.snapshot(state.sample_stamp, snapshot, map_version, &reason)) {
    RCLCPP_WARN_THROTTLE(
      node->get_logger(), *clock_, 2000, "MincoPlanner: 代价地图快照失败: %s", reason.c_str());
    return;
  }
  auto esdf = std::make_shared<srm27_map_core::Esdf2D>();
  if (!esdf->build(snapshot, esdf_config_)) {
    RCLCPP_WARN_THROTTLE(
      node->get_logger(), *clock_, 2000, "MincoPlanner: 距离场构建失败");
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    map_version_ = map_version;
    versions.map_version = map_version;
  }

  srm27_minco_pipeline::PlanningRequest request;
  request.versions = versions;
  request.path = path;
  request.state = state;
  request.limits = limits_;
  request.minco_config = minco_config_;
  request.initializer_config = initializer_config_;
  request.validator_config = validator_config_;
  request.request_stamp = state.sample_stamp;
  request.request_steady = begin;
  request.local_path_horizon = planning_horizon_;
  request.path_centering_max_displacement = path_centering_max_displacement_;
  request.validity_window = validity_window_;
  request.terminal_reached_radius = terminal_reached_radius_;
  request.esdf = esdf;
  request.replan_type = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (use_warm_start_ && has_warm_start_) {
      request.use_warm_start = true;
      request.warm_inner_points = warm_inner_points_;
      request.warm_durations = warm_durations_;
    }
  }

  const auto result = srm27_minco_pipeline::PlanningWorker::runOnce(request);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_result_ = result;
    has_result_ = true;
    last_result_stamp_ = state.sample_stamp;
  }

  if (!result.success) {
    if (result.status != "already_at_goal") {
      RCLCPP_WARN_THROTTLE(
        node->get_logger(), *clock_, 2000, "MincoPlanner: 规划失败 (%s): %s",
        result.status.c_str(), result.reason.c_str());
    }
    return;
  }

  publishTrajectory(
    result.trajectory, versions, state.sample_stamp, result.terminal_speed,
    result.terminal_reason);

  if (use_warm_start_) {
    const int pieces = result.trajectory.pieceCount();
    std::lock_guard<std::mutex> lock(mutex_);
    warm_inner_points_.clear();
    warm_durations_ = result.trajectory.durations();
    double accumulated = 0.0;
    for (int i = 0; i + 1 < pieces; ++i) {
      accumulated += result.trajectory.durations()[static_cast<std::size_t>(i)];
      warm_inner_points_.push_back(result.trajectory.positionAt(accumulated));
    }
    has_warm_start_ = pieces >= 2;
    warm_start_stamp_ = state.sample_stamp;
  }

  publishDiagnostics();
}

void MincoPlanner::publishDiagnostics()
{
  if (!diagnostics_pub_) {
    return;
  }
  const double now = clock_->now().seconds();
  if (last_diagnostics_stamp_ > 0.0 && (now - last_diagnostics_stamp_) < diagnostics_period_) {
    return;
  }
  last_diagnostics_stamp_ = now;

  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = clock_->now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.name = plugin_name_ + ": minco planner";
  status.message = "ok";

  const auto add_number = [&status](const std::string & _key, double _value) {
    diagnostic_msgs::msg::KeyValue pair;
    pair.key = _key;
    pair.value = std::to_string(_value);
    status.values.push_back(pair);
  };
  const auto add_text = [&status](const std::string & _key, const std::string & _value) {
    diagnostic_msgs::msg::KeyValue pair;
    pair.key = _key;
    pair.value = _value;
    status.values.push_back(pair);
  };
  {
    std::lock_guard<std::mutex> lock(mutex_);
    add_text("planning_result", last_result_.status);
    add_text("terminal_reason", last_result_.terminal_reason);
    add_number("planning_time_ms", last_result_.total_time_ms);
    add_number("frontend_time_ms", last_result_.frontend_time_ms);
    add_number("pre_time_ms", last_result_.pre_time_ms);
    add_number("finely_time_ms", last_result_.finely_time_ms);
    add_number("validation_time_ms", last_result_.validation_time_ms);
    add_number("minimum_clearance", last_result_.min_clearance);
    add_number("maximum_speed", last_result_.max_speed);
    add_number("corner_angle_max_deg", last_result_.corner_angle_max * kRadToDeg);
    add_number("corner_speed_limit_min", last_result_.corner_speed_limit_min);
    add_number("corner_limited_pieces", static_cast<double>(last_result_.corner_limited_pieces));
    add_number("goal_epoch", static_cast<double>(goal_epoch_));
    add_number("path_version", static_cast<double>(path_version_));
    add_number("local_path_points", static_cast<double>(last_result_.local_path.size()));
    add_number("iterations", static_cast<double>(last_result_.iterations));
  }
  array.status.push_back(status);
  diagnostics_pub_->publish(array);
}

}  // namespace srm27_minco_planner

PLUGINLIB_EXPORT_CLASS(srm27_minco_planner::MincoPlanner, nav2_core::GlobalPlanner)
