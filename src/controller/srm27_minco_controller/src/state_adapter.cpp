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

#include "srm27_minco_controller/state_adapter.hpp"

#include <tf2/utils.h>

#include <chrono>
#include <cmath>
#include <limits>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "srm27_minco_core/kinematics.hpp"

namespace srm27_minco_controller
{

namespace
{

/// \brief 当前单调时钟（秒），只用于超时与耗时。
double steadyNow()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
           std::chrono::steady_clock::now().time_since_epoch())
           .count() *
         1.0e-9;
}

}  // namespace

bool StateAdapter::configure(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & _node, std::shared_ptr<tf2_ros::Buffer> _tf,
  const Config & _config, std::string * _reason)
{
  const auto fail = [&_reason](const char * _message) {
    if (_reason != nullptr) {
      *_reason = _message;
    }
    return false;
  };

  reset();
  if (!_node) {
    return fail("state adapter requires a parent node");
  }
  if (_config.odometry_topic.empty()) {
    return fail("odometry_topic must be set");
  }
  if (_config.base_frame.empty()) {
    return fail("base_frame must be set");
  }
  if (!std::isfinite(_config.timeout) || _config.timeout <= 0.0) {
    return fail("state timeout must be positive");
  }
  if (!std::isfinite(_config.max_sample_gap) || _config.max_sample_gap <= 0.0) {
    return fail("max_sample_gap must be positive");
  }
  if (!std::isfinite(_config.max_position_jump) || _config.max_position_jump <= 0.0) {
    return fail("max_position_jump must be positive");
  }
  if (_config.use_tf && !_tf) {
    return fail("use_tf is enabled but no TF buffer was provided");
  }

  node_ = _node;
  tf_ = std::move(_tf);
  config_ = _config;

  // best_effort 与 reliable 发布者兼容，避免里程计 QoS 不匹配导致“状态永远不新鲜”。
  subscription_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    config_.odometry_topic, rclcpp::QoS(10).best_effort(),
    std::bind(&StateAdapter::odometryCallback, this, std::placeholders::_1));
  configured_ = true;
  return true;
}

void StateAdapter::reset()
{
  subscription_.reset();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_ = nav_msgs::msg::Odometry();
    previous_ = nav_msgs::msg::Odometry();
    has_latest_ = false;
    has_previous_ = false;
    latest_received_steady_ = 0.0;
    filtered_velocity_.setZero();
    has_filtered_velocity_ = false;
  }
  configured_ = false;
}

void StateAdapter::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr _message)
{
  if (!_message) {
    return;
  }
  const double received = steadyNow();
  std::lock_guard<std::mutex> lock(mutex_);

  // 时间戳重复或倒退的采样**直接丢弃**：它比手上已有的状态更旧，既不能用来做差分
  // （分母为负会造出方向相反的速度尖峰），也不该覆盖更新的位姿。
  const double incoming_stamp = rclcpp::Time(_message->header.stamp).seconds();
  if (has_latest_ && incoming_stamp <= rclcpp::Time(latest_.header.stamp).seconds() + 1.0e-9) {
    dropped_sample_count_.fetch_add(1);
    return;
  }

  if (has_latest_) {
    const double dx = _message->pose.pose.position.x - latest_.pose.pose.position.x;
    const double dy = _message->pose.pose.position.y - latest_.pose.pose.position.y;
    const double dyaw = srm27_minco_core::normalizeAngle(
      tf2::getYaw(_message->pose.pose.orientation) - tf2::getYaw(latest_.pose.pose.orientation));
    if (std::hypot(dx, dy) > config_.max_position_jump || std::abs(dyaw) > config_.max_yaw_jump) {
      // 位姿跳变按定位重置处理，计数变化会让控制器作废旧轨迹。
      reset_count_.fetch_add(1);
      has_previous_ = false;
      has_filtered_velocity_ = false;
      filtered_velocity_.setZero();
    }
  }
  previous_ = latest_;
  has_previous_ = has_latest_;
  latest_ = *_message;
  has_latest_ = true;
  latest_received_steady_ = received;
}

double StateAdapter::lastSampleStamp() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_latest_) {
    return 0.0;
  }
  return rclcpp::Time(latest_.header.stamp).seconds();
}

double StateAdapter::lastReceivedSteady() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return latest_received_steady_;
}

bool StateAdapter::fresh(double _now_stamp, double _now_steady, std::string * _reason) const
{
  const auto fail = [&_reason](const char * _message) {
    if (_reason != nullptr) {
      *_reason = _message;
    }
    return false;
  };
  if (!configured_) {
    return fail("state adapter not configured");
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_latest_) {
    return fail("no odometry received yet");
  }
  if (
    std::isfinite(_now_steady) && latest_received_steady_ > 0.0 &&
    (_now_steady - latest_received_steady_) > config_.timeout) {
    return fail("odometry reception is older than the configured timeout");
  }
  const double sample_stamp = rclcpp::Time(latest_.header.stamp).seconds();
  if (std::isfinite(_now_stamp) && (_now_stamp - sample_stamp) > config_.timeout) {
    return fail("odometry sample is older than the configured timeout");
  }
  return true;
}

bool StateAdapter::stateAt(
  double _now_stamp, double _now_steady, State2D & _state, std::string * _reason)
{
  _state = State2D();
  if (!fresh(_now_stamp, _now_steady, _reason)) {
    return false;
  }

  nav_msgs::msg::Odometry current;
  nav_msgs::msg::Odometry previous;
  bool has_previous = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    current = latest_;
    previous = previous_;
    has_previous = has_previous_;
  }

  const double sample_stamp = rclcpp::Time(current.header.stamp).seconds();
  const std::string odom_frame =
    current.header.frame_id.empty() ? std::string("odom") : current.header.frame_id;
  const std::string child_frame =
    current.child_frame_id.empty() ? config_.base_frame : current.child_frame_id;

  // 1) 位姿：里程计给出的位置在 header.frame_id 下。
  double x = current.pose.pose.position.x;
  double y = current.pose.pose.position.y;
  double yaw = tf2::getYaw(current.pose.pose.orientation);

  // 2) 位姿转换到规划系（`map` -> `odom` 变化时需要重新转换；这里每次都用最新 TF）。
  const std::string target_frame =
    config_.planning_frame.empty() ? odom_frame : config_.planning_frame;
  if (config_.use_tf && target_frame != odom_frame) {
    if (!tf_) {
      if (_reason != nullptr) {
        *_reason = "TF buffer unavailable for planning frame conversion";
      }
      return false;
    }
    try {
      // 不使用超时：本函数运行在 executor 回调里，带超时查询可能与 TF 监听器
      // 争用同一线程而阻塞控制周期（tf2 也会对此告警）。查不到就明确失效。
      const geometry_msgs::msg::TransformStamped transform = tf_->lookupTransform(
        target_frame, odom_frame, current.header.stamp, rclcpp::Duration::from_seconds(0.0));
      tf2::Transform tf_odom_to_planning;
      tf2::fromMsg(transform.transform, tf_odom_to_planning);
      const tf2::Vector3 point = tf_odom_to_planning * tf2::Vector3(x, y, 0.0);
      x = point.x();
      y = point.y();
      yaw = srm27_minco_core::normalizeAngle(yaw + tf2::getYaw(tf_odom_to_planning.getRotation()));
    } catch (const tf2::TransformException & exception) {
      // 明确失效，不用单位变换伪装成有效定位。
      if (_reason != nullptr) {
        *_reason = std::string("TF lookup failed for planning frame: ") + exception.what();
      }
      return false;
    }
  }

  // 3) 速度：twist 表达在 child_frame 下，旋转到规划系。
  Eigen::Vector2d velocity_odom = Eigen::Vector2d::Zero();
  double omega = current.twist.twist.angular.z;
  const double vx_child = current.twist.twist.linear.x;
  const double vy_child = current.twist.twist.linear.y;
  const bool twist_present =
    std::abs(vx_child) > 1.0e-9 || std::abs(vy_child) > 1.0e-9 || std::abs(omega) > 1.0e-9;
  if (twist_present) {
    velocity_odom = config_.twist_in_child_frame ? srm27_minco_core::bodyToOdomVelocity(
                                                     yaw, Eigen::Vector2d(vx_child, vy_child))
                                                 : Eigen::Vector2d(vx_child, vy_child);
  } else if (has_previous) {
    // 回退：用消息采样时间差做位姿差分（不是回调到达时间差）。
    const double previous_stamp = rclcpp::Time(previous.header.stamp).seconds();
    const double dt = sample_stamp - previous_stamp;
    if (dt > 1.0e-6 && dt <= config_.max_sample_gap) {
      // odom 系下的位姿差分本身就是车体原点在 odom 系的速度，不需要经过车体系
      // 往返旋转：`R(ψ_k)·R(ψ_{k-1})^T` 会额外引入一个 Δψ 的错误旋转分量。
      const Eigen::Vector2d delta(
        x - previous.pose.pose.position.x, y - previous.pose.pose.position.y);
      velocity_odom = delta / dt;
      const double previous_yaw = tf2::getYaw(previous.pose.pose.orientation);
      const double omega_delta = srm27_minco_core::normalizeAngle(yaw - previous_yaw);
      if (std::abs(omega_delta) <= config_.max_yaw_jump) {
        omega = omega_delta / dt;
      }
    } else {
      if (_reason != nullptr) {
        *_reason = "odometry twist is empty and the pose difference interval is unusable";
      }
      return false;
    }
  } else {
    // 只有一帧且没有 twist：位置可用，速度明确标记为 0 并靠调用方判断。
    velocity_odom.setZero();
    omega = 0.0;
  }

  // 4) 短窗口一阶低通，抑制差分噪声；不使用会掩盖真实跳变的强滤波。
  if (has_filtered_velocity_) {
    const double alpha = srm27_minco_core::clampToRange(config_.velocity_filter_alpha, 0.0, 1.0);
    filtered_velocity_ = (1.0 - alpha) * filtered_velocity_ + alpha * velocity_odom;
  } else {
    filtered_velocity_ = velocity_odom;
    has_filtered_velocity_ = true;
  }
  velocity_odom = filtered_velocity_;

  if (
    !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw) || !velocity_odom.allFinite() ||
    !std::isfinite(omega)) {
    if (_reason != nullptr) {
      *_reason = "non-finite odometry content";
    }
    return false;
  }

  _state.valid = true;
  _state.x = x;
  _state.y = y;
  _state.yaw = yaw;
  _state.velocity = velocity_odom;
  _state.omega = omega;
  _state.acceleration.setZero();
  _state.acceleration_valid = false;
  _state.sample_stamp = sample_stamp;
  _state.received_stamp = _now_steady;
  _state.source = "odometry(" + odom_frame + "->" + child_frame + ")";

  // 5) 外推到当前时刻（状态对齐）。轨迹进度是几何投影，不按墙钟推进，
  //    所以这里的外推不会与轨迹时间重复计算同一段延迟（方案 §5.2）。
  if (std::isfinite(_now_stamp)) {
    const double dt = _now_stamp - sample_stamp;
    if (dt > 0.0 && dt <= config_.max_sample_gap) {
      _state.x += _state.velocity.x() * dt;
      _state.y += _state.velocity.y() * dt;
      _state.yaw = srm27_minco_core::advanceUnwrappedYaw(_state.yaw, _state.omega, dt);
    }
  }
  return true;
}

}  // namespace srm27_minco_controller
