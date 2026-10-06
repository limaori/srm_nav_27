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

#include "srm27_minco_controller/yaw_policy.hpp"

#include <algorithm>
#include <cmath>

#include "srm27_minco_core/kinematics.hpp"

namespace srm27_minco_controller
{

bool parseYawMode(const std::string & _text, YawMode & _mode)
{
  if (_text == "xy_only") {
    _mode = YawMode::kXyOnly;
    return true;
  }
  if (_text == "follow_tangent") {
    _mode = YawMode::kFollowTangent;
    return true;
  }
  if (_text == "spin") {
    _mode = YawMode::kSpin;
    return true;
  }
  if (_text == "narrow_track") {
    _mode = YawMode::kNarrowTrack;
    return true;
  }
  return false;
}

const char * toString(YawMode _mode)
{
  switch (_mode) {
    case YawMode::kXyOnly:
      return "xy_only";
    case YawMode::kFollowTangent:
      return "follow_tangent";
    case YawMode::kSpin:
      return "spin";
    case YawMode::kNarrowTrack:
      return "narrow_track";
  }
  return "unknown";
}

const char * YawPolicy::modeName() const { return toString(config_.mode); }

bool YawPolicy::configure(const Config & _config, std::string * _reason)
{
  const auto fail = [&_reason](const char * _message) {
    if (_reason != nullptr) {
      *_reason = _message;
    }
    return false;
  };
  configured_ = false;
  if (!std::isfinite(_config.max_angular_speed) || _config.max_angular_speed < 0.0) {
    return fail("max_angular_speed must be non-negative and finite");
  }
  if (!std::isfinite(_config.max_angular_accel) || _config.max_angular_accel < 0.0) {
    return fail("max_angular_accel must be non-negative and finite");
  }
  if (
    !std::isfinite(_config.narrow_clearance_threshold) ||
    _config.narrow_clearance_threshold < 0.0) {
    return fail("narrow_clearance_threshold must be non-negative and finite");
  }
  if (_config.mode != YawMode::kXyOnly && _config.max_angular_speed <= 0.0) {
    return fail("non-xy_only yaw modes require a positive angular speed limit");
  }
  config_ = _config;
  configured_ = true;
  return true;
}

void YawPolicy::reset(double _yaw)
{
  current_yaw_ = _yaw;
  spin_omega_ = 0.0;
  spin_request_active_ = false;
  narrow_track_active_ = false;
  last_tangent_time_ = 0.0;
  last_tangent_yaw_ = _yaw;
  has_last_tangent_ = false;
}

void YawPolicy::setSpinRequest(double _omega)
{
  spin_omega_ = std::isfinite(_omega) ? _omega : 0.0;
  spin_request_active_ = true;
}

void YawPolicy::clearSpinRequest()
{
  spin_omega_ = 0.0;
  spin_request_active_ = false;
}

double YawPolicy::angularSpeedLimit() const
{
  // XY 模式：MPC 明确不许产生角速度，自转仍由独立链路负责（方案 §8.1 阶段一）。
  if (config_.mode == YawMode::kXyOnly) {
    return 0.0;
  }
  return config_.max_angular_speed;
}

double YawPolicy::angularAccelLimit() const
{
  if (config_.mode == YawMode::kXyOnly) {
    return 0.0;
  }
  return config_.max_angular_accel;
}

bool YawPolicy::yawReference(
  double _trajectory_time, const Eigen::Vector2d & _position, const Eigen::Vector2d & _velocity,
  bool _tangent_valid, const Eigen::Vector2d & _tangent, double _previous_yaw, double & _yaw,
  double & _omega)
{
  (void)_position;
  (void)_velocity;
  (void)_trajectory_time;
  if (!configured_) {
    return false;
  }

  switch (config_.mode) {
    case YawMode::kXyOnly: {
      // 平移模式下航向不进代价：参考恒等于当前连续航向，角速度参考为 0。
      _yaw = current_yaw_;
      _omega = 0.0;
      return true;
    }
    case YawMode::kFollowTangent:
    case YawMode::kNarrowTrack: {
      if (!_tangent_valid) {
        // 低速时切线不确定：保持上一有效朝向，避免 atan2(0,0) 触发跳变。
        _yaw = _previous_yaw;
        _omega = 0.0;
        return true;
      }
      const double tangent_yaw = std::atan2(_tangent.y(), _tangent.x());
      double candidate = srm27_minco_core::unwrapAngleNear(_previous_yaw, tangent_yaw);
      if (config_.allow_reverse) {
        // 正/反切向中选择相对当前连续航向更近者（方案 §8.2）。
        const double reversed =
          srm27_minco_core::unwrapAngleNear(_previous_yaw, tangent_yaw + 3.14159265358979323846);
        if (std::abs(reversed - _previous_yaw) < std::abs(candidate - _previous_yaw)) {
          candidate = reversed;
        }
      }
      _yaw = candidate;
      // 角速度参考由相邻采样点的切向参考差分得到；终点附近的收敛由跟踪参考的
      // taper 负责，这里不再叠加额外逻辑（方案 §8.2 的 EXIT_HOLD 属于后续阶段）。
      if (has_last_tangent_ && (_trajectory_time - last_tangent_time_) > 1.0e-6) {
        const double rate = srm27_minco_core::normalizeAngle(candidate - last_tangent_yaw_) /
                            (_trajectory_time - last_tangent_time_);
        _omega = srm27_minco_core::clampToRange(
          rate, -config_.max_angular_speed, config_.max_angular_speed);
      } else {
        _omega = 0.0;
      }
      last_tangent_time_ = _trajectory_time;
      last_tangent_yaw_ = candidate;
      has_last_tangent_ = true;
      return true;
    }
    case YawMode::kSpin: {
      // SPIN 参考 yaw 连续积分；请求过期后按停止处理，不自动沿用旧自转。
      // 没有有效请求时按停止处理，不自动沿用独立自转。
      const double requested = spin_request_active_ ? spin_omega_ : 0.0;
      _yaw = _previous_yaw;
      _omega = srm27_minco_core::clampToRange(
        requested, -config_.max_angular_speed, config_.max_angular_speed);
      return true;
    }
  }
  return false;
}

}  // namespace srm27_minco_controller
