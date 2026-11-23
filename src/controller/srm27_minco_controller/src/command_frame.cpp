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

#include "srm27_minco_controller/command_frame.hpp"

#include "srm27_minco_core/esdf_2d.hpp"

namespace srm27_minco_controller
{

bool resolveCommandFrame(
  const std::string & _command_frame, const std::string & _base_frame,
  const std::string & _fake_base_frame, CommandFrameMode & _mode, std::string * _reason)
{
  const auto fail = [&_reason](const std::string & _message) {
    if (_reason != nullptr) {
      *_reason = _message;
    }
    return false;
  };

  if (_command_frame.empty()) {
    return fail("command_frame must not be empty");
  }
  if (_command_frame == _base_frame) {
    _mode = CommandFrameMode::kRobotBase;
    return true;
  }
  if (_command_frame == _fake_base_frame) {
    _mode = CommandFrameMode::kOdomAlignedFake;
    return true;
  }
  // 关键：不允许"第三个名字"。命名与实际是否旋转必须一一对应，否则"到底转了几次"
  // 就成了隐含约定，而重复旋转的表现（psi≈pi/2 时前进变侧移）极难现场定位。
  return fail(
    "command_frame='" + _command_frame + "' 既不是 base_frame='" + _base_frame +
    "' 也不是 fake_base_frame='" + _fake_base_frame + "'");
}

Eigen::Vector2d brakingPathLimitedVelocity(
  const srm27_minco_core::Esdf2D & _esdf, const Eigen::Vector2d & _position,
  const Eigen::Vector2d & _velocity, double _required_clearance,
  double _braking_deceleration, double _reaction_latency)
{
  const auto here = _esdf.query(_position.x(), _position.y());
  if (!here.valid) {
    return Eigen::Vector2d::Zero();
  }
  const Eigen::Vector2d velocity = clearanceLimitedVelocity(
    _velocity, here.gradient, here.distance, _required_clearance,
    _braking_deceleration, _reaction_latency);
  const double speed = velocity.norm();
  if (speed < 1.0e-6) {
    return velocity;
  }

  const Eigen::Vector2d direction = velocity / speed;
  const double stopping_distance = speed * _reaction_latency +
    speed * speed / (2.0 * _braking_deceleration);
  const double step = 0.5 * _esdf.resolution();
  // If already inside the margin, allow an escape that does not reduce clearance.
  const double clearance = std::min(here.distance, _required_clearance);
  const auto clearAt = [&](double distance) {
    const Eigen::Vector2d point = _position + direction * distance;
    const auto query = _esdf.query(point.x(), point.y());
    return query.valid && query.distance + 1.0e-6 >= clearance;
  };

  double safe_distance = 0.0;
  while (safe_distance < stopping_distance) {
    double next = std::min(stopping_distance, safe_distance + step);
    if (clearAt(next)) {
      safe_distance = next;
      continue;
    }
    for (int i = 0; i < 5; ++i) {
      const double midpoint = (safe_distance + next) * 0.5;
      if (clearAt(midpoint)) {
        safe_distance = midpoint;
      } else {
        next = midpoint;
      }
    }
    const double denominator = _reaction_latency + std::sqrt(
      _reaction_latency * _reaction_latency + 2.0 * safe_distance / _braking_deceleration);
    const double safe_speed = denominator > 0.0 ? 2.0 * safe_distance / denominator : 0.0;
    return direction * safe_speed;
  }
  return velocity;
}

Eigen::Vector2d limitExecutionVelocity(
  const srm27_minco_core::Esdf2D & _esdf, const Eigen::Vector2d & _position,
  const Eigen::Vector2d & _requested, const Eigen::Vector2d & _measured,
  double _required_clearance, double _braking_deceleration, double _reaction_latency)
{
  const Eigen::Vector2d measured_limit = brakingPathLimitedVelocity(
    _esdf, _position, _measured, _required_clearance, _braking_deceleration, _reaction_latency);
  Eigen::Vector2d command = _requested;
  if ((_measured - measured_limit).norm() > 1.0e-6) {
    // The serial/smoother ramps cannot immediately redirect the measured
    // velocity. Keep its approach component within the available braking room.
    const Eigen::Vector2d approach = _measured.normalized();
    const double safe_approach = std::max(0.0, measured_limit.dot(approach));
    command -= std::max(0.0, command.dot(approach) - safe_approach) * approach;
  }
  return brakingPathLimitedVelocity(
    _esdf, _position, command, _required_clearance, _braking_deceleration, _reaction_latency);
}

}  // namespace srm27_minco_controller
