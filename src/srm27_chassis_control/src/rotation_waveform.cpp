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

#include "srm27_chassis_control/rotation_waveform.hpp"

#include <cmath>
#include <limits>

namespace srm27_chassis_control
{
namespace
{
constexpr double kTwoPi = 6.28318530717958647692;
}  // namespace

bool parseRotationMode(const std::string & _text, RotationMode & _mode)
{
  if (_text == "stop") {
    _mode = RotationMode::kStop;
    return true;
  }
  if (_text == "constant") {
    _mode = RotationMode::kConstant;
    return true;
  }
  if (_text == "periodic") {
    _mode = RotationMode::kPeriodic;
    return true;
  }
  return false;
}

std::string toString(RotationMode _mode)
{
  switch (_mode) {
    case RotationMode::kConstant:
      return "constant";
    case RotationMode::kPeriodic:
      return "periodic";
    case RotationMode::kStop:
    default:
      return "stop";
  }
}

bool validateWaveform(
  const RotationWaveformParams & _params, std::string * _error)
{
  const auto fail = [&_error](const std::string & _message) {
      if (_error != nullptr) {
        *_error = _message;
      }
      return false;
    };

  if (!std::isfinite(_params.angular_speed)) {
    return fail("angular_speed 必须是有限值");
  }
  if (!std::isfinite(_params.offset)) {
    return fail("offset 必须是有限值");
  }
  if (!std::isfinite(_params.amplitude)) {
    return fail("amplitude 必须是有限值");
  }
  if (_params.amplitude < 0.0) {
    return fail("amplitude 不能为负");
  }
  if (!std::isfinite(_params.phase)) {
    return fail("phase 必须是有限值");
  }
  if (!std::isfinite(_params.period)) {
    return fail("period 必须是有限值");
  }
  if (!std::isfinite(_params.max_accel)) {
    return fail("max_accel 必须是有限值");
  }
  if (_params.mode == RotationMode::kPeriodic && _params.period <= 0.0) {
    return fail("周期模式下 period 必须为正");
  }
  return true;
}

double computeRotationVelocity(
  const RotationWaveformParams & _params, double _t)
{
  switch (_params.mode) {
    case RotationMode::kConstant:
      return _params.angular_speed;
    case RotationMode::kPeriodic:
      {
        const double argument = kTwoPi * _t / _params.period + _params.phase;
        if (_params.sine_wave) {
          return _params.offset + _params.amplitude * std::sin(argument);
        }
        // 方波：sin 的符号决定正负幅度；sin 恰为 0 时按正幅度处理，
        // 保证输出在 offset ± amplitude 之间跳变。
        const double sign = std::sin(argument) < 0.0 ? -1.0 : 1.0;
        return _params.offset + _params.amplitude * sign;
      }
    case RotationMode::kStop:
    default:
      return 0.0;
  }
}

double applyAngularAccelLimit(
  double _target, double _previous, double _dt, double _max_accel)
{
  if (!std::isfinite(_target)) {
    return 0.0;
  }
  // 停止命令和超时清零优先：目标为零时直接清零，不做减速斜坡。
  if (_target == 0.0) {
    return 0.0;
  }
  if (!std::isfinite(_max_accel) || _max_accel <= 0.0 ||
    !std::isfinite(_previous) || !std::isfinite(_dt) || _dt <= 0.0)
  {
    return _target;
  }
  const double maxDelta = _max_accel * _dt;
  const double delta = _target - _previous;
  if (std::abs(delta) <= maxDelta) {
    return _target;
  }
  return _previous + std::copysign(maxDelta, delta);
}
}  // namespace srm27_chassis_control
