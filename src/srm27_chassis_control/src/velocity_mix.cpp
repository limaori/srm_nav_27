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

#include "srm27_chassis_control/velocity_mix.hpp"

#include <algorithm>

namespace srm27_chassis_control
{
double clampFinite(double _value, double _limit)
{
  if (!std::isfinite(_value)) {
    return 0.0;
  }
  const double limit = std::isfinite(_limit) ? std::abs(_limit) : 0.0;
  return std::clamp(_value, -limit, limit);
}

bool isNonZeroRate(double _wz)
{
  if (!std::isfinite(_wz)) {
    return true;
  }
  return std::abs(_wz) > 1.0e-9;
}

double slewLimit(double _target, double _previous, double _max_rate, double _dt)
{
  if (!std::isfinite(_target)) {
    return _previous;
  }
  if (!std::isfinite(_max_rate) || _max_rate <= 0.0 || !std::isfinite(_dt) || _dt <= 0.0) {
    return _target;
  }
  const double step = _max_rate * _dt;
  const double delta = _target - _previous;
  if (std::abs(delta) <= step) {
    return _target;
  }
  return _previous + (delta > 0.0 ? step : -step);
}

MixOutput mixVelocity(const MixInput & _input, const MixLimits & _limits)
{
  MixOutput output;

  output.vx = clampFinite(_input.nav_vx, _limits.vx_max);
  output.vy = clampFinite(_input.nav_vy, _limits.vy_max);

  const double magnitude = std::hypot(output.vx, output.vy);
  const double vMax = std::isfinite(_limits.v_max) ? std::abs(_limits.v_max) : 0.0;
  if (magnitude > 0.0 && vMax > 0.0 && magnitude > vMax) {
    output.scale = vMax / magnitude;
    output.clamped = true;
  } else if (magnitude > 0.0 && vMax <= 0.0) {
    // v_max 非法或为 0 时不允许运动，避免出现不受限的平移。
    output.scale = 0.0;
    output.clamped = true;
  }
  output.vx *= output.scale;
  output.vy *= output.scale;

  // ---- wz：两路输入，二选一（方案 §4.3）----
  if (_input.spin_enabled) {
    // 双输入：导航的 angular.z 一律忽略（它本该是 0，非零说明上游有节点在往导航出口发 wz）。
    output.wz_from_spin = true;
    output.nav_wz_ignored = isNonZeroRate(_input.nav_wz);
    // 决策层停发/超时 = 停转：按 0 处理，不沿用上一帧。
    const double requested = _input.spin_valid ? _input.spin_wz : 0.0;
    output.wz = clampFinite(requested, _limits.wz_max);
    // NaN 与任何值都不相等，因此非有限输入同样会被记为"被限幅"（它确实被改成了 0）。
    output.wz_clamped = !(requested == output.wz);
  } else {
    // 单输入（旧行为 / 仿真链 / 回退）：导航是 wz 的唯一来源，这里只做限幅。
    output.wz = clampFinite(_input.nav_wz, _limits.wz_max);
    output.wz_clamped = !(_input.nav_wz == output.wz);
  }
  return output;
}
}  // namespace srm27_chassis_control
