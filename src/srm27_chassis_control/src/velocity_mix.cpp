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

  output.wz = clampFinite(_input.rotation_wz, _limits.wz_max);
  return output;
}
}  // namespace srm27_chassis_control
