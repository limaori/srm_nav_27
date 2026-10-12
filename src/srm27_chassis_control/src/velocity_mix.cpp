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

bool parseMixMode(const std::string & _text, MixMode & _mode)
{
  if (_text == "legacy_independent_rotation" || _text == "legacy" || _text == "rotation") {
    _mode = MixMode::kLegacyIndependentRotation;
    return true;
  }
  if (_text == "nav_se2" || _text == "nav") {
    _mode = MixMode::kNavSe2;
    return true;
  }
  return false;
}

const char * toString(MixMode _mode)
{
  switch (_mode) {
    case MixMode::kLegacyIndependentRotation:
      return "legacy_independent_rotation";
    case MixMode::kNavSe2:
      return "nav_se2";
  }
  return "unknown";
}

const char * wzSourceName(MixMode _mode)
{
  switch (_mode) {
    case MixMode::kLegacyIndependentRotation:
      return "rotation_velocity";
    case MixMode::kNavSe2:
      return "navigation";
  }
  return "unknown";
}

MixOutput mixVelocity(const MixInput & _input, const MixLimits & _limits, MixMode _mode)
{
  MixOutput output;
  output.mode = _mode;

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

  // wz 的来源由模式决定：nav_se2 下只有导航能在合成结果里出现（方案 §4.3）。
  const double wz_source =
    (_mode == MixMode::kNavSe2) ? _input.nav_wz : _input.rotation_wz;
  output.wz = clampFinite(wz_source, _limits.wz_max);
  // NaN 与任何值都不相等，因此非有限输入同样会被记为"被限幅"（它确实被改成了 0）。
  output.wz_clamped = !(wz_source == output.wz);
  return output;
}
}  // namespace srm27_chassis_control
