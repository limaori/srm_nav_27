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

#ifndef SRM27_CHASSIS_CONTROL__VELOCITY_MIX_HPP_
#define SRM27_CHASSIS_CONTROL__VELOCITY_MIX_HPP_

#include <cmath>

namespace srm27_chassis_control
{
/// \brief 速度输出的限幅参数。
struct MixLimits
{
  /// \brief 前后方向速度上限（m/s）。
  double vx_max{0.5};
  /// \brief 左右方向速度上限（m/s）。
  double vy_max{0.5};
  /// \brief 平移合速度上限（m/s），按向量模长限幅。
  double v_max{0.5};
  /// \brief 自转角速度上限（rad/s）。
  double wz_max{2.0};
};

/// \brief 速度输入：**唯一**来源是导航（`vx` / `vy` / `wz`）。
///
/// 2026-10-19 起独立自转链路（`rotation_controller` → `rotation_velocity`）已经删除：
/// `wz` 归上位机（导航栈）所有，不存在第二路自转输入，也就不存在两路 `wz` 相加。
struct MixInput
{
  /// \brief 导航平移速度，车体系 x 前（m/s）。
  double nav_vx{0.0};
  /// \brief 导航平移速度，车体系 y 左（m/s）。
  double nav_vy{0.0};
  /// \brief 导航角速度（rad/s），`wz` 的唯一来源。
  double nav_wz{0.0};
};

/// \brief 速度输出。
struct MixOutput
{
  /// \brief 输出前向速度（m/s）。
  double vx{0.0};
  /// \brief 输出左向速度（m/s）。
  double vy{0.0};
  /// \brief 输出自转角速度（rad/s）。
  double wz{0.0};
  /// \brief 平移限幅比例，1 表示未限幅。
  double scale{1.0};
  /// \brief 是否发生了平移限幅。
  bool clamped{false};
  /// \brief 本次 `wz` 是否被 `wz_max` 限幅（含非有限输入按 0 处理）。
  bool wz_clamped{false};
};

/// \brief 按方案 §3.3 / §4.3 的规则对导航速度做限幅（不含任何自转叠加）。
///
/// \code
/// vx = clamp(nav_vx, -vx_max, vx_max)
/// vy = clamp(nav_vy, -vy_max, vy_max)
/// scale = min(1, v_max / hypot(vx, vy))   // 零向量时 scale = 1
/// vx *= scale
/// vy *= scale
/// wz = clamp(nav_wz, -wz_max, wz_max)
/// \endcode
///
/// 非有限输入按 0 处理。
MixOutput mixVelocity(const MixInput & _input, const MixLimits & _limits);

/// \brief 单轴限幅，非有限值返回 0。
double clampFinite(double _value, double _limit);
}  // namespace srm27_chassis_control

#endif  // SRM27_CHASSIS_CONTROL__VELOCITY_MIX_HPP_
