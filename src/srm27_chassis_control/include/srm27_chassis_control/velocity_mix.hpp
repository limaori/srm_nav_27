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
  /// \brief `wz` 变化率上限（rad/s²），<= 0 表示不做斜率限制。
  ///
  /// 与 `speed_limits.yaml` 的 `FollowPath.limits.max_angular_accel` 同档（默认 4.5）：
  /// 决策层发的是**阶跃式**的转速请求（`rotation_cmd` 只有目标值），不限制斜率的话
  /// 起停瞬间底盘会挨一次冲击。4.5 rad/s² 下从 0 到 1.0 rad/s 约 0.22 s。
  double wz_ramp{0.0};
};

/// \brief 速度输入：平移来自导航，自转来自决策层（方案 §4.3）。
///
/// 云台系（2026-11-08）之后三件事同时成立：
///   * 导航只出平移（`yaw_policy.mode = xy_only`），它的 `angular.z` 正常情况下恒为 0；
///   * 底盘 `wz` 由**决策层**通过 `rotation_cmd` 给出，在 mux 里合成；
///   * `spin_enabled = false` 时回到旧行为（单输入，导航同时拥有平移与 wz），
///     这样仿真链与旧链不受影响（回退开关就是 mux 的 `spin_topic` 留空）。
struct MixInput
{
  /// \brief 导航平移速度，车体系 x 前（m/s）。
  double nav_vx{0.0};
  /// \brief 导航平移速度，车体系 y 左（m/s）。
  double nav_vy{0.0};
  /// \brief 导航角速度（rad/s）。
  ///
  /// `spin_enabled = false` 时它是 `wz` 的唯一来源（旧行为）；
  /// `spin_enabled = true` 时它**一律被忽略**（并计入 `nav_wz_ignored`）。
  double nav_wz{0.0};
  /// \brief 是否启用第二路（决策层）自转输入。
  bool spin_enabled{false};
  /// \brief 决策层请求的底盘角速度（rad/s），俯视逆时针为正。
  double spin_wz{0.0};
  /// \brief 决策层输入本周期是否新鲜（未超时、非急停）。
  ///
  /// false 时按"停转"处理：`wz = 0`，而不是沿用上一帧。
  bool spin_valid{false};
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
  /// \brief `wz` 是否取自决策层输入（false = 取自导航 / 单输入模式）。
  bool wz_from_spin{false};
  /// \brief 本次导航输入带了非零 `angular.z` 且被忽略（双输入模式下才会为 true）。
  bool nav_wz_ignored{false};
};

/// \brief 按方案 §3.3 / §4.3 的规则对两路输入做合成与限幅（不含斜率限制）。
///
/// \code
/// vx = clamp(nav_vx, -vx_max, vx_max)
/// vy = clamp(nav_vy, -vy_max, vy_max)
/// scale = min(1, v_max / hypot(vx, vy))   // 零向量时 scale = 1
/// vx *= scale
/// vy *= scale
/// // 单输入（spin_enabled=false）：wz = clamp(nav_wz, -wz_max, wz_max)
/// // 双输入（spin_enabled=true）：  wz = clamp(spin_valid ? spin_wz : 0, -wz_max, wz_max)
/// \endcode
///
/// 非有限输入按 0 处理。
MixOutput mixVelocity(const MixInput & _input, const MixLimits & _limits);

/// \brief 单轴限幅，非有限值返回 0。
double clampFinite(double _value, double _limit);

/// \brief `wz` 的变化率限制（一阶 slew rate）。
///
/// 把 `_previous` 朝 `_target` 靠拢，单步最多变化 `_max_rate * _dt`。
/// `_max_rate` 非正、`_dt` 非正时直接返回 `_target`（等于不做限制）。
/// 结果按符号对称；`_target` 非有限时返回 `_previous`（不放大异常值）。
double slewLimit(
  double _target, double _previous, double _max_rate, double _dt);

/// \brief 判断 `wz` 的"非零"（含非有限值），用于"导航漏出 wz"的判定。
bool isNonZeroRate(double _wz);
}  // namespace srm27_chassis_control

#endif  // SRM27_CHASSIS_CONTROL__VELOCITY_MIX_HPP_
