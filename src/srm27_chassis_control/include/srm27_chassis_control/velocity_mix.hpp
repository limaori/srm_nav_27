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
/// \brief 速度合成的限幅参数。
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

/// \brief 速度合成输入。
struct MixInput
{
  /// \brief 导航平移速度，车体系 x 前（m/s）。
  double nav_vx{0.0};
  /// \brief 导航平移速度，车体系 y 左（m/s）。
  double nav_vy{0.0};
  /// \brief 独立自转角速度（rad/s）。导航输入的角速度不会出现在这里。
  double rotation_wz{0.0};
};

/// \brief 速度合成输出。
struct MixOutput
{
  /// \brief 合成后的前向速度（m/s）。
  double vx{0.0};
  /// \brief 合成后的左向速度（m/s）。
  double vy{0.0};
  /// \brief 合成后的自转角速度（rad/s）。
  double wz{0.0};
  /// \brief 平移限幅比例，1 表示未限幅。
  double scale{1.0};
  /// \brief 是否发生了限幅。
  bool clamped{false};
};

/// \brief 按方案 3.3 的规则合成导航平移与独立自转速度。
///
/// \code
/// vx = clamp(nav_vx, -vx_max, vx_max)
/// vy = clamp(nav_vy, -vy_max, vy_max)
/// scale = min(1, v_max / hypot(vx, vy))   // 零向量时 scale = 1
/// vx *= scale
/// vy *= scale
/// wz = clamp(rotation_wz, -wz_max, wz_max)
/// \endcode
///
/// 导航输入的 angular.z 在调用本函数之前就被丢弃，因此这里只可能拿到
/// 独立自转的 wz。非有限输入按 0 处理。
MixOutput mixVelocity(const MixInput & _input, const MixLimits & _limits);

/// \brief 单轴限幅，非有限值返回 0。
double clampFinite(double _value, double _limit);
}  // namespace srm27_chassis_control

#endif  // SRM27_CHASSIS_CONTROL__VELOCITY_MIX_HPP_
