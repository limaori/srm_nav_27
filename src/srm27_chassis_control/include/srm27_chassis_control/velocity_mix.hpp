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
#include <string>

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

/// \brief `wz` 的所有权模式（方案 §4.3 的模式表）。
enum class MixMode {
  /// \brief 旧模式：`wz` 只来自独立自转链路 `rotation_velocity`，导航 `wz` 一律丢弃。
  kLegacyIndependentRotation = 0,
  /// \brief 迁移模式：`wz` 只来自导航（经 fakeyaw 转换后的导航速度），
  /// 独立自转输入被拒绝，绝不允许两路相加。
  kNavSe2 = 1,
};

/// \brief 速度合成输入。
struct MixInput
{
  /// \brief 导航平移速度，车体系 x 前（m/s）。
  double nav_vx{0.0};
  /// \brief 导航平移速度，车体系 y 左（m/s）。
  double nav_vy{0.0};
  /// \brief 导航角速度（rad/s）。`nav_se2` 模式下这是**唯一**的 `wz` 来源。
  double nav_wz{0.0};
  /// \brief 独立自转角速度（rad/s）。只在旧模式下参与合成。
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
  /// \brief 是否发生了平移限幅。
  bool clamped{false};
  /// \brief 本次 `wz` 是否被 `wz_max` 限幅（含非有限输入按 0 处理）。
  bool wz_clamped{false};
  /// \brief 本次生效的模式（诊断用）。
  MixMode mode{MixMode::kLegacyIndependentRotation};
};

/// \brief 按方案 §3.3 / §4.3 的规则合成导航平移与自转角速度。
///
/// \code
/// vx = clamp(nav_vx, -vx_max, vx_max)
/// vy = clamp(nav_vy, -vy_max, vy_max)
/// scale = min(1, v_max / hypot(vx, vy))   // 零向量时 scale = 1
/// vx *= scale
/// vy *= scale
/// wz = clamp(wz_source, -wz_max, wz_max)  // wz_source 由模式决定，绝不两路相加
/// \endcode
///
/// `wz_source` 的选择是**模式化**的，不允许用"`wz` 是否为零"去猜所有权：
///
///  * `kLegacyIndependentRotation`：`wz_source = rotation_wz`（导航 `wz` 由调用方丢弃）。
///  * `kNavSe2`：`wz_source = nav_wz`（独立自转由调用方拒绝，不进入本函数）。
///
/// 非有限输入按 0 处理。
MixOutput mixVelocity(
  const MixInput & _input, const MixLimits & _limits,
  MixMode _mode = MixMode::kLegacyIndependentRotation);

/// \brief 单轴限幅，非有限值返回 0。
double clampFinite(double _value, double _limit);

/// \brief 由字符串解析 `wz` 所有权模式；无法识别时返回 false。
bool parseMixMode(const std::string & _text, MixMode & _mode);

/// \brief 模式转字符串（参数回读与诊断用）。
const char * toString(MixMode _mode);

/// \brief `wz` 来源的名字（诊断字段 `wz_source` 用）。
const char * wzSourceName(MixMode _mode);
}  // namespace srm27_chassis_control

#endif  // SRM27_CHASSIS_CONTROL__VELOCITY_MIX_HPP_
