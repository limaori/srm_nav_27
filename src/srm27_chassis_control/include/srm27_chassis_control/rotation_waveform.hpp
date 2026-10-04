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

#ifndef SRM27_CHASSIS_CONTROL__ROTATION_WAVEFORM_HPP_
#define SRM27_CHASSIS_CONTROL__ROTATION_WAVEFORM_HPP_

#include <string>

namespace srm27_chassis_control
{
/// \brief 独立自转的测试模式。
enum class RotationMode
{
  kStop,      ///< 持续输出零角速度
  kConstant,  ///< 恒速
  kPeriodic,  ///< 周期波形（正弦或方波）
};

/// \brief 解析模式字符串；无法识别时返回 false。
bool parseRotationMode(const std::string & _text, RotationMode & _mode);

/// \brief 模式转字符串。
std::string toString(RotationMode _mode);

/// \brief 自转波形参数。来自 launch / YAML，不塞进 Twist 的其他分量。
struct RotationWaveformParams
{
  /// \brief 测试模式。
  RotationMode mode{RotationMode::kStop};
  /// \brief 恒速模式的有符号角速度（rad/s）。
  double angular_speed{0.0};
  /// \brief 周期模式的平均角速度（rad/s）。
  double offset{0.0};
  /// \brief 周期模式的非负变化幅度（rad/s）。
  double amplitude{0.0};
  /// \brief 周期（s），周期模式必须为正。
  double period{4.0};
  /// \brief 初相位（rad）。
  double phase{0.0};
  /// \brief true 为正弦，false 为方波。
  bool sine_wave{true};
  /// \brief 角加速度上限（rad/s^2），<= 0 表示不限制。
  double max_accel{0.0};
};

/// \brief 校验波形参数。
///
/// 拒绝非有限速度、负幅度和周期模式下的非正周期。
bool validateWaveform(
  const RotationWaveformParams & _params, std::string * _error);

/// \brief 计算时刻 _t（秒，相对波形起点）的角速度（rad/s）。
double computeRotationVelocity(
  const RotationWaveformParams & _params, double _t);

/// \brief 按角加速度上限把目标角速度限速到上一时刻的值附近。
///
/// 停止命令（目标为 0）优先，不受加速度限制，直接清零。
double applyAngularAccelLimit(
  double _target, double _previous, double _dt, double _max_accel);
}  // namespace srm27_chassis_control

#endif  // SRM27_CHASSIS_CONTROL__ROTATION_WAVEFORM_HPP_
