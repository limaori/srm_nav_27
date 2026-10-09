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

#ifndef SRM27_MINCO_CORE__TYPES_HPP_
#define SRM27_MINCO_CORE__TYPES_HPP_

#include <Eigen/Core>
#include <cstdint>
#include <limits>
#include <string>

#include "srm27_minco_core/kinematics.hpp"

namespace srm27_minco_core
{

/// \brief 机器人二维状态，表达在连续的 `odom` 系。
///
/// 位置/速度/加速度全部在 `odom` 中；车体系分量由 kinematics.hpp 的转换函数得到。
/// 时间字段区分两类语义（方案 §5.2）：
///  * `sample_stamp`：传感器原始采样时刻（ROS 时间秒），用于状态对齐与外推。
///  * `received_stamp`：本地接收/产生该状态的单调时刻（秒），只用于超时与耗时统计。
struct State2D
{
  /// \brief 是否有效；无效状态不得参与控制。
  bool valid{false};
  /// \brief `odom` 系 x（m）。
  double x{0.0};
  /// \brief `odom` 系 y（m）。
  double y{0.0};
  /// \brief 连续展开后的航向（rad），不做 2*pi 折叠。
  double yaw{0.0};
  /// \brief `odom` 系平移速度（m/s）。
  Eigen::Vector2d velocity{Eigen::Vector2d::Zero()};
  /// \brief 平面角速度（rad/s），与坐标系旋转无关。
  double omega{0.0};
  /// \brief `odom` 系平移加速度（m/s^2）；`acceleration_valid` 为假时不得使用。
  Eigen::Vector2d acceleration{Eigen::Vector2d::Zero()};
  /// \brief 加速度估计是否可用。
  bool acceleration_valid{false};
  /// \brief 原始采样时间（ROS 时间秒）。
  double sample_stamp{0.0};
  /// \brief 接收/生成时刻（单调秒）。
  double received_stamp{0.0};
  /// \brief 状态来源描述，仅用于诊断。
  std::string source{"unknown"};

  /// \brief odom 系位置。
  Eigen::Vector2d position() const { return Eigen::Vector2d(x, y); }
  /// \brief odom 系平移速率（m/s）。
  double speed() const { return velocity.norm(); }
};

/// \brief 平移与自转的有效约束集合。
///
/// 所有分量都是“执行端真实能保证”的上限，不是优化器随手设置的期望值。
/// `max_linear_speed` 是平移合速度上限（向量模长），不是单轴上限。
struct Limits2D
{
  /// \brief 平移合速度上限（m/s）。
  double max_linear_speed{0.5};
  /// \brief 平移合加速度上限（m/s^2）。
  double max_linear_accel{0.3};
  /// \brief 平面角速度上限（rad/s）。
  double max_angular_speed{0.3};
  /// \brief 平面角加速度上限（rad/s^2）。
  double max_angular_accel{0.5};

  /// \brief 全部字段有限且非负。
  bool valid() const
  {
    return isFinite(max_linear_speed) && isFinite(max_linear_accel) &&
           isFinite(max_angular_speed) && isFinite(max_angular_accel) && max_linear_speed > 0.0 &&
           max_linear_accel > 0.0 && max_angular_speed >= 0.0 && max_angular_accel >= 0.0;
  }

  /// \brief 按比例收紧平移与自转上限（用于 Nav2 限速）。
  Limits2D scaled(const double _scale) const
  {
    Limits2D out = *this;
    const double scale = (_scale >= 0.0 && isFinite(_scale)) ? _scale : 0.0;
    out.max_linear_speed = max_linear_speed * scale;
    out.max_linear_accel = max_linear_accel * scale;
    out.max_angular_speed = max_angular_speed * scale;
    out.max_angular_accel = max_angular_accel * scale;
    return out;
  }
};

/// \brief 求解/校验的通用结果状态。
enum class SolveStatus {
  /// \brief 成功。
  kSuccess = 0,
  /// \brief 输入非法（非有限、维度不匹配、参数越界）。
  kInvalidInput,
  /// \brief QP 不可行。
  kInfeasible,
  /// \brief 超过允许的求解时间预算。
  kTimeout,
  /// \brief 数值失败（结果非有限或明显越界）。
  kNumericalFailure,
  /// \brief 未初始化。
  kNotInitialized,
};

/// \brief 结果状态的稳定字符串，用于日志与诊断。
inline const char * toString(const SolveStatus _status)
{
  switch (_status) {
    case SolveStatus::kSuccess:
      return "success";
    case SolveStatus::kInvalidInput:
      return "invalid_input";
    case SolveStatus::kInfeasible:
      return "infeasible";
    case SolveStatus::kTimeout:
      return "timeout";
    case SolveStatus::kNumericalFailure:
      return "numerical_failure";
    case SolveStatus::kNotInitialized:
      return "not_initialized";
  }
  return "unknown";
}

/// \brief 版本集合：一次规划请求/一条轨迹所属的地图、路径、限速与会话标识。
///
/// 任何字段变化都意味着旧轨迹与在途规划任务不可继续使用（方案 §4.4）。
struct VersionSet
{
  /// \brief 导航会话编号；新目标、取消后重发都会递增。
  std::uint64_t goal_epoch{0};
  /// \brief 全局路径版本；同目标 3 Hz 刷新且几何未变时不递增。
  std::uint64_t path_version{0};
  /// \brief 局部地图（占用）版本。
  std::uint64_t map_version{0};
  /// \brief 限速版本；Nav2 setSpeedLimit 变更时递增。
  std::uint64_t limits_version{0};

  bool operator==(const VersionSet & _other) const
  {
    return goal_epoch == _other.goal_epoch && path_version == _other.path_version &&
           map_version == _other.map_version && limits_version == _other.limits_version;
  }
  bool operator!=(const VersionSet & _other) const { return !(*this == _other); }
};

}  // namespace srm27_minco_core

#endif  // SRM27_MINCO_CORE__TYPES_HPP_
