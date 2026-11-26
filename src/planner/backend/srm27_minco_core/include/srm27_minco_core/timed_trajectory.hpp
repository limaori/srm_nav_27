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

#ifndef SRM27_MINCO_CORE__TIMED_TRAJECTORY_HPP_
#define SRM27_MINCO_CORE__TIMED_TRAJECTORY_HPP_

#include <Eigen/Core>

namespace srm27_minco_core
{

/// \brief 只读“时间参数化轨迹”接口：给定时刻，查位置/速度/加速度/切向。
///
/// 为什么需要这层抽象（2026-11-26 MINCO 迁到规划器时引入）：
///  * MINCO 在控制器内部时，控制器直接持有 `Trajectory2D`（多项式系数）；
///  * 迁到规划器后，控制器只能通过话题收到**采样点**（`MincoTrajectory`），
///    无法恢复多项式系数，也不需要恢复；
///  * 跟踪参考、可视化、诊断只用到"按时间取状态"这一组操作，抽出来之后
///    MPC 跟踪逻辑对两种来源完全一致 —— 这也是能做 internal/topic A/B 的前提。
///
/// 单位：位置 m、速度 m/s、加速度 m/s²、时间 s（相对轨迹起点，0..totalDuration）。
/// 坐标系：与调用方一致（控制器里是 odom）。
class TimedTrajectory
{
public:
  virtual ~TimedTrajectory() = default;

  /// \brief 是否为空轨迹（没有任何段）。
  virtual bool empty() const = 0;

  /// \brief 总时长（s），必须 >= 0。
  virtual double totalDuration() const = 0;

  /// \brief 时刻 `_t` 的位置；`_t` 超出范围时按端点处理。
  virtual Eigen::Vector2d positionAt(double _t) const = 0;

  /// \brief 时刻 `_t` 的速度。
  virtual Eigen::Vector2d velocityAt(double _t) const = 0;

  /// \brief 时刻 `_t` 的加速度（不可得时返回零向量）。
  virtual Eigen::Vector2d accelerationAt(double _t) const = 0;

  /// \brief 时刻 `_t` 的单位切向；退化时返回 false。
  virtual bool tangentAt(double _t, Eigen::Vector2d & _tangent) const = 0;

  /// \brief 末端位置。
  virtual Eigen::Vector2d endPosition() const = 0;

  /// \brief 末端是否就是全局目标。
  virtual bool isTerminalGlobalGoal() const = 0;

  /// \brief 末端是否要求停稳（速度为零）。
  virtual bool terminalRequiresStop() const = 0;
};

}  // namespace srm27_minco_core

#endif  // SRM27_MINCO_CORE__TIMED_TRAJECTORY_HPP_
