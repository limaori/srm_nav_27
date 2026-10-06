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

#ifndef SRM27_MINCO_CORE__KINEMATICS_HPP_
#define SRM27_MINCO_CORE__KINEMATICS_HPP_

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <limits>

/// \file
/// \brief 二维平面运动学与角度的基础工具。
///
/// 约定（与 docs/迁移minco实施方案(ai).md §5.1 一致）：
///  * 惯性系固定为连续的 `odom`；MINCO 多项式与 MPC 预测模型都在 `odom` 中表达。
///  * `base_link` 是随底盘旋转的车体系，只用于最终速度输出与反馈输入。
///  * 位置单位 m，时间单位 s，角度单位 rad，平面角速度单位 rad/s。

namespace srm27_minco_core
{

/// \brief 为真表示数值可用于计算（非 NaN 且非无穷）。
inline bool isFinite(const double _value) { return std::isfinite(_value); }

/// \brief 为真表示向量的所有分量都有限。
inline bool isFinite(const Eigen::Vector2d & _value) { return _value.allFinite(); }

/// \brief 把角度归一化到 (-pi, pi]，只用于显示与误差计算，不用于连续航向状态。
inline double normalizeAngle(const double _angle)
{
  return std::atan2(std::sin(_angle), std::cos(_angle));
}

/// \brief 把 `_angle` 展开到与 `_reference` 最接近的 2*pi 分支上。
inline double unwrapAngleNear(const double _reference, const double _angle)
{
  return _reference + normalizeAngle(_angle - _reference);
}

/// \brief 连续航向的增量更新：保证 yaw 状态在多次积分之间不跨 2*pi 跳变。
inline double advanceUnwrappedYaw(
  const double _yaw_unwrapped, const double _omega, const double _dt)
{
  if (!isFinite(_yaw_unwrapped) || !isFinite(_omega) || !isFinite(_dt)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return _yaw_unwrapped + _omega * _dt;
}

/// \brief 二维旋转矩阵 R(yaw)：把车体系分量旋转到 odom 系。
inline Eigen::Matrix2d rotation2d(const double _yaw)
{
  const double c = std::cos(_yaw);
  const double s = std::sin(_yaw);
  Eigen::Matrix2d rotation;
  rotation << c, -s, s, c;
  return rotation;
}

/// \brief odom 系平移速度 -> base_link 系平移速度：v_b = R(yaw)^T * v_o。
inline Eigen::Vector2d odomToBodyVelocity(const double _yaw, const Eigen::Vector2d & _v_odom)
{
  const Eigen::Matrix2d rotation = rotation2d(_yaw);
  return rotation.transpose() * _v_odom;
}

/// \brief base_link 系平移速度 -> odom 系平移速度：v_o = R(yaw) * v_b。
inline Eigen::Vector2d bodyToOdomVelocity(const double _yaw, const Eigen::Vector2d & _v_body)
{
  return rotation2d(_yaw) * _v_body;
}

/// \brief 构造函数默认为零的二维向量。
inline Eigen::Vector2d zero2d() { return Eigen::Vector2d::Zero(); }

/// \brief 限制到闭区间；`_low > _high` 时返回下界，避免出现方向相反的空区间。
inline double clampToRange(const double _value, const double _low, const double _high)
{
  if (_low > _high) {
    return _low;
  }
  return _value < _low ? _low : (_value > _high ? _high : _value);
}

/// \brief 在给定制动减速度下，从当前速度减速到零所需的距离（m）。
/// \param _speed 当前平移速率（m/s），负值按 0 处理。
/// \param _decel 可用制动减速度（m/s^2），必须为正且有限。
/// \return 制动距离；参数非法时返回无穷，调用方应视为“无法停车”。
inline double stoppingDistance(const double _speed, const double _decel)
{
  if (!isFinite(_speed) || !isFinite(_decel) || _decel <= 0.0) {
    return std::numeric_limits<double>::infinity();
  }
  const double speed = std::max(0.0, _speed);
  return speed * speed / (2.0 * _decel);
}

}  // namespace srm27_minco_core

#endif  // SRM27_MINCO_CORE__KINEMATICS_HPP_
