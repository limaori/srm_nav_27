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

#ifndef FAKE_VEL_TRANSFORM__VELOCITY_TRANSFORM_HPP_
#define FAKE_VEL_TRANSFORM__VELOCITY_TRANSFORM_HPP_

#include <cmath>
#include <string>

/// \file
/// \brief fakeyaw 链路的**纯数学**部分（不依赖 ROS，便于单元测试）。
///
/// 约定（见 docs/wz控制迁移导航与fakeyaw仿真实施方案.md §3.1）：
///
///  * `base_link` 是真实随底盘自转的车体系，它在 `odom` 里的 yaw 记为 `psi`；
///  * `base_link_fake` 与 `base_link` 原点重合、只有 yaw 不同，且**在 odom 平面内朝向稳定**，
///    即 `q(base_link -> base_link_fake) = yaw(-psi)`，等价于 fake 系与 odom 系平行；
///  * Nav2 / MINCO 在 fake 系（= odom 对齐）里输出速度，本模块把它一次性转到真实车体系：
///
/// \code
/// v_base.x =  cos(psi) * v_fake.x + sin(psi) * v_fake.y
/// v_base.y = -sin(psi) * v_fake.x + cos(psi) * v_fake.y
/// wz_base  = wz_fake          // 纯 yaw 旋转下绕 z 的角速度数值不变
/// \endcode
///
/// `wz` 必须逐条透传：任何"再叠加一次自转"的写法都会造成双重旋转。

namespace fake_vel_transform
{

/// \brief 平面速度（车体系 x 前、y 左；wz 绕 z，rad/s）。
struct PlanarVelocity
{
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
};

/// \brief 只绕 z 的四元数分量。
struct QuaternionZ
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double w{1.0};
};

/// \brief `base_link -> base_link_fake` 的 yaw：真实底盘 yaw 的逆。
inline double fakeFrameYaw(const double _yaw) { return -_yaw; }

/// \brief fake 系（与 odom 对齐）速度 -> 真实车体系速度；`wz` 原样透传。
///
/// \param _v_fake fake 系速度（等价于 odom 系速度）。
/// \param _yaw 真实底盘在 odom 系中的 yaw（rad）。
inline PlanarVelocity fakeToRobotBase(const PlanarVelocity & _v_fake, const double _yaw)
{
  const double c = std::cos(_yaw);
  const double s = std::sin(_yaw);
  PlanarVelocity v_base;
  v_base.vx = c * _v_fake.vx + s * _v_fake.vy;
  v_base.vy = -s * _v_fake.vx + c * _v_fake.vy;
  // 绕 z 的角速度在这次纯 yaw 坐标旋转下数值不变：绝不能再叠加 cmd_spin（方案 §3.1）。
  v_base.wz = _v_fake.wz;
  return v_base;
}

/// \brief 由 yaw 生成只绕 z 的四元数。
inline QuaternionZ quaternionFromYaw(const double _yaw)
{
  QuaternionZ q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(0.5 * _yaw);
  q.w = std::cos(0.5 * _yaw);
  return q;
}

/// \brief 姿态四元数是否可用于提取 yaw。
///
/// 全零 / 含 NaN 的四元数在把持久的里程计噪声直接喂给 `tf2::getYaw` 时会静默产出
/// 无意义的航向（甚至全链路按错误 yaw 变换速度），因此调用方必须先过这一关。
inline bool isUsableOrientation(
  const double _x, const double _y, const double _z, const double _w)
{
  if (
    !std::isfinite(_x) || !std::isfinite(_y) || !std::isfinite(_z) || !std::isfinite(_w)) {
    return false;
  }
  const double norm = std::sqrt(_x * _x + _y * _y + _z * _z + _w * _w);
  return std::isfinite(norm) && norm > 1.0e-6;
}

/// \brief 把速度按 `base_link_fake` 语义取回（用于诊断：转换前后的模长与 wz 误差）。
inline double planarSpeed(const PlanarVelocity & _v) { return std::hypot(_v.vx, _v.vy); }

}  // namespace fake_vel_transform

#endif  // FAKE_VEL_TRANSFORM__VELOCITY_TRANSFORM_HPP_
