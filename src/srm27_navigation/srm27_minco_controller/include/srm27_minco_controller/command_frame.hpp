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

#ifndef SRM27_MINCO_CONTROLLER__COMMAND_FRAME_HPP_
#define SRM27_MINCO_CONTROLLER__COMMAND_FRAME_HPP_

#include <Eigen/Core>
#include <string>

#include "srm27_minco_core/kinematics.hpp"

namespace srm27_minco_controller
{

/// \brief 速度命令的坐标系语义（方案 §3.2、§4.1.3）。
enum class CommandFrameMode {
  /// \brief 命令在真实车体系 `base_link`：输出前必须做一次 `R(-psi)`。
  kRobotBase = 0,
  /// \brief 命令在 `base_link_fake`：该系在 odom 平面内朝向稳定（与 odom 平行），
  /// 因此输出 `odom` 对齐的平移速度，**不再**提前乘一次 `R(-psi)` —— 转换由
  /// `fake_vel_transform` 唯一负责，否则就是"重复旋转"（方案 §5 的三类错误之一）。
  kOdomAlignedFake = 1,
};

/// \brief 由参数解析命令系语义。
///
/// \param _command_frame 配置的 `command_frame`。
/// \param _base_frame 真实车体系（`base_frame`）。
/// \param _fake_base_frame fakeyaw 迁移用的虚拟车体系（如 `base_link_fake`）。
/// \param _mode 输出：解析结果。
/// \param _reason 失败原因（仅在返回 false 时写入）。
/// \return 解析是否成功；既不等于车体系也不等于 fake 系时必须失败，
///         否则"到底转不转一次"会变成一个没人能查的隐含约定。
bool resolveCommandFrame(
  const std::string & _command_frame, const std::string & _base_frame,
  const std::string & _fake_base_frame, CommandFrameMode & _mode, std::string * _reason);

/// \brief 把 `odom` 系平移速度表达成命令系速度。
///
/// \param _velocity_odom odom 系平移速度（MPC 的直接输出，见 kinematic 约定）。
/// \param _yaw 命令生效时刻的真实底盘 yaw（rad）。
/// \param _mode 命令系语义。
inline Eigen::Vector2d velocityInCommandFrame(
  const Eigen::Vector2d & _velocity_odom, const double _yaw, const CommandFrameMode _mode)
{
  if (_mode == CommandFrameMode::kRobotBase) {
    return srm27_minco_core::odomToBodyVelocity(_yaw, _velocity_odom);
  }
  // fake 系与 odom 平行：平移分量直接沿用，交给 fake_vel_transform 做唯一一次转换。
  return _velocity_odom;
}

/// \brief 命令系语义的名字（诊断用；具体的 frame 名以 `command_frame` 参数为准）。
inline const char * toString(const CommandFrameMode _mode)
{
  return (_mode == CommandFrameMode::kRobotBase) ? "robot_base" : "odom_aligned_fake";
}

}  // namespace srm27_minco_controller

#endif  // SRM27_MINCO_CONTROLLER__COMMAND_FRAME_HPP_
