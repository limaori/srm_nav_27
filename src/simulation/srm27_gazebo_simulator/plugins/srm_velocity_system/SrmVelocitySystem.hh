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

#ifndef SRM27_GAZEBO_SIMULATOR__SRM_VELOCITY_SYSTEM_HH_
#define SRM27_GAZEBO_SIMULATOR__SRM_VELOCITY_SYSTEM_HH_

#include <memory>

#include <ignition/gazebo/System.hh>

namespace srm27
{
namespace gazebo
{
namespace systems
{
/// \brief SRM 自有底盘速度执行插件。
///
/// 只做三件事：
///   1. 在 PreUpdate 中读取最新的 vx / vy / wz 输入缓存，按 Gazebo API 的
///      参考系约定（Link 坐标系，即车体系）设置底盘平面速度；
///   2. 命令超时、仿真暂停或仿真重置时把速度清零；
///   3. 在 PostUpdate 中按固定频率输出真值里程计，供调试与对照使用。
///
/// 不做轮速 PID、力矩控制、底盘跟随云台或额外旋转策略；旧步兵的
/// MecanumDrive2 与云台/装甲/射击组件都不参与这条链路。
///
/// Gazebo Transport 订阅话题为 <model_name>/cmd_vel，真值里程计发布话题为
/// <model_name>/odometry，与 ros_gz_bridge 的既有映射保持一致。
class SrmVelocitySystem
  : public ignition::gazebo::System,
    public ignition::gazebo::ISystemConfigure,
    public ignition::gazebo::ISystemPreUpdate,
    public ignition::gazebo::ISystemPostUpdate
{
  /// \brief 构造函数。
  public: SrmVelocitySystem();

  /// \brief 析构函数。
  public: ~SrmVelocitySystem() override;

  // Documentation inherited
  public: void Configure(
      const ignition::gazebo::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      ignition::gazebo::EntityComponentManager &_ecm,
      ignition::gazebo::EventManager &_eventMgr) override;

  // Documentation inherited
  public: void PreUpdate(
      const ignition::gazebo::UpdateInfo &_info,
      ignition::gazebo::EntityComponentManager &_ecm) override;

  // Documentation inherited
  public: void PostUpdate(
      const ignition::gazebo::UpdateInfo &_info,
      const ignition::gazebo::EntityComponentManager &_ecm) override;

  /// \brief 私有实现。
  private: std::unique_ptr<class SrmVelocitySystemPrivate> dataPtr;
};
}  // namespace systems
}  // namespace gazebo
}  // namespace srm27

#endif  // SRM27_GAZEBO_SIMULATOR__SRM_VELOCITY_SYSTEM_HH_
