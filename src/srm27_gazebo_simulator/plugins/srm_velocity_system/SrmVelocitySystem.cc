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

#include "SrmVelocitySystem.hh"

#include <chrono>
#include <cmath>
#include <mutex>
#include <string>

#include <ignition/common/Console.hh>
#include <ignition/common/StringUtils.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/AngularVelocity.hh>
#include <ignition/gazebo/components/LinearVelocity.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/msgs/Utility.hh>
#include <ignition/msgs/odometry.pb.h>
#include <ignition/msgs/twist.pb.h>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <sdf/Element.hh>

namespace srm27
{
namespace gazebo
{
namespace systems
{
/// \brief SrmVelocitySystem 的私有实现。
class SrmVelocitySystemPrivate
{
  /// \brief 输入缓存与超时判定使用的最近一次命令时间。
  public: void OnCmdVel(const ignition::msgs::Twist &_msg);

  /// \brief 输出真值里程计。
  public: void UpdateOdometry(
      const ignition::gazebo::UpdateInfo &_info,
      const ignition::gazebo::EntityComponentManager &_ecm);

  /// \brief Gazebo Transport 节点。
  public: ignition::transport::Node node;

  /// \brief 真值里程计发布者。
  public: ignition::transport::Node::Publisher odomPub;

  /// \brief 底盘 link 实体。
  public: ignition::gazebo::Entity chassisLink{
    ignition::gazebo::kNullEntity};

  /// \brief 模型实体。
  public: ignition::gazebo::Entity modelEntity{
    ignition::gazebo::kNullEntity};

  /// \brief 输入缓存互斥锁。
  public: std::mutex targetVelMutex;

  /// \brief 最新的速度命令（车体系，x 前 y 左 z 上）。
  public: ignition::msgs::Twist targetVel;

  /// \brief 最近一次收到命令的仿真时间。
  public: std::chrono::steady_clock::duration lastCommandSimTime{0};

  /// \brief 是否已经收到过至少一条命令。
  public: bool commandReceived{false};

  /// \brief 命令超时（秒），<= 0 表示不启用超时清零。
  public: double commandTimeout{0.1};

  /// \brief 真值里程计发布频率（Hz）。
  public: double odomPublishFrequency{50.0};

  /// \brief 上一次发布里程计的仿真时间。
  public: std::chrono::steady_clock::duration lastOdomSimTime{0};

  /// \brief 是否已经发布过里程计。
  public: bool odomPublishInitialized{false};

  /// \brief 里程计 frame_id。
  public: std::string odomFrameId{"odom"};

  /// \brief 里程计 child_frame_id。
  public: std::string odomChildFrameId{"base_link"};

  /// \brief 是否已经打过一次超时清零日志，避免刷屏。
  public: bool timeoutLogged{false};
};

//////////////////////////////////////////////////
SrmVelocitySystem::SrmVelocitySystem()
  : dataPtr(std::make_unique<SrmVelocitySystemPrivate>())
{
}

//////////////////////////////////////////////////
SrmVelocitySystem::~SrmVelocitySystem() = default;

//////////////////////////////////////////////////
void SrmVelocitySystem::Configure(
    const ignition::gazebo::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    ignition::gazebo::EntityComponentManager &_ecm,
    ignition::gazebo::EventManager &/*_eventMgr*/)
{
  ignition::gazebo::Model model(_entity);
  if (!model.Valid(_ecm))
  {
    ignerr << "SrmVelocitySystem 需要挂在一个 model 上，插件被忽略。" << std::endl;
    return;
  }
  this->dataPtr->modelEntity = _entity;

  const auto modelName = model.Name(_ecm);
  auto chassisLinkName = std::string("base_link");
  if (_sdf && _sdf->HasElement("chassis_link"))
  {
    chassisLinkName = _sdf->Get<std::string>("chassis_link");
  }
  this->dataPtr->chassisLink = model.LinkByName(_ecm, chassisLinkName);
  if (this->dataPtr->chassisLink == ignition::gazebo::kNullEntity)
  {
    ignerr << "SrmVelocitySystem 找不到底盘 link [" << chassisLinkName
           << "]，插件被忽略。" << std::endl;
    return;
  }

  auto commandTopic = std::string("cmd_vel");
  if (_sdf && _sdf->HasElement("command_topic"))
  {
    commandTopic = _sdf->Get<std::string>("command_topic");
  }
  auto odomTopic = std::string("odometry");
  if (_sdf && _sdf->HasElement("odom_topic"))
  {
    odomTopic = _sdf->Get<std::string>("odom_topic");
  }
  if (_sdf && _sdf->HasElement("command_timeout"))
  {
    this->dataPtr->commandTimeout = _sdf->Get<double>("command_timeout");
  }
  if (_sdf && _sdf->HasElement("odom_publish_frequency"))
  {
    this->dataPtr->odomPublishFrequency =
        _sdf->Get<double>("odom_publish_frequency");
  }
  if (this->dataPtr->odomPublishFrequency <= 0.0)
  {
    this->dataPtr->odomPublishFrequency = 50.0;
  }

  const auto resolvedCommandTopic = modelName + "/" + commandTopic;
  const auto resolvedOdomTopic = modelName + "/" + odomTopic;

  // ROS/Transport 回调只更新输入缓存，速度在 PreUpdate 中统一施加。
  this->dataPtr->node.Subscribe(
      resolvedCommandTopic, &SrmVelocitySystemPrivate::OnCmdVel,
      this->dataPtr.get());
  this->dataPtr->odomPub =
      this->dataPtr->node.Advertise<ignition::msgs::Odometry>(
          resolvedOdomTopic);

  this->dataPtr->odomFrameId = modelName + "/odom";
  this->dataPtr->odomChildFrameId = modelName + "/" +
      ignition::common::replaceAll(chassisLinkName, "::", "/");

  ignmsg << "SrmVelocitySystem 已就绪: 底盘 [" << chassisLinkName
         << ", 车体系速度], 订阅 [" << resolvedCommandTopic << "], 发布 ["
         << resolvedOdomTopic << "], 命令超时 "
         << this->dataPtr->commandTimeout << " s" << std::endl;
}

//////////////////////////////////////////////////
void SrmVelocitySystemPrivate::OnCmdVel(const ignition::msgs::Twist &_msg)
{
  std::lock_guard<std::mutex> lock(this->targetVelMutex);
  this->targetVel = _msg;
}

//////////////////////////////////////////////////
void SrmVelocitySystem::PreUpdate(
    const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
  if (this->dataPtr->chassisLink == ignition::gazebo::kNullEntity)
  {
    return;
  }

  ignition::gazebo::Link chassisLink(this->dataPtr->chassisLink);
  // 与 MecanumDrive2 的速度分支保持一致：确保后续读取的组件存在。
  if (!_ecm.Component<ignition::gazebo::components::WorldPose>(
          this->dataPtr->chassisLink))
  {
    _ecm.CreateComponent(
        this->dataPtr->chassisLink, ignition::gazebo::components::WorldPose());
  }
  if (!_ecm.Component<ignition::gazebo::components::LinearVelocity>(
          this->dataPtr->chassisLink))
  {
    _ecm.CreateComponent(
        this->dataPtr->chassisLink,
        ignition::gazebo::components::LinearVelocity());
  }
  if (!_ecm.Component<ignition::gazebo::components::AngularVelocity>(
          this->dataPtr->chassisLink))
  {
    _ecm.CreateComponent(
        this->dataPtr->chassisLink,
        ignition::gazebo::components::AngularVelocity());
  }

  // 暂停时不推进，物理引擎也不会积分；直接清空缓存，恢复后必须重新收到命令
  // 才会运动，避免沿用暂停前的旧速度。
  if (_info.paused)
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->targetVelMutex);
    this->dataPtr->targetVel.clear_linear();
    this->dataPtr->targetVel.clear_angular();
    this->dataPtr->commandReceived = false;
    this->dataPtr->timeoutLogged = false;
    return;
  }

  ignition::msgs::Twist targetVel;
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->targetVelMutex);
    // 仿真重置会让 simTime 归零；此时旧命令与旧时间戳都失效。
    if (this->dataPtr->commandReceived &&
        _info.simTime < this->dataPtr->lastCommandSimTime)
    {
      this->dataPtr->targetVel.clear_linear();
      this->dataPtr->targetVel.clear_angular();
      this->dataPtr->commandReceived = false;
      this->dataPtr->timeoutLogged = false;
    }
    targetVel = this->dataPtr->targetVel;

    // 命令超时清零：输入中断后不允许底盘继续按旧速度运动。
    if (this->dataPtr->commandReceived && this->dataPtr->commandTimeout > 0.0)
    {
      const std::chrono::duration<double> sinceLastCommand =
          _info.simTime - this->dataPtr->lastCommandSimTime;
      if (sinceLastCommand.count() > this->dataPtr->commandTimeout)
      {
        targetVel.clear_linear();
        targetVel.clear_angular();
        this->dataPtr->targetVel.clear_linear();
        this->dataPtr->targetVel.clear_angular();
        this->dataPtr->commandReceived = false;
        if (!this->dataPtr->timeoutLogged)
        {
          this->dataPtr->timeoutLogged = true;
          ignwarn << "SrmVelocitySystem: 命令超时 "
                  << this->dataPtr->commandTimeout
                  << " s，已清零底盘速度。" << std::endl;
        }
      }
    }
  }

  const auto linearVel =
      _ecm.Component<ignition::gazebo::components::LinearVelocity>(
          this->dataPtr->chassisLink)->Data();
  const auto angularVel =
      _ecm.Component<ignition::gazebo::components::AngularVelocity>(
          this->dataPtr->chassisLink)->Data();

  // Gazebo 的 SetLinearVelocity / SetAngularVelocity 接受的就是 Link 坐标系
  // （车体系）速度，因此合成指令不需要再做世界系转换；转换只能做一次，
  // 这里刻意不做任何旋转。z 平移与 roll/pitch 角速度保留物理结果，
  // 不给正常平面运动额外施加持续旋转。
  const double vx = std::isfinite(targetVel.linear().x())
      ? targetVel.linear().x() : 0.0;
  const double vy = std::isfinite(targetVel.linear().y())
      ? targetVel.linear().y() : 0.0;
  const double wz = std::isfinite(targetVel.angular().z())
      ? targetVel.angular().z() : 0.0;

  chassisLink.SetLinearVelocity(
      _ecm, ignition::math::Vector3d(vx, vy, linearVel.Z()));
  chassisLink.SetAngularVelocity(
      _ecm, ignition::math::Vector3d(
          angularVel.X(), angularVel.Y(), wz));
}

//////////////////////////////////////////////////
void SrmVelocitySystem::PostUpdate(
    const ignition::gazebo::UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &_ecm)
{
  this->dataPtr->UpdateOdometry(_info, _ecm);
}

//////////////////////////////////////////////////
void SrmVelocitySystemPrivate::UpdateOdometry(
    const ignition::gazebo::UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &_ecm)
{
  if (this->chassisLink == ignition::gazebo::kNullEntity || _info.paused)
  {
    return;
  }
  const auto poseComp =
      _ecm.Component<ignition::gazebo::components::WorldPose>(
          this->chassisLink);
  const auto linearVelComp =
      _ecm.Component<ignition::gazebo::components::LinearVelocity>(
          this->chassisLink);
  const auto angularVelComp =
      _ecm.Component<ignition::gazebo::components::AngularVelocity>(
          this->chassisLink);
  if (!poseComp || !linearVelComp || !angularVelComp)
  {
    return;
  }

  const auto period = std::chrono::duration<double>(
      1.0 / this->odomPublishFrequency);
  if (this->odomPublishInitialized &&
      (_info.simTime - this->lastOdomSimTime) <
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(period))
  {
    return;
  }
  this->lastOdomSimTime = _info.simTime;
  this->odomPublishInitialized = true;

  const auto chassisPose = poseComp->Data();
  const auto linearVel = linearVelComp->Data();
  const auto angularVel = angularVelComp->Data();

  ignition::msgs::Odometry msg;
  msg.mutable_pose()->mutable_position()->set_x(chassisPose.X());
  msg.mutable_pose()->mutable_position()->set_y(chassisPose.Y());
  msg.mutable_pose()->mutable_position()->set_z(chassisPose.Z());
  // 真值位姿保留**完整三维姿态**（含 roll/pitch），不再压成只有 yaw：
  // 下游真值里程计适配器要用完整底盘位姿组合雷达外参、变换点云并喂给地形分析，
  // 只发 yaw 会让上坡时的雷达位姿失真（实测 pitch 约 -9°，车前方 1 m 的点高度
  // 误差约 0.2 m）。二维导航需要的 x/y/yaw 由状态适配层显式提取，不在这一层压平。
  ignition::msgs::Set(msg.mutable_pose()->mutable_orientation(), chassisPose.Rot());

  // Link 坐标系的真实执行速度，供"实际运动速度"与指令对照；角速度同样保留三维，
  // 便于事后判断爬坡时的俯仰/横滚角速度是真实存在还是链路误差。
  msg.mutable_twist()->mutable_linear()->set_x(linearVel.X());
  msg.mutable_twist()->mutable_linear()->set_y(linearVel.Y());
  msg.mutable_twist()->mutable_linear()->set_z(linearVel.Z());
  msg.mutable_twist()->mutable_angular()->set_x(angularVel.X());
  msg.mutable_twist()->mutable_angular()->set_y(angularVel.Y());
  msg.mutable_twist()->mutable_angular()->set_z(angularVel.Z());

  msg.mutable_header()->mutable_stamp()->CopyFrom(
      ignition::msgs::Convert(_info.simTime));
  auto frame = msg.mutable_header()->add_data();
  frame->set_key("frame_id");
  frame->add_value(this->odomFrameId);
  auto childFrame = msg.mutable_header()->add_data();
  childFrame->set_key("child_frame_id");
  childFrame->add_value(this->odomChildFrameId);

  this->odomPub.Publish(msg);
}
}  // namespace systems
}  // namespace gazebo
}  // namespace srm27

IGNITION_ADD_PLUGIN(
    srm27::gazebo::systems::SrmVelocitySystem,
    ignition::gazebo::System,
    srm27::gazebo::systems::SrmVelocitySystem::ISystemConfigure,
    srm27::gazebo::systems::SrmVelocitySystem::ISystemPreUpdate,
    srm27::gazebo::systems::SrmVelocitySystem::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(
    srm27::gazebo::systems::SrmVelocitySystem,
    "srm27::gazebo::systems::SrmVelocitySystem")
