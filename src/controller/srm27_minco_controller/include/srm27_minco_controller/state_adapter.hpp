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

#ifndef SRM27_MINCO_CONTROLLER__STATE_ADAPTER_HPP_
#define SRM27_MINCO_CONTROLLER__STATE_ADAPTER_HPP_

#include <tf2_ros/buffer.h>

#include <Eigen/Core>
#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <string>

#include "srm27_minco_core/types.hpp"

namespace srm27_minco_controller
{

using srm27_minco_core::State2D;

/// \brief 状态适配层：把 odometry / TF 转成 `odom` 系下的 `State2D`。
///
/// 语义约定（方案 §2.3、§5.1、§5.2）：
///  * `Odometry.pose` 在 `header.frame_id`（odom）下，`twist` 在 `child_frame_id`
///    （base_link）下。这里把 twist 显式旋转到 odom 系，不做“父系分量直接当车体系”的
///    隐式假设。
///  * 位姿差分使用**消息采样时间差**；本地单调时钟只用于超时和耗时统计，仿真倍速、
///    暂停、回放时两者不能混用。
///  * 重复时间戳、时间倒退、过大间隔与异常位姿跳变都不生成速度尖峰，而是标记为无效。
///  * TF 查询失败**不会**退化成单位变换伪装成有效定位；调用方必须看到明确的失效状态。
class StateAdapter
{
public:
  /// \brief 配置。
  struct Config
  {
    /// \brief 里程计话题（节点命名空间下的相对名称）。
    std::string odometry_topic{"odometry"};
    /// \brief 底盘坐标系。
    std::string base_frame{"base_link"};
    /// \brief 规划坐标系；与里程计 header.frame_id 不同时用 TF 转换。
    std::string planning_frame{"odom"};
    /// \brief 状态允许的最大年龄（s），超过则视为失效。
    double timeout{0.10};
    /// \brief 差分速度允许的最大采样间隔（s）；过大间隔不用于求速度。
    double max_sample_gap{0.20};
    /// \brief 单次采样允许的最大平移跳变（m），超过视为定位重置。
    double max_position_jump{0.75};
    /// \brief 单次采样允许的最大航向跳变（rad）。
    double max_yaw_jump{1.20};
    /// \brief twist 是否表达在 child_frame 下（标准 nav_msgs 语义为真）。
    bool twist_in_child_frame{true};
    /// \brief 是否用 TF 校验/转换位姿。
    bool use_tf{true};
    /// \brief 位置差分速度的指数平滑系数（0 表示不平滑）。
    double velocity_filter_alpha{0.35};
  };

  StateAdapter() = default;

  /// \brief 配置并建立订阅。重复调用会先释放旧订阅。
  bool configure(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & _node, std::shared_ptr<tf2_ros::Buffer> _tf,
    const Config & _config, std::string * _reason = nullptr);

  /// \brief 释放订阅（cleanup 时调用）。
  void reset();

  /// \brief 取得当前可用于控制的状态。
  /// \param _now_stamp 当前 ROS 时间（秒），用于外推与超时。
  /// \param _now_steady 当前单调时间（秒），仅用于超时统计。
  /// \param _state 输出状态（`odom` 系）。
  /// \param _reason 失效原因（可为 nullptr）。
  /// \return 状态有效时返回 true。
  bool stateAt(double _now_stamp, double _now_steady, State2D & _state, std::string * _reason);

  /// \brief 里程计是否在超时范围内。
  bool fresh(double _now_stamp, double _now_steady, std::string * _reason = nullptr) const;

  /// \brief 里程计“重置/跳变”计数；控制器在计数变化时必须作废旧轨迹与热启动。
  std::uint64_t resetCount() const { return reset_count_.load(); }

  /// \brief 被丢弃的乱序/重复时间戳采样数（诊断用）。
  std::uint64_t droppedSampleCount() const { return dropped_sample_count_.load(); }

  /// \brief 最近一次收到里程计的时间（ROS 秒）。
  double lastSampleStamp() const;

  /// \brief 最近一次收到里程计的时刻（单调秒）。
  double lastReceivedSteady() const;

private:
  /// \brief 里程计回调。
  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr _message);

  Config config_{};
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_{};
  std::shared_ptr<tf2_ros::Buffer> tf_{};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_{};

  mutable std::mutex mutex_{};
  nav_msgs::msg::Odometry latest_{};
  bool has_latest_{false};
  double latest_received_steady_{0.0};
  nav_msgs::msg::Odometry previous_{};
  bool has_previous_{false};
  std::atomic<std::uint64_t> reset_count_{0};
  std::atomic<std::uint64_t> dropped_sample_count_{0};
  bool configured_{false};
  /// \brief 平滑后的 odom 系平移速度（m/s）。
  Eigen::Vector2d filtered_velocity_{Eigen::Vector2d::Zero()};
  bool has_filtered_velocity_{false};
};

}  // namespace srm27_minco_controller

#endif  // SRM27_MINCO_CONTROLLER__STATE_ADAPTER_HPP_
