// Copyright 2025 Lihan Chen
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

#ifndef FAKE_VEL_TRANSFORM__FAKE_VEL_TRANSFORM_HPP_
#define FAKE_VEL_TRANSFORM__FAKE_VEL_TRANSFORM_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "example_interfaces/msg/float32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "fake_vel_transform/velocity_transform.hpp"

namespace fake_vel_transform
{

/// \brief fakeyaw 速度转换节点（方案 §4.2）。
///
/// 职责被刻意收敛为两件事："发布 `base_link -> base_link_fake` 动态 TF" 与
/// "把 fake 系速度**一次性**转换到真实 `base_link` 系"。它不再叠加任何自转，
/// 也不再发布 `odom -> base_link`。
///
/// 两种模式（`transform_mode` 参数）：
///
///  * `legacy`（默认，保持既有实车 / 旧仿真链路行为不变）：
///    沿用「odom 与 local_plan 近似时间同步后再转换」的时序，以及 `cmd_spin` +
///    `init_spin_speed` 的**自转叠加**。此时 `wz` 的唯一来源就是 `cmd_spin` 链路，
///    导航 `wz` 在 mux 里被丢弃。
///  * `nav_se2`（本次 wz 控制迁移模式）：导航 / MINCO 拥有 `wz`，因此
///    ① 不再做任何自转叠加（收到非零 `cmd_spin` 只告警、只计数）；
///    ② 每条输入都用**最新有效 yaw** 转换一次（不做 local_plan 同步）；
///    ③ 里程计缺失 / 超时 / yaw 非有限时输出零速并计数，TF 也不再更新；
///    ④ 输入与输出队列都是 KEEP_LAST=1，每条输出都带诊断计数。
class FakeVelTransform : public rclcpp::Node
{
public:
  explicit FakeVelTransform(const rclcpp::NodeOptions & options);

private:
  /// \brief 运行模式。
  enum class Mode {
    /// \brief 既有链路：自转叠加 + odom/local_plan 同步转换。
    kLegacy = 0,
    /// \brief 迁移链路：导航拥有 wz，单次坐标转换。
    kNavSe2 = 1,
  };

  // ---- legacy 模式路径（与改造前行为一致）----
  void syncCallback(
    const nav_msgs::msg::Odometry::ConstSharedPtr & odom,
    const nav_msgs::msg::Path::ConstSharedPtr & local_plan);
  void localPlanCallback(const nav_msgs::msg::Path::ConstSharedPtr & msg);
  void publishLegacyTransform();

  // ---- 两种模式共用的输入回调 ----
  void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr & msg);
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void cmdSpinCallback(const example_interfaces::msg::Float32::SharedPtr msg);

  // ---- nav_se2 模式路径 ----
  /// \brief 用最新有效 yaw 做一次坐标转换并发布；无法转换时输出零速。
  void publishTransformedVelocity(const geometry_msgs::msg::Twist & _cmd);
  /// \brief 发布零速（无有效 yaw / 非有限输入 / 里程计超时）。
  void publishZeroVelocity();
  /// \brief 发布 `base_link -> base_link_fake`，时间戳取里程计消息时间。
  void publishFakeTf(const rclcpp::Time & _stamp);
  /// \brief 发布本节点的诊断计数。
  void publishDiagnostics();

  /// \brief 按模式解释 `wz`：legacy 叠加自转，nav_se2 原样透传。
  geometry_msgs::msg::Twist transformVelocity(
    const geometry_msgs::msg::Twist & twist, double yaw);

  Mode mode_{Mode::kLegacy};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<example_interfaces::msg::Float32>::SharedPtr cmd_spin_sub_;
  /// \brief nav_se2 模式的里程计订阅（普通订阅：KEEP_LAST=1，不做任何缓冲）。
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_filter_;
  message_filters::Subscriber<nav_msgs::msg::Path> local_plan_sub_filter_;
  using SyncPolicy =
    message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry, nav_msgs::msg::Path>;
  std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_chassis_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  /// \brief legacy 模式的 50 Hz TF 定时器（nav_se2 模式不使用）。
  rclcpp::TimerBase::SharedPtr timer_;
  /// \brief nav_se2 模式的诊断定时器。
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;

  std::string robot_base_frame_;
  std::string fake_robot_base_frame_;
  std::string odom_topic_;
  std::string local_plan_topic_;
  std::string cmd_spin_topic_;
  std::string input_cmd_vel_topic_;
  std::string output_cmd_vel_topic_;
  float spin_speed_;
  /// \brief 里程计允许的最大年龄（s）；超过即认为 yaw 不可用。
  double odom_timeout_{0.25};
  double diagnostics_period_{0.5};

  std::mutex cmd_vel_mutex_;
  geometry_msgs::msg::Twist::SharedPtr latest_cmd_vel_;
  double current_robot_base_angle_{0.0};
  rclcpp::Time last_controller_activate_time_;

  // nav_se2 状态：只有拿到**新鲜且有限**的 odom yaw 才允许转换速度。
  std::mutex state_mutex_;
  double latest_yaw_{0.0};
  /// \brief 是否收到过至少一条可用的里程计姿态。
  bool has_yaw_{false};
  /// \brief 最近一条里程计姿态是否可用（非有限四元数会立即置假）。
  bool yaw_usable_{false};
  double last_odom_steady_{0.0};
  double last_odom_age_{0.0};

  // 诊断计数
  uint64_t input_count_{0};
  uint64_t output_count_{0};
  uint64_t zero_output_count_{0};
  uint64_t no_odom_count_{0};
  uint64_t stale_odom_count_{0};
  uint64_t nonfinite_yaw_count_{0};
  uint64_t nonfinite_input_count_{0};
  uint64_t tf_publish_count_{0};
  uint64_t stale_tf_suppressed_count_{0};
  uint64_t non_zero_spin_count_{0};
  /// \brief `wz` 逐条透传误差统计（nav_se2 模式；理论上恒为 0）。
  double wz_error_max_{0.0};
  double wz_error_sum_{0.0};
  uint64_t wz_error_count_{0};
  double last_output_wz_{0.0};
};

}  // namespace fake_vel_transform

#endif  // FAKE_VEL_TRANSFORM__FAKE_VEL_TRANSFORM_HPP_
