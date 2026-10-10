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

#ifndef SRM27_NAV_PLUGINS__GOAL_CHECKERS__OMNI_STOPPED_GOAL_CHECKER_HPP_
#define SRM27_NAV_PLUGINS__GOAL_CHECKERS__OMNI_STOPPED_GOAL_CHECKER_HPP_

#include <memory>
#include <string>

#include "nav2_core/goal_checker.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace srm27_nav_plugins
{

/// \brief 全向底盘用的"停稳到点"判定：**角速度不参与**。
///
/// 与 `nav2_controller::StoppedGoalChecker` 的唯一差别：
/// 后者在位置/航向都满足之后还要求
/// `|vx| <= trans_stopped_velocity && |vy| <= trans_stopped_velocity &&
///  |wz| <= rot_stopped_velocity`，
/// 也就是**自转会让它永远判不了到点**；本类把 `wz` 那一项去掉。
///
/// 为什么 SRM 需要这样：
///  * 导航链路不拥有自转。阶段一 `yaw_policy.mode = xy_only`（插件输出 `wz ≡ 0`），
///    `velocity_smoother` 也把 yaw 上界钳成 0；哨兵的自转由下位机/独立链路负责。
///    既然导航不产生也不控制 `wz`，把它放进"是否到达"的判据里就是拿别人的状态
///    否掉自己的结论。
///  * `yaw_goal_tolerance` 本来就是自由的（实车 6.28 rad ≥ π），也就是说
///    "朝向不算到达条件"这条已经写在配置里了，`wz` 再算条件在语义上自相矛盾。
///
/// 判定条件（全部满足才算到达）：
///  1. 平面距离 `hypot(dx, dy) <= xy_goal_tolerance`；
///  2. 航向差 `|Δyaw| <= yaw_goal_tolerance`（实车取 6.28，等于不约束朝向）；
///  3. 平动速度 `hypot(vx, vy) <= trans_stopped_velocity`
///     —— 这一条保留：它保证"到点"时车是真的停住了，而不是高速掠过目标点。
///
/// `Angular velocity` 完全不出现在判定里。
class OmniStoppedGoalChecker : public nav2_core::GoalChecker
{
public:
  OmniStoppedGoalChecker() = default;
  ~OmniStoppedGoalChecker() override = default;

  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, const std::string & plugin_name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void reset() override;

  bool isGoalReached(
    const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
    const geometry_msgs::msg::Twist & velocity) override;

  bool getTolerances(
    geometry_msgs::msg::Pose & pose_tolerance, geometry_msgs::msg::Twist & vel_tolerance) override;

private:
  double xy_goal_tolerance_{0.25};
  double yaw_goal_tolerance_{0.25};
  /// \brief 平动停稳阈值（m/s）；`vx`/`vy` 任一超过它就不算到达。
  double trans_stopped_velocity_{0.25};
  std::string plugin_name_{};
  rclcpp::Logger logger_{rclcpp::get_logger("OmniStoppedGoalChecker")};
};

}  // namespace srm27_nav_plugins

#endif  // SRM27_NAV_PLUGINS__GOAL_CHECKERS__OMNI_STOPPED_GOAL_CHECKER_HPP_
