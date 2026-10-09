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

#include "srm27_nav_plugins/goal_checkers/omni_stopped_goal_checker.hpp"

#include <cmath>
#include <limits>

#include "nav2_util/node_utils.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace srm27_nav_plugins
{

void OmniStoppedGoalChecker::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, const std::string & plugin_name,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> /*costmap_ros*/)
{
  plugin_name_ = plugin_name;
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("OmniStoppedGoalChecker: parent node is not available");
  }
  logger_ = node->get_logger();

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".xy_goal_tolerance", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".yaw_goal_tolerance", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".trans_stopped_velocity", rclcpp::ParameterValue(0.25));
  // 故意**不声明** rot_stopped_velocity：本类的契约就是角速度不参与判定。
  // 参数文件里若残留这一项，它不会被声明、也就不会被读取；
  // "配置里不该出现它"这件事由 scripts/srm_minco_real_preflight.py 在 YAML 文本层拦
  // （在 ROS 参数语义里做这个检查是不可靠的：未声明的键不会成为参数）。
  (void)logger_;

  node->get_parameter(plugin_name + ".xy_goal_tolerance", xy_goal_tolerance_);
  node->get_parameter(plugin_name + ".yaw_goal_tolerance", yaw_goal_tolerance_);
  node->get_parameter(plugin_name + ".trans_stopped_velocity", trans_stopped_velocity_);

  if (!(xy_goal_tolerance_ >= 0.0) || !std::isfinite(xy_goal_tolerance_)) {
    throw std::runtime_error("OmniStoppedGoalChecker: xy_goal_tolerance must be non-negative");
  }
  if (!(yaw_goal_tolerance_ >= 0.0) || !std::isfinite(yaw_goal_tolerance_)) {
    throw std::runtime_error("OmniStoppedGoalChecker: yaw_goal_tolerance must be non-negative");
  }
  if (!(trans_stopped_velocity_ >= 0.0) || !std::isfinite(trans_stopped_velocity_)) {
    throw std::runtime_error("OmniStoppedGoalChecker: trans_stopped_velocity must be non-negative");
  }
}

void OmniStoppedGoalChecker::reset()
{
  // 本类不保存任何跨调用状态：每次判定都只用传进来的位姿与速度。
  // （Nav2 的 SimpleGoalChecker 有 stateful 缓存在目标位姿的语义，本类不需要，
  //   因为 controller_server 每个周期都会把当前 FollowPath 的末点一并传进来。）
}

bool OmniStoppedGoalChecker::isGoalReached(
  const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
  const geometry_msgs::msg::Twist & velocity)
{
  const double dx = goal_pose.position.x - query_pose.position.x;
  const double dy = goal_pose.position.y - query_pose.position.y;
  if (std::hypot(dx, dy) > xy_goal_tolerance_) {
    return false;
  }

  // 航向只作**位姿**比较（实车把 yaw_goal_tolerance 取到 6.28 = 不约束朝向，
  // 因为哨兵的机头朝向由自转链路/下位机决定）。
  const double d_yaw = tf2::getYaw(goal_pose.orientation) - tf2::getYaw(query_pose.orientation);
  const double normalized = std::atan2(std::sin(d_yaw), std::cos(d_yaw));
  if (std::abs(normalized) > yaw_goal_tolerance_) {
    return false;
  }

  // 只判**平动**是否停稳。角速度（velocity.angular.z）刻意不参与：
  // 导航不拥有自转，拿它否掉到达结论等于把下位机的状态当成导航的失败。
  return std::abs(velocity.linear.x) <= trans_stopped_velocity_ &&
         std::abs(velocity.linear.y) <= trans_stopped_velocity_;
}

bool OmniStoppedGoalChecker::getTolerances(
  geometry_msgs::msg::Pose & pose_tolerance, geometry_msgs::msg::Twist & vel_tolerance)
{
  // 未测量的分量按基类约定填 lowest()。
  pose_tolerance.position.x = xy_goal_tolerance_;
  pose_tolerance.position.y = xy_goal_tolerance_;
  pose_tolerance.position.z = std::numeric_limits<double>::lowest();
  pose_tolerance.orientation.x = std::numeric_limits<double>::lowest();
  pose_tolerance.orientation.y = std::numeric_limits<double>::lowest();
  pose_tolerance.orientation.z = std::numeric_limits<double>::lowest();
  pose_tolerance.orientation.w = std::numeric_limits<double>::lowest();

  vel_tolerance.linear.x = trans_stopped_velocity_;
  vel_tolerance.linear.y = trans_stopped_velocity_;
  vel_tolerance.linear.z = std::numeric_limits<double>::lowest();
  // 角速度容差明确标记为"未测量"：它不参与判定，所以没有一个有意义的数值可给。
  vel_tolerance.angular.x = std::numeric_limits<double>::lowest();
  vel_tolerance.angular.y = std::numeric_limits<double>::lowest();
  vel_tolerance.angular.z = std::numeric_limits<double>::lowest();

  return true;
}

}  // namespace srm27_nav_plugins

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(srm27_nav_plugins::OmniStoppedGoalChecker, nav2_core::GoalChecker)
