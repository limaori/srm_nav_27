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

#ifndef SRM27_NAV_PLUGINS__BT__REMOVE_PASSED_GOALS_BY_ROUTE_HPP_
#define SRM27_NAV_PLUGINS__BT__REMOVE_PASSED_GOALS_BY_ROUTE_HPP_

#include <string>
#include <utility>
#include <vector>

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace srm27_nav_plugins
{

/**
 * @brief 把"已经走过去的途径点"从目标列表里删掉 —— 替代 nav2 的 RemovePassedGoals。
 *
 * 为什么不用 nav2 那个: 它只看"车离这个点多近(radius)", 而且判定不落地 ——
 * 本仓库实车/仿真实测: 车贴着途径点 0.05~0.34 m 开过去, 那个点照样留在目标列表里,
 * 于是 3 Hz 重规划每次都生成一条"回头去它"的腿, 车在两个点之间来回绕。此外它还会把
 * **终点**一起删掉(所以 radius 必须小于 goal 容差, 否则车在判定到达前就掉头)。
 *
 * 本节点的判据:
 *   1) 半径: 车离该点 <= radius(默认 0.6 m) —— 车确实到过它旁边;
 *   2) "到过又走开": 车曾进到该点 approach_radius(默认 1.0 m) 以内(这个记忆只挂在
 *      **当前队首**上, 队首换人就清零), 现在离它 >= route_margin(默认 0.3 m) 并且
 *      **比下一个点更近** —— 即正朝路线后面走。这一条覆盖"路径被障碍膨胀推开、
 *      贴不到点上"的情况。两个条件必须同时满足: 自交路线(八字)里另一条分支上的点
 *      可能离车很近, 但车的距离不会"由近变远再比下一个点更近", 所以不会被误判。
 *
 * 两条硬规矩(2026-10-09 两次仿真踩坑后定下来的, 别再改回去):
 *   a) **只删列表头部的连续前缀**: 一旦某个点不满足判据就停止, 后面的点一个都不动。
 *      非队首的点只有在"车已经在它旁边(radius)"且前面都删掉时才被一起删。途经点在语义上
 *      只能顺序通过, "跳过中间某个点"一定是误判 —— 8 字任务里 #2 与 #5 物理上只差 0.44 m。
 *   b) **绝不做"整条路线取最近点"的全局投影**(第一版就是这么写的, 结果 8 字跑不了):
 *      8 字/回环路线里车起点离终点往往比离第一个点还近(实测 227_1006_waypoint1: 车离 #1
 *      1.09 m、离终点 0.86 m), 全局投影会落到路线末端, 于是"车已经走过整条路线"成立,
 *      6 个途径点在 3.6 s 内被删光、车 7 s 冲终点。基于"弧长进度"的折中方案也不行:
 *      车在 8 字中间抄近道时, 到 #3→#4 那一小段的垂距只有 0.98 m(小于任何合理的走廊阈值+),
 *      照样把没到过的 #3 判成"已走过"。所以现在只认"物理上靠近过"这一种证据。
 *
 * 最后一个点(终点)**永不删除**(keep_last=true): 到达判定交给 goal checker, 于是
 * "radius 必须小于 xy_goal_tolerance" 这个耦合约束也不需要了。
 */
class RemovePassedGoalsByRoute : public BT::ActionNodeBase
{
public:
  using Goals = std::vector<geometry_msgs::msg::PoseStamped>;

  RemovePassedGoalsByRoute(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

  void halt() override {}

private:
  /// 车在 global_frame 里的位置; 拿不到(没 TF)返回 false
  bool robotXY(const std::string & global_frame, const std::string & base_frame,
    double & x, double & y);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  bool warned_no_tf_{false};

  std::pair<long, long> head_key_{0, 0};   // 当前队首(1 cm 量化坐标)
  bool have_head_key_{false};
  bool approached_{false};                 // 车是否到过"当前队首"附近
};

}  // namespace srm27_nav_plugins

#endif  // SRM27_NAV_PLUGINS__BT__REMOVE_PASSED_GOALS_BY_ROUTE_HPP_
