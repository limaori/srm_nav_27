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

#include "srm27_nav_plugins/bt/remove_passed_goals_by_route.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <utility>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "tf2/exceptions.h"
#include "tf2/time.h"

namespace srm27_nav_plugins
{

namespace
{
// 坐标量化(1 cm)后做键: 用来判断"队首还是不是同一个点"
constexpr double kKeyQuantum = 0.01;

std::pair<long, long> keyOf(const geometry_msgs::msg::PoseStamped & pose)
{
  const auto & p = pose.pose.position;
  return {
    static_cast<long>(std::llround(p.x / kKeyQuantum)),
    static_cast<long>(std::llround(p.y / kKeyQuantum))};
}
}  // namespace

RemovePassedGoalsByRoute::RemovePassedGoalsByRoute(
  const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  // nav2 的 BT navigator 会把 tf_buffer 与 node 放进 blackboard
  try {
    tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  } catch (const std::exception &) {
    tf_ = nullptr;
  }
  try {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  } catch (const std::exception &) {
    node_ = nullptr;
  }
}

BT::PortsList RemovePassedGoalsByRoute::providedPorts()
{
  return {
    BT::InputPort<Goals>("input_goals", "原始目标列表(顺序即规划意图的路线)"),
    BT::OutputPort<Goals>("output_goals", "删掉已通过的途径点之后的目标列表"),
    BT::InputPort<double>("radius", 0.6, "车离该点小于该半径即判为已通过"),
    BT::InputPort<double>(
      "approach_radius", 1.0,
      "判据2用: 车曾进到该点这么近才算'到过它' (要明显大于实际贴点距离, 又远小于"
      "自交路线里'另一条分支'的距离)"),
    BT::InputPort<double>(
      "route_margin", 0.3,
      "判据2用: 车要离开该点这么远、并且比下一个点更近时才判为已通过"),
    BT::InputPort<bool>("keep_last", true, "永不删除最后一个点(终点)"),
    BT::InputPort<std::string>("global_frame", std::string("map"), "全局坐标系"),
    BT::InputPort<std::string>("robot_base_frame", std::string("base_link"), "机器人坐标系"),
  };
}

bool RemovePassedGoalsByRoute::robotXY(
  const std::string & global_frame, const std::string & base_frame, double & x, double & y)
{
  if (!tf_) {
    if (node_ && !warned_no_tf_) {
      warned_no_tf_ = true;
      RCLCPP_WARN(
        node_->get_logger(),
        "RemovePassedGoalsByRoute: blackboard 里没有 tf_buffer, 无法判断'已通过', 本次不删任何点");
    }
    return false;
  }
  try {
    const auto tf = tf_->lookupTransform(global_frame, base_frame, tf2::TimePointZero);
    x = tf.transform.translation.x;
    y = tf.transform.translation.y;
    return true;
  } catch (const tf2::TransformException & ex) {
    if (node_ && !warned_no_tf_) {
      warned_no_tf_ = true;
      RCLCPP_WARN(
        node_->get_logger(),
        "RemovePassedGoalsByRoute: 拿不到 %s -> %s 的 TF (%s), 本次不删任何点",
        global_frame.c_str(), base_frame.c_str(), ex.what());
    }
    return false;
  }
}

BT::NodeStatus RemovePassedGoalsByRoute::tick()
{
  Goals input_goals;
  const auto read_result = getInput("input_goals", input_goals);
  if (!read_result) {
    throw BT::RuntimeError(
            "RemovePassedGoalsByRoute: 读 input_goals 失败: ", read_result.error());
  }

  double radius = 0.6;
  double approach_radius = 1.0;
  double route_margin = 0.3;
  bool keep_last = true;
  std::string global_frame = "map";
  std::string robot_base_frame = "base_link";
  (void)getInput("radius", radius);
  (void)getInput("approach_radius", approach_radius);
  (void)getInput("route_margin", route_margin);
  (void)getInput("keep_last", keep_last);
  (void)getInput("global_frame", global_frame);
  (void)getInput("robot_base_frame", robot_base_frame);

  const size_t count = input_goals.size();
  if (count < 2u) {
    // 只剩一个点(通常是终点)时不做任何处理, 交给 goal checker 判到达
    (void)setOutput("output_goals", input_goals);
    approached_ = false;
    have_head_key_ = false;
    return BT::NodeStatus::SUCCESS;
  }

  // "到过队首"这个记忆只对**当前队首**有意义: 队首换人了就重新开始记
  const auto head_key = keyOf(input_goals.front());
  if (!have_head_key_ || head_key != head_key_) {
    approached_ = false;
    head_key_ = head_key;
    have_head_key_ = true;
  }

  double robot_x = 0.0;
  double robot_y = 0.0;
  const bool have_pose = robotXY(global_frame, robot_base_frame, robot_x, robot_y);

  auto distTo = [&](const geometry_msgs::msg::PoseStamped & pose) {
      return std::hypot(
        robot_x - pose.pose.position.x, robot_y - pose.pose.position.y);
    };

  double head_dist = std::numeric_limits<double>::infinity();
  double next_dist = std::numeric_limits<double>::infinity();
  if (have_pose) {
    head_dist = distTo(input_goals.front());
    next_dist = distTo(input_goals[1]);
    if (head_dist <= approach_radius) {
      approached_ = true;      // 车确实进到过队首附近
    }
  }

  // 队首判据:
  //   1) 半径: 车就在它旁边(默认 0.6 m) —— 直接证据;
  //   2) "到过又走开": 车曾进到 approach_radius(默认 1.0 m) 以内, 现在离它 >= route_margin
  //      且比**下一个点**更近(即正朝后面走) —— 覆盖"贴不到点上"(被膨胀推开的路径)的情况。
  //      两个条件缺一不可: 自交路线(八字)里"另一条分支上的点"虽然可能离车很近, 但车并不会
  //      在它和它的下一个点之间"由近变远", 因此不会被误判。
  bool drop_head = false;
  if (have_pose) {
    if (head_dist <= radius) {
      drop_head = true;
    } else if (approached_ && head_dist >= route_margin && next_dist < head_dist) {
      drop_head = true;
    }
  }

  // 只删列表头部的连续前缀:
  //   - 队首按上面两个判据判;
  //   - 后面的点只有在"车已经在它旁边(radius)"且前面的点都删掉时才会被一起删 ——
  //     途经点按语义只能顺序通过, 中途出现豁口一定是误判(八字交叉处 #2/#5 只隔 0.44 m,
  //     少了这条规则就会把另一瓣的点当成"已到过"而删掉)。
  Goals output;
  output.reserve(count);
  size_t dropped = 0;
  bool prefix_open = true;
  for (size_t i = 0; i < count; ++i) {
    if (!prefix_open || (keep_last && i + 1u == count)) {
      output.push_back(input_goals[i]);     // 终点永不删; 前缀断了之后一律保留
      continue;
    }
    bool drop = false;
    if (have_pose) {
      const double dist = distTo(input_goals[i]);
      drop = (i == 0u) ? drop_head : (dist <= radius);
    }
    if (drop) {
      ++dropped;
      continue;
    }
    prefix_open = false;
    output.push_back(input_goals[i]);
  }
  if (output.empty()) {
    output.push_back(input_goals.back());     // 兜底: 目标列表不允许为空
  }

  if (dropped > 0u && node_) {
    RCLCPP_INFO(
      node_->get_logger(),
      "RemovePassedGoalsByRoute: 删掉 %zu 个已通过的途径点, 剩 %zu 个 "
      "(队首距离 %.2f m, 下一个点 %.2f m, 到过队首=%s)",
      dropped, output.size(), head_dist, next_dist, approached_ ? "是" : "否");
  }

  const auto write_result = setOutput("output_goals", output);
  if (!write_result) {
    throw BT::RuntimeError(
            "RemovePassedGoalsByRoute: 写 output_goals 失败: ", write_result.error());
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace srm27_nav_plugins

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<srm27_nav_plugins::RemovePassedGoalsByRoute>(
    "RemovePassedGoalsByRoute");
}
