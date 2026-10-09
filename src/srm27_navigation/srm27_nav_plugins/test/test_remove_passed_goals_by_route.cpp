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

#include <gtest/gtest.h>

#include <fstream>
#include <regex>
#include <iterator>
#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_msgs/msg/tf_message.hpp"

#include "srm27_nav_plugins/bt/remove_passed_goals_by_route.hpp"

using Pose = geometry_msgs::msg::PoseStamped;
using Goals = std::vector<Pose>;
using XY = std::vector<std::pair<double, double>>;

namespace
{

// 端口值必须与 behavior_trees/navigate_through_poses_route_aware.xml 保持一致
const char * kTreeXml = R"(
<root BTCPP_format="3">
  <BehaviorTree ID="MainTree">
    <RemovePassedGoalsByRoute input_goals="{goals}" output_goals="{goals_out}"
                              radius="0.60" approach_radius="1.0"
                              route_margin="0.30" keep_last="true"/>
  </BehaviorTree>
</root>
)";

// 任务用的 7 个航点(与 missions/227_1006_waypoint1.yaml 一致) —— 一个"八字":
// #1 -> #2 -> #3 -> #4 -> #5 -> #6 -> #7, 其中 #2 与 #5 在交叉处只差 0.44 m,
// 起点(#1)与终点(#7)也几乎贴在一起(车停在原点时: 离 #1 1.57 m、离 #7 0.77 m)。
// 这组坐标专门用来钉死"不许按路线弧长投影删点"这条规矩。
const XY kWaypoints = {
  {1.521, -0.401}, {1.752, -1.967}, {3.123, -1.643}, {2.457, -0.522},
  {1.478, -1.617}, {0.023, -1.555}, {-0.358, -0.678}};

// 判据本身的测试用一条简单直线路线(间距 1.2 m), 距离好算、边界清楚
const XY kLineRoute = {{2.0, 0.0}, {3.2, 0.0}, {4.4, 0.0}};

Pose makePose(double x, double y)
{
  Pose p;
  p.header.frame_id = "map";
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.orientation.w = 1.0;
  return p;
}

Goals makeGoals(const XY & pts, size_t from = 0, size_t to = SIZE_MAX)
{
  Goals goals;
  const size_t last = std::min(to, pts.size());
  for (size_t i = from; i < last; ++i) {
    goals.push_back(makePose(pts[i].first, pts[i].second));
  }
  return goals;
}

/// 测试夹具: 手工塞一个 map->base_link 的 TF, 不需要跑导航栈
class RemovePassedGoalsByRouteTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("test_remove_passed_goals");
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());

    blackboard_ = BT::Blackboard::create();
    blackboard_->set("tf_buffer", tf_buffer_);
    blackboard_->set("node", node_);

    factory_.registerNodeType<srm27_nav_plugins::RemovePassedGoalsByRoute>(
      "RemovePassedGoalsByRoute");
    tree_ = factory_.createTreeFromText(kTreeXml, blackboard_);
  }

  void setRobot(double x, double y)
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = node_->now();
    tf.header.frame_id = "map";
    tf.child_frame_id = "base_link";
    tf.transform.translation.x = x;
    tf.transform.translation.y = y;
    tf.transform.rotation.w = 1.0;
    tf_buffer_->setTransform(tf, "test");
  }

  std::vector<std::pair<double, double>> tick(const Goals & goals)
  {
    blackboard_->set("goals", goals);
    tree_.tickRoot();
    Goals out;
    blackboard_->get("goals_out", out);
    std::vector<std::pair<double, double>> result;
    for (const auto & g : out) {
      result.emplace_back(g.pose.position.x, g.pose.position.y);
    }
    return result;
  }

  /// 断言输出列表与某条路线的 [from, to) 段一模一样(顺序 + 坐标)
  void expectSame(const std::vector<std::pair<double, double>> & out, const XY & route,
    size_t from, size_t to)
  {
    ASSERT_EQ(out.size(), to - from);
    for (size_t i = from; i < to; ++i) {
      EXPECT_NEAR(out[i - from].first, route[i].first, 1e-6) << "第 " << i << " 个点";
      EXPECT_NEAR(out[i - from].second, route[i].second, 1e-6) << "第 " << i << " 个点";
    }
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  BT::Blackboard::Ptr blackboard_;
  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
};

// ============================================================================
// 判据 1 / 2 本身(直线路线 (2,0)-(3.2,0)-(4.4,0))
// ============================================================================

// 车站在 (2.8, 0.512): 离队首 A(2,0) 0.95 m(半径 0.6 够不着, 但进了 approach_radius 1.0),
// 离下一个点 B(3.2,0) 0.65 m 更近 → "到过 A 又走开" → 删 A; B/C 保留
TEST_F(RemovePassedGoalsByRouteTest, DropsHeadAfterApproachingAndMovingOn)
{
  setRobot(2.8, 0.512);
  const auto out = tick(makeGoals(kLineRoute));        // [A, B, C]
  ASSERT_EQ(out.size(), 2u);
  EXPECT_NEAR(out[0].first, kLineRoute[1].first, 1e-6);   // B
  EXPECT_NEAR(out[1].first, kLineRoute[2].first, 1e-6);   // C
}

// 车站在 (2.3, 0.6): 离 A 0.67 m(到过, 也 > radius), 但离 B(3.2,0) 1.08 m 更远
// → 不是"走开了" → 一个点都不删
TEST_F(RemovePassedGoalsByRouteTest, KeepsHeadWhenNotMovingOn)
{
  setRobot(2.3, 0.6);
  const auto out = tick(makeGoals(kLineRoute));
  expectSame(out, kLineRoute, 0, 3);
}

// 车站在 (2.8, 1.2): 虽然离 B(1.26 m) 比离 A(1.44 m) 近, 但从没进到 A 的 1.0 m 以内
// → 判据 2 的两个条件缺一不可 → 不删
TEST_F(RemovePassedGoalsByRouteTest, KeepsHeadWhenNeverApproached)
{
  setRobot(2.8, 1.2);
  const auto out = tick(makeGoals(kLineRoute));
  expectSame(out, kLineRoute, 0, 3);
}

// 车贴着队首(0.2 m) → 半径判据命中
TEST_F(RemovePassedGoalsByRouteTest, DropsHeadWithinRadius)
{
  setRobot(2.0, 0.2);
  const auto out = tick(makeGoals(kLineRoute));
  ASSERT_EQ(out.size(), 2u);
  EXPECT_NEAR(out[0].first, kLineRoute[1].first, 1e-6);   // B
}

// 只能删连续前缀: 车就站在终点 C 上, 但队首 A 没到过 → A/B/C 一个都不许删
TEST_F(RemovePassedGoalsByRouteTest, NeverSkipsInteriorPoints)
{
  setRobot(kLineRoute[2].first, kLineRoute[2].second);
  const auto out = tick(makeGoals(kLineRoute));
  expectSame(out, kLineRoute, 0, 3);
}

// 没有 TF 时不能乱删(宁可留着, 也不能把没开过的点删掉)
TEST_F(RemovePassedGoalsByRouteTest, KeepsEverythingWithoutTf)
{
  const auto out = tick(makeGoals(kLineRoute));   // 从没 setRobot
  EXPECT_EQ(out.size(), 3u);
}

// 队首换人后"到过"的记忆必须清零: 先在 A 附近(到过 A), 再把车挪到 C 附近, 此时列表只剩
// [B, C] —— 车并没到过 B, 所以不能因为它离 C 近就把 B/C 删掉
TEST_F(RemovePassedGoalsByRouteTest, ApproachMemoryIsPerHead)
{
  setRobot(2.0, 0.2);                                   // 到过 A(半径内)
  (void)tick(makeGoals(kLineRoute));
  setRobot(4.2, 0.1);                                   // 车挪到 C 附近
  const auto out = tick(makeGoals(kLineRoute, 1, 3));   // 新列表 [B, C]
  expectSame(out, kLineRoute, 1, 3);
}

// ============================================================================
// 回归: 2026-10-09 仿真里"八字跑不了 / 第一瓣被切掉"的真实场景
// ============================================================================

// 场景 1(第一次跑挂在这里): 车停在任务起点 (0,0) —— 离 #1 有 1.57 m, 离**终点 #7** 只有
// 0.77 m(八字的头和尾几乎贴在一起)。旧实现做"整条路线取最近点"的全局投影, 投影落到路线
// 末端(弧长 8.18 m = 全长), "车已经走过整条路线"成立 ⇒ 6 个途径点在 3.6 s 内被删光、
// 车 7 s 冲终点。现在必须一个点都不删。
TEST_F(RemovePassedGoalsByRouteTest, FigureEightStartKeepsAllGoals)
{
  setRobot(0.0, 0.0);
  const auto out = tick(makeGoals(kWaypoints));
  expectSame(out, kWaypoints, 0, 7);
}

// 场景 1b: 车停在 (0.43,-0.33) —— 实测那次任务的起点(离 #7 0.86 m、离 #1 1.09 m),
// 旧实现第一拍就把 #6(离车 1.29 m, 根本没到过)删掉了
TEST_F(RemovePassedGoalsByRouteTest, FigureEightNearStartKeepsAllGoals)
{
  setRobot(0.43, -0.33);
  const auto out = tick(makeGoals(kWaypoints));
  expectSame(out, kWaypoints, 0, 7);
}

// 场景 2: 车开到 #2 附近时, 与 #2 只差 0.44 m 的 #5(另一条分支)绝不能被当成"已通过"
// 删掉 —— 那样八字的第一瓣就被切掉了; 此刻 #1 也没到过(离 1.58 m), 同样要保留
TEST_F(RemovePassedGoalsByRouteTest, FigureEightCrossingKeepsLaterBranch)
{
  setRobot(kWaypoints[1].first, kWaypoints[1].second);   // 车正好在 #2 上
  const auto out = tick(makeGoals(kWaypoints));
  expectSame(out, kWaypoints, 0, 7);
}

// 场景 3(第二次跑挂在这里): 车抄近道从八字中间穿过去, 此时队首是 #3, 车离 #3 有 1.24 m
// (没到过), 而离路线 #3→#4 那一段的垂距只有 0.98 m。旧的"按弧长投影"版本因此把 #3 判成
// 已走过 ⇒ #3/#4 被删掉、车直线冲向终点, 第一瓣又没了。
// 现在: 队首离车 1.24 m > approach_radius(1.0) ⇒ 从没"到过" ⇒ 一个点都不许删。
TEST_F(RemovePassedGoalsByRouteTest, DoesNotDropHeadJustBecausePathIsNearTheRoute)
{
  setRobot(1.90, -1.50);
  const auto out = tick(makeGoals(kWaypoints, 2, 7));    // [#3,#4,#5,#6,#7]
  expectSame(out, kWaypoints, 2, 7);
}

// 场景 4: 任务跑到一半, 车在交叉处 #5 上, 列表头是 #3(没到过) → #3/#4 必须留着
TEST_F(RemovePassedGoalsByRouteTest, MidMissionCrossingKeepsUnvisitedHead)
{
  setRobot(kWaypoints[4].first, kWaypoints[4].second);   // 车在 #5 上
  const auto out = tick(makeGoals(kWaypoints, 2, 7));    // [#3,#4,#5,#6,#7]
  expectSame(out, kWaypoints, 2, 7);
}

// 终点永不删: 就算车站在终点上
TEST_F(RemovePassedGoalsByRouteTest, NeverDropsLastGoal)
{
  setRobot(kWaypoints[6].first, kWaypoints[6].second);
  const auto out = tick(makeGoals(kWaypoints));
  ASSERT_FALSE(out.empty());
  EXPECT_NEAR(out.back().first, kWaypoints[6].first, 1e-6);
  EXPECT_EQ(out.back().second, kWaypoints[6].second);
}

// 不存"谁已经通过"的持久记忆: 车贴着队首 #2 时, 同一串目标连判两拍都是"只删 #2"
// (nav2 的行为树里 input_goals 与 output_goals 是同一个 blackboard 变量, 列表只会变短;
//  没有持久记忆, "同一串点再来一轮"(loop)才不会被上一轮结果污染)
TEST_F(RemovePassedGoalsByRouteTest, TickingTwiceIsStable)
{
  setRobot(kWaypoints[1].first + 0.2, kWaypoints[1].second);
  const auto first = tick(makeGoals(kWaypoints, 1, 4));    // [#2,#3,#4]
  const auto second = tick(makeGoals(kWaypoints, 1, 4));
  ASSERT_EQ(first.size(), 2u);
  EXPECT_NEAR(first[0].first, kWaypoints[2].first, 1e-6);  // #2 被删, 队首变成 #3
  ASSERT_EQ(second.size(), 2u);
  EXPECT_NEAR(second[0].first, kWaypoints[2].first, 1e-6);
}

// 车站在 #2 上但队首是没到过的 #1 → 一个点都不能删(前缀规矩的八字版本:
// #2 与 #5 只差 0.44 m, 少了这条规矩就会把另一瓣的点当成"已到过")
TEST_F(RemovePassedGoalsByRouteTest, HeadNotVisitedBlocksEverythingBehindIt)
{
  setRobot(kWaypoints[1].first, kWaypoints[1].second);
  const auto out = tick(makeGoals(kWaypoints));
  expectSame(out, kWaypoints, 0, 7);
}

// ============================================================================
// route_aware 那棵 BT 的"节点名 + 库"接得对不对
//   1) 自研 BT 节点的库必须导出 BT_RegisterNodesFromPlugin(编译时要有 BT_PLUGIN_EXPORT);
//   2) xml 里的节点名/端口名必须和 providedPorts 一致;
//   3) 其它 nav2 节点库要能按 plugin_lib_names 里的库名加载。
// 这里不真的构造整棵树(nav2 的动作节点会在构造时等 action server, 单测里没有导航栈),
// 而是: 按库名加载所有插件 → 扫 xml 里用到的每个节点名 → 要求它已注册。
// 等价于 bt_navigator 启动时的那部分校验, 不需要跑导航栈。
// ============================================================================
TEST(RemovePassedGoalsByRouteTreeTest, NodesUsedByRouteAwareTreeAreRegistered)
{
  BT::BehaviorTreeFactory factory;
  const std::vector<std::string> libs = {
    "srm27_remove_passed_goals_bt_node",              // 本包自研节点
    "nav2_compute_path_through_poses_action_bt_node",
    "nav2_follow_path_action_bt_node",
    "nav2_recovery_node_bt_node",
    "nav2_pipeline_sequence_bt_node",
    "nav2_round_robin_node_bt_node",
    "nav2_rate_controller_bt_node",
    "nav2_goal_updated_condition_bt_node",
    "nav2_clear_costmap_service_bt_node",
    "nav2_back_up_action_bt_node",
  };
  // 与 nav2 的 bt_navigator 一样: plugin_lib_names 里是逻辑名, 实际要拼成 lib<name>.so
  for (const auto & lib : libs) {
    const std::string file = "lib" + lib + ".so";
    try {
      factory.registerFromPlugin(file);
    } catch (const std::exception & ex) {
      FAIL() << "加载 BT 节点库失败: " << file << " -> " << ex.what();
    }
  }
  // 自研节点必须真的注册上了(没定义 BT_PLUGIN_EXPORT 时就会漏)
  EXPECT_NE(factory.manifests().count("RemovePassedGoalsByRoute"), 0u)
    << "srm27_remove_passed_goals_bt_node 没有注册 RemovePassedGoalsByRoute"
    << "(多半是编译时漏了 BT_PLUGIN_EXPORT)";

  std::ifstream file(SRM27_ROUTE_AWARE_BT_XML);
  ASSERT_TRUE(file.good()) << "打不开 " << SRM27_ROUTE_AWARE_BT_XML;
  const std::string xml((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  ASSERT_FALSE(xml.empty());

  // 扫出 xml 里所有 <NodeName ...> 形式的标签, 逐个核对是否已注册
  const std::regex tag(R"(<\s*([A-Za-z_][A-Za-z0-9_]*)[\s>/])");
  size_t checked = 0;
  for (auto it = std::sregex_iterator(xml.begin(), xml.end(), tag);
    it != std::sregex_iterator(); ++it)
  {
    const std::string name = (*it)[1].str();
    if (name == "root" || name == "BehaviorTree") {
      continue;                                   // BT.CPP 的结构标签, 不是节点
    }
    ++checked;
    EXPECT_NE(factory.manifests().count(name), 0u)
      << "行为树里用到节点 <" << name << "> 但没有任何已加载的库注册它";
  }
  EXPECT_GT(checked, 5u) << "xml 解析出来的节点太少, 解析逻辑可能有问题";

  // 端口名必须是节点真正提供的 —— 拼错端口不会报错, 只会静默用默认值
  for (const char * port : {"approach_radius", "route_margin", "keep_last"}) {
    EXPECT_NE(xml.find(port), std::string::npos)
      << "route_aware 行为树里应当显式写出端口 " << port;
  }
  // 弧长投影那套端口已经废弃, 别再把它们加回来
  for (const char * gone : {"advance_window", "projection_max_dist", "corridor"}) {
    EXPECT_EQ(xml.find(gone), std::string::npos)
      << "route_aware 行为树里不该再出现已废弃的端口 " << gone;
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
