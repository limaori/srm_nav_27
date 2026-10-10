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

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_core/goal_checker.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "srm27_nav_plugins/goal_checkers/omni_stopped_goal_checker.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

/// \file
/// \brief `OmniStoppedGoalChecker` 的行为测试。
///
/// 核心断言只有一条：**角速度不参与到达判定**。
/// 这与 Nav2 的 `StoppedGoalChecker` 是唯一差别，也是 SRM 哨兵必须的语义 ——
/// 导航不拥有自转（`xy_only` 下插件输出 wz≡0，smoother 把 yaw 上界钳成 0），
/// 自转由下位机/独立链路负责，因此它不能否掉导航的到达结论。
namespace
{

constexpr double kPi = 3.14159265358979323846;

geometry_msgs::msg::Pose makePose(double _x, double _y, double _yaw = 0.0)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = _x;
  pose.position.y = _y;
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, _yaw);
  pose.orientation = tf2::toMsg(quaternion);
  return pose;
}

geometry_msgs::msg::Twist makeTwist(double _vx, double _vy, double _wz)
{
  geometry_msgs::msg::Twist twist;
  twist.linear.x = _vx;
  twist.linear.y = _vy;
  twist.angular.z = _wz;
  return twist;
}

/// \brief 用一组给定参数构造检查器；参数通过节点 override 提供。
std::shared_ptr<srm27_nav_plugins::OmniStoppedGoalChecker> makeChecker(
  double _xy_tolerance, double _yaw_tolerance, double _trans_stopped)
{
  const std::string name = "test_goal_checker";
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {{name + ".xy_goal_tolerance", _xy_tolerance},
     {name + ".yaw_goal_tolerance", _yaw_tolerance},
     {name + ".trans_stopped_velocity", _trans_stopped}});
  auto node =
    std::make_shared<rclcpp_lifecycle::LifecycleNode>("goal_checker_test_node", "", options);

  auto checker = std::make_shared<srm27_nav_plugins::OmniStoppedGoalChecker>();
  checker->initialize(node, name, nullptr);
  return checker;
}

}  // namespace

/// \brief 角速度任意大都不影响到达判定 —— 本插件的存在理由。
TEST(OmniStoppedGoalCheckerTest, AngularVelocityIsIrrelevant)
{
  auto checker = makeChecker(0.40, 6.28, 0.03);
  const auto goal = makePose(1.0, 0.0);

  // 位置在容差内、平动已停：无论自转多快都算到达。
  for (const double wz : {0.0, 0.05, 0.5, 1.0, 3.14, 100.0, -2.0}) {
    EXPECT_TRUE(checker->isGoalReached(makePose(0.9, 0.0), goal, makeTwist(0.0, 0.0, wz)))
      << "wz=" << wz << " 不应影响到达判定";
  }
}

/// \brief 与 Nav2 StoppedGoalChecker 的对照：同样的输入在那边会因 wz 被否掉。
///
/// 这里不实例化 Nav2 的检查器（会把 nav2_controller 拉成测试依赖），
/// 而是用文档化的判据把"差别在哪"固化下来：wz=0.5 > rot_stopped_velocity=0.05
/// 时 StoppedGoalChecker 会返回 false，本插件返回 true。
TEST(OmniStoppedGoalCheckerTest, DocumentsTheDifferenceFromStoppedGoalChecker)
{
  auto checker = makeChecker(0.40, 6.28, 0.03);
  const double nav_stopped_rot_velocity = 0.05;  // 实车原配置值
  const double wz = 0.5;
  ASSERT_GT(std::abs(wz), nav_stopped_rot_velocity)
    << "该用例的前提是 wz 超过 Nav2 的 rot_stopped_velocity";
  EXPECT_TRUE(
    checker->isGoalReached(makePose(0.9, 0.0), makePose(1.0, 0.0), makeTwist(0.0, 0.0, wz)));
}

/// \brief 平动停稳仍然必须检查：不能高速掠过目标点就算到达。
TEST(OmniStoppedGoalCheckerTest, TranslationMustBeStopped)
{
  auto checker = makeChecker(0.40, 6.28, 0.03);
  const auto goal = makePose(1.0, 0.0);
  EXPECT_TRUE(checker->isGoalReached(makePose(0.9, 0.0), goal, makeTwist(0.03, 0.0, 0.0)));
  EXPECT_FALSE(checker->isGoalReached(makePose(0.9, 0.0), goal, makeTwist(0.04, 0.0, 0.0)));
  // 全向底盘：横移速度同样要停住。
  EXPECT_TRUE(checker->isGoalReached(makePose(0.9, 0.0), goal, makeTwist(0.0, 0.03, 0.0)));
  EXPECT_FALSE(checker->isGoalReached(makePose(0.9, 0.0), goal, makeTwist(0.0, -0.04, 0.0)));
}

/// \brief 位置容差边界。
TEST(OmniStoppedGoalCheckerTest, PositionToleranceBoundary)
{
  auto checker = makeChecker(0.40, 6.28, 0.03);
  const auto goal = makePose(1.0, 0.0);
  const auto stopped = makeTwist(0.0, 0.0, 0.0);
  EXPECT_TRUE(checker->isGoalReached(makePose(0.60, 0.0), goal, stopped));
  EXPECT_FALSE(checker->isGoalReached(makePose(0.59, 0.0), goal, stopped));
  // 斜向：按欧氏距离，不是分轴。
  EXPECT_FALSE(checker->isGoalReached(makePose(1.0 - 0.29, 0.29), goal, stopped));
}

/// \brief 航向只作位姿比较；取 6.28（>= pi）时等于不约束朝向。
TEST(OmniStoppedGoalCheckerTest, YawToleranceIsFreeWhenSetToTwoPi)
{
  auto checker = makeChecker(0.40, 6.28, 0.03);
  const auto stopped = makeTwist(0.0, 0.0, 0.0);
  for (const double yaw : {0.0, 0.5, kPi / 2.0, kPi - 0.01, -kPi + 0.01}) {
    EXPECT_TRUE(checker->isGoalReached(makePose(0.9, 0.0, yaw), makePose(1.0, 0.0, 0.0), stopped))
      << "yaw=" << yaw;
  }
}

/// \brief 收紧航向容差后必须重新生效（本插件并未把朝向判定一并删掉）。
TEST(OmniStoppedGoalCheckerTest, YawToleranceIsHonouredWhenTightened)
{
  auto checker = makeChecker(0.40, 0.10, 0.03);
  const auto stopped = makeTwist(0.0, 0.0, 0.0);
  EXPECT_TRUE(checker->isGoalReached(makePose(0.9, 0.0, 0.05), makePose(1.0, 0.0, 0.0), stopped));
  EXPECT_FALSE(checker->isGoalReached(makePose(0.9, 0.0, 0.30), makePose(1.0, 0.0, 0.0), stopped));
}

/// \brief `getTolerances` 报出的角速度容差必须是"未测量"，与判定一致。
TEST(OmniStoppedGoalCheckerTest, ReportsAngularVelocityToleranceAsUnmeasured)
{
  auto checker = makeChecker(0.40, 6.28, 0.03);
  geometry_msgs::msg::Pose pose_tolerance;
  geometry_msgs::msg::Twist vel_tolerance;
  ASSERT_TRUE(checker->getTolerances(pose_tolerance, vel_tolerance));
  EXPECT_DOUBLE_EQ(pose_tolerance.position.x, 0.40);
  EXPECT_DOUBLE_EQ(pose_tolerance.position.y, 0.40);
  EXPECT_DOUBLE_EQ(vel_tolerance.linear.x, 0.03);
  EXPECT_DOUBLE_EQ(vel_tolerance.linear.y, 0.03);
  const double lowest = std::numeric_limits<double>::lowest();
  EXPECT_DOUBLE_EQ(vel_tolerance.angular.z, lowest)
    << "角速度不参与判定, 必须报成未测量而不是给一个假的容差";
  EXPECT_DOUBLE_EQ(pose_tolerance.position.z, lowest);
}

/// \brief 非法参数在 initialize 阶段直接失败，而不是带着坏值运行。
TEST(OmniStoppedGoalCheckerTest, RejectsInvalidParameters)
{
  EXPECT_THROW(makeChecker(-1.0, 6.28, 0.03), std::runtime_error);
  EXPECT_THROW(makeChecker(0.40, -1.0, 0.03), std::runtime_error);
  EXPECT_THROW(makeChecker(0.40, 6.28, -0.01), std::runtime_error);
}

/// \brief pluginlib 必须真的能按名字加载它 —— 这是 controller_server 的加载路径。
///
/// 直接实例化通过不代表能加载：少一个 XML 条目、少一次 pluginlib_export、或者库名写错，
/// 都会让 controller_server 在加载 FollowPath 时失败（那时整个导航栈起不来）。
namespace
{

constexpr char kPluginClass[] = "srm27_nav_plugins::OmniStoppedGoalChecker";

}  // namespace

TEST(OmniStoppedGoalCheckerPluginTest, DeclaredAsNav2GoalCheckerPlugin)
{
  pluginlib::ClassLoader<nav2_core::GoalChecker> loader("nav2_core", "nav2_core::GoalChecker");
  const std::vector<std::string> classes = loader.getDeclaredClasses();
  EXPECT_NE(std::find(classes.begin(), classes.end(), kPluginClass), classes.end())
    << "goal_checker_plugins.xml 没有把 " << kPluginClass << " 注册成 nav2_core::GoalChecker";
}

TEST(OmniStoppedGoalCheckerPluginTest, CanBeInstantiatedThroughPluginlib)
{
  pluginlib::ClassLoader<nav2_core::GoalChecker> loader("nav2_core", "nav2_core::GoalChecker");
  nav2_core::GoalChecker::Ptr instance;
  ASSERT_NO_THROW(instance = loader.createSharedInstance(kPluginClass));
  ASSERT_NE(instance, nullptr);
  // 默认参数（0.25/0.25/0.25）下：正好在容差边界上应判到达，wz 无所谓。
  EXPECT_TRUE(
    instance->isGoalReached(makePose(0.80, 0.0), makePose(1.0, 0.0), makeTwist(0.0, 0.0, 5.0)));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
