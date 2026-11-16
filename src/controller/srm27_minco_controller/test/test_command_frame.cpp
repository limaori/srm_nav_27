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

#include <cmath>
#include <string>

#include "srm27_minco_controller/command_frame.hpp"

/// \file
/// \brief `command_frame` 输出坐标系选择的回归（方案 §3.2、§4.1.3）。
///
/// 两个必须在数值上钉住的性质：
///
/// 1. `command_frame=base_link`：输出真实车体系速度，psi=0/±pi/2/pi 的方向必须正确；
/// 2. `command_frame=base_link_fake`：输出 odom 对齐的平移速度（**不做** `R(-psi)`）；
///    下游 `fake_vel_transform` 再做唯一一次 `R(-psi)` 之后，必须与 1 的结果完全一致 ——
///    这条"组合起来等于一次旋转"的断言就是"重复旋转"（方案 §5 三类错误之一）的护栏。

namespace
{

using srm27_minco_controller::CommandFrameMode;
using srm27_minco_controller::resolveCommandFrame;
using srm27_minco_controller::velocityInCommandFrame;

constexpr double kPi = 3.14159265358979323846;
constexpr double kTolerance = 1.0e-9;

/// \brief fake_vel_transform 对导航速度做的唯一一次转换：v_base = R(-psi) * v_fake。
/// 这里按方案 §3.1 的公式显式写出，不复用被测代码。
Eigen::Vector2d fakeTransform(const Eigen::Vector2d & _v_fake, const double _psi)
{
  return Eigen::Vector2d(
    std::cos(_psi) * _v_fake.x() + std::sin(_psi) * _v_fake.y(),
    -std::sin(_psi) * _v_fake.x() + std::cos(_psi) * _v_fake.y());
}

void expectVectorNear(const Eigen::Vector2d & _actual, const Eigen::Vector2d & _expected)
{
  EXPECT_NEAR(_actual.x(), _expected.x(), kTolerance);
  EXPECT_NEAR(_actual.y(), _expected.y(), kTolerance);
}

}  // namespace

// command_frame 只能是"车体系"或"fake 系"，第三个名字必须直接失败。
TEST(CommandFrameTest, ResolveAcceptsOnlyTheTwoKnownFrames)
{
  CommandFrameMode mode = CommandFrameMode::kOdomAlignedFake;
  std::string reason;
  EXPECT_TRUE(resolveCommandFrame("base_link", "base_link", "base_link_fake", mode, &reason));
  EXPECT_EQ(mode, CommandFrameMode::kRobotBase);

  EXPECT_TRUE(
    resolveCommandFrame("base_link_fake", "base_link", "base_link_fake", mode, &reason));
  EXPECT_EQ(mode, CommandFrameMode::kOdomAlignedFake);

  EXPECT_FALSE(resolveCommandFrame("gimbal_link", "base_link", "base_link_fake", mode, &reason));
  EXPECT_FALSE(reason.empty()) << "失败原因必须写清楚，否则现场只能猜";
  EXPECT_FALSE(resolveCommandFrame("", "base_link", "base_link_fake", mode, &reason));
}

// 车体系命令：psi = 0 时原样通过。
TEST(CommandFrameTest, RobotBaseIsIdentityAtZeroYaw)
{
  const Eigen::Vector2d v_odom(0.3, 0.2);
  expectVectorNear(
    velocityInCommandFrame(v_odom, 0.0, CommandFrameMode::kRobotBase), Eigen::Vector2d(0.3, 0.2));
}

// 车体系命令：psi = +pi/2（odom 前进落在车体右手边）。
TEST(CommandFrameTest, RobotBaseRotatesAtHalfPi)
{
  const Eigen::Vector2d v_odom(1.0, 0.0);
  expectVectorNear(
    velocityInCommandFrame(v_odom, kPi / 2.0, CommandFrameMode::kRobotBase),
    Eigen::Vector2d(0.0, -1.0));
  expectVectorNear(
    velocityInCommandFrame(v_odom, -kPi / 2.0, CommandFrameMode::kRobotBase),
    Eigen::Vector2d(0.0, 1.0));
}

// 车体系命令：psi = pi（两个分量都反号）。
TEST(CommandFrameTest, RobotBaseAtHalfTurn)
{
  const Eigen::Vector2d v_odom(0.3, -0.2);
  expectVectorNear(
    velocityInCommandFrame(v_odom, kPi, CommandFrameMode::kRobotBase),
    Eigen::Vector2d(-0.3, 0.2));
}

// fake 系命令：输出就是 odom 对齐的平移速度，绝不在这里再乘一次 R(-psi)。
TEST(CommandFrameTest, OdomAlignedFakePassesVelocityThroughUnchanged)
{
  const Eigen::Vector2d v_odom(0.3, 0.2);
  for (const double psi : {0.0, kPi / 2.0, kPi, -kPi / 2.0, 1.1}) {
    expectVectorNear(
      velocityInCommandFrame(v_odom, psi, CommandFrameMode::kOdomAlignedFake), v_odom);
  }
}

// 关键回归：fake 系输出 + 下游唯一一次 R(-psi) == 车体系输出。
// 若控制器"顺手"先转一次，这里就会在 psi≈pi/2 处把前进变成侧移（重复旋转）。
TEST(CommandFrameTest, FakeFrameOutputComposesToASingleRotation)
{
  const Eigen::Vector2d v_odom(0.3, 0.2);
  for (const double psi : {0.0, kPi / 2.0, kPi, -kPi / 2.0, -2.4, 0.7}) {
    const Eigen::Vector2d via_fake = fakeTransform(
      velocityInCommandFrame(v_odom, psi, CommandFrameMode::kOdomAlignedFake), psi);
    const Eigen::Vector2d direct =
      velocityInCommandFrame(v_odom, psi, CommandFrameMode::kRobotBase);
    expectVectorNear(via_fake, direct);
    // 模长在两种路径下都必须一致（方案 §6 的验收指标）。
    EXPECT_NEAR(via_fake.norm(), direct.norm(), kTolerance);
  }
}

// 重复旋转的反例检查：如果 fake 系路径也乘了 R(-psi)，角度会差 2*psi。
// 这里显式说明"错在哪里"，避免后来的人把这条断言当成多余。
TEST(CommandFrameTest, DoubleRotationWouldBeDetectable)
{
  const double psi = kPi / 2.0;
  const Eigen::Vector2d v_odom(0.3, 0.2);
  const Eigen::Vector2d correct =
    velocityInCommandFrame(v_odom, psi, CommandFrameMode::kOdomAlignedFake);
  const Eigen::Vector2d wrong =
    srm27_minco_core::odomToBodyVelocity(psi, correct);  // 被重复转换一次
  EXPECT_GT((wrong - correct).norm(), 0.1) << "重复旋转必须能被这条断言发现";
}
