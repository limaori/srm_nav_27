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
#include <vector>

#include "srm27_minco_controller/command_frame.hpp"
#include "srm27_minco_controller/minco_mpc_controller.hpp"
#include "srm27_minco_core/esdf_2d.hpp"
#include "srm27_minco_core/grid_snapshot.hpp"

namespace srm27_minco_controller
{
struct CommandContinuityTestPeer
{
  static void checkStoppedCommandHistory(bool _was_moving)
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("command_continuity_test");
    MincoMpcController controller;
    srm27_minco_core::MpcModel::Config config;
    ASSERT_TRUE(controller.mpc_model_.configure(config));
    controller.previous_applied_input_ = Eigen::Vector3d(2.0, -0.5, 0.0);
    controller.has_previous_input_ = _was_moving;
    controller.last_control_stamp_ = 99.0;
    const auto stop = controller.stopCommand(node, "candidate rejected", 100.0);
    EXPECT_DOUBLE_EQ(stop.twist.linear.x, 0.0);
    EXPECT_DOUBLE_EQ(stop.twist.linear.y, 0.0);
    // Stopping must preserve the actual zero as the next command's slew-rate
    // baseline, even if no previous normal command was available.
    ASSERT_TRUE(controller.has_previous_input_);
    ASSERT_TRUE(controller.has_last_control_stamp_);
    EXPECT_DOUBLE_EQ(controller.last_control_stamp_, 100.0);
    EXPECT_EQ(controller.previous_applied_input_.size(), controller.mpc_model_.inputDim());
    EXPECT_DOUBLE_EQ(controller.previous_applied_input_.norm(), 0.0);
  }
};
}  // namespace srm27_minco_controller

TEST(CommandContinuityTest, StopIsTheResumeAccelerationBaseline)
{
  srm27_minco_controller::CommandContinuityTestPeer::checkStoppedCommandHistory(true);
}

TEST(CommandContinuityTest, PlanningGraceZeroAlsoEstablishesCommandHistory)
{
  srm27_minco_controller::CommandContinuityTestPeer::checkStoppedCommandHistory(false);
}

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

TEST(ClearanceVelocityTest, FullSpeedTangentialMotionIsUnchanged)
{
  const Eigen::Vector2d velocity(2.6, 0.0);
  const auto limited = srm27_minco_controller::clearanceLimitedVelocity(
    velocity, Eigen::Vector2d(0, 1), 0.43, 0.38, 7.0, 0.1);
  EXPECT_LT((limited - velocity).norm(), 1.0e-12);
}

TEST(ClearanceVelocityTest, InwardComponentFitsReactionAndBrakingDistance)
{
  const auto limited = srm27_minco_controller::clearanceLimitedVelocity(
    Eigen::Vector2d(2.0, -1.0), Eigen::Vector2d(0, 2), 0.43, 0.38, 7.0, 0.1);
  const double approach = -limited.y();
  EXPECT_DOUBLE_EQ(limited.x(), 2.0);
  EXPECT_GT(approach, 0.0);
  EXPECT_LT(approach, 1.0);
  EXPECT_LE(approach * 0.1 + approach * approach / 14.0, 0.05 + 1.0e-12);
}

TEST(ClearanceVelocityTest, OutwardMotionIsUnchanged)
{
  const Eigen::Vector2d velocity(2.2, 0.4);
  const auto limited = srm27_minco_controller::clearanceLimitedVelocity(
    velocity, Eigen::Vector2d(0, 1), 0.39, 0.38, 7.0, 0.1);
  EXPECT_LT((limited - velocity).norm(), 1.0e-12);
}

TEST(ClearanceVelocityTest, ExistingClearanceViolationCannotMoveFurtherInward)
{
  const auto limited = srm27_minco_controller::clearanceLimitedVelocity(
    Eigen::Vector2d(2.0, -0.5), Eigen::Vector2d(0, 1), 0.36, 0.38, 7.0, 0.1);
  EXPECT_DOUBLE_EQ(limited.x(), 2.0);
  EXPECT_NEAR(limited.y(), 0.0, 1.0e-12);
}

TEST(ClearanceVelocityTest, MissingNormalNearObstacleStops)
{
  const auto limited = srm27_minco_controller::clearanceLimitedVelocity(
    Eigen::Vector2d(2.0, 0), Eigen::Vector2d::Zero(), 0.36, 0.38, 7.0, 0.1);
  EXPECT_DOUBLE_EQ(limited.norm(), 0.0);
}

TEST(ClearanceVelocityTest, InvalidBrakingModelFailsClosed)
{
  const auto limited = srm27_minco_controller::clearanceLimitedVelocity(
    Eigen::Vector2d(2.0, -1), Eigen::Vector2d(0, 1), 0.43, 0.38, 0.0, 0.1);
  EXPECT_DOUBLE_EQ(limited.norm(), 0.0);
}

TEST(ClearanceVelocityTest, ConstraintIsInvariantToFrameRotation)
{
  const Eigen::Vector2d velocity(1.8, -0.9);
  const Eigen::Vector2d normal(0.2, 0.8);
  const auto limited = srm27_minco_controller::clearanceLimitedVelocity(
    velocity, normal, 0.45, 0.38, 7.0, 0.1);
  const auto rotated = srm27_minco_controller::clearanceLimitedVelocity(
    Eigen::Vector2d(-velocity.y(), velocity.x()), Eigen::Vector2d(-normal.y(), normal.x()),
    0.45, 0.38, 7.0, 0.1);
  EXPECT_LT((rotated - Eigen::Vector2d(-limited.y(), limited.x())).norm(), 1.0e-12);
}

namespace
{
srm27_minco_core::Esdf2D corridor(bool _blocked_ahead)
{
  using srm27_minco_core::CellState;
  constexpr int size = 90;
  std::vector<CellState> cells(size * size, CellState::kFree);
  for (int i = 0; i < size; ++i) {
    cells[30 * size + i] = CellState::kOccupied;
    cells[49 * size + i] = CellState::kOccupied;
    if (_blocked_ahead) {
      cells[i * size + 58] = CellState::kOccupied;
    }
  }
  srm27_minco_core::GridSnapshot grid;
  EXPECT_TRUE(grid.reset(-2.0, -2.0, 0.05, size, size, std::move(cells), 1, 0.0));
  srm27_minco_core::Esdf2D esdf;
  EXPECT_TRUE(esdf.build(grid));
  return esdf;
}
}  // namespace

TEST(BrakingPathTest, ClearNarrowCorridorRetainsFullSpeed)
{
  const auto limited = srm27_minco_controller::brakingPathLimitedVelocity(
    corridor(false), Eigen::Vector2d::Zero(), Eigen::Vector2d(2.6, 0.0), 0.38, 7.0, 0.1);
  EXPECT_NEAR(limited.x(), 2.6, 1.0e-9);
  EXPECT_NEAR(limited.y(), 0.0, 1.0e-9);
}

TEST(BrakingPathTest, ObstacleBeyondTangentialNearestWallReducesSpeed)
{
  const auto esdf = corridor(true);
  const Eigen::Vector2d command(2.6, 0.0);
  const auto center = esdf.query(0.0, 0.0);
  const auto point_only = srm27_minco_controller::clearanceLimitedVelocity(
    command, center.gradient, center.distance, 0.38, 7.0, 0.1);
  EXPECT_NEAR(point_only.x(), 2.6, 1.0e-9);
  const auto limited = srm27_minco_controller::brakingPathLimitedVelocity(
    esdf, Eigen::Vector2d::Zero(), command, 0.38, 7.0, 0.1);
  EXPECT_GT(limited.x(), 0.0);
  EXPECT_LT(limited.x(), 2.6);
  const double stopped_at = limited.x() * 0.1 + limited.squaredNorm() / 14.0;
  EXPECT_GE(esdf.query(stopped_at, 0.0).distance, 0.38 - 1.0e-6);
}

TEST(BrakingPathTest, EscapeCanIncreaseClearanceInsideMargin)
{
  const auto limited = srm27_minco_controller::brakingPathLimitedVelocity(
    corridor(false), Eigen::Vector2d(0.0, 0.20), Eigen::Vector2d(0.0, -0.5), 0.38, 7.0, 0.1);
  EXPECT_NEAR(limited.y(), -0.5, 1.0e-9);
}

TEST(BrakingPathTest, CurrentMomentumAllowsAnAlreadySafeSlowerCommand)
{
  const auto esdf = corridor(true);
  const Eigen::Vector2d command(0.5, 0.0);
  const auto limited = srm27_minco_controller::limitExecutionVelocity(
    esdf, Eigen::Vector2d::Zero(), command, Eigen::Vector2d(2.6, 0.0), 0.38, 7.0, 0.1);
  EXPECT_NEAR(limited.x(), 0.5, 1.0e-9);
}

TEST(RotatingFrameLimitsTest, StationaryGimbalPreservesStraightLineLimits)
{
  const auto planar = srm27_minco_controller::rotatingFrameLimits(2.6, 7.0, 0.0);
  EXPECT_DOUBLE_EQ(planar.speed, 2.6);
  EXPECT_DOUBLE_EQ(planar.accel, 7.0);
}

TEST(RotatingFrameLimitsTest, BothFullRateDirectionsLeaveBrakingRoom)
{
  for (double yaw_rate : {8.5, -8.5}) {
    const auto planar = srm27_minco_controller::rotatingFrameLimits(2.6, 7.0, yaw_rate);
    EXPECT_NEAR(planar.speed, 3.5 / 8.5, 1.0e-9);
    EXPECT_DOUBLE_EQ(planar.accel, 3.5);
    EXPECT_LE(std::abs(yaw_rate) * planar.speed + planar.accel, 7.0 + 1.0e-9);
  }
}
