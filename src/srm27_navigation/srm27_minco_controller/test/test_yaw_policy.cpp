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

#include "srm27_minco_controller/yaw_policy.hpp"

/// \file
/// \brief 航向策略的行为测试（方案 §8.1、§8.2）。
///
/// 重点：`xy_only` 阶段 MPC 的角速度权限必须为 0（自转仍由独立链路负责），
/// FOLLOW 模式的航向参考必须连续展开且不跨 2*pi 分支，SPIN 模式在没有有效请求时
/// 必须按停止处理。

namespace
{

srm27_minco_controller::YawPolicy::Config makeConfig(srm27_minco_controller::YawMode _mode)
{
  srm27_minco_controller::YawPolicy::Config config;
  config.mode = _mode;
  config.max_angular_speed = 0.3;
  config.max_angular_accel = 0.5;
  config.narrow_clearance_threshold = 0.6;
  config.allow_reverse = true;
  return config;
}

}  // namespace

TEST(YawPolicyTest, ParseModeRoundTrip)
{
  srm27_minco_controller::YawMode mode = srm27_minco_controller::YawMode::kSpin;
  EXPECT_TRUE(srm27_minco_controller::parseYawMode("xy_only", mode));
  EXPECT_EQ(mode, srm27_minco_controller::YawMode::kXyOnly);
  EXPECT_TRUE(srm27_minco_controller::parseYawMode("follow_tangent", mode));
  EXPECT_EQ(mode, srm27_minco_controller::YawMode::kFollowTangent);
  EXPECT_TRUE(srm27_minco_controller::parseYawMode("spin", mode));
  EXPECT_EQ(mode, srm27_minco_controller::YawMode::kSpin);
  EXPECT_TRUE(srm27_minco_controller::parseYawMode("narrow_track", mode));
  EXPECT_EQ(mode, srm27_minco_controller::YawMode::kNarrowTrack);
  EXPECT_FALSE(srm27_minco_controller::parseYawMode("spin_up", mode));
  EXPECT_STREQ(
    srm27_minco_controller::toString(srm27_minco_controller::YawMode::kXyOnly), "xy_only");
}

TEST(YawPolicyTest, ConfigureRejectsInvalidValues)
{
  srm27_minco_controller::YawPolicy policy;
  std::string reason;
  auto config = makeConfig(srm27_minco_controller::YawMode::kXyOnly);
  config.max_angular_speed = -1.0;
  EXPECT_FALSE(policy.configure(config, &reason));
  EXPECT_FALSE(reason.empty());

  config = makeConfig(srm27_minco_controller::YawMode::kFollowTangent);
  config.max_angular_speed = 0.0;
  EXPECT_FALSE(policy.configure(config, &reason)) << "非 xy_only 模式必须要求正的角速度上限";

  config = makeConfig(srm27_minco_controller::YawMode::kXyOnly);
  config.narrow_clearance_threshold = std::nan("");
  EXPECT_FALSE(policy.configure(config, &reason));

  EXPECT_TRUE(policy.configure(makeConfig(srm27_minco_controller::YawMode::kXyOnly), &reason));
}

TEST(YawPolicyTest, XyOnlyRemovesAngularAuthority)
{
  srm27_minco_controller::YawPolicy policy;
  ASSERT_TRUE(policy.configure(makeConfig(srm27_minco_controller::YawMode::kXyOnly)));
  policy.reset(0.4);
  policy.setCurrentYaw(0.7);

  // 阶段一：MPC 不允许产生任何角速度，否则会与独立自转链路叠加（方案 §8.1）。
  EXPECT_DOUBLE_EQ(policy.angularSpeedLimit(), 0.0);
  EXPECT_DOUBLE_EQ(policy.angularAccelLimit(), 0.0);

  double yaw = 0.0;
  double omega = 0.0;
  const Eigen::Vector2d tangent(1.0, 0.0);
  ASSERT_TRUE(policy.yawReference(
    0.0, Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(), true, tangent, 0.4, yaw, omega));
  EXPECT_DOUBLE_EQ(yaw, 0.7);
  EXPECT_DOUBLE_EQ(omega, 0.0);
}

TEST(YawPolicyTest, FollowTangentUnwrapsAcrossPi)
{
  srm27_minco_controller::YawPolicy policy;
  ASSERT_TRUE(policy.configure(makeConfig(srm27_minco_controller::YawMode::kFollowTangent)));
  policy.reset(0.0);

  // 切线角刚好越过 +pi：连续参考必须选择与上一次相同的 2*pi 分支。
  const double previous_yaw = 3.10;
  const double tangent_angle = -3.10;
  double yaw = 0.0;
  double omega = 0.0;
  const Eigen::Vector2d tangent(std::cos(tangent_angle), std::sin(tangent_angle));
  ASSERT_TRUE(policy.yawReference(
    1.0, Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(), true, tangent, previous_yaw, yaw,
    omega));
  EXPECT_LT(std::abs(yaw - previous_yaw), 0.2) << "跨 ±pi 时应继续展开，而不是跳到另一个分支";
  EXPECT_GT(yaw, 3.14159265358979323846) << "展开后的参考应大于 pi";
  EXPECT_LE(std::abs(omega), 0.3 + 1.0e-12);
}

TEST(YawPolicyTest, FollowTangentKeepsPreviousYawWhenTangentIsUnavailable)
{
  srm27_minco_controller::YawPolicy policy;
  ASSERT_TRUE(policy.configure(makeConfig(srm27_minco_controller::YawMode::kFollowTangent)));
  policy.reset(-0.3);

  double yaw = 0.0;
  double omega = 0.0;
  ASSERT_TRUE(policy.yawReference(
    0.5, Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(), false, Eigen::Vector2d::Zero(), -0.3,
    yaw, omega));
  EXPECT_DOUBLE_EQ(yaw, -0.3) << "低速切线不可用时保持上一有效朝向，避免 atan2(0,0) 跳变";
  EXPECT_DOUBLE_EQ(omega, 0.0);
}

TEST(YawPolicyTest, ReverseTangentChoosesTheNearerBranch)
{
  srm27_minco_controller::YawPolicy policy;
  auto config = makeConfig(srm27_minco_controller::YawMode::kFollowTangent);
  config.allow_reverse = true;
  ASSERT_TRUE(policy.configure(config));
  policy.reset(0.0);

  // 当前航向为 pi（朝向 -x），切线指向 +x：应选择反向切向（更接近当前姿态）。
  double yaw = 0.0;
  double omega = 0.0;
  const Eigen::Vector2d tangent(1.0, 0.0);
  ASSERT_TRUE(policy.yawReference(
    0.0, Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(), true, tangent, 3.14159265358979323846,
    yaw, omega));
  EXPECT_NEAR(std::abs(std::abs(yaw) - 3.14159265358979323846), 0.0, 1.0e-9);
}

TEST(YawPolicyTest, SpinRequiresAnActiveRequest)
{
  srm27_minco_controller::YawPolicy policy;
  ASSERT_TRUE(policy.configure(makeConfig(srm27_minco_controller::YawMode::kSpin)));
  policy.reset(0.0);
  EXPECT_FALSE(policy.spinRequestActive());

  double yaw = 0.0;
  double omega = 0.0;
  // 没有有效请求时按停止处理，不自动沿用独立自转（方案 §8.1）。
  ASSERT_TRUE(policy.yawReference(
    0.0, Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(), true, Eigen::Vector2d(1.0, 0.0), 0.0,
    yaw, omega));
  EXPECT_DOUBLE_EQ(omega, 0.0);

  policy.setSpinRequest(1.0);
  EXPECT_TRUE(policy.spinRequestActive());
  ASSERT_TRUE(policy.yawReference(
    0.0, Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(), true, Eigen::Vector2d(1.0, 0.0), 0.0,
    yaw, omega));
  EXPECT_DOUBLE_EQ(omega, 0.3) << "请求超过有效上限时必须被裁剪";

  policy.clearSpinRequest();
  ASSERT_TRUE(policy.yawReference(
    0.0, Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(), true, Eigen::Vector2d(1.0, 0.0), 0.0,
    yaw, omega));
  EXPECT_DOUBLE_EQ(omega, 0.0);
}

TEST(YawPolicyTest, SpinModeRestoresAngularAuthority)
{
  srm27_minco_controller::YawPolicy policy;
  ASSERT_TRUE(policy.configure(makeConfig(srm27_minco_controller::YawMode::kSpin)));
  EXPECT_DOUBLE_EQ(policy.angularSpeedLimit(), 0.3);
  EXPECT_DOUBLE_EQ(policy.angularAccelLimit(), 0.5);
  EXPECT_STREQ(policy.modeName(), "spin");
}

// ---------------------------------------------------------------- se2（wz 迁移）
TEST(YawPolicyTest, ParseSe2Mode)
{
  srm27_minco_controller::YawMode mode = srm27_minco_controller::YawMode::kXyOnly;
  EXPECT_TRUE(srm27_minco_controller::parseYawMode("se2", mode));
  EXPECT_EQ(mode, srm27_minco_controller::YawMode::kSe2);
  EXPECT_STREQ(srm27_minco_controller::toString(srm27_minco_controller::YawMode::kSe2), "se2");
}

// se2 是"导航拥有 wz"的模式：MPC 必须有正的角速度权限（否则 se2 与 xy_only 无差别）。
TEST(YawPolicyTest, Se2GrantsAngularAuthority)
{
  srm27_minco_controller::YawPolicy policy;
  ASSERT_TRUE(policy.configure(makeConfig(srm27_minco_controller::YawMode::kSe2)));
  EXPECT_DOUBLE_EQ(policy.angularSpeedLimit(), 0.3);
  EXPECT_DOUBLE_EQ(policy.angularAccelLimit(), 0.5);
  EXPECT_STREQ(policy.modeName(), "se2");
}

// se2 的航向参考只来自路径切线（首版策略，方案 §4.1.2）。
TEST(YawPolicyTest, Se2FollowsThePathTangent)
{
  srm27_minco_controller::YawPolicy policy;
  ASSERT_TRUE(policy.configure(makeConfig(srm27_minco_controller::YawMode::kSe2)));
  policy.reset(0.0);

  double yaw = 0.0;
  double omega = 0.0;
  const Eigen::Vector2d tangent(std::cos(0.6), std::sin(0.6));
  ASSERT_TRUE(policy.yawReference(
    0.0, Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(), true, tangent, 0.0, yaw, omega));
  EXPECT_NEAR(yaw, 0.6, 1.0e-9) << "se2 的 yaw 参考应是路径切线";
  EXPECT_NEAR(omega, 0.0, 1.0e-9) << "首拍没有上一次切线，角速度参考为 0";

  // 第二拍切线变化 -> 角速度参考按差分给出，并被有效上限裁剪。
  const Eigen::Vector2d tangent2(std::cos(0.9), std::sin(0.9));
  ASSERT_TRUE(policy.yawReference(
    0.1, Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(), true, tangent2, yaw, yaw, omega));
  EXPECT_NEAR(yaw, 0.9, 1.0e-9);
  EXPECT_GT(omega, 0.0) << "切线转向时应给出同向角速度参考";
  EXPECT_LE(omega, 0.3 + 1.0e-12);
}

// se2 必须**拒绝**独立自转请求：否则独立自转可以绕过导航直接决定角速度（方案 §4.1.2）。
TEST(YawPolicyTest, Se2RejectsIndependentSpinRequests)
{
  srm27_minco_controller::YawPolicy policy;
  ASSERT_TRUE(policy.configure(makeConfig(srm27_minco_controller::YawMode::kSe2)));
  policy.reset(0.0);
  EXPECT_EQ(policy.rejectedSpinRequestCount(), 0u);

  policy.setSpinRequest(1.2);
  EXPECT_FALSE(policy.spinRequestActive()) << "se2 不允许外部自转请求生效";
  EXPECT_EQ(policy.rejectedSpinRequestCount(), 1u);

  double yaw = 0.0;
  double omega = 0.0;
  const Eigen::Vector2d tangent(1.0, 0.0);
  ASSERT_TRUE(policy.yawReference(
    0.0, Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(), true, tangent, 0.0, yaw, omega));
  EXPECT_NEAR(yaw, 0.0, 1.0e-9) << "yaw 参考不能被自转请求污染";
  EXPECT_NEAR(omega, 0.0, 1.0e-9);

  // 同一份策略在 SPIN 模式下必须照旧接受请求（回归：别把 spin 模式一起改坏）。
  srm27_minco_controller::YawPolicy spin_policy;
  ASSERT_TRUE(spin_policy.configure(makeConfig(srm27_minco_controller::YawMode::kSpin)));
  spin_policy.setSpinRequest(0.2);
  EXPECT_TRUE(spin_policy.spinRequestActive());
  EXPECT_EQ(spin_policy.rejectedSpinRequestCount(), 0u);
}

// 到点保持时 se2 不再产生自转参考（否则"停稳"永远不满足，方案 §4.1.5）。
TEST(YawPolicyTest, Se2GoalHoldFreezesYawReference)
{
  srm27_minco_controller::YawPolicy policy;
  ASSERT_TRUE(policy.configure(makeConfig(srm27_minco_controller::YawMode::kSe2)));
  policy.reset(0.0);

  double yaw = 0.0;
  double omega = 0.0;
  const Eigen::Vector2d tangent(std::cos(1.0), std::sin(1.0));
  policy.setGoalHold(true);
  ASSERT_TRUE(policy.yawReference(
    1.0, Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(), true, tangent, 0.25, yaw, omega));
  EXPECT_DOUBLE_EQ(yaw, 0.25) << "到点保持应冻结在当前连续航向";
  EXPECT_DOUBLE_EQ(omega, 0.0);

  policy.setGoalHold(false);
  ASSERT_TRUE(policy.yawReference(
    1.1, Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(), true, tangent, 0.25, yaw, omega));
  EXPECT_NEAR(yaw, 1.0, 1.0e-9) << "解除保持后应重新跟踪切线";
}
