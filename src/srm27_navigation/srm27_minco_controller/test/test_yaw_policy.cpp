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
