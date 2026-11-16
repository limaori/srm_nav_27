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
#include <limits>

#include "srm27_chassis_control/velocity_mix.hpp"

using srm27_chassis_control::clampFinite;
using srm27_chassis_control::isNonZeroRate;
using srm27_chassis_control::MixInput;
using srm27_chassis_control::MixLimits;
using srm27_chassis_control::mixVelocity;
using srm27_chassis_control::slewLimit;

namespace
{
MixLimits defaultLimits()
{
  MixLimits limits;
  limits.vx_max = 1.5;
  limits.vy_max = 1.5;
  limits.v_max = 2.0;
  limits.wz_max = 3.0;
  return limits;
}
}  // namespace

// 纯 vx 直行：符号与大小原样保留。
TEST(VelocityMix, PureForwardPassesThrough)
{
  MixInput input;
  input.nav_vx = 1.0;
  const auto out = mixVelocity(input, defaultLimits());
  EXPECT_DOUBLE_EQ(out.vx, 1.0);
  EXPECT_DOUBLE_EQ(out.vy, 0.0);
  EXPECT_DOUBLE_EQ(out.scale, 1.0);
  EXPECT_FALSE(out.clamped);
}

TEST(VelocityMix, PureLateralKeepsSign)
{
  MixInput input;
  input.nav_vy = -1.0;
  const auto out = mixVelocity(input, defaultLimits());
  EXPECT_DOUBLE_EQ(out.vx, 0.0);
  EXPECT_DOUBLE_EQ(out.vy, -1.0);
}

TEST(VelocityMix, PerAxisClampAppliesFirst)
{
  MixLimits limits = defaultLimits();
  limits.vx_max = 1.0;
  MixInput input;
  input.nav_vx = 5.0;
  const auto out = mixVelocity(input, limits);
  EXPECT_DOUBLE_EQ(out.vx, 1.0);
}

TEST(VelocityMix, DiagonalIsLimitedByMagnitude)
{
  MixLimits limits = defaultLimits();
  limits.v_max = 1.0;
  MixInput input;
  input.nav_vx = 1.0;
  input.nav_vy = 1.0;
  const auto out = mixVelocity(input, limits);
  EXPECT_TRUE(out.clamped);
  EXPECT_NEAR(std::hypot(out.vx, out.vy), 1.0, 1e-12);
  EXPECT_NEAR(out.vx, out.vy, 1e-12);
}

TEST(VelocityMix, ZeroVectorKeepsScaleOne)
{
  const auto out = mixVelocity(MixInput{}, defaultLimits());
  EXPECT_DOUBLE_EQ(out.scale, 1.0);
  EXPECT_FALSE(out.clamped);
}

// 导航 wz 是唯一来源，正常值原样输出。
TEST(VelocityMix, NavigationYawRatePassesThrough)
{
  MixInput input;
  input.nav_wz = 1.2;
  const auto out = mixVelocity(input, defaultLimits());
  EXPECT_DOUBLE_EQ(out.wz, 1.2);
  EXPECT_FALSE(out.wz_clamped);
}

TEST(VelocityMix, NavigationYawRateIsClamped)
{
  MixLimits limits = defaultLimits();
  limits.wz_max = 3.0;
  MixInput input;
  input.nav_wz = -4.5;
  const auto out = mixVelocity(input, limits);
  EXPECT_DOUBLE_EQ(out.wz, -3.0);
  EXPECT_TRUE(out.wz_clamped);
}

TEST(VelocityMix, TranslationIsIndependentOfYawRate)
{
  MixLimits limits = defaultLimits();
  limits.v_max = 1.0;
  MixInput input;
  input.nav_vx = 1.0;
  input.nav_vy = 1.0;
  input.nav_wz = 2.5;
  const auto out = mixVelocity(input, limits);
  // 平移被模长限幅，wz 不受平移缩放影响。
  EXPECT_TRUE(out.clamped);
  EXPECT_DOUBLE_EQ(out.wz, 2.5);
}

TEST(VelocityMix, NonFiniteInputsBecomeZero)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  MixInput input;
  input.nav_vx = nan;
  input.nav_vy = inf;
  input.nav_wz = nan;
  const auto out = mixVelocity(input, defaultLimits());
  EXPECT_DOUBLE_EQ(out.vx, 0.0);
  EXPECT_DOUBLE_EQ(out.vy, 0.0);
  EXPECT_DOUBLE_EQ(out.wz, 0.0);
  EXPECT_TRUE(out.wz_clamped);
}

TEST(VelocityMix, NonPositiveMaxSpeedBlocksTranslation)
{
  MixLimits limits = defaultLimits();
  limits.v_max = 0.0;
  MixInput input;
  input.nav_vx = 1.0;
  const auto out = mixVelocity(input, limits);
  EXPECT_DOUBLE_EQ(out.vx, 0.0);
  EXPECT_DOUBLE_EQ(out.scale, 0.0);
  EXPECT_TRUE(out.clamped);
}

TEST(VelocityMix, NonPositiveYawLimitBlocksRotation)
{
  MixLimits limits = defaultLimits();
  limits.wz_max = 0.0;
  MixInput input;
  input.nav_wz = 1.0;
  const auto out = mixVelocity(input, limits);
  EXPECT_DOUBLE_EQ(out.wz, 0.0);
  EXPECT_TRUE(out.wz_clamped);
}

TEST(VelocityMix, ClampFiniteHandlesLimits)
{
  EXPECT_DOUBLE_EQ(clampFinite(1.0, 2.0), 1.0);
  EXPECT_DOUBLE_EQ(clampFinite(-3.0, 2.0), -2.0);
  EXPECT_DOUBLE_EQ(
    clampFinite(1.0, std::numeric_limits<double>::quiet_NaN()), 0.0);
}

// ---------------------------------------------------------------------------
// 两路输入（方案 §4.3，2026-11-08 云台系改造）
// ---------------------------------------------------------------------------

/// \brief 双输入模式：平移只取导航，wz 只取决策层。
TEST(VelocityMix, DualInputSplitsTranslationAndSpin)
{
  MixInput input;
  input.nav_vx = 0.2;
  input.nav_vy = -0.1;
  input.spin_enabled = true;
  input.spin_valid = true;
  input.spin_wz = 0.8;
  const auto out = mixVelocity(input, defaultLimits());
  EXPECT_DOUBLE_EQ(out.vx, 0.2);
  EXPECT_DOUBLE_EQ(out.vy, -0.1);
  EXPECT_DOUBLE_EQ(out.wz, 0.8);
  EXPECT_TRUE(out.wz_from_spin);
  EXPECT_FALSE(out.nav_wz_ignored);
}

/// \brief 双输入模式：导航漏出的 wz 必须被忽略并标记（方案 §4.2/§4.3）。
TEST(VelocityMix, DualInputIgnoresNavigationYawRate)
{
  MixInput input;
  input.nav_wz = 1.5;             // 上游有节点往导航出口发 wz
  input.spin_enabled = true;
  input.spin_valid = true;
  input.spin_wz = -0.4;
  const auto out = mixVelocity(input, defaultLimits());
  EXPECT_DOUBLE_EQ(out.wz, -0.4) << "底盘 wz 必须只认决策层输入";
  EXPECT_TRUE(out.nav_wz_ignored);
}

/// \brief 决策层停发/超时 = 停转，不能沿用上一帧的 wz。
TEST(VelocityMix, StaleSpinInputMeansStop)
{
  MixInput input;
  input.nav_vx = 0.3;
  input.spin_enabled = true;
  input.spin_valid = false;       // 超时
  input.spin_wz = 1.0;            // 上一帧的请求
  const auto out = mixVelocity(input, defaultLimits());
  EXPECT_DOUBLE_EQ(out.vx, 0.3) << "自转超时不该影响导航平移";
  EXPECT_DOUBLE_EQ(out.wz, 0.0) << "自转超时必须停转";
  EXPECT_TRUE(out.wz_from_spin);
}

/// \brief 决策层的 wz 仍受 wz_max 钳位（超限不报错）。
TEST(VelocityMix, SpinRequestIsClamped)
{
  MixLimits limits = defaultLimits();
  limits.wz_max = 1.0;
  MixInput input;
  input.spin_enabled = true;
  input.spin_valid = true;
  input.spin_wz = -2.5;
  const auto out = mixVelocity(input, limits);
  EXPECT_DOUBLE_EQ(out.wz, -1.0);
  EXPECT_TRUE(out.wz_clamped);
}

/// \brief 单输入模式（spin_enabled=false）：行为与改造前逐字一致。
TEST(VelocityMix, SingleInputKeepsNavigationOwnership)
{
  MixInput input;
  input.nav_vx = 0.1;
  input.nav_wz = 0.6;
  input.spin_enabled = false;
  input.spin_wz = 2.0;            // 单输入下根本不看它
  const auto out = mixVelocity(input, defaultLimits());
  EXPECT_DOUBLE_EQ(out.wz, 0.6);
  EXPECT_FALSE(out.wz_from_spin);
  EXPECT_FALSE(out.nav_wz_ignored);
}

// ---------------------------------------------------------------------------
// wz 变化率限制（方案 §4.3：避免阶跃请求冲击底盘）
// ---------------------------------------------------------------------------

TEST(WzSlew, RampsTowardTheTargetAtTheConfiguredRate)
{
  // 4.5 rad/s² × 0.01 s = 0.045
  EXPECT_NEAR(slewLimit(1.0, 0.0, 4.5, 0.01), 0.045, 1e-12);
  EXPECT_NEAR(slewLimit(-1.0, 0.0, 4.5, 0.01), -0.045, 1e-12);
}

TEST(WzSlew, ReachesTheTargetWhenTheStepIsLarge)
{
  EXPECT_DOUBLE_EQ(slewLimit(1.0, 0.98, 4.5, 0.01), 1.0);
}

TEST(WzSlew, DisabledWhenTheRateIsNotPositive)
{
  EXPECT_DOUBLE_EQ(slewLimit(1.0, 0.0, 0.0, 0.01), 1.0);
  EXPECT_DOUBLE_EQ(slewLimit(1.0, 0.0, -1.0, 0.01), 1.0);
}

TEST(WzSlew, NonPositiveDeltaTimeMeansImmediate)
{
  EXPECT_DOUBLE_EQ(slewLimit(1.0, 0.0, 4.5, 0.0), 1.0);
  EXPECT_DOUBLE_EQ(slewLimit(1.0, 0.0, 4.5, -0.01), 1.0);
}

TEST(WzSlew, NonFiniteTargetKeepsThePreviousValue)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_DOUBLE_EQ(slewLimit(nan, 0.3, 4.5, 0.01), 0.3);
}

TEST(WzSlew, NonZeroRateDetection)
{
  EXPECT_FALSE(isNonZeroRate(0.0));
  EXPECT_FALSE(isNonZeroRate(1e-12));
  EXPECT_TRUE(isNonZeroRate(-0.001));
  EXPECT_TRUE(isNonZeroRate(std::numeric_limits<double>::quiet_NaN()));
}
