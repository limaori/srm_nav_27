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

using srm27_chassis_control::MixInput;
using srm27_chassis_control::MixLimits;
using srm27_chassis_control::MixMode;
using srm27_chassis_control::mixVelocity;
using srm27_chassis_control::parseMixMode;
using srm27_chassis_control::toString;

namespace
{
MixLimits defaultLimits()
{
  MixLimits limits;
  limits.vx_max = 0.5;
  limits.vy_max = 0.5;
  limits.v_max = 0.5;
  limits.wz_max = 2.0;
  return limits;
}
}  // namespace

// 纯 vx 直行：符号与大小原样保留。
TEST(VelocityMix, PureForwardPassesThrough)
{
  MixInput input;
  input.nav_vx = 0.3;
  const auto output = mixVelocity(input, defaultLimits());
  EXPECT_DOUBLE_EQ(output.vx, 0.3);
  EXPECT_DOUBLE_EQ(output.vy, 0.0);
  EXPECT_DOUBLE_EQ(output.scale, 1.0);
  EXPECT_FALSE(output.clamped);
}

// 纯 vy 横移：y 为正表示向左。
TEST(VelocityMix, PureLateralKeepsSign)
{
  MixInput input;
  input.nav_vy = -0.4;
  const auto output = mixVelocity(input, defaultLimits());
  EXPECT_DOUBLE_EQ(output.vx, 0.0);
  EXPECT_DOUBLE_EQ(output.vy, -0.4);
}

// 平移按向量模长限幅，避免对角运动超过总速度上限。
TEST(VelocityMix, DiagonalIsLimitedByMagnitude)
{
  MixInput input;
  input.nav_vx = 0.5;
  input.nav_vy = 0.5;
  const auto output = mixVelocity(input, defaultLimits());
  EXPECT_TRUE(output.clamped);
  EXPECT_NEAR(std::hypot(output.vx, output.vy), 0.5, 1e-12);
  EXPECT_NEAR(output.vx, output.vy, 1e-12);
}

// 单轴先限幅，再按模长限幅。
TEST(VelocityMix, PerAxisClampAppliesFirst)
{
  MixInput input;
  input.nav_vx = 10.0;
  input.nav_vy = 0.0;
  const auto output = mixVelocity(input, defaultLimits());
  EXPECT_DOUBLE_EQ(output.vx, 0.5);
  EXPECT_DOUBLE_EQ(output.vy, 0.0);
}

// 零向量时 scale = 1，不能产生 NaN。
TEST(VelocityMix, ZeroVectorKeepsScaleOne)
{
  MixInput input;
  const auto output = mixVelocity(input, defaultLimits());
  EXPECT_DOUBLE_EQ(output.scale, 1.0);
  EXPECT_DOUBLE_EQ(output.vx, 0.0);
  EXPECT_DOUBLE_EQ(output.vy, 0.0);
  EXPECT_DOUBLE_EQ(output.wz, 0.0);
}

// 自转角速度只来自 rotation_wz，并按 wz_max 限幅。
TEST(VelocityMix, RotationIsClampedIndependently)
{
  MixInput input;
  input.nav_vx = 0.2;
  input.rotation_wz = -5.0;
  const auto output = mixVelocity(input, defaultLimits());
  EXPECT_DOUBLE_EQ(output.vx, 0.2);
  EXPECT_DOUBLE_EQ(output.wz, -2.0);
}

// 导航与自转同时输入时互不干扰：平移不因为自转而改变。
TEST(VelocityMix, TranslationIsIndependentOfRotation)
{
  MixInput input;
  input.nav_vx = 0.25;
  input.nav_vy = 0.1;
  input.rotation_wz = 1.0;
  const auto output = mixVelocity(input, defaultLimits());
  EXPECT_DOUBLE_EQ(output.vx, 0.25);
  EXPECT_DOUBLE_EQ(output.vy, 0.1);
  EXPECT_DOUBLE_EQ(output.wz, 1.0);
}

// NaN / Inf 输入一律按零处理，绝不能传给执行端。
TEST(VelocityMix, NonFiniteInputsBecomeZero)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  MixInput input;
  input.nav_vx = nan;
  input.nav_vy = inf;
  input.rotation_wz = -inf;
  const auto output = mixVelocity(input, defaultLimits());
  EXPECT_DOUBLE_EQ(output.vx, 0.0);
  EXPECT_DOUBLE_EQ(output.vy, 0.0);
  EXPECT_DOUBLE_EQ(output.wz, 0.0);
}

// v_max 非正时不允许平移运动，避免出现不受限的速度。
TEST(VelocityMix, NonPositiveMaxSpeedBlocksTranslation)
{
  MixInput input;
  input.nav_vx = 0.3;
  MixLimits limits = defaultLimits();
  limits.v_max = 0.0;
  const auto output = mixVelocity(input, limits);
  EXPECT_DOUBLE_EQ(output.vx, 0.0);
  EXPECT_DOUBLE_EQ(output.vy, 0.0);
}

// 自转输出与平移限幅解耦：平移被限幅时自转不受影响。
TEST(VelocityMix, RotationSurvivesTranslationLimiting)
{
  MixInput input;
  input.nav_vx = 1.0;
  input.nav_vy = 1.0;
  input.rotation_wz = 1.5;
  const auto output = mixVelocity(input, defaultLimits());
  EXPECT_NEAR(std::hypot(output.vx, output.vy), 0.5, 1e-12);
  EXPECT_DOUBLE_EQ(output.wz, 1.5);
}

// ---------------------------------------------------------------- nav_se2 模式
// 迁移模式：wz 只来自导航，且**绝不**与独立自转相加（方案 §4.3）。
TEST(VelocityMixNavSe2, NavigationOwnsYawRate)
{
  MixInput input;
  input.nav_vx = 0.3;
  input.nav_wz = 0.8;
  // 故意同时塞入独立自转：它必须被完全忽略，否则就是"角速度双重叠加"。
  input.rotation_wz = 1.5;
  const auto output = mixVelocity(input, defaultLimits(), MixMode::kNavSe2);
  EXPECT_DOUBLE_EQ(output.vx, 0.3);
  EXPECT_DOUBLE_EQ(output.wz, 0.8);
  EXPECT_FALSE(output.wz_clamped);
}

// nav_se2 下的导航 wz 同样要按 wz_max 限幅，并给出限幅标志。
TEST(VelocityMixNavSe2, NavigationYawRateIsClamped)
{
  MixInput input;
  input.nav_wz = -7.5;
  const auto output = mixVelocity(input, defaultLimits(), MixMode::kNavSe2);
  EXPECT_DOUBLE_EQ(output.wz, -2.0);
  EXPECT_TRUE(output.wz_clamped);
}

// 旧模式下导航 wz 不得出现在输出里（回归：防止迁移改动把旧链路带坏）。
TEST(VelocityMixNavSe2, LegacyModeIgnoresNavigationYawRate)
{
  MixInput input;
  input.nav_vx = 0.2;
  input.nav_wz = 1.7;
  input.rotation_wz = 0.0;
  const auto output = mixVelocity(input, defaultLimits(), MixMode::kLegacyIndependentRotation);
  EXPECT_DOUBLE_EQ(output.wz, 0.0);
  EXPECT_EQ(output.mode, MixMode::kLegacyIndependentRotation);
}

// 非有限的导航 wz 在 nav_se2 下必须按 0 处理，并标记为被限幅。
TEST(VelocityMixNavSe2, NonFiniteNavigationYawRateBecomesZero)
{
  MixInput input;
  input.nav_wz = std::numeric_limits<double>::quiet_NaN();
  const auto output = mixVelocity(input, defaultLimits(), MixMode::kNavSe2);
  EXPECT_DOUBLE_EQ(output.wz, 0.0);
  EXPECT_TRUE(output.wz_clamped);
}

// wz_max 非正时自转必须被完全禁止（安全兜底）。
TEST(VelocityMixNavSe2, NonPositiveYawLimitBlocksRotation)
{
  MixInput input;
  input.nav_wz = 0.5;
  MixLimits limits = defaultLimits();
  limits.wz_max = 0.0;
  const auto output = mixVelocity(input, limits, MixMode::kNavSe2);
  EXPECT_DOUBLE_EQ(output.wz, 0.0);
  EXPECT_TRUE(output.wz_clamped);
}

// 模式解析：参数用全名，同时接受命令行常用的简写。
TEST(VelocityMixNavSe2, ParseModeRoundTrip)
{
  MixMode mode = MixMode::kNavSe2;
  EXPECT_TRUE(parseMixMode("legacy_independent_rotation", mode));
  EXPECT_EQ(mode, MixMode::kLegacyIndependentRotation);
  EXPECT_TRUE(parseMixMode("nav_se2", mode));
  EXPECT_EQ(mode, MixMode::kNavSe2);
  EXPECT_TRUE(parseMixMode("rotation", mode));
  EXPECT_EQ(mode, MixMode::kLegacyIndependentRotation);
  EXPECT_TRUE(parseMixMode("nav", mode));
  EXPECT_EQ(mode, MixMode::kNavSe2);
  EXPECT_FALSE(parseMixMode("guess_from_wz", mode));
  EXPECT_STREQ(toString(MixMode::kNavSe2), "nav_se2");
  EXPECT_STREQ(toString(MixMode::kLegacyIndependentRotation), "legacy_independent_rotation");
}
