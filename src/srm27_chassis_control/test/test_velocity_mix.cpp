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
using srm27_chassis_control::mixVelocity;

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
