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

#include <chrono>
#include <limits>
#include <string>

#include "srm27_chassis_control/rotation_waveform.hpp"
#include "srm27_chassis_control/twist_watchdog.hpp"

using srm27_chassis_control::applyAngularAccelLimit;
using srm27_chassis_control::computeRotationVelocity;
using srm27_chassis_control::parseRotationMode;
using srm27_chassis_control::RotationMode;
using srm27_chassis_control::RotationWaveformParams;
using srm27_chassis_control::TwistWatchdog;
using srm27_chassis_control::validateWaveform;

namespace
{
constexpr double kPi = 3.14159265358979323846;

RotationWaveformParams sineParams()
{
  RotationWaveformParams params;
  params.mode = RotationMode::kPeriodic;
  params.offset = 1.0;
  params.amplitude = 0.5;
  params.period = 4.0;
  params.phase = 0.0;
  params.sine_wave = true;
  return params;
}
}  // namespace

TEST(RotationWaveform, ParsesModes)
{
  RotationMode mode = RotationMode::kPeriodic;
  EXPECT_TRUE(parseRotationMode("stop", mode));
  EXPECT_EQ(mode, RotationMode::kStop);
  EXPECT_TRUE(parseRotationMode("constant", mode));
  EXPECT_EQ(mode, RotationMode::kConstant);
  EXPECT_TRUE(parseRotationMode("periodic", mode));
  EXPECT_EQ(mode, RotationMode::kPeriodic);
  EXPECT_FALSE(parseRotationMode("spin", mode));
}

TEST(RotationWaveform, StopModeAlwaysZero)
{
  RotationWaveformParams params;
  params.mode = RotationMode::kStop;
  params.angular_speed = 1.5;
  EXPECT_DOUBLE_EQ(computeRotationVelocity(params, 0.0), 0.0);
  EXPECT_DOUBLE_EQ(computeRotationVelocity(params, 12.3), 0.0);
}

TEST(RotationWaveform, ConstantModeFollowsSignedSpeed)
{
  RotationWaveformParams params;
  params.mode = RotationMode::kConstant;
  params.angular_speed = -0.8;
  EXPECT_DOUBLE_EQ(computeRotationVelocity(params, 0.0), -0.8);
  EXPECT_DOUBLE_EQ(computeRotationVelocity(params, 7.0), -0.8);
}

TEST(RotationWaveform, SineWaveMatchesFormula)
{
  const auto params = sineParams();
  EXPECT_NEAR(computeRotationVelocity(params, 0.0), 1.0, 1e-12);
  EXPECT_NEAR(computeRotationVelocity(params, 1.0), 1.5, 1e-12);
  EXPECT_NEAR(computeRotationVelocity(params, 2.0), 1.0, 1e-12);
  EXPECT_NEAR(computeRotationVelocity(params, 3.0), 0.5, 1e-12);
}

TEST(RotationWaveform, SineWavePhaseShiftIsApplied)
{
  auto params = sineParams();
  params.phase = kPi / 2.0;
  EXPECT_NEAR(computeRotationVelocity(params, 0.0), 1.5, 1e-12);
}

// offset = 0 时为正负换向，offset > amplitude 时为同方向周期变速。
TEST(RotationWaveform, OffsetZeroGivesReversal)
{
  auto params = sineParams();
  params.offset = 0.0;
  EXPECT_NEAR(computeRotationVelocity(params, 1.0), 0.5, 1e-12);
  EXPECT_NEAR(computeRotationVelocity(params, 3.0), -0.5, 1e-12);
}

TEST(RotationWaveform, SquareWaveSwitchesBetweenOffsetPlusMinusAmplitude)
{
  auto params = sineParams();
  params.sine_wave = false;
  EXPECT_DOUBLE_EQ(computeRotationVelocity(params, 0.0), 1.5);
  EXPECT_DOUBLE_EQ(computeRotationVelocity(params, 1.9), 1.5);
  EXPECT_DOUBLE_EQ(computeRotationVelocity(params, 2.1), 0.5);
  EXPECT_DOUBLE_EQ(computeRotationVelocity(params, 3.9), 0.5);
  EXPECT_DOUBLE_EQ(computeRotationVelocity(params, 4.1), 1.5);
}

TEST(RotationWaveform, RejectsInvalidParameters)
{
  std::string error;
  const double nan = std::numeric_limits<double>::quiet_NaN();

  auto params = sineParams();
  params.angular_speed = nan;
  EXPECT_FALSE(validateWaveform(params, &error));
  EXPECT_FALSE(error.empty());

  params = sineParams();
  params.amplitude = -0.1;
  EXPECT_FALSE(validateWaveform(params, &error));

  params = sineParams();
  params.period = 0.0;
  EXPECT_FALSE(validateWaveform(params, &error));

  params = sineParams();
  params.offset = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(validateWaveform(params, &error));

  EXPECT_TRUE(validateWaveform(sineParams(), &error));
}

// 非周期模式下 period 不参与计算，因此不要求为正。
TEST(RotationWaveform, ConstantModeIgnoresPeriod)
{
  RotationWaveformParams params;
  params.mode = RotationMode::kConstant;
  params.angular_speed = 1.0;
  params.period = -1.0;
  std::string error;
  EXPECT_TRUE(validateWaveform(params, &error));
}

TEST(RotationWaveform, StopCommandBypassesAccelerationLimit)
{
  EXPECT_DOUBLE_EQ(applyAngularAccelLimit(0.0, 1.5, 0.005, 3.0), 0.0);
}

TEST(RotationWaveform, AccelerationLimitRampsTowardsTarget)
{
  // 上限 3 rad/s^2、步长 0.1 s -> 每步最多变化 0.3 rad/s
  EXPECT_NEAR(applyAngularAccelLimit(2.0, 0.0, 0.1, 3.0), 0.3, 1e-12);
  EXPECT_NEAR(applyAngularAccelLimit(2.0, 1.9, 0.1, 3.0), 2.0, 1e-12);
  EXPECT_NEAR(applyAngularAccelLimit(-2.0, 0.0, 0.1, 3.0), -0.3, 1e-12);
}

TEST(RotationWaveform, AccelerationLimitDisabledWhenNotPositive)
{
  EXPECT_DOUBLE_EQ(applyAngularAccelLimit(1.4, 0.0, 0.005, 0.0), 1.4);
  EXPECT_DOUBLE_EQ(applyAngularAccelLimit(1.4, 0.0, 0.0, 3.0), 1.4);
}

TEST(TwistWatchdog, ExpiredWithoutAnyInput)
{
  TwistWatchdog watchdog(0.5);
  EXPECT_FALSE(watchdog.hasInput());
  EXPECT_TRUE(watchdog.expired(TwistWatchdog::TimePoint{}));
}

TEST(TwistWatchdog, TimeoutIsMeasuredFromLastInput)
{
  TwistWatchdog watchdog(0.5);
  const auto start = TwistWatchdog::TimePoint{};
  watchdog.markInput(start);
  EXPECT_TRUE(watchdog.hasInput());
  EXPECT_FALSE(watchdog.expired(start + std::chrono::milliseconds(400)));
  EXPECT_FALSE(watchdog.expired(start + std::chrono::milliseconds(500)));
  EXPECT_TRUE(watchdog.expired(start + std::chrono::milliseconds(501)));
  // 新输入重新计时，不允许沿用旧的时间戳。
  watchdog.markInput(start + std::chrono::milliseconds(600));
  EXPECT_FALSE(watchdog.expired(start + std::chrono::milliseconds(900)));
}

TEST(TwistWatchdog, ResetClearsPreviousInput)
{
  TwistWatchdog watchdog(0.5);
  const auto start = TwistWatchdog::TimePoint{};
  watchdog.markInput(start);
  EXPECT_FALSE(watchdog.expired(start));
  watchdog.reset();
  EXPECT_FALSE(watchdog.hasInput());
  EXPECT_TRUE(watchdog.expired(start));
}

TEST(TwistWatchdog, NonFiniteTimeoutFallsBackToImmediate)
{
  TwistWatchdog watchdog(0.5);
  watchdog.setTimeout(std::numeric_limits<double>::quiet_NaN());
  EXPECT_DOUBLE_EQ(watchdog.timeout(), 0.0);
  watchdog.markInput(TwistWatchdog::TimePoint{});
  EXPECT_TRUE(watchdog.expired(TwistWatchdog::TimePoint{} + std::chrono::seconds(1)));
}
