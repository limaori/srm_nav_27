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

#include "srm27_chassis_control/twist_watchdog.hpp"

using srm27_chassis_control::TwistWatchdog;

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
