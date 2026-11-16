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

/// \brief 方案 §6.1：yaw 外推改用**实测转速**（云台陀螺仪 / Mid-360 IMU）之后，
/// 积分与"断流按 0 处理"的行为回归。

#include <gtest/gtest.h>

#include <vector>

#include "fake_vel_transform/rate_integration.hpp"

using fake_vel_transform::integrateMeasuredRate;
using fake_vel_transform::RateSample;

namespace
{
/// \brief 100 Hz、恒速 `_rate` 的样本序列，覆盖 [0, _duration]。
std::vector<RateSample> constantRate(double _rate, double _duration, double _dt = 0.01)
{
  std::vector<RateSample> samples;
  for (double t = 0.0; t <= _duration + 1e-9; t += _dt) {
    samples.push_back({t, _rate});
  }
  return samples;
}
}  // namespace

/// \brief 恒速积分：∫ 0.5 rad/s dt over 0.2 s = 0.1 rad。
TEST(RateIntegration, ConstantRateIntegratesOverTheWindow)
{
  bool partial = true;
  const auto samples = constantRate(0.5, 1.0);
  const double integral = integrateMeasuredRate(samples, 0.3, 0.5, 0.1, partial);
  EXPECT_NEAR(integral, 0.1, 1e-9);
  EXPECT_FALSE(partial);
}

/// \brief 起停剖面：前半 0（0~0.2 s）、后半 1 rad/s（0.2~0.4 s），区间 0~0.4 s
///        的积分应为 0.2 rad。
TEST(RateIntegration, StepProfileUsesZeroOrderHold)
{
  std::vector<RateSample> samples;
  for (double t = 0.0; t <= 0.2 + 1e-9; t += 0.01) {
    samples.push_back({t, 0.0});
  }
  for (double t = 0.21; t <= 0.4 + 1e-9; t += 0.01) {
    samples.push_back({t, 1.0});
  }
  bool partial = true;
  const double integral = integrateMeasuredRate(samples, 0.0, 0.4, 0.1, partial);
  EXPECT_NEAR(integral, 0.2, 0.02);
  EXPECT_FALSE(partial);
}

/// \brief 断流：最新样本比区间右端老超过门限 -> 推算项按 0（方案 §4.4.2）。
TEST(RateIntegration, StaleStreamReturnsZero)
{
  bool partial = false;
  const auto samples = constantRate(1.0, 0.2);   // 最新样本在 0.2 s
  // 区间右端 0.5 s，门限 0.1 s -> 断流 0.3 s > 0.1 s
  const double integral = integrateMeasuredRate(samples, 0.0, 0.5, 0.1, partial);
  EXPECT_DOUBLE_EQ(integral, 0.0);
  EXPECT_TRUE(partial) << "断流必须被标成 partial（只计数告警，不停止输出）";
}

/// \brief 区间右端超出最后一条样本但没超门限：只积分到"最后样本 + 门限"。
TEST(RateIntegration, TailIsClampedToTheStaleLimit)
{
  bool partial = false;
  const auto samples = constantRate(1.0, 0.2);   // 最后样本 0.20
  // 区间 [0, 0.28]，门限 0.1 -> 只算到 0.28（没超 0.30），全程覆盖
  double integral = integrateMeasuredRate(samples, 0.0, 0.28, 0.1, partial);
  EXPECT_NEAR(integral, 0.28, 1e-9);
  EXPECT_FALSE(partial);

  // 区间 [0, 0.5] 会先被"断流"判掉；构造一个刚好在门限边缘的区间 [0, 0.295]
  integral = integrateMeasuredRate(samples, 0.0, 0.295, 0.1, partial);
  EXPECT_NEAR(integral, 0.295, 1e-9);
  EXPECT_FALSE(partial);
}

/// \brief 左端没有覆盖（最早样本晚于 from）：从最早样本开始算并标记 partial。
TEST(RateIntegration, LeftEdgeNotCoveredIsPartial)
{
  bool partial = false;
  const auto samples = constantRate(1.0, 1.0);   // 最早样本在 0
  const double integral = integrateMeasuredRate(samples, -0.2, 0.1, 0.1, partial);
  EXPECT_NEAR(integral, 0.1, 1e-9);
  EXPECT_TRUE(partial) << "左端没覆盖时必须标记 partial（宁可少算，不要瞎猜）";
}

/// \brief 空样本：按 0，且标记 partial。
TEST(RateIntegration, EmptySamplesAreZero)
{
  bool partial = false;
  const double integral = integrateMeasuredRate({}, 0.0, 0.1, 0.1, partial);
  EXPECT_DOUBLE_EQ(integral, 0.0);
  EXPECT_TRUE(partial);
}

/// \brief 区间退化（to <= from）不产生任何推算量。
TEST(RateIntegration, DegenerateWindowIsZero)
{
  bool partial = true;
  const auto samples = constantRate(2.0, 1.0);
  EXPECT_DOUBLE_EQ(integrateMeasuredRate(samples, 0.5, 0.5, 0.1, partial), 0.0);
  EXPECT_FALSE(partial);
  EXPECT_DOUBLE_EQ(integrateMeasuredRate(samples, 0.5, 0.4, 0.1, partial), 0.0);
  EXPECT_FALSE(partial);
}

/// \brief 门限 <= 0 等价于"不许有任何尾段间隔"：最新样本没盖到区间右端就按 0。
///
/// 方案 §4.4.2 的语义是"断流就丢掉这一项"，而不是"拿最后一条样本继续外推" ——
/// 后者在云台停转/串口掉线时会把一个过期的转速当成常数积分下去。
TEST(RateIntegration, NonPositiveTimeoutReturnsZeroWhenTheTailIsUncovered)
{
  bool partial = false;
  std::vector<RateSample> samples{{0.0, 1.0}, {0.1, 1.0}};
  const double integral = integrateMeasuredRate(samples, 0.0, 0.2, 0.0, partial);
  EXPECT_DOUBLE_EQ(integral, 0.0);
  EXPECT_TRUE(partial);

  // 右端正好落在最后一条样本上时不算断流，正常积分。
  const double exact = integrateMeasuredRate(samples, 0.0, 0.1, 0.0, partial);
  EXPECT_NEAR(exact, 0.1, 1e-9);
  EXPECT_FALSE(partial);
}

/// \brief 负转速：积分带符号（云台逆时针为正是约定，方向反了由实测符号修正）。
TEST(RateIntegration, NegativeRateIntegratesBackwards)
{
  bool partial = false;
  const auto samples = constantRate(-1.5, 1.0);
  const double integral = integrateMeasuredRate(samples, 0.0, 0.4, 0.1, partial);
  EXPECT_NEAR(integral, -0.6, 1e-9);
}
