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

#include "srm27_minco_core/smooth_cost.hpp"

namespace
{

/// \brief 中心差分使用的步长，与实施方案 §11.1 的约定一致。
const double kCentralDifferenceStep = 1e-6;

/// \brief 相对误差；期望值接近 0 时退化为绝对误差，避免除零放大噪声。
double relativeError(const double _expected, const double _actual)
{
  if (std::abs(_expected) > 1e-12) {
    return std::abs(_actual - _expected) / std::abs(_expected);
  }
  return std::abs(_actual - _expected);
}

/// \brief 用中心差分估计 softplusHinge 对 h 的导数。
/// \return 数值导数；若函数调用失败则返回 0（同时由 EXPECT_TRUE 报错）。
double numericHingeDerivative(const double _h, const double _beta)
{
  double value_plus = 0.0;
  double value_minus = 0.0;
  double derivative_unused = 0.0;
  EXPECT_TRUE(srm27_minco_core::softplusHinge(
    _h + kCentralDifferenceStep, _beta, value_plus, derivative_unused));
  EXPECT_TRUE(srm27_minco_core::softplusHinge(
    _h - kCentralDifferenceStep, _beta, value_minus, derivative_unused));
  return (value_plus - value_minus) / (2.0 * kCentralDifferenceStep);
}

/// \brief 用中心差分估计 timeRatioPenalty 对 ratio 的导数。
/// \return 数值导数；若函数调用失败则返回 0（同时由 EXPECT_TRUE 报错）。
double numericTimeRatioDerivative(const double _ratio, const double _low, const double _high)
{
  double value_plus = 0.0;
  double value_minus = 0.0;
  double derivative_unused = 0.0;
  EXPECT_TRUE(srm27_minco_core::timeRatioPenalty(
    _ratio + kCentralDifferenceStep, _low, _high, value_plus, derivative_unused));
  EXPECT_TRUE(srm27_minco_core::timeRatioPenalty(
    _ratio - kCentralDifferenceStep, _low, _high, value_minus, derivative_unused));
  return (value_plus - value_minus) / (2.0 * kCentralDifferenceStep);
}

}  // namespace

// ---------------------------------------------------------------------------
// stableSigmoid
// ---------------------------------------------------------------------------

/// \brief z = 0 时必须精确返回 0.5。
TEST(StableSigmoid, ZeroValue_IsHalf)
{
  EXPECT_DOUBLE_EQ(srm27_minco_core::stableSigmoid(0.0), 0.5);
  EXPECT_DOUBLE_EQ(srm27_minco_core::stableSigmoid(-0.0), 0.5);
}

/// \brief 大正数趋近 1、大负数趋近 0，且两端都不产生 NaN/inf。
TEST(StableSigmoid, SaturatesForLargeAbsZ)
{
  EXPECT_DOUBLE_EQ(srm27_minco_core::stableSigmoid(1000.0), 1.0);
  EXPECT_DOUBLE_EQ(srm27_minco_core::stableSigmoid(-1000.0), 0.0);
  EXPECT_TRUE(std::isfinite(srm27_minco_core::stableSigmoid(1000.0)));
  EXPECT_TRUE(std::isfinite(srm27_minco_core::stableSigmoid(-1000.0)));
  EXPECT_FALSE(std::isnan(srm27_minco_core::stableSigmoid(1000.0)));
  EXPECT_FALSE(std::isnan(srm27_minco_core::stableSigmoid(-1000.0)));

  // 中等幅值下的饱和趋势：z 越大越接近 1，越小越接近 0。
  EXPECT_NEAR(srm27_minco_core::stableSigmoid(30.0), 1.0, 1e-9);
  EXPECT_NEAR(srm27_minco_core::stableSigmoid(-30.0), 0.0, 1e-9);
  EXPECT_GT(srm27_minco_core::stableSigmoid(20.0), srm27_minco_core::stableSigmoid(10.0));
  EXPECT_GT(srm27_minco_core::stableSigmoid(-10.0), srm27_minco_core::stableSigmoid(-20.0));
}

/// \brief 中等 z 时必须与朴素公式 1/(1+exp(-z)) 数值一致。
TEST(StableSigmoid, MatchesNaiveFormulaForModerateZ)
{
  for (double z = -20.0; z <= 20.0; z += 0.25) {
    const double naive = 1.0 / (1.0 + std::exp(-z));
    EXPECT_NEAR(srm27_minco_core::stableSigmoid(z), naive, 1e-15) << "z = " << z;
  }
}

/// \brief 值域必须落在 [0, 1] 且满足对称性 s(z) + s(-z) = 1。
TEST(StableSigmoid, RangeAndSymmetry)
{
  for (double z = -40.0; z <= 40.0; z += 0.5) {
    const double positive = srm27_minco_core::stableSigmoid(z);
    const double negative = srm27_minco_core::stableSigmoid(-z);
    EXPECT_GE(positive, 0.0) << "z = " << z;
    EXPECT_LE(positive, 1.0) << "z = " << z;
    EXPECT_NEAR(positive + negative, 1.0, 1e-15) << "z = " << z;
  }
}

// ---------------------------------------------------------------------------
// softplus
// ---------------------------------------------------------------------------

/// \brief z = 0 时必须返回 log(2)，即 log(1 + exp(0))。
TEST(Softplus, ZeroValue_IsLogTwo)
{
  EXPECT_NEAR(srm27_minco_core::softplus(0.0), std::log(2.0), 1e-15);
  EXPECT_NEAR(srm27_minco_core::softplus(-0.0), std::log(2.0), 1e-15);
}

/// \brief 大 z 时 ≈ z，大负 z 时 ≈ 0，且两端都必须有限（无溢出）。
TEST(Softplus, SaturatesForLargeAbsZ)
{
  // 大正数：softplus(z) - z = log1p(exp(-z)) -> 0。
  EXPECT_NEAR(srm27_minco_core::softplus(50.0), 50.0, 1e-12);
  EXPECT_NEAR(srm27_minco_core::softplus(1000.0), 1000.0, 1e-9);
  EXPECT_TRUE(std::isfinite(srm27_minco_core::softplus(1000.0)));

  // 大负数：softplus(z) -> 0。
  EXPECT_NEAR(srm27_minco_core::softplus(-50.0), 0.0, 1e-12);
  EXPECT_NEAR(srm27_minco_core::softplus(-1000.0), 0.0, 1e-9);
  EXPECT_TRUE(std::isfinite(srm27_minco_core::softplus(-1000.0)));

  // 极值处不得出现 NaN/inf。
  EXPECT_FALSE(std::isnan(srm27_minco_core::softplus(1000.0)));
  EXPECT_FALSE(std::isnan(srm27_minco_core::softplus(-1000.0)));
  EXPECT_NE(std::abs(srm27_minco_core::softplus(1000.0)), std::numeric_limits<double>::infinity());
  EXPECT_NE(std::abs(srm27_minco_core::softplus(-1000.0)), std::numeric_limits<double>::infinity());
}

/// \brief 中等 z 时必须与朴素公式 log(1 + exp(z)) 数值一致。
TEST(Softplus, MatchesNaiveFormulaForModerateZ)
{
  for (double z = -20.0; z <= 20.0; z += 0.25) {
    const double naive = std::log(1.0 + std::exp(z));
    EXPECT_NEAR(srm27_minco_core::softplus(z), naive, 1e-12) << "z = " << z;
  }
}

// ---------------------------------------------------------------------------
// softplusInverse
// ---------------------------------------------------------------------------

/// \brief softplus(softplusInverse(y)) 必须还原 y。
TEST(SoftplusInverse, RoundTripWithSoftplus)
{
  const double values[] = {1e-6, 0.01, 0.1, 1.0, 5.0, 30.0, 200.0};
  for (const double y : values) {
    const double inverse = srm27_minco_core::softplusInverse(y);
    const double round_trip = srm27_minco_core::softplus(inverse);
    EXPECT_TRUE(std::isfinite(inverse)) << "y = " << y;
    // 超大 y 走渐近分支，允许放宽到 1e-6。
    const double tolerance = (y > 30.0) ? 1e-6 : 1e-9;
    EXPECT_NEAR(round_trip / y, 1.0, tolerance) << "y = " << y;
  }
}

/// \brief y <= 30 时实现必须等价于 log(expm1(y))。
TEST(SoftplusInverse, MatchesLogExpm1ForSmallY)
{
  const double values[] = {1e-6, 0.01, 0.1, 1.0, 5.0, 30.0};
  for (const double y : values) {
    const double expected = std::log(std::expm1(y));
    EXPECT_NEAR(srm27_minco_core::softplusInverse(y) / expected, 1.0, 1e-12) << "y = " << y;
  }
  // 逆函数必须单调递增。
  EXPECT_LT(srm27_minco_core::softplusInverse(0.5), srm27_minco_core::softplusInverse(0.6));
}

/// \brief y <= 0 或非有限时返回有限哨兵值 -40，调用方据此判断不可用。
TEST(SoftplusInverse, NonPositiveOrNonFinite_ReturnsSentinel)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double infinity = std::numeric_limits<double>::infinity();
  EXPECT_DOUBLE_EQ(srm27_minco_core::softplusInverse(0.0), -40.0);
  EXPECT_DOUBLE_EQ(srm27_minco_core::softplusInverse(-0.0), -40.0);
  EXPECT_DOUBLE_EQ(srm27_minco_core::softplusInverse(-1e-300), -40.0);
  EXPECT_DOUBLE_EQ(srm27_minco_core::softplusInverse(-1.0), -40.0);
  EXPECT_DOUBLE_EQ(srm27_minco_core::softplusInverse(-1e300), -40.0);
  EXPECT_DOUBLE_EQ(srm27_minco_core::softplusInverse(nan), -40.0);
  EXPECT_DOUBLE_EQ(srm27_minco_core::softplusInverse(infinity), -40.0);
  EXPECT_DOUBLE_EQ(srm27_minco_core::softplusInverse(-infinity), -40.0);
  EXPECT_TRUE(std::isfinite(srm27_minco_core::softplusInverse(-1.0)));
}

// ---------------------------------------------------------------------------
// softplusHinge
// ---------------------------------------------------------------------------

/// \brief 值与解析导数必须同时成立：解析导数用中心差分交叉验证。
TEST(SoftplusHinge, ValueAndDerivativeMatchCentralDifference)
{
  const double betas[] = {1.0, 20.0};
  const double offsets[] = {-2.0, -0.5, -0.01, 0.01, 0.5, 2.0};
  for (const double beta : betas) {
    for (const double h : offsets) {
      double value = 0.0;
      double derivative = 0.0;
      ASSERT_TRUE(srm27_minco_core::softplusHinge(h, beta, value, derivative));
      const double numeric = numericHingeDerivative(h, beta);
      EXPECT_LT(relativeError(derivative, numeric), 1e-5) << "beta = " << beta << ", h = " << h;
      // 导数必须落在 [0, 1] 区间内（sigmoid 值域）。
      EXPECT_GE(derivative, 0.0) << "beta = " << beta << ", h = " << h;
      EXPECT_LE(derivative, 1.0) << "beta = " << beta << ", h = " << h;
      // 值必须非负，并且当 h <= 0 时不超过 softplus 的界。
      EXPECT_GE(value, 0.0) << "beta = " << beta << ", h = " << h;
      EXPECT_TRUE(std::isfinite(value));
    }
  }
}

/// \brief 值与解析式 softplus(beta*h)/beta 在中等 z 处一致。
TEST(SoftplusHinge, ValueMatchesNaiveFormulaForModerateZ)
{
  const double betas[] = {1.0, 20.0};
  const double offsets[] = {-1.0, -0.1, 0.0, 0.1, 1.0};
  for (const double beta : betas) {
    for (const double h : offsets) {
      double value = 0.0;
      double derivative = 0.0;
      ASSERT_TRUE(srm27_minco_core::softplusHinge(h, beta, value, derivative));
      const double expected = std::log1p(std::exp(beta * h)) / beta;
      EXPECT_NEAR(value, expected, 1e-12) << "beta = " << beta << ", h = " << h;
    }
  }
}

/// \brief h 很大时值 ≈ h、导数 ≈ 1；h 很小时值 ≈ 0、导数 ≈ 0。
TEST(SoftplusHinge, SaturatesForLargeAbsH)
{
  double value = 0.0;
  double derivative = 0.0;
  ASSERT_TRUE(srm27_minco_core::softplusHinge(50.0, 1.0, value, derivative));
  EXPECT_NEAR(value, 50.0, 1e-9);
  EXPECT_NEAR(derivative, 1.0, 1e-9);

  ASSERT_TRUE(srm27_minco_core::softplusHinge(-50.0, 1.0, value, derivative));
  EXPECT_NEAR(value, 0.0, 1e-9);
  EXPECT_NEAR(derivative, 0.0, 1e-9);

  // beta 越大越接近硬 hinge：h = 2、beta = 20 时已基本饱和。
  ASSERT_TRUE(srm27_minco_core::softplusHinge(2.0, 20.0, value, derivative));
  EXPECT_NEAR(value, 2.0, 1e-6);
  EXPECT_NEAR(derivative, 1.0, 1e-6);
  ASSERT_TRUE(srm27_minco_core::softplusHinge(-2.0, 20.0, value, derivative));
  EXPECT_NEAR(value, 0.0, 1e-9);
  EXPECT_NEAR(derivative, 0.0, 1e-9);
}

/// \brief beta <= 0 或输入非有限时返回 false，且两个输出都被清零。
TEST(SoftplusHinge, InvalidParameter_ReturnsFalseWithZeroOutputs)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double infinity = std::numeric_limits<double>::infinity();
  const double invalid_betas[] = {0.0, -1e-12, -1.0, nan, infinity, -infinity};
  for (const double beta : invalid_betas) {
    double value = 7.0;
    double derivative = 7.0;
    EXPECT_FALSE(srm27_minco_core::softplusHinge(1.0, beta, value, derivative))
      << "beta = " << beta;
    EXPECT_DOUBLE_EQ(value, 0.0) << "beta = " << beta;
    EXPECT_DOUBLE_EQ(derivative, 0.0) << "beta = " << beta;
  }

  // h 非有限同样非法。
  const double invalid_offsets[] = {nan, infinity, -infinity};
  for (const double h : invalid_offsets) {
    double value = 7.0;
    double derivative = 7.0;
    EXPECT_FALSE(srm27_minco_core::softplusHinge(h, 1.0, value, derivative)) << "h = " << h;
    EXPECT_DOUBLE_EQ(value, 0.0) << "h = " << h;
    EXPECT_DOUBLE_EQ(derivative, 0.0) << "h = " << h;
  }
}

// ---------------------------------------------------------------------------
// timeRatioPenalty
// ---------------------------------------------------------------------------

/// \brief 三种情形（区间内 / 上越界 / 下越界）的值与导数都要与中心差分一致。
TEST(TimeRatioPenalty, ValueAndDerivativeMatchCentralDifference)
{
  const double low = 0.8;
  const double high = 1.25;
  double value = -1.0;
  double derivative = -1.0;

  // 情形 1：ratio 落在允许区间内，惩罚项与其导数都必须是 0。
  ASSERT_TRUE(srm27_minco_core::timeRatioPenalty(1.0, low, high, value, derivative));
  EXPECT_DOUBLE_EQ(value, 0.0);
  EXPECT_DOUBLE_EQ(derivative, 0.0);
  EXPECT_LT(relativeError(derivative, numericTimeRatioDerivative(1.0, low, high)), 1e-5);

  // 情形 2：ratio 超过上界，value = (r-high)^2，导数 = 2*(r-high)。
  ASSERT_TRUE(srm27_minco_core::timeRatioPenalty(1.5, low, high, value, derivative));
  EXPECT_NEAR(value, 0.0625, 1e-15);
  EXPECT_NEAR(derivative, 0.5, 1e-15);
  EXPECT_LT(relativeError(derivative, numericTimeRatioDerivative(1.5, low, high)), 1e-5);

  // 情形 3：ratio 低于下界，value = (low-r)^2，导数 = -2*(low-r)。
  ASSERT_TRUE(srm27_minco_core::timeRatioPenalty(0.5, low, high, value, derivative));
  EXPECT_NEAR(value, 0.09, 1e-15);
  EXPECT_NEAR(derivative, -0.6, 1e-15);
  EXPECT_LT(relativeError(derivative, numericTimeRatioDerivative(0.5, low, high)), 1e-5);
}

/// \brief 上下界本身不产生惩罚，越界程度越大惩罚越大。
TEST(TimeRatioPenalty, BoundariesAndMonotonicity)
{
  const double low = 0.8;
  const double high = 1.25;
  double value = -1.0;
  double derivative = -1.0;
  ASSERT_TRUE(srm27_minco_core::timeRatioPenalty(high, low, high, value, derivative));
  EXPECT_DOUBLE_EQ(value, 0.0);
  EXPECT_DOUBLE_EQ(derivative, 0.0);
  ASSERT_TRUE(srm27_minco_core::timeRatioPenalty(low, low, high, value, derivative));
  EXPECT_DOUBLE_EQ(value, 0.0);
  EXPECT_DOUBLE_EQ(derivative, 0.0);

  // 越界越多，惩罚单调递增。
  double value_far = 0.0;
  ASSERT_TRUE(srm27_minco_core::timeRatioPenalty(1.75, low, high, value_far, derivative));
  EXPECT_GT(value_far, 0.0625);
  ASSERT_TRUE(srm27_minco_core::timeRatioPenalty(0.4, low, high, value_far, derivative));
  EXPECT_GT(value_far, 0.09);
}

/// \brief _low > _high 的空区间必须返回 false，输出清零。
TEST(TimeRatioPenalty, DegenerateInterval_ReturnsFalse)
{
  double value = 9.0;
  double derivative = 9.0;
  EXPECT_FALSE(srm27_minco_core::timeRatioPenalty(1.0, 1.25, 0.8, value, derivative));
  EXPECT_DOUBLE_EQ(value, 0.0);
  EXPECT_DOUBLE_EQ(derivative, 0.0);

  value = 9.0;
  derivative = 9.0;
  EXPECT_FALSE(srm27_minco_core::timeRatioPenalty(0.0, 1.0, 0.0, value, derivative));
  EXPECT_DOUBLE_EQ(value, 0.0);
  EXPECT_DOUBLE_EQ(derivative, 0.0);
}

/// \brief 非有限输入必须返回 false，不得把 NaN 当成合法惩罚值传下去。
TEST(TimeRatioPenalty, NonFiniteInput_ReturnsFalse)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double infinity = std::numeric_limits<double>::infinity();
  double value = 9.0;
  double derivative = 9.0;

  EXPECT_FALSE(srm27_minco_core::timeRatioPenalty(nan, 0.8, 1.25, value, derivative));
  EXPECT_FALSE(srm27_minco_core::timeRatioPenalty(1.0, nan, 1.25, value, derivative));
  EXPECT_FALSE(srm27_minco_core::timeRatioPenalty(1.0, 0.8, nan, value, derivative));
  EXPECT_FALSE(srm27_minco_core::timeRatioPenalty(infinity, 0.8, 1.25, value, derivative));
  EXPECT_FALSE(srm27_minco_core::timeRatioPenalty(1.0, 0.8, infinity, value, derivative));
  EXPECT_FALSE(srm27_minco_core::timeRatioPenalty(1.0, -infinity, 1.25, value, derivative));
  EXPECT_DOUBLE_EQ(value, 0.0);
  EXPECT_DOUBLE_EQ(derivative, 0.0);
}
