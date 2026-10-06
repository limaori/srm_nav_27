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
#include <string>
#include <vector>

#include "srm27_minco_core/trajectory_2d.hpp"
#include "srm27_minco_core/types.hpp"

namespace
{

// 逐个引入被测类型（不做 `using namespace` 级别的全局展开）。
using srm27_minco_core::Trajectory2D;

/// \brief 全零系数（常数 0 的多项式）。
const Trajectory2D::Coefficients kZero{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

/// \brief 按 c0..c5 顺序解析求 `_order` 阶导数。
///
/// p^(m)(tau) = sum_{k>=m} c_k * k! / (k-m)! * tau^(k-m)。
/// 与 Trajectory2D 内部求值相互独立，用于验证系数顺序（低阶在前）。
double evalDerivative(const Trajectory2D::Coefficients & _c, double _tau, int _order)
{
  double value = 0.0;
  for (int k = Trajectory2D::kCoefficientCount - 1; k >= _order; --k) {
    double factor = 1.0;
    for (int j = 0; j < _order; ++j) {
      factor *= static_cast<double>(k - j);
    }
    value = value * _tau + factor * _c[static_cast<std::size_t>(k)];
  }
  return value;
}

/// \brief 构造段间 p/v/a 连续的 2 段轨迹（时长 1.0 s + 1.5 s）。
Trajectory2D makeContinuousTrajectory()
{
  Trajectory2D trajectory;
  const Trajectory2D::Coefficients x0{0.0, 1.0, 0.5, 0.0, 0.0, 0.0};
  const Trajectory2D::Coefficients y0{1.0, -1.0, 0.25, 0.0, 0.0, 0.0};
  EXPECT_TRUE(trajectory.addPiece(x0, y0, 1.0));
  // 第二段以第一段末端的 p/v/a 作为初值：c0 = p, c1 = v, c2 = a / 2。
  const Trajectory2D::Coefficients x1{
    evalDerivative(x0, 1.0, 0),
    evalDerivative(x0, 1.0, 1),
    evalDerivative(x0, 1.0, 2) / 2.0,
    0.0,
    0.0,
    0.0};
  const Trajectory2D::Coefficients y1{
    evalDerivative(y0, 1.0, 0),
    evalDerivative(y0, 1.0, 1),
    evalDerivative(y0, 1.0, 2) / 2.0,
    0.0,
    0.0,
    0.0};
  EXPECT_TRUE(trajectory.addPiece(x1, y1, 1.5));
  return trajectory;
}

/// \brief 构造“位置在段边界跳变”的轨迹：两段常数多项式，常数项不同。
Trajectory2D makeDiscontinuousTrajectory()
{
  Trajectory2D trajectory;
  Trajectory2D::Coefficients x1 = kZero;
  x1[0] = 1.0;
  EXPECT_TRUE(trajectory.addPiece(kZero, kZero, 1.0));
  EXPECT_TRUE(trajectory.addPiece(x1, kZero, 1.0));
  return trajectory;
}

}  // namespace

// ---------------------------------------------------------------------------
// addPiece / addPieceHighOrderFirst
// ---------------------------------------------------------------------------

TEST(Trajectory2DAddPiece, InvalidArguments_ReturnFalse)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  Trajectory2D trajectory;
  EXPECT_FALSE(trajectory.addPiece(kZero, kZero, 0.0));
  EXPECT_FALSE(trajectory.addPiece(kZero, kZero, -1.0));
  EXPECT_FALSE(trajectory.addPiece(kZero, kZero, nan));
  EXPECT_FALSE(trajectory.addPiece(kZero, kZero, inf));

  Trajectory2D::Coefficients bad_x = kZero;
  bad_x[2] = nan;
  Trajectory2D::Coefficients bad_y = kZero;
  bad_y[0] = inf;
  EXPECT_FALSE(trajectory.addPiece(bad_x, kZero, 1.0));
  EXPECT_FALSE(trajectory.addPiece(kZero, bad_y, 1.0));
  EXPECT_FALSE(trajectory.addPieceHighOrderFirst(kZero, kZero, 0.0));
  EXPECT_FALSE(trajectory.addPieceHighOrderFirst(bad_x, kZero, 1.0));

  // 失败不得改变轨迹。
  EXPECT_TRUE(trajectory.empty());
  EXPECT_EQ(trajectory.pieceCount(), 0);
  EXPECT_DOUBLE_EQ(trajectory.totalDuration(), 0.0);
  EXPECT_TRUE(trajectory.durations().empty());

  EXPECT_TRUE(trajectory.addPiece(kZero, kZero, 1.0));
  EXPECT_EQ(trajectory.pieceCount(), 1);
  EXPECT_FALSE(trajectory.empty());
  EXPECT_DOUBLE_EQ(trajectory.totalDuration(), 1.0);
}

TEST(Trajectory2DAddPiece, HighOrderFirst_EquivalentToAddPiece)
{
  const Trajectory2D::Coefficients x{1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
  const Trajectory2D::Coefficients y{-1.0, 0.5, -2.0, 0.0, 1.0, 0.25};
  Trajectory2D low_order;
  ASSERT_TRUE(low_order.addPiece(x, y, 2.0));

  // 适配器输入顺序是 [c5, c4, c3, c2, c1, c0]。
  Trajectory2D::Coefficients x_high{};
  Trajectory2D::Coefficients y_high{};
  for (int k = 0; k < Trajectory2D::kCoefficientCount; ++k) {
    const std::size_t src = static_cast<std::size_t>(Trajectory2D::kCoefficientCount - 1 - k);
    x_high[static_cast<std::size_t>(k)] = x[src];
    y_high[static_cast<std::size_t>(k)] = y[src];
  }
  Trajectory2D high_order;
  ASSERT_TRUE(high_order.addPieceHighOrderFirst(x_high, y_high, 2.0));

  EXPECT_EQ(low_order.pieceCount(), high_order.pieceCount());
  EXPECT_EQ(low_order.durations(), high_order.durations());
  ASSERT_EQ(low_order.xCoefficients().size(), 1u);
  ASSERT_EQ(high_order.xCoefficients().size(), 1u);
  for (int k = 0; k < Trajectory2D::kCoefficientCount; ++k) {
    const std::size_t index = static_cast<std::size_t>(k);
    EXPECT_DOUBLE_EQ(low_order.xCoefficients()[0][index], x[index]);
    EXPECT_DOUBLE_EQ(low_order.xCoefficients()[0][index], high_order.xCoefficients()[0][index]);
    EXPECT_DOUBLE_EQ(low_order.yCoefficients()[0][index], high_order.yCoefficients()[0][index]);
  }
  EXPECT_EQ(low_order.xCoefficients(), high_order.xCoefficients());
  EXPECT_EQ(low_order.yCoefficients(), high_order.yCoefficients());
  for (const double tau : {0.0, 0.5, 1.0, 2.0}) {
    EXPECT_TRUE(low_order.positionAt(tau).isApprox(high_order.positionAt(tau), 1e-15));
    EXPECT_TRUE(low_order.velocityAt(tau).isApprox(high_order.velocityAt(tau), 1e-15));
  }
}

// ---------------------------------------------------------------------------
// 系数顺序与解析求值往返
// ---------------------------------------------------------------------------

TEST(Trajectory2DEvaluate, CoefficientOrder_RoundTripWithAnalyticDerivatives)
{
  // x(tau) = 1 + 2 tau + 3 tau^2 + 4 tau^3 + 5 tau^4 + 6 tau^5
  const Trajectory2D::Coefficients x{1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
  const Trajectory2D::Coefficients y{-2.0, 0.5, 0.0, -0.25, 0.0, 1.0};
  Trajectory2D trajectory;
  ASSERT_TRUE(trajectory.addPiece(x, y, 2.0));
  ASSERT_EQ(trajectory.pieceCount(), 1);

  // 顺序往返：xCoefficients()[i][k] 必须就是 tau^k 的系数（低阶在前）。
  ASSERT_EQ(trajectory.xCoefficients().size(), 1u);
  ASSERT_EQ(trajectory.yCoefficients().size(), 1u);
  for (int k = 0; k < Trajectory2D::kCoefficientCount; ++k) {
    const std::size_t index = static_cast<std::size_t>(k);
    EXPECT_DOUBLE_EQ(trajectory.xCoefficients()[0][index], x[index]) << "k = " << k;
    EXPECT_DOUBLE_EQ(trajectory.yCoefficients()[0][index], y[index]) << "k = " << k;
  }

  for (const double tau : {0.0, 0.25, 0.5, 1.0, 1.75, 2.0}) {
    EXPECT_NEAR(trajectory.positionAt(tau).x(), evalDerivative(x, tau, 0), 1e-12) << "tau=" << tau;
    EXPECT_NEAR(trajectory.velocityAt(tau).x(), evalDerivative(x, tau, 1), 1e-12) << "tau=" << tau;
    EXPECT_NEAR(trajectory.accelerationAt(tau).x(), evalDerivative(x, tau, 2), 1e-12)
      << "tau=" << tau;
    EXPECT_NEAR(trajectory.jerkAt(tau).x(), evalDerivative(x, tau, 3), 1e-12) << "tau=" << tau;
    EXPECT_NEAR(trajectory.positionAt(tau).y(), evalDerivative(y, tau, 0), 1e-12) << "tau=" << tau;
    EXPECT_NEAR(trajectory.velocityAt(tau).y(), evalDerivative(y, tau, 1), 1e-12) << "tau=" << tau;
    EXPECT_NEAR(trajectory.accelerationAt(tau).y(), evalDerivative(y, tau, 2), 1e-12)
      << "tau=" << tau;
    EXPECT_NEAR(trajectory.jerkAt(tau).y(), evalDerivative(y, tau, 3), 1e-12) << "tau=" << tau;
    // 段内求值与整轨迹求值在同一点必须一致。
    EXPECT_NEAR(trajectory.piecePosition(0, tau).x(), trajectory.positionAt(tau).x(), 1e-15);
    EXPECT_NEAR(trajectory.pieceVelocity(0, tau).y(), trajectory.velocityAt(tau).y(), 1e-15);
  }

  // 手算值（tau = 1，纯整数，便于人工复核）：
  // p = 1+2+3+4+5+6 = 21，p' = 2+6+12+20+30 = 70，
  // p'' = 6+24+60+120 = 210，p''' = 24+120+360 = 504。
  EXPECT_NEAR(trajectory.positionAt(1.0).x(), 21.0, 1e-12);
  EXPECT_NEAR(trajectory.velocityAt(1.0).x(), 70.0, 1e-12);
  EXPECT_NEAR(trajectory.accelerationAt(1.0).x(), 210.0, 1e-12);
  EXPECT_NEAR(trajectory.jerkAt(1.0).x(), 504.0, 1e-12);
  // 起点/终点状态也必须取自同一多项式。
  EXPECT_NEAR(trajectory.startPosition().x(), 1.0, 1e-12);
  EXPECT_NEAR(trajectory.startVelocity().x(), 2.0, 1e-12);
  EXPECT_NEAR(trajectory.startAcceleration().x(), 6.0, 1e-12);
  EXPECT_NEAR(trajectory.endPosition().x(), evalDerivative(x, 2.0, 0), 1e-12);
  EXPECT_NEAR(trajectory.endVelocity().x(), evalDerivative(x, 2.0, 1), 1e-12);
  EXPECT_NEAR(trajectory.endAcceleration().x(), evalDerivative(x, 2.0, 2), 1e-12);
}

TEST(Trajectory2DEvaluate, TimeUnitIsSeconds_NotMilliseconds)
{
  // c1 = 2 m/s，段时长 2 s：1 s 内应前进 2 m，而不是 0.002 m（毫秒错误）。
  const Trajectory2D::Coefficients x{0.0, 2.0, 0.0, 0.0, 0.0, 0.0};
  const Trajectory2D::Coefficients y{1.5, 0.0, 0.0, 0.0, 0.0, 0.0};
  Trajectory2D trajectory;
  ASSERT_TRUE(trajectory.addPiece(x, y, 2.0));
  EXPECT_DOUBLE_EQ(trajectory.totalDuration(), 2.0);
  EXPECT_NEAR(trajectory.positionAt(1.0).x() - trajectory.positionAt(0.0).x(), 2.0, 1e-12);
  EXPECT_NEAR(trajectory.positionAt(2.0).x(), 4.0, 1e-12);
  EXPECT_NEAR(trajectory.startVelocity().x(), 2.0, 1e-12);
  EXPECT_DOUBLE_EQ(trajectory.positionAt(0.5).y(), 1.5);
  EXPECT_DOUBLE_EQ(trajectory.positionAt(2.0).y(), 1.5);
}

TEST(Trajectory2DPiece, TauClampedToPieceDuration)
{
  const Trajectory2D::Coefficients x{0.0, 2.0, 1.0, 0.0, 0.0, 0.0};
  Trajectory2D trajectory;
  ASSERT_TRUE(trajectory.addPiece(x, kZero, 1.0));
  EXPECT_TRUE(trajectory.piecePosition(0, 5.0).isApprox(trajectory.piecePosition(0, 1.0), 1e-15));
  EXPECT_TRUE(trajectory.piecePosition(0, -3.0).isApprox(trajectory.piecePosition(0, 0.0), 1e-15));
  EXPECT_TRUE(trajectory.pieceVelocity(0, -3.0).isApprox(trajectory.pieceVelocity(0, 0.0), 1e-15));
  // 非法段索引返回零而不是崩溃。
  EXPECT_TRUE(trajectory.piecePosition(-1, 0.0).isApprox(Eigen::Vector2d::Zero(), 0.0));
  EXPECT_TRUE(trajectory.piecePosition(3, 0.0).isApprox(Eigen::Vector2d::Zero(), 0.0));
  EXPECT_TRUE(trajectory.pieceVelocity(3, 0.0).isApprox(Eigen::Vector2d::Zero(), 0.0));
  EXPECT_TRUE(trajectory.pieceAcceleration(-2, 0.0).isApprox(Eigen::Vector2d::Zero(), 0.0));
  EXPECT_TRUE(trajectory.pieceJerk(9, 0.0).isApprox(Eigen::Vector2d::Zero(), 0.0));
}

// ---------------------------------------------------------------------------
// locate / clampTime
// ---------------------------------------------------------------------------

TEST(Trajectory2DLocate, PieceBoundary_BelongsToNextPiece)
{
  Trajectory2D trajectory;
  ASSERT_TRUE(trajectory.addPiece(kZero, kZero, 1.0));
  ASSERT_TRUE(trajectory.addPiece(kZero, kZero, 2.0));
  EXPECT_DOUBLE_EQ(trajectory.totalDuration(), 3.0);

  int index = -1;
  double tau = -1.0;
  trajectory.locate(0.0, index, tau);
  EXPECT_EQ(index, 0);
  EXPECT_NEAR(tau, 0.0, 1e-15);
  trajectory.locate(0.5, index, tau);
  EXPECT_EQ(index, 0);
  EXPECT_NEAR(tau, 0.5, 1e-15);
  // 恰好落在段边界：归入后一段，tau 约为 0。
  trajectory.locate(1.0, index, tau);
  EXPECT_EQ(index, 1);
  EXPECT_NEAR(tau, 0.0, 1e-15);
  trajectory.locate(2.5, index, tau);
  EXPECT_EQ(index, 1);
  EXPECT_NEAR(tau, 1.5, 1e-15);
  // 末端归入最后一段。
  trajectory.locate(3.0, index, tau);
  EXPECT_EQ(index, 1);
  EXPECT_NEAR(tau, 2.0, 1e-15);
  // 越界时间先被裁剪。
  trajectory.locate(-2.0, index, tau);
  EXPECT_EQ(index, 0);
  EXPECT_NEAR(tau, 0.0, 1e-15);
  trajectory.locate(100.0, index, tau);
  EXPECT_EQ(index, 1);
  EXPECT_NEAR(tau, 2.0, 1e-15);

  // 空轨迹：无段可归。
  Trajectory2D empty;
  empty.locate(1.0, index, tau);
  EXPECT_EQ(index, -1);
  EXPECT_NEAR(tau, 0.0, 1e-15);
}

TEST(Trajectory2DClampTime, NonFiniteAndOutOfRange)
{
  Trajectory2D trajectory;
  ASSERT_TRUE(trajectory.addPiece(kZero, kZero, 1.0));
  ASSERT_TRUE(trajectory.addPiece(kZero, kZero, 2.0));
  EXPECT_DOUBLE_EQ(trajectory.clampTime(-1.0), 0.0);
  EXPECT_DOUBLE_EQ(trajectory.clampTime(0.0), 0.0);
  EXPECT_DOUBLE_EQ(trajectory.clampTime(1.25), 1.25);
  EXPECT_DOUBLE_EQ(trajectory.clampTime(3.0), 3.0);
  EXPECT_DOUBLE_EQ(trajectory.clampTime(10.0), 3.0);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_DOUBLE_EQ(trajectory.clampTime(nan), 0.0);
  // 非有限时间一律回落到 0，避免 NaN 扩散。
  EXPECT_DOUBLE_EQ(trajectory.clampTime(std::numeric_limits<double>::infinity()), 0.0);
  EXPECT_DOUBLE_EQ(trajectory.clampTime(-std::numeric_limits<double>::infinity()), 0.0);

  // 越界时间在求值时被裁剪到 [0, totalDuration]。
  EXPECT_TRUE(trajectory.positionAt(-5.0).isApprox(trajectory.positionAt(0.0), 1e-15));
  EXPECT_TRUE(trajectory.positionAt(1e6).isApprox(trajectory.endPosition(), 1e-15));
  EXPECT_TRUE(trajectory.velocityAt(-5.0).isApprox(trajectory.velocityAt(0.0), 1e-15));
}

// ---------------------------------------------------------------------------
// setDurations
// ---------------------------------------------------------------------------

TEST(Trajectory2DSetDurations, MismatchOrNonPositive_ReturnFalse)
{
  Trajectory2D trajectory;
  ASSERT_TRUE(trajectory.addPiece(kZero, kZero, 1.0));
  ASSERT_TRUE(trajectory.addPiece(kZero, kZero, 2.0));

  EXPECT_FALSE(trajectory.setDurations({1.0}));
  EXPECT_FALSE(trajectory.setDurations({1.0, 2.0, 3.0}));
  EXPECT_FALSE(trajectory.setDurations({}));
  EXPECT_FALSE(trajectory.setDurations({1.0, 0.0}));
  EXPECT_FALSE(trajectory.setDurations({1.0, -2.0}));
  EXPECT_FALSE(trajectory.setDurations({1.0, std::numeric_limits<double>::quiet_NaN()}));
  EXPECT_FALSE(trajectory.setDurations({1.0, std::numeric_limits<double>::infinity()}));
  // 失败不得改动原有时间轴。
  ASSERT_EQ(trajectory.durations().size(), 2u);
  EXPECT_DOUBLE_EQ(trajectory.durations()[0], 1.0);
  EXPECT_DOUBLE_EQ(trajectory.durations()[1], 2.0);
  EXPECT_DOUBLE_EQ(trajectory.totalDuration(), 3.0);
}

TEST(Trajectory2DSetDurations, Success_RecomputesTimeAxis)
{
  Trajectory2D trajectory = makeContinuousTrajectory();
  const std::vector<double> durations{0.5, 2.5};
  ASSERT_TRUE(trajectory.setDurations(durations));
  EXPECT_EQ(trajectory.durations(), durations);
  EXPECT_DOUBLE_EQ(trajectory.totalDuration(), 3.0);
  EXPECT_NEAR(trajectory.totalDuration(), durations[0] + durations[1], 1e-15);

  // 总时长处的位置必须等于最后一段在其新时长处的解析值。
  const Trajectory2D::Coefficients & x1 = trajectory.xCoefficients()[1];
  const Trajectory2D::Coefficients & y1 = trajectory.yCoefficients()[1];
  EXPECT_NEAR(
    trajectory.positionAt(trajectory.totalDuration()).x(), evalDerivative(x1, 2.5, 0), 1e-12);
  EXPECT_NEAR(
    trajectory.positionAt(trajectory.totalDuration()).y(), evalDerivative(y1, 2.5, 0), 1e-12);
  EXPECT_TRUE(
    trajectory.positionAt(trajectory.totalDuration()).isApprox(trajectory.endPosition(), 1e-15));

  // 定位必须使用新的累计时间轴（原第一段 1.0 s 现在只有 0.5 s）。
  int index = -1;
  double tau = -1.0;
  trajectory.locate(0.5, index, tau);
  EXPECT_EQ(index, 1);
  EXPECT_NEAR(tau, 0.0, 1e-15);
  trajectory.locate(0.25, index, tau);
  EXPECT_EQ(index, 0);
  EXPECT_NEAR(tau, 0.25, 1e-15);
}

// ---------------------------------------------------------------------------
// sanityCheck / continuityResiduals
// ---------------------------------------------------------------------------

TEST(Trajectory2DSanityCheck, ContinuousTrajectory_Passes)
{
  Trajectory2D trajectory = makeContinuousTrajectory();
  trajectory.generated_stamp = 12.0;
  trajectory.valid_after = 12.5;
  trajectory.valid_until = 13.5;
  // _reason 只在失败时被写入，成功时不要求清空。
  std::string reason;
  EXPECT_TRUE(trajectory.sanityCheck(&reason));
  EXPECT_TRUE(trajectory.sanityCheck(nullptr));

  // 单段轨迹同样通过。
  Trajectory2D single;
  ASSERT_TRUE(single.addPiece({1.0, 0.0, 0.0, 0.0, 0.0, 0.0}, kZero, 1.0));
  EXPECT_TRUE(single.sanityCheck(&reason));

  // 空轨迹必须失败并给出原因。
  Trajectory2D empty;
  reason.clear();
  EXPECT_FALSE(empty.sanityCheck(&reason));
  EXPECT_FALSE(reason.empty());
}

TEST(Trajectory2DSanityCheck, DiscontinuousTrajectory_FailsWithReason)
{
  const Trajectory2D trajectory = makeDiscontinuousTrajectory();
  std::string reason;
  EXPECT_FALSE(trajectory.sanityCheck(&reason));
  EXPECT_FALSE(reason.empty());
  // 失败原因应指明段边界不连续，而不是别的无关问题。
  EXPECT_NE(reason.find("discontinu"), std::string::npos) << reason;
  EXPECT_FALSE(trajectory.sanityCheck(nullptr));
}

TEST(Trajectory2DSanityCheck, InvalidValidityWindow_Fails)
{
  Trajectory2D trajectory = makeContinuousTrajectory();
  trajectory.valid_after = 10.0;
  trajectory.valid_until = 9.0;
  std::string reason;
  EXPECT_FALSE(trajectory.sanityCheck(&reason));
  EXPECT_NE(reason.find("valid_until"), std::string::npos) << reason;

  // 相等或更大的有效期上限是允许的。
  trajectory.valid_until = 10.0;
  EXPECT_TRUE(trajectory.sanityCheck(&reason)) << reason;
  trajectory.valid_until = 20.0;
  EXPECT_TRUE(trajectory.sanityCheck(&reason)) << reason;

  // 非有限的时间元数据也必须被拒绝。
  trajectory.generated_stamp = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(trajectory.sanityCheck(&reason));
}

TEST(Trajectory2DContinuity, Residuals_ContinuousAndDiscontinuous)
{
  const Trajectory2D continuous = makeContinuousTrajectory();
  double max_pos = -1.0;
  double max_vel = -1.0;
  double max_acc = -1.0;
  continuous.continuityResiduals(max_pos, max_vel, max_acc);
  EXPECT_LT(max_pos, 1e-12);
  EXPECT_LT(max_vel, 1e-12);
  EXPECT_LT(max_acc, 1e-12);
  EXPECT_GE(max_pos, 0.0);
  EXPECT_GE(max_vel, 0.0);
  EXPECT_GE(max_acc, 0.0);

  const Trajectory2D discontinuous = makeDiscontinuousTrajectory();
  discontinuous.continuityResiduals(max_pos, max_vel, max_acc);
  EXPECT_NEAR(max_pos, 1.0, 1e-12);
  EXPECT_GT(max_pos, 0.0);
  EXPECT_LT(max_vel, 1e-12);
  EXPECT_LT(max_acc, 1e-12);

  // 只有一段时没有段边界，残差为零。
  Trajectory2D single;
  ASSERT_TRUE(single.addPiece(kZero, kZero, 1.0));
  single.continuityResiduals(max_pos, max_vel, max_acc);
  EXPECT_DOUBLE_EQ(max_pos, 0.0);
  EXPECT_DOUBLE_EQ(max_vel, 0.0);
  EXPECT_DOUBLE_EQ(max_acc, 0.0);
}

// ---------------------------------------------------------------------------
// tangentAt / samplePositions / clear
// ---------------------------------------------------------------------------

TEST(Trajectory2DTangent, StraightLine_UnitVector)
{
  // 匀速直线：v = (3, 4) m/s，速率为 5 m/s。
  const Trajectory2D::Coefficients x{0.0, 3.0, 0.0, 0.0, 0.0, 0.0};
  const Trajectory2D::Coefficients y{0.0, 4.0, 0.0, 0.0, 0.0, 0.0};
  Trajectory2D trajectory;
  ASSERT_TRUE(trajectory.addPiece(x, y, 1.0));
  Eigen::Vector2d tangent(-1.0, -1.0);
  ASSERT_TRUE(trajectory.tangentAt(0.5, tangent));
  EXPECT_NEAR(tangent.norm(), 1.0, 1e-12);
  EXPECT_NEAR(tangent.x(), 0.6, 1e-12);
  EXPECT_NEAR(tangent.y(), 0.8, 1e-12);
  // 越界时间被裁剪后切线方向不变。
  ASSERT_TRUE(trajectory.tangentAt(-10.0, tangent));
  EXPECT_NEAR(tangent.x(), 0.6, 1e-12);
  ASSERT_TRUE(trajectory.tangentAt(10.0, tangent));
  EXPECT_NEAR(tangent.y(), 0.8, 1e-12);
}

TEST(Trajectory2DTangent, StationaryPiece_ReturnsFalse)
{
  Trajectory2D trajectory;
  ASSERT_TRUE(trajectory.addPiece(kZero, kZero, 1.0));
  Eigen::Vector2d tangent(7.0, -7.0);
  EXPECT_FALSE(trajectory.tangentAt(0.5, tangent));
  // 失败时不得写入输出。
  EXPECT_DOUBLE_EQ(tangent.x(), 7.0);
  EXPECT_DOUBLE_EQ(tangent.y(), -7.0);

  // 空轨迹的切线同样不可用。
  Trajectory2D empty;
  EXPECT_FALSE(empty.tangentAt(0.0, tangent));
  EXPECT_DOUBLE_EQ(tangent.x(), 7.0);
}

TEST(Trajectory2DSamplePositions, Endpoints_MatchPositionAt)
{
  Trajectory2D trajectory = makeContinuousTrajectory();
  ASSERT_DOUBLE_EQ(trajectory.totalDuration(), 2.5);
  const std::vector<Eigen::Vector2d> samples = trajectory.samplePositions(0.5);
  ASSERT_GE(samples.size(), 2u);
  EXPECT_TRUE(samples.front().isApprox(trajectory.positionAt(0.0), 1e-9));
  EXPECT_TRUE(samples.back().isApprox(trajectory.endPosition(), 1e-9));
  EXPECT_TRUE(samples.front().isApprox(trajectory.startPosition(), 1e-9));

  // 非法步长与空轨迹返回空采样。
  EXPECT_TRUE(trajectory.samplePositions(0.0).empty());
  EXPECT_TRUE(trajectory.samplePositions(-0.1).empty());
  EXPECT_TRUE(trajectory.samplePositions(std::numeric_limits<double>::quiet_NaN()).empty());
  Trajectory2D empty;
  EXPECT_TRUE(empty.samplePositions(0.1).empty());
}

TEST(Trajectory2DClear, ResetsGeometry)
{
  Trajectory2D trajectory = makeContinuousTrajectory();
  ASSERT_EQ(trajectory.pieceCount(), 2);
  trajectory.clear();
  EXPECT_TRUE(trajectory.empty());
  EXPECT_EQ(trajectory.pieceCount(), 0);
  EXPECT_DOUBLE_EQ(trajectory.totalDuration(), 0.0);
  EXPECT_TRUE(trajectory.durations().empty());
  EXPECT_TRUE(trajectory.xCoefficients().empty());
  EXPECT_TRUE(trajectory.yCoefficients().empty());
  EXPECT_TRUE(trajectory.positionAt(0.0).isApprox(Eigen::Vector2d::Zero(), 0.0));
  EXPECT_TRUE(trajectory.endPosition().isApprox(Eigen::Vector2d::Zero(), 0.0));
  EXPECT_FALSE(trajectory.sanityCheck(nullptr));
}
