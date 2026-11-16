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

#include <algorithm>
#include <cmath>
#include <limits>

#include "srm27_minco_core/kinematics.hpp"

namespace
{

/// \brief 圆周率常量，避免依赖未标准化的 M_PI 宏。
const double kPi = std::acos(-1.0);

/// \brief 两个角度之间的最短有向差（rad），用于判断角度是否等价。
double angleError(const double _lhs, const double _rhs)
{
  return std::atan2(std::sin(_lhs - _rhs), std::cos(_lhs - _rhs));
}

/// \brief 从 `_start` 起按固定步长连续推进，用 unwrapAngleNear 维护连续航向。
/// \param _start 展开航向的起始值（rad）。
/// \param _step 每步的真实增量（rad）。
/// \param _steps 推进步数。
/// \return 各步展开值与解析期望值的最大绝对偏差（rad）。
double maxUnwrapContinuityError(const double _start, const double _step, const int _steps)
{
  double unwrapped = srm27_minco_core::normalizeAngle(_start);
  double error = std::abs(unwrapped - _start);
  for (int i = 1; i <= _steps; ++i) {
    const double truth = _start + _step * static_cast<double>(i);
    const double raw = srm27_minco_core::normalizeAngle(truth);
    unwrapped = srm27_minco_core::unwrapAngleNear(unwrapped, raw);
    error = std::max(error, std::abs(unwrapped - truth));
  }
  return error;
}

/// \brief 校验车体系方向到 odom 系方向的映射，并顺带验证往返一致性。
void expectBodyToOdomDirection(
  const double _yaw, const Eigen::Vector2d & _body, const Eigen::Vector2d & _expected_odom)
{
  const Eigen::Vector2d odom = srm27_minco_core::bodyToOdomVelocity(_yaw, _body);
  EXPECT_TRUE(odom.isApprox(_expected_odom, 1e-12)) << "yaw = " << _yaw;
  const Eigen::Vector2d back = srm27_minco_core::odomToBodyVelocity(_yaw, odom);
  EXPECT_TRUE(back.isApprox(_body, 1e-12)) << "yaw = " << _yaw;
}

}  // namespace

// ---------------------------------------------------------------------------
// normalizeAngle
// ---------------------------------------------------------------------------

/// \brief 0 与 2*pi 整数倍必须折到 0，而 +/-pi 必须保留（不能被折成 0）。
TEST(NormalizeAngle, Boundary_MultiplesOfPi)
{
  EXPECT_DOUBLE_EQ(srm27_minco_core::normalizeAngle(0.0), 0.0);
  EXPECT_NEAR(srm27_minco_core::normalizeAngle(2.0 * kPi), 0.0, 1e-12);
  EXPECT_NEAR(srm27_minco_core::normalizeAngle(-2.0 * kPi), 0.0, 1e-12);
  EXPECT_NEAR(srm27_minco_core::normalizeAngle(4.0 * kPi), 0.0, 1e-12);
  EXPECT_NEAR(srm27_minco_core::normalizeAngle(kPi), kPi, 1e-12);
  EXPECT_NEAR(srm27_minco_core::normalizeAngle(-kPi), -kPi, 1e-12);
  // +/-3*pi 是奇数倍，折到 +/-pi 而不是 0。
  EXPECT_NEAR(srm27_minco_core::normalizeAngle(3.0 * kPi), kPi, 1e-12);
  EXPECT_NEAR(srm27_minco_core::normalizeAngle(-3.0 * kPi), -kPi, 1e-12);
  EXPECT_NEAR(srm27_minco_core::normalizeAngle(5.0 * kPi), kPi, 1e-12);
  EXPECT_NEAR(srm27_minco_core::normalizeAngle(-5.0 * kPi), -kPi, 1e-12);
}

/// \brief 归一化只允许改变 2*pi 的整数倍，且结果必须落在 [-pi, pi]。
TEST(NormalizeAngle, Range_TrigonometryPreserved)
{
  for (double angle = -12.0 * kPi; angle <= 12.0 * kPi; angle += 0.37) {
    const double normalized = srm27_minco_core::normalizeAngle(angle);
    EXPECT_GE(normalized, -kPi - 1e-12) << "angle = " << angle;
    EXPECT_LE(normalized, kPi + 1e-12) << "angle = " << angle;
    EXPECT_NEAR(std::sin(normalized), std::sin(angle), 1e-9) << "angle = " << angle;
    EXPECT_NEAR(std::cos(normalized), std::cos(angle), 1e-9) << "angle = " << angle;
    // 结果与输入必须代表同一个方向。
    EXPECT_NEAR(angleError(normalized, angle), 0.0, 1e-9) << "angle = " << angle;
  }
}

// ---------------------------------------------------------------------------
// unwrapAngleNear
// ---------------------------------------------------------------------------

/// \brief 展开前后与参考值的差必须落在 (-pi, pi]（数值上允许 1e-9 的舍入余量）。
TEST(UnwrapAngleNear, DifferenceWithinPi)
{
  const double references[] = {0.0, kPi * 0.5, kPi, -kPi * 0.5, 7.0, -7.0, 100.0};
  const double angles[] = {0.0, kPi, -kPi, 3.0 * kPi, -3.0 * kPi, kPi * 0.5, -kPi * 0.5};
  for (const double reference : references) {
    for (const double angle : angles) {
      const double unwrapped = srm27_minco_core::unwrapAngleNear(reference, angle);
      const double difference = unwrapped - reference;
      EXPECT_GT(difference, -kPi - 1e-9) << "reference = " << reference << ", angle = " << angle;
      EXPECT_LE(difference, kPi + 1e-9) << "reference = " << reference << ", angle = " << angle;
      // 展开只允许相差 2*pi 的整数倍。
      EXPECT_NEAR(angleError(unwrapped, angle), 0.0, 1e-9);
      EXPECT_TRUE(srm27_minco_core::isFinite(unwrapped));
    }
  }
}

/// \brief 参考值附近的小角度输入不能被改写。
TEST(UnwrapAngleNear, SmallOffset_NoWrap)
{
  EXPECT_NEAR(srm27_minco_core::unwrapAngleNear(0.0, 0.1), 0.1, 1e-15);
  EXPECT_NEAR(srm27_minco_core::unwrapAngleNear(kPi * 0.5, 0.3), 0.3, 1e-15);
  EXPECT_NEAR(srm27_minco_core::unwrapAngleNear(-2.0, -2.5), -2.5, 1e-15);
  // 参考值本身可以落在 (-pi, pi] 之外，展开结果仍应贴近参考值。
  const double unwrapped = srm27_minco_core::unwrapAngleNear(7.0, 0.3);
  EXPECT_NEAR(unwrapped, 7.0 + angleError(0.3, 7.0), 1e-12);
  EXPECT_NEAR(angleError(unwrapped, 0.3), 0.0, 1e-12);
}

/// \brief 从 0、pi/2、pi 等起始值连续跨 pi（正负方向）时航向必须保持连续。
TEST(UnwrapAngleNear, ContinuityAcrossPi)
{
  // 起始角度取 0、pi/2（区间内不跨界）与 +/-pi 附近（必然跨界）。
  const double starts[] = {0.0, kPi * 0.5, kPi - 0.05, -kPi + 0.05, kPi};
  for (const double start : starts) {
    const double forward = maxUnwrapContinuityError(start, 0.02, 10);
    const double backward = maxUnwrapContinuityError(start, -0.02, 10);
    EXPECT_LT(forward, 1e-9) << "start = " << start;
    EXPECT_LT(backward, 1e-9) << "start = " << start;
  }
}

/// \brief 显式检查 +pi -> -pi 与 -pi -> +pi 两个方向的跨界连续性。
TEST(UnwrapAngleNear, ContinuityAcrossPi_ExplicitJump)
{
  // 从 pi-0.05 前进两步跨过 +pi，展开值应为 pi+0.05 而不是 -pi+0.05。
  const double first = srm27_minco_core::unwrapAngleNear(kPi - 0.05, kPi - 0.03);
  const double second = srm27_minco_core::unwrapAngleNear(first, -kPi + 0.05);
  EXPECT_NEAR(first, kPi - 0.03, 1e-12);
  EXPECT_NEAR(second, kPi + 0.05, 1e-12);
  EXPECT_GT(second, kPi);

  // 从 -pi+0.05 后退两步跨过 -pi，展开值应为 -pi-0.05。
  const double third = srm27_minco_core::unwrapAngleNear(-kPi + 0.05, -kPi + 0.03);
  const double fourth = srm27_minco_core::unwrapAngleNear(third, kPi - 0.05);
  EXPECT_NEAR(third, -kPi + 0.03, 1e-12);
  EXPECT_NEAR(fourth, -kPi - 0.05, 1e-12);
  EXPECT_LT(fourth, -kPi);
}

// ---------------------------------------------------------------------------
// rotation2d / odomToBodyVelocity / bodyToOdomVelocity
// ---------------------------------------------------------------------------

/// \brief yaw = 0 时旋转矩阵必须是单位阵。
TEST(Rotation2d, ZeroYaw_Identity)
{
  const Eigen::Matrix2d rotation = srm27_minco_core::rotation2d(0.0);
  EXPECT_TRUE(rotation.isApprox(Eigen::Matrix2d::Identity(), 1e-15));
}

/// \brief yaw = 0、pi/2、pi、-pi/2 时的方向映射必须符合右手系定义。
TEST(Rotation2d, CardinalYaw_Directions)
{
  const Eigen::Vector2d x_body(1.0, 0.0);
  const Eigen::Vector2d y_body(0.0, 1.0);

  // yaw = 0：车体系与 odom 重合。
  expectBodyToOdomDirection(0.0, x_body, Eigen::Vector2d(1.0, 0.0));
  expectBodyToOdomDirection(0.0, y_body, Eigen::Vector2d(0.0, 1.0));

  // yaw = pi/2：车体系 x 轴指向 odom 的 +y。
  expectBodyToOdomDirection(kPi * 0.5, x_body, Eigen::Vector2d(0.0, 1.0));
  expectBodyToOdomDirection(kPi * 0.5, y_body, Eigen::Vector2d(-1.0, 0.0));

  // yaw = pi：车体系 x 轴指向 odom 的 -x。
  expectBodyToOdomDirection(kPi, x_body, Eigen::Vector2d(-1.0, 0.0));
  expectBodyToOdomDirection(kPi, y_body, Eigen::Vector2d(0.0, -1.0));

  // yaw = -pi/2：车体系 x 轴指向 odom 的 -y。
  expectBodyToOdomDirection(-kPi * 0.5, x_body, Eigen::Vector2d(0.0, -1.0));
  expectBodyToOdomDirection(-kPi * 0.5, y_body, Eigen::Vector2d(1.0, 0.0));
}

/// \brief odom->body 转换在四个基准 yaw 下的方向也必须正确。
TEST(OdomToBodyVelocity, CardinalYaw_Directions)
{
  const Eigen::Vector2d v_odom(0.0, 2.0);
  EXPECT_TRUE(srm27_minco_core::odomToBodyVelocity(0.0, v_odom).isApprox(v_odom, 1e-12));
  EXPECT_TRUE(srm27_minco_core::odomToBodyVelocity(kPi * 0.5, v_odom)
                .isApprox(Eigen::Vector2d(2.0, 0.0), 1e-12));
  EXPECT_TRUE(
    srm27_minco_core::odomToBodyVelocity(kPi, v_odom).isApprox(Eigen::Vector2d(0.0, -2.0), 1e-12));
  EXPECT_TRUE(srm27_minco_core::odomToBodyVelocity(-kPi * 0.5, v_odom)
                .isApprox(Eigen::Vector2d(-2.0, 0.0), 1e-12));
}

/// \brief 往返转换误差必须小于 1e-12，且旋转矩阵保持正交。
TEST(VelocityConversion, RoundTrip_ErrorBelowTolerance)
{
  const double yaws[] = {0.0, kPi * 0.5, kPi, -kPi * 0.5, 0.37, -2.9, 12.5};
  const Eigen::Vector2d velocities[] = {
    Eigen::Vector2d(1.0, 0.0), Eigen::Vector2d(0.0, 1.0), Eigen::Vector2d(0.3, -0.4),
    Eigen::Vector2d(-2.0, 1.5), Eigen::Vector2d(0.0, 0.0)};

  for (const double yaw : yaws) {
    const Eigen::Matrix2d rotation = srm27_minco_core::rotation2d(yaw);
    const Eigen::Matrix2d identity = rotation.transpose() * rotation;
    EXPECT_TRUE(identity.isApprox(Eigen::Matrix2d::Identity(), 1e-12)) << "yaw = " << yaw;
    for (const Eigen::Vector2d & velocity : velocities) {
      const Eigen::Vector2d body = srm27_minco_core::odomToBodyVelocity(yaw, velocity);
      const Eigen::Vector2d back = srm27_minco_core::bodyToOdomVelocity(yaw, body);
      EXPECT_LT((back - velocity).norm(), 1e-12) << "yaw = " << yaw;
      // 转换是纯旋转，必须保持速度模长。
      EXPECT_NEAR(body.norm(), velocity.norm(), 1e-12) << "yaw = " << yaw;
    }
  }
}

// ---------------------------------------------------------------------------
// clampToRange
// ---------------------------------------------------------------------------

/// \brief 正常区间的上下界与实际值裁剪。
TEST(ClampToRange, NormalInterval)
{
  EXPECT_DOUBLE_EQ(srm27_minco_core::clampToRange(-5.0, 0.0, 10.0), 0.0);
  EXPECT_DOUBLE_EQ(srm27_minco_core::clampToRange(5.0, 0.0, 10.0), 5.0);
  EXPECT_DOUBLE_EQ(srm27_minco_core::clampToRange(50.0, 0.0, 10.0), 10.0);
  // 区间端点属于闭区间，必须原样返回。
  EXPECT_DOUBLE_EQ(srm27_minco_core::clampToRange(0.0, 0.0, 10.0), 0.0);
  EXPECT_DOUBLE_EQ(srm27_minco_core::clampToRange(10.0, 0.0, 10.0), 10.0);
  // 负区间与单点区间。
  EXPECT_DOUBLE_EQ(srm27_minco_core::clampToRange(-3.0, -2.0, -1.0), -2.0);
  EXPECT_DOUBLE_EQ(srm27_minco_core::clampToRange(-0.5, -2.0, -1.0), -1.0);
  EXPECT_DOUBLE_EQ(srm27_minco_core::clampToRange(3.0, 2.0, 2.0), 2.0);
  EXPECT_DOUBLE_EQ(srm27_minco_core::clampToRange(-3.0, 2.0, 2.0), 2.0);
}

/// \brief _low > _high 是方向相反的空区间，按约定返回下界。
TEST(ClampToRange, DegenerateInterval_ReturnsLow)
{
  EXPECT_DOUBLE_EQ(srm27_minco_core::clampToRange(5.0, 10.0, 0.0), 10.0);
  EXPECT_DOUBLE_EQ(srm27_minco_core::clampToRange(-5.0, 10.0, 0.0), 10.0);
  EXPECT_DOUBLE_EQ(srm27_minco_core::clampToRange(0.0, 10.0, 0.0), 10.0);
  EXPECT_DOUBLE_EQ(srm27_minco_core::clampToRange(100.0, -1.0, -2.0), -1.0);
}

// ---------------------------------------------------------------------------
// stoppingDistance
// ---------------------------------------------------------------------------

/// \brief v = 0 时制动距离为 0，正常情况下与解析式一致。
TEST(StoppingDistance, ZeroSpeed_AndAnalyticValue)
{
  EXPECT_DOUBLE_EQ(srm27_minco_core::stoppingDistance(0.0, 1.0), 0.0);
  EXPECT_DOUBLE_EQ(srm27_minco_core::stoppingDistance(0.0, 2.5), 0.0);

  // v^2 / (2*a)。
  const double speed = 1.5;
  const double decel = 0.8;
  EXPECT_NEAR(
    srm27_minco_core::stoppingDistance(speed, decel), speed * speed / (2.0 * decel), 1e-15);
  EXPECT_NEAR(srm27_minco_core::stoppingDistance(3.0, 2.0), 2.25, 1e-15);
  EXPECT_NEAR(srm27_minco_core::stoppingDistance(1.0, 0.5), 1.0, 1e-15);
  // 距离随速度单调递增、随减速度单调递减。
  EXPECT_GT(
    srm27_minco_core::stoppingDistance(2.0, 1.0), srm27_minco_core::stoppingDistance(1.0, 1.0));
  EXPECT_LT(
    srm27_minco_core::stoppingDistance(2.0, 2.0), srm27_minco_core::stoppingDistance(2.0, 1.0));
  // 负速度按 0 处理，不允许出现负的制动距离。
  EXPECT_DOUBLE_EQ(srm27_minco_core::stoppingDistance(-2.0, 1.0), 0.0);
}

/// \brief 减速度非法（0 / 负 / NaN / inf）时返回 +inf，表示“无法停车”。
TEST(StoppingDistance, InvalidDeceleration_ReturnsInfinity)
{
  const double infinity = std::numeric_limits<double>::infinity();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(srm27_minco_core::stoppingDistance(1.0, 0.0), infinity);
  EXPECT_EQ(srm27_minco_core::stoppingDistance(1.0, -1.0), infinity);
  EXPECT_EQ(srm27_minco_core::stoppingDistance(1.0, -0.0), infinity);
  EXPECT_EQ(srm27_minco_core::stoppingDistance(1.0, nan), infinity);
  EXPECT_EQ(srm27_minco_core::stoppingDistance(1.0, infinity), infinity);
  // 速度非有限同样视为参数非法。
  EXPECT_EQ(srm27_minco_core::stoppingDistance(nan, 1.0), infinity);
  EXPECT_EQ(srm27_minco_core::stoppingDistance(infinity, 1.0), infinity);
  EXPECT_EQ(srm27_minco_core::stoppingDistance(-infinity, 1.0), infinity);
}

// ---------------------------------------------------------------------------
// advanceUnwrappedYaw
// ---------------------------------------------------------------------------

/// \brief 任一输入非有限都必须返回 NaN，绝不能把 NaN 传染成看似有效的航向。
TEST(AdvanceUnwrappedYaw, NonFiniteInput_ReturnsNaN)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double infinity = std::numeric_limits<double>::infinity();
  EXPECT_TRUE(std::isnan(srm27_minco_core::advanceUnwrappedYaw(nan, 0.1, 0.1)));
  EXPECT_TRUE(std::isnan(srm27_minco_core::advanceUnwrappedYaw(0.0, nan, 0.1)));
  EXPECT_TRUE(std::isnan(srm27_minco_core::advanceUnwrappedYaw(0.0, 0.1, nan)));
  EXPECT_TRUE(std::isnan(srm27_minco_core::advanceUnwrappedYaw(infinity, 0.1, 0.1)));
  EXPECT_TRUE(std::isnan(srm27_minco_core::advanceUnwrappedYaw(0.0, infinity, 0.1)));
  EXPECT_TRUE(std::isnan(srm27_minco_core::advanceUnwrappedYaw(0.0, 0.1, infinity)));
  EXPECT_TRUE(std::isnan(srm27_minco_core::advanceUnwrappedYaw(-infinity, -infinity, -infinity)));
}

/// \brief 连续积分不得做 2*pi 折叠，结果必须等于解析积分值。
TEST(AdvanceUnwrappedYaw, AccumulatesWithoutWrapping)
{
  const double omega = 0.5;
  const double dt = 0.02;
  const int steps = 500;
  double yaw = 0.0;
  for (int i = 0; i < steps; ++i) {
    yaw = srm27_minco_core::advanceUnwrappedYaw(yaw, omega, dt);
  }
  EXPECT_NEAR(yaw, omega * dt * static_cast<double>(steps), 1e-12);
  // 累计转角约 5 rad，已经越过 pi，说明内部没有做区间折叠。
  EXPECT_GT(yaw, kPi);

  // 单步退化为直接相加；dt = 0 时保持原值。
  EXPECT_NEAR(srm27_minco_core::advanceUnwrappedYaw(1.0, 0.25, 0.4), 1.1, 1e-15);
  EXPECT_DOUBLE_EQ(srm27_minco_core::advanceUnwrappedYaw(1.234, 3.0, 0.0), 1.234);
}

// ---------------------------------------------------------------------------
// isFinite
// ---------------------------------------------------------------------------

/// \brief 标量重载：有限值为真，NaN 与正负无穷为假。
TEST(IsFinite, ScalarOverload)
{
  EXPECT_TRUE(srm27_minco_core::isFinite(0.0));
  EXPECT_TRUE(srm27_minco_core::isFinite(-1.0e300));
  EXPECT_TRUE(srm27_minco_core::isFinite(1.0e-300));
  EXPECT_FALSE(srm27_minco_core::isFinite(std::numeric_limits<double>::quiet_NaN()));
  EXPECT_FALSE(srm27_minco_core::isFinite(std::numeric_limits<double>::infinity()));
  EXPECT_FALSE(srm27_minco_core::isFinite(-std::numeric_limits<double>::infinity()));
}

/// \brief 向量重载：任一分量非有限即为假。
TEST(IsFinite, Vector2dOverload)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double infinity = std::numeric_limits<double>::infinity();
  EXPECT_TRUE(srm27_minco_core::isFinite(Eigen::Vector2d(1.0, -2.0)));
  EXPECT_TRUE(srm27_minco_core::isFinite(Eigen::Vector2d(0.0, 0.0)));
  EXPECT_FALSE(srm27_minco_core::isFinite(Eigen::Vector2d(nan, 0.0)));
  EXPECT_FALSE(srm27_minco_core::isFinite(Eigen::Vector2d(0.0, nan)));
  EXPECT_FALSE(srm27_minco_core::isFinite(Eigen::Vector2d(0.0, infinity)));
  // 默认零向量必须有限且确实为零。
  EXPECT_TRUE(srm27_minco_core::isFinite(srm27_minco_core::zero2d()));
  EXPECT_TRUE(srm27_minco_core::zero2d().isZero(0.0));
}
