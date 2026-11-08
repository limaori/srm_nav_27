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

#include "fake_vel_transform/velocity_transform.hpp"

/// \file
/// \brief fakeyaw 坐标转换的纯数学回归（方案 §3.1、§4.1.3、§5 P0）。
///
/// 通过条件（方案 §5 P0）：线速度变换误差 < 1e-6 m/s，`wz` 输入输出误差 < 1e-6 rad/s。
/// 这里把四个基准 yaw（0、+pi/2、pi、-pi/2）的方向、模长保持与 `wz` 透传全部固化。

namespace
{

using fake_vel_transform::fakeFrameYaw;
using fake_vel_transform::fakeToRobotBase;
using fake_vel_transform::isUsableOrientation;
using fake_vel_transform::PlanarVelocity;
using fake_vel_transform::planarSpeed;
using fake_vel_transform::quaternionFromYaw;

constexpr double kPi = 3.14159265358979323846;
/// \brief 与 gtest 默认精度相比更紧的门槛：方案 §5 P0 要求 < 1e-6。
constexpr double kTolerance = 1.0e-6;

/// \brief 四元数转 yaw（与 tf2::getYaw 同式），用于验证 TF 四元数。
double yawFromQuaternion(const fake_vel_transform::QuaternionZ & _q)
{
  return std::atan2(2.0 * (_q.w * _q.z + _q.x * _q.y), 1.0 - 2.0 * (_q.y * _q.y + _q.z * _q.z));
}

}  // namespace

// psi = 0：fake 系与车体系重合，速度原样通过。
TEST(VelocityTransformTest, IdentityAtZeroYaw)
{
  const PlanarVelocity v_fake{0.3, 0.2, 0.5};
  const PlanarVelocity v_base = fakeToRobotBase(v_fake, 0.0);
  EXPECT_NEAR(v_base.vx, 0.3, kTolerance);
  EXPECT_NEAR(v_base.vy, 0.2, kTolerance);
  EXPECT_NEAR(v_base.wz, 0.5, kTolerance);
}

// psi = +pi/2：odom(即 fake 系) +x 落在车体 -y（车头朝左，前方在车的右手边）。
TEST(VelocityTransformTest, ForwardAlongOdomXBecomesRightAtHalfPi)
{
  const PlanarVelocity v_base = fakeToRobotBase(PlanarVelocity{1.0, 0.0, 0.0}, kPi / 2.0);
  EXPECT_NEAR(v_base.vx, 0.0, kTolerance);
  EXPECT_NEAR(v_base.vy, -1.0, kTolerance);

  // odom +y 落在车体 +x：横向平移在车体里变成前进。
  const PlanarVelocity lateral = fakeToRobotBase(PlanarVelocity{0.0, 0.4, 0.0}, kPi / 2.0);
  EXPECT_NEAR(lateral.vx, 0.4, kTolerance);
  EXPECT_NEAR(lateral.vy, 0.0, kTolerance);
}

// psi = -pi/2：方向与 +pi/2 相反（不能有符号错误）。
TEST(VelocityTransformTest, DirectionsFlipAtMinusHalfPi)
{
  const PlanarVelocity v_base = fakeToRobotBase(PlanarVelocity{1.0, 0.0, 0.0}, -kPi / 2.0);
  EXPECT_NEAR(v_base.vx, 0.0, kTolerance);
  EXPECT_NEAR(v_base.vy, 1.0, kTolerance);

  const PlanarVelocity lateral = fakeToRobotBase(PlanarVelocity{0.0, 0.4, 0.0}, -kPi / 2.0);
  EXPECT_NEAR(lateral.vx, -0.4, kTolerance);
  EXPECT_NEAR(lateral.vy, 0.0, kTolerance);
}

// psi = pi：只要有一次符号写错这里就会露出来（差 180 度）。
TEST(VelocityTransformTest, HalfTurnInvertsBothAxes)
{
  const PlanarVelocity v_base = fakeToRobotBase(PlanarVelocity{0.3, -0.2, 0.0}, kPi);
  EXPECT_NEAR(v_base.vx, -0.3, kTolerance);
  EXPECT_NEAR(v_base.vy, 0.2, kTolerance);
}

// wz 必须逐条透传：方案 §3.1 明确"不能再叠加 cmd_spin"。
TEST(VelocityTransformTest, AngularVelocityPassesThroughExactly)
{
  for (const double yaw : {0.0, kPi / 2.0, kPi, -kPi / 2.0, 2.7}) {
    for (const double wz : {0.0, 0.5, -1.25}) {
      const PlanarVelocity v_base = fakeToRobotBase(PlanarVelocity{0.1, 0.2, wz}, yaw);
      EXPECT_NEAR(v_base.wz, wz, kTolerance) << "yaw=" << yaw;
    }
  }
}

// 纯 yaw 旋转保持平移模长（方案 §6 首轮验收门槛）。
TEST(VelocityTransformTest, TranslationMagnitudeIsPreserved)
{
  const PlanarVelocity v_fake{0.3, 0.2, 0.0};
  for (const double yaw : {0.0, kPi / 2.0, kPi, -kPi / 2.0, 1.234}) {
    const PlanarVelocity v_base = fakeToRobotBase(v_fake, yaw);
    EXPECT_NEAR(planarSpeed(v_base), planarSpeed(v_fake), kTolerance) << "yaw=" << yaw;
  }
}

// 先按 psi 变换、再按 -psi 变换必须回到原值（可逆性，防止出现重复转换）。
TEST(VelocityTransformTest, RoundTripInvertsTheTransform)
{
  const PlanarVelocity v_fake{0.35, -0.15, 0.4};
  const double yaw = 0.83;
  const PlanarVelocity v_base = fakeToRobotBase(v_fake, yaw);
  const PlanarVelocity back = fakeToRobotBase(v_base, fakeFrameYaw(yaw));
  EXPECT_NEAR(back.vx, v_fake.vx, kTolerance);
  EXPECT_NEAR(back.vy, v_fake.vy, kTolerance);
  EXPECT_NEAR(back.wz, v_fake.wz, kTolerance);
}

// TF 四元数：base_link -> base_link_fake 的 yaw 必须是 -psi，且只绕 z。
TEST(VelocityTransformTest, FakeFrameYawIsTheInverseOfTheChassisYaw)
{
  EXPECT_NEAR(fakeFrameYaw(0.7), -0.7, kTolerance);
  for (const double yaw : {0.0, kPi / 2.0, kPi, -kPi / 2.0, -2.9}) {
    const auto q = quaternionFromYaw(fakeFrameYaw(yaw));
    EXPECT_NEAR(q.x, 0.0, kTolerance) << "yaw=" << yaw;
    EXPECT_NEAR(q.y, 0.0, kTolerance) << "yaw=" << yaw;
    // 范数必须是 1，否则 RViz / tf2 会丢弃或错误使用这条 TF。
    const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    EXPECT_NEAR(norm, 1.0, kTolerance) << "yaw=" << yaw;
    // atan2 只在 (-pi, pi] 唯一，比较时按圆周归一化。
    const double delta = std::remainder(yawFromQuaternion(q) - fakeFrameYaw(yaw), 2.0 * kPi);
    EXPECT_NEAR(delta, 0.0, kTolerance) << "yaw=" << yaw;
  }
}

// 姿态可用性检查：全零 / NaN / 无穷都必须在进入 tf2::getYaw 之前被拦下。
TEST(VelocityTransformTest, RejectsUnusableOrientation)
{
  EXPECT_TRUE(isUsableOrientation(0.0, 0.0, 0.0, 1.0));
  EXPECT_FALSE(isUsableOrientation(0.0, 0.0, 0.0, 0.0));
  EXPECT_FALSE(isUsableOrientation(
    std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, 1.0));
  EXPECT_FALSE(isUsableOrientation(
    0.0, std::numeric_limits<double>::infinity(), 0.0, 1.0));
}
