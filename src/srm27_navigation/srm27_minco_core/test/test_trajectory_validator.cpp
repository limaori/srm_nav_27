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

#include <Eigen/Core>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "srm27_minco_core/esdf_2d.hpp"
#include "srm27_minco_core/grid_snapshot.hpp"
#include "srm27_minco_core/trajectory_2d.hpp"
#include "srm27_minco_core/trajectory_validator.hpp"
#include "srm27_minco_core/types.hpp"

namespace minco = srm27_minco_core;

namespace
{

/// \brief 测试地图：200x200，分辨率 0.05 m，原点 (-5,-5)，覆盖 [-5,5]^2。
constexpr int kSizeX = 200;
constexpr int kSizeY = 200;
constexpr double kResolution = 0.05;
constexpr double kOriginX = -5.0;
constexpr double kOriginY = -5.0;
/// \brief 地图时间戳（s）。
constexpr double kStamp = 100.0;

/// \brief 中间的竖墙：单元 x 索引 [98,101]、y 索引 [60,140]，
/// 对应世界坐标 x ∈ [-0.1, 0.1)、y ∈ [-2.0, 2.05)。
constexpr int kWallMinX = 98;
constexpr int kWallMaxX = 101;
constexpr int kWallMinY = 60;
constexpr int kWallMaxY = 140;

/// \brief 便捷构造二维点。
Eigen::Vector2d P(const double _x, const double _y) { return Eigen::Vector2d(_x, _y); }

/// \brief 构造带一堵竖墙的占用快照。
minco::GridSnapshot MakeWallSnapshot(const double _stamp)
{
  std::vector<minco::CellState> cells(
    static_cast<std::size_t>(kSizeX) * static_cast<std::size_t>(kSizeY), minco::CellState::kFree);
  for (int my = kWallMinY; my <= kWallMaxY; ++my) {
    for (int mx = kWallMinX; mx <= kWallMaxX; ++mx) {
      const std::size_t index = static_cast<std::size_t>(my) * static_cast<std::size_t>(kSizeX) +
                                static_cast<std::size_t>(mx);
      cells[index] = minco::CellState::kOccupied;
    }
  }

  minco::GridSnapshot snapshot;
  const bool ok =
    snapshot.reset(kOriginX, kOriginY, kResolution, kSizeX, kSizeY, std::move(cells), 1u, _stamp);
  EXPECT_TRUE(ok);
  return snapshot;
}

/// \brief 由快照建立距离场。
minco::Esdf2D MakeWallEsdf(const double _stamp)
{
  const minco::GridSnapshot snapshot = MakeWallSnapshot(_stamp);
  minco::Esdf2D esdf;
  const bool ok = esdf.build(snapshot);
  EXPECT_TRUE(ok);
  return esdf;
}

/// \brief 校验器默认配置（满足全部参数自检）。
minco::TrajectoryValidatorConfig MakeConfig()
{
  minco::TrajectoryValidatorConfig config;
  config.robot_radius = 0.33;
  config.clearance_margin = 0.05;
  config.max_linear_speed = 0.5;
  config.max_linear_accel = 0.3;
  config.min_piece_duration = 0.05;
  config.braking_deceleration = 0.3;
  config.reaction_latency = 0.10;
  config.required_prefix_duration = 0.6;
  config.map_timeout = 0.30;
  config.trajectory_max_age = 0.30;
  return config;
}

/// \brief 构造一段匀速直线（`p(tau) = p0 + v*tau`）。
minco::Trajectory2D MakeConstantVelocityTrajectory(
  const Eigen::Vector2d & _start, const Eigen::Vector2d & _velocity, const double _duration)
{
  minco::Trajectory2D trajectory;
  minco::Trajectory2D::Coefficients x{};
  minco::Trajectory2D::Coefficients y{};
  x[0] = _start.x();
  x[1] = _velocity.x();
  y[0] = _start.y();
  y[1] = _velocity.y();
  const bool ok = trajectory.addPiece(x, y, _duration);
  EXPECT_TRUE(ok);
  return trajectory;
}

/// \brief 构造一段匀加速运动（`p(tau) = p0 + v0*tau + 0.5*a*tau^2`）。
minco::Trajectory2D MakeConstantAccelerationTrajectory(
  const Eigen::Vector2d & _start, const Eigen::Vector2d & _initial_velocity,
  const Eigen::Vector2d & _acceleration, const double _duration)
{
  minco::Trajectory2D trajectory;
  minco::Trajectory2D::Coefficients x{};
  minco::Trajectory2D::Coefficients y{};
  x[0] = _start.x();
  x[1] = _initial_velocity.x();
  x[2] = 0.5 * _acceleration.x();
  y[0] = _start.y();
  y[1] = _initial_velocity.y();
  y[2] = 0.5 * _acceleration.y();
  const bool ok = trajectory.addPiece(x, y, _duration);
  EXPECT_TRUE(ok);
  return trajectory;
}

/// \brief 远离障碍的匀速直线轨迹：从 (-2,-3) 以 0.4 m/s 沿 +x 走 10 s。
minco::Trajectory2D MakeSafeTrajectory()
{
  return MakeConstantVelocityTrajectory(P(-2.0, -3.0), P(0.4, 0.0), 10.0);
}

/// \brief 构造一个有效状态（平移速度给定）。
minco::State2D MakeState(const double _x, const double _y, const double _speed)
{
  minco::State2D state;
  state.valid = true;
  state.x = _x;
  state.y = _y;
  state.velocity = Eigen::Vector2d(_speed, 0.0);
  state.sample_stamp = kStamp;
  state.received_stamp = kStamp;
  return state;
}

}  // namespace

// ---------------------------------------------------------------------------
// configure：参数自检
// ---------------------------------------------------------------------------

TEST(TrajectoryValidatorTest, Configure_AcceptsValidConfig)
{
  minco::TrajectoryValidator validator;
  std::string reason;
  EXPECT_TRUE(validator.configure(MakeConfig(), &reason)) << reason;
  EXPECT_TRUE(validator.configured());
  EXPECT_TRUE(reason.empty());
  EXPECT_DOUBLE_EQ(validator.config().robot_radius, 0.33);
  EXPECT_DOUBLE_EQ(validator.config().clearance_margin, 0.05);

  // _reason 为 nullptr 时也必须正常工作。
  minco::TrajectoryValidator other;
  EXPECT_TRUE(other.configure(MakeConfig(), nullptr));
  EXPECT_TRUE(other.configured());
}

TEST(TrajectoryValidatorTest, Configure_RejectsInvalidParameters)
{
  const auto expect_config_fail =
    [](const minco::TrajectoryValidatorConfig & _config, const char * _label) {
      minco::TrajectoryValidator validator;
      std::string reason;
      EXPECT_FALSE(validator.configure(_config, &reason)) << _label;
      EXPECT_FALSE(reason.empty()) << _label;
      EXPECT_FALSE(validator.configured()) << _label;
    };

  const minco::TrajectoryValidatorConfig base = MakeConfig();
  const double nan = std::numeric_limits<double>::quiet_NaN();

  minco::TrajectoryValidatorConfig bad = base;
  bad.robot_radius = 0.0;
  expect_config_fail(bad, "robot_radius = 0");

  bad = base;
  bad.robot_radius = -0.1;
  expect_config_fail(bad, "robot_radius < 0");

  bad = base;
  bad.robot_radius = nan;
  expect_config_fail(bad, "robot_radius = NaN");

  bad = base;
  bad.clearance_margin = -0.01;
  expect_config_fail(bad, "clearance_margin < 0");

  bad = base;
  bad.max_linear_speed = 0.0;
  expect_config_fail(bad, "max_linear_speed = 0");

  bad = base;
  bad.max_linear_speed = -1.0;
  expect_config_fail(bad, "max_linear_speed < 0");

  bad = base;
  bad.max_linear_accel = 0.0;
  expect_config_fail(bad, "max_linear_accel = 0");

  bad = base;
  bad.braking_deceleration = 0.0;
  expect_config_fail(bad, "braking_deceleration = 0");

  bad = base;
  bad.braking_deceleration = -0.5;
  expect_config_fail(bad, "braking_deceleration < 0");

  bad = base;
  bad.min_piece_duration = 0.0;
  expect_config_fail(bad, "min_piece_duration = 0");

  bad = base;
  bad.sample_dt = 0.0;
  expect_config_fail(bad, "sample_dt = 0");

  bad = base;
  bad.max_sample_spacing = 0.0;
  expect_config_fail(bad, "max_sample_spacing = 0");

  bad = base;
  bad.speed_tolerance_ratio = -0.1;
  expect_config_fail(bad, "speed_tolerance_ratio < 0");

  bad = base;
  bad.accel_tolerance_ratio = -0.1;
  expect_config_fail(bad, "accel_tolerance_ratio < 0");

  bad = base;
  bad.reaction_latency = -0.1;
  expect_config_fail(bad, "reaction_latency < 0");

  bad = base;
  bad.required_prefix_duration = -1.0;
  expect_config_fail(bad, "required_prefix_duration < 0");

  bad = base;
  bad.map_timeout = 0.0;
  expect_config_fail(bad, "map_timeout = 0");

  bad = base;
  bad.trajectory_max_age = 0.0;
  expect_config_fail(bad, "trajectory_max_age = 0");

  // 配置失败后校验器必须保持未配置状态。
  minco::TrajectoryValidator validator;
  std::string reason;
  bad = base;
  bad.robot_radius = 0.0;
  EXPECT_FALSE(validator.configure(bad, &reason));
  EXPECT_FALSE(validator.configured());
  EXPECT_TRUE(validator.configure(base, &reason));
  EXPECT_TRUE(validator.configured());
}

// ---------------------------------------------------------------------------
// computeExtrema：速度/加速度极值
// ---------------------------------------------------------------------------

TEST(TrajectoryValidatorTest, ComputeExtrema_ConstantVelocityHasZeroAcceleration)
{
  minco::TrajectoryValidator validator;
  std::string reason;
  ASSERT_TRUE(validator.configure(MakeConfig(), &reason)) << reason;

  // x 方向 0.4 m/s 匀速直线。
  const minco::Trajectory2D trajectory =
    MakeConstantVelocityTrajectory(P(-2.0, -3.0), P(0.4, 0.0), 10.0);

  double max_speed = -1.0;
  double max_acceleration = -1.0;
  ASSERT_TRUE(validator.computeExtrema(trajectory, max_speed, max_acceleration));
  EXPECT_NEAR(max_speed, 0.4, 0.4 * 1.0e-6);
  EXPECT_LT(max_acceleration, 1.0e-9);
}

TEST(TrajectoryValidatorTest, ComputeExtrema_ConstantAccelerationSegment)
{
  minco::TrajectoryValidator validator;
  std::string reason;
  ASSERT_TRUE(validator.configure(MakeConfig(), &reason)) << reason;

  // x(tau) = 0.5*a*tau^2，a = 0.4 m/s^2，时长 2 s：末端速度 0.8 m/s。
  const double accel = 0.4;
  const minco::Trajectory2D trajectory =
    MakeConstantAccelerationTrajectory(P(-2.0, -3.0), Eigen::Vector2d::Zero(), P(accel, 0.0), 2.0);

  double max_speed = 0.0;
  double max_acceleration = 0.0;
  ASSERT_TRUE(validator.computeExtrema(trajectory, max_speed, max_acceleration));
  EXPECT_NEAR(max_acceleration, accel, 1.0e-9);
  EXPECT_NEAR(max_speed, accel * 2.0, 1.0e-9);
}

TEST(TrajectoryValidatorTest, ComputeExtrema_DetectsInteriorSpeedPeak)
{
  minco::TrajectoryValidator validator;
  std::string reason;
  ASSERT_TRUE(validator.configure(MakeConfig(), &reason)) << reason;

  // 先加速后减速：v(tau) = 0.2 + 0.9*tau - 0.3*tau^2，在 tau = 1.5 s 处
  // 有内部峰值 v = 0.875 m/s；两端点速度仅 0.2 m/s，只比较端点会漏掉峰值。
  minco::Trajectory2D trajectory;
  minco::Trajectory2D::Coefficients x{};
  minco::Trajectory2D::Coefficients y{};
  x[0] = 0.0;
  x[1] = 0.2;
  x[2] = 0.45;
  x[3] = -0.1;
  y[0] = 0.0;
  ASSERT_TRUE(trajectory.addPiece(x, y, 3.0));
  EXPECT_NEAR(trajectory.velocityAt(0.0).norm(), 0.2, 1.0e-12);
  EXPECT_NEAR(trajectory.velocityAt(3.0).norm(), 0.2, 1.0e-12);

  double max_speed = 0.0;
  double max_acceleration = 0.0;
  ASSERT_TRUE(validator.computeExtrema(trajectory, max_speed, max_acceleration));
  EXPECT_NEAR(max_speed, 0.875, 1.0e-6);
  EXPECT_GT(max_speed, 0.8);
  // a(tau) = 0.9 - 0.6*tau，端点处最大 |a| = 0.9。
  EXPECT_NEAR(max_acceleration, 0.9, 1.0e-6);
}

TEST(TrajectoryValidatorTest, ComputeExtrema_DetectsInteriorAccelerationPeak)
{
  minco::TrajectoryValidator validator;
  std::string reason;
  ASSERT_TRUE(validator.configure(MakeConfig(), &reason)) << reason;

  // a(tau) = -0.45 + 1.8*tau - 0.6*tau^2，在 tau = 1.5 s 处达到内部峰值
  // |a| = 0.9 m/s^2，两端点仅 0.45 m/s^2。
  minco::Trajectory2D trajectory;
  minco::Trajectory2D::Coefficients x{};
  minco::Trajectory2D::Coefficients y{};
  x[2] = -0.225;
  x[3] = 0.3;
  x[4] = -0.05;
  ASSERT_TRUE(trajectory.addPiece(x, y, 3.0));
  EXPECT_NEAR(trajectory.accelerationAt(0.0).norm(), 0.45, 1.0e-12);
  EXPECT_NEAR(trajectory.accelerationAt(3.0).norm(), 0.45, 1.0e-12);
  EXPECT_NEAR(trajectory.accelerationAt(1.5).norm(), 0.9, 1.0e-12);

  double max_speed = 0.0;
  double max_acceleration = 0.0;
  ASSERT_TRUE(validator.computeExtrema(trajectory, max_speed, max_acceleration));
  EXPECT_NEAR(max_acceleration, 0.9, 1.0e-6);
  EXPECT_GT(max_acceleration, 0.5);
}

TEST(TrajectoryValidatorTest, ComputeExtrema_RejectsEmptyTrajectory)
{
  minco::TrajectoryValidator validator;
  std::string reason;
  ASSERT_TRUE(validator.configure(MakeConfig(), &reason)) << reason;

  const minco::Trajectory2D empty;
  double max_speed = 123.0;
  double max_acceleration = 123.0;
  EXPECT_FALSE(validator.computeExtrema(empty, max_speed, max_acceleration));
  EXPECT_DOUBLE_EQ(max_speed, 0.0);
  EXPECT_DOUBLE_EQ(max_acceleration, 0.0);
}

// ---------------------------------------------------------------------------
// checkCollision：扫掠净空
// ---------------------------------------------------------------------------

TEST(TrajectoryValidatorTest, CheckCollision_PassesFarFromObstacle)
{
  minco::TrajectoryValidator validator;
  std::string reason;
  const minco::TrajectoryValidatorConfig config = MakeConfig();
  ASSERT_TRUE(validator.configure(config, &reason)) << reason;

  const minco::Esdf2D esdf = MakeWallEsdf(kStamp);
  ASSERT_TRUE(esdf.valid());
  ASSERT_TRUE(esdf.query(-2.0, -3.0).valid);

  const minco::Trajectory2D trajectory = MakeSafeTrajectory();
  double min_clearance = 0.0;
  double first_violation = 0.0;
  EXPECT_TRUE(validator.checkCollision(
    trajectory, esdf, 0.0, trajectory.totalDuration(), min_clearance, first_violation));

  // 轨迹在 y = -3，距离墙下沿 y = -2 约 1 m（保守化后约 0.99 m）。
  EXPECT_GT(min_clearance, config.robot_radius + config.clearance_margin);
  EXPECT_GT(min_clearance, 0.9);
  EXPECT_LT(min_clearance, 1.05);
  // 无违例时首次违例时间等于扫描终点。
  EXPECT_NEAR(first_violation, trajectory.totalDuration(), 1.0e-9);
}

TEST(TrajectoryValidatorTest, CheckCollision_FailsThroughWallCenter)
{
  minco::TrajectoryValidator validator;
  std::string reason;
  const minco::TrajectoryValidatorConfig config = MakeConfig();
  ASSERT_TRUE(validator.configure(config, &reason)) << reason;

  const minco::Esdf2D esdf = MakeWallEsdf(kStamp);
  // 墙内的查询距离必须为负。
  EXPECT_LT(esdf.query(0.0, 0.0).distance, 0.0);

  // 从 (-2,0) 以 0.4 m/s 穿过墙中心到 (2,0)，总时长 10 s。
  const minco::Trajectory2D trajectory =
    MakeConstantVelocityTrajectory(P(-2.0, 0.0), P(0.4, 0.0), 10.0);
  double min_clearance = 0.0;
  double first_violation = 0.0;
  EXPECT_FALSE(validator.checkCollision(
    trajectory, esdf, 0.0, trajectory.totalDuration(), min_clearance, first_violation));

  const double total = trajectory.totalDuration();
  EXPECT_GT(first_violation, 0.3 * total);
  EXPECT_LT(first_violation, 0.7 * total);
  EXPECT_GT(min_clearance, 0.0);
}

TEST(TrajectoryValidatorTest, CheckCollision_FailsOutsideMap)
{
  minco::TrajectoryValidator validator;
  std::string reason;
  const minco::TrajectoryValidatorConfig config = MakeConfig();
  ASSERT_TRUE(validator.configure(config, &reason)) << reason;

  const minco::Esdf2D esdf = MakeWallEsdf(kStamp);
  EXPECT_TRUE(esdf.query(4.0, 0.0).valid);
  EXPECT_FALSE(esdf.query(5.2, 0.0).valid);
  EXPECT_FALSE(esdf.query(0.0, -5.4).valid);

  // 从 (3,0) 走到 (7,0)：越过地图右边界后查询无效，按不可通行处理。
  const minco::Trajectory2D trajectory =
    MakeConstantVelocityTrajectory(P(3.0, 0.0), P(0.4, 0.0), 10.0);
  double min_clearance = 0.0;
  double first_violation = 0.0;
  EXPECT_FALSE(validator.checkCollision(
    trajectory, esdf, 0.0, trajectory.totalDuration(), min_clearance, first_violation));

  EXPECT_TRUE(std::isinf(min_clearance));
  EXPECT_LT(min_clearance, 0.0);
  EXPECT_GT(first_violation, 4.9);
  EXPECT_LT(first_violation, 5.3);
}

TEST(TrajectoryValidatorTest, CheckCollision_RequiresRobotRadiusPlusMargin)
{
  const minco::Esdf2D esdf = MakeWallEsdf(kStamp);
  const minco::Trajectory2D trajectory = MakeSafeTrajectory();
  const double total = trajectory.totalDuration();

  // 实际最小净空约 0.99 m：半径 0.33 m 可通过。
  minco::TrajectoryValidatorConfig config = MakeConfig();
  {
    minco::TrajectoryValidator validator;
    std::string reason;
    ASSERT_TRUE(validator.configure(config, &reason)) << reason;
    double min_clearance = 0.0;
    double first_violation = 0.0;
    EXPECT_TRUE(
      validator.checkCollision(trajectory, esdf, 0.0, total, min_clearance, first_violation));
  }

  // 半径放大到超过实际净空后，同一条轨迹必须失败。
  config.robot_radius = 1.2;
  {
    minco::TrajectoryValidator validator;
    std::string reason;
    ASSERT_TRUE(validator.configure(config, &reason)) << reason;
    double min_clearance = 0.0;
    double first_violation = 0.0;
    EXPECT_FALSE(
      validator.checkCollision(trajectory, esdf, 0.0, total, min_clearance, first_violation));
  }

  // 净空要求 = robot_radius + clearance_margin：加大裕量同样会使轨迹失败。
  config = MakeConfig();
  config.clearance_margin = 1.0;
  {
    minco::TrajectoryValidator validator;
    std::string reason;
    ASSERT_TRUE(validator.configure(config, &reason)) << reason;
    double min_clearance = 0.0;
    double first_violation = 0.0;
    EXPECT_FALSE(
      validator.checkCollision(trajectory, esdf, 0.0, total, min_clearance, first_violation));
  }
}

TEST(TrajectoryValidatorTest, CheckCollision_RejectsEmptyTrajectoryOrInvalidEsdf)
{
  minco::TrajectoryValidator validator;
  std::string reason;
  ASSERT_TRUE(validator.configure(MakeConfig(), &reason)) << reason;

  const minco::Esdf2D esdf = MakeWallEsdf(kStamp);
  const minco::Trajectory2D empty;
  double min_clearance = 0.0;
  double first_violation = 0.0;
  EXPECT_FALSE(validator.checkCollision(empty, esdf, 0.0, 1.0, min_clearance, first_violation));

  const minco::Esdf2D invalid_esdf;
  const minco::Trajectory2D trajectory = MakeSafeTrajectory();
  EXPECT_FALSE(validator.checkCollision(
    trajectory, invalid_esdf, 0.0, trajectory.totalDuration(), min_clearance, first_violation));
}

// ---------------------------------------------------------------------------
// validate：完整验证
// ---------------------------------------------------------------------------

TEST(TrajectoryValidatorTest, Validate_PassesCompleteTrajectory)
{
  minco::TrajectoryValidator validator;
  std::string reason;
  const minco::TrajectoryValidatorConfig config = MakeConfig();
  ASSERT_TRUE(validator.configure(config, &reason)) << reason;

  const minco::Esdf2D esdf = MakeWallEsdf(kStamp);
  minco::Trajectory2D trajectory = MakeSafeTrajectory();
  trajectory.generated_stamp = kStamp;
  trajectory.valid_after = 0.0;
  trajectory.valid_until = 0.0;
  trajectory.terminal_is_global_goal = false;

  const minco::State2D state = MakeState(-2.0, -3.0, 0.0);
  minco::TrajectoryValidationReport report;
  ASSERT_TRUE(validator.validate(trajectory, esdf, state, kStamp, report)) << report.reason;

  EXPECT_TRUE(report.valid);
  EXPECT_TRUE(report.reason.empty());
  EXPECT_TRUE(report.coefficients_ok);
  EXPECT_TRUE(report.boundary_ok);
  EXPECT_TRUE(report.continuity_ok);
  EXPECT_TRUE(report.dynamics_ok);
  EXPECT_TRUE(report.collision_ok);
  EXPECT_TRUE(report.coverage_ok);
  EXPECT_TRUE(report.timing_ok);

  EXPECT_NEAR(report.max_speed, 0.4, 0.4 * 1.0e-6);
  EXPECT_LT(report.max_acceleration, 1.0e-9);
  EXPECT_GT(report.min_clearance, config.robot_radius + config.clearance_margin);
  EXPECT_LT(report.continuity_max_position, 1.0e-12);
  EXPECT_LT(report.continuity_max_velocity, 1.0e-12);
  EXPECT_LT(report.continuity_max_acceleration, 1.0e-12);

  EXPECT_NEAR(report.first_violation_time, trajectory.totalDuration(), 1.0e-9);
  EXPECT_NEAR(report.effective_prefix_duration, trajectory.totalDuration(), 1.0e-9);
  EXPECT_NEAR(report.required_stop_time, config.reaction_latency, 1.0e-12);
  EXPECT_NEAR(report.required_prefix_duration, config.required_prefix_duration, 1.0e-12);
}

TEST(TrajectoryValidatorTest, Validate_FailsWhenNotConfigured)
{
  const minco::TrajectoryValidator validator;
  const minco::Esdf2D esdf = MakeWallEsdf(kStamp);
  const minco::Trajectory2D trajectory = MakeSafeTrajectory();
  const minco::State2D state = MakeState(-2.0, -3.0, 0.0);

  minco::TrajectoryValidationReport report;
  EXPECT_FALSE(validator.validate(trajectory, esdf, state, kStamp, report));
  EXPECT_FALSE(report.valid);
  EXPECT_FALSE(report.reason.empty());
}

TEST(TrajectoryValidatorTest, Validate_RejectsSpeedOverLimit)
{
  minco::TrajectoryValidator validator;
  std::string reason;
  minco::TrajectoryValidatorConfig config = MakeConfig();
  config.max_linear_speed = 0.1;  // 轨迹实际 0.4 m/s
  ASSERT_TRUE(validator.configure(config, &reason)) << reason;

  const minco::Esdf2D esdf = MakeWallEsdf(kStamp);
  minco::Trajectory2D trajectory = MakeSafeTrajectory();
  trajectory.generated_stamp = kStamp;

  minco::TrajectoryValidationReport report;
  EXPECT_FALSE(validator.validate(trajectory, esdf, MakeState(-2.0, -3.0, 0.0), kStamp, report));
  EXPECT_FALSE(report.valid);
  EXPECT_TRUE(report.dynamics_ok);
  EXPECT_NE(report.reason.find("speed"), std::string::npos) << report.reason;
  EXPECT_NEAR(report.max_speed, 0.4, 0.4 * 1.0e-6);
}

TEST(TrajectoryValidatorTest, Validate_RejectsAccelerationOverLimit)
{
  minco::TrajectoryValidator validator;
  std::string reason;
  minco::TrajectoryValidatorConfig config = MakeConfig();
  config.max_linear_speed = 2.0;   // 避免先触发速度上限
  config.max_linear_accel = 0.05;  // 轨迹实际 0.4 m/s^2
  ASSERT_TRUE(validator.configure(config, &reason)) << reason;

  const minco::Esdf2D esdf = MakeWallEsdf(kStamp);
  minco::Trajectory2D trajectory =
    MakeConstantAccelerationTrajectory(P(-2.0, -3.0), Eigen::Vector2d::Zero(), P(0.4, 0.0), 2.0);
  trajectory.generated_stamp = kStamp;

  minco::TrajectoryValidationReport report;
  EXPECT_FALSE(validator.validate(trajectory, esdf, MakeState(-2.0, -3.0, 0.0), kStamp, report));
  EXPECT_FALSE(report.valid);
  EXPECT_NE(report.reason.find("acceleration"), std::string::npos) << report.reason;
  EXPECT_NEAR(report.max_acceleration, 0.4, 1.0e-9);
}

TEST(TrajectoryValidatorTest, Validate_RejectsPieceDurationBelowMinimum)
{
  minco::TrajectoryValidator validator;
  std::string reason;
  const minco::TrajectoryValidatorConfig config = MakeConfig();
  ASSERT_TRUE(validator.configure(config, &reason)) << reason;

  const minco::Esdf2D esdf = MakeWallEsdf(kStamp);
  const minco::Trajectory2D trajectory =
    MakeConstantVelocityTrajectory(P(-2.0, -3.0), P(0.4, 0.0), 0.02);
  ASSERT_LT(trajectory.totalDuration(), config.min_piece_duration);

  minco::TrajectoryValidationReport report;
  EXPECT_FALSE(validator.validate(trajectory, esdf, MakeState(-2.0, -3.0, 0.0), kStamp, report));
  EXPECT_FALSE(report.valid);
  EXPECT_NE(report.reason.find("duration"), std::string::npos) << report.reason;
}

TEST(TrajectoryValidatorTest, Validate_RejectsCollidingTrajectory)
{
  minco::TrajectoryValidator validator;
  std::string reason;
  const minco::TrajectoryValidatorConfig config = MakeConfig();
  ASSERT_TRUE(validator.configure(config, &reason)) << reason;

  const minco::Esdf2D esdf = MakeWallEsdf(kStamp);
  minco::Trajectory2D trajectory = MakeConstantVelocityTrajectory(P(-2.0, 0.0), P(0.4, 0.0), 10.0);
  trajectory.generated_stamp = kStamp;

  minco::TrajectoryValidationReport report;
  EXPECT_FALSE(validator.validate(trajectory, esdf, MakeState(-2.0, 0.0, 0.0), kStamp, report));
  EXPECT_FALSE(report.valid);
  EXPECT_TRUE(report.dynamics_ok);
  EXPECT_FALSE(report.collision_ok);
  EXPECT_NE(report.reason.find("collides"), std::string::npos) << report.reason;
  EXPECT_GT(report.first_violation_time, 0.0);
  EXPECT_LT(report.first_violation_time, trajectory.totalDuration());
}

TEST(TrajectoryValidatorTest, Validate_RejectsStaleDistanceField)
{
  minco::TrajectoryValidator validator;
  std::string reason;
  const minco::TrajectoryValidatorConfig config = MakeConfig();
  ASSERT_TRUE(validator.configure(config, &reason)) << reason;

  const minco::Esdf2D esdf = MakeWallEsdf(kStamp);
  minco::Trajectory2D trajectory = MakeSafeTrajectory();
  // 地图已过期：now 比地图时间戳晚超过 map_timeout。
  const double now = kStamp + config.map_timeout + 1.0;
  trajectory.generated_stamp = now;
  trajectory.valid_after = 0.0;

  minco::TrajectoryValidationReport report;
  EXPECT_FALSE(validator.validate(trajectory, esdf, MakeState(-2.0, -3.0, 0.0), now, report));
  EXPECT_FALSE(report.valid);
  EXPECT_FALSE(report.timing_ok);
  EXPECT_TRUE(report.collision_ok);
  EXPECT_NE(report.reason.find("map_timeout"), std::string::npos) << report.reason;
}

TEST(TrajectoryValidatorTest, Validate_RejectsTrajectoryNotYetValid)
{
  minco::TrajectoryValidator validator;
  std::string reason;
  const minco::TrajectoryValidatorConfig config = MakeConfig();
  ASSERT_TRUE(validator.configure(config, &reason)) << reason;

  const minco::Esdf2D esdf = MakeWallEsdf(kStamp);
  minco::Trajectory2D trajectory = MakeSafeTrajectory();
  trajectory.generated_stamp = kStamp;
  trajectory.valid_after = kStamp + 0.5;  // 未来切换时刻
  // valid_until 必须不早于 valid_after，否则 sanityCheck 会先判失败。
  trajectory.valid_until = kStamp + 2.0;

  minco::TrajectoryValidationReport report;
  EXPECT_FALSE(validator.validate(trajectory, esdf, MakeState(-2.0, -3.0, 0.0), kStamp, report));
  EXPECT_FALSE(report.valid);
  EXPECT_FALSE(report.timing_ok);
  EXPECT_TRUE(report.coefficients_ok);
  EXPECT_NE(report.reason.find("not valid yet"), std::string::npos) << report.reason;
}

TEST(TrajectoryValidatorTest, Validate_RejectsIncoherentValidityWindow)
{
  // valid_until 早于 valid_after 属于自相矛盾的时间元数据，必须在系数自检阶段被拒绝。
  minco::TrajectoryValidator validator;
  std::string reason;
  const minco::TrajectoryValidatorConfig config = MakeConfig();
  ASSERT_TRUE(validator.configure(config, &reason)) << reason;

  const minco::Esdf2D esdf = MakeWallEsdf(kStamp);
  minco::Trajectory2D trajectory = MakeSafeTrajectory();
  trajectory.generated_stamp = kStamp;
  trajectory.valid_after = kStamp + 1.0;
  trajectory.valid_until = kStamp + 0.5;

  minco::TrajectoryValidationReport report;
  EXPECT_FALSE(validator.validate(trajectory, esdf, MakeState(-2.0, -3.0, 0.0), kStamp, report));
  EXPECT_FALSE(report.valid);
  EXPECT_FALSE(report.coefficients_ok);
}

TEST(TrajectoryValidatorTest, Validate_RejectsTooOldTrajectory)
{
  minco::TrajectoryValidator validator;
  std::string reason;
  const minco::TrajectoryValidatorConfig config = MakeConfig();
  ASSERT_TRUE(validator.configure(config, &reason)) << reason;

  const minco::Esdf2D esdf = MakeWallEsdf(kStamp);
  minco::Trajectory2D trajectory = MakeSafeTrajectory();
  trajectory.valid_after = 0.0;
  trajectory.generated_stamp = kStamp - config.trajectory_max_age - 1.0;

  minco::TrajectoryValidationReport report;
  EXPECT_FALSE(validator.validate(trajectory, esdf, MakeState(-2.0, -3.0, 0.0), kStamp, report));
  EXPECT_FALSE(report.valid);
  EXPECT_FALSE(report.timing_ok);
  EXPECT_NE(report.reason.find("trajectory_max_age"), std::string::npos) << report.reason;
}

TEST(TrajectoryValidatorTest, Validate_RejectsExpiredValidityWindow)
{
  minco::TrajectoryValidator validator;
  std::string reason;
  const minco::TrajectoryValidatorConfig config = MakeConfig();
  ASSERT_TRUE(validator.configure(config, &reason)) << reason;

  const minco::Esdf2D esdf = MakeWallEsdf(kStamp);
  minco::Trajectory2D trajectory = MakeSafeTrajectory();
  trajectory.generated_stamp = kStamp;
  trajectory.valid_after = 0.0;
  trajectory.valid_until = kStamp - 0.5;  // 已过期

  minco::TrajectoryValidationReport report;
  EXPECT_FALSE(validator.validate(trajectory, esdf, MakeState(-2.0, -3.0, 0.0), kStamp, report));
  EXPECT_FALSE(report.valid);
  EXPECT_FALSE(report.timing_ok);
  EXPECT_NE(report.reason.find("expired"), std::string::npos) << report.reason;
}

TEST(TrajectoryValidatorTest, Validate_RejectsGlobalGoalWithNonzeroEndVelocity)
{
  minco::TrajectoryValidator validator;
  std::string reason;
  const minco::TrajectoryValidatorConfig config = MakeConfig();
  ASSERT_TRUE(validator.configure(config, &reason)) << reason;

  const minco::Esdf2D esdf = MakeWallEsdf(kStamp);
  minco::Trajectory2D trajectory = MakeSafeTrajectory();
  trajectory.generated_stamp = kStamp;
  trajectory.valid_after = 0.0;
  trajectory.terminal_is_global_goal = true;  // 但末端速度为 0.4 m/s
  ASSERT_GT(trajectory.endVelocity().norm(), 0.05);

  minco::TrajectoryValidationReport report;
  EXPECT_FALSE(validator.validate(trajectory, esdf, MakeState(-2.0, -3.0, 0.0), kStamp, report));
  EXPECT_FALSE(report.valid);
  EXPECT_FALSE(report.timing_ok);
  EXPECT_NE(report.reason.find("does not end at rest"), std::string::npos) << report.reason;
}

TEST(TrajectoryValidatorTest, Validate_RejectsInsufficientCoverage)
{
  minco::TrajectoryValidator validator;
  std::string reason;
  minco::TrajectoryValidatorConfig config = MakeConfig();
  config.required_prefix_duration = 20.0;  // 大于轨迹总时长 10 s
  ASSERT_TRUE(validator.configure(config, &reason)) << reason;

  const minco::Esdf2D esdf = MakeWallEsdf(kStamp);
  minco::Trajectory2D trajectory = MakeSafeTrajectory();
  trajectory.generated_stamp = kStamp;
  trajectory.valid_after = 0.0;

  minco::TrajectoryValidationReport report;
  EXPECT_FALSE(validator.validate(trajectory, esdf, MakeState(-2.0, -3.0, 0.0), kStamp, report));
  EXPECT_FALSE(report.valid);
  EXPECT_FALSE(report.coverage_ok);
  EXPECT_LT(report.effective_prefix_duration, report.required_prefix_duration);
  EXPECT_NEAR(report.effective_prefix_duration, trajectory.totalDuration(), 1.0e-9);
  EXPECT_NE(report.reason.find("prefix"), std::string::npos) << report.reason;
}

TEST(TrajectoryValidatorTest, Validate_RequiredStopTimeGrowsWithSpeed)
{
  minco::TrajectoryValidator validator;
  std::string reason;
  const minco::TrajectoryValidatorConfig config = MakeConfig();
  ASSERT_TRUE(validator.configure(config, &reason)) << reason;

  const minco::Esdf2D esdf = MakeWallEsdf(kStamp);
  minco::Trajectory2D trajectory = MakeSafeTrajectory();
  trajectory.generated_stamp = kStamp;
  trajectory.valid_after = 0.0;

  minco::TrajectoryValidationReport slow_report;
  ASSERT_TRUE(validator.validate(trajectory, esdf, MakeState(-2.0, -3.0, 0.0), kStamp, slow_report))
    << slow_report.reason;

  minco::TrajectoryValidationReport fast_report;
  ASSERT_TRUE(validator.validate(trajectory, esdf, MakeState(-2.0, -3.0, 0.4), kStamp, fast_report))
    << fast_report.reason;

  EXPECT_NEAR(slow_report.required_stop_time, config.reaction_latency, 1.0e-12);
  EXPECT_NEAR(
    fast_report.required_stop_time, config.reaction_latency + 0.4 / config.braking_deceleration,
    1.0e-12);
  EXPECT_GT(fast_report.required_stop_time, slow_report.required_stop_time);
  EXPECT_GT(fast_report.required_prefix_duration, slow_report.required_prefix_duration);

  // 速度足够大时，所需停车时间超过可执行前缀，覆盖检查必须失败。
  minco::TrajectoryValidationReport very_fast_report;
  EXPECT_FALSE(
    validator.validate(trajectory, esdf, MakeState(-2.0, -3.0, 6.0), kStamp, very_fast_report));
  EXPECT_FALSE(very_fast_report.valid);
  EXPECT_FALSE(very_fast_report.coverage_ok);
  EXPECT_GT(very_fast_report.required_stop_time, trajectory.totalDuration());
}
