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
#include <vector>

#include "srm27_minco_core/trajectory_2d.hpp"
#include "srm27_minco_core/trajectory_initializer.hpp"

namespace minco = srm27_minco_core;

TEST(TrajectoryInitializerTest, MovingTerminalRespectsSpeedAndReachability)
{
  minco::TrajectoryInitializer::Config config;
  config.terminal_is_global_goal = false;
  config.terminal_speed = 10.0;
  minco::TrajectoryInitialGuess guess;
  ASSERT_TRUE(minco::TrajectoryInitializer::initialize(
    {Eigen::Vector2d(0, 0), Eigen::Vector2d(0.02, 0)}, Eigen::Vector2d::Zero(), config, guess,
    nullptr));
  EXPECT_LE(guess.tail_velocity.norm(), config.max_speed);
  EXPECT_LE(guess.tail_velocity.norm(), std::sqrt(2.0 * config.max_accel * 0.02) + 1.0e-9);
  EXPECT_GT(guess.tail_velocity.x(), 0.0);
  config.terminal_speed = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(minco::TrajectoryInitializer::initialize(
    {Eigen::Vector2d(0, 0), Eigen::Vector2d(1, 0)}, Eigen::Vector2d::Zero(), config, guess,
    nullptr));
}

namespace
{

/// \brief 便捷构造二维点。
Eigen::Vector2d P(const double _x, const double _y) { return Eigen::Vector2d(_x, _y); }

/// \brief 初值总时长（各段时长之和）。
double SumDurations(const minco::TrajectoryInitialGuess & _guess)
{
  double total = 0.0;
  for (const double duration : _guess.durations) {
    total += duration;
  }
  return total;
}

/// \brief 以绝对容差比较两个向量。
void ExpectNearVector(
  const Eigen::Vector2d & _actual, const Eigen::Vector2d & _expected, const double _tolerance)
{
  EXPECT_NEAR(_actual.x(), _expected.x(), _tolerance);
  EXPECT_NEAR(_actual.y(), _expected.y(), _tolerance);
}

/// \brief 生成折线 (0,0) -> (5,0)，长度 5 m 的直线。
std::vector<Eigen::Vector2d> StraightPolyline()
{
  return std::vector<Eigen::Vector2d>{P(0.0, 0.0), P(5.0, 0.0)};
}

/// \brief 生成折线 (0,0) -> (2.5,0) -> (2.5,2.5)，总长度同为 5 m 的 90 度折角路径。
std::vector<Eigen::Vector2d> RightAnglePolyline()
{
  return std::vector<Eigen::Vector2d>{P(0.0, 0.0), P(2.5, 0.0), P(2.5, 2.5)};
}

}  // namespace

// ---------------------------------------------------------------------------
// cleanAndResample：清理与弧长重采样
// ---------------------------------------------------------------------------

TEST(TrajectoryInitializerTest, CleanAndResample_DropsDuplicatePoints)
{
  // 完全重合点与小于 duplicate_epsilon 的极近点都应被丢弃。
  const std::vector<Eigen::Vector2d> input{
    P(0.0, 0.0), P(0.0, 0.0), P(0.0002, 0.0001), P(1.0, 0.0), P(1.0, 0.0)};
  const std::vector<Eigen::Vector2d> output =
    minco::TrajectoryInitializer::cleanAndResample(input, 1.0e-3, 0.5);

  ASSERT_EQ(output.size(), 3u);
  ExpectNearVector(output[0], P(0.0, 0.0), 1.0e-12);
  ExpectNearVector(output[1], P(0.5, 0.0), 1.0e-12);
  ExpectNearVector(output[2], P(1.0, 0.0), 1.0e-12);
}

TEST(TrajectoryInitializerTest, CleanAndResample_DropsCollinearJitter)
{
  // 中间点在首尾连线上的偏离量远小于 epsilon：应被当作抖动删除。
  const std::vector<Eigen::Vector2d> input{P(0.0, 0.0), P(1.0, 0.0001), P(2.0, 0.0)};
  const std::vector<Eigen::Vector2d> output =
    minco::TrajectoryInitializer::cleanAndResample(input, 1.0e-3, 0.5);

  ASSERT_EQ(output.size(), 5u);
  for (const Eigen::Vector2d & point : output) {
    EXPECT_NEAR(point.y(), 0.0, 1.0e-12);
  }
  ExpectNearVector(output.front(), P(0.0, 0.0), 1.0e-12);
  ExpectNearVector(output.back(), P(2.0, 0.0), 1.0e-12);
}

TEST(TrajectoryInitializerTest, CleanAndResample_DropsShortBacktrack)
{
  // (1,0) -> (0.9992,0.0008) 是短小折返：绕行量 < 2*epsilon 且偏离量 < epsilon，应被删除。
  const std::vector<Eigen::Vector2d> input{
    P(0.0, 0.0), P(1.0, 0.0), P(0.9992, 0.0008), P(2.0, 0.0)};
  const std::vector<Eigen::Vector2d> output =
    minco::TrajectoryInitializer::cleanAndResample(input, 1.0e-3, 0.5);

  ASSERT_EQ(output.size(), 5u);
  for (const Eigen::Vector2d & point : output) {
    EXPECT_NEAR(point.y(), 0.0, 1.0e-12);
  }
  ExpectNearVector(output.front(), P(0.0, 0.0), 1.0e-12);
  ExpectNearVector(output.back(), P(2.0, 0.0), 1.0e-12);
}

TEST(TrajectoryInitializerTest, CleanAndResample_SpacingApproximatesStep)
{
  const double step = 0.25;
  const std::vector<Eigen::Vector2d> input{P(0.0, 0.0), P(1.03, 0.0)};
  const std::vector<Eigen::Vector2d> output =
    minco::TrajectoryInitializer::cleanAndResample(input, 1.0e-3, step);

  // 1.03 m 上按 0.25 m 采样得到 0.25/0.50/0.75/1.00，末点 1.03 原样追加。
  ASSERT_EQ(output.size(), 6u);
  for (std::size_t i = 0; i + 1 < output.size(); ++i) {
    const double spacing = (output[i + 1] - output[i]).norm();
    if (i + 2 < output.size()) {
      EXPECT_NEAR(spacing, step, 1.0e-9) << "第 " << i << " 个间隔";
    } else {
      EXPECT_NEAR(spacing, 0.03, 1.0e-9) << "末段为残余长度";
    }
  }
  ExpectNearVector(output.back(), P(1.03, 0.0), 0.0);
}

TEST(TrajectoryInitializerTest, CleanAndResample_KeepsPointsOnPolylineAcrossCorner)
{
  const std::vector<Eigen::Vector2d> input{P(0.0, 0.0), P(0.6, 0.0), P(0.6, 0.7)};
  const double step = 0.25;
  const std::vector<Eigen::Vector2d> output =
    minco::TrajectoryInitializer::cleanAndResample(input, 1.0e-3, step);

  ASSERT_EQ(output.size(), 7u);
  ExpectNearVector(output.front(), P(0.0, 0.0), 1.0e-12);
  ExpectNearVector(output.back(), P(0.6, 0.7), 1.0e-12);

  // 每个重采样点都必须落在输入折线上（折角两条边之一）。
  for (const Eigen::Vector2d & point : output) {
    const bool on_horizontal =
      std::abs(point.y()) < 1.0e-12 && point.x() >= -1.0e-12 && point.x() <= 0.6 + 1.0e-12;
    const bool on_vertical =
      std::abs(point.x() - 0.6) < 1.0e-12 && point.y() >= -1.0e-12 && point.y() <= 0.7 + 1.0e-12;
    EXPECT_TRUE(on_horizontal || on_vertical) << point.transpose();
  }

  // 相邻点间距不超过 step；跨折角的那一段会切角，因此整条折线的弦长略小于输入长度。
  double length = 0.0;
  for (std::size_t i = 0; i + 1 < output.size(); ++i) {
    const double spacing = (output[i + 1] - output[i]).norm();
    EXPECT_LE(spacing, step + 1.0e-9);
    length += spacing;
  }
  EXPECT_LT(length, 1.3);
  EXPECT_GT(length, 1.3 - step);
}

TEST(TrajectoryInitializerTest, CleanAndResample_RejectsInvalidInput)
{
  const std::vector<Eigen::Vector2d> two_points{P(0.0, 0.0), P(1.0, 0.0)};
  const double nan = std::numeric_limits<double>::quiet_NaN();

  // 点数不足 2。
  EXPECT_TRUE(minco::TrajectoryInitializer::cleanAndResample({}, 1.0e-3, 0.1).empty());
  EXPECT_TRUE(minco::TrajectoryInitializer::cleanAndResample({P(0.0, 0.0)}, 1.0e-3, 0.1).empty());

  // 全部点重合时清理后不足 2 点。
  const std::vector<Eigen::Vector2d> collapsed{P(1.0, 1.0), P(1.0, 1.0)};
  EXPECT_TRUE(minco::TrajectoryInitializer::cleanAndResample(collapsed, 1.0e-3, 0.1).empty());

  // step 非正或非有限。
  EXPECT_TRUE(minco::TrajectoryInitializer::cleanAndResample(two_points, 1.0e-3, 0.0).empty());
  EXPECT_TRUE(minco::TrajectoryInitializer::cleanAndResample(two_points, 1.0e-3, -1.0).empty());
  EXPECT_TRUE(minco::TrajectoryInitializer::cleanAndResample(two_points, 1.0e-3, nan).empty());

  // duplicate_epsilon 为负或非有限。
  EXPECT_TRUE(minco::TrajectoryInitializer::cleanAndResample(two_points, -1.0e-3, 0.1).empty());
  EXPECT_TRUE(minco::TrajectoryInitializer::cleanAndResample(two_points, nan, 0.1).empty());
}

// ---------------------------------------------------------------------------
// initialize：折线 -> 路标点 + 段时长初值
// ---------------------------------------------------------------------------

TEST(TrajectoryInitializerTest, Initialize_StraightPolylineProducesValidGuess)
{
  const std::vector<Eigen::Vector2d> polyline = StraightPolyline();
  const minco::TrajectoryInitializer::Config config;
  minco::TrajectoryInitialGuess guess;
  std::string reason;

  ASSERT_TRUE(minco::TrajectoryInitializer::initialize(
    polyline, Eigen::Vector2d::Zero(), config, guess, &reason))
    << reason;

  EXPECT_EQ(guess.pieceCount(), static_cast<int>(guess.durations.size()));
  EXPECT_EQ(guess.waypoints.size(), static_cast<std::size_t>(guess.pieceCount()) + 1);
  EXPECT_GE(guess.pieceCount(), config.min_pieces);

  for (const double duration : guess.durations) {
    EXPECT_GE(duration, config.min_piece_duration);
    EXPECT_LE(duration, config.max_piece_duration);
  }

  // 首尾路标点必须与折线首尾一致。
  ExpectNearVector(guess.waypoints.front(), polyline.front(), 1.0e-12);
  ExpectNearVector(guess.waypoints.back(), polyline.back(), 1.0e-12);
  ExpectNearVector(guess.head_position, polyline.front(), 1.0e-12);
  ExpectNearVector(guess.tail_position, polyline.back(), 1.0e-12);

  // 起点速度与输入一致；终点为全局目标时末端速度为零、两端加速度为零。
  ExpectNearVector(guess.head_velocity, Eigen::Vector2d::Zero(), 0.0);
  EXPECT_TRUE(guess.tail_velocity.isZero(0.0));
  EXPECT_TRUE(guess.head_acceleration.isZero(0.0));
  EXPECT_TRUE(guess.tail_acceleration.isZero(0.0));
  EXPECT_TRUE(config.terminal_is_global_goal);

  // 路标点应沿直线单调递增。
  for (std::size_t i = 0; i + 1 < guess.waypoints.size(); ++i) {
    EXPECT_GT(guess.waypoints[i + 1].x(), guess.waypoints[i].x());
    EXPECT_NEAR(guess.waypoints[i].y(), 0.0, 1.0e-12);
  }
  EXPECT_GT(SumDurations(guess), 0.0);
}

TEST(TrajectoryInitializerTest, Initialize_NonzeroHeadVelocityShortensTotalDuration)
{
  const std::vector<Eigen::Vector2d> polyline = StraightPolyline();
  const minco::TrajectoryInitializer::Config config;

  minco::TrajectoryInitialGuess from_rest;
  std::string reason;
  ASSERT_TRUE(minco::TrajectoryInitializer::initialize(
    polyline, Eigen::Vector2d::Zero(), config, from_rest, &reason))
    << reason;

  const Eigen::Vector2d head_velocity(0.3, 0.0);
  minco::TrajectoryInitialGuess from_motion;
  ASSERT_TRUE(
    minco::TrajectoryInitializer::initialize(polyline, head_velocity, config, from_motion, &reason))
    << reason;

  // 起点速度沿路径方向：前向扫描允许更快加速，总时长必须更短。
  ExpectNearVector(from_motion.head_velocity, head_velocity, 0.0);
  EXPECT_LT(SumDurations(from_motion), SumDurations(from_rest));

  // 起点速度不能超过场景上限。
  minco::TrajectoryInitialGuess clipped;
  ASSERT_TRUE(minco::TrajectoryInitializer::initialize(
    polyline, Eigen::Vector2d(1.5, 0.0), config, clipped, &reason))
    << reason;
  ExpectNearVector(clipped.head_velocity, Eigen::Vector2d(1.5, 0.0), 0.0);
}

TEST(TrajectoryInitializerTest, Initialize_SharpCornerIncreasesTotalDuration)
{
  // 两条折线长度相同（5 m），一条是直线，一条含 90 度折角；折角处必须降速，
  // 因此折角路径的总时长更长。
  minco::TrajectoryInitializer::Config config;
  config.max_pieces = 64;  // 放开段数上限，避免时长被 max_piece_duration 截断影响比较

  minco::TrajectoryInitialGuess straight;
  minco::TrajectoryInitialGuess corner;
  std::string reason;
  ASSERT_TRUE(minco::TrajectoryInitializer::initialize(
    StraightPolyline(), Eigen::Vector2d::Zero(), config, straight, &reason))
    << reason;
  ASSERT_TRUE(minco::TrajectoryInitializer::initialize(
    RightAnglePolyline(), Eigen::Vector2d::Zero(), config, corner, &reason))
    << reason;

  for (const double duration : corner.durations) {
    EXPECT_GE(duration, config.min_piece_duration);
    EXPECT_LE(duration, config.max_piece_duration);
  }
  ExpectNearVector(corner.waypoints.front(), P(0.0, 0.0), 1.0e-12);
  ExpectNearVector(corner.waypoints.back(), P(2.5, 2.5), 1.0e-12);

  EXPECT_GT(SumDurations(corner), SumDurations(straight));

  // 折角路径的转角确实变慢：总时长更长说明转角速度上限生效。
  EXPECT_GT(SumDurations(corner) - SumDurations(straight), 0.1);
}

TEST(TrajectoryInitializerTest, Initialize_RejectsDegenerateInput)
{
  const minco::TrajectoryInitializer::Config config;
  minco::TrajectoryInitialGuess guess;
  std::string reason;

  const auto expect_fail =
    [&guess, &reason](
      const std::vector<Eigen::Vector2d> & _polyline, const Eigen::Vector2d & _head_velocity,
      const minco::TrajectoryInitializer::Config & _config, const char * _label) {
      reason.clear();
      EXPECT_FALSE(minco::TrajectoryInitializer::initialize(
        _polyline, _head_velocity, _config, guess, &reason))
        << _label;
      EXPECT_FALSE(reason.empty()) << _label;
    };

  // 空折线与单点折线。
  expect_fail({}, Eigen::Vector2d::Zero(), config, "empty polyline");
  expect_fail({P(1.0, 1.0)}, Eigen::Vector2d::Zero(), config, "single point");

  // 零长度折线（清理后只剩一个点）。
  expect_fail({P(1.0, 1.0), P(1.0, 1.0)}, Eigen::Vector2d::Zero(), config, "zero length");

  // 非法配置。
  minco::TrajectoryInitializer::Config bad = config;
  bad.max_speed = 0.0;
  expect_fail(StraightPolyline(), Eigen::Vector2d::Zero(), bad, "max_speed = 0");

  bad = config;
  bad.max_accel = -1.0;
  expect_fail(StraightPolyline(), Eigen::Vector2d::Zero(), bad, "max_accel < 0");

  bad = config;
  bad.max_brake = 0.0;
  expect_fail(StraightPolyline(), Eigen::Vector2d::Zero(), bad, "max_brake = 0");

  bad = config;
  bad.resample_step = 0.0;
  expect_fail(StraightPolyline(), Eigen::Vector2d::Zero(), bad, "resample_step = 0");

  bad = config;
  bad.min_piece_duration = 0.0;
  expect_fail(StraightPolyline(), Eigen::Vector2d::Zero(), bad, "min_piece_duration = 0");

  bad = config;
  bad.max_piece_duration = 0.01;
  bad.min_piece_duration = 0.5;
  expect_fail(StraightPolyline(), Eigen::Vector2d::Zero(), bad, "max < min piece duration");

  bad = config;
  bad.nominal_piece_duration = 0.0;
  expect_fail(StraightPolyline(), Eigen::Vector2d::Zero(), bad, "nominal_piece_duration = 0");

  // 起点速度非有限。
  const double nan = std::numeric_limits<double>::quiet_NaN();
  expect_fail(StraightPolyline(), Eigen::Vector2d(nan, 0.0), config, "NaN head velocity");

  // _reason 为 nullptr 时同样返回 false 且不崩溃。
  EXPECT_FALSE(
    minco::TrajectoryInitializer::initialize({}, Eigen::Vector2d::Zero(), config, guess, nullptr));
}

// ---------------------------------------------------------------------------
// buildInitialTrajectory：初值 -> 分段五次多项式轨迹
// ---------------------------------------------------------------------------

TEST(TrajectoryInitializerTest, BuildInitialTrajectory_MatchesGuessBoundaryAndWaypoints)
{
  const std::vector<Eigen::Vector2d> polyline{P(0.0, 0.0), P(3.0, 0.0), P(3.0, 3.0)};
  const minco::TrajectoryInitializer::Config config;
  minco::TrajectoryInitialGuess guess;
  std::string reason;
  ASSERT_TRUE(minco::TrajectoryInitializer::initialize(
    polyline, Eigen::Vector2d::Zero(), config, guess, &reason))
    << reason;

  minco::Trajectory2D trajectory;
  ASSERT_TRUE(minco::TrajectoryInitializer::buildInitialTrajectory(guess, trajectory));

  std::string sanity_reason;
  EXPECT_TRUE(trajectory.sanityCheck(&sanity_reason)) << sanity_reason;

  // 段数与段时长与初值一致。
  EXPECT_EQ(trajectory.pieceCount(), guess.pieceCount());
  ASSERT_EQ(trajectory.durations().size(), guess.durations.size());
  for (std::size_t i = 0; i < guess.durations.size(); ++i) {
    EXPECT_DOUBLE_EQ(trajectory.durations()[i], guess.durations[i]);
  }

  // 边界条件：p/v/a 在两端与初值一致。
  ExpectNearVector(trajectory.startPosition(), guess.head_position, 1.0e-8);
  ExpectNearVector(trajectory.startVelocity(), guess.head_velocity, 1.0e-8);
  ExpectNearVector(trajectory.startAcceleration(), guess.head_acceleration, 1.0e-8);
  ExpectNearVector(trajectory.endPosition(), guess.tail_position, 1.0e-8);
  ExpectNearVector(trajectory.endVelocity(), guess.tail_velocity, 1.0e-8);
  ExpectNearVector(trajectory.endAcceleration(), guess.tail_acceleration, 1.0e-8);

  // 经过点：第 i 个内部路标点位于第 i 段结束时刻。
  double elapsed = 0.0;
  for (int i = 1; i < guess.pieceCount(); ++i) {
    elapsed += guess.durations[static_cast<std::size_t>(i - 1)];
    ExpectNearVector(
      trajectory.positionAt(elapsed), guess.waypoints[static_cast<std::size_t>(i)], 1.0e-8);
  }

  // 折角位于路径中点，应落在某个路标点上并被轨迹精确经过。
  ExpectNearVector(trajectory.positionAt(trajectory.totalDuration() * 0.5), P(3.0, 0.0), 1.0e-8);
}

TEST(TrajectoryInitializerTest, BuildInitialTrajectory_StraightLineKeepsDurationSum)
{
  const minco::TrajectoryInitializer::Config config;
  minco::TrajectoryInitialGuess guess;
  std::string reason;
  ASSERT_TRUE(minco::TrajectoryInitializer::initialize(
    StraightPolyline(), Eigen::Vector2d(0.2, 0.0), config, guess, &reason))
    << reason;

  minco::Trajectory2D trajectory;
  ASSERT_TRUE(minco::TrajectoryInitializer::buildInitialTrajectory(guess, trajectory));

  double sum = 0.0;
  for (const double duration : trajectory.durations()) {
    sum += duration;
  }
  EXPECT_NEAR(trajectory.totalDuration(), sum, 1.0e-12);
  EXPECT_NEAR(trajectory.totalDuration(), SumDurations(guess), 1.0e-12);
  ExpectNearVector(trajectory.startVelocity(), Eigen::Vector2d(0.2, 0.0), 1.0e-8);
  ExpectNearVector(trajectory.endPosition(), P(5.0, 0.0), 1.0e-8);
}

TEST(TrajectoryInitializerTest, BuildInitialTrajectory_RejectsMalformedGuess)
{
  const minco::TrajectoryInitializer::Config config;
  minco::TrajectoryInitialGuess guess;
  std::string reason;
  ASSERT_TRUE(minco::TrajectoryInitializer::initialize(
    StraightPolyline(), Eigen::Vector2d::Zero(), config, guess, &reason))
    << reason;

  minco::Trajectory2D trajectory;

  // 段数为 0。
  minco::TrajectoryInitialGuess empty_guess;
  EXPECT_FALSE(minco::TrajectoryInitializer::buildInitialTrajectory(empty_guess, trajectory));

  // 路标点数量与段数不匹配。
  minco::TrajectoryInitialGuess mismatched = guess;
  mismatched.waypoints.pop_back();
  EXPECT_FALSE(minco::TrajectoryInitializer::buildInitialTrajectory(mismatched, trajectory));

  // 段时长非正。
  minco::TrajectoryInitialGuess zero_duration = guess;
  zero_duration.durations.front() = 0.0;
  EXPECT_FALSE(minco::TrajectoryInitializer::buildInitialTrajectory(zero_duration, trajectory));

  // 路标点非有限。
  minco::TrajectoryInitialGuess bad_point = guess;
  bad_point.waypoints.front() = Eigen::Vector2d(std::numeric_limits<double>::quiet_NaN(), 0.0);
  EXPECT_FALSE(minco::TrajectoryInitializer::buildInitialTrajectory(bad_point, trajectory));

  // 合法初值仍然可以成功构造，说明上面的失败不是对象被破坏导致。
  EXPECT_TRUE(minco::TrajectoryInitializer::buildInitialTrajectory(guess, trajectory));
  EXPECT_FALSE(trajectory.empty());
}
