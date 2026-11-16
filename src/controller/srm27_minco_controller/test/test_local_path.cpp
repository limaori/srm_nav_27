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
#include <vector>

#include "srm27_minco_controller/local_path.hpp"
#include "srm27_minco_core/grid_snapshot.hpp"

namespace controller = srm27_minco_controller;
namespace core = srm27_minco_core;

namespace
{
using Point = Eigen::Vector2d;

core::Esdf2D makeMap(double _half_width = 6.0, double _wall_x = 100.0, bool _unknown = false)
{
  const double resolution = 0.05;
  const int size = static_cast<int>(2.0 * _half_width / resolution);
  std::vector<core::CellState> cells(size * size, core::CellState::kFree);
  for (int y = 0; y < size; ++y) {
    for (int x = 0; x < size; ++x) {
      if (-_half_width + (x + 0.5) * resolution >= _wall_x) {
        cells[y * size + x] = _unknown ? core::CellState::kUnknown : core::CellState::kOccupied;
      }
    }
  }
  core::GridSnapshot grid;
  EXPECT_TRUE(grid.reset(-_half_width, -_half_width, resolution, size, size, cells, 1, 100.0));
  core::Esdf2D esdf;
  EXPECT_TRUE(esdf.build(grid));
  return esdf;
}

controller::LocalPathConfig config()
{
  controller::LocalPathConfig result;
  result.terminal_speed = 0.8;
  return result;
}
}  // namespace

TEST(LocalPathTest, ProjectsOntoSegmentWithoutSkippingUpcomingCorner)
{
  auto settings = config();
  settings.horizon = 1.0;
  const auto result = controller::extractLocalPath(
    {Point(0, 0), Point(1, 0), Point(1, 2)}, Point(0.8, 0), makeMap(), settings, true);
  ASSERT_EQ(result.points.size(), 3u);
  EXPECT_TRUE(result.points[1].isApprox(Point(1, 0)));
  EXPECT_TRUE(result.points.back().isApprox(Point(1, 0.8)));
  EXPECT_EQ(result.end, controller::LocalPathEnd::kHorizon);
}

TEST(LocalPathTest, ClearHorizonRetainsCruiseSpeed)
{
  const auto result = controller::extractLocalPath(
    {Point(0, 0), Point(5, 0)}, Point(0, 0), makeMap(), config(), true);
  EXPECT_EQ(result.end, controller::LocalPathEnd::kHorizon);
  ASSERT_GE(result.points.size(), 2u);
  EXPECT_NEAR(result.points.back().x(), 3.0, 1.0e-9);
  EXPECT_DOUBLE_EQ(result.terminal_speed, 0.8);
}

TEST(LocalPathTest, FinalGoalAndNonGoalPathEndStop)
{
  for (bool is_goal : {false, true}) {
    const auto result = controller::extractLocalPath(
      {Point(0, 0), Point(3, 0)}, Point(0, 0), makeMap(), config(), is_goal);
    EXPECT_EQ(
      result.end,
      is_goal ? controller::LocalPathEnd::kGlobalGoal : controller::LocalPathEnd::kPathEnd);
    EXPECT_DOUBLE_EQ(result.terminal_speed, 0.0);
  }
}

TEST(LocalPathTest, HorizonInterpolationCannotJumpOutsideMap)
{
  const auto result = controller::extractLocalPath(
    {Point(0, 0), Point(10, 0)}, Point(0, 0), makeMap(2.5), config(), true);
  EXPECT_EQ(result.end, controller::LocalPathEnd::kMapBoundary);
  ASSERT_GE(result.points.size(), 2u);
  EXPECT_GT(result.points.back().x(), 2.0);
  EXPECT_LT(result.points.back().x(), 2.5 - config().required_clearance);
  EXPECT_DOUBLE_EQ(result.terminal_speed, 0.0);
}

TEST(LocalPathTest, SparsePathCannotJumpAcrossObstacleOrUnknownCells)
{
  for (bool unknown : {false, true}) {
    const auto result = controller::extractLocalPath(
      {Point(0, 0), Point(5, 0)}, Point(0, 0), makeMap(6.0, 1.5, unknown), config(), true);
    EXPECT_EQ(result.end, controller::LocalPathEnd::kBlocked);
    ASSERT_GE(result.points.size(), 2u);
    EXPECT_LT(result.points.back().x(), 1.5 - config().required_clearance);
    EXPECT_DOUBLE_EQ(result.terminal_speed, 0.0);
  }
}

TEST(LocalPathTest, BlockedBrakingContinuationForcesStopAtHorizon)
{
  const auto result = controller::extractLocalPath(
    {Point(0, 0), Point(5, 0)}, Point(0, 0), makeMap(6.0, 3.5), config(), true);
  EXPECT_EQ(result.end, controller::LocalPathEnd::kHorizon);
  EXPECT_NEAR(result.points.back().x(), 3.0, 1.0e-9);
  EXPECT_DOUBLE_EQ(result.terminal_speed, 0.0);
}

TEST(LocalPathTest, MapEdgeInBrakingContinuationForcesStop)
{
  const auto result = controller::extractLocalPath(
    {Point(0, 0), Point(5, 0)}, Point(0, 0), makeMap(3.5), config(), true);
  EXPECT_EQ(result.end, controller::LocalPathEnd::kHorizon);
  EXPECT_DOUBLE_EQ(result.terminal_speed, 0.0);
}

TEST(LocalPathTest, GoalJustBeyondHorizonDoesNotGetCruiseBoundary)
{
  const auto result = controller::extractLocalPath(
    {Point(0, 0), Point(3.1, 0)}, Point(0, 0), makeMap(), config(), true);
  EXPECT_EQ(result.end, controller::LocalPathEnd::kHorizon);
  EXPECT_DOUBLE_EQ(result.terminal_speed, 0.0);
}

TEST(LocalPathTest, ClampsCruiseSpeedAndUsesIdentifiedBraking)
{
  auto settings = config();
  settings.terminal_speed = 10.0;
  auto result = controller::extractLocalPath(
    {Point(0, 0), Point(5.5, 0)}, Point(0, 0), makeMap(), settings, true);
  EXPECT_DOUBLE_EQ(result.terminal_speed, settings.max_speed);
  settings.braking_deceleration = 0.1;
  result = controller::extractLocalPath(
    {Point(0, 0), Point(5.5, 0)}, Point(0, 0), makeMap(), settings, true);
  EXPECT_DOUBLE_EQ(result.terminal_speed, 0.0);
}

TEST(LocalPathTest, AlreadyAtGoalIsDistinctFromBlockedStart)
{
  auto result = controller::extractLocalPath(
    {Point(0, 0), Point(1, 0)}, Point(1, 0), makeMap(), config(), true);
  EXPECT_EQ(result.end, controller::LocalPathEnd::kGlobalGoal);
  ASSERT_EQ(result.points.size(), 1u);
  result = controller::extractLocalPath(
    {Point(0, 0), Point(1, 0)}, Point(1, 0), makeMap(6.0, 1.0), config(), true);
  EXPECT_EQ(result.end, controller::LocalPathEnd::kBlocked);
}

TEST(LocalPathTest, RejectsInvalidPaths)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const std::vector<std::vector<Point>> paths = {
    {}, {Point(0, 0)}, {Point(0, 0), Point(0, 0)}, {Point(0, 0), Point(nan, 0)}};
  for (const auto & path : paths) {
    const auto result = controller::extractLocalPath(path, Point(0, 0), makeMap(), config(), true);
    EXPECT_EQ(result.end, controller::LocalPathEnd::kInvalid);
    EXPECT_TRUE(result.points.empty());
  }
}
