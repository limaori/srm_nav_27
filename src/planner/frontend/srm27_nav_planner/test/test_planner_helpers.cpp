// Copyright 2026 SRM27 Navigation Team
// Licensed under the Apache License, Version 2.0.
//
// 规划器专用的地图逻辑：YAstar 角点→Nav2 格中心、目标吸附稳定性。
// 2026-11-24 从 srm27_nav_common/planner_geometry 抽到规划前端包；
// 通用占据几何在 srm27_map_core（test_occupancy_geometry）。

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "srm27_map_core/grid_snapshot.hpp"
#include "srm27_map_core/occupancy_geometry.hpp"
#include "srm27_nav_planner/goal_resolver.hpp"
#include "srm27_nav_planner/yastar_path.hpp"
#include "yastar.hpp"

namespace
{
using Point = Eigen::Vector2f;
using srm27_map_core::CellState;
using srm27_map_core::GridSnapshot;
using srm27_nav_planner::nav2PathFromYastar;
using srm27_nav_planner::StableGoal;

struct MutableMap
{
  static constexpr int kSizeX = 12;
  static constexpr int kSizeY = 10;
  static constexpr double kResolution = 0.05;
  static constexpr double kOriginX = -0.30;
  static constexpr double kOriginY = -0.20;

  std::vector<CellState> cells{kSizeX * kSizeY, CellState::kFree};

  void set(int x, int y, CellState state) { cells[y * kSizeX + x] = state; }

  GridSnapshot snapshot() const
  {
    GridSnapshot map;
    map.reset(kOriginX, kOriginY, kResolution, kSizeX, kSizeY, cells, 0, 0.0);
    return map;
  }
};

Point center(int x, int y)
{
  return Point(MutableMap::kOriginX, MutableMap::kOriginY) +
         static_cast<float>(MutableMap::kResolution) * Point(x + .5f, y + .5f);
}

}  // namespace

TEST(PlannerHelpers, ConvertsCornersToCentersAndKeepsExactWorldEndpoints)
{
  const MutableMap builder;
  const auto map = builder.snapshot();
  const Point start = center(1, 2) + Point(.009f, -.007f);
  const Point goal = center(8, 6) + Point(-.008f, .011f);
  const std::vector<Point> raw{
    center(1, 2) - Point::Constant(.025f), center(4, 4) - Point::Constant(.025f),
    center(8, 6) - Point::Constant(.025f)};
  const auto path = nav2PathFromYastar(map, raw, start, goal);
  ASSERT_EQ(path.size(), 3u);
  EXPECT_LT((path.front() - start).norm(), 1e-6f);
  EXPECT_LT((path.back() - goal).norm(), 1e-6f);
  EXPECT_LT((path[1] - center(4, 4)).norm(), 1e-6f);
  EXPECT_TRUE(srm27_map_core::pathIsFree(map, path));
}

TEST(PlannerHelpers, SameCellGoalIsNotConfusedWithFailedSearch)
{
  const MutableMap builder;
  const auto map = builder.snapshot();
  const Point start = center(3, 4);
  const Point close_goal = start + Point(.005f, .008f);
  const auto path = nav2PathFromYastar(map, {start}, start, close_goal);
  ASSERT_EQ(path.size(), 2u);
  EXPECT_TRUE(srm27_map_core::pathIsFree(map, path));
  EXPECT_TRUE(nav2PathFromYastar(map, {start}, start, center(9, 4)).empty());
}

TEST(PlannerHelpers, ReplanKeepsResolvedGoalWhenCloserCellsBecomeFree)
{
  MutableMap builder;
  for (auto & cell : builder.cells) {
    cell = CellState::kOccupied;
  }
  builder.set(7, 5, CellState::kFree);
  const Point requested = center(5, 5);

  StableGoal goal;
  const auto first = goal.resolve(builder.snapshot(), requested, .2f);
  ASSERT_TRUE(first);
  builder.set(6, 5, CellState::kFree);
  EXPECT_EQ(*goal.resolve(builder.snapshot(), requested, .2f), *first);
  builder.set(5, 5, CellState::kFree);
  EXPECT_EQ(*goal.resolve(builder.snapshot(), requested, .2f), *first);
  // 不同目标仍然建立新的吸附点。
  EXPECT_EQ(*goal.resolve(builder.snapshot(), center(6, 5), .2f), center(6, 5));
}

TEST(PlannerHelpers, ExactGoalRejectsBlockedCellAndPreservesRequestedCoordinates)
{
  MutableMap builder;
  const Point requested = center(5, 5) + Point(.004f, .003f);
  StableGoal goal;
  builder.set(5, 5, CellState::kOccupied);
  EXPECT_FALSE(goal.resolve(builder.snapshot(), requested, 0.0f));

  builder.set(5, 5, CellState::kFree);
  const auto resolved = goal.resolve(builder.snapshot(), requested, 0.0f);
  ASSERT_TRUE(resolved);
  EXPECT_EQ(*resolved, requested);

  builder.set(5, 5, CellState::kOccupied);
  EXPECT_FALSE(goal.resolve(builder.snapshot(), requested, 0.0f));
}

TEST(PlannerHelpers, BlockedResolvedGoalFailsInsteadOfJumpingToAnotherCell)
{
  MutableMap builder;
  const Point requested = center(5, 5);
  builder.set(5, 5, CellState::kOccupied);
  StableGoal goal;
  const auto first = goal.resolve(builder.snapshot(), requested, .2f);
  ASSERT_TRUE(first);
  const int x = static_cast<int>((first->x() - MutableMap::kOriginX) / MutableMap::kResolution);
  const int y = static_cast<int>((first->y() - MutableMap::kOriginY) / MutableMap::kResolution);
  builder.set(x, y, CellState::kOccupied);
  EXPECT_FALSE(goal.resolve(builder.snapshot(), requested, .2f));
  goal.reset();
  const auto next = goal.resolve(builder.snapshot(), requested, .2f);
  ASSERT_TRUE(next);
  EXPECT_GT((*next - *first).norm(), .01f);
}

TEST(PlannerHelpers, SearchAndSimplificationRetainSubcellEndpoints)
{
  MutableMap builder;
  for (int y = 0; y < 7; ++y) {
    builder.set(6, y, CellState::kOccupied);
  }
  const auto map = builder.snapshot();
  std::vector<std::int8_t> mask = map.buildBinaryMask(true);

  YAstar planner;
  planner.setMap(
    map.sizeX(), map.sizeY(), static_cast<float>(map.resolution()),
    static_cast<float>(map.originX()), static_cast<float>(map.originY()), mask);
  planner.setMaskNumLayers(1);
  planner.setCostField(1.f, [](float) { return 0.f; });
  planner.initCostMap();
  const Point start = center(2, 3) + Point(.01f, .005f);
  const Point goal = center(9, 3) + Point(-.008f, -.007f);
  const auto path = nav2PathFromYastar(map, planner.search(start, goal), start, goal);
  ASSERT_TRUE(srm27_map_core::pathIsFree(map, path));
  auto simplified = path;
  simplified = planner.simplifyPath(simplified, .1f);
  simplified = planner.simplifyPathHypot(simplified);
  EXPECT_EQ(simplified.front(), start);
  EXPECT_EQ(simplified.back(), goal);
}
