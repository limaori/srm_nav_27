// Copyright 2026 SRM27 Navigation Team
// Licensed under the Apache License, Version 2.0.
//
// 地图几何查询：占据判断、连续线段穿角、最近自由点。
// 2026-11-24 从 srm27_nav_common/planner_geometry 抽到地图模块；规划器/平滑器
// 都会调用，地图语义（未知是否算障碍）由调用方显式传入。

#include <gtest/gtest.h>

#include <limits>
#include <utility>
#include <vector>

#include "srm27_map_core/occupancy_geometry.hpp"

namespace
{
using Point = Eigen::Vector2f;
using srm27_map_core::CellState;
using srm27_map_core::GridSnapshot;

GridSnapshot makeMap(
  const std::vector<std::pair<int, int>> & occupied = {},
  const std::vector<std::pair<int, int>> & unknown = {})
{
  constexpr int kSizeX = 12;
  constexpr int kSizeY = 10;
  std::vector<CellState> cells(kSizeX * kSizeY, CellState::kFree);
  for (const auto & cell : occupied) {
    cells[cell.second * kSizeX + cell.first] = CellState::kOccupied;
  }
  for (const auto & cell : unknown) {
    cells[cell.second * kSizeX + cell.first] = CellState::kUnknown;
  }
  GridSnapshot map;
  map.reset(-0.30, -0.20, 0.05, kSizeX, kSizeY, std::move(cells), 0, 0.0);
  return map;
}

Point center(const GridSnapshot & map, int x, int y)
{
  return Point(
    static_cast<float>(map.originX()), static_cast<float>(map.originY())) +
         static_cast<float>(map.resolution()) * Point(x + .5f, y + .5f);
}
}  // namespace

TEST(OccupancyGeometry, ContinuousSegmentCannotCutBlockedCorner)
{
  auto map = makeMap({{3, 2}, {2, 3}});
  EXPECT_FALSE(
    srm27_map_core::segmentIsFree(map, center(map, 2, 2), center(map, 3, 3)));
  EXPECT_TRUE(
    srm27_map_core::segmentIsFree(map, center(map, 2, 2), center(map, 1, 3)));

  // 一条长的简化线段不能跳过单个薄障碍格。
  map = makeMap({{3, 2}, {2, 3}, {6, 5}});
  EXPECT_FALSE(
    srm27_map_core::segmentIsFree(map, center(map, 1, 5), center(map, 10, 5)));
}

TEST(OccupancyGeometry, RejectsOutOfMapNonfiniteAndBlockedEndpoints)
{
  const auto map = makeMap({{3, 4}});
  EXPECT_FALSE(srm27_map_core::pointIsFree(map, Point(map.originX() - .001f, 0)));
  EXPECT_FALSE(srm27_map_core::pointIsFree(map, Point(100, 100)));
  EXPECT_FALSE(
    srm27_map_core::pointIsFree(
      map, Point(std::numeric_limits<float>::quiet_NaN(), 0)));
  EXPECT_FALSE(
    srm27_map_core::segmentIsFree(map, center(map, 3, 4), center(map, 8, 4)));
}

TEST(OccupancyGeometry, UnknownCellsFollowExplicitPolicy)
{
  const auto map = makeMap({}, {{5, 5}});
  EXPECT_FALSE(srm27_map_core::pointIsFree(map, center(map, 5, 5), true));
  EXPECT_TRUE(srm27_map_core::pointIsFree(map, center(map, 5, 5), false));

  // 未知格被当作障碍时，穿过的线段也必须拒绝。
  EXPECT_FALSE(
    srm27_map_core::segmentIsFree(map, center(map, 3, 5), center(map, 8, 5), true));
  EXPECT_TRUE(
    srm27_map_core::segmentIsFree(map, center(map, 3, 5), center(map, 8, 5), false));
}

TEST(OccupancyGeometry, SnapHonorsEuclideanRadiusAndPreservesFreePoint)
{
  auto map = makeMap();
  const Point goal = center(map, 5, 5) + Point(.004f, .003f);
  const auto exact = srm27_map_core::nearestFreePoint(map, goal, 0);
  ASSERT_TRUE(exact);
  EXPECT_EQ(*exact, goal);

  // 除 (7,5) 外全部占满：半径 0.09 m 差半格到不了，0.10 m 才够。
  std::vector<std::pair<int, int>> occupied;
  for (int y = 0; y < map.sizeY(); ++y) {
    for (int x = 0; x < map.sizeX(); ++x) {
      if (!(x == 7 && y == 5)) {
        occupied.emplace_back(x, y);
      }
    }
  }
  map = makeMap(occupied);
  EXPECT_FALSE(srm27_map_core::nearestFreePoint(map, goal, .09f));
  const auto snapped = srm27_map_core::nearestFreePoint(map, goal, .10f);
  ASSERT_TRUE(snapped);
  EXPECT_LT((*snapped - center(map, 7, 5)).norm(), 1e-6f);
}
