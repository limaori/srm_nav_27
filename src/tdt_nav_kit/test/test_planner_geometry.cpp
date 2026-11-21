// Copyright 2026 SRM27 Navigation Team
// Licensed under the Apache License, Version 2.0.

#include <gtest/gtest.h>

#include <limits>

#include "tdt_nav_kit/planner_geometry.hpp"
#include "yastar.hpp"

namespace
{
using Point = Eigen::Vector2f;
using tdt_nav_kit::CostmapSnapshot;

CostmapSnapshot map()
{
  CostmapSnapshot m;
  m.size_x = 12;
  m.size_y = 10;
  m.resolution = .05f;
  m.origin_x = -.30f;
  m.origin_y = -.20f;
  m.data.assign(m.size_x * m.size_y, 0);
  return m;
}

Point center(const CostmapSnapshot & m, int x, int y)
{
  return Point(m.origin_x, m.origin_y) + m.resolution * Point(x + .5f, y + .5f);
}
}  // namespace

TEST(PlannerGeometry, ConvertsCornersToCentersAndKeepsExactWorldEndpoints)
{
  const auto m = map();
  const Point start = center(m, 1, 2) + Point(.009f, -.007f);
  const Point goal = center(m, 8, 6) + Point(-.008f, .011f);
  const std::vector<Point> raw{
    center(m, 1, 2) - Point::Constant(.025f), center(m, 4, 4) - Point::Constant(.025f),
    center(m, 8, 6) - Point::Constant(.025f)};
  const auto path = tdt_nav_kit::nav2PathFromYastar(m, raw, start, goal);
  ASSERT_EQ(path.size(), 3u);
  EXPECT_LT((path.front() - start).norm(), 1e-7f);
  EXPECT_LT((path.back() - goal).norm(), 1e-7f);
  EXPECT_LT((path[1] - center(m, 4, 4)).norm(), 1e-7f);
  EXPECT_TRUE(tdt_nav_kit::pathIsFree(m, path));
}

TEST(PlannerGeometry, SameCellGoalIsNotConfusedWithFailedSearch)
{
  const auto m = map();
  const Point start = center(m, 3, 4);
  const Point close_goal = start + Point(.005f, .008f);
  const auto path = tdt_nav_kit::nav2PathFromYastar(m, {start}, start, close_goal);
  ASSERT_EQ(path.size(), 2u);
  EXPECT_TRUE(tdt_nav_kit::pathIsFree(m, path));
  EXPECT_TRUE(tdt_nav_kit::nav2PathFromYastar(m, {start}, start, center(m, 9, 4)).empty());
}

TEST(PlannerGeometry, ContinuousSegmentCannotCutBlockedCorner)
{
  auto m = map();
  m.data[2 * m.size_x + 3] = 1;
  m.data[3 * m.size_x + 2] = 1;
  EXPECT_FALSE(tdt_nav_kit::segmentIsFree(m, center(m, 2, 2), center(m, 3, 3)));
  EXPECT_TRUE(tdt_nav_kit::segmentIsFree(m, center(m, 2, 2), center(m, 1, 3)));
  // A long simplified segment must not skip a single thin obstacle cell.
  m.data[5 * m.size_x + 6] = 1;
  EXPECT_FALSE(tdt_nav_kit::segmentIsFree(m, center(m, 1, 5), center(m, 10, 5)));
}

TEST(PlannerGeometry, RejectsOutOfMapNonfiniteAndBlockedEndpoints)
{
  auto m = map();
  EXPECT_FALSE(tdt_nav_kit::pointIsFree(m, Point(m.origin_x - .001f, 0)));
  EXPECT_FALSE(tdt_nav_kit::pointIsFree(m, Point(100, 100)));
  EXPECT_FALSE(tdt_nav_kit::pointIsFree(m, Point(std::numeric_limits<float>::quiet_NaN(), 0)));
  m.data[4 * m.size_x + 3] = 1;
  EXPECT_FALSE(tdt_nav_kit::segmentIsFree(m, center(m, 3, 4), center(m, 8, 4)));
}

TEST(PlannerGeometry, SnapHonorsEuclideanRadiusAndPreservesFreePoint)
{
  auto m = map();
  const Point goal = center(m, 5, 5) + Point(.004f, .003f);
  const auto exact = tdt_nav_kit::nearestFreePoint(m, goal, 0);
  ASSERT_TRUE(exact);
  EXPECT_EQ(*exact, goal);
  m.data.assign(m.data.size(), 1);
  m.data[5 * m.size_x + 7] = 0;
  EXPECT_FALSE(tdt_nav_kit::nearestFreePoint(m, goal, .09f));
  const auto snapped = tdt_nav_kit::nearestFreePoint(m, goal, .10f);
  ASSERT_TRUE(snapped);
  EXPECT_EQ(*snapped, center(m, 7, 5));
}

TEST(PlannerGeometry, ReplanKeepsResolvedGoalWhenCloserCellsBecomeFree)
{
  auto m = map();
  m.data.assign(m.data.size(), 1);
  m.data[5 * m.size_x + 7] = 0;
  const Point requested = center(m, 5, 5);
  tdt_nav_kit::StableGoal goal;
  const auto first = goal.resolve(m, requested, .2f);
  ASSERT_TRUE(first);
  m.data[5 * m.size_x + 6] = 0;
  EXPECT_EQ(*goal.resolve(m, requested, .2f), *first);
  m.data[5 * m.size_x + 5] = 0;
  EXPECT_EQ(*goal.resolve(m, requested, .2f), *first);
  // A different requested target still creates a new navigation endpoint.
  EXPECT_EQ(*goal.resolve(m, center(m, 6, 5), .2f), center(m, 6, 5));
}

TEST(PlannerGeometry, ExactGoalRejectsBlockedCellAndPreservesRequestedCoordinates)
{
  auto m = map();
  const Point requested = center(m, 5, 5) + Point(.004f, .003f);
  tdt_nav_kit::StableGoal goal;
  m.data[5 * m.size_x + 5] = 1;
  EXPECT_FALSE(goal.resolve(m, requested, 0.0f));

  m.data[5 * m.size_x + 5] = 0;
  const auto resolved = goal.resolve(m, requested, 0.0f);
  ASSERT_TRUE(resolved);
  EXPECT_EQ(*resolved, requested);

  m.data[5 * m.size_x + 5] = 1;
  EXPECT_FALSE(goal.resolve(m, requested, 0.0f));
}

TEST(PlannerGeometry, BlockedResolvedGoalFailsInsteadOfJumpingToAnotherCell)
{
  auto m = map();
  const Point requested = center(m, 5, 5);
  m.data[5 * m.size_x + 5] = 1;
  tdt_nav_kit::StableGoal goal;
  const auto first = goal.resolve(m, requested, .2f);
  ASSERT_TRUE(first);
  // Block the selected endpoint while other candidate endpoints remain free.
  const int x = static_cast<int>((first->x() - m.origin_x) / m.resolution);
  const int y = static_cast<int>((first->y() - m.origin_y) / m.resolution);
  m.data[y * m.size_x + x] = 1;
  EXPECT_FALSE(goal.resolve(m, requested, .2f));
  goal.reset();
  const auto next = goal.resolve(m, requested, .2f);
  ASSERT_TRUE(next);
  EXPECT_GT((*next - *first).norm(), .01f);
}

TEST(PlannerGeometry, SearchAndSimplificationRetainSubcellEndpoints)
{
  auto m = map();
  for (int y = 0; y < 7; ++y) {
    m.data[y * m.size_x + 6] = 1;
  }
  YAstar planner;
  planner.setMap(m.size_x, m.size_y, m.resolution, m.origin_x, m.origin_y, m.data);
  planner.setMaskNumLayers(1);
  planner.setCostField(1.f, [](float) { return 0.f; });
  planner.initCostMap();
  const Point start = center(m, 2, 3) + Point(.01f, .005f);
  const Point goal = center(m, 9, 3) + Point(-.008f, -.007f);
  const auto path = tdt_nav_kit::nav2PathFromYastar(m, planner.search(start, goal), start, goal);
  ASSERT_TRUE(tdt_nav_kit::pathIsFree(m, path));
  auto simplified = path;
  simplified = planner.simplifyPath(simplified, .1f);
  simplified = planner.simplifyPathHypot(simplified);
  EXPECT_EQ(simplified.front(), start);
  EXPECT_EQ(simplified.back(), goal);
}
