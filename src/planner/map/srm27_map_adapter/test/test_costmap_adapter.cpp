// Copyright 2026 SRM27 Navigation Team
// Licensed under the Apache License, Version 2.0.
//
// Nav2 代价值 → GridSnapshot 语义。253 / 252 两档阈值分别对应
// MINCO/controller（只把 LETHAL 254 当障碍）与 YAstar/SFC（253/254 都算障碍）。

#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "srm27_map_adapter/costmap_adapter.hpp"

namespace
{
using srm27_map_adapter::CostmapConfig;
using srm27_map_adapter::makeGridSnapshot;
using srm27_map_core::CellState;
using srm27_map_core::GridSnapshot;

// 3x2 图，分辨率 0.5，原点 (-1, -2)。单元按行取下面的代价值。
std::vector<unsigned char> makeCostmapValues()
{
  return {
    0,                                    // free
    nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE,  // 253
    nav2_costmap_2d::LETHAL_OBSTACLE,     // 254
    nav2_costmap_2d::NO_INFORMATION,      // 255
    nav2_costmap_2d::MAX_NON_OBSTACLE,     // 252
    0,
  };
}

GridSnapshot snapshotWith(int threshold, std::uint64_t * hash = nullptr)
{
  nav2_costmap_2d::Costmap2D costmap(3, 2, 0.5, -1.0, -2.0, 0);
  const auto values = makeCostmapValues();
  for (unsigned int y = 0; y < 2; ++y) {
    for (unsigned int x = 0; x < 3; ++x) {
      costmap.setCost(x, y, values[y * 3 + x]);
    }
  }
  CostmapConfig config;
  config.occupied_cost_threshold = threshold;
  GridSnapshot snapshot;
  std::string reason;
  EXPECT_TRUE(makeGridSnapshot(&costmap, config, 1.0, snapshot, hash, &reason)) << reason;
  return snapshot;
}

}  // namespace

TEST(CostmapAdapterTest, LethalOnlyThresholdKeepsInscribedFree)
{
  const auto snapshot = snapshotWith(253);
  ASSERT_TRUE(snapshot.valid());
  EXPECT_EQ(snapshot.sizeX(), 3);
  EXPECT_EQ(snapshot.sizeY(), 2);
  EXPECT_DOUBLE_EQ(snapshot.resolution(), 0.5);
  EXPECT_DOUBLE_EQ(snapshot.originX(), -1.0);
  EXPECT_DOUBLE_EQ(snapshot.originY(), -2.0);

  EXPECT_EQ(snapshot.cell(0, 0), CellState::kFree);
  EXPECT_EQ(snapshot.cell(1, 0), CellState::kFree);       // 253 是内切膨胀，不算原始障碍
  EXPECT_EQ(snapshot.cell(2, 0), CellState::kOccupied);   // 254
  EXPECT_EQ(snapshot.cell(0, 1), CellState::kUnknown);    // 255
  EXPECT_EQ(snapshot.cell(1, 1), CellState::kFree);       // 252
}

TEST(CostmapAdapterTest, Threshold252MarksInscribedObstacle)
{
  const auto snapshot = snapshotWith(252);
  EXPECT_EQ(snapshot.cell(1, 0), CellState::kOccupied);   // 253 > 252
  EXPECT_EQ(snapshot.cell(2, 0), CellState::kOccupied);   // 254
  EXPECT_EQ(snapshot.cell(0, 1), CellState::kUnknown);    // 255 仍是未知
}

TEST(CostmapAdapterTest, ContentHashIsStableAndChangesWithCosts)
{
  std::uint64_t first = 0;
  std::uint64_t second = 0;
  snapshotWith(253, &first);
  snapshotWith(253, &second);
  EXPECT_EQ(first, second);

  std::uint64_t changed = 0;
  nav2_costmap_2d::Costmap2D costmap(3, 2, 0.5, -1.0, -2.0, 0);
  for (unsigned int y = 0; y < 2; ++y) {
    for (unsigned int x = 0; x < 3; ++x) {
      costmap.setCost(x, y, 0);
    }
  }
  CostmapConfig config;
  config.occupied_cost_threshold = 253;
  GridSnapshot snapshot;
  ASSERT_TRUE(makeGridSnapshot(&costmap, config, 1.0, snapshot, &changed));
  EXPECT_NE(first, changed);
}

TEST(CostmapAdapterTest, RejectsNullCostmap)
{
  CostmapConfig config;
  GridSnapshot snapshot;
  std::string reason;
  EXPECT_FALSE(makeGridSnapshot(nullptr, config, 1.0, snapshot, nullptr, &reason));
  EXPECT_FALSE(reason.empty());
}
