// Copyright 2026 SRM27 Navigation Team
// Licensed under the Apache License, Version 2.0.
//
// GridSnapshot 是地图模块的规范类型：几何、单元语义、版本/时间戳与两种编码。
// 这里把 reset 的失败契约和编码语义钉死，避免下游把未知格误当自由。

#include <gtest/gtest.h>

#include <cstdint>
#include <utility>
#include <vector>

#include "srm27_map_core/grid_snapshot.hpp"

namespace
{
using srm27_map_core::CellState;
using srm27_map_core::GridSnapshot;
}  // namespace

TEST(GridSnapshotTest, ResetRejectsInconsistentGeometry)
{
  GridSnapshot snapshot;
  EXPECT_FALSE(snapshot.reset(0, 0, 0.0, 2, 2, std::vector<CellState>(4), 0, 0.0));
  EXPECT_FALSE(snapshot.reset(0, 0, 0.05, 0, 2, std::vector<CellState>(4), 0, 0.0));
  EXPECT_FALSE(snapshot.reset(0, 0, 0.05, 2, 2, std::vector<CellState>(3), 0, 0.0));
  EXPECT_FALSE(
    snapshot.reset(
      0, 0, 0.05, 2, 2, std::vector<CellState>(4),
      0, std::numeric_limits<double>::quiet_NaN()));
  EXPECT_FALSE(snapshot.valid());
}

TEST(GridSnapshotTest, ResetStoresGeometryAndTransforms)
{
  GridSnapshot snapshot;
  ASSERT_TRUE(
    snapshot.reset(-1.0, -2.0, 0.5, 4, 3, std::vector<CellState>(12), 7, 1.5));
  EXPECT_TRUE(snapshot.valid());
  EXPECT_EQ(snapshot.sizeX(), 4);
  EXPECT_EQ(snapshot.sizeY(), 3);
  EXPECT_DOUBLE_EQ(snapshot.resolution(), 0.5);
  EXPECT_EQ(snapshot.mapVersion(), 7u);
  EXPECT_DOUBLE_EQ(snapshot.stamp(), 1.5);

  int mx = -1;
  int my = -1;
  snapshot.worldToCell(-0.9, -1.9, mx, my);
  EXPECT_EQ(mx, 0);
  EXPECT_EQ(my, 0);
  double wx = 0.0;
  double wy = 0.0;
  snapshot.cellToWorld(0, 0, wx, wy);
  EXPECT_DOUBLE_EQ(wx, -0.75);
  EXPECT_DOUBLE_EQ(wy, -1.75);
}

TEST(GridSnapshotTest, OutOfRangeCellsAreUnknown)
{
  GridSnapshot snapshot;
  ASSERT_TRUE(snapshot.reset(0, 0, 0.1, 2, 2, std::vector<CellState>(4), 0, 0.0));
  EXPECT_EQ(snapshot.cell(-1, 0), CellState::kUnknown);
  EXPECT_EQ(snapshot.cell(2, 0), CellState::kUnknown);
  EXPECT_EQ(snapshot.cellAt(100.0, 100.0), CellState::kUnknown);
}

TEST(GridSnapshotTest, BinaryAndByteMasksShareUnknownPolicy)
{
  std::vector<CellState> cells{
    CellState::kFree, CellState::kOccupied, CellState::kUnknown, CellState::kFree};
  GridSnapshot snapshot;
  ASSERT_TRUE(snapshot.reset(0, 0, 0.1, 2, 2, std::move(cells), 0, 0.0));

  const auto binary_blocked = snapshot.buildBinaryMask(true);   // 0 可行 / 1 阻挡
  const auto binary_free = snapshot.buildBinaryMask(false);
  ASSERT_EQ(binary_blocked.size(), 4u);
  EXPECT_EQ(binary_blocked[0], 0);
  EXPECT_EQ(binary_blocked[1], 1);
  EXPECT_EQ(binary_blocked[2], 1);   // 未知按阻挡
  EXPECT_EQ(binary_free[2], 0);      // 未知按自由

  const auto sfc = snapshot.buildByteMask(255, 0, true);  // SFC：255 可通行 / 0 障碍
  ASSERT_EQ(sfc.size(), 4u);
  EXPECT_EQ(sfc[0], 255);
  EXPECT_EQ(sfc[1], 0);
  EXPECT_EQ(sfc[2], 0);

  const auto seeds = snapshot.buildSeedMask(true);
  ASSERT_EQ(seeds.size(), 4u);
  EXPECT_EQ(seeds[0], 1);  // 1 = 非种子
  EXPECT_EQ(seeds[1], 0);  // 0 = 种子
  EXPECT_EQ(seeds[2], 0);
}
