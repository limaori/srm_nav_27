// Copyright 2026 SRM27 Navigation Team
// Licensed under the Apache License, Version 2.0.
//
// SFC 栅格语义（0 = 障碍，255 = 可通行）与边界补圈。
// 这两点是 MinimumSnap/SfcSquare 的正确性前提：
//  - 若沿用 YAstar 的 0 = 可行编码，SFC 会把整张图当成障碍，走廊退化为单点；
//  - 若不补边界，SfcSquare::_getBound() 会越界读。

#include <gtest/gtest.h>

#include <cstdint>
#include <stdexcept>
#include <utility>
#include <vector>

#include "srm27_map_core/grid_snapshot.hpp"
#include "srm27_map_core/sfc_grid.hpp"

namespace
{
using srm27_map_core::CellState;
using srm27_map_core::GridSnapshot;
using srm27_map_core::makeSfcGrid;

// 3x2 图：只有 (x=1, y=1) 是障碍，其余自由。
GridSnapshot makeTinySnapshot()
{
  std::vector<CellState> cells(6, CellState::kFree);
  cells[1 * 3 + 1] = CellState::kOccupied;  // 行主序，y=1 行的 x=1
  GridSnapshot snapshot;
  snapshot.reset(-1.0, -2.0, 0.5, 3, 2, std::move(cells), 0, 0.0);
  return snapshot;
}
}  // namespace

TEST(SfcGridTest, InvertsOccupancySemantics)
{
  const auto grid = makeSfcGrid(makeTinySnapshot());

  // 内部区域：原 (1,1) 障碍 -> 0；其余 -> 255
  EXPECT_EQ(grid.data[2 * grid.size_x + 2], 0);    // 中心障碍
  EXPECT_EQ(grid.data[1 * grid.size_x + 1], 255);  // 左上自由
  EXPECT_EQ(grid.data[2 * grid.size_x + 1], 255);  // 左下自由
}

TEST(SfcGridTest, PadsOneCellObstacleBorder)
{
  const auto snapshot = makeTinySnapshot();
  const auto grid = makeSfcGrid(snapshot);

  EXPECT_EQ(grid.size_x, snapshot.sizeX() + 2);
  EXPECT_EQ(grid.size_y, snapshot.sizeY() + 2);

  // 最外圈必须全为障碍（0），否则 SfcSquare::_getBound() 会越界读
  for (int x = 0; x < grid.size_x; ++x) {
    EXPECT_EQ(grid.data[x], 0) << "上边界 x=" << x;
    EXPECT_EQ(grid.data[(grid.size_y - 1) * grid.size_x + x], 0) << "下边界 x=" << x;
  }
  for (int y = 0; y < grid.size_y; ++y) {
    EXPECT_EQ(grid.data[y * grid.size_x], 0) << "左边界 y=" << y;
    EXPECT_EQ(grid.data[y * grid.size_x + grid.size_x - 1], 0) << "右边界 y=" << y;
  }
}

TEST(SfcGridTest, ShiftsOriginByOneCell)
{
  const auto snapshot = makeTinySnapshot();
  const auto grid = makeSfcGrid(snapshot);

  // 补一圈后原点必须左移/下移一格，否则世界坐标与索引会整体错位
  EXPECT_FLOAT_EQ(grid.origin_x, static_cast<float>(snapshot.originX() - snapshot.resolution()));
  EXPECT_FLOAT_EQ(grid.origin_y, static_cast<float>(snapshot.originY() - snapshot.resolution()));
  EXPECT_FLOAT_EQ(grid.resolution, static_cast<float>(snapshot.resolution()));
}

TEST(SfcGridTest, PreservesCellAlignment)
{
  const auto snapshot = makeTinySnapshot();
  const auto grid = makeSfcGrid(snapshot);

  // 原图 (x, y) 必须落在新图 (x+1, y+1)
  for (int y = 0; y < snapshot.sizeY(); ++y) {
    for (int x = 0; x < snapshot.sizeX(); ++x) {
      const int src = snapshot.cell(x, y) == CellState::kFree ? 255 : 0;
      EXPECT_EQ(grid.data[(y + 1) * grid.size_x + (x + 1)], src)
        << "cell (" << x << "," << y << ") 未对齐";
    }
  }
}

TEST(SfcGridTest, RejectsEmptySnapshot)
{
  const GridSnapshot empty;
  EXPECT_THROW(makeSfcGrid(empty), std::invalid_argument);
}

TEST(SfcGridTest, UnknownCellsCanBeEncodedAsObstacles)
{
  std::vector<CellState> cells(4, CellState::kFree);
  cells[1] = CellState::kUnknown;
  GridSnapshot snapshot;
  snapshot.reset(0.0, 0.0, 0.5, 2, 2, std::move(cells), 0, 0.0);

  const auto with_unknown = makeSfcGrid(snapshot, true);
  EXPECT_EQ(with_unknown.data[1 * with_unknown.size_x + 2], 0);
  const auto without_unknown = makeSfcGrid(snapshot, false);
  EXPECT_EQ(without_unknown.data[1 * without_unknown.size_x + 2], 255);
}

TEST(SfcGridTest, SfcEncodingDiffersFromYastarEncoding)
{
  // 同一份快照的两种编码必须互补，避免误用。
  std::vector<CellState> cells{
    CellState::kFree, CellState::kOccupied, CellState::kUnknown, CellState::kFree};
  GridSnapshot snapshot;
  snapshot.reset(0.0, 0.0, 0.05, 2, 2, std::move(cells), 0, 0.0);

  const auto binary = snapshot.buildBinaryMask(true);      // 0 = 可行 / 1 = 阻挡
  const auto sfc = snapshot.buildByteMask(255, 0, true);   // 255 = 可行 / 0 = 阻挡

  ASSERT_EQ(binary.size(), sfc.size());
  for (std::size_t i = 0; i < binary.size(); ++i) {
    if (binary[i] != 0) {
      EXPECT_EQ(sfc[i], 0) << "index " << i;
    } else {
      EXPECT_EQ(sfc[i], 255) << "index " << i;
    }
  }
}
