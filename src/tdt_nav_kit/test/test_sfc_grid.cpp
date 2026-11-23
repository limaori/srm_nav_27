// Copyright 2026 SRM27 Navigation Team
// Licensed under the Apache License, Version 2.0.
//
// 阶段二新增：验证 SFC 栅格语义（0 = 障碍，255 = 可通行）与边界补圈。
// 这两点是 MinimumSnap/SfcSquare 的正确性前提：
//  - 若沿用 YAstar 的 0 = 可行编码，SFC 会把整张图当成障碍，走廊退化为单点；
//  - 若不补边界，SfcSquare::_getBound() 会越界读。

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "tdt_nav_kit/costmap_converter.hpp"

namespace
{
using tdt_nav_kit::ConverterConfig;
using tdt_nav_kit::CostmapSnapshot;
using tdt_nav_kit::makeSfcGrid;

// 3x2 图：只有 (x=1, y=1) 是障碍，其余自由。
CostmapSnapshot makeTinySnapshot()
{
  CostmapSnapshot snap;
  snap.size_x = 3;
  snap.size_y = 2;
  snap.resolution = 0.5f;
  snap.origin_x = -1.0f;
  snap.origin_y = -2.0f;
  snap.data = {0, 0, 0, 0, 1, 0};  // 行主序，y=1 行的 x=1 为障碍
  return snap;
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
  const auto snap = makeTinySnapshot();
  const auto grid = makeSfcGrid(snap);

  EXPECT_EQ(grid.size_x, snap.size_x + 2);
  EXPECT_EQ(grid.size_y, snap.size_y + 2);

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
  const auto snap = makeTinySnapshot();
  const auto grid = makeSfcGrid(snap);

  // 补一圈后原点必须左移/下移一格，否则世界坐标与索引会整体错位
  EXPECT_FLOAT_EQ(grid.origin_x, snap.origin_x - snap.resolution);
  EXPECT_FLOAT_EQ(grid.origin_y, snap.origin_y - snap.resolution);
  EXPECT_FLOAT_EQ(grid.resolution, snap.resolution);
}

TEST(SfcGridTest, PreservesCellAlignment)
{
  const auto snap = makeTinySnapshot();
  const auto grid = makeSfcGrid(snap);

  // 原图 (x, y) 必须落在新图 (x+1, y+1)
  for (int y = 0; y < snap.size_y; ++y) {
    for (int x = 0; x < snap.size_x; ++x) {
      const int src = snap.data[y * snap.size_x + x] != 0 ? 0 : 255;
      EXPECT_EQ(grid.data[(y + 1) * grid.size_x + (x + 1)], src)
        << "cell (" << x << "," << y << ") 未对齐";
    }
  }
}

TEST(SfcGridTest, RejectsEmptySnapshot)
{
  CostmapSnapshot empty;
  EXPECT_THROW(makeSfcGrid(empty), std::invalid_argument);
}

TEST(SfcGridTest, SfcEncodingDiffersFromYastarEncoding)
{
  // 同一份 costmap 的两种编码必须互补，避免误用
  const unsigned char char_map[4] = {0, 254, 255, 100};
  const ConverterConfig cfg;

  const auto ya = tdt_nav_kit::convertCostmapCells(char_map, 4, cfg);
  const auto sfc = tdt_nav_kit::convertCostmapCellsToSfc(char_map, 4, cfg);

  ASSERT_EQ(ya.size(), sfc.size());
  for (std::size_t i = 0; i < ya.size(); ++i) {
    if (ya[i] != 0) {
      EXPECT_EQ(sfc[i], 0) << "index " << i;      // 障碍：YAstar=1 / SFC=0
    } else {
      EXPECT_EQ(sfc[i], 255) << "index " << i;    // 自由：YAstar=0 / SFC=255
    }
  }
}
