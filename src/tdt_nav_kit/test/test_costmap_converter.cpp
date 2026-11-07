// Copyright 2026 SRM27 Navigation Team
// Licensed under the Apache License, Version 2.0.

#include <gtest/gtest.h>

#include <cstdint>
#include <initializer_list>
#include <stdexcept>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"
#include "tdt_nav_kit/costmap_converter.hpp"

using tdt_nav_kit::ConverterConfig;
using tdt_nav_kit::convertCostmapCells;

namespace
{

std::vector<unsigned char> makeCells(std::initializer_list<unsigned char> values)
{
  return std::vector<unsigned char>(values);
}

}  // namespace

TEST(CostmapConverterTest, OccupancySemantics)
{
  const auto cells = makeCells({
    nav2_costmap_2d::FREE_SPACE,
    nav2_costmap_2d::MAX_NON_OBSTACLE,
    nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE,
    nav2_costmap_2d::LETHAL_OBSTACLE,
    nav2_costmap_2d::NO_INFORMATION,
  });

  const auto out = convertCostmapCells(cells.data(), cells.size(), ConverterConfig{});
  ASSERT_EQ(out.size(), cells.size());
  // MAX_NON_OBSTACLE(252) 是阈值本身，不判障碍；253/254 判障碍。
  EXPECT_EQ(out[0], 0);
  EXPECT_EQ(out[1], 0);
  EXPECT_EQ(out[2], 1);
  EXPECT_EQ(out[3], 1);
  EXPECT_EQ(out[4], 1) << "NO_INFORMATION 默认按障碍处理";
}

TEST(CostmapConverterTest, UnknownCanBeTreatedAsFree)
{
  const auto cells = makeCells({nav2_costmap_2d::NO_INFORMATION,
                                nav2_costmap_2d::LETHAL_OBSTACLE});
  ConverterConfig cfg;
  cfg.treat_unknown_as_obstacle = false;
  const auto out = convertCostmapCells(cells.data(), cells.size(), cfg);
  EXPECT_EQ(out[0], 0);
  EXPECT_EQ(out[1], 1) << "关闭 unknown 处理不应影响致命障碍";
}

TEST(CostmapConverterTest, ThresholdBoundaryIsConfigurable)
{
  const auto cells = makeCells({0, 100, 200, 252, 253});
  ConverterConfig cfg;
  cfg.occupancy_threshold = 200;
  const auto out = convertCostmapCells(cells.data(), cells.size(), cfg);
  EXPECT_EQ(out[0], 0);
  EXPECT_EQ(out[1], 0);
  EXPECT_EQ(out[2], 0) << "等于阈值不算障碍";
  EXPECT_EQ(out[3], 1);
  EXPECT_EQ(out[4], 1);
}

TEST(CostmapConverterTest, RowMajorOrderIsPreservedNoYFlip)
{
  // 非方形地图 7x5：只在对角两端放不同的值，验证线性索引 y*size_x+x 未被翻转。
  constexpr int kWidth = 7;
  constexpr int kHeight = 5;
  std::vector<unsigned char> cells(kWidth * kHeight, nav2_costmap_2d::FREE_SPACE);
  cells[0] = nav2_costmap_2d::LETHAL_OBSTACLE;                       // (x=0, y=0)
  cells[kHeight * kWidth - 1] = nav2_costmap_2d::LETHAL_OBSTACLE;   // (x=6, y=4)
  cells[1 * kWidth + 3] = nav2_costmap_2d::LETHAL_OBSTACLE;         // (x=3, y=1)

  const auto out = convertCostmapCells(cells.data(), cells.size(), ConverterConfig{});
  ASSERT_EQ(out.size(), static_cast<std::size_t>(kWidth * kHeight));
  EXPECT_EQ(out[0], 1);
  EXPECT_EQ(out[kHeight * kWidth - 1], 1);
  EXPECT_EQ(out[1 * kWidth + 3], 1);
  EXPECT_EQ(out[4 * kWidth + 0], 0) << "若发生 Y 翻转，(0,0) 与 (0,4) 会互换";
  EXPECT_EQ(out[0 * kWidth + 6], 0);
}

TEST(CostmapConverterTest, ConversionIsADeepCopy)
{
  std::vector<unsigned char> cells = {0, 0, 0, 0};
  const auto out = convertCostmapCells(cells.data(), cells.size(), ConverterConfig{});
  // 模拟 Costmap 更新线程改写原缓冲，快照不得随之变化。
  cells[0] = nav2_costmap_2d::LETHAL_OBSTACLE;
  cells[2] = 255;
  EXPECT_EQ(out[0], 0);
  EXPECT_EQ(out[2], 0);
}

TEST(CostmapConverterTest, EmptyAndInvalidInput)
{
  ConverterConfig cfg;
  EXPECT_TRUE(convertCostmapCells(nullptr, 0, cfg).empty());
  EXPECT_THROW(convertCostmapCells(nullptr, 4, cfg), std::invalid_argument);
}
