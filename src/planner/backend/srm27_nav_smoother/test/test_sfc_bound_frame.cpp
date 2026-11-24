// Copyright 2026 SRM27 Navigation Team
// Licensed under the Apache License, Version 2.0.
//
// P10 补丁的回归测试：SFC 方形走廊必须落在**世界坐标系**里。
//
// 上游 sfcSquare.cpp 的世界 -> 索引方向写作 (x + originPos) / mapping，
// 索引 -> 世界方向写作 index * mapping - originPos，两者是互逆的。
// P7 把前一个方向统一成 (x - originPos) / mapping（与 isOutside /
// MinimumSnap::lineToCell 一致），但没改后一个方向，于是两个方向不再互逆：
// 负原点时每个方块整体平移 2*|origin|，不再包含它对应的路径点。
//
// 这里用**非零且为负**的原点断言闭环；补丁前 getBound 的方块会偏移好几米，
// 下面的 contains 断言必然失败。

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <vector>

#include "sfcSquare.hpp"

namespace
{
/// 60x60 全可通行（SFC 语义 255 = 自由）。范围取得比 maxRange 大得多，
/// 扩张不会摸到地图边界（上游 _getBound 不判边界，靠调用方补边界圈）。
SfcSquare::Map makeFreeMap()
{
  SfcSquare::Map map(60, 60);
  map.setConstant(static_cast<unsigned char>(255));
  return map;
}

bool contains(const SfcSquare::PointPair & box, float x, float y)
{
  constexpr float kEps = 1e-3f;
  return x >= box[0] - kEps && x <= box[2] + kEps &&
         y >= box[1] - kEps && y <= box[3] + kEps;
}

constexpr float kRes = 0.5f;
}  // namespace

TEST(SfcBoundFrameTest, BoundContainsQueryPointWithNegativeOrigin)
{
  // 原点取非整栅格值（-2.13/0.5 = -4.26），顺带说明「世界坐标不一定落在栅格整数倍上」
  const float ox = -2.13f;
  const float oy = -3.27f;

  SfcSquare sfc;
  sfc.setMap(makeFreeMap(), kRes, Eigen::Vector2f(ox, oy));

  const Eigen::Vector2f point(-1.0f, -2.0f);
  const auto box = sfc.getBound(point.x(), point.y(), 2.0f);

  ASSERT_TRUE(std::all_of(box.begin(), box.end(), [](float v) {return std::isfinite(v);}));
  EXPECT_TRUE(contains(box, point.x(), point.y()))
    << "方块 x[" << box[0] << ", " << box[2] << "] y[" << box[1] << ", " << box[3] << "]"
    << " 不包含查询点 (" << point.x() << ", " << point.y() << ")";
  EXPECT_LT(box[0], box[2]) << "走廊块应该有面积（不是退化成点）";
  EXPECT_LT(box[1], box[3]);

  // 四条边必须落在「原点 + 整数格 * 分辨率」上，否则说明回算时原点没补对。
  for (float edge : {box[0], box[2]}) {
    const float index = (edge - ox) / kRes;
    EXPECT_NEAR(index, std::round(index), 1e-3f) << "边缘 " << edge << " 不在栅格线上";
  }
  for (float edge : {box[1], box[3]}) {
    const float index = (edge - oy) / kRes;
    EXPECT_NEAR(index, std::round(index), 1e-3f) << "边缘 " << edge << " 不在栅格线上";
  }
}

TEST(SfcBoundFrameTest, CorridorSquaresContainTheirWaypoints)
{
  const float ox = -2.0f;
  const float oy = -3.0f;

  SfcSquare sfc;
  sfc.setMap(makeFreeMap(), kRes, Eigen::Vector2f(ox, oy));

  const std::vector<Eigen::Vector2f> path = {
    {-1.0f, -2.0f}, {0.0f, -1.0f}, {1.0f, -0.5f}, {0.5f, 0.0f}};
  const auto corridor = sfc.getCorridor(path, 2.0f);

  ASSERT_EQ(corridor.corridor.size(), path.size());
  ASSERT_EQ(corridor.index.size(), path.size());
  for (std::size_t i = 0; i < path.size(); ++i) {
    const auto & box = corridor.corridor[i];
    EXPECT_EQ(corridor.index[i], static_cast<int>(i));
    EXPECT_TRUE(contains(box, path[i].x(), path[i].y()))
      << "第 " << i << " 块 x[" << box[0] << ", " << box[2] << "] y["
      << box[1] << ", " << box[3] << "] 不包含路径点 ("
      << path[i].x() << ", " << path[i].y() << ")";
  }
}

TEST(SfcBoundFrameTest, ShrinkKeepsQueryPointInside)
{
  // MinimumSnap 在碰撞迭代时会对自动走廊调用 shrink()（见 update()），
  // 传 limits 保证收缩后仍包含控制点。原点同样是负值。
  const float ox = -2.0f;
  const float oy = -3.0f;

  SfcSquare sfc;
  sfc.setMap(makeFreeMap(), kRes, Eigen::Vector2f(ox, oy));

  const Eigen::Vector2f point(0.0f, -1.0f);
  const auto bound = sfc.getBound(point.x(), point.y(), 2.0f);
  const auto shrunk = SfcSquare::shrink(bound, 0.3f, {point});

  EXPECT_TRUE(contains(shrunk, point.x(), point.y()));
  EXPECT_LE(shrunk[2] - shrunk[0], bound[2] - bound[0] + 1e-4f);
  EXPECT_LE(shrunk[3] - shrunk[1], bound[3] - bound[1] + 1e-4f);
}
