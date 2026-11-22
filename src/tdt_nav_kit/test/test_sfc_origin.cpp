// Copyright 2026 SRM27 Navigation Team
// Licensed under the Apache License, Version 2.0.
//
// 阶段二关键验证：P7 补丁（SfcSquare 原点符号）
//
// 上游 sfcSquare.cpp 原先用 (x + originPos)/mapping 反算索引，而同一文件的
// isOutside() 与 MinimumSnap::toCell() 用 (x - originPos)/mapping。上游 main.cpp
// 的 origin 恒为 0，两种写法等价，所以这个 bug 一直没暴露；Nav2 costmap 的原点
// 通常是负值，符号一错就会整体偏移甚至索引越界。
//
// 这里用「非零且为负」的原点直接断言索引换算，补丁前必然失败。

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <vector>

#include "minimumSnap.hpp"
#include "sfcSquare.hpp"

namespace
{
/// 10x10，0 = 障碍（SFC 语义），255 = 可通行。
SfcSquare::Map makeMap()
{
  SfcSquare::Map m(10, 10);
  m.setConstant(static_cast<unsigned char>(255));
  return m;
}
}  // namespace

TEST(SfcOriginPatchTest, IsOutsideRespectsNegativeOrigin)
{
  SfcSquare sfc;
  sfc.setMap(makeMap(), 0.5f, Eigen::Vector2f(-2.0f, -3.0f));

  // 世界坐标 (-2, -3) 就是地图左下角原点本身，必须判为「在图内」
  EXPECT_FALSE(sfc.isOutside(-2.0f, -3.0f));
  EXPECT_FALSE(sfc.isOutside(-2.0f + 0.25f, -3.0f + 0.25f));

  // 原点外侧一格仍应判为图外
  EXPECT_TRUE(sfc.isOutside(-2.5f, -3.0f));
  EXPECT_TRUE(sfc.isOutside(-2.0f, -3.5f));
}

TEST(SfcOriginPatchTest, ValueAtIndexMatchesWorldPosition)
{
  SfcSquare sfc;
  auto m = makeMap();
  m(0, 0) = 0;  // 世界 (-2, -3) 处放障碍
  sfc.setMap(m, 0.5f, Eigen::Vector2f(-2.0f, -3.0f));

  // 原点所在格 -> 障碍
  EXPECT_FLOAT_EQ(sfc.valueAt(-2.0f + 0.25f, -3.0f + 0.25f), 0.0f);
  // 偏移 (4, 7) 格 -> 世界 (0.25, 0.75)，该格为自由
  EXPECT_FLOAT_EQ(sfc.valueAt(0.25f, 0.75f), 255.0f);
  // 再做一次对称校验：偏移 (7, 4) 格 -> 世界 (1.75, -0.75)
  EXPECT_FLOAT_EQ(sfc.valueAt(1.75f, -0.75f), 255.0f);
}

TEST(SfcOriginPatchTest, LineInObstacleAgreesWithValueAt)
{
  // 同一套 world -> cell 换算被两个类各写了一遍：
  //   SfcSquare::valueAt   (sfcSquare.cpp, P7 补丁处)
  //   MinimumSnap::toCell  (minimumSnap.cpp:711-714，本来就是减号)
  // 两者不一致时（P7 之前正是如此），lineInObsticle 的结论会与地图实际内容矛盾。
  const float res = 0.5f;
  const float ox = -2.0f;
  const float oy = -3.0f;

  auto m = makeMap();
  for (int y = 0; y < 10; ++y) {
    m(y, 5) = 0;  // 竖墙：世界 x = ox + 5*res = 0.5
  }

  SfcSquare sfc;
  sfc.setMap(m, res, Eigen::Vector2f(ox, oy));

  MinimumSnap ms;
  ms.setMap(m, res, ox, oy);
  // 上游非严格模式硬编码 obstacleTolerance = 2（minimumSnap.cpp:737），
  // 允许检测线擦过最多 2 个障碍栅格，穿 1 格薄墙会被放过。
  // 这里开严格模式，才能验证坐标换算本身而不是容差策略。
  ms.setStrictCollision(true);

  // 墙两侧的点：一个必须判碰撞，一个必须判自由
  const Eigen::Vector2f left(-1.0f, -2.0f);
  const Eigen::Vector2f right(1.0f, -2.0f);
  const Eigen::Vector2f left2(-1.0f, 1.0f);

  EXPECT_GT(sfc.valueAt(left.x(), left.y()), 0.0f);
  EXPECT_EQ(sfc.valueAt(0.5f, -2.0f), 0.0f);

  EXPECT_TRUE(ms.lineInObsticle(left, right)) << "跨越墙体应判为碰撞";
  EXPECT_FALSE(ms.lineInObsticle(left, left2)) << "平行于墙体不应判碰撞";
}

TEST(MinimumSnapSolveTest, ClosedFormBackendProducesPathThroughWaypoints)
{
  MinimumSnap ms;
  ms.setMap(makeMap(), 0.5f, -2.0f, -3.0f);
  ms.setOrder(5);
  ms.setMaxDx(3);
  ms.setTL(2.0);

  const std::vector<Eigen::Vector2f> waypoints = {
    {-1.0f, -2.0f}, {0.0f, -1.0f}, {1.0f, -0.5f}};

  MinimumSnap::SolveInput input;
  input.setPath(waypoints);
  input.setBackend(MinimumSnap::Backend::Close);
  input.setMaxSpeed(2.0f);
  input.setMaxAcc(2.0f);
  std::vector<double> time_allocated =
    MinimumSnap::trapezoidalTimeAllocation(waypoints, 2.0f, 2.0f);
  input.setTimeAllocated(time_allocated);
  input.setNormTime(true);

  const auto out = ms.solve(input);
  ASSERT_TRUE(out.success) << "闭式后端求解失败，iter=" << out.iter;
  ASSERT_GT(out.path.size(), waypoints.size());

  // 端点必须落在给定首末点上，否则说明坐标换算（原点符号）有问题
  EXPECT_NEAR(out.path.front().x(), waypoints.front().x(), 1e-2f);
  EXPECT_NEAR(out.path.front().y(), waypoints.front().y(), 1e-2f);
  EXPECT_NEAR(out.path.back().x(), waypoints.back().x(), 1e-2f);
  EXPECT_NEAR(out.path.back().y(), waypoints.back().y(), 1e-2f);
}

TEST(MinimumSnapSolveTest, SolveIsDeterministic)
{
  auto run = []() {
      MinimumSnap ms;
      ms.setMap(makeMap(), 0.5f, -2.0f, -3.0f);
      ms.setOrder(5);
      ms.setMaxDx(3);
      ms.setTL(2.0);
      const std::vector<Eigen::Vector2f> waypoints = {
        {-1.0f, -2.0f}, {0.0f, -1.0f}, {1.0f, -0.5f}};
      MinimumSnap::SolveInput input;
      input.setPath(waypoints);
      input.setBackend(MinimumSnap::Backend::Close);
      std::vector<double> t =
        MinimumSnap::trapezoidalTimeAllocation(waypoints, 2.0f, 2.0f);
      input.setTimeAllocated(t);
      input.setNormTime(true);
      return ms.solve(input);
    };

  const auto a = run();
  const auto b = run();
  ASSERT_EQ(a.success, b.success);
  ASSERT_EQ(a.path.size(), b.path.size());
  for (std::size_t i = 0; i < a.path.size(); ++i) {
    EXPECT_FLOAT_EQ(a.path[i].x(), b.path[i].x());
    EXPECT_FLOAT_EQ(a.path[i].y(), b.path[i].y());
  }
}
