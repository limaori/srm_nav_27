// Copyright 2026 SRM27 Navigation Team
// Licensed under the Apache License, Version 2.0.
//
// 不创建 ROS 节点，直接链接 tdt_nav_kit 库跑纯算法用例。
// 其中 SDF 用例用于验证 PATCHES.md 的 P1/P2/P4/P5：vendored 源码改为
// C++17、强制走无 OpenCV 的 Felzenszwalb 分支后，数值必须仍与独立参考一致。

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "yastar.hpp"

namespace
{

constexpr int kWidth = 12;
constexpr int kHeight = 8;
constexpr float kResolution = 0.05f;
constexpr float kOriginX = -0.30f;
constexpr float kOriginY = -0.20f;

/// 12x8，x=6 列从 y=0..5 是一堵墙，y=6/7 留缺口。
std::vector<int8_t> makeMap()
{
  std::vector<int8_t> data(kWidth * kHeight, 0);
  for (int y = 0; y <= 5; ++y) {
    data[y * kWidth + 6] = 1;
  }
  return data;
}

Eigen::Vector2f worldOf(int x, int y)
{
  return Eigen::Vector2f(
    kOriginX + static_cast<float>(x) * kResolution,
    kOriginY + static_cast<float>(y) * kResolution);
}

/// YAstar 含 std::mutex，不可拷贝/移动，因此就地配置而不是按值返回。
void configurePlanner(YAstar & planner, std::vector<int8_t> & data)
{
  // 只用 6 参重载：0 = 可行、非 0 = 占用，且同时设置 resolution / 原点。
  planner.setMap(kWidth, kHeight, kResolution, kOriginX, kOriginY, data);
  // 必须在 setMap 之后：内部按 occMap 尺寸重建掩码。
  planner.setMaskNumLayers(1);
  planner.setCostField(1.0f, [](float x) { return 0.5f / (0.1f + x); });
  planner.initCostMap();
}

}  // namespace

TEST(YAstarChainTest, FullChainProducesCollisionFreePath)
{
  auto data = makeMap();
  YAstar planner;
  configurePlanner(planner, data);

  const Eigen::Vector2f start = worldOf(1, 3);
  const Eigen::Vector2f goal = worldOf(10, 3);

  std::vector<Eigen::Vector2f> raw = planner.search(start, goal);
  ASSERT_GE(raw.size(), 2u) << "search 应返回可通行路径，而不是仅含起点的失败路径";

  std::vector<Eigen::Vector2f> simplified = planner.simplifyPath(raw, 0.1f);
  simplified = planner.simplifyPathHypot(simplified);
  ASSERT_GE(simplified.size(), 2u);
  EXPECT_LE(simplified.size(), raw.size()) << "化简不应增加路径点";

  // lineInObsticle 返回 true 表示碰撞；化简后逐段校验。
  for (std::size_t i = 0; i + 1 < simplified.size(); ++i) {
    EXPECT_FALSE(planner.lineInObsticle(simplified[i], simplified[i + 1]))
      << "第 " << i << " 段穿过障碍";
  }

  EXPECT_NEAR(simplified.front().x(), start.x(), kResolution);
  EXPECT_NEAR(simplified.front().y(), start.y(), kResolution);
  EXPECT_NEAR(simplified.back().x(), goal.x(), kResolution);
  EXPECT_NEAR(simplified.back().y(), goal.y(), kResolution);
}

TEST(YAstarChainTest, FailedSearchReturnsStartOnlyPath)
{
  auto data = makeMap();
  YAstar planner;
  configurePlanner(planner, data);

  const Eigen::Vector2f start = worldOf(1, 3);
  const Eigen::Vector2f far_away(100.0f, 100.0f);  // 地图外
  const std::vector<Eigen::Vector2f> path = planner.search(start, far_away);

  // 上游实现在越界/无解时返回「仅含 start 的单点路径」，不是空数组；
  // 插件据此判定失败并抛 PlannerException。
  EXPECT_LT(path.size(), 2u);
}

TEST(YAstarChainTest, SdfMatchesBruteForceNearestObstacle)
{
  auto data = makeMap();
  YAstar planner;
  configurePlanner(planner, data);

  const auto sdf = planner.getSDF();
  ASSERT_EQ(sdf.rows(), kHeight);
  ASSERT_EQ(sdf.cols(), kWidth);

  std::vector<Eigen::Vector2i> obstacles;
  for (int y = 0; y < kHeight; ++y) {
    for (int x = 0; x < kWidth; ++x) {
      if (data[y * kWidth + x] != 0) {
        obstacles.emplace_back(x, y);
      }
    }
  }
  ASSERT_FALSE(obstacles.empty());

  // 独立参考：到最近障碍格的欧氏距离 × resolution。不依赖补丁后的实现。
  for (int y = 0; y < kHeight; ++y) {
    for (int x = 0; x < kWidth; ++x) {
      float best = std::numeric_limits<float>::infinity();
      for (const auto & obs : obstacles) {
        const float dx = static_cast<float>(x - obs.x());
        const float dy = static_cast<float>(y - obs.y());
        best = std::min(best, std::sqrt(dx * dx + dy * dy) * kResolution);
      }
      EXPECT_NEAR(sdf(y, x), best, 2e-4f)
        << "格 (" << x << "," << y << ") 的 SDF 与暴力参考不符；"
        << "若此用例失败，请先用未打补丁的上游源码跑同一张图，"
        << "以区分是补丁问题还是上游实现本身的差异";
    }
  }
}

TEST(YAstarChainTest, SdfAndSearchAreDeterministic)
{
  auto data_a = makeMap();
  auto data_b = makeMap();
  YAstar a;
  YAstar b;
  configurePlanner(a, data_a);
  configurePlanner(b, data_b);

  const auto sdf_a = a.getSDF();
  const auto sdf_b = b.getSDF();
  for (int y = 0; y < kHeight; ++y) {
    for (int x = 0; x < kWidth; ++x) {
      EXPECT_FLOAT_EQ(sdf_a(y, x), sdf_b(y, x));
    }
  }

  const auto path_a = a.search(worldOf(1, 3), worldOf(10, 3));
  const auto path_b = b.search(worldOf(1, 3), worldOf(10, 3));
  ASSERT_EQ(path_a.size(), path_b.size());
  for (std::size_t i = 0; i < path_a.size(); ++i) {
    EXPECT_FLOAT_EQ(path_a[i].x(), path_b[i].x());
    EXPECT_FLOAT_EQ(path_a[i].y(), path_b[i].y());
  }
}
