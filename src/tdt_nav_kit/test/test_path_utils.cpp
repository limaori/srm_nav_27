// Copyright 2026 T-DT
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "tdt_nav_kit/path_utils.hpp"

namespace
{

using tdt_nav_kit::decimatePath;
using tdt_nav_kit::PointVec;
using tdt_nav_kit::resamplePath;
using tdt_nav_kit::roundCorners;

PointVec makeLine(float x0, float y0, float x1, float y1, int n)
{
  PointVec pts;
  for (int i = 0; i < n; ++i) {
    const float t = static_cast<float>(i) / static_cast<float>(n - 1);
    pts.emplace_back(x0 + t * (x1 - x0), y0 + t * (y1 - y0));
  }
  return pts;
}

double maxStep(const PointVec & pts)
{
  double m = 0.0;
  for (std::size_t i = 0; i + 1 < pts.size(); ++i) {
    m = std::max(m, static_cast<double>((pts[i + 1] - pts[i]).norm()));
  }
  return m;
}

/// 相邻三点之间的最大转角（度）
double maxTurnDeg(const PointVec & pts)
{
  double m = 0.0;
  for (std::size_t i = 1; i + 1 < pts.size(); ++i) {
    const Eigen::Vector2f v1 = pts[i] - pts[i - 1];
    const Eigen::Vector2f v2 = pts[i + 1] - pts[i];
    if (v1.norm() < 1e-6f || v2.norm() < 1e-6f) {
      continue;
    }
    double c = v1.dot(v2) / (v1.norm() * v2.norm());
    c = std::clamp(c, -1.0, 1.0);
    m = std::max(m, std::acos(c) * 180.0 / M_PI);
  }
  return m;
}

}  // namespace

TEST(PathUtils, ResampleRejectsDegenerateInput)
{
  PointVec empty;
  EXPECT_TRUE(resamplePath(empty, 0.3f, 100).empty());

  const PointVec one{Eigen::Vector2f(1.0f, 2.0f)};
  EXPECT_EQ(resamplePath(one, 0.3f, 100).size(), 1u);

  const PointVec two{Eigen::Vector2f(0.0f, 0.0f), Eigen::Vector2f(1.0f, 0.0f)};
  // step <= 0 表示不处理
  EXPECT_EQ(resamplePath(two, 0.0f, 100).size(), 2u);
  EXPECT_EQ(resamplePath(two, -1.0f, 100).size(), 2u);
}

TEST(PathUtils, ResampleHonorsStepAndKeepsEndpoints)
{
  const PointVec line{Eigen::Vector2f(0.0f, 0.0f), Eigen::Vector2f(1.0f, 0.0f)};
  const PointVec dense = resamplePath(line, 0.3f, 1000);

  ASSERT_GE(dense.size(), 4u);
  EXPECT_LE(maxStep(dense), 0.3 + 1e-4);
  EXPECT_NEAR(dense.front().x(), 0.0, 1e-6);
  EXPECT_NEAR(dense.back().x(), 1.0, 1e-6);
}

TEST(PathUtils, ResampleCapsPointCountAndKeepsLastPoint)
{
  const PointVec line{Eigen::Vector2f(0.0f, 0.0f), Eigen::Vector2f(10.0f, 0.0f)};
  const PointVec capped = resamplePath(line, 0.01f, 10);

  EXPECT_LE(capped.size(), 10u);
  ASSERT_FALSE(capped.empty());
  // 截断后末点仍必须是原路径末点，否则路径会被偷偷缩短
  EXPECT_NEAR(capped.back().x(), 10.0, 1e-6);
}

TEST(PathUtils, RoundCornersLeavesStraightLineUntouched)
{
  const PointVec line{
    Eigen::Vector2f(0.0f, 0.0f), Eigen::Vector2f(1.0f, 0.0f),
    Eigen::Vector2f(2.0f, 0.0f)};
  const PointVec out = roundCorners(line, 0.4f, 25.0f, 0.2f);
  EXPECT_EQ(out.size(), line.size());
}

TEST(PathUtils, RoundCornersNeedsThreePointsAndPositiveCut)
{
  const PointVec two{Eigen::Vector2f(0.0f, 0.0f), Eigen::Vector2f(1.0f, 0.0f)};
  EXPECT_EQ(roundCorners(two, 0.4f, 25.0f, 0.2f).size(), 2u);

  const PointVec corner{
    Eigen::Vector2f(0.0f, 0.0f), Eigen::Vector2f(1.0f, 0.0f),
    Eigen::Vector2f(1.0f, 1.0f)};
  // cut <= 0 表示不切角
  EXPECT_EQ(roundCorners(corner, 0.0f, 25.0f, 0.2f).size(), 3u);
}

TEST(PathUtils, RoundCornersSmoothsRightAngle)
{
  // 原始为 90° 直角
  const PointVec corner{
    Eigen::Vector2f(0.0f, 0.0f), Eigen::Vector2f(1.0f, 0.0f),
    Eigen::Vector2f(1.0f, 1.0f)};
  ASSERT_NEAR(maxTurnDeg(corner), 90.0, 1e-3);

  const PointVec out = roundCorners(corner, 0.3f, 25.0f, 0.05f);
  ASSERT_GT(out.size(), corner.size());

  // 端点不变
  EXPECT_NEAR((out.front() - corner.front()).norm(), 0.0, 1e-6);
  EXPECT_NEAR((out.back() - corner.back()).norm(), 0.0, 1e-6);

  // 转角被显著削平（切角 0.3 m 时理论上远小于 45°）
  EXPECT_LT(maxTurnDeg(out), 45.0);

  // 曲线落在原折线的包围盒内（不会甩到外面撞墙）
  for (const auto & p : out) {
    EXPECT_GE(p.x(), -1e-4f);
    EXPECT_LE(p.x(), 1.0f + 1e-4f);
    EXPECT_GE(p.y(), -1e-4f);
    EXPECT_LE(p.y(), 1.0f + 1e-4f);
  }
}

TEST(PathUtils, DecimateRejectsDegenerateInput)
{
  PointVec empty;
  EXPECT_TRUE(decimatePath(empty, 0.8f, 100).empty());

  const PointVec two{Eigen::Vector2f(0.0f, 0.0f), Eigen::Vector2f(1.0f, 0.0f)};
  EXPECT_EQ(decimatePath(two, 0.8f, 100).size(), 2u);

  const PointVec line = makeLine(0.0f, 0.0f, 5.0f, 0.0f, 11);
  // step <= 0 表示不处理
  EXPECT_EQ(decimatePath(line, 0.0f, 100).size(), line.size());
  EXPECT_EQ(decimatePath(line, -1.0f, 100).size(), line.size());
}

TEST(PathUtils, DecimateHonorsStepAndKeepsEndpoints)
{
  const PointVec line = makeLine(0.0f, 0.0f, 10.0f, 0.0f, 51);
  const PointVec out = decimatePath(line, 0.8f, 1000);

  ASSERT_GE(out.size(), 3u);
  EXPECT_LE(maxStep(out), 0.8 + 1e-4);
  EXPECT_NEAR(out.front().x(), 0.0, 1e-6);
  EXPECT_NEAR(out.back().x(), 10.0, 1e-6);
  // 均匀弧长：各段等长
  EXPECT_NEAR(maxStep(out), 10.0 / static_cast<double>(out.size() - 1), 1e-4);
}

TEST(PathUtils, DecimateCapsPointCount)
{
  const PointVec line = makeLine(0.0f, 0.0f, 10.0f, 0.0f, 1001);
  const PointVec out = decimatePath(line, 0.05f, 10);

  EXPECT_LE(out.size(), 10u);
  EXPECT_NEAR(out.back().x(), 10.0, 1e-6);
}

TEST(PathUtils, DecimateLetsCornerRoundingUseFullCut)
{
  // 0.2 m 采样的直角路径：不抽稀时切角被 0.45*段长 夹到 0.09 m；
  // 抽稀到 0.8 m 后 cut=0.4 才能用满，圆弧应更平缓（逐点转角更小）。
  PointVec dense = makeLine(0.0f, 0.0f, 4.0f, 0.0f, 21);
  const PointVec leg = makeLine(4.0f, 0.0f, 4.0f, 4.0f, 21);
  dense.insert(dense.end(), leg.begin() + 1, leg.end());
  ASSERT_NEAR(maxStep(dense), 0.2, 1e-3);

  const PointVec clustered = roundCorners(dense, 0.4f, 25.0f, 0.05f);
  const PointVec sparse = roundCorners(
    decimatePath(dense, 0.8f, 1000), 0.4f, 25.0f, 0.05f);

  EXPECT_LT(maxTurnDeg(sparse), maxTurnDeg(clustered));
  EXPECT_NEAR((sparse.front() - dense.front()).norm(), 0.0, 1e-5);
  EXPECT_NEAR((sparse.back() - dense.back()).norm(), 0.0, 1e-5);
}

TEST(PathUtils, RoundCornersRespectsAdjacentSegmentLimits)
{
  // 极短的相邻段：切角长度必须被 0.45*段长 夹住，否则曲线会越过下下个拐点
  const PointVec zig{
    Eigen::Vector2f(0.0f, 0.0f), Eigen::Vector2f(0.2f, 0.0f),
    Eigen::Vector2f(0.2f, 0.2f), Eigen::Vector2f(0.4f, 0.2f)};
  const PointVec out = roundCorners(zig, 1.0f, 25.0f, 0.05f);

  EXPECT_NEAR((out.front() - zig.front()).norm(), 0.0, 1e-6);
  EXPECT_NEAR((out.back() - zig.back()).norm(), 0.0, 1e-6);
  // 不允许出现回退（x 单调不减的分段不应被切出负增量）
  for (std::size_t i = 0; i + 1 < out.size(); ++i) {
    EXPECT_GE(out[i + 1].x(), out[i].x() - 1e-5f);
  }
}
