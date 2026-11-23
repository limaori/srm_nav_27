// 离线台架：只量 TdtSmoother 的 MinimumSnap QP 代价随输入点数的变化。
// 复刻 tdt_smoother.cpp 的参数（order 5 / maxDx 3 / backend path / normTime /
// collisionIter 6 / maxCorridorRange 2.5），只跳过 roundCorners。
#include <chrono>
#include <cmath>
#include <cstdio>
#include <vector>

#include "minimumSnap.hpp"

namespace
{
constexpr float kRes = 0.05f;
constexpr int kWidth = 1000;    // 50 m
constexpr int kHeight = 240;    // 12 m

MinimumSnap::Map makeMap()
{
  MinimumSnap::Map map(kHeight, kWidth);
  map.setConstant(255);  // 255 = 可通行
  for (int y = 0; y < kHeight; ++y) {
    for (int x = 0; x < kWidth; ++x) {
      // y < 1 m 与 y > 11 m 是墙（3 m 宽通道）
      if (y * kRes < 1.0f || y * kRes > 11.0f) {
        map(y, x) = 0;
      }
    }
  }
  return map;
}

std::vector<Eigen::Vector2f> makePath(int count, float spacing)
{
  std::vector<Eigen::Vector2f> path;
  path.reserve(count);
  for (int i = 0; i < count; ++i) {
    const float s = i * spacing;
    // 轻微横向摆动，模拟栅格 A* 的锯齿
    path.emplace_back(s, 6.0f + 0.03f * std::sin(s * 2.0f));
  }
  return path;
}

void run(int count, float spacing, const MinimumSnap::Map & map)
{
  MinimumSnap smoother;
  smoother.setMap(map, kRes, 0.0f, 0.0f);
  smoother.setOrder(5);
  smoother.setMaxDx(3);
  smoother.setStrictCollision(false);
  smoother.setTL(10.0);

  const auto points = makePath(count, spacing);

  MinimumSnap::SolveInput input;
  input.setPath(points);
  input.setBackend(MinimumSnap::Backend::OSQPPath);  // 实车 config: backend "path"
  input.setMaxSpeed(3.0f);
  input.setMaxAcc(1.0f);
  std::vector<double> ta = MinimumSnap::trapezoidalTimeAllocation(points, 3.0f, 1.0f);
  input.setTimeAllocated(ta);
  input.setNormTime(true);
  input.setCollisionCheckIter(6);
  input.setMaxCorridorRange(2.5f);
  input.setCorridorShrink(0.0f);

  const auto t0 = std::chrono::steady_clock::now();
  MinimumSnap::SolveOutput result = smoother.solve(input);
  const double ms =
    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();

  std::printf(
    "输入 %4d 点 (间距 %.3f m, 长 %6.1f m) -> 输出 %4zu 点, success=%d iter=%d, "
    "QP+检测 %8.1f ms (solve %6.1f ms)\n",
    count, static_cast<double>(spacing), static_cast<double>((count - 1) * spacing),
    result.path.size(), static_cast<int>(result.success), result.iter, ms,
    result.time * 1000.0);
}
}  // namespace

int main()
{
  const auto map = makeMap();
  run(11, 3.6f, map);    // YAstar 典型输出
  run(101, 0.36f, map);  // TdtSmoother 自己的输出密度
  run(598, 0.074f, map); // Theta* 典型输出（0.05 m 栅格）
  run(1200, 0.037f, map);  // Theta* 长路径上限
  return 0;
}
