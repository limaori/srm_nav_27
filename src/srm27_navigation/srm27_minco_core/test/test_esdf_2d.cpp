// Copyright 2026 SRM
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
#include <cstdint>
#include <limits>
#include <random>
#include <vector>

#include "srm27_minco_core/esdf_2d.hpp"
#include "srm27_minco_core/grid_snapshot.hpp"

namespace
{

// 逐个引入被测类型（不做 `using namespace` 级别的全局展开）。
using srm27_minco_core::CellState;
using srm27_minco_core::Esdf2D;
using srm27_minco_core::EsdfQueryResult;
using srm27_minco_core::GridSnapshot;

/// \brief 测试统一使用的分辨率（m），取 0.1 便于手算解析距离。
constexpr double kResolution = 0.1;

/// \brief 圆周率常量，避免依赖非标准的 M_PI 宏。
const double kPi = std::acos(-1.0);

/// \brief 保守化减去的半格对角线（m）：sqrt(2) / 2 * res。
double halfDiagonal() { return std::sqrt(2.0) * kResolution * 0.5; }

/// \brief 测试地图构造器：默认全自由格，可按格覆盖语义。
class GridBuilder
{
public:
  GridBuilder(int _size_x, int _size_y)
  : size_x_(_size_x),
    size_y_(_size_y),
    cells_(static_cast<std::size_t>(_size_x) * static_cast<std::size_t>(_size_y), CellState::kFree)
  {
  }

  GridBuilder & set(int _mx, int _my, CellState _state)
  {
    cells_[index(_mx, _my)] = _state;
    return *this;
  }

  /// \brief 把矩形区域 [x0,x1] x [y0,y1]（含端点）整体设为给定语义。
  GridBuilder & setBlock(int _x0, int _y0, int _x1, int _y1, CellState _state)
  {
    for (int my = _y0; my <= _y1; ++my) {
      for (int mx = _x0; mx <= _x1; ++mx) {
        set(mx, my, _state);
      }
    }
    return *this;
  }

  /// \brief 生成快照；左下角固定在原点。
  GridSnapshot make(std::uint64_t _map_version = 7u, double _stamp = 1.5) const
  {
    GridSnapshot snapshot;
    const bool ok =
      snapshot.reset(0.0, 0.0, kResolution, size_x_, size_y_, cells_, _map_version, _stamp);
    EXPECT_TRUE(ok);
    return snapshot;
  }

private:
  std::size_t index(int _mx, int _my) const
  {
    return static_cast<std::size_t>(_my) * static_cast<std::size_t>(size_x_) +
           static_cast<std::size_t>(_mx);
  }

  int size_x_;
  int size_y_;
  std::vector<CellState> cells_;
};

/// \brief O(n^2) 暴力 EDT：每格取到最近种子的格单位平方距离（无种子时为 inf）。
std::vector<double> bruteForceEdt(int _width, int _height, const std::vector<bool> & _seed)
{
  std::vector<double> result(_seed.size(), std::numeric_limits<double>::infinity());
  for (int y = 0; y < _height; ++y) {
    for (int x = 0; x < _width; ++x) {
      double best = std::numeric_limits<double>::infinity();
      for (int sy = 0; sy < _height; ++sy) {
        for (int sx = 0; sx < _width; ++sx) {
          if (!_seed
                [static_cast<std::size_t>(sy) * static_cast<std::size_t>(_width) +
                 static_cast<std::size_t>(sx)]) {
            continue;
          }
          const double dx = static_cast<double>(x - sx);
          const double dy = static_cast<double>(y - sy);
          best = std::min(best, dx * dx + dy * dy);
        }
      }
      result
        [static_cast<std::size_t>(y) * static_cast<std::size_t>(_width) +
         static_cast<std::size_t>(x)] = best;
    }
  }
  return result;
}

/// \brief 把 computeEdt2D 的结果与暴力基准逐格比较（相对误差 < 1e-9）。
void expectEdtMatchesBruteForce(int _width, int _height, const std::vector<bool> & _seed)
{
  std::vector<double> dist_sq;
  ASSERT_TRUE(Esdf2D::computeEdt2D(_width, _height, _seed, dist_sq));
  ASSERT_EQ(dist_sq.size(), _seed.size());
  const std::vector<double> reference = bruteForceEdt(_width, _height, _seed);
  for (int y = 0; y < _height; ++y) {
    for (int x = 0; x < _width; ++x) {
      const std::size_t i = static_cast<std::size_t>(y) * static_cast<std::size_t>(_width) +
                            static_cast<std::size_t>(x);
      if (!std::isfinite(reference[i])) {
        // 完全没有种子时不得返回 0 或 NaN，否则会污染有符号距离。
        EXPECT_TRUE(std::isfinite(dist_sq[i]));
        EXPECT_GE(dist_sq[i], 1.0e19) << "cell(" << x << "," << y << ")";
        continue;
      }
      EXPECT_NEAR(dist_sq[i], reference[i], 1.0e-9 * std::max(1.0, reference[i]))
        << "cell(" << x << "," << y << ")";
    }
  }
}

/// \brief 取格中心处的查询结果（双线性插值在格中心等于该格值）。
EsdfQueryResult queryCellCenter(
  const GridSnapshot & _snapshot, const Esdf2D & _esdf, int _mx, int _my)
{
  double x = 0.0;
  double y = 0.0;
  _snapshot.cellToWorld(_mx, _my, x, y);
  return _esdf.query(x, y);
}

}  // namespace

// ---------------------------------------------------------------------------
// computeEdt2D：与暴力基准对照
// ---------------------------------------------------------------------------

TEST(Esdf2DComputeEdt2D, BruteForce_RandomSeedMaps)
{
  const int width = 20;
  const int height = 15;
  const std::size_t count = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  const double densities[5] = {0.05, 0.2, 0.4, 0.6, 0.9};
  std::mt19937 generator(20261006u);
  std::uniform_real_distribution<double> uniform(0.0, 1.0);
  for (const double density : densities) {
    std::vector<bool> seed(count, false);
    for (std::size_t i = 0; i < count; ++i) {
      seed[i] = uniform(generator) < density;
    }
    // 固定保留一个种子，保证覆盖“存在种子”的分支。
    seed[0] = true;
    expectEdtMatchesBruteForce(width, height, seed);
  }
}

TEST(Esdf2DComputeEdt2D, BruteForce_FixedPatterns)
{
  const int width = 20;
  const int height = 15;
  const std::size_t count = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);

  // 单个种子位于角落。
  {
    std::vector<bool> seed(count, false);
    seed[0] = true;
    expectEdtMatchesBruteForce(width, height, seed);
  }
  // 单个种子位于中心。
  {
    std::vector<bool> seed(count, false);
    seed[7 * width + 9] = true;
    expectEdtMatchesBruteForce(width, height, seed);
  }
  // 全图都是种子：距离恒为 0。
  {
    std::vector<bool> seed(count, true);
    expectEdtMatchesBruteForce(width, height, seed);
  }
  // 棋盘：对角邻居也是种子，检验两遍一维变换不会漏掉对角项。
  {
    std::vector<bool> seed(count, false);
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        seed[static_cast<std::size_t>(y) * width + x] = ((x + y) % 2 == 0);
      }
    }
    expectEdtMatchesBruteForce(width, height, seed);
  }
  // 单列墙。
  {
    std::vector<bool> seed(count, false);
    for (int y = 0; y < height; ++y) {
      seed[static_cast<std::size_t>(y) * width + 7] = true;
    }
    expectEdtMatchesBruteForce(width, height, seed);
  }
  // 完全没有种子：输出必须是有限的大哨兵值。
  {
    std::vector<bool> seed(count, false);
    std::vector<double> dist_sq;
    ASSERT_TRUE(Esdf2D::computeEdt2D(width, height, seed, dist_sq));
    ASSERT_EQ(dist_sq.size(), count);
    for (std::size_t i = 0; i < count; ++i) {
      EXPECT_TRUE(std::isfinite(dist_sq[i])) << "index " << i;
      EXPECT_GE(dist_sq[i], 1.0e19) << "index " << i;
    }
  }
}

TEST(Esdf2DComputeEdt2D, InvalidArguments_ReturnFalse)
{
  std::vector<double> dist_sq;
  const std::vector<bool> seed_2x2(4, false);
  const std::vector<bool> seed_empty;
  EXPECT_FALSE(Esdf2D::computeEdt2D(0, 2, seed_empty, dist_sq));
  EXPECT_FALSE(Esdf2D::computeEdt2D(-3, 2, seed_empty, dist_sq));
  EXPECT_FALSE(Esdf2D::computeEdt2D(2, 0, seed_empty, dist_sq));
  EXPECT_FALSE(Esdf2D::computeEdt2D(2, -1, seed_empty, dist_sq));
  // mask 长度不匹配（过短、过长）都必须拒绝。
  EXPECT_FALSE(Esdf2D::computeEdt2D(2, 2, std::vector<bool>(3, false), dist_sq));
  EXPECT_FALSE(Esdf2D::computeEdt2D(2, 2, std::vector<bool>(5, false), dist_sq));
  EXPECT_FALSE(Esdf2D::computeEdt2D(2, 2, seed_empty, dist_sq));
  // 合法参数仍能正常工作。
  EXPECT_TRUE(Esdf2D::computeEdt2D(2, 2, seed_2x2, dist_sq));
  EXPECT_EQ(dist_sq.size(), 4u);
  EXPECT_GE(dist_sq[0], 1.0e19);
}

// ---------------------------------------------------------------------------
// GridSnapshot::reset 参数校验
// ---------------------------------------------------------------------------

TEST(GridSnapshotReset, InvalidParameters_ReturnFalse)
{
  const std::vector<CellState> cells_4(4, CellState::kFree);
  GridSnapshot snapshot;
  EXPECT_TRUE(snapshot.reset(0.0, 0.0, 0.1, 2, 2, cells_4, 1u, 0.0));
  ASSERT_TRUE(snapshot.valid());

  // 分辨率非正。
  EXPECT_FALSE(snapshot.reset(0.0, 0.0, 0.0, 2, 2, cells_4, 1u, 0.0));
  EXPECT_FALSE(snapshot.valid());
  EXPECT_FALSE(snapshot.reset(0.0, 0.0, -0.1, 2, 2, cells_4, 1u, 0.0));
  EXPECT_FALSE(snapshot.valid());
  // 尺寸非正。
  EXPECT_FALSE(snapshot.reset(0.0, 0.0, 0.1, 0, 2, cells_4, 1u, 0.0));
  EXPECT_FALSE(snapshot.valid());
  EXPECT_FALSE(snapshot.reset(0.0, 0.0, 0.1, 2, 0, cells_4, 1u, 0.0));
  EXPECT_FALSE(snapshot.valid());
  // cells 长度与尺寸不匹配。
  EXPECT_FALSE(snapshot.reset(0.0, 0.0, 0.1, 2, 2, std::vector<CellState>(3), 1u, 0.0));
  EXPECT_FALSE(snapshot.valid());
  // 原点非有限。
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(snapshot.reset(nan, 0.0, 0.1, 2, 2, cells_4, 1u, 0.0));
  EXPECT_FALSE(snapshot.valid());
  EXPECT_FALSE(snapshot.reset(0.0, inf, 0.1, 2, 2, cells_4, 1u, 0.0));
  EXPECT_FALSE(snapshot.valid());
  // 时间戳非有限同样拒绝。
  EXPECT_FALSE(snapshot.reset(0.0, 0.0, 0.1, 2, 2, cells_4, 1u, nan));
  EXPECT_FALSE(snapshot.valid());
  // 失败后对象不得再对外表现为可用：valid() 为假，且所有取值接口都退化为“未知”。
  // 注：实现只把 valid_ 置假，历史 cells_ 仍留在对象里，但 cell()/cellAt() 都会因
  // valid() 为假而返回 kUnknown，因此下面的断言只覆盖对外可见的语义。
  EXPECT_EQ(snapshot.cell(0, 0), CellState::kUnknown);
  EXPECT_EQ(snapshot.cellAt(0.05, 0.05), CellState::kUnknown);
  EXPECT_EQ(snapshot.buildSeedMask(true).size(), 0u);
  EXPECT_FALSE(snapshot.hasAnySeed(true));
}

TEST(GridSnapshotReset, ValidParameters_RoundTrip)
{
  std::vector<CellState> cells(6, CellState::kFree);
  cells[1] = CellState::kOccupied;
  cells[4] = CellState::kUnknown;
  GridSnapshot snapshot;
  ASSERT_TRUE(snapshot.reset(-1.0, -2.0, 0.1, 3, 2, cells, 42u, 3.25));
  EXPECT_TRUE(snapshot.valid());
  EXPECT_EQ(snapshot.sizeX(), 3);
  EXPECT_EQ(snapshot.sizeY(), 2);
  EXPECT_DOUBLE_EQ(snapshot.resolution(), 0.1);
  EXPECT_DOUBLE_EQ(snapshot.originX(), -1.0);
  EXPECT_DOUBLE_EQ(snapshot.originY(), -2.0);
  EXPECT_EQ(snapshot.mapVersion(), 42u);
  EXPECT_DOUBLE_EQ(snapshot.stamp(), 3.25);
  EXPECT_EQ(snapshot.cellCount(), 6u);
  EXPECT_EQ(snapshot.cell(1, 0), CellState::kOccupied);
  EXPECT_EQ(snapshot.cell(1, 1), CellState::kUnknown);
  EXPECT_TRUE(snapshot.isOccupied(1, 0));
  EXPECT_FALSE(snapshot.isOccupied(0, 0));
  EXPECT_EQ(snapshot.cell(-1, 0), CellState::kUnknown);
  EXPECT_EQ(snapshot.cell(3, 0), CellState::kUnknown);
  // worldToCell / cellToWorld 必须互逆（取格中心）。
  int mx = 0;
  int my = 0;
  double x = 0.0;
  double y = 0.0;
  snapshot.cellToWorld(2, 1, x, y);
  EXPECT_NEAR(x, -0.75, 1e-12);
  EXPECT_NEAR(y, -1.85, 1e-12);
  snapshot.worldToCell(x, y, mx, my);
  EXPECT_EQ(mx, 2);
  EXPECT_EQ(my, 1);
  EXPECT_EQ(snapshot.cellAt(x, y), CellState::kFree);
}

// ---------------------------------------------------------------------------
// 距离与符号：解析对照
// ---------------------------------------------------------------------------

TEST(Esdf2DCellDistance, SingleCellObstacle_MatchesAnalyticDistance)
{
  const int size = 9;
  const int obstacle_x = 4;
  const int obstacle_y = 4;
  GridBuilder builder(size, size);
  builder.set(obstacle_x, obstacle_y, CellState::kOccupied);
  const GridSnapshot snapshot = builder.make();
  Esdf2D esdf;
  ASSERT_TRUE(esdf.build(snapshot));
  ASSERT_TRUE(esdf.valid());
  ASSERT_TRUE(esdf.hasSeed());
  EXPECT_EQ(esdf.sizeX(), size);
  EXPECT_EQ(esdf.sizeY(), size);
  EXPECT_NEAR(esdf.halfCellDiagonal(), halfDiagonal(), 1e-15);

  const double half_diag = esdf.halfCellDiagonal();
  // 障碍格本身：到最近自由格 1 格，符号为负。
  EXPECT_LT(esdf.cellDistance(obstacle_x, obstacle_y), 0.0);
  EXPECT_NEAR(esdf.cellDistance(obstacle_x, obstacle_y), -(kResolution - half_diag), 1e-12);

  // 全图逐格与解析欧氏距离（格单位）对照。
  for (int my = 0; my < size; ++my) {
    for (int mx = 0; mx < size; ++mx) {
      if (mx == obstacle_x && my == obstacle_y) {
        continue;
      }
      const double d =
        std::hypot(static_cast<double>(mx - obstacle_x), static_cast<double>(my - obstacle_y));
      EXPECT_NEAR(esdf.cellDistance(mx, my), d * kResolution - half_diag, 1e-12)
        << "cell(" << mx << "," << my << ")";
    }
  }
  // 若干手算格：1、2、sqrt(2)、sqrt(8)、5 格。
  EXPECT_NEAR(esdf.cellDistance(obstacle_x + 1, obstacle_y), 1.0 * kResolution - half_diag, 1e-12);
  EXPECT_NEAR(esdf.cellDistance(obstacle_x + 2, obstacle_y), 2.0 * kResolution - half_diag, 1e-12);
  EXPECT_NEAR(
    esdf.cellDistance(obstacle_x + 1, obstacle_y + 1), std::sqrt(2.0) * kResolution - half_diag,
    1e-12);
  EXPECT_NEAR(
    esdf.cellDistance(obstacle_x + 2, obstacle_y + 2), std::sqrt(8.0) * kResolution - half_diag,
    1e-12);
  EXPECT_NEAR(
    esdf.cellDistance(obstacle_x + 3, obstacle_y + 4), 5.0 * kResolution - half_diag, 1e-12);
  // 自由区距离恒为正，不会被保守化压到负数。
  for (int my = 0; my < size; ++my) {
    for (int mx = 0; mx < size; ++mx) {
      if (mx == obstacle_x && my == obstacle_y) {
        continue;
      }
      EXPECT_GT(esdf.cellDistance(mx, my), 0.0) << "cell(" << mx << "," << my << ")";
    }
  }
}

TEST(Esdf2DCellDistance, SingleColumnWall_AllCellsAnalytic)
{
  const int size_x = 9;
  const int size_y = 7;
  const int wall_x = 4;
  GridBuilder builder(size_x, size_y);
  builder.setBlock(wall_x, 0, wall_x, size_y - 1, CellState::kOccupied);
  const GridSnapshot snapshot = builder.make();
  Esdf2D esdf;
  ASSERT_TRUE(esdf.build(snapshot));
  const double half_diag = esdf.halfCellDiagonal();

  // 墙格与自由格全部逐格对照：水平方向解析距离 = |dx| * res - half_diag。
  for (int my = 0; my < size_y; ++my) {
    for (int mx = 0; mx < size_x; ++mx) {
      const double expected =
        (mx == wall_x) ? -(kResolution - half_diag)
                       : std::abs(static_cast<double>(mx - wall_x)) * kResolution - half_diag;
      EXPECT_NEAR(esdf.cellDistance(mx, my), expected, 1e-12) << "cell(" << mx << "," << my << ")";
    }
  }

  // 防止出现“整片平坦区域”：沿同一行每远离墙一格，距离必须恰好增加一格。
  // 从 wall_x + 2 起比较，避免把墙格自身的负距离算进来。
  for (int mx = wall_x + 2; mx < size_x; ++mx) {
    EXPECT_NEAR(esdf.cellDistance(mx, 3) - esdf.cellDistance(mx - 1, 3), kResolution, 1e-12)
      << "mx=" << mx;
  }
  for (int mx = wall_x - 1; mx >= 1; --mx) {
    EXPECT_NEAR(esdf.cellDistance(mx - 1, 3) - esdf.cellDistance(mx, 3), kResolution, 1e-12)
      << "mx=" << mx;
  }
  std::vector<double> row;
  for (int mx = 0; mx < size_x; ++mx) {
    if (mx != wall_x) {
      row.push_back(esdf.cellDistance(mx, 3));
    }
  }
  ASSERT_EQ(row.size(), static_cast<std::size_t>(size_x - 1));
  std::sort(row.begin(), row.end());
  EXPECT_NEAR(row.front(), kResolution - half_diag, 1e-12);
  EXPECT_NEAR(row.back(), 4.0 * kResolution - half_diag, 1e-12);
  // 距离场关于墙左右对称，因此该行只有 4 个互不相同的净空层级。
  int distinct = 1;
  for (std::size_t i = 1; i < row.size(); ++i) {
    if (std::abs(row[i] - row[i - 1]) > 1e-12) {
      ++distinct;
    }
  }
  EXPECT_EQ(distinct, (size_x - 1) / 2);
}

TEST(Esdf2DCellDistance, OutOfRange_ReturnsNegativeInfinity)
{
  GridBuilder builder(5, 5);
  builder.set(2, 2, CellState::kOccupied);
  const GridSnapshot snapshot = builder.make();
  Esdf2D esdf;
  ASSERT_TRUE(esdf.build(snapshot));
  const double neg_inf = -std::numeric_limits<double>::infinity();
  EXPECT_EQ(esdf.cellDistance(-1, 0), neg_inf);
  EXPECT_EQ(esdf.cellDistance(0, -1), neg_inf);
  EXPECT_EQ(esdf.cellDistance(5, 0), neg_inf);
  EXPECT_EQ(esdf.cellDistance(0, 5), neg_inf);
  EXPECT_EQ(esdf.cellDistance(5, 5), neg_inf);
  EXPECT_TRUE(std::isfinite(esdf.cellDistance(4, 4)));

  // 距离场还没建立时同样返回 -inf。
  Esdf2D empty;
  EXPECT_FALSE(empty.valid());
  EXPECT_EQ(empty.cellDistance(0, 0), neg_inf);
  EXPECT_FALSE(empty.query(0.0, 0.0).valid);
}

TEST(Esdf2DCellDistance, SignedSign_InsideObstacleNegative)
{
  GridBuilder builder(7, 7);
  builder.setBlock(2, 2, 3, 3, CellState::kOccupied);
  const GridSnapshot snapshot = builder.make();
  Esdf2D esdf;
  ASSERT_TRUE(esdf.build(snapshot));
  for (int my = 0; my < 7; ++my) {
    for (int mx = 0; mx < 7; ++mx) {
      const bool inside = (mx >= 2 && mx <= 3 && my >= 2 && my <= 3);
      if (inside) {
        EXPECT_LT(esdf.cellDistance(mx, my), 0.0) << "cell(" << mx << "," << my << ")";
      } else {
        EXPECT_GT(esdf.cellDistance(mx, my), 0.0) << "cell(" << mx << "," << my << ")";
      }
    }
  }
  const EsdfQueryResult inside = queryCellCenter(snapshot, esdf, 2, 2);
  ASSERT_TRUE(inside.valid);
  EXPECT_LT(inside.distance, 0.0);
  EXPECT_EQ(inside.cell_state, CellState::kOccupied);
  EXPECT_TRUE(inside.insideObstacle());
  const EsdfQueryResult outside = queryCellCenter(snapshot, esdf, 0, 0);
  ASSERT_TRUE(outside.valid);
  EXPECT_GT(outside.distance, 0.0);
  EXPECT_EQ(outside.cell_state, CellState::kFree);
  EXPECT_FALSE(outside.insideObstacle());
}

// ---------------------------------------------------------------------------
// 斜缝：势谷中心
// ---------------------------------------------------------------------------

TEST(Esdf2DQuery, DiagonalSlit_ClearanceFinitePositive_AndGradientOutward)
{
  // 两个 5x5 障碍块只在角上相接：(4,4) 属于左下块，(5,5) 属于右上块，
  // 于是自由格 (5,4) 与 (4,5) 构成只有 1 格宽的斜缝。
  const int size = 10;
  GridBuilder builder(size, size);
  builder.setBlock(0, 0, 4, 4, CellState::kOccupied);
  builder.setBlock(5, 5, size - 1, size - 1, CellState::kOccupied);
  const GridSnapshot snapshot = builder.make();
  Esdf2D esdf;
  ASSERT_TRUE(esdf.build(snapshot));
  const double half_diag = esdf.halfCellDiagonal();
  const double expected_center = kResolution - half_diag;
  ASSERT_GT(expected_center, 0.0);

  // 缝中心：到两个障碍块都只有 1 格，距离有限且为正。
  EXPECT_NEAR(esdf.cellDistance(5, 4), expected_center, 1e-12);
  EXPECT_NEAR(esdf.cellDistance(4, 5), expected_center, 1e-12);
  const EsdfQueryResult lower = queryCellCenter(snapshot, esdf, 5, 4);
  const EsdfQueryResult upper = queryCellCenter(snapshot, esdf, 4, 5);
  ASSERT_TRUE(lower.valid);
  ASSERT_TRUE(upper.valid);
  EXPECT_TRUE(std::isfinite(lower.distance));
  EXPECT_TRUE(std::isfinite(upper.distance));
  EXPECT_GT(lower.distance, 0.0);
  EXPECT_GT(upper.distance, 0.0);

  // 缝中心之外的第一个自由格净空应明显变大（距离场不是一整片平坦值）。
  EXPECT_GT(esdf.cellDistance(6, 3), esdf.cellDistance(5, 4) + 1e-9);
  EXPECT_GT(esdf.cellDistance(3, 6), esdf.cellDistance(4, 5) + 1e-9);

  // 梯度方向指向缝外：下侧缝格的梯度应朝 -y（进入下方开阔区），
  // 上侧缝格的梯度应朝 -x（进入上方开阔区）。
  // 注：方案 §1.2 已说明势谷中心零梯度也可能是正确解，这里只要求
  // “指向缝外”，不要求缝中心梯度一定非零。
  ASSERT_TRUE(lower.gradient_valid);
  ASSERT_TRUE(upper.gradient_valid);
  const Eigen::Vector2d lower_out(0.0, -1.0);
  const Eigen::Vector2d upper_out(-1.0, 0.0);
  EXPECT_GT(lower.gradient.dot(lower_out), 0.0) << "grad=" << lower.gradient.transpose();
  EXPECT_GT(upper.gradient.dot(upper_out), 0.0) << "grad=" << upper.gradient.transpose();
  // 横穿缝的方向上不允许出现明显分量（缝两侧对称）。
  EXPECT_LE(std::abs(lower.gradient.x()), std::abs(lower.gradient.y()))
    << "grad=" << lower.gradient.transpose();
  EXPECT_LE(std::abs(upper.gradient.y()), std::abs(upper.gradient.x()))
    << "grad=" << upper.gradient.transpose();
}

// ---------------------------------------------------------------------------
// 圆形障碍：与解析距离对照
// ---------------------------------------------------------------------------

TEST(Esdf2DQuery, CircularObstacle_AnalyticDistanceBounds)
{
  // 半径 1.0 m 恰好是 10 格（res = 0.1），圆盘中心取格 (22,22) 的中心。
  const int size = 45;
  const double radius = 1.0;
  const double center_x = 2.25;
  const double center_y = 2.25;
  GridBuilder builder(size, size);
  for (int my = 0; my < size; ++my) {
    for (int mx = 0; mx < size; ++mx) {
      const double px = (static_cast<double>(mx) + 0.5) * kResolution;
      const double py = (static_cast<double>(my) + 0.5) * kResolution;
      if (std::hypot(px - center_x, py - center_y) <= radius) {
        builder.set(mx, my, CellState::kOccupied);
      }
    }
  }
  const GridSnapshot snapshot = builder.make();
  Esdf2D esdf;
  ASSERT_TRUE(esdf.build(snapshot));
  const double half_diag = esdf.halfCellDiagonal();

  // 8 个格对齐方向（0/45/.../315）：栅格化误差为 0，可套用方案 §11.1 的严格界。
  for (int direction = 0; direction < 8; ++direction) {
    const double angle = static_cast<double>(direction) * kPi / 4.0;
    for (int step = 1; step <= 8; ++step) {
      const double target_x =
        center_x + (radius + static_cast<double>(step) * kResolution) * std::cos(angle);
      const double target_y =
        center_y + (radius + static_cast<double>(step) * kResolution) * std::sin(angle);
      const int mx = static_cast<int>(std::lround(target_x / kResolution - 0.5));
      const int my = static_cast<int>(std::lround(target_y / kResolution - 0.5));
      ASSERT_GE(mx, 0);
      ASSERT_GE(my, 0);
      ASSERT_LT(mx, size);
      ASSERT_LT(my, size);
      double px = 0.0;
      double py = 0.0;
      snapshot.cellToWorld(mx, my, px, py);
      // 格单位解析距离：格中心到理想圆盘边界。
      const double d_cells = (std::hypot(px - center_x, py - center_y) - radius) / kResolution;
      ASSERT_GT(d_cells, 0.0) << "direction " << direction << " step " << step;
      const double analytic = std::sqrt(d_cells * d_cells) * kResolution;
      const double d_esdf = esdf.cellDistance(mx, my);
      EXPECT_GE(d_esdf, analytic - half_diag - 1e-9)
        << "direction " << direction << " step " << step;
      EXPECT_LE(d_esdf, analytic + 1e-9) << "direction " << direction << " step " << step;
    }
  }

  // 其它方向：种子集相对理想圆盘只会“向内退缩”，退缩量不超过约 1 格，
  // 因此只保证保守下界与 1 格量级的上界（详见方案 §6.1 的保守化约定）。
  for (int direction = 0; direction < 16; ++direction) {
    const double angle = static_cast<double>(direction) * kPi / 8.0;
    for (int step = 1; step <= 6; ++step) {
      const double target_x =
        center_x + (radius + static_cast<double>(step) * kResolution) * std::cos(angle);
      const double target_y =
        center_y + (radius + static_cast<double>(step) * kResolution) * std::sin(angle);
      const int mx = static_cast<int>(std::lround(target_x / kResolution - 0.5));
      const int my = static_cast<int>(std::lround(target_y / kResolution - 0.5));
      double px = 0.0;
      double py = 0.0;
      snapshot.cellToWorld(mx, my, px, py);
      const double d_cells = (std::hypot(px - center_x, py - center_y) - radius) / kResolution;
      if (d_cells <= 0.0) {
        continue;
      }
      const double analytic = std::sqrt(d_cells * d_cells) * kResolution;
      const double d_esdf = esdf.cellDistance(mx, my);
      EXPECT_GE(d_esdf, analytic - half_diag - 1e-9)
        << "direction " << direction << " step " << step;
      EXPECT_LE(d_esdf, analytic + kResolution + 1e-9)
        << "direction " << direction << " step " << step;
    }
  }

  // 远离圆盘的方向上距离应单调增加，且圆盘内部距离为负。
  EXPECT_GT(esdf.cellDistance(22 + 12, 22), esdf.cellDistance(22 + 11, 22));
  EXPECT_LT(esdf.cellDistance(22, 22), 0.0);
  EXPECT_LT(esdf.cellDistance(22 + 5, 22), 0.0);
}

// ---------------------------------------------------------------------------
// 梯度：中心差分交叉验证
// ---------------------------------------------------------------------------

TEST(Esdf2DQuery, Gradient_MatchesCentralDifference)
{
  const int size = 41;
  GridBuilder builder(size, size);
  builder.set(20, 20, CellState::kOccupied);
  builder.set(5, 5, CellState::kOccupied);
  const GridSnapshot snapshot = builder.make();
  Esdf2D esdf;
  ASSERT_TRUE(esdf.build(snapshot));

  // 采样点取格中心加固定偏移：偏移 0.31/0.27 格远大于 eps/res = 0.002 格，
  // 保证中心差分的两个端点仍落在同一个格内，比较的是同一个双线性分片。
  const double eps = 1e-4;
  double worst = 0.0;
  int checked = 0;
  for (int my = 8; my <= 30; my += 2) {
    for (int mx = 8; mx <= 30; mx += 2) {
      // 只取离障碍至少 4 格、离栅格边界至少 8 格的点：距离场在那里光滑。
      if (std::hypot(static_cast<double>(mx - 5), static_cast<double>(my - 5)) < 4.0) {
        continue;
      }
      if (std::hypot(static_cast<double>(mx - 20), static_cast<double>(my - 20)) < 4.0) {
        continue;
      }
      double cx = 0.0;
      double cy = 0.0;
      snapshot.cellToWorld(mx, my, cx, cy);
      const double px = cx + 0.31 * kResolution;
      const double py = cy - 0.27 * kResolution;
      const EsdfQueryResult result = esdf.query(px, py);
      ASSERT_TRUE(result.valid);
      ASSERT_TRUE(result.gradient_valid);
      EXPECT_GT(result.distance, 0.3) << "cell(" << mx << "," << my << ")";
      const double dx =
        (esdf.query(px + eps, py).distance - esdf.query(px - eps, py).distance) / (2.0 * eps);
      const double dy =
        (esdf.query(px, py + eps).distance - esdf.query(px, py - eps).distance) / (2.0 * eps);
      worst = std::max(
        worst, std::max(std::abs(dx - result.gradient.x()), std::abs(dy - result.gradient.y())));
      // 远场距离场的梯度接近单位向量；双线性插值的截断误差是 O((res/d)^2)，
      // 在 d ≈ 0.3 m、res = 0.1 m 时可达百分之几，因此容差取 5e-2。
      EXPECT_NEAR(result.gradient.norm(), 1.0, 5e-2) << "cell(" << mx << "," << my << ")";
      ++checked;
    }
  }
  EXPECT_GT(checked, 20);
  // 双线性插值的梯度就是同一分片双线性函数的解析导数，理论上与格内中心差分一致；
  // 这里按方案 §11.1 取 2e-2 作为宽松容差，实测误差在 1e-10 量级。
  EXPECT_LT(worst, 2e-2);
}

// ---------------------------------------------------------------------------
// 无种子、地图外与未知区域
// ---------------------------------------------------------------------------

TEST(Esdf2DBuild, NoSeedMap_UsesNoObstacleDistance)
{
  const int size = 5;
  GridBuilder builder(size, size);
  const GridSnapshot snapshot = builder.make();
  Esdf2D esdf;
  ASSERT_TRUE(esdf.build(snapshot));
  EXPECT_TRUE(esdf.valid());
  EXPECT_FALSE(esdf.hasSeed());

  const EsdfQueryResult result = queryCellCenter(snapshot, esdf, 2, 2);
  EXPECT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(result.distance, Esdf2D::Config().no_obstacle_distance);
  EXPECT_TRUE(result.gradient_valid);
  EXPECT_TRUE(result.gradient.isApprox(Eigen::Vector2d::Zero(), 1e-15));
  for (int my = 0; my < size; ++my) {
    for (int mx = 0; mx < size; ++mx) {
      EXPECT_DOUBLE_EQ(esdf.cellDistance(mx, my), 1000.0);
    }
  }

  // 自定义的 no_obstacle_distance 必须被采用。
  Esdf2D::Config config;
  config.no_obstacle_distance = 5.0;
  Esdf2D custom;
  ASSERT_TRUE(custom.build(snapshot, config));
  EXPECT_FALSE(custom.hasSeed());
  EXPECT_DOUBLE_EQ(custom.cellDistance(0, 0), 5.0);
  EXPECT_DOUBLE_EQ(custom.query(0.25, 0.25).distance, 5.0);
}

TEST(Esdf2DBuild, InvalidSnapshot_ReturnsFalse)
{
  Esdf2D esdf;
  const GridSnapshot invalid;
  EXPECT_FALSE(esdf.build(invalid));
  EXPECT_FALSE(esdf.valid());
  EXPECT_FALSE(esdf.hasSeed());
  EXPECT_EQ(esdf.cellDistance(0, 0), -std::numeric_limits<double>::infinity());
}

TEST(Esdf2DQuery, OutOfMap_Invalid)
{
  GridBuilder builder(5, 5);
  builder.set(4, 4, CellState::kOccupied);
  const GridSnapshot snapshot = builder.make();
  Esdf2D esdf;
  ASSERT_TRUE(esdf.build(snapshot));

  // 地图覆盖 [0, 0.5] x [0, 0.5]：边界属于地图内，越界一律无效。
  EXPECT_TRUE(esdf.query(0.0, 0.25).valid);
  EXPECT_TRUE(esdf.query(0.5, 0.25).valid);
  EXPECT_FALSE(esdf.query(-1e-9, 0.25).valid);
  EXPECT_FALSE(esdf.query(0.25, -1e-9).valid);
  EXPECT_FALSE(esdf.query(0.5 + 1e-9, 0.25).valid);
  EXPECT_FALSE(esdf.query(0.25, 0.5 + 1e-9).valid);
  // 非有限坐标同样无效。
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(esdf.query(nan, 0.25).valid);
  EXPECT_FALSE(esdf.query(0.25, std::numeric_limits<double>::infinity()).valid);
  // 失败时距离为 +inf（distanceAt 约定）。
  EXPECT_EQ(esdf.distanceAt(-0.5, 0.25), std::numeric_limits<double>::infinity());
  EXPECT_EQ(esdf.distanceAt(nan, 0.25), std::numeric_limits<double>::infinity());
}

TEST(Esdf2DQuery, UnknownCells_SeedSemantics)
{
  const int size = 5;
  GridBuilder builder(size, size);
  builder.set(0, 0, CellState::kOccupied);
  builder.set(3, 0, CellState::kUnknown);
  const GridSnapshot snapshot = builder.make();

  // unknown_is_seed = true（默认）：未知格作为种子，距离为负。
  Esdf2D seeded;
  Esdf2D::Config seeded_config;
  seeded_config.unknown_is_seed = true;
  ASSERT_TRUE(seeded.build(snapshot, seeded_config));
  EXPECT_TRUE(seeded.hasSeed());
  const double half_diag = seeded.halfCellDiagonal();
  EXPECT_LT(seeded.cellDistance(3, 0), 0.0);
  EXPECT_NEAR(seeded.cellDistance(3, 0), -(kResolution - half_diag), 1e-12);
  const EsdfQueryResult unknown = queryCellCenter(snapshot, seeded, 3, 0);
  ASSERT_TRUE(unknown.valid);
  EXPECT_LT(unknown.distance, 0.0);
  EXPECT_EQ(unknown.cell_state, CellState::kUnknown);
  EXPECT_TRUE(unknown.insideObstacle());

  // unknown_is_seed = false：未知格不再作为种子，只能作为“自由”参与距离场。
  Esdf2D unseeded;
  Esdf2D::Config unseeded_config;
  unseeded_config.unknown_is_seed = false;
  ASSERT_TRUE(unseeded.build(snapshot, unseeded_config));
  EXPECT_TRUE(unseeded.hasSeed());
  EXPECT_GT(unseeded.cellDistance(3, 0), 0.0);
  EXPECT_NEAR(unseeded.cellDistance(3, 0), 3.0 * kResolution - half_diag, 1e-12);
  // 只有未知格、没有真实障碍时，unknow_is_seed = false 使得整图没有种子。
  GridBuilder unknown_only(size, size);
  unknown_only.set(2, 2, CellState::kUnknown);
  const GridSnapshot unknown_snapshot = unknown_only.make();
  Esdf2D none_seeded;
  ASSERT_TRUE(none_seeded.build(unknown_snapshot, unseeded_config));
  EXPECT_FALSE(none_seeded.hasSeed());
  EXPECT_GT(none_seeded.cellDistance(2, 2), 0.0);
}

// ---------------------------------------------------------------------------
// 元数据
// ---------------------------------------------------------------------------

TEST(Esdf2DQuery, Metadata_MatchesSnapshot)
{
  GridBuilder builder(6, 4);
  builder.set(1, 1, CellState::kOccupied);
  const GridSnapshot snapshot = builder.make(42u, 3.25);
  Esdf2D esdf;
  ASSERT_TRUE(esdf.build(snapshot));
  EXPECT_EQ(esdf.mapVersion(), 42u);
  EXPECT_DOUBLE_EQ(esdf.stamp(), 3.25);
  EXPECT_EQ(esdf.resolution(), kResolution);
  const EsdfQueryResult result = queryCellCenter(snapshot, esdf, 4, 2);
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.map_version, snapshot.mapVersion());
  EXPECT_DOUBLE_EQ(snapshot.stamp(), 3.25);
}
