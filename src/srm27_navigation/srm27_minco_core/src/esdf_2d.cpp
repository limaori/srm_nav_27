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

#include "srm27_minco_core/esdf_2d.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace srm27_minco_core
{

namespace
{

/// \brief 一维抛物线距离变换使用的“无穷大”哨兵值。
constexpr double kEdtInf = 1.0e20;

/// \brief 在无额外分配的缓冲区上执行一维精确距离变换。
void edt1dInPlace(
  const std::vector<double> & _f, std::vector<double> & _d, std::vector<int> & _v,
  std::vector<double> & _z)
{
  const int n = static_cast<int>(_f.size());
  std::fill(_d.begin(), _d.end(), kEdtInf);
  if (n <= 0) {
    return;
  }

  int k = 0;
  _v[0] = 0;
  _z[0] = -kEdtInf;
  _z[1] = kEdtInf;

  for (int q = 1; q < n; ++q) {
    double s = 0.0;
    while (true) {
      const int p = _v[static_cast<std::size_t>(k)];
      const double fq = _f[static_cast<std::size_t>(q)];
      const double fp = _f[static_cast<std::size_t>(p)];
      if (fq >= kEdtInf && fp >= kEdtInf) {
        s = kEdtInf;
      } else {
        s = ((fq + static_cast<double>(q) * q) - (fp + static_cast<double>(p) * p)) /
            (2.0 * static_cast<double>(q - p));
      }
      if (k <= 0 || s > _z[static_cast<std::size_t>(k)]) {
        break;
      }
      --k;
    }
    ++k;
    _v[static_cast<std::size_t>(k)] = q;
    _z[static_cast<std::size_t>(k)] = s;
    _z[static_cast<std::size_t>(k) + 1] = kEdtInf;
  }

  int kk = 0;
  for (int q = 0; q < n; ++q) {
    while (_z[static_cast<std::size_t>(kk) + 1] < static_cast<double>(q)) {
      ++kk;
    }
    const int p = _v[static_cast<std::size_t>(kk)];
    _d[static_cast<std::size_t>(q)] =
      static_cast<double>((q - p) * (q - p)) + _f[static_cast<std::size_t>(p)];
  }
}

}  // namespace

bool Esdf2D::computeEdt2D(
  int _width, int _height, const std::vector<bool> & _seed, std::vector<double> & _dist_sq_out)
{
  if (_width <= 0 || _height <= 0) {
    return false;
  }
  const std::size_t expected = static_cast<std::size_t>(_width) * static_cast<std::size_t>(_height);
  if (_seed.size() != expected) {
    return false;
  }

  std::vector<double> tmp(expected, kEdtInf);
  {
    std::vector<double> f_row(static_cast<std::size_t>(_width));
    std::vector<double> d_row(static_cast<std::size_t>(_width));
    std::vector<int> v_row(static_cast<std::size_t>(_width));
    std::vector<double> z_row(static_cast<std::size_t>(_width) + 1);
    for (int y = 0; y < _height; ++y) {
      const std::size_t row = static_cast<std::size_t>(y) * static_cast<std::size_t>(_width);
      for (int x = 0; x < _width; ++x) {
        f_row[static_cast<std::size_t>(x)] =
          _seed[row + static_cast<std::size_t>(x)] ? 0.0 : kEdtInf;
      }
      edt1dInPlace(f_row, d_row, v_row, z_row);
      for (int x = 0; x < _width; ++x) {
        tmp[row + static_cast<std::size_t>(x)] = d_row[static_cast<std::size_t>(x)];
      }
    }
  }

  _dist_sq_out.assign(expected, kEdtInf);
  {
    std::vector<double> f_col(static_cast<std::size_t>(_height));
    std::vector<double> d_col(static_cast<std::size_t>(_height));
    std::vector<int> v_col(static_cast<std::size_t>(_height));
    std::vector<double> z_col(static_cast<std::size_t>(_height) + 1);
    for (int x = 0; x < _width; ++x) {
      for (int y = 0; y < _height; ++y) {
        f_col[static_cast<std::size_t>(y)] = tmp
          [static_cast<std::size_t>(y) * static_cast<std::size_t>(_width) +
           static_cast<std::size_t>(x)];
      }
      edt1dInPlace(f_col, d_col, v_col, z_col);
      for (int y = 0; y < _height; ++y) {
        _dist_sq_out
          [static_cast<std::size_t>(y) * static_cast<std::size_t>(_width) +
           static_cast<std::size_t>(x)] = d_col[static_cast<std::size_t>(y)];
      }
    }
  }
  return true;
}

double Esdf2D::signedFromEdt(double _edt_sq, bool _occupied) const
{
  if (!std::isfinite(_edt_sq) || _edt_sq >= kEdtInf) {
    return config_.no_obstacle_distance;
  }
  double distance = std::sqrt(std::max(0.0, _edt_sq)) * resolution_;
  if (config_.conservative) {
    distance = std::max(0.0, distance - half_cell_diagonal_);
  }
  return _occupied ? -distance : distance;
}

bool Esdf2D::build(const GridSnapshot & _snapshot, const Config & _config)
{
  valid_ = false;
  has_seed_ = false;
  signed_distance_.clear();
  cells_.clear();
  config_ = _config;

  if (!_snapshot.valid()) {
    return false;
  }
  if (!std::isfinite(_config.no_obstacle_distance) || _config.no_obstacle_distance <= 0.0) {
    return false;
  }

  size_x_ = _snapshot.sizeX();
  size_y_ = _snapshot.sizeY();
  origin_x_ = _snapshot.originX();
  origin_y_ = _snapshot.originY();
  resolution_ = _snapshot.resolution();
  map_version_ = _snapshot.mapVersion();
  stamp_ = _snapshot.stamp();
  half_cell_diagonal_ = std::sqrt(2.0) * resolution_ * 0.5;

  const std::size_t count = _snapshot.cellCount();
  cells_.resize(count);
  std::vector<bool> obstacle_seed(count, false);
  std::vector<bool> free_seed(count, false);
  for (std::size_t i = 0; i < count; ++i) {
    int mx = 0;
    int my = 0;
    mx = static_cast<int>(i % static_cast<std::size_t>(size_x_));
    my = static_cast<int>(i / static_cast<std::size_t>(size_x_));
    const CellState state = _snapshot.cell(mx, my);
    cells_[i] = state;
    const bool is_obstacle =
      state == CellState::kOccupied || (_config.unknown_is_seed && state == CellState::kUnknown);
    obstacle_seed[i] = is_obstacle;
    free_seed[i] = !is_obstacle;
    has_seed_ = has_seed_ || is_obstacle;
  }

  std::vector<double> dist_to_obstacle;
  std::vector<double> dist_to_free;
  if (!computeEdt2D(size_x_, size_y_, obstacle_seed, dist_to_obstacle)) {
    return false;
  }
  if (!computeEdt2D(size_x_, size_y_, free_seed, dist_to_free)) {
    return false;
  }

  signed_distance_.resize(count);
  for (std::size_t i = 0; i < count; ++i) {
    if (!has_seed_) {
      signed_distance_[i] = config_.no_obstacle_distance;
      continue;
    }
    const bool occupied = obstacle_seed[i];
    signed_distance_[i] =
      occupied ? signedFromEdt(dist_to_free[i], true) : signedFromEdt(dist_to_obstacle[i], false);
  }

  valid_ = true;
  return true;
}

double Esdf2D::cellDistance(int _mx, int _my) const
{
  if (!valid_ || _mx < 0 || _my < 0 || _mx >= size_x_ || _my >= size_y_) {
    return -std::numeric_limits<double>::infinity();
  }
  return signed_distance_
    [static_cast<std::size_t>(_my) * static_cast<std::size_t>(size_x_) +
     static_cast<std::size_t>(_mx)];
}

EsdfQueryResult Esdf2D::query(double _x, double _y) const
{
  EsdfQueryResult result;
  if (!valid_ || !std::isfinite(_x) || !std::isfinite(_y)) {
    return result;
  }
  result.map_version = map_version_;

  const double fx = (_x - origin_x_) / resolution_;
  const double fy = (_y - origin_y_) / resolution_;
  // 查询点必须落在栅格单元中心张成的多边形内，否则插值会外推出无意义的值。
  if (
    fx < 0.0 || fy < 0.0 || fx > static_cast<double>(size_x_) ||
    fy > static_cast<double>(size_y_)) {
    return result;
  }

  double u = fx - 0.5;
  double v = fy - 0.5;
  int mx = static_cast<int>(std::floor(u));
  int my = static_cast<int>(std::floor(v));
  double lu = u - static_cast<double>(mx);
  double lv = v - static_cast<double>(my);

  // 双线性模板需要 mx+1、my+1；退化到最后一格时把模板整体前移。
  if (mx >= size_x_ - 1) {
    if (size_x_ < 2) {
      mx = 0;
      lu = 0.0;
    } else {
      mx = size_x_ - 2;
      lu = 1.0;
    }
  }
  if (my >= size_y_ - 1) {
    if (size_y_ < 2) {
      my = 0;
      lv = 0.0;
    } else {
      my = size_y_ - 2;
      lv = 1.0;
    }
  }
  if (mx < 0) {
    mx = 0;
    lu = 0.0;
  }
  if (my < 0) {
    my = 0;
    lv = 0.0;
  }

  const double f00 = cellDistance(mx, my);
  const double f10 = cellDistance(mx + 1, my);
  const double f01 = cellDistance(mx, my + 1);
  const double f11 = cellDistance(mx + 1, my + 1);

  result.cell_state = cells_
    [static_cast<std::size_t>(my) * static_cast<std::size_t>(size_x_) +
     static_cast<std::size_t>(mx)];
  result.valid = true;

  if (size_x_ < 2 || size_y_ < 2) {
    // 单行/单列地图无法做双线性插值，退化为最近格取值，明确不给出梯度。
    result.distance = f00;
    return result;
  }

  result.distance =
    (1.0 - lu) * (1.0 - lv) * f00 + lu * (1.0 - lv) * f10 + (1.0 - lu) * lv * f01 + lu * lv * f11;

  if (!has_seed_) {
    // 没有任何障碍种子：距离恒为 no_obstacle_distance，梯度为零，优化器不需要障碍项。
    result.distance = config_.no_obstacle_distance;
    result.gradient.setZero();
    result.gradient_valid = true;
    return result;
  }

  const Eigen::Vector2d gradient(
    ((1.0 - lv) * (f10 - f00) + lv * (f11 - f01)) / resolution_,
    ((1.0 - lu) * (f01 - f00) + lu * (f11 - f10)) / resolution_);
  if (gradient.allFinite()) {
    result.gradient = gradient;
    result.gradient_valid = true;
  }
  return result;
}

double Esdf2D::distanceAt(double _x, double _y) const
{
  const EsdfQueryResult result = query(_x, _y);
  if (!result.valid) {
    return std::numeric_limits<double>::infinity();
  }
  return result.distance;
}

}  // namespace srm27_minco_core
