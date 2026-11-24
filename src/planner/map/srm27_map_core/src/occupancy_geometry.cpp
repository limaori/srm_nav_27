// Copyright 2026 SRM27 Navigation Team
// Licensed under the Apache License, Version 2.0.

#include "srm27_map_core/occupancy_geometry.hpp"

#include <algorithm>
#include <cmath>

namespace srm27_map_core
{
namespace
{

bool cellBlocked(const GridSnapshot & map, int mx, int my, bool unknown_is_obstacle)
{
  const CellState state = map.cell(mx, my);
  return state == CellState::kOccupied ||
         (unknown_is_obstacle && state == CellState::kUnknown);
}

Eigen::Vector2d gridPoint(const GridSnapshot & map, const Eigen::Vector2f & point)
{
  return (point.cast<double>() -
         Eigen::Vector2d(map.originX(), map.originY())) / map.resolution();
}

bool touchesCell(const Eigen::Vector2d & start, const Eigen::Vector2d & delta, int x, int y)
{
  double lower = 0.0;
  double upper = 1.0;
  for (int axis = 0; axis < 2; ++axis) {
    const double cell = axis == 0 ? x : y;
    if (std::abs(delta[axis]) < 1e-12) {
      if (start[axis] < cell || start[axis] > cell + 1.0) {
        return false;
      }
    } else {
      const double a = (cell - start[axis]) / delta[axis];
      const double b = (cell + 1.0 - start[axis]) / delta[axis];
      lower = std::max(lower, std::min(a, b));
      upper = std::min(upper, std::max(a, b));
      if (lower > upper + 1e-10) {
        return false;
      }
    }
  }
  return true;
}

}  // namespace

bool pointIsFree(
  const GridSnapshot & map, const Eigen::Vector2f & point, bool unknown_is_obstacle)
{
  if (!map.valid() || !point.allFinite()) {
    return false;
  }
  const auto cell = gridPoint(map, point);
  if (
    cell.x() < 0 || cell.y() < 0 || cell.x() >= map.sizeX() || cell.y() >= map.sizeY()) {
    return false;
  }
  return !cellBlocked(
    map, static_cast<int>(cell.x()), static_cast<int>(cell.y()), unknown_is_obstacle);
}

bool segmentIsFree(
  const GridSnapshot & map, const Eigen::Vector2f & from, const Eigen::Vector2f & to,
  bool unknown_is_obstacle)
{
  if (!pointIsFree(map, from, unknown_is_obstacle) ||
    !pointIsFree(map, to, unknown_is_obstacle))
  {
    return false;
  }
  const Eigen::Vector2d a = gridPoint(map, from);
  const Eigen::Vector2d b = gridPoint(map, to);
  const int x0 = std::max(0, static_cast<int>(std::floor(std::min(a.x(), b.x()))) - 1);
  const int y0 = std::max(0, static_cast<int>(std::floor(std::min(a.y(), b.y()))) - 1);
  const int x1 = std::min(map.sizeX() - 1, static_cast<int>(std::floor(std::max(a.x(), b.x()))));
  const int y1 = std::min(map.sizeY() - 1, static_cast<int>(std::floor(std::max(a.y(), b.y()))));
  for (int y = y0; y <= y1; ++y) {
    for (int x = x0; x <= x1; ++x) {
      if (cellBlocked(map, x, y, unknown_is_obstacle) && touchesCell(a, b - a, x, y)) {
        return false;
      }
    }
  }
  return true;
}

bool pathIsFree(
  const GridSnapshot & map, const std::vector<Eigen::Vector2f> & path,
  bool unknown_is_obstacle)
{
  if (path.size() < 2) {
    return false;
  }
  for (std::size_t i = 1; i < path.size(); ++i) {
    if (!segmentIsFree(map, path[i - 1], path[i], unknown_is_obstacle)) {
      return false;
    }
  }
  return true;
}

std::optional<Eigen::Vector2f> nearestFreePoint(
  const GridSnapshot & map, const Eigen::Vector2f & requested, float max_radius,
  bool unknown_is_obstacle)
{
  if (
    !map.valid() || !requested.allFinite() || !std::isfinite(max_radius) || max_radius < 0)
  {
    return std::nullopt;
  }
  if (pointIsFree(map, requested, unknown_is_obstacle)) {
    return requested;
  }
  const auto cell = gridPoint(map, requested);
  const double radius = static_cast<double>(max_radius) / map.resolution();
  const int x0 = static_cast<int>(std::clamp(cell.x() - radius, 0.0, double(map.sizeX())));
  const int x1 = static_cast<int>(std::clamp(cell.x() + radius, -1.0, double(map.sizeX() - 1)));
  const int y0 = static_cast<int>(std::clamp(cell.y() - radius, 0.0, double(map.sizeY())));
  const int y1 = static_cast<int>(std::clamp(cell.y() + radius, -1.0, double(map.sizeY() - 1)));
  float best_distance = max_radius;
  std::optional<Eigen::Vector2f> result;
  for (int y = y0; y <= y1; ++y) {
    for (int x = x0; x <= x1; ++x) {
      if (cellBlocked(map, x, y, unknown_is_obstacle)) {
        continue;
      }
      double wx = 0.0;
      double wy = 0.0;
      map.cellToWorld(x, y, wx, wy);
      const Eigen::Vector2f center(static_cast<float>(wx), static_cast<float>(wy));
      const float distance = (center - requested).norm();
      if (distance <= best_distance) {
        best_distance = distance;
        result = center;
      }
    }
  }
  return result;
}

}  // namespace srm27_map_core
