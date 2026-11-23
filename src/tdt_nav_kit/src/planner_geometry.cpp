// Copyright 2026 SRM27 Navigation Team
// Licensed under the Apache License, Version 2.0.

#include "tdt_nav_kit/planner_geometry.hpp"

#include <algorithm>
#include <cmath>

namespace tdt_nav_kit
{
namespace
{
bool validMap(const CostmapSnapshot & map)
{
  return map.size_x > 0 && map.size_y > 0 && std::isfinite(map.resolution) && map.resolution > 0 &&
         std::isfinite(map.origin_x) && std::isfinite(map.origin_y) &&
         map.data.size() == static_cast<std::size_t>(map.size_x) * map.size_y;
}

Eigen::Vector2d gridPoint(const CostmapSnapshot & map, const Eigen::Vector2f & point)
{
  return (point.cast<double>() - Eigen::Vector2d(map.origin_x, map.origin_y)) / map.resolution;
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

bool pointIsFree(const CostmapSnapshot & map, const Eigen::Vector2f & point)
{
  if (!validMap(map) || !point.allFinite()) {
    return false;
  }
  const auto cell = gridPoint(map, point);
  if (cell.x() < 0 || cell.y() < 0 || cell.x() >= map.size_x || cell.y() >= map.size_y) {
    return false;
  }
  return map.data[static_cast<int>(cell.y()) * map.size_x + static_cast<int>(cell.x())] == 0;
}

bool segmentIsFree(
  const CostmapSnapshot & map, const Eigen::Vector2f & from, const Eigen::Vector2f & to)
{
  if (!pointIsFree(map, from) || !pointIsFree(map, to)) {
    return false;
  }
  const Eigen::Vector2d a = gridPoint(map, from);
  const Eigen::Vector2d b = gridPoint(map, to);
  const int x0 = std::max(0, static_cast<int>(std::floor(std::min(a.x(), b.x()))) - 1);
  const int y0 = std::max(0, static_cast<int>(std::floor(std::min(a.y(), b.y()))) - 1);
  const int x1 = std::min(map.size_x - 1, static_cast<int>(std::floor(std::max(a.x(), b.x()))));
  const int y1 = std::min(map.size_y - 1, static_cast<int>(std::floor(std::max(a.y(), b.y()))));
  for (int y = y0; y <= y1; ++y) {
    for (int x = x0; x <= x1; ++x) {
      if (map.data[y * map.size_x + x] != 0 && touchesCell(a, b - a, x, y)) {
        return false;
      }
    }
  }
  return true;
}

bool pathIsFree(const CostmapSnapshot & map, const std::vector<Eigen::Vector2f> & path)
{
  if (path.size() < 2) {
    return false;
  }
  for (std::size_t i = 1; i < path.size(); ++i) {
    if (!segmentIsFree(map, path[i - 1], path[i])) {
      return false;
    }
  }
  return true;
}

std::optional<Eigen::Vector2f> nearestFreePoint(
  const CostmapSnapshot & map, const Eigen::Vector2f & requested, float max_radius)
{
  if (!validMap(map) || !requested.allFinite() || !std::isfinite(max_radius) || max_radius < 0) {
    return std::nullopt;
  }
  if (pointIsFree(map, requested)) {
    return requested;
  }
  const auto cell = gridPoint(map, requested);
  const double radius = static_cast<double>(max_radius) / map.resolution;
  const int x0 = static_cast<int>(std::clamp(cell.x() - radius, 0.0, double(map.size_x)));
  const int x1 = static_cast<int>(std::clamp(cell.x() + radius, -1.0, double(map.size_x - 1)));
  const int y0 = static_cast<int>(std::clamp(cell.y() - radius, 0.0, double(map.size_y)));
  const int y1 = static_cast<int>(std::clamp(cell.y() + radius, -1.0, double(map.size_y - 1)));
  float best_distance = max_radius;
  std::optional<Eigen::Vector2f> result;
  for (int y = y0; y <= y1; ++y) {
    for (int x = x0; x <= x1; ++x) {
      if (map.data[y * map.size_x + x] != 0) {
        continue;
      }
      const Eigen::Vector2f center = Eigen::Vector2f(map.origin_x, map.origin_y) +
                                     map.resolution * Eigen::Vector2f(x + .5f, y + .5f);
      const float distance = (center - requested).norm();
      if (distance <= best_distance) {
        best_distance = distance;
        result = center;
      }
    }
  }
  return result;
}

std::optional<Eigen::Vector2f> StableGoal::resolve(
  const CostmapSnapshot & map, const Eigen::Vector2f & requested, float max_radius)
{
  if (max_radius == 0.0f) {
    return pointIsFree(map, requested) ? std::optional<Eigen::Vector2f>(requested) : std::nullopt;
  }
  if (resolved_ && requested.allFinite() && (requested - requested_).norm() <= 1e-4f) {
    if (
      std::isfinite(max_radius) && max_radius >= 0 &&
      (*resolved_ - requested).norm() <= max_radius + 1e-5f && pointIsFree(map, *resolved_)) {
      return resolved_;
    }
    return std::nullopt;
  }
  requested_ = requested;
  resolved_ = nearestFreePoint(map, requested, max_radius);
  return resolved_;
}

std::vector<Eigen::Vector2f> nav2PathFromYastar(
  const CostmapSnapshot & map, const std::vector<Eigen::Vector2f> & raw,
  const Eigen::Vector2f & start, const Eigen::Vector2f & goal)
{
  if (raw.empty() || !pointIsFree(map, start) || !pointIsFree(map, goal)) {
    return {};
  }
  if (raw.size() == 1) {
    const auto a = gridPoint(map, start).cast<int>().eval();
    const auto b = gridPoint(map, goal).cast<int>().eval();
    return a == b ? std::vector<Eigen::Vector2f>{start, goal} : std::vector<Eigen::Vector2f>{};
  }
  auto result = raw;
  for (auto & point : result) {
    point += Eigen::Vector2f::Constant(.5f * map.resolution);
  }
  result.front() = start;
  result.back() = goal;
  return result;
}

}  // namespace tdt_nav_kit
