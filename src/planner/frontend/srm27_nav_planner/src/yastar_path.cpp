// Copyright 2026 SRM27 Navigation Team
// Licensed under the Apache License, Version 2.0.

#include "srm27_nav_planner/yastar_path.hpp"

#include "srm27_map_core/occupancy_geometry.hpp"

namespace srm27_nav_planner
{
namespace
{

Eigen::Vector2d gridPoint(
  const srm27_map_core::GridSnapshot & map, const Eigen::Vector2f & point)
{
  return (point.cast<double>() -
         Eigen::Vector2d(map.originX(), map.originY())) / map.resolution();
}

}  // namespace

std::vector<Eigen::Vector2f> nav2PathFromYastar(
  const srm27_map_core::GridSnapshot & map, const std::vector<Eigen::Vector2f> & raw,
  const Eigen::Vector2f & start, const Eigen::Vector2f & goal, bool unknown_is_obstacle)
{
  if (
    raw.empty() || !srm27_map_core::pointIsFree(map, start, unknown_is_obstacle) ||
    !srm27_map_core::pointIsFree(map, goal, unknown_is_obstacle))
  {
    return {};
  }
  if (raw.size() == 1) {
    const auto a = gridPoint(map, start).cast<int>().eval();
    const auto b = gridPoint(map, goal).cast<int>().eval();
    return a == b ? std::vector<Eigen::Vector2f>{start, goal} : std::vector<Eigen::Vector2f>{};
  }
  auto result = raw;
  for (auto & point : result) {
    point += Eigen::Vector2f::Constant(
      static_cast<float>(0.5 * map.resolution()));
  }
  result.front() = start;
  result.back() = goal;
  return result;
}

}  // namespace srm27_nav_planner
