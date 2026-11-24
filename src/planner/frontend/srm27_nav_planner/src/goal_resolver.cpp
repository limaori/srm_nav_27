// Copyright 2026 SRM27 Navigation Team
// Licensed under the Apache License, Version 2.0.

#include "srm27_nav_planner/goal_resolver.hpp"

#include <cmath>

#include "srm27_map_core/occupancy_geometry.hpp"

namespace srm27_nav_planner
{

std::optional<Eigen::Vector2f> StableGoal::resolve(
  const srm27_map_core::GridSnapshot & map, const Eigen::Vector2f & requested,
  float max_radius, bool unknown_is_obstacle)
{
  if (max_radius == 0.0f) {
    return srm27_map_core::pointIsFree(map, requested, unknown_is_obstacle) ?
           std::optional<Eigen::Vector2f>(requested) : std::nullopt;
  }
  if (resolved_ && requested.allFinite() && (requested - requested_).norm() <= 1e-4f) {
    if (
      std::isfinite(max_radius) && max_radius >= 0 &&
      (*resolved_ - requested).norm() <= max_radius + 1e-5f &&
      srm27_map_core::pointIsFree(map, *resolved_, unknown_is_obstacle))
    {
      return resolved_;
    }
    return std::nullopt;
  }
  requested_ = requested;
  resolved_ = srm27_map_core::nearestFreePoint(
    map, requested, max_radius, unknown_is_obstacle);
  return resolved_;
}

}  // namespace srm27_nav_planner
