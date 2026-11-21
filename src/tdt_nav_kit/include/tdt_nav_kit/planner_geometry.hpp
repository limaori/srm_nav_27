// Copyright 2026 SRM27 Navigation Team
// Licensed under the Apache License, Version 2.0.

#ifndef TDT_NAV_KIT__PLANNER_GEOMETRY_HPP_
#define TDT_NAV_KIT__PLANNER_GEOMETRY_HPP_

#include <Eigen/Core>
#include <optional>
#include <vector>

#include "tdt_nav_kit/costmap_converter.hpp"

namespace tdt_nav_kit
{

bool pointIsFree(const CostmapSnapshot & map, const Eigen::Vector2f & point);

/// Check every occupied cell touched by the continuous segment, including corners.
bool segmentIsFree(
  const CostmapSnapshot & map, const Eigen::Vector2f & from, const Eigen::Vector2f & to);
bool pathIsFree(const CostmapSnapshot & map, const std::vector<Eigen::Vector2f> & path);

std::optional<Eigen::Vector2f> nearestFreePoint(
  const CostmapSnapshot & map, const Eigen::Vector2f & requested, float max_radius);

/// Preserve a resolved goal across replans of the same requested world position.
/// A newly blocked cached goal fails planning instead of silently moving the goal.
class StableGoal
{
public:
  std::optional<Eigen::Vector2f> resolve(
    const CostmapSnapshot & map, const Eigen::Vector2f & requested, float max_radius);
  void reset() { resolved_.reset(); }

private:
  Eigen::Vector2f requested_{Eigen::Vector2f::Zero()};
  std::optional<Eigen::Vector2f> resolved_;
};

/// YAstar reconstructs index * resolution + origin (cell corners). Convert only
/// the returned geometry to Nav2 cell centers, retaining exact resolved endpoints.
std::vector<Eigen::Vector2f> nav2PathFromYastar(
  const CostmapSnapshot & map, const std::vector<Eigen::Vector2f> & raw,
  const Eigen::Vector2f & start, const Eigen::Vector2f & goal);

}  // namespace tdt_nav_kit
#endif  // TDT_NAV_KIT__PLANNER_GEOMETRY_HPP_
