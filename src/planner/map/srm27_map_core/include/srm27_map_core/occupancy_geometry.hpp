// Copyright 2026 SRM27 Navigation Team
// Licensed under the Apache License, Version 2.0.

#ifndef SRM27_MAP_CORE__OCCUPANCY_GEOMETRY_HPP_
#define SRM27_MAP_CORE__OCCUPANCY_GEOMETRY_HPP_

#include <Eigen/Core>
#include <optional>
#include <vector>

#include "srm27_map_core/grid_snapshot.hpp"

namespace srm27_map_core
{

/// 未知格是否按障碍处理。默认 true（与 Nav2 / MINCO 的保守口径一致）；
/// YAstar 规划器按自己的 `treat_unknown_as_obstacle` 参数显式传入。
///
/// 所有函数都把地图外与非法点视为不可通行；`nearestFreePoint` 只返回地图内的格中心。

bool pointIsFree(
  const GridSnapshot & map, const Eigen::Vector2f & point, bool unknown_is_obstacle = true);

/// 检查连续线段接触到的所有占据格（含只碰到格角的薄障碍）。
bool segmentIsFree(
  const GridSnapshot & map, const Eigen::Vector2f & from, const Eigen::Vector2f & to,
  bool unknown_is_obstacle = true);

bool pathIsFree(
  const GridSnapshot & map, const std::vector<Eigen::Vector2f> & path,
  bool unknown_is_obstacle = true);

/// 在 `max_radius`（米，欧氏距离）内找最近的可通行格中心。
/// 请求点本身可通行时原样返回（保留亚格精度）。
std::optional<Eigen::Vector2f> nearestFreePoint(
  const GridSnapshot & map, const Eigen::Vector2f & requested, float max_radius,
  bool unknown_is_obstacle = true);

}  // namespace srm27_map_core

#endif  // SRM27_MAP_CORE__OCCUPANCY_GEOMETRY_HPP_
