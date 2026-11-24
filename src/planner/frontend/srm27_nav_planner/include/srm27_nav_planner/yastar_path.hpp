// Copyright 2026 SRM27 Navigation Team
// Licensed under the Apache License, Version 2.0.

#ifndef SRM27_NAV_PLANNER__YASTAR_PATH_HPP_
#define SRM27_NAV_PLANNER__YASTAR_PATH_HPP_

#include <Eigen/Core>
#include <vector>

#include "srm27_map_core/grid_snapshot.hpp"

namespace srm27_nav_planner
{

/// YAstar 回溯输出 `origin + index * resolution`，对应栅格角点；Nav2 期望格中心。
/// 这里只转换中间点，首末端点保留解析后的精确世界坐标（起点/已吸附目标）。
/// 原始结果为空、起点/目标不可通行或起终点同格失败时返回空 vector。
std::vector<Eigen::Vector2f> nav2PathFromYastar(
  const srm27_map_core::GridSnapshot & map, const std::vector<Eigen::Vector2f> & raw,
  const Eigen::Vector2f & start, const Eigen::Vector2f & goal,
  bool unknown_is_obstacle = true);

}  // namespace srm27_nav_planner

#endif  // SRM27_NAV_PLANNER__YASTAR_PATH_HPP_
