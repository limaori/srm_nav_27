// Copyright 2026 SRM27 Navigation Team
// Licensed under the Apache License, Version 2.0.

#ifndef SRM27_NAV_PLANNER__GOAL_RESOLVER_HPP_
#define SRM27_NAV_PLANNER__GOAL_RESOLVER_HPP_

#include <Eigen/Core>
#include <optional>

#include "srm27_map_core/grid_snapshot.hpp"

namespace srm27_nav_planner
{

/// 同一请求目标在重规划之间保持已吸附的终点：
/// 该点后来被占据时规划失败，而不是静默换到另一个格，避免下游把路径刷新当成新目标。
class StableGoal
{
public:
  std::optional<Eigen::Vector2f> resolve(
    const srm27_map_core::GridSnapshot & map, const Eigen::Vector2f & requested,
    float max_radius, bool unknown_is_obstacle = true);

  void reset() { resolved_.reset(); }

private:
  Eigen::Vector2f requested_{Eigen::Vector2f::Zero()};
  std::optional<Eigen::Vector2f> resolved_;
};

}  // namespace srm27_nav_planner

#endif  // SRM27_NAV_PLANNER__GOAL_RESOLVER_HPP_
