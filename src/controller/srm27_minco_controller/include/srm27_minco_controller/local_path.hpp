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

#ifndef SRM27_MINCO_CONTROLLER__LOCAL_PATH_HPP_
#define SRM27_MINCO_CONTROLLER__LOCAL_PATH_HPP_

#include <Eigen/Core>
#include <vector>

#include "srm27_minco_core/esdf_2d.hpp"

namespace srm27_minco_controller
{

enum class LocalPathEnd { kInvalid, kGlobalGoal, kHorizon, kMapBoundary, kBlocked, kPathEnd };

const char * toString(LocalPathEnd _end);

struct LocalPathConfig
{
  double horizon{3.0};
  double required_clearance{0.38};
  double terminal_speed{0.0};
  double max_speed{1.5};
  double braking_deceleration{3.0};
  double reaction_latency{0.1};
};

struct LocalPathResult
{
  std::vector<Eigen::Vector2d> points{};
  LocalPathEnd end{LocalPathEnd::kInvalid};
  // Only a length-limited horizon with a checked braking continuation may retain speed.
  double terminal_speed{0.0};
};

/// Project onto path segments, preserve upcoming corners and check every clipped segment.
/// This produces a planning reference, never an executable trajectory.
LocalPathResult extractLocalPath(
  const std::vector<Eigen::Vector2d> & _path, const Eigen::Vector2d & _position,
  const srm27_minco_core::Esdf2D & _esdf, const LocalPathConfig & _config, bool _path_ends_at_goal);

}  // namespace srm27_minco_controller

#endif  // SRM27_MINCO_CONTROLLER__LOCAL_PATH_HPP_
