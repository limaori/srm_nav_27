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

#include "srm27_minco_controller/local_path.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace srm27_minco_controller
{
namespace
{
constexpr double kEpsilon = 1.0e-6;

LocalPathEnd pointFailure(
  const Eigen::Vector2d & _point, const srm27_minco_core::Esdf2D & _esdf, double _clearance)
{
  const auto query = _esdf.query(_point.x(), _point.y());
  // The entire circular footprint must remain in the observed map rectangle.
  if (
    !query.valid || !_esdf.query(_point.x() - _clearance, _point.y()).valid ||
    !_esdf.query(_point.x() + _clearance, _point.y()).valid ||
    !_esdf.query(_point.x(), _point.y() - _clearance).valid ||
    !_esdf.query(_point.x(), _point.y() + _clearance).valid) {
    return LocalPathEnd::kMapBoundary;
  }
  if (query.insideObstacle() || query.distance < _clearance) {
    return LocalPathEnd::kBlocked;
  }
  return LocalPathEnd::kInvalid;
}

void appendPoint(std::vector<Eigen::Vector2d> & _points, const Eigen::Vector2d & _point)
{
  if (_points.empty() || (_points.back() - _point).norm() > kEpsilon) {
    _points.push_back(_point);
  }
}

bool clearSegment(
  const Eigen::Vector2d & _from, const Eigen::Vector2d & _to,
  const srm27_minco_core::Esdf2D & _esdf, double _clearance, double _step)
{
  const int count = std::max(1, static_cast<int>(std::ceil((_to - _from).norm() / _step)));
  for (int i = 0; i <= count; ++i) {
    const Eigen::Vector2d point = _from + (_to - _from) * (static_cast<double>(i) / count);
    if (pointFailure(point, _esdf, _clearance) != LocalPathEnd::kInvalid) {
      return false;
    }
  }
  return true;
}
}  // namespace

const char * toString(LocalPathEnd _end)
{
  switch (_end) {
    case LocalPathEnd::kGlobalGoal:
      return "global_goal";
    case LocalPathEnd::kHorizon:
      return "planning_horizon";
    case LocalPathEnd::kMapBoundary:
      return "map_boundary";
    case LocalPathEnd::kBlocked:
      return "blocked";
    case LocalPathEnd::kPathEnd:
      return "path_end";
    default:
      return "invalid";
  }
}

LocalPathResult extractLocalPath(
  const std::vector<Eigen::Vector2d> & _path, const Eigen::Vector2d & _position,
  const srm27_minco_core::Esdf2D & _esdf, const LocalPathConfig & _config, bool _path_ends_at_goal)
{
  LocalPathResult result;
  if (
    _path.size() < 2 || !_position.allFinite() || !_esdf.valid() ||
    !std::isfinite(_config.horizon) || _config.horizon <= 0.0 ||
    !std::isfinite(_config.required_clearance) || _config.required_clearance < 0.0 ||
    !std::isfinite(_config.terminal_speed) || _config.terminal_speed < 0.0 ||
    !std::isfinite(_config.max_speed) || _config.max_speed <= 0.0 ||
    !std::isfinite(_config.braking_deceleration) || _config.braking_deceleration <= 0.0 ||
    !std::isfinite(_config.reaction_latency) || _config.reaction_latency < 0.0) {
    return result;
  }
  for (const auto & point : _path) {
    if (!point.allFinite()) {
      return result;
    }
  }

  // Nearest waypoint + 1 skips a corner when the robot has not reached that waypoint yet.
  // Project onto segments instead, then retain that segment's forward endpoint.
  std::size_t segment_index = 0;
  double best_distance = std::numeric_limits<double>::infinity();
  Eigen::Vector2d projection = _path.front();
  for (std::size_t i = 0; i + 1 < _path.size(); ++i) {
    const Eigen::Vector2d delta = _path[i + 1] - _path[i];
    if (delta.squaredNorm() < kEpsilon * kEpsilon) {
      continue;
    }
    const double alpha =
      std::clamp((_position - _path[i]).dot(delta) / delta.squaredNorm(), 0.0, 1.0);
    const Eigen::Vector2d candidate = _path[i] + alpha * delta;
    const double distance = (candidate - _position).squaredNorm();
    if (distance < best_distance) {
      best_distance = distance;
      segment_index = i;
      projection = candidate;
    }
  }
  if (!std::isfinite(best_distance)) {
    return result;
  }

  std::vector<Eigen::Vector2d> route{_position};
  appendPoint(route, projection);
  for (std::size_t i = segment_index + 1; i < _path.size(); ++i) {
    appendPoint(route, _path[i]);
  }
  result.points.push_back(_position);
  const auto start_failure = pointFailure(_position, _esdf, _config.required_clearance);
  if (start_failure != LocalPathEnd::kInvalid) {
    result.end = start_failure;
    return result;
  }

  const double horizon = std::min(_config.horizon, 20.0);
  const double step = std::max(1.0e-4, std::min(0.05, 0.5 * _esdf.resolution()));
  double accumulated = 0.0;
  std::size_t next_index = route.size();
  for (std::size_t i = 1; i < route.size(); ++i) {
    const Eigen::Vector2d from = route[i - 1];
    const Eigen::Vector2d delta = route[i] - from;
    const double length = delta.norm();
    const double take = std::min(length, std::max(0.0, horizon - accumulated));
    const Eigen::Vector2d direction = delta / length;
    const int count = std::max(1, static_cast<int>(std::ceil(take / step)));
    Eigen::Vector2d last_safe = from;
    for (int j = 1; j <= count; ++j) {
      const Eigen::Vector2d point = from + direction * (take * j / count);
      const auto failure = pointFailure(point, _esdf, _config.required_clearance);
      if (failure != LocalPathEnd::kInvalid) {
        appendPoint(result.points, last_safe);
        result.end = failure;
        return result;
      }
      last_safe = point;
    }
    appendPoint(result.points, last_safe);
    accumulated += take;
    const bool route_finished = i + 1 == route.size() && take >= length - kEpsilon;
    if (route_finished) {
      result.end = _path_ends_at_goal ? LocalPathEnd::kGlobalGoal : LocalPathEnd::kPathEnd;
      return result;
    }
    if (accumulated >= horizon - kEpsilon) {
      result.end = LocalPathEnd::kHorizon;
      next_index = i;
      break;
    }
  }
  if (result.end != LocalPathEnd::kHorizon || result.points.size() < 2) {
    result.end = _path_ends_at_goal ? LocalPathEnd::kGlobalGoal : LocalPathEnd::kPathEnd;
    return result;
  }

  const double speed = std::min(_config.terminal_speed, _config.max_speed);
  if (speed <= kEpsilon) {
    return result;
  }
  // Check both the route continuation and a straight braking corridor aligned with the
  // terminal velocity. A corner or unknown/map edge must not turn into a cruise endpoint.
  const double stopping_distance = speed * _config.reaction_latency +
                                   speed * speed / (2.0 * _config.braking_deceleration) +
                                   _esdf.resolution();
  const Eigen::Vector2d end = result.points.back();
  const Eigen::Vector2d tangent = (end - result.points[result.points.size() - 2]).normalized();
  if (!clearSegment(
        end, end + stopping_distance * tangent, _esdf, _config.required_clearance, step)) {
    return result;
  }
  double remaining = stopping_distance;
  Eigen::Vector2d from = end;
  for (std::size_t i = next_index; i < route.size() && remaining > kEpsilon; ++i) {
    const Eigen::Vector2d delta = route[i] - from;
    const double length = delta.norm();
    if (length <= kEpsilon) {
      from = route[i];
      continue;
    }
    const double take = std::min(length, remaining);
    const Eigen::Vector2d to = from + delta * (take / length);
    if (!clearSegment(from, to, _esdf, _config.required_clearance, step)) {
      return result;
    }
    remaining -= take;
    from = route[i];
  }
  if (remaining <= kEpsilon) {
    result.terminal_speed = speed;
  }
  return result;
}

}  // namespace srm27_minco_controller
