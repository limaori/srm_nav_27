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

#include "srm27_minco_core/replan_manager.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "srm27_minco_core/kinematics.hpp"

namespace srm27_minco_core
{

namespace
{

/// \brief 点到轨迹的最近距离（m）。按固定时间间隔采样，仅用于状态跳变检测。
double distanceToTrajectory(const Trajectory2D & _trajectory, const Eigen::Vector2d & _position)
{
  if (_trajectory.empty() || !_position.allFinite()) {
    return std::numeric_limits<double>::infinity();
  }
  const double total = _trajectory.totalDuration();
  const double step = std::max(1.0e-3, total / 200.0);
  double best = std::numeric_limits<double>::infinity();
  for (double t = 0.0; t <= total + 1.0e-9; t += step) {
    best = std::min(best, (_trajectory.positionAt(std::min(t, total)) - _position).norm());
  }
  return best;
}

}  // namespace

bool ReplanManager::configure(const Config & _config, std::string * _reason)
{
  const auto fail = [&_reason](const char * _message) {
    if (_reason != nullptr) {
      *_reason = _message;
    }
    return false;
  };
  configured_ = false;
  if (!isFinite(_config.replan_period) || _config.replan_period <= 0.0) {
    return fail("replan_period must be positive");
  }
  if (!isFinite(_config.map_change_min_period) || _config.map_change_min_period < 0.0) {
    return fail("map_change_min_period must be non-negative");
  }
  if (!isFinite(_config.trajectory_max_age) || _config.trajectory_max_age <= 0.0) {
    return fail("trajectory_max_age must be positive");
  }
  if (!isFinite(_config.prefix_reuse_min_duration) || _config.prefix_reuse_min_duration < 0.0) {
    return fail("prefix_reuse_min_duration must be non-negative");
  }
  if (!isFinite(_config.state_jump_threshold) || _config.state_jump_threshold <= 0.0) {
    return fail("state_jump_threshold must be positive");
  }
  config_ = _config;
  configured_ = true;
  return true;
}

void ReplanManager::reset()
{
  has_committed_ = false;
  committed_versions_ = VersionSet();
  committed_trajectory_.clear();
  committed_stamp_ = 0.0;
  last_plan_stamp_ = 0.0;
  has_last_plan_ = false;
  last_planned_map_version_ = 0;
  has_planned_map_version_ = false;
}

void ReplanManager::onPlanCommitted(
  const VersionSet & _versions, const Trajectory2D & _trajectory, double _now_stamp)
{
  has_committed_ = true;
  committed_versions_ = _versions;
  committed_trajectory_ = _trajectory;
  committed_stamp_ = _now_stamp;
  last_plan_stamp_ = _now_stamp;
  has_last_plan_ = true;
  last_planned_map_version_ = _versions.map_version;
  has_planned_map_version_ = true;
}

ReplanDecision ReplanManager::evaluate(const ReplanRequest & _request) const
{
  ReplanDecision decision;
  if (!configured_) {
    decision.reason = "replan manager not configured";
    return decision;
  }

  // 1) 没有目标：作废，不规划。
  if (!_request.has_goal) {
    decision.type = ReplanType::kNone;
    decision.allowed = false;
    decision.reason = "no active goal; trajectory authorization revoked";
    return decision;
  }

  // 2) 没有路径：无法构造初值。
  if (!_request.has_path) {
    decision.type = ReplanType::kNone;
    decision.allowed = false;
    decision.reason = "no global path available";
    return decision;
  }

  // 3) 没有已提交轨迹：必须完整重规划。
  if (!has_committed_) {
    decision.type = ReplanType::kFull;
    decision.allowed = true;
    decision.reason = "no committed trajectory";
    return decision;
  }

  // 4) 版本变化：目标/路径/限速变化都必须完整重规划。
  if (committed_versions_.goal_epoch != _request.versions.goal_epoch) {
    decision.type = ReplanType::kFull;
    decision.allowed = true;
    decision.reason = "goal epoch changed";
    return decision;
  }
  if (committed_versions_.limits_version != _request.versions.limits_version) {
    decision.type = ReplanType::kFull;
    decision.allowed = true;
    decision.reason = "speed limits changed";
    return decision;
  }
  if (committed_versions_.path_version != _request.versions.path_version) {
    decision.type = ReplanType::kFull;
    decision.allowed = true;
    decision.reason = "path version changed";
    return decision;
  }

  // 5) 轨迹过期或复验失败：完整重规划（不能无条件续用旧轨迹）。
  const double trajectory_age =
    (_request.now_stamp > committed_stamp_) ? (_request.now_stamp - committed_stamp_) : 0.0;
  if (trajectory_age > config_.trajectory_max_age) {
    decision.type = ReplanType::kFull;
    decision.allowed = true;
    decision.reason = "committed trajectory is stale";
    return decision;
  }
  if (!_request.previous_trajectory_reusable) {
    decision.type = ReplanType::kFull;
    decision.allowed = true;
    decision.reason = "committed trajectory failed revalidation on the latest map";
    return decision;
  }

  // 6) 状态不可信：先不重规划，由控制层制动（避免用不可能的拼接状态继续）。
  if (!_request.projection_reliable) {
    decision.type = ReplanType::kNone;
    decision.allowed = false;
    decision.reason = "state/projection is not trustworthy; braking instead of replanning";
    return decision;
  }

  // 6.1) 状态与已提交轨迹的几何偏差过大：按定位跳变/严重跟踪误差处理，完整重规划。
  if (_request.has_state_position) {
    const double offset = distanceToTrajectory(committed_trajectory_, _request.state_position);
    if (offset > config_.state_jump_threshold) {
      decision.type = ReplanType::kFull;
      decision.allowed = true;
      decision.reason = "state is further from the committed trajectory than state_jump_threshold";
      return decision;
    }
  }

  // 7) 地图变化：按最小间隔做热启动重规划，避免 10 Hz 地图把规划线程压满。
  const bool map_changed =
    (!has_planned_map_version_ || (_request.versions.map_version >=
                                   last_planned_map_version_ + config_.map_version_replan_delta));
  const double since_last_plan = has_last_plan_ && (_request.now_stamp > last_plan_stamp_)
                                   ? (_request.now_stamp - last_plan_stamp_)
                                   : config_.replan_period;
  if (map_changed && since_last_plan >= config_.map_change_min_period) {
    decision.type = ReplanType::kHotStart;
    decision.allowed = true;
    decision.use_warm_start = true;
    decision.reason = "map version advanced; hot-start optimization";
    return decision;
  }

  // 8) 周期性重规划。
  if (since_last_plan >= config_.replan_period) {
    decision.type = ReplanType::kHotStart;
    decision.allowed = true;
    decision.use_warm_start = true;
    decision.reason = "periodic replan";
    return decision;
  }

  // 9) 若轨迹**剩余**（不是总时长）不足以覆盖“规划周期 + 停车”，提前触发完整重规划。
  const double remaining = std::max(
    0.0, committed_trajectory_.totalDuration() - std::max(0.0, _request.trajectory_progress));
  if (remaining < config_.replan_period + config_.prefix_reuse_min_duration) {
    decision.type = ReplanType::kFull;
    decision.allowed = true;
    decision.reason = "remaining trajectory is too short to bridge the next planning period";
    return decision;
  }

  decision.type = ReplanType::kNone;
  decision.allowed = false;
  decision.reason = "continue current trajectory";
  return decision;
}

bool ReplanManager::canCommit(
  const Trajectory2D & _candidate, const VersionSet & _versions, double _now_stamp,
  std::string * _reason) const
{
  const auto fail = [&_reason](const char * _message) {
    if (_reason != nullptr) {
      *_reason = _message;
    }
    return false;
  };

  if (!configured_) {
    return fail("replan manager not configured");
  }
  std::string sanity_reason;
  if (!_candidate.sanityCheck(&sanity_reason)) {
    return fail("candidate trajectory failed its sanity check");
  }
  if (!_versions.goal_epoch) {
    // goal_epoch == 0 表示没有有效会话，禁止提交（会话编号从 1 开始）。
    return fail("candidate has no valid goal session");
  }
  if (_candidate.versions != _versions) {
    return fail("candidate version set does not match the current request");
  }
  if (
    isFinite(_now_stamp) && _candidate.generated_stamp > 0.0 &&
    (_candidate.generated_stamp - _now_stamp) > 0.05) {
    return fail("candidate was generated in the future");
  }
  if (_candidate.valid_until > 0.0 && isFinite(_now_stamp) && _candidate.valid_until < _now_stamp) {
    return fail("candidate already expired");
  }
  if (has_committed_ && _candidate.versions.goal_epoch != committed_versions_.goal_epoch) {
    return fail("candidate belongs to a stale goal session");
  }
  return true;
}

}  // namespace srm27_minco_core
