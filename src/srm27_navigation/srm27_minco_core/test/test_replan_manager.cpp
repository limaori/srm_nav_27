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

/// \file
/// \brief 重规划触发与提交策略的单元测试（方案 §6.7 重规划、§4.4 版本与作废）。
///
/// 覆盖点：
///  * 无目标/无路径/无已提交轨迹的决策；
///  * `goal_epoch`、`path_version`、`limits_version` 变化必须完整重规划；
///  * 轨迹过期与“旧轨迹复验失败”不能被无条件容错；
///  * 状态不可信时先制动（`kNone` 且不允许规划）；
///  * 地图版本前进要受最小间隔约束，周期到点用热启动；
///  * `canCommit` 的版本、会话、时间窗与 `sanityCheck` 检查。

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <cstdint>
#include <string>

#include "srm27_minco_core/replan_manager.hpp"
#include "srm27_minco_core/trajectory_2d.hpp"
#include "srm27_minco_core/types.hpp"

namespace
{

/// \brief 提交时刻（ROS 秒），用于构造所有时间断言。
const double kCommitStamp = 100.0;

/// \brief 小于最小重规划间隔的时间推进量（s）。
const double kSmallDelta = 0.05;

/// \brief 与产品默认一致的配置。
srm27_minco_core::ReplanManager::Config replanConfig()
{
  srm27_minco_core::ReplanManager::Config config;
  config.replan_period = 0.10;
  config.map_change_min_period = 0.10;
  config.trajectory_max_age = 0.30;
  config.prefix_reuse_min_duration = 0.20;
  config.state_jump_threshold = 0.50;
  config.map_version_replan_delta = 1;
  return config;
}

/// \brief 构造版本集合。
srm27_minco_core::VersionSet makeVersions(
  const std::uint64_t _goal_epoch, const std::uint64_t _path_version,
  const std::uint64_t _map_version, const std::uint64_t _limits_version)
{
  srm27_minco_core::VersionSet versions;
  versions.goal_epoch = _goal_epoch;
  versions.path_version = _path_version;
  versions.map_version = _map_version;
  versions.limits_version = _limits_version;
  return versions;
}

/// \brief 构造满足 `sanityCheck` 的直线轨迹（单段，段内常速，时间元数据有限）。
///
/// `sanityCheck` 要求：`generated_stamp`/`valid_after`/`valid_until` 有限、
/// `valid_until >= valid_after`、各段时长 > 0、段间位置/速度/加速度连续。
srm27_minco_core::Trajectory2D makeCommittableTrajectory(
  const srm27_minco_core::VersionSet & _versions, const double _duration = 2.0,
  const double _stamp = kCommitStamp)
{
  srm27_minco_core::Trajectory2D trajectory;
  srm27_minco_core::Trajectory2D::Coefficients cx{};
  srm27_minco_core::Trajectory2D::Coefficients cy{};
  cx[1] = 0.5;  // 常速 0.5 m/s 沿 +x
  EXPECT_TRUE(trajectory.addPiece(cx, cy, _duration));
  trajectory.versions = _versions;
  trajectory.generated_stamp = _stamp;
  trajectory.valid_after = _stamp;
  trajectory.valid_until = _stamp + 10.0;
  return trajectory;
}

/// \brief 构造“一切正常”的请求，按需覆盖单个字段。
srm27_minco_core::ReplanRequest makeRequest(
  const srm27_minco_core::VersionSet & _versions, const double _now_stamp)
{
  srm27_minco_core::ReplanRequest request;
  request.versions = _versions;
  request.state_stamp = _now_stamp;
  request.now_stamp = _now_stamp;
  request.has_goal = true;
  request.has_path = true;
  request.previous_trajectory_reusable = true;
  request.projection_reliable = true;
  return request;
}

}  // namespace

// ---------------------------------------------------------------------------
// 配置与基本决策
// ---------------------------------------------------------------------------

TEST(ReplanManagerTest, Configure_RejectsInvalidParameters)
{
  srm27_minco_core::ReplanManager manager;
  std::string reason;

  srm27_minco_core::ReplanManager::Config config = replanConfig();
  config.replan_period = 0.0;
  EXPECT_FALSE(manager.configure(config, &reason));
  EXPECT_FALSE(reason.empty());

  config = replanConfig();
  config.replan_period = -0.1;
  EXPECT_FALSE(manager.configure(config));

  config = replanConfig();
  config.map_change_min_period = -0.01;
  EXPECT_FALSE(manager.configure(config));

  config = replanConfig();
  config.trajectory_max_age = 0.0;
  EXPECT_FALSE(manager.configure(config));

  config = replanConfig();
  config.prefix_reuse_min_duration = -0.5;
  EXPECT_FALSE(manager.configure(config));

  config = replanConfig();
  config.state_jump_threshold = 0.0;
  EXPECT_FALSE(manager.configure(config));

  EXPECT_FALSE(manager.hasCommittedTrajectory());
}

TEST(ReplanManagerTest, Evaluate_NotConfiguredIsNone)
{
  srm27_minco_core::ReplanManager manager;
  const srm27_minco_core::VersionSet versions = makeVersions(7, 3, 5, 1);
  const srm27_minco_core::ReplanDecision decision =
    manager.evaluate(makeRequest(versions, kCommitStamp));

  EXPECT_EQ(decision.type, srm27_minco_core::ReplanType::kNone);
  EXPECT_FALSE(decision.allowed);
  EXPECT_FALSE(decision.reason.empty());
}

TEST(ReplanManagerTest, Evaluate_WithoutGoalOrPathIsNoneAndNotAllowed)
{
  srm27_minco_core::ReplanManager manager;
  ASSERT_TRUE(manager.configure(replanConfig()));
  const srm27_minco_core::VersionSet versions = makeVersions(7, 3, 5, 1);

  // 无目标：授权作废，不规划。
  srm27_minco_core::ReplanRequest no_goal = makeRequest(versions, kCommitStamp);
  no_goal.has_goal = false;
  const srm27_minco_core::ReplanDecision no_goal_decision = manager.evaluate(no_goal);
  EXPECT_EQ(no_goal_decision.type, srm27_minco_core::ReplanType::kNone);
  EXPECT_FALSE(no_goal_decision.allowed);
  EXPECT_FALSE(no_goal_decision.use_warm_start);
  EXPECT_STREQ(srm27_minco_core::toString(no_goal_decision.type), "none");

  // 无局部路径：无法构造初值。
  srm27_minco_core::ReplanRequest no_path = makeRequest(versions, kCommitStamp);
  no_path.has_path = false;
  const srm27_minco_core::ReplanDecision no_path_decision = manager.evaluate(no_path);
  EXPECT_EQ(no_path_decision.type, srm27_minco_core::ReplanType::kNone);
  EXPECT_FALSE(no_path_decision.allowed);
  EXPECT_FALSE(no_path_decision.reason.empty());
}

TEST(ReplanManagerTest, Evaluate_WithoutCommittedTrajectoryIsFull)
{
  srm27_minco_core::ReplanManager manager;
  ASSERT_TRUE(manager.configure(replanConfig()));
  const srm27_minco_core::VersionSet versions = makeVersions(7, 3, 5, 1);

  // 刚 reset（或从未提交）时必须完整重规划。
  manager.reset();
  EXPECT_FALSE(manager.hasCommittedTrajectory());
  const srm27_minco_core::ReplanDecision decision =
    manager.evaluate(makeRequest(versions, kCommitStamp));
  EXPECT_EQ(decision.type, srm27_minco_core::ReplanType::kFull);
  EXPECT_TRUE(decision.allowed);
  EXPECT_FALSE(decision.use_warm_start);
  EXPECT_STREQ(srm27_minco_core::toString(decision.type), "full");
}

TEST(ReplanManagerTest, Evaluate_UnchangedVersionsContinueExecution)
{
  srm27_minco_core::ReplanManager manager;
  ASSERT_TRUE(manager.configure(replanConfig()));
  const srm27_minco_core::VersionSet versions = makeVersions(7, 3, 5, 1);
  const srm27_minco_core::Trajectory2D trajectory = makeCommittableTrajectory(versions);

  manager.onPlanCommitted(versions, trajectory, kCommitStamp);
  EXPECT_TRUE(manager.hasCommittedTrajectory());
  EXPECT_EQ(manager.committedVersions().goal_epoch, versions.goal_epoch);
  EXPECT_EQ(manager.committedVersions().map_version, versions.map_version);
  EXPECT_DOUBLE_EQ(manager.committedStamp(), kCommitStamp);

  // 版本一致、旧轨迹复验通过、距上次规划很近 → 继续执行，不允许新规划。
  const srm27_minco_core::ReplanDecision decision =
    manager.evaluate(makeRequest(versions, kCommitStamp + kSmallDelta));
  EXPECT_EQ(decision.type, srm27_minco_core::ReplanType::kNone);
  EXPECT_FALSE(decision.allowed);
  EXPECT_FALSE(decision.reason.empty());
}

TEST(ReplanManagerTest, Evaluate_VersionChangesForceFullReplan)
{
  srm27_minco_core::ReplanManager manager;
  ASSERT_TRUE(manager.configure(replanConfig()));
  const srm27_minco_core::VersionSet versions = makeVersions(7, 3, 5, 1);
  manager.onPlanCommitted(versions, makeCommittableTrajectory(versions), kCommitStamp);

  // 目标会话、路径、限速任一变化都必须完整重规划。
  srm27_minco_core::VersionSet changed = versions;
  changed.goal_epoch += 1;
  EXPECT_EQ(
    manager.evaluate(makeRequest(changed, kCommitStamp + kSmallDelta)).type,
    srm27_minco_core::ReplanType::kFull);

  changed = versions;
  changed.path_version += 1;
  EXPECT_EQ(
    manager.evaluate(makeRequest(changed, kCommitStamp + kSmallDelta)).type,
    srm27_minco_core::ReplanType::kFull);

  changed = versions;
  changed.limits_version += 1;
  EXPECT_EQ(
    manager.evaluate(makeRequest(changed, kCommitStamp + kSmallDelta)).type,
    srm27_minco_core::ReplanType::kFull);

  // 只有地图版本前进不属于“版本作废”，走热启动路径（见地图用例）。
  changed = versions;
  changed.map_version += 1;
  EXPECT_EQ(
    manager.evaluate(makeRequest(changed, kCommitStamp + kSmallDelta)).type,
    srm27_minco_core::ReplanType::kNone);
}

TEST(ReplanManagerTest, Evaluate_StaleTrajectoryIsFull)
{
  srm27_minco_core::ReplanManager manager;
  ASSERT_TRUE(manager.configure(replanConfig()));
  const srm27_minco_core::VersionSet versions = makeVersions(7, 3, 5, 1);
  manager.onPlanCommitted(versions, makeCommittableTrajectory(versions), kCommitStamp);

  // 提交后 0.4 s > trajectory_max_age(0.3) → 必须完整重规划。
  const srm27_minco_core::ReplanDecision decision =
    manager.evaluate(makeRequest(versions, kCommitStamp + 0.40));
  EXPECT_EQ(decision.type, srm27_minco_core::ReplanType::kFull);
  EXPECT_TRUE(decision.allowed);
}

TEST(ReplanManagerTest, Evaluate_NonReusableTrajectoryIsFull)
{
  srm27_minco_core::ReplanManager manager;
  ASSERT_TRUE(manager.configure(replanConfig()));
  const srm27_minco_core::VersionSet versions = makeVersions(7, 3, 5, 1);
  manager.onPlanCommitted(versions, makeCommittableTrajectory(versions), kCommitStamp);

  // 最新地图复验未通过：保留旧轨迹不是无条件容错策略。
  srm27_minco_core::ReplanRequest request = makeRequest(versions, kCommitStamp + kSmallDelta);
  request.previous_trajectory_reusable = false;
  const srm27_minco_core::ReplanDecision decision = manager.evaluate(request);
  EXPECT_EQ(decision.type, srm27_minco_core::ReplanType::kFull);
  EXPECT_TRUE(decision.allowed);
  EXPECT_FALSE(decision.reason.empty());
}

TEST(ReplanManagerTest, Evaluate_UnreliableProjectionIsNoneAndNotAllowed)
{
  srm27_minco_core::ReplanManager manager;
  ASSERT_TRUE(manager.configure(replanConfig()));
  const srm27_minco_core::VersionSet versions = makeVersions(7, 3, 5, 1);
  manager.onPlanCommitted(versions, makeCommittableTrajectory(versions), kCommitStamp);

  // 定位/投影不可信：先制动，不用不可信的拼接状态去规划。
  srm27_minco_core::ReplanRequest request = makeRequest(versions, kCommitStamp + kSmallDelta);
  request.projection_reliable = false;
  const srm27_minco_core::ReplanDecision decision = manager.evaluate(request);
  EXPECT_EQ(decision.type, srm27_minco_core::ReplanType::kNone);
  EXPECT_FALSE(decision.allowed);
  EXPECT_FALSE(decision.use_warm_start);
  EXPECT_FALSE(decision.reason.empty());
}

TEST(ReplanManagerTest, Evaluate_MapChangeTriggersHotStartAfterMinPeriod)
{
  srm27_minco_core::ReplanManager manager;
  ASSERT_TRUE(manager.configure(replanConfig()));
  const srm27_minco_core::VersionSet versions = makeVersions(7, 3, 5, 1);
  manager.onPlanCommitted(versions, makeCommittableTrajectory(versions), kCommitStamp);

  // 地图版本前进且已过 map_change_min_period(0.1 s) → 热启动优化。
  srm27_minco_core::VersionSet advanced = versions;
  advanced.map_version += 1;
  const srm27_minco_core::ReplanDecision decision =
    manager.evaluate(makeRequest(advanced, kCommitStamp + 0.20));
  EXPECT_EQ(decision.type, srm27_minco_core::ReplanType::kHotStart);
  EXPECT_TRUE(decision.allowed);
  EXPECT_TRUE(decision.use_warm_start);
  EXPECT_FALSE(decision.reuse_prefix);
  EXPECT_STREQ(srm27_minco_core::toString(decision.type), "hot_start");
}

TEST(ReplanManagerTest, Evaluate_MapChangeTooSoonDoesNotTrigger)
{
  srm27_minco_core::ReplanManager manager;
  ASSERT_TRUE(manager.configure(replanConfig()));
  const srm27_minco_core::VersionSet versions = makeVersions(7, 3, 5, 1);
  manager.onPlanCommitted(versions, makeCommittableTrajectory(versions), kCommitStamp);

  // 地图版本刚前进但距上次规划太近：不能重规划，避免 10 Hz 地图压满规划线程。
  srm27_minco_core::VersionSet advanced = versions;
  advanced.map_version += 1;
  const srm27_minco_core::ReplanDecision decision =
    manager.evaluate(makeRequest(advanced, kCommitStamp + kSmallDelta));
  EXPECT_EQ(decision.type, srm27_minco_core::ReplanType::kNone);
  EXPECT_FALSE(decision.allowed);
  EXPECT_FALSE(decision.use_warm_start);
}

TEST(ReplanManagerTest, Evaluate_PeriodicReplanTriggersHotStart)
{
  srm27_minco_core::ReplanManager manager;
  ASSERT_TRUE(manager.configure(replanConfig()));
  const srm27_minco_core::VersionSet versions = makeVersions(7, 3, 5, 1);
  manager.onPlanCommitted(versions, makeCommittableTrajectory(versions), kCommitStamp);

  // 地图没变但到达 replan_period(0.1 s) → 周期性热启动重规划。
  const srm27_minco_core::ReplanDecision decision =
    manager.evaluate(makeRequest(versions, kCommitStamp + 0.15));
  EXPECT_EQ(decision.type, srm27_minco_core::ReplanType::kHotStart);
  EXPECT_TRUE(decision.allowed);
  EXPECT_TRUE(decision.use_warm_start);
}

TEST(ReplanManagerTest, Evaluate_ShortTrajectoryForcesFullReplan)
{
  srm27_minco_core::ReplanManager manager;
  ASSERT_TRUE(manager.configure(replanConfig()));
  const srm27_minco_core::VersionSet versions = makeVersions(7, 3, 5, 1);
  // 总时长 0.25 s < replan_period + prefix_reuse_min_duration(0.3 s)：
  // 剩余轨迹不足以覆盖“规划周期 + 停车”。
  manager.onPlanCommitted(versions, makeCommittableTrajectory(versions, 0.25), kCommitStamp);

  const srm27_minco_core::ReplanDecision decision =
    manager.evaluate(makeRequest(versions, kCommitStamp + kSmallDelta));
  EXPECT_EQ(decision.type, srm27_minco_core::ReplanType::kFull);
  EXPECT_TRUE(decision.allowed);
}

TEST(ReplanManagerTest, Evaluate_ResetClearsCommittedTrajectory)
{
  srm27_minco_core::ReplanManager manager;
  ASSERT_TRUE(manager.configure(replanConfig()));
  const srm27_minco_core::VersionSet versions = makeVersions(7, 3, 5, 1);
  manager.onPlanCommitted(versions, makeCommittableTrajectory(versions), kCommitStamp);
  ASSERT_TRUE(manager.hasCommittedTrajectory());

  manager.reset();
  EXPECT_FALSE(manager.hasCommittedTrajectory());
  EXPECT_EQ(manager.committedVersions().goal_epoch, 0U);
  EXPECT_DOUBLE_EQ(manager.committedStamp(), 0.0);

  const srm27_minco_core::ReplanDecision decision =
    manager.evaluate(makeRequest(versions, kCommitStamp + kSmallDelta));
  EXPECT_EQ(decision.type, srm27_minco_core::ReplanType::kFull);
  EXPECT_TRUE(decision.allowed);
}

// ---------------------------------------------------------------------------
// 提交检查
// ---------------------------------------------------------------------------

TEST(ReplanManagerTest, CanCommit_RejectsVersionMismatchAndInvalidSession)
{
  srm27_minco_core::ReplanManager manager;
  std::string reason;
  const srm27_minco_core::VersionSet versions = makeVersions(7, 3, 5, 1);
  const srm27_minco_core::Trajectory2D candidate = makeCommittableTrajectory(versions);

  // 未配置时不允许提交。
  EXPECT_FALSE(manager.canCommit(candidate, versions, kCommitStamp, &reason));
  EXPECT_FALSE(reason.empty());

  ASSERT_TRUE(manager.configure(replanConfig()));
  reason.clear();
  EXPECT_TRUE(manager.canCommit(candidate, versions, kCommitStamp, &reason)) << reason;

  // 候选自身版本与请求版本不一致。
  srm27_minco_core::Trajectory2D stale = candidate;
  stale.versions.path_version += 1;
  reason.clear();
  EXPECT_FALSE(manager.canCommit(stale, versions, kCommitStamp, &reason));
  EXPECT_FALSE(reason.empty());

  // 请求版本集合与候选声明不一致。
  srm27_minco_core::VersionSet mismatched = versions;
  mismatched.limits_version += 1;
  reason.clear();
  EXPECT_FALSE(manager.canCommit(candidate, mismatched, kCommitStamp, &reason));
  EXPECT_FALSE(reason.empty());

  // goal_epoch == 0 表示没有有效会话，禁止提交。
  const srm27_minco_core::VersionSet no_session = makeVersions(0, 0, 0, 0);
  const srm27_minco_core::Trajectory2D no_session_candidate = makeCommittableTrajectory(no_session);
  reason.clear();
  EXPECT_FALSE(manager.canCommit(no_session_candidate, no_session, kCommitStamp, &reason));
  EXPECT_FALSE(reason.empty());

  // 已经提交了 goal_epoch = 7 的轨迹后，属于旧会话的候选不得替换它。
  manager.onPlanCommitted(versions, candidate, kCommitStamp);
  const srm27_minco_core::VersionSet next_session = makeVersions(8, 3, 5, 1);
  const srm27_minco_core::Trajectory2D next_candidate = makeCommittableTrajectory(next_session);
  reason.clear();
  EXPECT_FALSE(manager.canCommit(next_candidate, next_session, kCommitStamp, &reason));
  EXPECT_FALSE(reason.empty());
}

TEST(ReplanManagerTest, CanCommit_RejectsInsaneExpiredOrFutureCandidate)
{
  srm27_minco_core::ReplanManager manager;
  ASSERT_TRUE(manager.configure(replanConfig()));
  const srm27_minco_core::VersionSet versions = makeVersions(7, 3, 5, 1);
  const srm27_minco_core::Trajectory2D candidate = makeCommittableTrajectory(versions);
  std::string reason;

  // 空轨迹过不了 sanityCheck。
  const srm27_minco_core::Trajectory2D empty;
  reason.clear();
  EXPECT_FALSE(manager.canCommit(empty, versions, kCommitStamp, &reason));
  EXPECT_FALSE(reason.empty());

  // 段间位置跳变（不连续）同样过不了。
  srm27_minco_core::Trajectory2D discontinuous;
  srm27_minco_core::Trajectory2D::Coefficients cx{};
  srm27_minco_core::Trajectory2D::Coefficients cy{};
  cx[1] = 0.5;
  EXPECT_TRUE(discontinuous.addPiece(cx, cy, 1.0));
  cx[0] = 5.0;  // 下一段起点跳到 5 m 处
  EXPECT_TRUE(discontinuous.addPiece(cx, cy, 1.0));
  discontinuous.versions = versions;
  discontinuous.generated_stamp = kCommitStamp;
  discontinuous.valid_after = kCommitStamp;
  discontinuous.valid_until = kCommitStamp + 10.0;
  reason.clear();
  EXPECT_FALSE(manager.canCommit(discontinuous, versions, kCommitStamp, &reason));
  EXPECT_FALSE(reason.empty());

  // 已经过期。
  srm27_minco_core::Trajectory2D expired = candidate;
  expired.valid_after = kCommitStamp - 10.0;
  expired.valid_until = kCommitStamp - 1.0;
  reason.clear();
  EXPECT_FALSE(manager.canCommit(expired, versions, kCommitStamp, &reason));
  EXPECT_FALSE(reason.empty());

  // 生成时刻明显在未来。
  srm27_minco_core::Trajectory2D future = candidate;
  future.generated_stamp = kCommitStamp + 0.50;
  reason.clear();
  EXPECT_FALSE(manager.canCommit(future, versions, kCommitStamp, &reason));
  EXPECT_FALSE(reason.empty());

  // 合法候选仍然可以提交。
  reason.clear();
  EXPECT_TRUE(manager.canCommit(candidate, versions, kCommitStamp, &reason)) << reason;
}
