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

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "srm27_minco_core/esdf_2d.hpp"
#include "srm27_minco_core/grid_snapshot.hpp"
#include "srm27_minco_core/recovery_decider.hpp"

namespace minco = srm27_minco_core;

namespace
{

/// \brief 测试地图：200x200，分辨率 0.05 m，左下角在原点，覆盖 [0,10]^2。
constexpr int kSizeX = 200;
constexpr int kSizeY = 200;
constexpr double kResolution = 0.05;

/// \brief 构造一张只有指定障碍块的地图。
/// \param _block 是否放置障碍，并给出矩形（格坐标，含端点）。
minco::Esdf2D MakeEsdf(
  bool _put_block, int _x0 = 0, int _y0 = 0, int _x1 = 0, int _y1 = 0)
{
  std::vector<minco::CellState> cells(
    static_cast<std::size_t>(kSizeX) * static_cast<std::size_t>(kSizeY), minco::CellState::kFree);
  if (_put_block) {
    for (int my = _y0; my <= _y1; ++my) {
      for (int mx = _x0; mx <= _x1; ++mx) {
        cells[static_cast<std::size_t>(my) * static_cast<std::size_t>(kSizeX) +
              static_cast<std::size_t>(mx)] = minco::CellState::kOccupied;
      }
    }
  }
  minco::GridSnapshot snapshot;
  EXPECT_TRUE(
    snapshot.reset(0.0, 0.0, kResolution, kSizeX, kSizeY, cells, /*map_version=*/3u, 10.0));
  minco::Esdf2D esdf;
  minco::Esdf2D::Config config;
  config.unknown_is_seed = false;
  EXPECT_TRUE(esdf.build(snapshot, config));
  return esdf;
}

/// \brief 默认参数下、门限为 3 的决策器。
minco::RecoveryDecider MakeDecider(int _fail_threshold = 3)
{
  minco::RecoveryDecider decider;
  minco::RecoveryConfig config;
  config.fail_threshold = _fail_threshold;
  decider.configure(config);
  return decider;
}

/// \brief 环形搜索用例里机器人的位置（开阔区正中心）。
const Eigen::Vector2d kOpenPosition(5.0, 5.0);

// -----------------------------------------------------------------------------
// 1) 脱困判定
// -----------------------------------------------------------------------------

TEST(RecoveryDecider, ConfigureClampsIllegalValues)
{
  minco::RecoveryDecider decider;
  minco::RecoveryConfig config;
  config.fail_threshold = 0;
  config.cooldown_sec = -5.0;
  config.window_sec = 0.0;
  config.escape_speed = -1.0;
  config.search_samples = 0;
  config.search_radius = -1.0;
  decider.configure(config);

  EXPECT_EQ(decider.config().fail_threshold, 1);
  EXPECT_DOUBLE_EQ(decider.config().cooldown_sec, 0.0);
  EXPECT_GT(decider.config().window_sec, 0.0);
  EXPECT_GT(decider.config().escape_speed, 0.0);
  EXPECT_GT(decider.config().search_samples, 0);
  EXPECT_GT(decider.config().search_radius, 0.0);
}

TEST(RecoveryDecider, EscapedClearanceIsForcedAboveStuckClearance)
{
  // 若允许两者相等，脱困会一进一出高频振荡；configure 必须强行拉开间隔。
  minco::RecoveryDecider decider;
  minco::RecoveryConfig config;
  config.stuck_clearance = 0.5;
  config.escaped_clearance = 0.5;
  decider.configure(config);
  EXPECT_GT(decider.config().escaped_clearance, decider.config().stuck_clearance);
}

TEST(RecoveryDecider, FailureCountGatesRecovery)
{
  const minco::Esdf2D esdf = MakeEsdf(false);
  minco::RecoveryDecider decider = MakeDecider(3);
  Eigen::Vector2d escape = Eigen::Vector2d::Zero();

  // 开阔地图里净空很大、梯度接近 0 -> 走环形搜索分支，方向一定可解。
  EXPECT_EQ(
    decider.decide(1.0, Eigen::Vector2d(5.0, 5.0), esdf, &escape),
    minco::RecoveryDecision::kNone);
  EXPECT_EQ(decider.consecutiveFailures(), 1);
  EXPECT_EQ(
    decider.decide(1.1, Eigen::Vector2d(5.0, 5.0), esdf, &escape),
    minco::RecoveryDecision::kNone);
  EXPECT_EQ(decider.consecutiveFailures(), 2);

  // 第 3 次跨过门限。
  EXPECT_EQ(
    decider.decide(1.2, Eigen::Vector2d(5.0, 5.0), esdf, &escape),
    minco::RecoveryDecision::kEscape);
  EXPECT_TRUE(decider.isRecovering());
  EXPECT_NEAR(escape.norm(), decider.config().escape_speed, 1e-9);
}

TEST(RecoveryDecider, InvalidTimeNeverTriggers)
{
  const minco::Esdf2D esdf = MakeEsdf(false);
  minco::RecoveryDecider decider = MakeDecider(1);
  Eigen::Vector2d escape = Eigen::Vector2d::Zero();

  EXPECT_EQ(
    decider.decide(std::numeric_limits<double>::quiet_NaN(), Eigen::Vector2d(5.0, 5.0), esdf,
      &escape),
    minco::RecoveryDecision::kNone);
  EXPECT_EQ(decider.consecutiveFailures(), 0);
  EXPECT_EQ(
    decider.decide(-1.0, Eigen::Vector2d(5.0, 5.0), esdf, &escape),
    minco::RecoveryDecision::kNone);
  EXPECT_EQ(decider.consecutiveFailures(), 0);
}

TEST(RecoveryDecider, PlanningSuccessClearsFailures)
{
  minco::RecoveryDecider decider = MakeDecider(3);
  EXPECT_FALSE(decider.onPlanningFailure(1.0));
  EXPECT_FALSE(decider.onPlanningFailure(1.1));
  EXPECT_EQ(decider.consecutiveFailures(), 2);
  decider.onPlanningSuccess();
  EXPECT_EQ(decider.consecutiveFailures(), 0);
  // 复位后需要重新累计到门限。
  EXPECT_FALSE(decider.onPlanningFailure(1.2));
  EXPECT_EQ(decider.consecutiveFailures(), 1);
}

TEST(RecoveryDecider, CooldownBlocksImmediateReentry)
{
  const minco::Esdf2D esdf = MakeEsdf(false);
  minco::RecoveryDecider decider = MakeDecider(1);
  Eigen::Vector2d escape = Eigen::Vector2d::Zero();

  EXPECT_EQ(
    decider.decide(10.0, Eigen::Vector2d(5.0, 5.0), esdf, &escape), minco::RecoveryDecision::kEscape);
  // 超时结束（净空仍远低于成功阈值），记录结束时刻。
  const double end_sec = 10.0 + decider.config().window_sec + 0.1;
  Eigen::Vector2d out = Eigen::Vector2d::Zero();
  EXPECT_EQ(decider.updateRecovery(end_sec, kOpenPosition, esdf, 0.30, &out), minco::RecoveryExit::kTimeout);
  EXPECT_FALSE(decider.isRecovering());

  // 冷却未到：即使计数立刻回来也不能再进。
  EXPECT_FALSE(decider.onPlanningFailure(end_sec + 0.01));
  EXPECT_FALSE(decider.shouldStartRecovery(end_sec + 0.01));
  // 冷却已过：允许再进。
  const double after_cooldown = end_sec + decider.config().cooldown_sec + 0.01;
  EXPECT_TRUE(decider.shouldStartRecovery(after_cooldown));
}

TEST(RecoveryDecider, NoEscapeDirectionDoesNotLockRecoveryActive)
{
  // 参考实现在"算不出方向"时会留下 recovery_active_ = true 且从不复位，
  // 导致之后 shouldTryRecovery 恒为 false（永久无法再脱困）。本实现必须自愈，
  // 并且失败计数要保持在门限之上，冷却一过就能立刻重试。
  minco::RecoveryDecider decider = MakeDecider(1);
  minco::Esdf2D empty;  // 未 build 的距离场
  Eigen::Vector2d escape = Eigen::Vector2d::Zero();

  EXPECT_EQ(
    decider.decide(1.0, Eigen::Vector2d(5.0, 5.0), empty, &escape),
    minco::RecoveryDecision::kNoEscapeDirection);
  EXPECT_FALSE(decider.isRecovering());
  EXPECT_EQ(decider.consecutiveFailures(), 1);
  // 冷却过后必须还能再来一次（不需要重新累计 fail_threshold 次失败）。
  EXPECT_TRUE(decider.shouldStartRecovery(1.0 + decider.config().cooldown_sec + 0.01));
}

TEST(RecoveryDecider, EsdfWithoutSeedsStillReportsFiniteClearance)
{
  // no_obstacle_distance 兜底值必须被当作"实时净空"正常使用：
  // 它在阈值以上，于是脱困会在下一个周期立刻判成功退出，而不是空转到超时。
  const minco::Esdf2D esdf = MakeEsdf(false);
  minco::RecoveryDecider decider = MakeDecider(1);
  Eigen::Vector2d escape = Eigen::Vector2d::Zero();
  ASSERT_EQ(
    decider.decide(1.0, Eigen::Vector2d(5.0, 5.0), esdf, &escape),
    minco::RecoveryDecision::kEscape);
  Eigen::Vector2d out = Eigen::Vector2d::Zero();
  EXPECT_EQ(
    decider.updateRecovery(1.01, kOpenPosition, esdf, esdf.distanceAt(5.0, 5.0), &out),
    minco::RecoveryExit::kEscaped);
}

// -----------------------------------------------------------------------------
// 2) 脱困方向规划
// -----------------------------------------------------------------------------

TEST(RecoveryDecider, EscapeDirectionFollowsAnalyticGradient)
{
  // 一堵竖墙放在 x ∈ [0, 2.5) 格（世界 x ∈ [0, 0.125)）。机器人放在墙右侧，
  // 梯度应指向 +x（远离墙）。
  const minco::Esdf2D esdf = MakeEsdf(true, 0, 0, 49, kSizeY - 1);
  minco::RecoveryConfig config;
  Eigen::Vector2d direction = Eigen::Vector2d::Zero();
  double clearance = 0.0;
  ASSERT_TRUE(
    minco::RecoveryDecider::planEscapeDirection(
      esdf, Eigen::Vector2d(0.5, 5.0), config, &direction, &clearance));
  EXPECT_GT(direction.x(), 0.9);
  EXPECT_NEAR(direction.norm(), 1.0, 1e-9);
}

TEST(RecoveryDecider, EscapeDirectionFallsBackToCircularSearchOnFlatGradient)
{
  // 空旷地图（无任何障碍种子）：解析梯度恒为 0 -> 必须走环形搜索并给出一个单位向量，
  // 且选取依据是 `no_obstacle_distance` 兜底值（有限、很大），不能判失败。
  const minco::Esdf2D esdf = MakeEsdf(false);
  minco::RecoveryConfig config;
  Eigen::Vector2d direction = Eigen::Vector2d::Zero();
  double clearance = 0.0;
  ASSERT_TRUE(
    minco::RecoveryDecider::planEscapeDirection(
      esdf, Eigen::Vector2d(5.0, 5.0), config, &direction, &clearance));
  EXPECT_NEAR(direction.norm(), 1.0, 1e-9);
  EXPECT_TRUE(std::isfinite(clearance));
  EXPECT_GT(clearance, config.escaped_clearance);
}

TEST(RecoveryDecider, CircularSearchPrefersTheOnlyOpenGap)
{
  // 机器人被三面墙围住，只在 +y 方向留一个缺口。环形搜索必须选到 +y。
  // 墙体构造成一个 U 形：左右两侧竖墙 + 下方横墙。
  std::vector<minco::CellState> cells(
    static_cast<std::size_t>(kSizeX) * static_cast<std::size_t>(kSizeY), minco::CellState::kFree);
  const auto set_block = [&cells](int _x0, int _y0, int _x1, int _y1) {
    for (int my = _y0; my <= _y1; ++my) {
      for (int mx = _x0; mx <= _x1; ++mx) {
        cells[static_cast<std::size_t>(my) * static_cast<std::size_t>(kSizeX) +
              static_cast<std::size_t>(mx)] = minco::CellState::kOccupied;
      }
    }
  };
  // 机器人位于格 (100,100) => 世界 (5.0, 5.0)。缺口留在 +y。
  set_block(70, 85, 70, 130);   // 左墙
  set_block(130, 85, 130, 130); // 右墙
  set_block(70, 85, 130, 85);   // 下墙
  minco::GridSnapshot snapshot;
  ASSERT_TRUE(
    snapshot.reset(0.0, 0.0, kResolution, kSizeX, kSizeY, cells, /*map_version=*/5u, 20.0));
  minco::Esdf2D esdf;
  minco::Esdf2D::Config esdf_config;
  esdf_config.unknown_is_seed = false;
  ASSERT_TRUE(esdf.build(snapshot, esdf_config));

  minco::RecoveryConfig config;
  config.search_radius = 0.4;  // 8 格，仍落在口袋内部
  Eigen::Vector2d direction = Eigen::Vector2d::Zero();
  double clearance = 0.0;
  ASSERT_TRUE(
    minco::RecoveryDecider::planEscapeDirection(
      esdf, Eigen::Vector2d(5.0, 5.0), config, &direction, &clearance));
  EXPECT_GT(direction.y(), 0.9) << "环形搜索没有选到唯一的开口方向";
  EXPECT_NEAR(direction.norm(), 1.0, 1e-9);
}

TEST(RecoveryDecider, EscapeDirectionFailsOutsideMap)
{
  const minco::Esdf2D esdf = MakeEsdf(false);
  minco::RecoveryConfig config;
  Eigen::Vector2d direction = Eigen::Vector2d::Zero();
  double clearance = 0.0;
  // 完全在地图之外 -> 查询无效 -> 不得给出方向（否则会朝未知方向盲冲）。
  EXPECT_FALSE(
    minco::RecoveryDecider::planEscapeDirection(
      esdf, Eigen::Vector2d(-100.0, -100.0), config, &direction, &clearance));
}

TEST(RecoveryDecider, EscapeDirectionRejectsInvalidEsdf)
{
  minco::Esdf2D empty;
  minco::RecoveryConfig config;
  Eigen::Vector2d direction = Eigen::Vector2d::Zero();
  double clearance = 0.0;
  EXPECT_FALSE(
    minco::RecoveryDecider::planEscapeDirection(
      empty, Eigen::Vector2d(5.0, 5.0), config, &direction, &clearance));
}

TEST(RecoveryDecider, GradientBranchIgnoresSingleSampleOfZeroGradientAtWall)
{
  // 紧贴墙面（墙右侧一格）时仍需给出远离墙的方向，而不是退化到随机环形方向。
  const minco::Esdf2D esdf = MakeEsdf(true, 0, 0, 49, kSizeY - 1);
  minco::RecoveryConfig config;
  Eigen::Vector2d direction = Eigen::Vector2d::Zero();
  double clearance = 0.0;
  ASSERT_TRUE(
    minco::RecoveryDecider::planEscapeDirection(
      esdf, Eigen::Vector2d(0.14, 5.0), config, &direction, &clearance));
  EXPECT_GT(direction.x(), 0.5);
}

// -----------------------------------------------------------------------------
// 3) 脱困成功判定
// -----------------------------------------------------------------------------

TEST(RecoveryDecider, UpdateRecoveryReportsEscapedWhenClearanceRecovers)
{
  const minco::Esdf2D esdf = MakeEsdf(false);
  minco::RecoveryDecider decider = MakeDecider(1);
  Eigen::Vector2d escape = Eigen::Vector2d::Zero();
  ASSERT_EQ(
    decider.decide(100.0, Eigen::Vector2d(5.0, 5.0), esdf, &escape),
    minco::RecoveryDecision::kEscape);

  Eigen::Vector2d out = Eigen::Vector2d::Zero();
  // 净空仍在阈值以下：继续脱困，速度被回填。
  EXPECT_EQ(decider.updateRecovery(100.01, kOpenPosition, esdf, 0.30, &out), minco::RecoveryExit::kNone);
  EXPECT_NEAR(out.norm(), decider.config().escape_speed, 1e-9);

  // 净空恢复：判成功，速度清零。
  out = Eigen::Vector2d(1.0, 1.0);
  EXPECT_EQ(decider.updateRecovery(100.02, kOpenPosition, esdf, 0.45, &out), minco::RecoveryExit::kEscaped);
  EXPECT_FALSE(decider.isRecovering());
  EXPECT_NEAR(out.norm(), 0.0, 1e-12);
  EXPECT_EQ(decider.consecutiveFailures(), 0);
}

TEST(RecoveryDecider, UpdateRecoveryTimesOutInsideWindowBoundary)
{
  const minco::Esdf2D esdf = MakeEsdf(false);
  minco::RecoveryDecider decider = MakeDecider(1);
  Eigen::Vector2d escape = Eigen::Vector2d::Zero();
  ASSERT_EQ(
    decider.decide(200.0, Eigen::Vector2d(5.0, 5.0), esdf, &escape),
    minco::RecoveryDecision::kEscape);
  const double window = decider.config().window_sec;

  Eigen::Vector2d out = Eigen::Vector2d::Zero();
  // 恰好在窗口边界上仍算在窗口内。
  EXPECT_EQ(decider.updateRecovery(200.0 + window, kOpenPosition, esdf, 0.30, &out), minco::RecoveryExit::kNone);
  // 超出窗口 -> 超时。
  EXPECT_EQ(
    decider.updateRecovery(200.0 + window + 1e-6, kOpenPosition, esdf, 0.30, &out), minco::RecoveryExit::kTimeout);
  EXPECT_FALSE(decider.isRecovering());
}

TEST(RecoveryDecider, NonFiniteClearanceNeverCountsAsSuccess)
{
  // NaN（TF 失败 / 查询失败）不得判成功；只按时间推进。
  const minco::Esdf2D esdf = MakeEsdf(false);
  minco::RecoveryDecider decider = MakeDecider(1);
  Eigen::Vector2d escape = Eigen::Vector2d::Zero();
  ASSERT_EQ(
    decider.decide(300.0, Eigen::Vector2d(5.0, 5.0), esdf, &escape),
    minco::RecoveryDecision::kEscape);
  Eigen::Vector2d out = Eigen::Vector2d::Zero();
  EXPECT_EQ(
    decider.updateRecovery(300.01, kOpenPosition, esdf, std::numeric_limits<double>::quiet_NaN(), &out),
    minco::RecoveryExit::kNone);
  EXPECT_TRUE(decider.isRecovering());
}

TEST(RecoveryDecider, UpdateRecoveryRecomputesDirectionFromCurrentPosition)
{
  // 回归测试：方向必须随车移动而更新，不能沿用进入脱困时的那一个。
  //
  // 场景：一堵竖墙在世界 x ∈ [4.75, 5.25)（格 95..104）。机器人先在墙右侧，
  // 脱困方向应为 +x（远离墙）；同一个脱困会话中把它移到墙左侧，方向必须翻成 -x。
  // 若实现把方向冻结在进入时刻，本用例会失败 —— 现场表现就是"车被推着贴死墙面"。
  const minco::Esdf2D esdf = MakeEsdf(true, 95, 0, 104, kSizeY - 1);
  minco::RecoveryDecider decider = MakeDecider(1);
  Eigen::Vector2d escape = Eigen::Vector2d::Zero();

  ASSERT_EQ(
    decider.decide(10.0, Eigen::Vector2d(5.6, 5.0), esdf, &escape),
    minco::RecoveryDecision::kEscape);
  ASSERT_GT(escape.x(), 0.0);

  // 车被推到墙左侧：下一个周期方向必须翻成 -x。
  Eigen::Vector2d out = Eigen::Vector2d::Zero();
  ASSERT_EQ(
    decider.updateRecovery(10.01, Eigen::Vector2d(4.4, 5.0), esdf, 0.30, &out),
    minco::RecoveryExit::kNone);
  EXPECT_LT(out.x(), 0.0) << "脱困方向被冻结在进入时刻，没有按当前位置重算";
  EXPECT_NEAR(out.norm(), decider.config().escape_speed, 1e-9);
}

TEST(RecoveryDecider, UpdateRecoveryWithoutEsdfKeepsPreviousDirection)
{
  // 距离场暂时不可用时不得输出零速把车停在原地，也不得判成功；窗口照常计时。
  const minco::Esdf2D esdf = MakeEsdf(true, 95, 0, 104, kSizeY - 1);
  minco::RecoveryDecider decider = MakeDecider(1);
  Eigen::Vector2d escape = Eigen::Vector2d::Zero();
  ASSERT_EQ(
    decider.decide(10.0, Eigen::Vector2d(5.6, 5.0), esdf, &escape),
    minco::RecoveryDecision::kEscape);
  ASSERT_GT(escape.norm(), 0.0);

  minco::Esdf2D empty;
  Eigen::Vector2d out = Eigen::Vector2d::Zero();
  EXPECT_EQ(
    decider.updateRecovery(10.01, Eigen::Vector2d(5.6, 5.0), empty, 0.30, &out),
    minco::RecoveryExit::kNone);
  EXPECT_GT(out.norm(), 0.0) << "距离场不可用时不应该把逃逸速度清零";
  EXPECT_TRUE(decider.isRecovering());
}

// -----------------------------------------------------------------------------
// 2b) 脱困路径（可视化用的绿线）
// -----------------------------------------------------------------------------

TEST(RecoveryDecider, TraceEscapePathMovesAwayFromWall)
{
  // 竖墙在世界 x ∈ [4.75, 5.25)。起点在墙右侧：整条路径的 x 必须单调不减、
  // 且总体朝 +x 走。
  const minco::Esdf2D esdf = MakeEsdf(true, 95, 0, 104, kSizeY - 1);
  minco::RecoveryConfig config;
  std::vector<Eigen::Vector2d> path;
  const Eigen::Vector2d start(5.5, 5.0);
  ASSERT_TRUE(minco::RecoveryDecider::traceEscapePath(esdf, start, config, 0.1, 64, &path));
  ASSERT_GE(path.size(), 2u);
  EXPECT_NEAR(path.front().x(), start.x(), 1e-12);
  EXPECT_NEAR(path.front().y(), start.y(), 1e-12);
  EXPECT_GT(path.back().x(), start.x()) << "路径没有远离墙";

  double previous_x = path.front().x();
  for (const Eigen::Vector2d & point : path) {
    EXPECT_GE(point.x(), previous_x - 1e-9) << "路径出现回退";
    previous_x = point.x();
  }
}

TEST(RecoveryDecider, TraceEscapePathStopsOnceOutOfTheStuckZone)
{
  const minco::Esdf2D esdf = MakeEsdf(true, 95, 0, 104, kSizeY - 1);
  minco::RecoveryConfig config;
  std::vector<Eigen::Vector2d> path;
  ASSERT_TRUE(
    minco::RecoveryDecider::traceEscapePath(
      esdf, Eigen::Vector2d(5.5, 5.0), config, 0.1, 64, &path));
  // 末点净空必须已经超过成功阈值：否则可视化会在卡死区里画出一条悬空的线，
  // 现场会误读成"脱困路径无效"。
  EXPECT_GT(esdf.distanceAt(path.back().x(), path.back().y()), config.escaped_clearance);
}

TEST(RecoveryDecider, TraceEscapePathRespectsStepBudget)
{
  // 步数上限必须被严格遵守，否则可视化会在每个控制周期画出任意长的线。
  const minco::Esdf2D esdf = MakeEsdf(false);
  minco::RecoveryConfig config;
  std::vector<Eigen::Vector2d> path;
  minco::RecoveryDecider::traceEscapePath(esdf, kOpenPosition, config, 0.1, 5, &path);
  EXPECT_LE(path.size(), 6u);  // 起点 + 最多 5 步
}

TEST(RecoveryDecider, TraceEscapePathWithInvalidEsdfYieldsNoPath)
{
  minco::Esdf2D empty;
  minco::RecoveryConfig config;
  std::vector<Eigen::Vector2d> path;
  EXPECT_FALSE(
    minco::RecoveryDecider::traceEscapePath(empty, kOpenPosition, config, 0.1, 10, &path));
  EXPECT_TRUE(path.empty());
}

TEST(RecoveryDecider, TraceEscapePathRejectsNullOutput)
{
  const minco::Esdf2D esdf = MakeEsdf(false);
  minco::RecoveryConfig config;
  EXPECT_FALSE(
    minco::RecoveryDecider::traceEscapePath(esdf, kOpenPosition, config, 0.1, 10, nullptr));
}

TEST(RecoveryDecider, ResetClearsEverything)
{
  const minco::Esdf2D esdf = MakeEsdf(false);
  minco::RecoveryDecider decider = MakeDecider(1);
  Eigen::Vector2d escape = Eigen::Vector2d::Zero();
  ASSERT_EQ(
    decider.decide(1.0, Eigen::Vector2d(5.0, 5.0), esdf, &escape),
    minco::RecoveryDecision::kEscape);
  decider.reset();
  EXPECT_FALSE(decider.isRecovering());
  EXPECT_EQ(decider.consecutiveFailures(), 0);
  EXPECT_LT(decider.lastRecoveryStartSec(), 0.0);
  EXPECT_LT(decider.lastRecoveryEndSec(), 0.0);
}

}  // namespace
