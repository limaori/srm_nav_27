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

#include <cmath>
#include <limits>

#include "srm27_minco_controller/terminal_stop.hpp"

/// \file
/// \brief 终点急停的行为测试（实车终点振荡回归，2026-10-09）。
///
/// 现场现象：车以较快速度进入目标容差时判不了到点，控制器继续追精确末点，
/// 冲过末点再反向修正 —— 终点附近来回走。
///
/// 这里固化三条契约：
///  * **进入只看位置**：以接近满速进容差也必须接管；
///  * **接管即停**：没有任何"慢慢收速度"的后段逻辑，控制器不再输出朝末点或背向末点的速度；
///  * **滞回与坏样本**：不会在阈值附近抖动，也不会因一次坏样本就解锁。
namespace
{

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

srm27_minco_controller::TerminalStop makeStop(
  double _tolerance, double _exit_margin = 0.15, double _fallback = 0.20)
{
  srm27_minco_controller::TerminalStop stop;
  srm27_minco_controller::TerminalStop::Config config;
  config.enabled = true;
  config.tolerance = _tolerance;
  config.fallback_tolerance = _fallback;
  config.exit_margin = _exit_margin;
  stop.configure(config);
  return stop;
}

}  // namespace

/// \brief 核心回归：以满速进入容差也必须立即接管。
TEST(TerminalStopTest, TakesOverImmediatelyAtFullSpeed)
{
  auto stop = makeStop(0.0);
  // 目标检查器容差 0.40，距末点 0.40（刚进圈）。速度不参与判定。
  EXPECT_TRUE(stop.update(0.40, 0.40));
  EXPECT_DOUBLE_EQ(stop.entryTolerance(), 0.40);
  EXPECT_DOUBLE_EQ(stop.exitTolerance(), 0.55);
}

/// \brief 进入时机只看位置：不因为"还很快"而推迟。
TEST(TerminalStopTest, PositionOnlyEntry)
{
  auto stop = makeStop(0.0);
  // 距离 0.399 在容差内 —— 无论速度多少都接管（本类不接收速度参数）。
  EXPECT_TRUE(stop.update(0.399, 0.40));
  auto another = makeStop(0.0);
  EXPECT_FALSE(another.update(0.401, 0.40));
}

/// \brief 滞回：略超出容差仍保持，超过 容差+滞回 才解除。
TEST(TerminalStopTest, HysteresisKeepsStop)
{
  auto stop = makeStop(0.0);
  ASSERT_TRUE(stop.update(0.30, 0.40));
  EXPECT_TRUE(stop.update(0.45, 0.40));
  EXPECT_TRUE(stop.update(0.54, 0.40));
  EXPECT_FALSE(stop.update(0.60, 0.40));
}

/// \brief 进入阈值优先用目标检查器的 xy 容差；读不到时退回兜底值。
TEST(TerminalStopTest, ToleranceSourcePriority)
{
  // 显式 0.15：即使目标检查器给 0.40，也只在 0.15 内接管。
  auto explicit_tol = makeStop(0.15);
  EXPECT_FALSE(explicit_tol.update(0.20, 0.40));
  EXPECT_TRUE(explicit_tol.update(0.14, 0.40));
  EXPECT_DOUBLE_EQ(explicit_tol.entryTolerance(), 0.15);

  // 不给显式值：用目标检查器容差。
  auto from_checker = makeStop(0.0);
  EXPECT_FALSE(from_checker.update(0.30, 0.15));
  EXPECT_TRUE(from_checker.update(0.10, 0.15));

  // 容差不可用（0 / NaN）：退回兜底 0.20，不能退化成永远不接管或永远接管。
  for (const double unavailable : {0.0, kNaN}) {
    auto fallback = makeStop(0.0);
    EXPECT_FALSE(fallback.update(0.30, unavailable));
    EXPECT_TRUE(fallback.update(0.15, unavailable));
    EXPECT_DOUBLE_EQ(fallback.entryTolerance(), 0.20);
  }
}

/// \brief 坏样本保持现状：不主动进入，也不解除已有急停。
TEST(TerminalStopTest, BadSampleKeepsState)
{
  auto stop = makeStop(0.0);
  EXPECT_FALSE(stop.update(kNaN, 0.40));
  EXPECT_FALSE(stop.active());
  ASSERT_TRUE(stop.update(0.30, 0.40));
  EXPECT_TRUE(stop.update(kNaN, 0.40));
  EXPECT_TRUE(stop.update(std::numeric_limits<double>::infinity(), 0.40));
  EXPECT_TRUE(stop.active());
}

/// \brief 关闭开关后永不接管。
TEST(TerminalStopTest, DisabledNeverTakesOver)
{
  srm27_minco_controller::TerminalStop stop;
  srm27_minco_controller::TerminalStop::Config config;
  config.enabled = false;
  stop.configure(config);
  EXPECT_FALSE(stop.update(0.0, 0.40));
}

/// \brief 重置清空状态（换目标 / 定位重置 / 失活时调用）。
TEST(TerminalStopTest, ResetClearsState)
{
  auto stop = makeStop(0.0);
  ASSERT_TRUE(stop.update(0.10, 0.40));
  stop.reset();
  EXPECT_FALSE(stop.active());
  EXPECT_DOUBLE_EQ(stop.entryTolerance(), 0.0);
}

/// \brief 非法配置被就地纠正，不会退化成"永远不接管"或"永远接管"。
TEST(TerminalStopTest, SanitizesInvalidConfig)
{
  srm27_minco_controller::TerminalStop stop;
  srm27_minco_controller::TerminalStop::Config config;
  config.enabled = true;
  config.tolerance = -1.0;    // -> 0（表示用目标检查器容差）
  config.exit_margin = -0.5;  // -> 0（无滞回）
  config.fallback_tolerance = 0.20;
  stop.configure(config);
  EXPECT_FALSE(stop.update(0.30, 0.0));  // 容差不可用 -> 兜底 0.20
  EXPECT_TRUE(stop.update(0.15, 0.0));
  EXPECT_DOUBLE_EQ(stop.exitTolerance(), 0.20);  // 滞回被纠正为 0
}

/// \brief 急刹之后能不能停在容差内，取决于**底盘自己**的减速能力。
///
/// 车在距末点 `tol` 处被给零，之后滑行 `v²/(2·a_chassis)`。容差对末点是对称的，
/// 所以停车点落在末点 ±tol 内的条件就是 `v²/(2·a_chassis) <= 2·tol`。
/// 这条算术就是预检脚本那条阻塞级检查的依据，这里把它固化成用例。
TEST(TerminalStopTest, StoppingDistanceDecidesWhetherItStaysInsideTolerance)
{
  const double tolerance = 0.40;
  const double entry_speed = 1.5;  // 实车 limits.max_linear_speed
  // 边界解: v²/(2a) <= 2·tol  =>  a >= v²/(4·tol) = 1.40625 m/s²
  const double boundary = entry_speed * entry_speed / (4.0 * tolerance);
  EXPECT_NEAR(boundary, 1.40625, 1.0e-9);

  struct Case
  {
    double chassis_deceleration;
    bool expect_inside;
  };
  const Case cases[] = {
    {3.0, true}, {2.0, true}, {1.5, true}, {1.45, true}, {1.40, false}, {1.0, false}, {0.5, false},
  };
  for (const Case & item : cases) {
    const double slide = entry_speed * entry_speed / (2.0 * item.chassis_deceleration);
    const double final_distance = std::abs(slide - tolerance);
    EXPECT_EQ(final_distance <= tolerance, item.expect_inside)
      << "a=" << item.chassis_deceleration << " 滑行 " << slide << " m -> 距末点 " << final_distance
      << " m";
  }
}

/// \brief 想减小急刹冲击就收紧进入阈值：进入越晚，速度越低。
///
/// 这条用例说明 `terminal.tolerance` 才是"急刹有多猛"的旋钮，而不是重新引入减速曲线。
TEST(TerminalStopTest, TighterToleranceMeansLowerEntrySpeed)
{
  const double a_plan = 3.0;  // 轨迹停车剖面用的减速度
  const double v_max = 1.5;   // limits.max_linear_speed
  const auto entry_speed = [a_plan, v_max](double _tolerance) {
    return std::min(v_max, std::sqrt(2.0 * a_plan * _tolerance));
  };
  EXPECT_NEAR(entry_speed(0.40), 1.5, 1.0e-9);     // 用目标检查器容差 -> 满速急刹
  EXPECT_NEAR(entry_speed(0.15), 0.9487, 1.0e-3);  // 收紧到 0.15 m -> 约 0.95 m/s
  EXPECT_NEAR(entry_speed(0.05), 0.5477, 1.0e-3);  // 0.05 m -> 约 0.55 m/s
}
