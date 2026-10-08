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

#include <chrono>
#include <memory>
#include <thread>
#include <vector>

#include "srm27_minco_controller/planning_worker.hpp"
#include "srm27_minco_core/grid_snapshot.hpp"

namespace controller = srm27_minco_controller;
namespace core = srm27_minco_core;

namespace
{
controller::PlanningRequest request(double _end_x)
{
  controller::PlanningRequest result;
  core::GridSnapshot grid;
  EXPECT_TRUE(grid.reset(
    -6, -6, 0.05, 240, 240, std::vector<core::CellState>(240 * 240, core::CellState::kFree), 1,
    100.0));
  auto esdf = std::make_shared<core::Esdf2D>();
  EXPECT_TRUE(esdf->build(grid));
  result.esdf = esdf;
  result.path = {Eigen::Vector2d(0, 0), Eigen::Vector2d(_end_x, 0)};
  result.state.valid = true;
  result.state.sample_stamp = 100.0;
  result.limits.max_linear_speed = 1.5;
  result.limits.max_linear_accel = 3.0;
  result.local_path_horizon = 3.0;
  result.request_stamp = 100.0;
  result.initializer_config.terminal_speed = 0.8;
  result.minco_config.w_jerk = 0.2;
  result.minco_config.w_time = 10.0;
  result.minco_config.w_velocity = 60.0;
  result.minco_config.w_acceleration = 300.0;
  result.validator_config.braking_deceleration = 3.0;
  result.validator_config.required_prefix_duration = 0.6;
  return result;
}

controller::PlanningResult runPlan(const controller::PlanningRequest & _request)
{
  controller::PlanningWorker worker;
  EXPECT_TRUE(worker.start());
  EXPECT_TRUE(worker.submit(_request));
  controller::PlanningResult result;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  bool ready = false;
  while (!(ready = worker.takeResult(result)) && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  EXPECT_TRUE(ready);
  worker.stop();
  return result;
}
}  // namespace

TEST(PlanningWorkerTest, HorizonProducesValidatedMovingEndpoint)
{
  const auto result = runPlan(request(5.0));
  ASSERT_TRUE(result.success) << result.status << ": " << result.reason;
  EXPECT_EQ(result.terminal_reason, "planning_horizon");
  EXPECT_FALSE(result.trajectory.terminal_is_global_goal);
  EXPECT_FALSE(result.trajectory.terminal_requires_stop);
  EXPECT_NEAR(result.trajectory.endVelocity().norm(), 0.8, 1.0e-6);
  EXPECT_NEAR(result.trajectory.endPosition().x(), 3.0, 1.0e-6);
  EXPECT_FALSE(result.local_path.empty());
  EXPECT_LE(result.max_speed, 1.5 + 1.0e-6);
}

TEST(PlanningWorkerTest, FinalGoalRetainsStopBoundary)
{
  const auto result = runPlan(request(1.0));
  ASSERT_TRUE(result.success) << result.status << ": " << result.reason;
  EXPECT_EQ(result.terminal_reason, "global_goal");
  EXPECT_TRUE(result.trajectory.terminal_is_global_goal);
  EXPECT_TRUE(result.trajectory.terminal_requires_stop);
  EXPECT_LT(result.trajectory.endVelocity().norm(), 1.0e-6);
}

TEST(PlanningWorkerTest, MapClippingStopsWithoutClaimingGoal)
{
  auto input = request(10.0);
  input.local_path_horizon = 10.0;
  const auto result = runPlan(input);
  ASSERT_TRUE(result.success) << result.status << ": " << result.reason;
  EXPECT_EQ(result.terminal_reason, "map_boundary");
  EXPECT_FALSE(result.trajectory.terminal_is_global_goal);
  EXPECT_TRUE(result.trajectory.terminal_requires_stop);
  EXPECT_LT(result.trajectory.endVelocity().norm(), 1.0e-6);
  EXPECT_LT(result.trajectory.endPosition().x(), 6.0 - 0.38);
}

TEST(PlanningWorkerTest, EmptyPathFailsWithoutDereferencingEnd)
{
  auto input = request(1.0);
  input.path.clear();
  const auto result = runPlan(input);
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.status, "no_path");
}

TEST(PlanningWorkerTest, ShortGoalTrajectoryCanHoldAtRestForMpc)
{
  const auto result = runPlan(request(0.02));
  ASSERT_TRUE(result.success) << result.status << ": " << result.reason;
  EXPECT_TRUE(result.trajectory.terminal_requires_stop);
  EXPECT_TRUE(result.validation.coverage_ok);
  EXPECT_LT(result.trajectory.totalDuration(), 0.6);
  EXPECT_GE(result.validation.effective_prefix_duration, 0.6);
}
