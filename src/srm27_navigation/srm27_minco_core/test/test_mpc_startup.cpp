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
#include <memory>
#include <string>
#include <vector>

#include "srm27_minco_core/esdf_2d.hpp"
#include "srm27_minco_core/grid_snapshot.hpp"
#include "srm27_minco_core/minco_optimizer.hpp"
#include "srm27_minco_core/mpc_solver.hpp"
#include "srm27_minco_core/tracking_reference.hpp"
#include "srm27_minco_core/trajectory_initializer.hpp"

/// \file
/// \brief 起步回归：速度层 MPC + MINCO 参考必须真的能把车开动。
///
/// 现场故障：机器人收到目标后完全不动（实测 commanded vx 在 ±0.01 m/s 之间来回变号，
/// 车原地缓慢漂移），minco_trajectory 本身是完全正常的（最大速度 1.43 m/s）。
///
/// 根因（已离线定位）：
///  1. 速度层 MPC 的决策量**就是速度**，而参考按当前进度采样时，轨迹起步段的
///     `u_ref(0) = 0`（MINCO 轨迹从静止起步）；
///  2. 该目标下 QP 的最优解把加速“后置”——手工重算目标函数值：`U*=0.0023`、
///     `U=Uref=0.0077`、`U=0=6.32`，即“先不动”确实是数学上的最优；
///  3. 于是第 0 拍命令 ≈ 0，车不动 → 投影进度永远停在 0 → 参考永远是“从静止起步”，
///     形成自锁。
///
/// 修复：`mpc.command_lookahead > 0`，取预测序列里 `t_now + 前瞻` 那一拍作为输出速度
/// （方案 §7.4 的“预测速度输出”做法）。本文件把它固化成闭环回归。

namespace
{

using namespace srm27_minco_core;

/// \brief 最简航向提供者：`xy_only` 语义（航向不变、角速度参考恒为 0）。
class XyOnlyYawProvider : public YawReferenceProvider
{
public:
  bool yawReference(
    double, const Eigen::Vector2d &, const Eigen::Vector2d &, bool, const Eigen::Vector2d &,
    double _previous_yaw, double & _yaw, double & _omega) override
  {
    _yaw = _previous_yaw;
    _omega = 0.0;
    return true;
  }
};

/// \brief 造一条与仿真同性质的局部轨迹：2 m 直线、静止起步、起停。
Trajectory2D makeLocalTrajectory(const Limits2D & _limits)
{
  const int size = 240;
  std::vector<CellState> cells(static_cast<std::size_t>(size) * size, CellState::kFree);
  GridSnapshot grid;
  grid.reset(-5.0, -5.0, 0.05, size, size, std::move(cells), 1, 100.0);
  Esdf2D esdf;
  esdf.build(grid);
  auto esdf_ptr = std::make_shared<Esdf2D>(esdf);

  std::vector<Eigen::Vector2d> path;
  for (int i = 0; i <= 40; ++i) {
    path.emplace_back(0.05 * static_cast<double>(i), 0.0);
  }

  TrajectoryInitializer::Config initializer_config;
  initializer_config.max_speed = _limits.max_linear_speed;
  initializer_config.max_accel = _limits.max_linear_accel;
  initializer_config.max_brake = _limits.max_linear_accel;
  initializer_config.resample_step = 0.10;
  initializer_config.nominal_piece_duration = 0.30;
  initializer_config.terminal_is_global_goal = false;

  TrajectoryInitialGuess guess;
  std::string reason;
  if (!TrajectoryInitializer::initialize(
        path, Eigen::Vector2d::Zero(), initializer_config, guess, &reason)) {
    return Trajectory2D();
  }

  MincoOptimizerConfig minco_config;
  minco_config.w_jerk = 0.2;
  minco_config.w_time = 10.0;
  minco_config.w_obstacle = 100.0;
  minco_config.w_velocity = 60.0;
  minco_config.w_acceleration = 300.0;
  minco_config.two_stage = true;
  minco_config.lbfgs_past = 3;
  minco_config.robot_radius = 0.33;
  minco_config.clearance_margin = 0.05;

  MincoOptimizer optimizer;
  if (!optimizer.configure(minco_config, &reason)) {
    return Trajectory2D();
  }
  optimizer.setEsdf(esdf_ptr);

  MincoOptimizeResult result;
  if (!optimizer.optimize(guess, _limits, result) || result.status != SolveStatus::kSuccess) {
    return Trajectory2D();
  }
  Trajectory2D trajectory = result.trajectory;
  trajectory.terminal_is_global_goal = false;
  trajectory.generated_stamp = 100.0;
  trajectory.valid_after = 100.0;
  trajectory.valid_until = 101.0;
  return trajectory;
}

/// \brief 闭环仿真：50 个控制周期（50 Hz，共 1.0 s），机器人按返回的指令积分。
///
/// 与插件一致：参考按投影进度构造，命令用 `commandAt(lookahead)` 取预测序列中的一拍。
/// \return 成功执行返回 true。
bool runClosedLoop(
  const Trajectory2D & _trajectory, double _lookahead, double & _advance, double & _last_speed)
{
  Limits2D limits;
  limits.max_linear_speed = 1.5;
  limits.max_linear_accel = 3.0;
  limits.max_angular_speed = 0.0;
  limits.max_angular_accel = 0.0;

  TrackingReferenceConfig reference_config;
  reference_config.projection_backward_window = 0.30;
  reference_config.projection_forward_window = 1.00;
  reference_config.max_progress_step = 0.60;
  reference_config.backtrack_limit = 0.05;
  reference_config.goal_slowdown_time = 0.40;
  reference_config.two_pass_reference = false;

  TrackingReferenceBuilder builder;
  std::string reason;
  if (!builder.configure(reference_config, &reason)) {
    return false;
  }
  XyOnlyYawProvider yaw;

  MpcModel::Config model_config;
  model_config.type = MpcModelType::kVelocityIntegrator;
  model_config.prediction_steps = 30;
  model_config.prediction_dt = 0.02;

  MpcSolverConfig solver_config;
  solver_config.q_position = 20.0;
  solver_config.q_yaw = 5.0;
  solver_config.r_translation = 1.0;
  solver_config.r_angular = 1.0;
  solver_config.max_linear_accel = limits.max_linear_accel;
  solver_config.max_angular_accel = 1.0;
  solver_config.use_hot_start = true;

  MpcSolver solver;
  if (
    !solver.configure(model_config, solver_config, &reason) || !solver.setLimits(limits, &reason)) {
    return false;
  }
  solver.warmUp();

  double x = 0.0;
  double speed = 0.0;
  double previous = 0.0;
  bool has_previous = false;

  for (int cycle = 0; cycle < 50; ++cycle) {
    State2D state;
    state.valid = true;
    state.x = x;
    state.y = 0.0;
    state.yaw = 0.0;
    state.velocity = Eigen::Vector2d(speed, 0.0);
    state.omega = 0.0;
    state.sample_stamp = 100.0 + 0.02 * static_cast<double>(cycle);
    state.received_stamp = state.sample_stamp;

    TrackingReferenceResult reference;
    if (!builder.build(_trajectory, state, solver.model(), yaw, reference)) {
      return false;
    }

    MpcInitialState initial_state;
    initial_state.z0 = Eigen::VectorXd::Zero(3);
    initial_state.z0 << x, 0.0, 0.0;
    initial_state.has_previous_input = has_previous;
    initial_state.previous_applied_input = Eigen::VectorXd::Zero(3);
    initial_state.previous_applied_input(0) = previous;
    initial_state.control_interval = 0.02;

    MpcReference mpc_reference;
    mpc_reference.z_ref = reference.z_ref;
    mpc_reference.u_ref = reference.u_ref;
    mpc_reference.tangent_angle = reference.tangent_angle;

    MpcSolution solution;
    if (!solver.solve(initial_state, mpc_reference, solution)) {
      return false;
    }
    Eigen::VectorXd command;
    if (!solver.commandAt(solution, _lookahead, command)) {
      return false;
    }
    previous = command(0);
    has_previous = true;
    speed = previous;
    x += speed * 0.02;  // 一阶：指令即速度（仿真无内环延迟）
  }

  _advance = x;
  _last_speed = speed;
  return true;
}

}  // namespace

/// \brief 正向前瞻下必须真的起步（本次故障的回归）。
TEST(MpcStartupTest, PositiveLookaheadStartsFromRest)
{
  Limits2D limits;
  limits.max_linear_speed = 1.5;
  limits.max_linear_accel = 3.0;
  const Trajectory2D trajectory = makeLocalTrajectory(limits);
  ASSERT_FALSE(trajectory.empty()) << "局部轨迹构造失败，无法执行起步回归";

  double advance = 0.0;
  double speed = 0.0;
  ASSERT_TRUE(runClosedLoop(trajectory, 0.20, advance, speed));
  EXPECT_GT(advance, 0.15) << "1.0 s 内应至少前进 0.15 m；实测 " << advance;
  EXPECT_GT(speed, 0.10) << "1.0 s 末速度应为正；实测 " << speed;
}

/// \brief 前瞻为 0 时，速度层 MPC 在“轨迹从静止起步”的场景下会自锁。
///
/// 这条用例**固定当时定位到的失败模式**：`u_ref(0)=0` 使 QP 的最优解把加速后置，
/// 第 0 拍命令≈0，车不动 → 投影进度停在 0 → 参考永远是“从静止起步”。
/// 若将来换成六维加速度模型或按计划时钟推进进度后此用例不再成立，
/// 应当连同 `mpc.command_lookahead` 的配置说明一起更新，而不是直接删掉。
TEST(MpcStartupTest, ZeroLookaheadStallsFromRest)
{
  Limits2D limits;
  limits.max_linear_speed = 1.5;
  limits.max_linear_accel = 3.0;
  const Trajectory2D trajectory = makeLocalTrajectory(limits);
  ASSERT_FALSE(trajectory.empty());

  double advance = 0.0;
  double speed = 0.0;
  ASSERT_TRUE(runClosedLoop(trajectory, 0.0, advance, speed));
  EXPECT_LT(advance, 0.05) << "前瞻为 0 时预期几乎不动；实测 " << advance;
  EXPECT_LT(std::abs(speed), 0.05) << "前瞻为 0 时末速度应接近 0；实测 " << speed;
}

/// \brief `commandAt` 在前瞻恰为整数拍时返回对应的预测输入。
TEST(MpcStartupTest, CommandAtReturnsTheRequestedPredictionStep)
{
  Limits2D limits;
  limits.max_linear_speed = 1.5;
  limits.max_linear_accel = 3.0;
  const Trajectory2D trajectory = makeLocalTrajectory(limits);
  ASSERT_FALSE(trajectory.empty());

  TrackingReferenceConfig reference_config;
  reference_config.two_pass_reference = false;
  TrackingReferenceBuilder builder;
  std::string reason;
  ASSERT_TRUE(builder.configure(reference_config, &reason));

  MpcModel::Config model_config;
  model_config.type = MpcModelType::kVelocityIntegrator;
  model_config.prediction_steps = 30;
  model_config.prediction_dt = 0.02;
  MpcSolverConfig solver_config;
  solver_config.max_linear_accel = 3.0;
  solver_config.use_hot_start = false;
  MpcSolver solver;
  ASSERT_TRUE(solver.configure(model_config, solver_config, &reason));
  ASSERT_TRUE(solver.setLimits(limits, &reason));

  XyOnlyYawProvider yaw;
  State2D state;
  state.valid = true;
  state.x = 0.0;
  state.y = 0.0;
  state.yaw = 0.0;
  state.velocity = Eigen::Vector2d::Zero();
  state.omega = 0.0;
  state.sample_stamp = 100.0;
  state.received_stamp = 100.0;
  TrackingReferenceResult reference;
  ASSERT_TRUE(builder.build(trajectory, state, solver.model(), yaw, reference));

  MpcInitialState initial_state;
  initial_state.z0 = Eigen::VectorXd::Zero(3);
  initial_state.has_previous_input = false;
  initial_state.control_interval = 0.02;
  MpcReference mpc_reference;
  mpc_reference.z_ref = reference.z_ref;
  mpc_reference.u_ref = reference.u_ref;

  MpcSolution solution;
  ASSERT_TRUE(solver.solve(initial_state, mpc_reference, solution));

  Eigen::VectorXd at_zero;
  Eigen::VectorXd at_five;
  ASSERT_TRUE(solver.commandAt(solution, 0.0, at_zero));
  ASSERT_TRUE(solver.commandAt(solution, 0.10, at_five));
  for (int axis = 0; axis < 3; ++axis) {
    EXPECT_NEAR(at_zero(axis), solution.u(axis, 0), 1e-12);
    EXPECT_NEAR(at_five(axis), solution.u(axis, 5), 1e-12);
  }

  // 落在两拍之间时必须是线性插值，而不是四舍五入到某一拍。
  Eigen::VectorXd between;
  ASSERT_TRUE(solver.commandAt(solution, 0.11, between));
  for (int axis = 0; axis < 3; ++axis) {
    EXPECT_NEAR(between(axis), 0.5 * (solution.u(axis, 5) + solution.u(axis, 6)), 1e-12);
  }
}
