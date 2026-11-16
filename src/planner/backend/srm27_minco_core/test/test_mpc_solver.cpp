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
/// \brief MPC 预测模型与凝聚 QP 求解器的单元测试（方案 §11.1「MPC 模型」「QP」两行）。
///
/// 覆盖点：
///  * 速度层/六维加速度层模型的离散矩阵、逐步预测与解析解；
///  * 凝聚矩阵 `Sx`、`Su` 的展平顺序与 `predict()` 一致；
///  * 无约束二次型解析解、常速度参考可精确跟踪；
///  * 合速度多边形约束（斜向也不能突破 v_max）、首步差分用真实控制间隔、角速度界；
///  * 平面各向异性权重矩阵的交叉项符号（方案 §3.5 修正项 2）；
///  * `commandAt` 的线性插值、horizon 裁剪与不可用返回。

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <random>
#include <string>

#include "srm27_minco_core/mpc_model.hpp"
#include "srm27_minco_core/mpc_solver.hpp"
#include "srm27_minco_core/types.hpp"

namespace
{

/// \brief 圆周率常量，避免依赖未标准化的 M_PI 宏。
const double kPi = std::acos(-1.0);

/// \brief 速度层模型配置：决策量就是 odom 系速度，是 P2 基线（方案 §7.1）。
srm27_minco_core::MpcModel::Config velocityModelConfig(const int _steps, const double _dt = 0.02)
{
  srm27_minco_core::MpcModel::Config config;
  config.type = srm27_minco_core::MpcModelType::kVelocityIntegrator;
  config.prediction_steps = _steps;
  config.prediction_dt = _dt;
  return config;
}

/// \brief 六维加速度层模型配置：决策量是加速度，用于报告对照。
srm27_minco_core::MpcModel::Config accelerationModelConfig(
  const int _steps, const double _dt = 0.02)
{
  srm27_minco_core::MpcModel::Config config;
  config.type = srm27_minco_core::MpcModelType::kAccelerationDoubleIntegrator;
  config.prediction_steps = _steps;
  config.prediction_dt = _dt;
  return config;
}

/// \brief 单位权重配置：Q = R = I，且关闭输入差分不等式（用于解析解对照）。
srm27_minco_core::MpcSolverConfig unitWeightConfig()
{
  srm27_minco_core::MpcSolverConfig config;
  config.q_position = 1.0;
  config.q_yaw = 1.0;
  config.q_velocity = 1.0;
  config.q_omega = 1.0;
  config.r_translation = 1.0;
  config.r_angular = 1.0;
  config.r_input_change = 0.0;
  config.enforce_input_change = false;
  return config;
}

/// \brief 与产品默认一致的求解器配置（差分约束打开，多边形 8 边）。
srm27_minco_core::MpcSolverConfig productSolverConfig()
{
  srm27_minco_core::MpcSolverConfig config;
  config.speed_polygon_sides = 8;
  config.enforce_input_change = true;
  config.max_linear_accel = 0.3;
  config.max_angular_accel = 0.5;
  config.r_input_change = 0.0;
  return config;
}

/// \brief 构造“参考本身可被精确达到”的常输入参考（仅适用于速度层模型）。
///
/// 速度层模型下 `Z_i = z0 + h (i+1) u_ref`（`Z_i` 对应预测的第 i+1 列），
/// 因此把 `u_ref` 按该式积分得到 `z_ref` 时，代价函数在 `U = u_ref` 处为 0。
srm27_minco_core::MpcReference constantReference(
  const srm27_minco_core::MpcModel & _model, const Eigen::Vector3d & _z0,
  const Eigen::Vector3d & _u_ref)
{
  const int steps = _model.steps();
  const double h = _model.dt();
  srm27_minco_core::MpcReference reference;
  reference.z_ref = Eigen::MatrixXd::Zero(3, steps);
  reference.u_ref = Eigen::MatrixXd::Zero(3, steps);
  for (int i = 0; i < steps; ++i) {
    reference.z_ref.col(i) = _z0 + static_cast<double>(i + 1) * h * _u_ref;
    reference.u_ref.col(i) = _u_ref;
  }
  return reference;
}

/// \brief 零参考（`z_ref = 0`、`u_ref = 0`）。
srm27_minco_core::MpcReference zeroReference(
  const int _state_dim, const int _input_dim, const int _steps)
{
  srm27_minco_core::MpcReference reference;
  reference.z_ref = Eigen::MatrixXd::Zero(_state_dim, _steps);
  reference.u_ref = Eigen::MatrixXd::Zero(_input_dim, _steps);
  return reference;
}

/// \brief 打印实测值，便于在测试日志中直接核对报告数据。
void printMeasurement(const std::string & _name, const double _value)
{
  std::cout << "[MEASURE] " << _name << " = " << _value << std::endl;
}

}  // namespace

// ---------------------------------------------------------------------------
// 模型：离散矩阵、逐步预测、参数校验、凝聚矩阵一致性
// ---------------------------------------------------------------------------

TEST(MpcModelTest, VelocityIntegrator_DimensionsAndDiscreteMatrices)
{
  srm27_minco_core::MpcModel model;
  ASSERT_TRUE(model.configure(velocityModelConfig(5)));
  EXPECT_TRUE(model.configured());
  EXPECT_EQ(model.type(), srm27_minco_core::MpcModelType::kVelocityIntegrator);

  const double h = 0.02;
  EXPECT_EQ(model.stateDim(), 3);
  EXPECT_EQ(model.inputDim(), 3);
  EXPECT_EQ(model.steps(), 5);
  EXPECT_DOUBLE_EQ(model.dt(), h);
  EXPECT_EQ(model.decisionDim(), 3 * 5);

  // 速度层模型 `z = [x, y, yaw]`，`z(k+1) = z(k) + h u(k)`。
  EXPECT_TRUE(model.a().isApprox(Eigen::MatrixXd::Identity(3, 3), 1e-15));
  EXPECT_TRUE(model.b().isApprox(h * Eigen::MatrixXd::Identity(3, 3), 1e-15));
  EXPECT_EQ(model.velocityIndex(), -1);
  EXPECT_EQ(model.omegaIndex(), -1);
  EXPECT_EQ(model.positionIndex(), 0);
  EXPECT_EQ(model.yawIndex(), 2);

  // 凝聚矩阵形状：`Sx` 为 `(nz*N, nz)`，`Su` 为 `(nz*N, nu*N)`。
  EXPECT_EQ(model.sx().rows(), 3 * 5);
  EXPECT_EQ(model.sx().cols(), 3);
  EXPECT_EQ(model.su().rows(), 3 * 5);
  EXPECT_EQ(model.su().cols(), 3 * 5);
}

TEST(MpcModelTest, VelocityIntegrator_PureVxOnlyAdvancesX)
{
  srm27_minco_core::MpcModel model;
  ASSERT_TRUE(model.configure(velocityModelConfig(5)));
  const double h = model.dt();

  const Eigen::VectorXd z0 = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd u = Eigen::VectorXd::Zero(model.decisionDim());
  u(0) = 0.5;  // 只有第 0 步的 vx 非零

  const Eigen::MatrixXd prediction = model.predict(z0, u);
  ASSERT_EQ(prediction.rows(), 3);
  ASSERT_EQ(prediction.cols(), 6);

  // x 在第 1 步跃升一次后保持不变（只有第 0 步有输入，因此是弱单调增）。
  for (int k = 0; k <= 5; ++k) {
    const double expected_x = (k >= 1) ? 0.5 * h : 0.0;
    EXPECT_NEAR(prediction(0, k), expected_x, 1e-15) << "k = " << k;
    EXPECT_NEAR(prediction(1, k), 0.0, 1e-15) << "k = " << k;
    EXPECT_NEAR(prediction(2, k), 0.0, 1e-15) << "k = " << k;
  }
  for (int k = 0; k < 5; ++k) {
    EXPECT_GE(prediction(0, k + 1), prediction(0, k) - 1e-15) << "k = " << k;
  }
  EXPECT_NEAR(prediction(0, 5), 0.5 * h * 1.0, 1e-15);
}

TEST(MpcModelTest, VelocityIntegrator_PureVyOnlyAdvancesY)
{
  srm27_minco_core::MpcModel model;
  ASSERT_TRUE(model.configure(velocityModelConfig(5)));
  const double h = model.dt();

  const Eigen::VectorXd z0 = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd u = Eigen::VectorXd::Zero(model.decisionDim());
  u(1) = 0.4;  // 只有第 0 步的 vy 非零

  const Eigen::MatrixXd prediction = model.predict(z0, u);
  ASSERT_EQ(prediction.cols(), 6);
  for (int k = 0; k <= 5; ++k) {
    const double expected_y = (k >= 1) ? 0.4 * h : 0.0;
    EXPECT_NEAR(prediction(0, k), 0.0, 1e-15) << "k = " << k;
    EXPECT_NEAR(prediction(1, k), expected_y, 1e-15) << "k = " << k;
    EXPECT_NEAR(prediction(2, k), 0.0, 1e-15) << "k = " << k;
  }
  EXPECT_NEAR(prediction(1, 5), 0.4 * h, 1e-15);
}

TEST(MpcModelTest, VelocityIntegrator_PureOmegaOnlyAdvancesYaw)
{
  srm27_minco_core::MpcModel model;
  ASSERT_TRUE(model.configure(velocityModelConfig(5)));
  const double h = model.dt();

  const Eigen::VectorXd z0 = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd u = Eigen::VectorXd::Zero(model.decisionDim());
  u(2) = 0.3;  // 只有第 0 步的 omega 非零

  const Eigen::MatrixXd prediction = model.predict(z0, u);
  ASSERT_EQ(prediction.cols(), 6);
  for (int k = 0; k <= 5; ++k) {
    const double expected_yaw = (k >= 1) ? 0.3 * h : 0.0;
    EXPECT_NEAR(prediction(0, k), 0.0, 1e-15) << "k = " << k;
    EXPECT_NEAR(prediction(1, k), 0.0, 1e-15) << "k = " << k;
    EXPECT_NEAR(prediction(2, k), expected_yaw, 1e-15) << "k = " << k;
  }
  EXPECT_NEAR(prediction(2, 5), 0.3 * h, 1e-15);
}

TEST(MpcModelTest, AccelerationIntegrator_ConstantAccelerationAnalytic)
{
  const double h = 0.02;
  const double a = 0.5;
  const int steps = 30;
  srm27_minco_core::MpcModel model;
  ASSERT_TRUE(model.configure(accelerationModelConfig(steps, h)));

  EXPECT_EQ(model.stateDim(), 6);
  EXPECT_EQ(model.inputDim(), 3);
  EXPECT_EQ(model.decisionDim(), 3 * steps);
  EXPECT_EQ(model.velocityIndex(), 3);
  EXPECT_EQ(model.omegaIndex(), 5);

  // 零阶保持双积分：`z = [x, y, yaw, vx, vy, omega]`，`u = [ax, ay, alpha]`。
  EXPECT_EQ(model.a().rows(), 6);
  EXPECT_NEAR(model.a()(0, 3), h, 1e-15);
  EXPECT_NEAR(model.a()(1, 4), h, 1e-15);
  EXPECT_NEAR(model.a()(2, 5), h, 1e-15);
  EXPECT_NEAR(model.b()(0, 0), 0.5 * h * h, 1e-15);
  EXPECT_NEAR(model.b()(3, 0), h, 1e-15);

  const Eigen::VectorXd z0 = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd u = Eigen::VectorXd::Zero(model.decisionDim());
  for (int i = 0; i < steps; ++i) {
    u(3 * i + 0) = a;  // 恒定 ax，其余分量为 0
  }

  const Eigen::MatrixXd prediction = model.predict(z0, u);
  ASSERT_EQ(prediction.rows(), 6);
  ASSERT_EQ(prediction.cols(), steps + 1);

  // 解析解 `vx(k) = a h k`、`x(k) = 0.5 a (h k)^2`；容差 1e-12 用于排除报告排版列错位。
  double max_velocity_error = 0.0;
  double max_position_error = 0.0;
  for (int k = 0; k <= steps; ++k) {
    const double t = h * static_cast<double>(k);
    max_velocity_error = std::max(max_velocity_error, std::abs(prediction(3, k) - a * t));
    max_position_error = std::max(max_position_error, std::abs(prediction(0, k) - 0.5 * a * t * t));
    EXPECT_NEAR(prediction(3, k), a * t, 1e-12) << "k = " << k;
    EXPECT_NEAR(prediction(0, k), 0.5 * a * t * t, 1e-12) << "k = " << k;
    EXPECT_NEAR(prediction(1, k), 0.0, 1e-15) << "k = " << k;
    EXPECT_NEAR(prediction(2, k), 0.0, 1e-15) << "k = " << k;
    EXPECT_NEAR(prediction(4, k), 0.0, 1e-15) << "k = " << k;
    EXPECT_NEAR(prediction(5, k), 0.0, 1e-15) << "k = " << k;
  }
  printMeasurement("accel_model_max_velocity_error", max_velocity_error);
  printMeasurement("accel_model_max_position_error", max_position_error);
}

TEST(MpcModelTest, Predict_RejectsInvalidArguments)
{
  srm27_minco_core::MpcModel model;
  ASSERT_TRUE(model.configure(velocityModelConfig(5)));

  const Eigen::VectorXd z0 = Eigen::VectorXd::Zero(3);
  const Eigen::VectorXd u = Eigen::VectorXd::Zero(model.decisionDim());

  // 初值长度不匹配。
  EXPECT_EQ(model.predict(Eigen::VectorXd::Zero(2), u).size(), 0);
  EXPECT_EQ(model.predict(Eigen::VectorXd::Zero(6), u).size(), 0);
  // 输入长度不匹配。
  EXPECT_EQ(model.predict(z0, Eigen::VectorXd::Zero(model.decisionDim() - 1)).size(), 0);
  EXPECT_EQ(model.predict(z0, Eigen::VectorXd::Zero(model.decisionDim() + 3)).size(), 0);
  // 参数合法时结果非空。
  EXPECT_EQ(model.predict(z0, u).size(), 3 * 6);
}

TEST(MpcModelTest, Configure_RejectsInvalidParameters)
{
  srm27_minco_core::MpcModel model;
  std::string reason;

  srm27_minco_core::MpcModel::Config config = velocityModelConfig(5);
  config.prediction_steps = 0;
  EXPECT_FALSE(model.configure(config, &reason));
  EXPECT_FALSE(reason.empty());
  EXPECT_FALSE(model.configured());

  config = velocityModelConfig(5);
  config.prediction_dt = 0.0;
  EXPECT_FALSE(model.configure(config));
  config.prediction_dt = -0.02;
  EXPECT_FALSE(model.configure(config));
  config.prediction_dt = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(model.configure(config));
  EXPECT_FALSE(model.configured());

  // 配置失败后模型不可用，`predict()` 必须返回空矩阵。
  EXPECT_EQ(model.predict(Eigen::VectorXd::Zero(3), Eigen::VectorXd::Zero(15)).size(), 0);
}

TEST(MpcModelTest, Collocation_MatchesStepwisePrediction)
{
  const int steps = 6;
  srm27_minco_core::MpcModel model;
  ASSERT_TRUE(model.configure(velocityModelConfig(steps)));
  const int nz = model.stateDim();

  std::mt19937 generator(20260227U);
  std::uniform_real_distribution<double> distribution(-0.4, 0.4);
  Eigen::VectorXd z0(nz);
  Eigen::VectorXd u(model.decisionDim());
  for (int i = 0; i < nz; ++i) {
    z0(i) = distribution(generator);
  }
  for (int i = 0; i < u.size(); ++i) {
    u(i) = distribution(generator);
  }

  const Eigen::MatrixXd prediction = model.predict(z0, u);
  ASSERT_EQ(prediction.cols(), steps + 1);

  // 展平顺序：`Z = [Z_0; Z_1; ...; Z_{N-1}]`，每块 nz 行（`predict()` 结果跳过初值列）。
  const Eigen::Map<const Eigen::VectorXd> flat(
    prediction.data() + nz, static_cast<Eigen::Index>(nz) * steps);
  const Eigen::VectorXd collocated = model.sx() * z0 + model.su() * u;
  ASSERT_EQ(flat.size(), collocated.size());
  const double error = (flat - collocated).cwiseAbs().maxCoeff();
  printMeasurement("collocation_max_abs_error", error);
  EXPECT_LT(error, 1e-10);

  // 逐步累加也应与 `predict()` 一致（显式写出，避免只看凝聚式自洽）。
  Eigen::VectorXd manual = z0;
  for (int i = 0; i < steps; ++i) {
    manual = model.a() * manual + model.b() * u.segment(i * model.inputDim(), model.inputDim());
    EXPECT_TRUE(manual.isApprox(prediction.col(i + 1), 1e-12)) << "i = " << i;
  }
}

// ---------------------------------------------------------------------------
// QP：解析解、约束、非法输入、诊断量
// ---------------------------------------------------------------------------

TEST(MpcSolverTest, UnconstrainedQuadratic_HasZeroSolution)
{
  const int steps = 5;
  srm27_minco_core::MpcSolver solver;
  ASSERT_TRUE(solver.configure(velocityModelConfig(steps), unitWeightConfig()));

  srm27_minco_core::MpcInitialState state;
  state.z0 = Eigen::VectorXd::Zero(3);
  state.has_previous_input = false;

  const srm27_minco_core::MpcReference reference = zeroReference(3, 3, steps);
  srm27_minco_core::MpcSolution solution;
  ASSERT_TRUE(solver.solve(state, reference, solution));
  EXPECT_EQ(solution.status, srm27_minco_core::SolveStatus::kSuccess);
  EXPECT_TRUE(solution.usable());

  // `J(U) = U^T (Su^T Q Su + R) U`，唯一最优解为 U = 0。
  // 注意：合速度多边形约束始终存在，但在 U = 0 处是松弛约束，不改变该解析解。
  const double max_abs = solution.u.cwiseAbs().maxCoeff();
  printMeasurement("unconstrained_max_abs_u", max_abs);
  EXPECT_LT(max_abs, 1e-8);
  EXPECT_EQ(solution.u.rows(), 3);
  EXPECT_EQ(solution.u.cols(), steps);
}

TEST(MpcSolverTest, ConstantVelocityReference_IsTrackedExactly)
{
  const int steps = 10;
  srm27_minco_core::MpcSolverConfig config = unitWeightConfig();
  config.q_position = 20.0;
  config.q_yaw = 5.0;

  srm27_minco_core::MpcSolver solver;
  ASSERT_TRUE(solver.configure(velocityModelConfig(steps), config));

  const Eigen::Vector3d z0 = Eigen::Vector3d::Zero();
  const Eigen::Vector3d u_ref(0.3, 0.0, 0.0);
  const srm27_minco_core::MpcReference reference = constantReference(solver.model(), z0, u_ref);

  srm27_minco_core::MpcInitialState state;
  state.z0 = z0;
  state.has_previous_input = false;

  srm27_minco_core::MpcSolution solution;
  ASSERT_TRUE(solver.solve(state, reference, solution));
  ASSERT_EQ(solution.status, srm27_minco_core::SolveStatus::kSuccess);

  // 参考本身可精确达到，代价在该点严格凸且为 0，因此解应等于 `u_ref`。
  double max_error = 0.0;
  for (int i = 0; i < steps; ++i) {
    max_error = std::max(max_error, (solution.u.col(i) - u_ref).cwiseAbs().maxCoeff());
  }
  printMeasurement("constant_reference_max_u_error", max_error);
  EXPECT_LT(max_error, 1e-6);

  // 预测状态应与参考位置一致（末列对应预测的最后一步）。
  EXPECT_NEAR(solution.z(0, steps), reference.z_ref(0, steps - 1), 1e-6);
  EXPECT_NEAR(solution.z(1, steps), reference.z_ref(1, steps - 1), 1e-6);
}

TEST(MpcSolverTest, SpeedPolygon_LimitsResultantSpeedForAxialReference)
{
  const int steps = 5;
  const double v_max = 0.5;
  srm27_minco_core::MpcSolverConfig config = productSolverConfig();
  config.speed_polygon_sides = 8;

  srm27_minco_core::MpcSolver solver;
  ASSERT_TRUE(solver.configure(velocityModelConfig(steps), config));
  srm27_minco_core::Limits2D limits;
  limits.max_linear_speed = v_max;
  limits.max_angular_speed = 0.3;
  ASSERT_TRUE(solver.setLimits(limits));

  // 故意要求 2.0 m/s（远超上限），检查解被多边形约束截住。
  const Eigen::Vector3d u_ref(2.0, 0.0, 0.0);
  const srm27_minco_core::MpcReference reference =
    constantReference(solver.model(), Eigen::Vector3d::Zero(), u_ref);

  srm27_minco_core::MpcInitialState state;
  state.z0 = Eigen::Vector3d::Zero();
  state.has_previous_input = false;

  srm27_minco_core::MpcSolution solution;
  ASSERT_TRUE(solver.solve(state, reference, solution));

  double max_speed = 0.0;
  for (int i = 0; i < steps; ++i) {
    const double speed = std::hypot(solution.u(0, i), solution.u(1, i));
    max_speed = std::max(max_speed, speed);
    EXPECT_LE(speed, v_max + 1e-6) << "i = " << i;
  }
  printMeasurement("polygon_axial_max_speed", max_speed);
  printMeasurement("polygon_axial_inradius", v_max * std::cos(kPi / 8.0));
  printMeasurement("polygon_axial_solution_vx_step0", solution.u(0, 0));
  // 约束确实被激活：内接多边形沿 x 轴的内接半径为 v_max*cos(pi/8) ≈ 0.462。
  EXPECT_GE(max_speed, 0.45);
  EXPECT_LE(solution.predicted_max_speed, v_max + 1e-6);
}

TEST(MpcSolverTest, SpeedPolygon_LimitsResultantSpeedForDiagonalReference)
{
  const int steps = 5;
  const double v_max = 0.5;
  srm27_minco_core::MpcSolverConfig config = productSolverConfig();
  config.speed_polygon_sides = 8;

  srm27_minco_core::MpcSolver solver;
  ASSERT_TRUE(solver.configure(velocityModelConfig(steps), config));
  srm27_minco_core::Limits2D limits;
  limits.max_linear_speed = v_max;
  limits.max_angular_speed = 0.3;
  ASSERT_TRUE(solver.setLimits(limits));

  // 45° 斜向参考：只限制 |vx|、|vy| 时会允许 sqrt(2)*v_max ≈ 0.707 m/s。
  const double diagonal = 2.0 / std::sqrt(2.0);
  const Eigen::Vector3d u_ref(diagonal, diagonal, 0.0);
  const srm27_minco_core::MpcReference reference =
    constantReference(solver.model(), Eigen::Vector3d::Zero(), u_ref);

  srm27_minco_core::MpcInitialState state;
  state.z0 = Eigen::Vector3d::Zero();
  state.has_previous_input = false;

  srm27_minco_core::MpcSolution solution;
  ASSERT_TRUE(solver.solve(state, reference, solution));

  double max_speed = 0.0;
  for (int i = 0; i < steps; ++i) {
    const double speed = std::hypot(solution.u(0, i), solution.u(1, i));
    max_speed = std::max(max_speed, speed);
    EXPECT_LE(speed, v_max + 1e-6) << "i = " << i;
    // 对称参考下解也应保持对称（不会靠单轴“绕开”合速度约束）。
    EXPECT_NEAR(solution.u(0, i), solution.u(1, i), 1e-6) << "i = " << i;
  }
  printMeasurement("polygon_diagonal_max_speed", max_speed);
  printMeasurement("polygon_diagonal_vx_step0", solution.u(0, 0));
  printMeasurement("polygon_diagonal_vy_step0", solution.u(1, 0));
  EXPECT_GE(max_speed, 0.45);
  // 分量盒约束单独允许 (0.5, 0.5)，模长 0.707；此处必须已明显低于它。
  EXPECT_LT(max_speed, std::sqrt(2.0) * v_max - 0.1);
}

TEST(MpcSolverTest, FirstStepDifference_UsesRealControlInterval)
{
  const int steps = 5;
  srm27_minco_core::MpcModel::Config model_config = velocityModelConfig(steps);
  srm27_minco_core::MpcSolverConfig solver_config = productSolverConfig();
  solver_config.max_linear_accel = 0.3;

  srm27_minco_core::MpcModel reference_model;
  ASSERT_TRUE(reference_model.configure(model_config));
  const Eigen::Vector3d u_ref(2.0, 0.0, 0.0);
  const srm27_minco_core::MpcReference reference_prototype =
    constantReference(reference_model, Eigen::Vector3d::Zero(), u_ref);

  const auto first_step_vx = [&](const double _control_interval) {
    srm27_minco_core::MpcSolver solver;
    if (!solver.configure(model_config, solver_config)) {
      return -1.0;
    }
    srm27_minco_core::Limits2D limits;
    limits.max_linear_speed = 1.0;
    limits.max_angular_speed = 0.3;
    if (!solver.setLimits(limits)) {
      return -1.0;
    }
    srm27_minco_core::MpcInitialState state;
    state.z0 = Eigen::Vector3d::Zero();
    state.has_previous_input = true;
    state.previous_applied_input = Eigen::VectorXd::Zero(3);
    state.control_interval = _control_interval;
    srm27_minco_core::MpcSolution solution;
    if (!solver.solve(state, reference_prototype, solution)) {
      return -1.0;
    }
    return solution.u(0, 0);
  };

  // 首步差分上限 = max_linear_accel * control_interval（不是模型 dt）。
  const double vx_at_002 = first_step_vx(0.02);
  const double vx_at_005 = first_step_vx(0.05);
  printMeasurement("first_step_vx_interval_0.02", vx_at_002);
  printMeasurement("first_step_vx_interval_0.05", vx_at_005);
  printMeasurement("first_step_bound_interval_0.02", 0.3 * 0.02);
  printMeasurement("first_step_bound_interval_0.05", 0.3 * 0.05);

  ASSERT_GE(vx_at_002, 0.0);
  ASSERT_GE(vx_at_005, 0.0);
  EXPECT_LE(vx_at_002, 0.3 * 0.02 + 1e-6);
  EXPECT_LE(vx_at_005, 0.3 * 0.05 + 1e-6);
  // 0.02 的上限必须严格小于 0.05；若实现按模型 dt 放宽/错用间隔，两者会相等。
  EXPECT_LT(vx_at_002, vx_at_005);
  // 约束激活：参考很大时首步解顶到各自的上限。
  EXPECT_GE(vx_at_002, 0.9 * 0.3 * 0.02);
  EXPECT_GE(vx_at_005, 0.9 * 0.3 * 0.05);
}

TEST(MpcSolverTest, FirstStepDifference_PreservesNonzeroPreviousInputDirection)
{
  // 零速基准无法发现移项符号错误；同时覆盖正负分量及冷热启动。
  for (const bool hot_start : {false, true}) {
    auto config = productSolverConfig();
    config.max_linear_accel = 3.0;
    config.max_angular_accel = 0.5;
    config.use_hot_start = hot_start;
    srm27_minco_core::MpcSolver solver;
    ASSERT_TRUE(solver.configure(velocityModelConfig(30), config));
    srm27_minco_core::Limits2D limits;
    limits.max_linear_speed = 1.5;
    limits.max_angular_speed = 1.0;
    ASSERT_TRUE(solver.setLimits(limits));

    for (const double sign : {1.0, -1.0}) {
      SCOPED_TRACE(::testing::Message() << "hot_start=" << hot_start << " sign=" << sign);
      const Eigen::Vector3d previous = sign * Eigen::Vector3d(0.5, -0.3, 0.2);
      srm27_minco_core::MpcInitialState state;
      state.z0 = Eigen::Vector3d::Zero();
      state.has_previous_input = true;
      state.previous_applied_input = previous;
      state.control_interval = 0.02;
      const auto reference = constantReference(solver.model(), Eigen::Vector3d::Zero(), previous);
      srm27_minco_core::MpcSolution solution;
      ASSERT_TRUE(solver.solve(state, reference, solution)) << solution.message;
      std::cout << "[MEASURE] previous=" << previous.transpose()
                << " first_input=" << solution.u.col(0).transpose() << std::endl;
      // 匀速轨迹本身是零代价可行解，不能被差分约束强迫反向。
      EXPECT_TRUE(solution.u.col(0).isApprox(previous, 1e-6));
      for (int i = 0; i < solver.model().steps(); ++i) {
        Eigen::Vector3d baseline = previous;
        if (i > 0) {
          baseline = solution.u.col(i - 1);
        }
        const Eigen::Vector3d delta = solution.u.col(i) - baseline;
        EXPECT_LE(std::abs(delta.x()), config.max_linear_accel * 0.02 + 1e-6);
        EXPECT_LE(std::abs(delta.y()), config.max_linear_accel * 0.02 + 1e-6);
        EXPECT_LE(std::abs(delta.z()), config.max_angular_accel * 0.02 + 1e-6);
      }
    }
  }
}

TEST(MpcSolverTest, FirstStepDifference_BrakesBeforeReversing)
{
  auto config = productSolverConfig();
  config.max_linear_accel = 3.0;
  srm27_minco_core::MpcSolver solver;
  ASSERT_TRUE(solver.configure(velocityModelConfig(30), config));
  srm27_minco_core::Limits2D limits;
  limits.max_linear_speed = 1.5;
  limits.max_angular_speed = 1.0;
  ASSERT_TRUE(solver.setLimits(limits));
  srm27_minco_core::MpcInitialState state;
  state.z0 = Eigen::Vector3d::Zero();
  state.has_previous_input = true;
  state.previous_applied_input = Eigen::Vector3d(0.5, 0.0, 0.0);
  state.control_interval = 0.02;
  const auto reference =
    constantReference(solver.model(), Eigen::Vector3d::Zero(), Eigen::Vector3d(-0.5, 0.0, 0.0));
  srm27_minco_core::MpcSolution solution;
  ASSERT_TRUE(solver.solve(state, reference, solution)) << solution.message;
  EXPECT_NEAR(solution.u(0, 0), 0.44, 1e-6);
}

TEST(MpcSolverTest, PreviousInputFlag_TogglesFirstStepDifference)
{
  const int steps = 5;
  srm27_minco_core::MpcSolverConfig config = productSolverConfig();
  config.max_linear_accel = 0.3;

  const Eigen::Vector3d u_ref(2.0, 0.0, 0.0);
  srm27_minco_core::MpcSolver solver;
  ASSERT_TRUE(solver.configure(velocityModelConfig(steps), config));
  srm27_minco_core::Limits2D limits;
  limits.max_linear_speed = 0.5;
  limits.max_angular_speed = 0.3;
  ASSERT_TRUE(solver.setLimits(limits));
  const srm27_minco_core::MpcReference reference =
    constantReference(solver.model(), Eigen::Vector3d::Zero(), u_ref);

  srm27_minco_core::MpcInitialState with_previous;
  with_previous.z0 = Eigen::Vector3d::Zero();
  with_previous.has_previous_input = true;
  with_previous.previous_applied_input = Eigen::VectorXd::Zero(3);
  with_previous.control_interval = 0.02;

  srm27_minco_core::MpcInitialState without_previous = with_previous;
  without_previous.has_previous_input = false;

  srm27_minco_core::MpcSolution solution_with;
  srm27_minco_core::MpcSolution solution_without;
  ASSERT_TRUE(solver.solve(with_previous, reference, solution_with));
  ASSERT_TRUE(solver.solve(without_previous, reference, solution_without));

  printMeasurement("first_step_vx_with_previous", solution_with.u(0, 0));
  printMeasurement("first_step_vx_without_previous", solution_without.u(0, 0));

  // 有上一条命令：首步受 0.3*0.02 = 0.006 限制。
  EXPECT_LE(solution_with.u(0, 0), 0.3 * 0.02 + 1e-6);
  // 没有上一条命令：首步不做差分约束，仅受合速度多边形限制。
  EXPECT_GT(solution_without.u(0, 0), solution_with.u(0, 0));
  EXPECT_GE(solution_without.u(0, 0), 0.45);
  EXPECT_LE(solution_without.u(0, 0), 0.5 + 1e-6);
}

TEST(MpcSolverTest, AngularSpeedBound_LimitsYawRate)
{
  const int steps = 5;
  srm27_minco_core::MpcSolverConfig config = productSolverConfig();

  srm27_minco_core::MpcSolver solver;
  ASSERT_TRUE(solver.configure(velocityModelConfig(steps), config));
  srm27_minco_core::Limits2D limits;
  limits.max_linear_speed = 0.5;
  limits.max_angular_speed = 0.3;
  ASSERT_TRUE(solver.setLimits(limits));

  // 参考要求 1.0 rad/s，远超 0.3 上限。
  const Eigen::Vector3d u_ref(0.0, 0.0, 1.0);
  const srm27_minco_core::MpcReference reference =
    constantReference(solver.model(), Eigen::Vector3d::Zero(), u_ref);

  srm27_minco_core::MpcInitialState state;
  state.z0 = Eigen::Vector3d::Zero();
  state.has_previous_input = false;

  srm27_minco_core::MpcSolution solution;
  ASSERT_TRUE(solver.solve(state, reference, solution));

  double max_omega = 0.0;
  for (int i = 0; i < steps; ++i) {
    max_omega = std::max(max_omega, std::abs(solution.u(2, i)));
    EXPECT_LE(std::abs(solution.u(2, i)), 0.3 + 1e-6) << "i = " << i;
    // 平移参考为 0，平移分量不应被角速度约束耦合出来。
    EXPECT_NEAR(solution.u(0, i), 0.0, 1e-6) << "i = " << i;
    EXPECT_NEAR(solution.u(1, i), 0.0, 1e-6) << "i = " << i;
  }
  printMeasurement("angular_bound_max_omega", max_omega);
  EXPECT_GE(max_omega, 0.3 - 1e-6);
}

TEST(MpcSolverTest, InvalidInputs_ReturnInvalidStatus)
{
  const int steps = 5;
  srm27_minco_core::MpcSolver solver;
  ASSERT_TRUE(solver.configure(velocityModelConfig(steps), productSolverConfig()));

  srm27_minco_core::MpcInitialState state;
  state.z0 = Eigen::Vector3d::Zero();
  state.has_previous_input = false;
  const srm27_minco_core::MpcReference reference =
    constantReference(solver.model(), Eigen::Vector3d::Zero(), Eigen::Vector3d(0.3, 0.0, 0.0));

  const auto expect_invalid = [&](
                                const srm27_minco_core::MpcInitialState & _state,
                                const srm27_minco_core::MpcReference & _reference,
                                const char * _label) {
    srm27_minco_core::MpcSolution solution;
    EXPECT_FALSE(solver.solve(_state, _reference, solution)) << _label;
    EXPECT_EQ(solution.status, srm27_minco_core::SolveStatus::kInvalidInput) << _label;
    EXPECT_FALSE(solution.usable()) << _label;
  };

  // 参考状态维度错误。
  srm27_minco_core::MpcReference bad_reference = reference;
  bad_reference.z_ref = Eigen::MatrixXd::Zero(2, steps);
  expect_invalid(state, bad_reference, "z_ref rows");

  bad_reference = reference;
  bad_reference.z_ref = Eigen::MatrixXd::Zero(3, steps + 1);
  expect_invalid(state, bad_reference, "z_ref cols");

  bad_reference = reference;
  bad_reference.u_ref = Eigen::MatrixXd::Zero(2, steps);
  expect_invalid(state, bad_reference, "u_ref rows");

  // 初值含 NaN。
  srm27_minco_core::MpcInitialState nan_state = state;
  nan_state.z0(1) = std::numeric_limits<double>::quiet_NaN();
  expect_invalid(nan_state, reference, "z0 NaN");

  // 初值长度错误。
  srm27_minco_core::MpcInitialState short_state = state;
  short_state.z0 = Eigen::VectorXd::Zero(2);
  expect_invalid(short_state, reference, "z0 size");

  // 上一周期命令维度错误。
  srm27_minco_core::MpcInitialState bad_previous = state;
  bad_previous.has_previous_input = true;
  bad_previous.previous_applied_input = Eigen::VectorXd::Zero(2);
  expect_invalid(bad_previous, reference, "previous input size");
}

TEST(MpcSolverTest, PlanarWeight_MatchesTangentNormalQuadraticForm)
{
  const double theta = kPi / 4.0;
  const double q_along = 1.0;
  const double q_cross = 4.0;
  const Eigen::Matrix2d q = srm27_minco_core::MpcSolver::planarWeight(theta, q_along, q_cross);

  const Eigen::Vector2d tangent(std::cos(theta), std::sin(theta));
  const Eigen::Vector2d normal(-std::sin(theta), std::cos(theta));

  // `Q = q_along t t^T + q_cross n n^T` 的三条基本性质。
  EXPECT_NEAR(tangent.dot(q * tangent), q_along, 1e-12);
  EXPECT_NEAR(normal.dot(q * normal), q_cross, 1e-12);
  EXPECT_NEAR(tangent.dot(q * normal), 0.0, 1e-12);

  // 交叉项符号必须是 `(q_along - q_cross) cos(theta) sin(theta)`（方案 §3.5 修正项 2）。
  const double expected_cross = (q_along - q_cross) * std::cos(theta) * std::sin(theta);
  printMeasurement("planar_weight_cross_term", q(0, 1));
  EXPECT_NEAR(q(0, 1), expected_cross, 1e-12);
  EXPECT_NEAR(q(1, 0), expected_cross, 1e-12);
  EXPECT_LT(q(0, 1), 0.0);  // q_along < q_cross 时符号为负，写反会得到 +1.5
  EXPECT_NEAR(std::abs(q(0, 1)), 1.5, 1e-12);

  // 主对角项与直接展开一致。
  const double c = std::cos(theta);
  const double s = std::sin(theta);
  EXPECT_NEAR(q(0, 0), q_along * c * c + q_cross * s * s, 1e-12);
  EXPECT_NEAR(q(1, 1), q_along * s * s + q_cross * c * c, 1e-12);

  // 任意向量的二次型等于按切/法分量加权的结果。
  const Eigen::Vector2d probe(0.6, -0.8);
  const double along_component = probe.dot(tangent);
  const double cross_component = probe.dot(normal);
  EXPECT_NEAR(
    probe.dot(q * probe),
    q_along * along_component * along_component + q_cross * cross_component * cross_component,
    1e-12);
}

TEST(MpcSolverTest, PlanarWeight_AxisAlignedAnglesAreDiagonal)
{
  const double q_along = 20.0;
  const double q_cross = 40.0;

  const Eigen::Matrix2d at_zero = srm27_minco_core::MpcSolver::planarWeight(0.0, q_along, q_cross);
  EXPECT_NEAR(at_zero(0, 1), 0.0, 1e-12);
  EXPECT_NEAR(at_zero(1, 0), 0.0, 1e-12);
  EXPECT_NEAR(at_zero(0, 0), q_along, 1e-12);
  EXPECT_NEAR(at_zero(1, 1), q_cross, 1e-12);

  const Eigen::Matrix2d at_half_pi =
    srm27_minco_core::MpcSolver::planarWeight(kPi / 2.0, q_along, q_cross);
  EXPECT_NEAR(at_half_pi(0, 1), 0.0, 1e-12);
  EXPECT_NEAR(at_half_pi(1, 0), 0.0, 1e-12);
  EXPECT_NEAR(at_half_pi(0, 0), q_cross, 1e-12);
  EXPECT_NEAR(at_half_pi(1, 1), q_along, 1e-12);
}

TEST(MpcSolverTest, CommandAt_InterpolatesLinearlyAndClampsToHorizon)
{
  const int steps = 5;
  const double h = 0.02;
  srm27_minco_core::MpcSolver solver;
  ASSERT_TRUE(solver.configure(velocityModelConfig(steps, h), productSolverConfig()));

  // 手工构造已知命令序列，便于手算插值结果。
  srm27_minco_core::MpcSolution solution;
  solution.status = srm27_minco_core::SolveStatus::kSuccess;
  solution.u = Eigen::MatrixXd::Zero(3, steps);
  solution.u(0, 0) = 0.1;
  solution.u(0, 1) = 0.2;
  solution.u(0, 2) = 0.4;
  solution.u(0, 3) = 0.8;
  solution.u(0, 4) = 1.6;
  solution.u(2, 4) = 0.3;

  Eigen::VectorXd command;
  // lookahead = 0 返回第一步命令。
  ASSERT_TRUE(solver.commandAt(solution, 0.0, command));
  ASSERT_EQ(command.size(), 3);
  EXPECT_NEAR(command(0), 0.1, 1e-12);

  // lookahead = 0.02 正好落在第 1 个预测点上，不做插值。
  ASSERT_TRUE(solver.commandAt(solution, h, command));
  EXPECT_NEAR(command(0), 0.2, 1e-12);

  // lookahead = 0.03 = 1.5*h，落在第 1、2 步之间：0.5*(0.2 + 0.4) = 0.3。
  ASSERT_TRUE(solver.commandAt(solution, 0.03, command));
  EXPECT_NEAR(command(0), 0.5 * (0.2 + 0.4), 1e-12);
  EXPECT_GT(command(0), 0.2);
  EXPECT_LT(command(0), 0.4);

  // lookahead = 0.07 = 3.5*h：0.5*(0.8 + 1.6) = 1.2。
  ASSERT_TRUE(solver.commandAt(solution, 0.07, command));
  EXPECT_NEAR(command(0), 0.5 * (0.8 + 1.6), 1e-12);

  // 超过 horizon 时裁剪到最后一步（含角速度分量）。
  ASSERT_TRUE(solver.commandAt(solution, 10.0, command));
  EXPECT_NEAR(command(0), 1.6, 1e-12);
  EXPECT_NEAR(command(2), 0.3, 1e-12);
}

TEST(MpcSolverTest, CommandAt_RejectsUnusableSolution)
{
  const int steps = 5;
  srm27_minco_core::MpcSolver solver;
  ASSERT_TRUE(solver.configure(velocityModelConfig(steps), productSolverConfig()));

  srm27_minco_core::MpcSolution solution;
  solution.u = Eigen::MatrixXd::Zero(3, steps);
  Eigen::VectorXd command;
  // 未求解/求解失败的结果不可用。
  EXPECT_FALSE(solver.commandAt(solution, 0.0, command));
  solution.status = srm27_minco_core::SolveStatus::kTimeout;
  EXPECT_FALSE(solver.commandAt(solution, 0.0, command));

  solution.status = srm27_minco_core::SolveStatus::kSuccess;
  EXPECT_TRUE(solver.commandAt(solution, 0.0, command));
  // 负的 lookahead 属于非法输入。
  EXPECT_FALSE(solver.commandAt(solution, -0.01, command));
  EXPECT_FALSE(solver.commandAt(solution, std::numeric_limits<double>::quiet_NaN(), command));
}

TEST(MpcSolverTest, Solve_FillsDiagnostics)
{
  const int steps = 10;
  srm27_minco_core::MpcSolverConfig config = productSolverConfig();
  srm27_minco_core::MpcSolver solver;
  ASSERT_TRUE(solver.configure(velocityModelConfig(steps), config));
  srm27_minco_core::Limits2D limits;
  limits.max_linear_speed = 0.5;
  limits.max_angular_speed = 0.3;
  ASSERT_TRUE(solver.setLimits(limits));

  srm27_minco_core::MpcInitialState state;
  state.z0 = Eigen::Vector3d::Zero();
  state.has_previous_input = false;
  const srm27_minco_core::MpcReference reference =
    constantReference(solver.model(), Eigen::Vector3d::Zero(), Eigen::Vector3d(0.3, 0.0, 0.0));

  srm27_minco_core::MpcSolution solution;
  ASSERT_TRUE(solver.solve(state, reference, solution));
  EXPECT_EQ(solution.status, srm27_minco_core::SolveStatus::kSuccess);
  EXPECT_TRUE(solution.message.empty());
  EXPECT_EQ(solution.u.rows(), 3);
  EXPECT_EQ(solution.u.cols(), steps);
  EXPECT_EQ(solution.z.rows(), 3);
  EXPECT_EQ(solution.z.cols(), steps + 1);
  EXPECT_GE(solution.solve_time_ms, 0.0);
  EXPECT_GE(solution.iterations, 0);
  EXPECT_GE(solution.predicted_max_speed, 0.0);
  EXPECT_GE(solution.max_constraint_violation, 0.0);
  EXPECT_LE(solution.max_constraint_violation, 1e-6);
  EXPECT_NEAR(solution.predicted_max_speed, 0.3, 1e-6);
  printMeasurement("solve_time_ms", solution.solve_time_ms);
  printMeasurement("max_constraint_violation", solution.max_constraint_violation);
  printMeasurement("predicted_max_speed", solution.predicted_max_speed);

  // 求解后的命令应与解的第一列一致，并支持插值读取。
  Eigen::VectorXd command;
  ASSERT_TRUE(solver.commandAt(solution, 0.0, command));
  EXPECT_TRUE(command.isApprox(solution.u.col(0), 1e-12));
  ASSERT_TRUE(solver.commandAt(solution, 100.0, command));
  EXPECT_TRUE(command.isApprox(solution.u.col(steps - 1), 1e-12));
}
