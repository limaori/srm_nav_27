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
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>
#include <numeric>
#include <string>
#include <vector>

#include "srm27_minco_core/esdf_2d.hpp"
#include "srm27_minco_core/grid_snapshot.hpp"
#include "srm27_minco_core/minco_optimizer.hpp"
#include "srm27_minco_core/smooth_cost.hpp"
#include "srm27_minco_core/trajectory_2d.hpp"
#include "srm27_minco_core/trajectory_initializer.hpp"
#include "srm27_minco_core/trajectory_validator.hpp"
#include "srm27_minco_core/types.hpp"

using srm27_minco_core::CellState;
using srm27_minco_core::Esdf2D;
using srm27_minco_core::GridSnapshot;
using srm27_minco_core::Limits2D;
using srm27_minco_core::MincoOptimizer;
using srm27_minco_core::MincoOptimizerConfig;
using srm27_minco_core::MincoOptimizeResult;
using srm27_minco_core::SolveStatus;
using srm27_minco_core::Trajectory2D;
using srm27_minco_core::TrajectoryInitialGuess;
using srm27_minco_core::TrajectoryInitializer;

namespace
{

/// \brief 把实测指标格式化成科学计数法字符串（gtest XML 里可读）。
std::string formatMetric(double _value)
{
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.3e", _value);
  return std::string(buffer);
}

/// \brief 把秒数格式化成分秒字符串。
std::string formatSeconds(double _value)
{
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.6f", _value);
  return std::string(buffer);
}

/// \brief 测试统一使用的段时长下限（s），与 MincoOptimizerConfig 默认值一致。
constexpr double kMinPieceDuration = 0.05;
/// \brief 位置分量的中心差分步长（m）。
constexpr double kPositionEpsilon = 1.0e-6;
/// \brief 时间分量的中心差分步长（无量纲的 s 变量）。
///
/// s 到 T 的映射是 T = Tmin + softplus(s)，dT/ds = sigmoid(s) ∈ (0, 1]，
/// 因此 s 的扰动被压缩到 T 上；取 1e-5 是为了让 (T*Δ) 仍在双精度可靠的
/// 差分区间内（实测该步长下位置分量可达 1e-9、时间分量可达 1e-8 量级）。
constexpr double kTimeEpsilon = 1.0e-5;

/// \brief ESDF 场景使用的机器人包络半径（m），与默认配置一致。
constexpr double kRobotRadius = 0.33;
/// \brief ESDF 场景使用的额外净空裕量（m）。
constexpr double kClearanceMargin = 0.05;

/// \brief 构造梯度检查用的配置：无障碍权重，关掉两阶段比例惩罚。
MincoOptimizerConfig makeGradientConfig()
{
  MincoOptimizerConfig config;
  config.w_jerk = 1.0;
  config.w_time = 10.0;
  config.w_obstacle = 0.0;
  config.w_velocity = 20.0;
  config.w_acceleration = 2.0;
  config.w_reference = 0.0;
  config.w_time_ratio = 0.0;
  config.samples_per_piece = 8;
  config.two_stage = false;
  return config;
}

/// \brief 构造梯度检查用的约束：阈值让速度/加速度软约束处于“部分激活”的平滑区间。
Limits2D makeGradientLimits()
{
  Limits2D limits;
  limits.max_linear_speed = 1.5;
  limits.max_linear_accel = 4.0;
  limits.max_angular_speed = 0.3;
  limits.max_angular_accel = 0.5;
  return limits;
}

/// \brief 梯度检查用的三段初值（手工填写，保证每次运行完全可复现）。
TrajectoryInitialGuess makeGradientGuess()
{
  TrajectoryInitialGuess guess;
  guess.waypoints = {{0.0, 0.0}, {1.0, 0.25}, {2.0, -0.15}, {3.0, 0.0}};
  guess.durations = {0.70, 0.80, 0.75};
  guess.head_position = {0.0, 0.0};
  guess.head_velocity = Eigen::Vector2d::Zero();
  guess.head_acceleration = Eigen::Vector2d::Zero();
  guess.tail_position = {3.0, 0.0};
  guess.tail_velocity = Eigen::Vector2d::Zero();
  guess.tail_acceleration = Eigen::Vector2d::Zero();
  return guess;
}

/// \brief 把内部自变量 x 还原成完整初值（encodeVariables 的逆映射）。
///
/// 时间分量按 T = Tmin + softplus(s) 还原，因此差分扰动的对象与
/// `MincoOptimizer::evaluate()` 内部使用的自变量完全一致。
TrajectoryInitialGuess decodeVariables(
  const Eigen::VectorXd & _x, const TrajectoryInitialGuess & _base, double _min_duration)
{
  TrajectoryInitialGuess guess = _base;
  const int pieces = _base.pieceCount();
  for (int i = 0; i + 1 < pieces; ++i) {
    guess.waypoints[static_cast<std::size_t>(i) + 1] = Eigen::Vector2d(_x(2 * i), _x(2 * i + 1));
  }
  for (int i = 0; i < pieces; ++i) {
    guess.durations[static_cast<std::size_t>(i)] =
      _min_duration + srm27_minco_core::softplus(_x(2 * (pieces - 1) + i));
  }
  return guess;
}

/// \brief 解析梯度与中心差分梯度的对比结果。
struct GradientCheck
{
  /// \brief 自变量（内部编码）。
  Eigen::VectorXd variables{};
  /// \brief 解析梯度。
  Eigen::VectorXd analytic{};
  /// \brief 中心差分梯度。
  Eigen::VectorXd numeric{};
  /// \brief 自变量编码往返误差（用于确认差分点与解析点一致）。
  double round_trip_error{0.0};
  /// \brief 位置分量最大相对误差。
  double max_point_error{0.0};
  /// \brief 时间分量最大相对误差。
  double max_time_error{0.0};
};

/// \brief 对 `evaluateObjective` 做中心差分梯度检查。
///
/// 相对误差定义为 |numeric - analytic| / max(1, |numeric|)：解析梯度量级通常是
/// 1e3 ~ 1e5，用 max(1, ...) 作分母既保持相对误差语义，又避免分量接近 0 时除零。
GradientCheck runGradientCheck(
  MincoOptimizer & _optimizer, const TrajectoryInitialGuess & _base, const Limits2D & _limits,
  double _min_duration, double _position_eps, double _time_eps)
{
  GradientCheck check;
  const Eigen::VectorXd x0 = _optimizer.encodeVariables(_base);
  check.variables = x0;

  // 差分点必须与解析点完全同一个自变量：先验证 encode/decode 往返一致。
  const TrajectoryInitialGuess center = decodeVariables(x0, _base, _min_duration);
  const Eigen::VectorXd round_trip = _optimizer.encodeVariables(center);
  check.round_trip_error = (round_trip - x0).norm();

  double cost = 0.0;
  Eigen::VectorXd gradient;
  const bool evaluated = _optimizer.evaluateObjective(center, _limits, cost, gradient);
  EXPECT_TRUE(evaluated) << "evaluateObjective 在合法初值上返回了 false";
  if (!evaluated) {
    return check;
  }
  check.analytic = gradient;
  check.numeric = Eigen::VectorXd::Zero(x0.size());

  const int inner_count = 2 * (_base.pieceCount() - 1);
  for (int i = 0; i < x0.size(); ++i) {
    const bool is_time = i >= inner_count;
    const double eps = is_time ? _time_eps : _position_eps;
    Eigen::VectorXd plus = x0;
    plus(i) += eps;
    Eigen::VectorXd minus = x0;
    minus(i) -= eps;
    double cost_plus = 0.0;
    double cost_minus = 0.0;
    Eigen::VectorXd scratch;
    EXPECT_TRUE(_optimizer.evaluateObjective(
      decodeVariables(plus, _base, _min_duration), _limits, cost_plus, scratch));
    EXPECT_TRUE(_optimizer.evaluateObjective(
      decodeVariables(minus, _base, _min_duration), _limits, cost_minus, scratch));
    check.numeric(i) = (cost_plus - cost_minus) / (2.0 * eps);
    const double denominator = std::max(1.0, std::abs(check.numeric(i)));
    const double error = std::abs(check.numeric(i) - gradient(i)) / denominator;
    if (is_time) {
      check.max_time_error = std::max(check.max_time_error, error);
    } else {
      check.max_point_error = std::max(check.max_point_error, error);
    }
  }
  return check;
}

/// \brief 构造段间 p/v/a 的最大拼接残差，供连续性断言使用。
struct ContinuityResiduals
{
  double position{0.0};
  double velocity{0.0};
  double acceleration{0.0};
};

ContinuityResiduals continuityOf(const Trajectory2D & _trajectory)
{
  ContinuityResiduals residuals;
  _trajectory.continuityResiduals(residuals.position, residuals.velocity, residuals.acceleration);
  return residuals;
}

/// \brief 全自由单元数组。
std::vector<CellState> makeFreeCells(int _size_x, int _size_y)
{
  return std::vector<CellState>(
    static_cast<std::size_t>(_size_x) * static_cast<std::size_t>(_size_y), CellState::kFree);
}

/// \brief 把世界坐标矩形内的单元置为指定语义（半开区间，按单元中心判定）。
void fillRectangle(
  std::vector<CellState> & _cells, int _size_x, int _size_y, double _origin_x, double _origin_y,
  double _resolution, double _x_min, double _y_min, double _x_max, double _y_max, CellState _state)
{
  for (int my = 0; my < _size_y; ++my) {
    const double y = _origin_y + (static_cast<double>(my) + 0.5) * _resolution;
    if (y < _y_min || y >= _y_max) {
      continue;
    }
    for (int mx = 0; mx < _size_x; ++mx) {
      const double x = _origin_x + (static_cast<double>(mx) + 0.5) * _resolution;
      if (x < _x_min || x >= _x_max) {
        continue;
      }
      _cells
        [static_cast<std::size_t>(my) * static_cast<std::size_t>(_size_x) +
         static_cast<std::size_t>(mx)] = _state;
    }
  }
}

/// \brief 由单元数组建立距离场。
std::shared_ptr<Esdf2D> makeEsdf(
  const std::vector<CellState> & _cells, int _size_x, int _size_y, double _origin_x,
  double _origin_y, double _resolution)
{
  GridSnapshot snapshot;
  if (!snapshot.reset(_origin_x, _origin_y, _resolution, _size_x, _size_y, _cells, 1U, 0.0)) {
    return nullptr;
  }
  auto esdf = std::make_shared<Esdf2D>();
  if (!esdf->build(snapshot)) {
    return nullptr;
  }
  return esdf;
}

/// \brief 测试地图的几何参数（x ∈ [-1.5, 4.5]，y ∈ [-2.0, 2.0]，分辨率 0.05）。
constexpr double kMapOriginX = -1.5;
constexpr double kMapOriginY = -2.0;
constexpr double kMapResolution = 0.05;
constexpr int kMapSizeX = 120;
constexpr int kMapSizeY = 80;

/// \brief 在轨迹长度上密集采样，返回最小净空与对应位置。
double minClearanceAlong(
  const Esdf2D & _esdf, const Trajectory2D & _trajectory, double _dt, Eigen::Vector2d & _where,
  double & _when)
{
  const double total = _trajectory.totalDuration();
  double best = std::numeric_limits<double>::infinity();
  _where = Eigen::Vector2d::Zero();
  _when = 0.0;
  const int steps = std::max(2, static_cast<int>(std::ceil(total / _dt)));
  for (int i = 0; i <= steps; ++i) {
    const double t = total * static_cast<double>(i) / static_cast<double>(steps);
    const Eigen::Vector2d position = _trajectory.positionAt(t);
    const double distance = _esdf.distanceAt(position.x(), position.y());
    if (distance < best) {
      best = distance;
      _where = position;
      _when = t;
    }
  }
  return best;
}

/// \brief 直线初值：4 段、每段 0.5 s，可选整体时间缩放。
TrajectoryInitialGuess makeStraightGuess(double _duration_scale)
{
  TrajectoryInitialGuess guess;
  guess.waypoints = {{0.0, 0.0}, {0.75, 0.0}, {1.5, 0.0}, {2.25, 0.0}, {3.0, 0.0}};
  const double duration = 0.5 * _duration_scale;
  guess.durations = {duration, duration, duration, duration};
  guess.head_position = {0.0, 0.0};
  guess.head_velocity = Eigen::Vector2d::Zero();
  guess.head_acceleration = Eigen::Vector2d::Zero();
  guess.tail_position = {3.0, 0.0};
  guess.tail_velocity = Eigen::Vector2d::Zero();
  guess.tail_acceleration = Eigen::Vector2d::Zero();
  return guess;
}

/// \brief 无障碍直线优化使用的配置。
MincoOptimizerConfig makeLineConfig()
{
  MincoOptimizerConfig config;
  config.w_jerk = 1.0;
  config.w_time = 10.0;
  config.w_obstacle = 100.0;
  config.w_velocity = 20.0;
  config.w_acceleration = 2.0;
  config.w_reference = 0.0;
  config.w_time_ratio = 1.0;
  config.robot_radius = kRobotRadius;
  config.clearance_margin = kClearanceMargin;
  config.max_iterations = 300;
  return config;
}

/// \brief 断言轨迹首末 p/v/a 与初值边界一致。
void expectBoundaryMatch(
  const Trajectory2D & _trajectory, const TrajectoryInitialGuess & _guess, double _tolerance)
{
  EXPECT_NEAR((_trajectory.startPosition() - _guess.head_position).norm(), 0.0, _tolerance);
  EXPECT_NEAR((_trajectory.startVelocity() - _guess.head_velocity).norm(), 0.0, _tolerance);
  EXPECT_NEAR((_trajectory.startAcceleration() - _guess.head_acceleration).norm(), 0.0, _tolerance);
  EXPECT_NEAR((_trajectory.endPosition() - _guess.tail_position).norm(), 0.0, _tolerance);
  EXPECT_NEAR((_trajectory.endVelocity() - _guess.tail_velocity).norm(), 0.0, _tolerance);
  EXPECT_NEAR((_trajectory.endAcceleration() - _guess.tail_acceleration).norm(), 0.0, _tolerance);
}

}  // namespace

// ---------------------------------------------------------------------------
// 1) configure 参数校验
// ---------------------------------------------------------------------------

TEST(MincoOptimizerConfigure, RejectsNonFiniteAndNegativeWeights)
{
  std::string reason;
  MincoOptimizer optimizer;

  MincoOptimizerConfig negative = makeGradientConfig();
  negative.w_obstacle = -1.0;
  reason.clear();
  EXPECT_FALSE(optimizer.configure(negative, &reason));
  EXPECT_FALSE(reason.empty());

  MincoOptimizerConfig nan_weight = makeGradientConfig();
  nan_weight.w_time = std::numeric_limits<double>::quiet_NaN();
  reason.clear();
  EXPECT_FALSE(optimizer.configure(nan_weight, &reason));
  EXPECT_FALSE(reason.empty());

  // 所有主项权重都为 0 时目标函数没有意义，必须拒绝。
  MincoOptimizerConfig all_zero = makeGradientConfig();
  all_zero.w_jerk = 0.0;
  all_zero.w_time = 0.0;
  all_zero.w_velocity = 0.0;
  all_zero.w_acceleration = 0.0;
  reason.clear();
  EXPECT_FALSE(optimizer.configure(all_zero, &reason));
  EXPECT_FALSE(reason.empty());

  // configure 失败后不能留下“已配置”状态。
  MincoOptimizeResult result;
  EXPECT_FALSE(optimizer.optimize(makeStraightGuess(1.0), Limits2D(), result));
  EXPECT_EQ(result.status, SolveStatus::kNotInitialized);
}

TEST(MincoOptimizerConfigure, RejectsInvalidSamplingAndDurationBounds)
{
  std::string reason;
  MincoOptimizer optimizer;

  MincoOptimizerConfig few_samples = makeGradientConfig();
  few_samples.samples_per_piece = 1;
  reason.clear();
  EXPECT_FALSE(optimizer.configure(few_samples, &reason));
  EXPECT_FALSE(reason.empty());

  MincoOptimizerConfig zero_min_duration = makeGradientConfig();
  zero_min_duration.min_piece_duration = 0.0;
  reason.clear();
  EXPECT_FALSE(optimizer.configure(zero_min_duration, &reason));
  EXPECT_FALSE(reason.empty());

  MincoOptimizerConfig inverted_bounds = makeGradientConfig();
  inverted_bounds.min_piece_duration = 0.5;
  inverted_bounds.max_piece_duration = 0.4;
  reason.clear();
  EXPECT_FALSE(optimizer.configure(inverted_bounds, &reason));
  EXPECT_FALSE(reason.empty());

  MincoOptimizerConfig bad_beta = makeGradientConfig();
  bad_beta.soft_hinge_beta = 0.0;
  reason.clear();
  EXPECT_FALSE(optimizer.configure(bad_beta, &reason));
  EXPECT_FALSE(reason.empty());

  MincoOptimizerConfig bad_memory = makeGradientConfig();
  bad_memory.lbfgs_memory = 2;
  reason.clear();
  EXPECT_FALSE(optimizer.configure(bad_memory, &reason));
  EXPECT_FALSE(reason.empty());

  MincoOptimizerConfig bad_iterations = makeGradientConfig();
  bad_iterations.max_iterations = 0;
  reason.clear();
  EXPECT_FALSE(optimizer.configure(bad_iterations, &reason));
  EXPECT_FALSE(reason.empty());
}

TEST(MincoOptimizerConfigure, RejectsInvalidGeometryAndAcceptsValidConfig)
{
  std::string reason;
  MincoOptimizer optimizer;

  MincoOptimizerConfig zero_radius = makeGradientConfig();
  zero_radius.robot_radius = 0.0;
  reason.clear();
  EXPECT_FALSE(optimizer.configure(zero_radius, &reason));
  EXPECT_FALSE(reason.empty());

  MincoOptimizerConfig negative_radius = makeGradientConfig();
  negative_radius.robot_radius = -0.33;
  reason.clear();
  EXPECT_FALSE(optimizer.configure(negative_radius, &reason));
  EXPECT_FALSE(reason.empty());

  MincoOptimizerConfig negative_margin = makeGradientConfig();
  negative_margin.clearance_margin = -0.01;
  reason.clear();
  EXPECT_FALSE(optimizer.configure(negative_margin, &reason));
  EXPECT_FALSE(reason.empty());

  MincoOptimizerConfig bad_ratio = makeGradientConfig();
  bad_ratio.pre_time_ratio_min = 1.2;
  bad_ratio.pre_time_ratio_max = 0.9;
  reason.clear();
  EXPECT_FALSE(optimizer.configure(bad_ratio, &reason));
  EXPECT_FALSE(reason.empty());

  MincoOptimizerConfig bad_probe = makeGradientConfig();
  bad_probe.fine_probe_step = 0.0;
  reason.clear();
  EXPECT_FALSE(optimizer.configure(bad_probe, &reason));
  EXPECT_FALSE(reason.empty());

  // 合法配置必须通过，并且此时不再写失败原因。
  MincoOptimizerConfig valid = makeGradientConfig();
  reason = "untouched";
  EXPECT_TRUE(optimizer.configure(valid, &reason));
  EXPECT_EQ(reason, "untouched");
}

// ---------------------------------------------------------------------------
// 2) 解析梯度 vs 中心差分
// ---------------------------------------------------------------------------

TEST(MincoOptimizerGradient, MatchesCentralDifferenceWithoutEsdf)
{
  MincoOptimizer optimizer;
  MincoOptimizerConfig config = makeGradientConfig();
  std::string reason;
  ASSERT_TRUE(optimizer.configure(config, &reason)) << reason;
  optimizer.setEsdf(nullptr);

  const TrajectoryInitialGuess guess = makeGradientGuess();
  const Limits2D limits = makeGradientLimits();
  const GradientCheck check = runGradientCheck(
    optimizer, guess, limits, config.min_piece_duration, kPositionEpsilon, kTimeEpsilon);

  EXPECT_NEAR(check.round_trip_error, 0.0, 1.0e-12);
  // 位置分量实测相对误差 ~1e-11 ~ 1e-9，这里留 3 个数量级余量。
  EXPECT_LT(check.max_point_error, 1.0e-6) << "最大位置分量相对误差 " << check.max_point_error;
  // 时间分量经过 softplus 链式映射（dT/ds = sigmoid(s) 可能很小，条件数变差），
  // 实测 ~1e-8，按方案 §11.1 放宽到 1e-4。
  EXPECT_LT(check.max_time_error, 1.0e-4) << "最大时间分量相对误差 " << check.max_time_error;
  RecordProperty("max_point_relative_error", formatMetric(check.max_point_error));
  RecordProperty("max_time_relative_error", formatMetric(check.max_time_error));
}

TEST(MincoOptimizerGradient, MatchesCentralDifferenceWithEsdfSoftCost)
{
  MincoOptimizer optimizer;
  MincoOptimizerConfig config = makeGradientConfig();
  config.w_obstacle = 100.0;
  config.robot_radius = kRobotRadius;
  config.clearance_margin = kClearanceMargin;
  std::string reason;
  ASSERT_TRUE(optimizer.configure(config, &reason)) << reason;

  // 轨迹下方放一堵横墙（墙顶在 y = -0.30，轨迹在 y ≈ -0.16 ~ 0.25）：
  // 轨迹离障碍较近、软约束显著激活，但采样点仍在自由区内、离墙面有若干厘米，
  // 既不在 hinge 的折点上，也不跨越栅格边界，因此双线性距离场在采样点附近光滑。
  std::vector<CellState> cells = makeFreeCells(kMapSizeX, kMapSizeY);
  fillRectangle(
    cells, kMapSizeX, kMapSizeY, kMapOriginX, kMapOriginY, kMapResolution, -1.5, -0.60, 4.5, -0.30,
    CellState::kOccupied);
  const std::shared_ptr<Esdf2D> esdf =
    makeEsdf(cells, kMapSizeX, kMapSizeY, kMapOriginX, kMapOriginY, kMapResolution);
  ASSERT_NE(esdf, nullptr);
  ASSERT_TRUE(esdf->valid());
  ASSERT_TRUE(esdf->hasSeed());
  optimizer.setEsdf(esdf);

  const TrajectoryInitialGuess guess = makeGradientGuess();
  const Limits2D limits = makeGradientLimits();

  // 先把“场景前提”断言清楚：轨迹在自由区内，但净空小于安全距离，障碍项确实激活。
  Trajectory2D initial_trajectory;
  ASSERT_TRUE(TrajectoryInitializer::buildInitialTrajectory(guess, initial_trajectory));
  Eigen::Vector2d nearest;
  double nearest_time = 0.0;
  const double clearance =
    minClearanceAlong(*esdf, initial_trajectory, 0.002, nearest, nearest_time);
  const double safety_distance = kRobotRadius + kClearanceMargin;
  EXPECT_GT(clearance, 0.05) << "采样点必须仍在自由区内部，实测最小净空 " << clearance;
  EXPECT_LT(clearance, safety_distance)
    << "障碍软约束必须处于激活状态，实测最小净空 " << clearance << " 安全距离 " << safety_distance;
  RecordProperty("initial_min_clearance", formatSeconds(clearance));

  const GradientCheck check = runGradientCheck(
    optimizer, guess, limits, config.min_piece_duration, kPositionEpsilon, kTimeEpsilon);

  EXPECT_NEAR(check.round_trip_error, 0.0, 1.0e-12);
  EXPECT_LT(check.max_point_error, 1.0e-5) << "最大位置分量相对误差 " << check.max_point_error;
  EXPECT_LT(check.max_time_error, 1.0e-4) << "最大时间分量相对误差 " << check.max_time_error;
  RecordProperty("max_point_relative_error", formatMetric(check.max_point_error));
  RecordProperty("max_time_relative_error", formatMetric(check.max_time_error));

  // 障碍软约束非负：带 ESDF 的代价一定不小于不带 ESDF 的代价。
  MincoOptimizer plain;
  MincoOptimizerConfig plain_config = makeGradientConfig();
  ASSERT_TRUE(plain.configure(plain_config, &reason)) << reason;
  double cost_with_obstacle = 0.0;
  double cost_without_obstacle = 0.0;
  Eigen::VectorXd gradient;
  ASSERT_TRUE(optimizer.evaluateObjective(guess, limits, cost_with_obstacle, gradient));
  ASSERT_TRUE(plain.evaluateObjective(guess, limits, cost_without_obstacle, gradient));
  EXPECT_GT(cost_with_obstacle, cost_without_obstacle)
    << "障碍软约束必须带来正的代价（安全距离 " << safety_distance << " m）";
}

TEST(MincoOptimizerGradient, MatchesCentralDifferenceWithTimeRatioPenaltyDisabled)
{
  // 关闭两阶段（two_stage = false）且 w_time_ratio = 0：目标里只剩 jerk 能量、
  // 总时长、速度与加速度软约束，梯度必须与差分一致。
  MincoOptimizer optimizer;
  MincoOptimizerConfig config = makeGradientConfig();
  config.two_stage = false;
  config.w_time_ratio = 0.0;
  std::string reason;
  ASSERT_TRUE(optimizer.configure(config, &reason)) << reason;
  optimizer.setEsdf(nullptr);

  const TrajectoryInitialGuess guess = makeGradientGuess();
  const Limits2D limits = makeGradientLimits();
  const GradientCheck check = runGradientCheck(
    optimizer, guess, limits, config.min_piece_duration, kPositionEpsilon, kTimeEpsilon);
  EXPECT_LT(check.max_point_error, 1.0e-6) << "最大位置分量相对误差 " << check.max_point_error;
  EXPECT_LT(check.max_time_error, 1.0e-4) << "最大时间分量相对误差 " << check.max_time_error;
  RecordProperty("max_point_relative_error", formatMetric(check.max_point_error));
  RecordProperty("max_time_relative_error", formatMetric(check.max_time_error));

  // 打开段时长比例惩罚（w_time_ratio > 0）：mean(T) 参与求导，必须把均值的
  // 导数一起带上，否则时间梯度会出现与 T 无关的系统性偏差。
  MincoOptimizerConfig ratio_config = config;
  ratio_config.w_time_ratio = 1.0;
  MincoOptimizer optimizer_with_ratio;
  ASSERT_TRUE(optimizer_with_ratio.configure(ratio_config, &reason)) << reason;
  const GradientCheck ratio_check = runGradientCheck(
    optimizer_with_ratio, guess, limits, ratio_config.min_piece_duration, kPositionEpsilon,
    kTimeEpsilon);
  EXPECT_LT(ratio_check.max_point_error, 1.0e-6);
  EXPECT_LT(ratio_check.max_time_error, 1.0e-4)
    << "最大时间分量相对误差 " << ratio_check.max_time_error;
  RecordProperty("ratio_max_point_relative_error", formatMetric(ratio_check.max_point_error));
  RecordProperty("ratio_max_time_relative_error", formatMetric(ratio_check.max_time_error));
}

TEST(MincoOptimizerGradient, MatchesCentralDifferenceWithReferenceWeight)
{
  // w_reference > 0：参考吸引项只作用于内部路标点，代价与梯度都要计入。
  MincoOptimizer optimizer;
  MincoOptimizerConfig config = makeGradientConfig();
  config.w_reference = 5.0;
  std::string reason;
  ASSERT_TRUE(optimizer.configure(config, &reason)) << reason;
  optimizer.setEsdf(nullptr);

  TrajectoryInitialGuess guess = makeGradientGuess();
  // 参考点与初值路标点故意错开，保证该项处于激活状态。
  guess.waypoints[1] = Eigen::Vector2d(0.8, 0.5);
  guess.waypoints[2] = Eigen::Vector2d(2.3, 0.4);

  const Limits2D limits = makeGradientLimits();
  const GradientCheck check = runGradientCheck(
    optimizer, guess, limits, config.min_piece_duration, kPositionEpsilon, kTimeEpsilon);
  EXPECT_LT(check.max_point_error, 1.0e-6) << "最大位置分量相对误差 " << check.max_point_error;
  EXPECT_LT(check.max_time_error, 1.0e-4);
  RecordProperty("max_point_relative_error", formatMetric(check.max_point_error));
}

TEST(MincoOptimizerGradient, FineHeuristicKeepsCostButBreaksGradientConsistency)
{
  // enable_report_fine_heuristic = true 时，`applyFineHeuristic()` 会按探测斜率
  // 缩放障碍梯度：**代价不变、梯度被人为改写**，因此该模式下代价与梯度不再是
  // 严格的导数关系（方案 §6.5 明确列为启发式分支）。
  // 本用例只断言：函数仍能返回有限值、不崩溃；并在启发式确实生效时，明确断言
  // “梯度与差分不一致”是预期行为，而不是把它当成需要修复的错误。
  MincoOptimizerConfig config = makeGradientConfig();
  config.w_obstacle = 100.0;
  config.enable_report_fine_heuristic = true;
  config.fine_slope_threshold = 0.5;
  config.fine_probe_step = 0.10;
  config.robot_radius = kRobotRadius;
  config.clearance_margin = kClearanceMargin;

  std::vector<CellState> cells = makeFreeCells(kMapSizeX, kMapSizeY);
  // 一堵正对轨迹前进方向的短墙：切向与距离场梯度接近平行，
  // 探测方向上的距离改善很慢，启发式的 gain < 1 会真正生效。
  fillRectangle(
    cells, kMapSizeX, kMapSizeY, kMapOriginX, kMapOriginY, kMapResolution, 1.40, -0.60, 1.55, 0.60,
    CellState::kOccupied);
  const std::shared_ptr<Esdf2D> esdf =
    makeEsdf(cells, kMapSizeX, kMapSizeY, kMapOriginX, kMapOriginY, kMapResolution);
  ASSERT_NE(esdf, nullptr);

  MincoOptimizer heuristic;
  std::string reason;
  ASSERT_TRUE(heuristic.configure(config, &reason)) << reason;
  heuristic.setEsdf(esdf);

  MincoOptimizer exact;
  MincoOptimizerConfig exact_config = config;
  exact_config.enable_report_fine_heuristic = false;
  ASSERT_TRUE(exact.configure(exact_config, &reason)) << reason;
  exact.setEsdf(esdf);

  const TrajectoryInitialGuess guess = makeGradientGuess();
  const Limits2D limits = makeGradientLimits();

  double heuristic_cost = 0.0;
  double exact_cost = 0.0;
  Eigen::VectorXd heuristic_gradient;
  Eigen::VectorXd exact_gradient;
  ASSERT_TRUE(heuristic.evaluateObjective(guess, limits, heuristic_cost, heuristic_gradient));
  ASSERT_TRUE(exact.evaluateObjective(guess, limits, exact_cost, exact_gradient));

  EXPECT_TRUE(std::isfinite(heuristic_cost));
  EXPECT_TRUE(heuristic_gradient.allFinite());

  const GradientCheck check = runGradientCheck(
    heuristic, guess, limits, config.min_piece_duration, kPositionEpsilon, kTimeEpsilon);
  EXPECT_TRUE(std::isfinite(check.analytic.norm()));
  EXPECT_TRUE(check.analytic.allFinite());
  EXPECT_TRUE(check.numeric.allFinite());

  const bool heuristic_active = (heuristic_gradient - exact_gradient).norm() > 1.0e-9;
  if (heuristic_active) {
    // 启发式生效：代价与梯度不一致是设计如此，差分比较必然出现偏差。
    EXPECT_GT(check.max_point_error, 1.0e-6)
      << "启发式生效时，梯度被改写，差分不应再与解析梯度一致";
    RecordProperty("fine_heuristic_max_point_error", formatMetric(check.max_point_error));
  } else {
    // 该布置下 gain 恰好为 1，退化成与严格梯度一致，同样不算失败。
    EXPECT_LT(check.max_point_error, 1.0e-6);
    RecordProperty("fine_heuristic_max_point_error", "inactive");
  }
  // 无论启发式是否生效，代价本身不受影响（启发式只改梯度方向/尺度）。
  EXPECT_NEAR(heuristic_cost, exact_cost, 1.0e-12);

  // 对照实验：同一个窗口、同一个两阶段流程，关闭启发式时必须稳定成功。
  MincoOptimizerConfig two_stage_config = config;
  two_stage_config.enable_report_fine_heuristic = false;
  two_stage_config.two_stage = true;
  two_stage_config.max_iterations = 50;
  MincoOptimizer plain_runner;
  ASSERT_TRUE(plain_runner.configure(two_stage_config, &reason)) << reason;
  plain_runner.setEsdf(esdf);
  MincoOptimizeResult plain_result;
  ASSERT_TRUE(plain_runner.optimize(guess, limits, plain_result)) << plain_result.message;
  EXPECT_EQ(plain_result.status, SolveStatus::kSuccess);

  // 打开启发式后不保证优化成功：启发式按探测斜率缩放障碍梯度，方向不再等于
  // 真实下降方向，FINELY 阶段可能在第一次线搜索就报“梯度无法下降”。
  // 这里不断言“必须成功”，只断言：要么成功且结果有限，要么明确失败并给出
  // 原因，绝不出现崩溃或半成品结果——这正是“启发式需要单独评估”的含义。
  MincoOptimizerConfig fine_config = config;
  fine_config.two_stage = true;
  fine_config.max_iterations = 50;
  MincoOptimizer fine_runner;
  ASSERT_TRUE(fine_runner.configure(fine_config, &reason)) << reason;
  fine_runner.setEsdf(esdf);
  MincoOptimizeResult fine_result;
  const bool fine_optimized = fine_runner.optimize(guess, limits, fine_result);
  if (fine_optimized) {
    EXPECT_EQ(fine_result.status, SolveStatus::kSuccess);
    EXPECT_TRUE(std::isfinite(fine_result.cost));
    EXPECT_TRUE(fine_result.trajectory.pieceCount() > 0);
  } else {
    EXPECT_NE(fine_result.status, SolveStatus::kSuccess);
    EXPECT_FALSE(fine_result.message.empty());
    EXPECT_TRUE(fine_result.trajectory.empty());
  }
  RecordProperty("fine_optimize_succeeded", fine_optimized ? "true" : "false");
  RecordProperty(
    "fine_optimize_message", fine_result.message.empty() ? "(none)" : fine_result.message);
}

// ---------------------------------------------------------------------------
// 3) optimize
// ---------------------------------------------------------------------------

TEST(MincoOptimizerOptimize, StraightLineKeepsBoundaryAndContinuity)
{
  MincoOptimizer optimizer;
  MincoOptimizerConfig config = makeLineConfig();
  std::string reason;
  ASSERT_TRUE(optimizer.configure(config, &reason)) << reason;
  optimizer.setEsdf(nullptr);

  const TrajectoryInitialGuess guess = makeStraightGuess(1.0);
  Limits2D limits;
  limits.max_linear_speed = 0.8;
  limits.max_linear_accel = 0.5;

  MincoOptimizeResult result;
  ASSERT_TRUE(optimizer.optimize(guess, limits, result)) << result.message;
  EXPECT_EQ(result.status, SolveStatus::kSuccess);
  EXPECT_GT(result.trajectory.pieceCount(), 0);
  EXPECT_EQ(result.trajectory.pieceCount(), guess.pieceCount());
  EXPECT_GE(result.iterations, 1);

  std::string sanity_reason;
  EXPECT_TRUE(result.trajectory.sanityCheck(&sanity_reason)) << sanity_reason;
  expectBoundaryMatch(result.trajectory, guess, 1.0e-6);

  const ContinuityResiduals residuals = continuityOf(result.trajectory);
  EXPECT_LT(residuals.position, 1.0e-8) << "位置拼接残差";
  EXPECT_LT(residuals.velocity, 1.0e-8) << "速度拼接残差";
  EXPECT_LT(residuals.acceleration, 1.0e-8) << "加速度拼接残差";

  for (const double duration : result.trajectory.durations()) {
    EXPECT_GT(duration, 0.0) << "softplus 参数化必须保证段时长严格为正";
    EXPECT_GT(duration, config.min_piece_duration);
  }
  EXPECT_TRUE(result.time_optimized);
  RecordProperty("total_duration", formatSeconds(result.trajectory.totalDuration()));
  RecordProperty("iterations", std::to_string(result.iterations));
}

TEST(MincoOptimizerOptimize, KeepsNonZeroBoundaryVelocities)
{
  MincoOptimizer optimizer;
  MincoOptimizerConfig config = makeLineConfig();
  std::string reason;
  ASSERT_TRUE(optimizer.configure(config, &reason)) << reason;
  optimizer.setEsdf(nullptr);

  TrajectoryInitialGuess guess = makeStraightGuess(1.0);
  guess.head_velocity = {0.25, 0.10};
  guess.tail_velocity = {-0.05, 0.02};
  guess.head_acceleration = {0.0, 0.05};
  guess.tail_acceleration = {0.0, -0.05};
  Limits2D limits;
  limits.max_linear_speed = 0.8;
  limits.max_linear_accel = 0.5;

  MincoOptimizeResult result;
  ASSERT_TRUE(optimizer.optimize(guess, limits, result)) << result.message;
  ASSERT_EQ(result.status, SolveStatus::kSuccess);
  expectBoundaryMatch(result.trajectory, guess, 1.0e-6);

  // 边界速度确实被保留（不是被优化成 0）。
  EXPECT_NEAR(result.trajectory.startVelocity().x(), 0.25, 1.0e-6);
  EXPECT_NEAR(result.trajectory.endVelocity().x(), -0.05, 1.0e-6);
  const ContinuityResiduals residuals = continuityOf(result.trajectory);
  EXPECT_LT(residuals.velocity, 1.0e-8);
}

TEST(MincoOptimizerOptimize, TimeIsOptimizedShorterFromInflatedDurations)
{
  // 时间项要真正起作用：w_time 相对 w_jerk 足够大、且速度/加速度上限不成为
  // 主要瓶颈时，最优总时长由 w_time 与 jerk 能量的平衡决定（约 2.6 s）。
  MincoOptimizerConfig config = makeLineConfig();
  config.w_time = 100.0;
  config.max_iterations = 300;
  MincoOptimizer optimizer;
  std::string reason;
  ASSERT_TRUE(optimizer.configure(config, &reason)) << reason;
  optimizer.setEsdf(nullptr);

  Limits2D limits;
  limits.max_linear_speed = 3.0;
  limits.max_linear_accel = 5.0;

  const TrajectoryInitialGuess nominal = makeStraightGuess(1.0);
  const TrajectoryInitialGuess inflated = makeStraightGuess(3.0);
  const double inflated_duration =
    std::accumulate(inflated.durations.begin(), inflated.durations.end(), 0.0);

  MincoOptimizeResult nominal_result;
  MincoOptimizeResult inflated_result;
  ASSERT_TRUE(optimizer.optimize(nominal, limits, nominal_result)) << nominal_result.message;
  ASSERT_TRUE(optimizer.optimize(inflated, limits, inflated_result)) << inflated_result.message;
  ASSERT_EQ(inflated_result.status, SolveStatus::kSuccess);

  const double optimized_inflated = inflated_result.trajectory.totalDuration();
  const double optimized_nominal = nominal_result.trajectory.totalDuration();
  EXPECT_LT(optimized_inflated, 0.6 * inflated_duration)
    << "初值时长放大 3 倍（" << inflated_duration << " s）后，优化后的总时长必须明显缩短，实测 "
    << optimized_inflated << " s";
  // 时间项在起作用的最直接证据：两个相差 3 倍的初值都收敛到同一个时间量级。
  EXPECT_NEAR(optimized_inflated, optimized_nominal, 0.25 * optimized_nominal)
    << "优化后的总时长 " << optimized_inflated << " s 应与常规初值的 " << optimized_nominal
    << " s 同量级";
  for (const double duration : inflated_result.trajectory.durations()) {
    EXPECT_GT(duration, config.min_piece_duration);
  }
  RecordProperty("inflated_initial_duration", formatSeconds(inflated_duration));
  RecordProperty("inflated_optimized_duration", formatSeconds(optimized_inflated));
  RecordProperty("nominal_optimized_duration", formatSeconds(optimized_nominal));
}

TEST(MincoOptimizerOptimize, WallObstacleImprovesMinimumClearance)
{
  MincoOptimizer optimizer;
  MincoOptimizerConfig config = makeLineConfig();
  config.max_iterations = 400;
  std::string reason;
  ASSERT_TRUE(optimizer.configure(config, &reason)) << reason;

  // 一堵立在直线路径上的矮墙（x ∈ [1.35, 1.75]，y ∈ [-1.0, 0.15]）：
  // 初值直线在 y = 0 处正好从墙内部穿过（净空为负），优化后应当从墙顶绕过去。
  // 墙必须足够厚：ESDF 对障碍内部给的是“到最近自由格”的穿透深度，太薄的墙
  // 内部距离场是平的（梯度 ≈ 0），初值又正好落在墙的中线上，软约束会失去推力。
  std::vector<CellState> cells = makeFreeCells(kMapSizeX, kMapSizeY);
  fillRectangle(
    cells, kMapSizeX, kMapSizeY, kMapOriginX, kMapOriginY, kMapResolution, 1.35, -1.00, 1.75, 0.15,
    CellState::kOccupied);
  const std::shared_ptr<Esdf2D> esdf =
    makeEsdf(cells, kMapSizeX, kMapSizeY, kMapOriginX, kMapOriginY, kMapResolution);
  ASSERT_NE(esdf, nullptr);
  optimizer.setEsdf(esdf);

  const TrajectoryInitialGuess guess = makeStraightGuess(1.0);
  Limits2D limits;
  limits.max_linear_speed = 0.8;
  limits.max_linear_accel = 0.5;

  // 初值轨迹（未优化）的最小净空，用同一套采样方式对比。
  Trajectory2D initial_trajectory;
  ASSERT_TRUE(TrajectoryInitializer::buildInitialTrajectory(guess, initial_trajectory));
  Eigen::Vector2d initial_where;
  double initial_when = 0.0;
  const double initial_clearance =
    minClearanceAlong(*esdf, initial_trajectory, 0.002, initial_where, initial_when);
  EXPECT_LT(initial_clearance, kRobotRadius) << "初值直线应当与墙体重叠（净空小于包络半径）";

  MincoOptimizeResult result;
  ASSERT_TRUE(optimizer.optimize(guess, limits, result)) << result.message;
  ASSERT_EQ(result.status, SolveStatus::kSuccess);

  Eigen::Vector2d optimized_where;
  double optimized_when = 0.0;
  const double optimized_clearance =
    minClearanceAlong(*esdf, result.trajectory, 0.002, optimized_where, optimized_when);

  // 优化器不保证可行，因此这里断言的是“显著改善”，同时把实测值记录出来。
  EXPECT_GT(optimized_clearance, initial_clearance)
    << "优化后最小净空 " << optimized_clearance << " 必须大于优化前 " << initial_clearance;
  // 本布置下墙顶绕行是可行解，因此进一步要求优化后净空为正且留有余量：
  // 实测约 0.47 m（安全距离 robot_radius + clearance_margin = 0.38 m）。
  EXPECT_GE(optimized_clearance, 0.20)
    << "可行布置下优化后最小净空应为正且有余量，实测 " << optimized_clearance;
  EXPECT_GT(optimized_clearance, kRobotRadius)
    << "优化后轨迹不应再与墙体包络重叠，实测 " << optimized_clearance;
  RecordProperty("initial_min_clearance", formatSeconds(initial_clearance));
  RecordProperty("optimized_min_clearance", formatSeconds(optimized_clearance));
  RecordProperty("optimized_min_clearance_x", formatSeconds(optimized_where.x()));
  RecordProperty("optimized_min_clearance_y", formatSeconds(optimized_where.y()));
  RecordProperty("iterations", std::to_string(result.iterations));

  for (const double duration : result.trajectory.durations()) {
    EXPECT_GT(duration, 0.0);
  }
  std::string sanity_reason;
  EXPECT_TRUE(result.trajectory.sanityCheck(&sanity_reason)) << sanity_reason;
}

TEST(MincoOptimizerOptimize, RejectsInvalidInputs)
{
  MincoOptimizer optimizer;
  MincoOptimizerConfig config = makeLineConfig();
  std::string reason;
  ASSERT_TRUE(optimizer.configure(config, &reason)) << reason;
  optimizer.setEsdf(nullptr);

  Limits2D limits;
  limits.max_linear_speed = 0.8;
  limits.max_linear_accel = 0.5;

  const TrajectoryInitialGuess valid = makeStraightGuess(1.0);

  // 非法约束（速度为 0 不满足 Limits2D::valid()）。
  Limits2D bad_limits = limits;
  bad_limits.max_linear_speed = 0.0;
  MincoOptimizeResult result;
  EXPECT_FALSE(optimizer.optimize(valid, bad_limits, result));
  EXPECT_EQ(result.status, SolveStatus::kInvalidInput);
  EXPECT_FALSE(result.message.empty());

  // 路标点数量与段数不一致。
  TrajectoryInitialGuess mismatched = valid;
  mismatched.waypoints.pop_back();
  EXPECT_FALSE(optimizer.optimize(mismatched, limits, result));
  EXPECT_EQ(result.status, SolveStatus::kInvalidInput);

  // 路标点数量多一个同样非法。
  TrajectoryInitialGuess extra = valid;
  extra.waypoints.push_back(Eigen::Vector2d(4.0, 0.0));
  EXPECT_FALSE(optimizer.optimize(extra, limits, result));
  EXPECT_EQ(result.status, SolveStatus::kInvalidInput);

  // 段数为 0。
  TrajectoryInitialGuess empty;
  empty.waypoints = {{0.0, 0.0}};
  EXPECT_FALSE(optimizer.optimize(empty, limits, result));
  EXPECT_EQ(result.status, SolveStatus::kInvalidInput);

  // 边界状态含 NaN。
  TrajectoryInitialGuess nan_head = valid;
  nan_head.head_position = Eigen::Vector2d(std::numeric_limits<double>::quiet_NaN(), 0.0);
  EXPECT_FALSE(optimizer.optimize(nan_head, limits, result));
  EXPECT_EQ(result.status, SolveStatus::kInvalidInput);

  TrajectoryInitialGuess nan_tail = valid;
  nan_tail.tail_velocity = Eigen::Vector2d(0.0, std::numeric_limits<double>::quiet_NaN());
  EXPECT_FALSE(optimizer.optimize(nan_tail, limits, result));
  EXPECT_EQ(result.status, SolveStatus::kInvalidInput);

  // 段时长非有限：encodeVariables 的下限保护会把 NaN 静默换成 Tmin，必须提前拒绝。
  TrajectoryInitialGuess nan_duration = valid;
  nan_duration.durations[2] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(optimizer.optimize(nan_duration, limits, result));
  EXPECT_EQ(result.status, SolveStatus::kInvalidInput);

  TrajectoryInitialGuess inf_duration = valid;
  inf_duration.durations[0] = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(optimizer.optimize(inf_duration, limits, result));
  EXPECT_EQ(result.status, SolveStatus::kInvalidInput);

  // 合法输入仍然可以求解（确认前面的失败没有污染内部状态）。
  EXPECT_TRUE(optimizer.optimize(valid, limits, result)) << result.message;
  EXPECT_EQ(result.status, SolveStatus::kSuccess);
}

// ---------------------------------------------------------------------------
// 4) 热启动
// ---------------------------------------------------------------------------

TEST(MincoOptimizerWarmStart, ValidatesLengthsFinitenessAndDurations)
{
  MincoOptimizer optimizer;
  MincoOptimizerConfig config = makeLineConfig();
  std::string reason;
  ASSERT_TRUE(optimizer.configure(config, &reason)) << reason;

  const std::vector<Eigen::Vector2d> inner = {{0.75, 0.0}, {1.5, 0.0}, {2.25, 0.0}};
  const std::vector<double> durations = {0.5, 0.5, 0.5, 0.5};

  // 长度不匹配：内部路标点数量必须是段数 - 1（= 时长个数 - 1）。
  EXPECT_FALSE(optimizer.setWarmStart(inner, {0.5, 0.5}));
  EXPECT_FALSE(optimizer.setWarmStart(inner, {0.5, 0.5, 0.5}));
  EXPECT_FALSE(optimizer.setWarmStart(inner, {0.5, 0.5, 0.5, 0.5, 0.5}));
  EXPECT_FALSE(optimizer.setWarmStart({{0.5, 0.0}, {1.5, 0.0}}, {0.5, 0.5}));
  // 完全空的热启动同样非法（段数必须 >= 1）。
  EXPECT_FALSE(optimizer.setWarmStart({}, {}));
  // 含 NaN 的路标点。
  std::vector<Eigen::Vector2d> nan_inner = inner;
  nan_inner[1] = Eigen::Vector2d(std::numeric_limits<double>::quiet_NaN(), 0.0);
  EXPECT_FALSE(optimizer.setWarmStart(nan_inner, durations));
  // 时长不大于 min_piece_duration。
  EXPECT_FALSE(optimizer.setWarmStart(inner, {0.5, 0.5, 0.5, kMinPieceDuration}));
  EXPECT_FALSE(optimizer.setWarmStart(inner, {0.5, 0.5, 0.5, 0.01}));
  // 时长含 NaN。
  EXPECT_FALSE(
    optimizer.setWarmStart(inner, {0.5, 0.5, 0.5, std::numeric_limits<double>::quiet_NaN()}));

  // 合法热启动。
  EXPECT_TRUE(optimizer.setWarmStart(inner, durations));
}

TEST(MincoOptimizerWarmStart, OptimizeSucceedsWithWarmStart)
{
  MincoOptimizer optimizer;
  MincoOptimizerConfig config = makeLineConfig();
  std::string reason;
  ASSERT_TRUE(optimizer.configure(config, &reason)) << reason;
  optimizer.setEsdf(nullptr);

  // 4 段需要 3 个内部路标点，热启动给出一组“已经不错”的路标点与时长。
  const std::vector<Eigen::Vector2d> inner = {{0.75, 0.02}, {1.50, -0.02}, {2.25, 0.01}};
  const std::vector<double> durations = {0.45, 0.45, 0.45, 0.45};
  ASSERT_TRUE(optimizer.setWarmStart(inner, durations));

  const TrajectoryInitialGuess guess = makeStraightGuess(1.0);
  Limits2D limits;
  limits.max_linear_speed = 0.8;
  limits.max_linear_accel = 0.5;

  MincoOptimizeResult result;
  ASSERT_TRUE(optimizer.optimize(guess, limits, result)) << result.message;
  EXPECT_EQ(result.status, SolveStatus::kSuccess);
  expectBoundaryMatch(result.trajectory, guess, 1.0e-6);
  // 热启动只替换内部路标点与时长，边界状态必须仍然来自初值。
  const ContinuityResiduals residuals = continuityOf(result.trajectory);
  EXPECT_LT(residuals.position, 1.0e-8);

  // 证明热启动确实被用作迭代起点：把热启动内容当成完整初值再解一次，
  // 两者必须给出**完全相同**的结果（同一个起点、同一个确定性 L-BFGS 流程）。
  TrajectoryInitialGuess warm_guess = guess;
  for (std::size_t i = 0; i < inner.size(); ++i) {
    warm_guess.waypoints[i + 1] = inner[i];
  }
  warm_guess.durations = durations;
  MincoOptimizer cold;
  ASSERT_TRUE(cold.configure(config, &reason)) << reason;
  cold.setEsdf(nullptr);
  MincoOptimizeResult warm_as_guess;
  ASSERT_TRUE(cold.optimize(warm_guess, limits, warm_as_guess)) << warm_as_guess.message;
  EXPECT_EQ(warm_as_guess.iterations, result.iterations);
  EXPECT_NEAR(warm_as_guess.cost, result.cost, 1.0e-9);
  EXPECT_NEAR(warm_as_guess.trajectory.totalDuration(), result.trajectory.totalDuration(), 1.0e-9);
  RecordProperty("warm_start_iterations", std::to_string(result.iterations));
  RecordProperty("warm_start_cost", formatSeconds(result.cost));

  // 清除热启动后仍然可以求解。
  optimizer.clearWarmStart();
  MincoOptimizeResult second;
  EXPECT_TRUE(optimizer.optimize(guess, limits, second)) << second.message;
  EXPECT_EQ(second.status, SolveStatus::kSuccess);

  // 段数不匹配的热启动会被忽略（时长个数与当前初值段数不一致），
  // 既不崩溃也不改变求解结果。
  MincoOptimizer tolerant;
  ASSERT_TRUE(tolerant.configure(config, &reason)) << reason;
  ASSERT_TRUE(tolerant.setWarmStart(
    {{0.6, 0.1}, {1.2, -0.1}, {1.8, 0.1}, {2.4, -0.1}}, {0.4, 0.4, 0.4, 0.4, 0.4}));
  MincoOptimizeResult tolerated;
  EXPECT_TRUE(tolerant.optimize(guess, limits, tolerated)) << tolerated.message;
  EXPECT_EQ(tolerated.status, SolveStatus::kSuccess);
  EXPECT_NEAR(tolerated.cost, second.cost, 1.0e-9) << "段数不匹配的热启动必须被忽略";
}

// ---------------------------------------------------------------------------
// 5) evaluateObjective 的输入校验
// ---------------------------------------------------------------------------

/// \brief `rescaleDurations` 的回归测试：确定性时间拉伸修复。
///
/// 现场背景：MINCO 的速度/加速度是软约束，最优点可能轻微越过硬上限，而验证器按硬上限
/// 判定 —— 越限即整条丢弃，机器人会完全不动。修复手段是“同一组路标点与首末边界条件、
/// 段时长整体放大 k”，因此必须验证：边界 p/v/a 不变、经过点不变、速度约降为 1/k、
/// 加速度约降为 1/k²，且非法入参被拒绝。
TEST(MincoOptimizerRescale, PreservesBoundariesAndReducesExtrema)
{
  MincoOptimizer optimizer;
  srm27_minco_core::MincoOptimizerConfig config;
  config.two_stage = false;
  config.w_time_ratio = 0.0;
  std::string reason;
  ASSERT_TRUE(optimizer.configure(config, &reason)) << reason;

  const TrajectoryInitialGuess guess = makeStraightGuess(1.0);
  Limits2D limits;
  limits.max_linear_speed = 0.5;
  limits.max_linear_accel = 0.3;

  MincoOptimizeResult result;
  ASSERT_TRUE(optimizer.optimize(guess, limits, result));
  ASSERT_EQ(result.status, SolveStatus::kSuccess) << result.message;

  srm27_minco_core::TrajectoryValidator validator;
  srm27_minco_core::TrajectoryValidatorConfig validator_config;
  validator_config.robot_radius = config.robot_radius;
  validator_config.clearance_margin = config.clearance_margin;
  validator_config.max_linear_speed = limits.max_linear_speed;
  validator_config.max_linear_accel = limits.max_linear_accel;
  ASSERT_TRUE(validator.configure(validator_config, &reason)) << reason;

  const Trajectory2D & original = result.trajectory;
  double speed_before = 0.0;
  double accel_before = 0.0;
  ASSERT_TRUE(validator.computeExtrema(original, speed_before, accel_before));

  const double scale = 1.5;
  Trajectory2D repaired;
  ASSERT_TRUE(optimizer.rescaleDurations(original, scale, repaired));

  // 段数与时长：整体放大 k。
  ASSERT_EQ(repaired.pieceCount(), original.pieceCount());
  EXPECT_NEAR(repaired.totalDuration(), original.totalDuration() * scale, 1e-9);
  for (int i = 0; i < repaired.pieceCount(); ++i) {
    EXPECT_NEAR(
      repaired.durations()[static_cast<std::size_t>(i)],
      original.durations()[static_cast<std::size_t>(i)] * scale, 1e-9);
  }

  // 首末 p/v/a 保持不变：修复不能改变边界条件。
  EXPECT_NEAR((repaired.startPosition() - original.startPosition()).norm(), 0.0, 1e-9);
  EXPECT_NEAR((repaired.endPosition() - original.endPosition()).norm(), 0.0, 1e-9);
  EXPECT_NEAR((repaired.startVelocity() - original.startVelocity()).norm(), 0.0, 1e-9);
  EXPECT_NEAR((repaired.endVelocity() - original.endVelocity()).norm(), 0.0, 1e-9);
  EXPECT_NEAR((repaired.startAcceleration() - original.startAcceleration()).norm(), 0.0, 1e-9);
  EXPECT_NEAR((repaired.endAcceleration() - original.endAcceleration()).norm(), 0.0, 1e-9);

  // 经过点保持不变（MINCO 语义）。
  double accumulated = 0.0;
  for (int i = 0; i + 1 < original.pieceCount(); ++i) {
    accumulated += original.durations()[static_cast<std::size_t>(i)];
    const Eigen::Vector2d before = original.positionAt(accumulated);
    double accumulated_repaired = 0.0;
    for (int j = 0; j <= i; ++j) {
      accumulated_repaired += repaired.durations()[static_cast<std::size_t>(j)];
    }
    const Eigen::Vector2d after = repaired.positionAt(accumulated_repaired);
    EXPECT_NEAR((after - before).norm(), 0.0, 1e-9) << "经过点 i=" << i;
  }

  // 极值下降：速度约 1/k，加速度约 1/k²（几何略有变化，故只断言方向与量级）。
  double speed_after = 0.0;
  double accel_after = 0.0;
  ASSERT_TRUE(validator.computeExtrema(repaired, speed_after, accel_after));
  EXPECT_LT(speed_after, speed_before);
  EXPECT_LT(accel_after, accel_before);
  EXPECT_NEAR(speed_after, speed_before / scale, 0.25 * speed_before / scale);
  EXPECT_NEAR(accel_after, accel_before / (scale * scale), 0.4 * accel_before / (scale * scale));

  // 修复后的轨迹仍必须是自洽的。
  EXPECT_TRUE(repaired.sanityCheck(&reason)) << reason;

  // 非法入参：缩水（<1）与空轨迹都必须被拒绝。
  Trajectory2D rejected;
  EXPECT_FALSE(optimizer.rescaleDurations(original, 0.5, rejected));
  EXPECT_FALSE(optimizer.rescaleDurations(Trajectory2D(), 1.5, rejected));
}

TEST(MincoOptimizerObjective, RejectsNonFiniteInputs)
{
  MincoOptimizer optimizer;
  MincoOptimizerConfig config = makeGradientConfig();
  std::string reason;
  ASSERT_TRUE(optimizer.configure(config, &reason)) << reason;
  optimizer.setEsdf(nullptr);

  const Limits2D limits = makeGradientLimits();
  double cost = 0.0;
  Eigen::VectorXd gradient;

  // NaN 路标点。
  TrajectoryInitialGuess nan_waypoint = makeGradientGuess();
  nan_waypoint.waypoints[1] = Eigen::Vector2d(std::numeric_limits<double>::quiet_NaN(), 0.0);
  EXPECT_FALSE(optimizer.evaluateObjective(nan_waypoint, limits, cost, gradient, nullptr));

  // 无穷大路标点。
  TrajectoryInitialGuess inf_waypoint = makeGradientGuess();
  inf_waypoint.waypoints[2] = Eigen::Vector2d(std::numeric_limits<double>::infinity(), 0.0);
  EXPECT_FALSE(optimizer.evaluateObjective(inf_waypoint, limits, cost, gradient, nullptr));

  // NaN 段时长。
  TrajectoryInitialGuess nan_duration = makeGradientGuess();
  nan_duration.durations[1] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(optimizer.evaluateObjective(nan_duration, limits, cost, gradient, nullptr));

  // 路标点数量与段数不一致。
  TrajectoryInitialGuess mismatched = makeGradientGuess();
  mismatched.waypoints.pop_back();
  EXPECT_FALSE(optimizer.evaluateObjective(mismatched, limits, cost, gradient, nullptr));

  // 非法约束。
  Limits2D bad_limits = limits;
  bad_limits.max_linear_accel = -1.0;
  EXPECT_FALSE(
    optimizer.evaluateObjective(makeGradientGuess(), bad_limits, cost, gradient, nullptr));

  // 合法输入必须成功，并且能一并输出轨迹。
  Trajectory2D trajectory;
  EXPECT_TRUE(
    optimizer.evaluateObjective(makeGradientGuess(), limits, cost, gradient, &trajectory));
  EXPECT_TRUE(std::isfinite(cost));
  EXPECT_TRUE(gradient.allFinite());
  EXPECT_EQ(trajectory.pieceCount(), makeGradientGuess().pieceCount());
  EXPECT_TRUE(std::abs(trajectory.startPosition().x()) < 1.0e-9);
  EXPECT_TRUE(std::abs(trajectory.endPosition().x() - 3.0) < 1.0e-9);
  // 输出的轨迹必须与内部采样使用同一套“绝对局部时间”系数约定：
  // 采样点位置应落在轨迹上（用段内求值对照）。
  const Eigen::Vector2d midpoint = trajectory.positionAt(0.5 * trajectory.totalDuration());
  EXPECT_GT(midpoint.x(), 0.0);
  EXPECT_LT(midpoint.x(), 3.0);
}
