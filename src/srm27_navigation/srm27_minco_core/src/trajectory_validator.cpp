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

#include "srm27_minco_core/trajectory_validator.hpp"

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "srm27_minco_core/kinematics.hpp"

namespace srm27_minco_core
{

namespace
{

/// \brief 多项式乘法（系数按幂次递增）。
Eigen::VectorXd polynomialMultiply(const Eigen::VectorXd & _a, const Eigen::VectorXd & _b)
{
  if (_a.size() == 0 || _b.size() == 0) {
    return Eigen::VectorXd::Zero(0);
  }
  Eigen::VectorXd product = Eigen::VectorXd::Zero(_a.size() + _b.size() - 1);
  for (int i = 0; i < _a.size(); ++i) {
    for (int j = 0; j < _b.size(); ++j) {
      product(i + j) += _a(i) * _b(j);
    }
  }
  return product;
}

/// \brief 多项式求导（系数按幂次递增）。
Eigen::VectorXd polynomialDerivative(const Eigen::VectorXd & _coefficients)
{
  if (_coefficients.size() <= 1) {
    return Eigen::VectorXd::Zero(0);
  }
  Eigen::VectorXd derivative = Eigen::VectorXd::Zero(_coefficients.size() - 1);
  for (int i = 1; i < _coefficients.size(); ++i) {
    derivative(i - 1) = static_cast<double>(i) * _coefficients(i);
  }
  return derivative;
}

/// \brief 用伴随矩阵特征值求多项式在 `(_low, _high)` 内的实根（系数按幂次递增）。
std::vector<double> realRootsInRange(
  const Eigen::VectorXd & _coefficients, double _low, double _high)
{
  std::vector<double> roots;
  // 去掉最高阶的零系数，得到真实次数。
  int degree = static_cast<int>(_coefficients.size()) - 1;
  while (degree > 0 && std::abs(_coefficients(degree)) < 1.0e-14) {
    --degree;
  }
  if (degree < 1) {
    return roots;
  }

  Eigen::MatrixXd companion = Eigen::MatrixXd::Zero(degree, degree);
  for (int i = 0; i < degree; ++i) {
    companion(0, i) = -_coefficients(degree - 1 - i) / _coefficients(degree);
  }
  for (int i = 1; i < degree; ++i) {
    companion(i, i - 1) = 1.0;
  }

  const Eigen::EigenSolver<Eigen::MatrixXd> solver(companion, false);
  if (solver.info() != Eigen::Success) {
    return roots;
  }
  const auto eigenvalues = solver.eigenvalues();
  for (int i = 0; i < eigenvalues.size(); ++i) {
    if (std::abs(eigenvalues(i).imag()) > 1.0e-8) {
      continue;
    }
    const double root = eigenvalues(i).real();
    if (root > _low && root < _high) {
      roots.push_back(root);
    }
  }
  return roots;
}

/// \brief 单段内某分量多项式的系数（按幂次递增），并对时间求 `_derivative_order` 阶导数。
Eigen::VectorXd pieceComponentDerivative(
  const Trajectory2D::Coefficients & _coefficients, int _derivative_order)
{
  Eigen::VectorXd values(Trajectory2D::kCoefficientCount);
  for (int k = 0; k < Trajectory2D::kCoefficientCount; ++k) {
    values(k) = _coefficients[static_cast<std::size_t>(k)];
  }
  Eigen::VectorXd result = values;
  for (int order = 0; order < _derivative_order; ++order) {
    result = polynomialDerivative(result);
  }
  return result;
}

/// \brief 由多项式系数求在 `_t` 处的值（Horner，系数按幂次递增）。
double evaluatePolynomial(const Eigen::VectorXd & _coefficients, double _t)
{
  double value = 0.0;
  for (int i = static_cast<int>(_coefficients.size()) - 1; i >= 0; --i) {
    value = value * _t + _coefficients(i);
  }
  return value;
}

}  // namespace

bool TrajectoryValidatorConfig::valid(std::string * _reason) const
{
  const auto fail = [&_reason](const char * _message) {
    if (_reason != nullptr) {
      *_reason = _message;
    }
    return false;
  };
  if (!isFinite(robot_radius) || robot_radius <= 0.0) {
    return fail("robot_radius must be positive");
  }
  if (!isFinite(clearance_margin) || clearance_margin < 0.0) {
    return fail("clearance_margin must be non-negative");
  }
  if (!isFinite(max_linear_speed) || max_linear_speed <= 0.0) {
    return fail("max_linear_speed must be positive");
  }
  if (!isFinite(max_linear_accel) || max_linear_accel <= 0.0) {
    return fail("max_linear_accel must be positive");
  }
  if (
    !isFinite(speed_tolerance_ratio) || speed_tolerance_ratio < 0.0 ||
    !isFinite(accel_tolerance_ratio) || accel_tolerance_ratio < 0.0) {
    return fail("tolerance ratios must be non-negative");
  }
  if (!isFinite(min_piece_duration) || min_piece_duration <= 0.0) {
    return fail("min_piece_duration must be positive");
  }
  if (
    !isFinite(sample_dt) || sample_dt <= 0.0 || !isFinite(max_sample_spacing) ||
    max_sample_spacing <= 0.0) {
    return fail("sampling parameters must be positive");
  }
  if (!isFinite(braking_deceleration) || braking_deceleration <= 0.0) {
    return fail("braking_deceleration must be positive and must come from identification");
  }
  if (!isFinite(reaction_latency) || reaction_latency < 0.0) {
    return fail("reaction_latency must be non-negative");
  }
  if (!isFinite(required_prefix_duration) || required_prefix_duration < 0.0) {
    return fail("required_prefix_duration must be non-negative");
  }
  if (
    !isFinite(map_timeout) || map_timeout <= 0.0 || !isFinite(trajectory_max_age) ||
    trajectory_max_age <= 0.0) {
    return fail("timeouts must be positive");
  }
  return true;
}

bool TrajectoryValidator::configure(
  const TrajectoryValidatorConfig & _config, std::string * _reason)
{
  configured_ = false;
  if (!_config.valid(_reason)) {
    return false;
  }
  config_ = _config;
  configured_ = true;
  return true;
}

bool TrajectoryValidator::computeExtrema(
  const Trajectory2D & _trajectory, double & _max_speed, double & _max_acceleration) const
{
  _max_speed = 0.0;
  _max_acceleration = 0.0;
  if (_trajectory.empty()) {
    return false;
  }

  const std::vector<double> & durations = _trajectory.durations();
  const std::vector<Trajectory2D::Coefficients> & x_coefficients = _trajectory.xCoefficients();
  const std::vector<Trajectory2D::Coefficients> & y_coefficients = _trajectory.yCoefficients();

  for (int i = 0; i < _trajectory.pieceCount(); ++i) {
    const double duration = durations[static_cast<std::size_t>(i)];
    if (!isFinite(duration) || duration <= 0.0) {
      return false;
    }

    // 速度：vx(t)、vy(t) 的多项式（物理时间）。
    const Eigen::VectorXd vx =
      pieceComponentDerivative(x_coefficients[static_cast<std::size_t>(i)], 1);
    const Eigen::VectorXd vy =
      pieceComponentDerivative(y_coefficients[static_cast<std::size_t>(i)], 1);
    const Eigen::VectorXd ax =
      pieceComponentDerivative(x_coefficients[static_cast<std::size_t>(i)], 2);
    const Eigen::VectorXd ay =
      pieceComponentDerivative(y_coefficients[static_cast<std::size_t>(i)], 2);

    // ||v||^2 的 8 次多项式与其导数的实根给出速度极值候选点。
    const Eigen::VectorXd speed_sq = polynomialMultiply(vx, vx) + polynomialMultiply(vy, vy);
    const Eigen::VectorXd speed_sq_derivative = polynomialDerivative(speed_sq);
    std::vector<double> candidates = realRootsInRange(speed_sq_derivative, 0.0, duration);
    candidates.push_back(0.0);
    candidates.push_back(duration);

    const Eigen::VectorXd accel_sq = polynomialMultiply(ax, ax) + polynomialMultiply(ay, ay);
    const Eigen::VectorXd accel_sq_derivative = polynomialDerivative(accel_sq);
    std::vector<double> accel_candidates = realRootsInRange(accel_sq_derivative, 0.0, duration);
    accel_candidates.push_back(0.0);
    accel_candidates.push_back(duration);

    for (const double t : candidates) {
      const double value = evaluatePolynomial(speed_sq, t);
      if (isFinite(value) && value > 0.0) {
        _max_speed = std::max(_max_speed, std::sqrt(value));
      }
    }
    for (const double t : accel_candidates) {
      const double value = evaluatePolynomial(accel_sq, t);
      if (isFinite(value) && value > 0.0) {
        _max_acceleration = std::max(_max_acceleration, std::sqrt(value));
      }
    }
  }

  // 保底：多项式由数值求解得到的系数可能条件数较差，这里再用加密采样兜底。
  const int dense_samples = 200;
  for (int i = 0; i < dense_samples; ++i) {
    const double t =
      _trajectory.totalDuration() * static_cast<double>(i) / static_cast<double>(dense_samples - 1);
    _max_speed = std::max(_max_speed, _trajectory.velocityAt(t).norm());
    _max_acceleration = std::max(_max_acceleration, _trajectory.accelerationAt(t).norm());
  }
  return isFinite(_max_speed) && isFinite(_max_acceleration);
}

bool TrajectoryValidator::checkCollision(
  const Trajectory2D & _trajectory, const Esdf2D & _esdf, double _from_time, double _to_time,
  double & _min_clearance, double & _first_violation_time) const
{
  _min_clearance = std::numeric_limits<double>::infinity();
  _first_violation_time = _to_time;
  if (_trajectory.empty() || !_esdf.valid()) {
    return false;
  }
  const double low = clampToRange(_from_time, 0.0, _trajectory.totalDuration());
  const double high = clampToRange(_to_time, low, _trajectory.totalDuration());
  if (high <= low) {
    return false;
  }

  const double required_clearance = config_.robot_radius + config_.clearance_margin;

  // 采样步长同时受时间步长与最大空间间距约束，避免高速时跳过薄障碍。
  double step = config_.sample_dt;
  double max_speed = 0.0;
  double max_accel = 0.0;
  computeExtrema(_trajectory, max_speed, max_accel);
  if (max_speed > 1.0e-6) {
    step = std::min(step, config_.max_sample_spacing / max_speed);
  }
  step = std::max(step, 1.0e-4);

  bool collision_free = true;
  for (double t = low; t < high + 1.0e-12; t += step) {
    const double sample_time = std::min(t, high);
    const Eigen::Vector2d sample_position = _trajectory.positionAt(sample_time);
    const EsdfQueryResult query = _esdf.query(sample_position.x(), sample_position.y());
    if (!query.valid) {
      // 轨迹越出地图有效区域：按不可通行处理，不放行。
      collision_free = false;
      _first_violation_time = std::min(_first_violation_time, sample_time);
      _min_clearance = std::min(_min_clearance, -std::numeric_limits<double>::infinity());
      break;
    }
    if (query.distance < _min_clearance) {
      _min_clearance = query.distance;
    }
    if (query.distance < required_clearance) {
      collision_free = false;
      _first_violation_time = std::min(_first_violation_time, sample_time);
      break;
    }
    if (sample_time >= high) {
      break;
    }
  }
  if (!isFinite(_min_clearance)) {
    _min_clearance = -std::numeric_limits<double>::infinity();
  }
  return collision_free;
}

bool TrajectoryValidator::validate(
  const Trajectory2D & _trajectory, const Esdf2D & _esdf, const State2D & _state, double _now_stamp,
  TrajectoryValidationReport & _report) const
{
  _report = TrajectoryValidationReport();
  if (!configured_) {
    _report.reason = "validator not configured";
    return false;
  }
  if (!_esdf.valid()) {
    _report.reason = "no valid distance field";
    return false;
  }
  if (_trajectory.empty()) {
    _report.reason = "empty trajectory";
    return false;
  }

  // 1) 系数、时长与段间连续性。
  std::string sanity_reason;
  _report.coefficients_ok = _trajectory.sanityCheck(&sanity_reason);
  _trajectory.continuityResiduals(
    _report.continuity_max_position, _report.continuity_max_velocity,
    _report.continuity_max_acceleration);
  _report.continuity_ok = _report.continuity_max_position < 1.0e-6 &&
                          _report.continuity_max_velocity < 1.0e-5 &&
                          _report.continuity_max_acceleration < 1.0e-4;
  if (!_report.coefficients_ok || !_report.continuity_ok) {
    // 不要把具体原因吞掉：只报“自检失败”会让现场无法定位是时长、系数、连续性还是
    // 有效期元数据出的问题（实际就是这么埋过一次 bug）。
    _report.reason = "trajectory sanity check failed: " + sanity_reason;
    return false;
  }
  _report.boundary_ok = true;

  // 2) 段时长下限。
  for (const double duration : _trajectory.durations()) {
    if (!isFinite(duration) || duration < config_.min_piece_duration) {
      _report.reason = "piece duration below the configured minimum";
      return false;
    }
  }

  // 3) 动力学极值。
  _report.dynamics_ok = computeExtrema(_trajectory, _report.max_speed, _report.max_acceleration);
  if (!_report.dynamics_ok) {
    _report.reason = "failed to evaluate trajectory extrema";
    return false;
  }
  const double speed_limit = config_.max_linear_speed * (1.0 + config_.speed_tolerance_ratio);
  const double accel_limit = config_.max_linear_accel * (1.0 + config_.accel_tolerance_ratio);
  if (_report.max_speed > speed_limit) {
    _report.reason = "max speed exceeds the effective limit";
    return false;
  }
  if (_report.max_acceleration > accel_limit) {
    _report.reason = "max acceleration exceeds the effective limit";
    return false;
  }

  // 4) 全段扫掠净空。
  double min_clearance = 0.0;
  double first_violation = 0.0;
  _report.collision_ok = checkCollision(
    _trajectory, _esdf, 0.0, _trajectory.totalDuration(), min_clearance, first_violation);
  _report.min_clearance = min_clearance;
  _report.first_violation_time = first_violation;
  _report.min_clearance_time = 0.0;
  if (!_report.collision_ok) {
    // 仍然记录最小净空出现的位置，便于诊断。
    double best_distance = std::numeric_limits<double>::infinity();
    double best_time = 0.0;
    for (double t = 0.0; t <= _trajectory.totalDuration(); t += config_.sample_dt) {
      const Eigen::Vector2d sample_position = _trajectory.positionAt(t);
      const EsdfQueryResult query = _esdf.query(sample_position.x(), sample_position.y());
      if (query.valid && query.distance < best_distance) {
        best_distance = query.distance;
        best_time = t;
      }
    }
    if (isFinite(best_distance)) {
      _report.min_clearance = best_distance;
      _report.min_clearance_time = best_time;
    }
    _report.reason = "trajectory collides with the raw obstacle set";
    return false;
  }

  // 5) 有效前缀与覆盖需求。
  _report.effective_prefix_duration = first_violation;
  const double current_speed = _state.valid ? _state.velocity.norm() : 0.0;
  _report.required_stop_time =
    config_.reaction_latency + current_speed / config_.braking_deceleration;
  _report.required_prefix_duration =
    std::max(config_.required_prefix_duration, _report.required_stop_time);
  _report.coverage_ok =
    _report.effective_prefix_duration + 1.0e-6 >= _report.required_prefix_duration;
  if (!_report.coverage_ok) {
    _report.reason = "safe prefix is shorter than the MPC window plus stopping time";
    return false;
  }

  // 6) 时效：地图与轨迹都必须新鲜。
  _report.timing_ok = true;
  if (isFinite(_now_stamp)) {
    if ((_now_stamp - _esdf.stamp()) > config_.map_timeout) {
      _report.timing_ok = false;
      _report.reason = "distance field is older than map_timeout";
      return false;
    }
    if (_now_stamp < _trajectory.valid_after) {
      _report.timing_ok = false;
      _report.reason = "trajectory is not valid yet";
      return false;
    }
    if (_trajectory.valid_until > 0.0 && _now_stamp > _trajectory.valid_until) {
      _report.timing_ok = false;
      _report.reason = "trajectory has expired";
      return false;
    }
    if (
      _trajectory.generated_stamp > 0.0 &&
      (_now_stamp - _trajectory.generated_stamp) > config_.trajectory_max_age) {
      _report.timing_ok = false;
      _report.reason = "trajectory is older than trajectory_max_age";
      return false;
    }
  }
  if (_trajectory.terminal_is_global_goal && _trajectory.endVelocity().norm() > 0.05) {
    _report.timing_ok = false;
    _report.reason = "global-goal trajectory does not end at rest";
    return false;
  }

  _report.valid = true;
  _report.reason.clear();
  return true;
}

}  // namespace srm27_minco_core
