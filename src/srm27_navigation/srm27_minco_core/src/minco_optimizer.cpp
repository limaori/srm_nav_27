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

#include "srm27_minco_core/minco_optimizer.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

#include "gcopter/lbfgs.hpp"
#include "srm27_minco_core/kinematics.hpp"
#include "srm27_minco_core/smooth_cost.hpp"

namespace srm27_minco_core
{

namespace
{

/// \brief 速度视为零、无法定义切向的阈值（m/s）。
constexpr double kTangentSpeedEpsilon = 1.0e-3;

}  // namespace

bool MincoOptimizerConfig::valid(std::string * _reason) const
{
  const auto fail = [&_reason](const char * _message) {
    if (_reason != nullptr) {
      *_reason = _message;
    }
    return false;
  };

  const double weights[] = {w_jerk,         w_time,      w_obstacle,  w_velocity,
                            w_acceleration, w_reference, w_time_ratio};
  for (const double weight : weights) {
    if (!isFinite(weight) || weight < 0.0) {
      return fail("cost weight must be finite and non-negative");
    }
  }
  if (
    w_jerk <= 0.0 && w_time <= 0.0 && w_obstacle <= 0.0 && w_velocity <= 0.0 &&
    w_acceleration <= 0.0) {
    return fail("at least one primary cost weight must be positive");
  }
  if (!isFinite(soft_hinge_beta) || soft_hinge_beta <= 0.0) {
    return fail("soft_hinge_beta must be positive");
  }
  if (samples_per_piece < 2) {
    return fail("samples_per_piece must be at least 2");
  }
  if (!isFinite(min_piece_duration) || min_piece_duration <= 0.0) {
    return fail("min_piece_duration must be positive");
  }
  if (!isFinite(max_piece_duration) || max_piece_duration <= min_piece_duration) {
    return fail("max_piece_duration must be greater than min_piece_duration");
  }
  if (max_iterations < 1) {
    return fail("max_iterations must be at least 1");
  }
  if (!isFinite(gradient_tolerance) || gradient_tolerance < 0.0) {
    return fail("gradient_tolerance must be non-negative");
  }
  if (lbfgs_memory < 3) {
    return fail("lbfgs_memory must be at least 3");
  }
  if (
    !isFinite(pre_time_ratio_min) || !isFinite(pre_time_ratio_max) || pre_time_ratio_min <= 0.0 ||
    pre_time_ratio_min > pre_time_ratio_max) {
    return fail("pre_time_ratio bounds are invalid");
  }
  if (!isFinite(fine_slope_threshold) || !isFinite(fine_probe_step) || fine_probe_step <= 0.0) {
    return fail("fine heuristic parameters are invalid");
  }
  if (!isFinite(robot_radius) || robot_radius <= 0.0) {
    return fail("robot_radius must be positive and finite");
  }
  if (!isFinite(clearance_margin) || clearance_margin < 0.0) {
    return fail("clearance_margin must be non-negative and finite");
  }
  return true;
}

bool MincoOptimizer::configure(const MincoOptimizerConfig & _config, std::string * _reason)
{
  configured_ = false;
  if (!_config.valid(_reason)) {
    return false;
  }
  config_ = _config;
  configured_ = true;
  return true;
}

bool MincoOptimizer::setWarmStart(
  const std::vector<Eigen::Vector2d> & _inner_points, const std::vector<double> & _durations)
{
  // 内部路标点**不含首末点**，所以数量必须是段数 - 1，而段数等于时长个数：
  // 这里必须用 size() + 1 == durations.size() 判定，才能和 optimize() 里
  // “warm_inner_points_.size() == pieces - 1 && warm_durations_.size() == pieces”
  // 的取值条件同时成立（先前用 size() == size() 会让热启动永远无法生效）。
  if (_inner_points.size() + 1 != _durations.size()) {
    return false;
  }
  for (const Eigen::Vector2d & point : _inner_points) {
    if (!point.allFinite()) {
      return false;
    }
  }
  for (const double duration : _durations) {
    if (!isFinite(duration) || duration <= config_.min_piece_duration) {
      return false;
    }
  }
  warm_inner_points_ = _inner_points;
  warm_durations_ = _durations;
  has_warm_start_ = !_inner_points.empty();
  return true;
}

void MincoOptimizer::clearWarmStart()
{
  warm_inner_points_.clear();
  warm_durations_.clear();
  has_warm_start_ = false;
}

Eigen::VectorXd MincoOptimizer::encodeVariables(const TrajectoryInitialGuess & _guess) const
{
  const int pieces = _guess.pieceCount();
  Eigen::VectorXd x = Eigen::VectorXd::Zero(2 * std::max(0, pieces - 1) + pieces);
  for (int i = 0; i < pieces - 1; ++i) {
    const Eigen::Vector2d & point = _guess.waypoints[static_cast<std::size_t>(i) + 1];
    x(2 * i) = point.x();
    x(2 * i + 1) = point.y();
  }
  for (int i = 0; i < pieces; ++i) {
    const double duration = std::max(
      config_.min_piece_duration * (1.0 + 1.0e-6), _guess.durations[static_cast<std::size_t>(i)]);
    x(2 * (pieces - 1) + i) = softplusInverse(duration - config_.min_piece_duration);
  }
  return x;
}

double MincoOptimizer::evaluateCallback(
  void * _instance, const Eigen::VectorXd & _x, Eigen::VectorXd & _g)
{
  auto * self = static_cast<MincoOptimizer *>(_instance);
  return self->evaluate(_x, _g, true);
}

int MincoOptimizer::progressCallback(
  void * _instance, const Eigen::VectorXd & _x, const Eigen::VectorXd & _g, double _fx,
  double _step, int _k, int _ls)
{
  (void)_x;
  (void)_g;
  (void)_fx;
  (void)_step;
  (void)_ls;
  auto * self = static_cast<MincoOptimizer *>(_instance);
  if (self != nullptr) {
    self->last_iteration_ = _k + 1;
  }
  return 0;
}

double MincoOptimizer::evaluate(
  const Eigen::VectorXd & _x, Eigen::VectorXd & _gradient, bool _need_gradient)
{
  const int pieces = problem_.pieces;
  const int inner_count = std::max(0, pieces - 1);
  const int expected = 2 * inner_count + pieces;
  if (pieces < 1 || _x.size() != expected) {
    return std::numeric_limits<double>::infinity();
  }
  if (!_x.allFinite()) {
    return std::numeric_limits<double>::infinity();
  }

  // 1) 解码自变量：内部路标点 + 正值段时长（T = Tmin + softplus(s)）。
  problem_.inner.resize(3, inner_count);
  problem_.durations.resize(pieces);
  problem_.time_parameters.resize(pieces);
  for (int i = 0; i < inner_count; ++i) {
    problem_.inner.col(i) << _x(2 * i), _x(2 * i + 1), 0.0;
  }
  for (int i = 0; i < pieces; ++i) {
    const double s = _x(2 * inner_count + i);
    problem_.time_parameters(i) = s;
    const double duration = config_.min_piece_duration + softplus(s);
    if (!isFinite(duration) || duration <= 0.0) {
      return std::numeric_limits<double>::infinity();
    }
    problem_.durations(i) = duration;
  }

  // 2) 生成 MINCO 系数（段内 c0..c5，按幂次递增存放）。
  generator_.setParameters(problem_.inner, problem_.durations);
  const Eigen::MatrixX3d & coeffs = generator_.getCoeffs();

  // 3) jerk 能量项。
  double energy = 0.0;
  generator_.getEnergy(energy);
  Eigen::MatrixX3d grad_by_coeffs;
  Eigen::VectorXd grad_by_times;
  if (_need_gradient) {
    generator_.getEnergyPartialGradByCoeffs(grad_by_coeffs);
    generator_.getEnergyPartialGradByTimes(grad_by_times);
    grad_by_coeffs *= config_.w_jerk;
    grad_by_times *= config_.w_jerk;
  } else {
    grad_by_coeffs = Eigen::MatrixX3d::Zero(6 * pieces, 3);
    grad_by_times = Eigen::VectorXd::Zero(pieces);
  }

  double cost = config_.w_jerk * energy;

  // 4) 总时长项：dJ/dT_i = w_time。
  cost += config_.w_time * problem_.durations.sum();
  if (_need_gradient) {
    grad_by_times.array() += config_.w_time;
  }

  // 5) 段时长比例惩罚（PRE 阶段）。mean(T) 参与求导，不能当常数。
  if (problem_.use_time_ratio_penalty && config_.w_time_ratio > 0.0) {
    const double mean_duration = problem_.durations.mean();
    if (mean_duration > 1.0e-9) {
      const double inverse_mean = 1.0 / mean_duration;
      std::vector<double> ratio_derivative(static_cast<std::size_t>(pieces), 0.0);
      std::vector<double> ratio(static_cast<std::size_t>(pieces), 0.0);
      for (int i = 0; i < pieces; ++i) {
        ratio[static_cast<std::size_t>(i)] = problem_.durations(i) * inverse_mean;
        double value = 0.0;
        double derivative = 0.0;
        if (!timeRatioPenalty(
              ratio[static_cast<std::size_t>(i)], config_.pre_time_ratio_min,
              config_.pre_time_ratio_max, value, derivative)) {
          return std::numeric_limits<double>::infinity();
        }
        cost += config_.w_time_ratio * value;
        ratio_derivative[static_cast<std::size_t>(i)] = config_.w_time_ratio * derivative;
      }
      if (_need_gradient) {
        for (int j = 0; j < pieces; ++j) {
          double accumulated = 0.0;
          for (int i = 0; i < pieces; ++i) {
            const double delta = (i == j) ? 1.0 : 0.0;
            accumulated +=
              ratio_derivative[static_cast<std::size_t>(i)] *
              (delta - ratio[static_cast<std::size_t>(i)] / static_cast<double>(pieces)) *
              inverse_mean;
          }
          grad_by_times(j) += accumulated;
        }
      }
    }
  }

  // 6) 参考吸引项（只作用于内部路标点，可选）。代价在这里累加，保证
  //    “只求值不求梯度”的分支（_need_gradient = false）与带梯度的分支
  //    返回同一个代价；对应导数在步骤 11 累加到路标点梯度上。
  const bool use_reference = config_.w_reference > 0.0 && !problem_.reference_points.empty() &&
                             static_cast<int>(problem_.reference_points.size()) == inner_count;
  if (use_reference) {
    for (int i = 0; i < inner_count; ++i) {
      const Eigen::Vector2d difference =
        problem_.inner.col(i).head<2>() - problem_.reference_points[static_cast<std::size_t>(i)];
      cost += config_.w_reference * difference.squaredNorm();
    }
  }

  // 7) 段内数值积分：障碍 / 速度 / 加速度软约束。
  const double safety_distance = safetyDistance();
  const double max_speed_sq = limits_max_speed_sq_;
  const double max_accel_sq = limits_max_accel_sq_;
  const int samples = config_.samples_per_piece;
  const double quadrature_weight = 1.0 / static_cast<double>(samples);
  const double beta = config_.soft_hinge_beta;

  // 每个采样点的 ∂L/∂p、∂L/∂v、∂L/∂a 缓存，以及采样点的 v/a/jerk 缓存
  // （v/a/jerk 同时供步骤 9 的 ∂J/∂T 使用，避免两处各写一遍求值公式）。
  Eigen::Matrix<double, 2, Eigen::Dynamic> dldp(2, samples);
  Eigen::Matrix<double, 2, Eigen::Dynamic> dldv(2, samples);
  Eigen::Matrix<double, 2, Eigen::Dynamic> dlda(2, samples);
  Eigen::Matrix<double, 2, Eigen::Dynamic> sample_velocity(2, samples);
  Eigen::Matrix<double, 2, Eigen::Dynamic> sample_acceleration(2, samples);
  Eigen::Matrix<double, 2, Eigen::Dynamic> sample_jerk(2, samples);
  std::vector<double> sample_value(static_cast<std::size_t>(samples), 0.0);
  std::vector<double> sample_fraction(static_cast<std::size_t>(samples), 0.0);

  for (int i = 0; i < pieces; ++i) {
    const double duration = problem_.durations(i);
    const Eigen::VectorXd xc = coeffs.block<6, 1>(6 * i, 0);
    const Eigen::VectorXd yc = coeffs.block<6, 1>(6 * i, 1);

    for (int j = 0; j < samples; ++j) {
      // midpoint 规则：节点取在段内部，避免在段边界处取到 hinge 的折点。
      const double fraction = (static_cast<double>(j) + 0.5) / static_cast<double>(samples);
      sample_fraction[static_cast<std::size_t>(j)] = fraction;
      const double tau = fraction * duration;

      // MINCO 系数保存的是段内**绝对**局部时间 tau ∈ [0, T] 的幂次
      // （p(tau) = Σ c_k tau^k，见 MINCO_S3NU::setParameters 中带 T^k 的约束行），
      // 所以 p/v/a/jerk 就是对 tau 逐阶求导，**不能**再除以 T：
      //   p = Σ c_k tau^k, v = p'(tau), a = p''(tau), jerk = p'''(tau)。
      // 若除以 T（等价于把系数当成“归一化时间 t/T”的系数），采样点的速度与
      // 加速度会分别偏大 1/T、1/T² 倍，速度/加速度软约束的阈值随之失效。
      double px = 0.0;
      double py = 0.0;
      double vx = 0.0;
      double vy = 0.0;
      double ax = 0.0;
      double ay = 0.0;
      double jx = 0.0;
      double jy = 0.0;
      Eigen::Matrix<double, 6, 1> tau_powers;
      tau_powers(0) = 1.0;
      for (int k = 1; k < 6; ++k) {
        tau_powers(k) = tau_powers(k - 1) * tau;
      }
      for (int k = 0; k < 6; ++k) {
        px += xc(k) * tau_powers(k);
        py += yc(k) * tau_powers(k);
      }
      for (int k = 1; k < 6; ++k) {
        vx += static_cast<double>(k) * xc(k) * tau_powers(k - 1);
        vy += static_cast<double>(k) * yc(k) * tau_powers(k - 1);
      }
      for (int k = 2; k < 6; ++k) {
        const double factor = static_cast<double>(k) * static_cast<double>(k - 1);
        ax += factor * xc(k) * tau_powers(k - 2);
        ay += factor * yc(k) * tau_powers(k - 2);
      }
      for (int k = 3; k < 6; ++k) {
        const double factor =
          static_cast<double>(k) * static_cast<double>(k - 1) * static_cast<double>(k - 2);
        jx += factor * xc(k) * tau_powers(k - 3);
        jy += factor * yc(k) * tau_powers(k - 3);
      }

      const Eigen::Vector2d position(px, py);
      const Eigen::Vector2d velocity(vx, vy);
      const Eigen::Vector2d acceleration(ax, ay);
      const Eigen::Vector2d jerk(jx, jy);

      double integrand = 0.0;
      Eigen::Vector2d grad_position = Eigen::Vector2d::Zero();
      Eigen::Vector2d grad_velocity = Eigen::Vector2d::Zero();
      Eigen::Vector2d grad_acceleration = Eigen::Vector2d::Zero();

      // 7.1 障碍软约束：h = d_safe - d(p)。
      if (config_.w_obstacle > 0.0 && esdf_ && esdf_->valid()) {
        const EsdfQueryResult query = esdf_->query(position.x(), position.y());
        if (query.valid) {
          double value = 0.0;
          double derivative = 0.0;
          if (!softplusHinge(safety_distance - query.distance, beta, value, derivative)) {
            return std::numeric_limits<double>::infinity();
          }
          integrand += config_.w_obstacle * value;
          if (query.gradient_valid && derivative > 0.0) {
            Eigen::Vector2d gradient = query.gradient;
            if (problem_.use_fine_heuristic) {
              gradient = applyFineHeuristic(position, velocity, gradient, query.distance);
            }
            grad_position += -config_.w_obstacle * derivative * gradient;
          }
        }
      }

      // 7.2 速度软约束：h = ||v||^2 - v_max^2。
      if (config_.w_velocity > 0.0) {
        const double speed_sq = velocity.squaredNorm();
        double value = 0.0;
        double derivative = 0.0;
        if (!softplusHinge(speed_sq - max_speed_sq, beta, value, derivative)) {
          return std::numeric_limits<double>::infinity();
        }
        integrand += config_.w_velocity * value;
        grad_velocity += config_.w_velocity * derivative * 2.0 * velocity;
      }

      // 7.3 加速度软约束：h = ||a||^2 - a_max^2。
      if (config_.w_acceleration > 0.0) {
        const double accel_sq = acceleration.squaredNorm();
        double value = 0.0;
        double derivative = 0.0;
        if (!softplusHinge(accel_sq - max_accel_sq, beta, value, derivative)) {
          return std::numeric_limits<double>::infinity();
        }
        integrand += config_.w_acceleration * value;
        grad_acceleration += config_.w_acceleration * derivative * 2.0 * acceleration;
      }

      sample_value[static_cast<std::size_t>(j)] = integrand;
      dldp.col(j) = grad_position;
      dldv.col(j) = grad_velocity;
      dlda.col(j) = grad_acceleration;
      sample_velocity.col(j) = velocity;
      sample_acceleration.col(j) = acceleration;
      sample_jerk.col(j) = jerk;

      cost += duration * quadrature_weight * integrand;
    }

    if (!_need_gradient) {
      continue;
    }

    // 8) 回传到段系数（tau_j = f_j * T）：
    //    ∂J/∂c_k = Σ_j w*T * ( tau^k ∂L/∂p + k*tau^(k-1) ∂L/∂v
    //                          + k*(k-1)*tau^(k-2) ∂L/∂a )。
    for (int k = 0; k < 6; ++k) {
      Eigen::Vector2d accumulated = Eigen::Vector2d::Zero();
      for (int j = 0; j < samples; ++j) {
        const double tau = sample_fraction[static_cast<std::size_t>(j)] * duration;
        const double power_k = std::pow(tau, k);
        double power_v = 0.0;
        if (k >= 1) {
          power_v = static_cast<double>(k) * std::pow(tau, k - 1);
        }
        double power_a = 0.0;
        if (k >= 2) {
          power_a = static_cast<double>(k) * static_cast<double>(k - 1) * std::pow(tau, k - 2);
        }
        accumulated += quadrature_weight * duration *
                       (power_k * dldp.col(j) + power_v * dldv.col(j) + power_a * dlda.col(j));
      }
      grad_by_coeffs(6 * i + k, 0) += accumulated.x();
      grad_by_coeffs(6 * i + k, 1) += accumulated.y();
    }

    // 9) 回传到段时长：固定系数 c、只让 T 变化（tau_j = f_j * T 随之变化）。
    //    每段的积分权重是 T/samples，所以
    //    ∂J/∂T_i|_c = Σ_j w * L_j                                  （积分权重 T/samples）
    //               + Σ_j w * T * f_j * ( ∂L/∂p·v_j + ∂L/∂v·a_j
    //                                     + ∂L/∂a·jerk_j )，
    //    因为 ∂p_j/∂T = f_j*v_j、∂v_j/∂T = f_j*a_j、∂a_j/∂T = f_j*jerk_j，
    //    而权重里的 T 必须一起带上（漏掉它会在时间梯度上少一个 T 因子）。
    //    系数 c 随 T 变化的间接项由 propogateGrad() 在步骤 10 补上。
    double time_gradient = 0.0;
    for (int j = 0; j < samples; ++j) {
      const double fraction = sample_fraction[static_cast<std::size_t>(j)];
      time_gradient += quadrature_weight * sample_value[static_cast<std::size_t>(j)];
      time_gradient +=
        quadrature_weight * duration * fraction *
        (dldp.col(j).dot(sample_velocity.col(j)) + dldv.col(j).dot(sample_acceleration.col(j)) +
         dlda.col(j).dot(sample_jerk.col(j)));
    }
    grad_by_times(i) += time_gradient;
  }

  if (!_need_gradient) {
    // 只需要函数值：problem_ 与 generator_ 已按当前自变量更新完毕，可直接返回。
    return cost;
  }

  // 10) MINCO 梯度回传：得到 ∂J/∂路标点 与 ∂J/∂段时长。
  Eigen::Matrix3Xd grad_by_points;
  Eigen::VectorXd grad_by_durations;
  generator_.propogateGrad(grad_by_coeffs, grad_by_times, grad_by_points, grad_by_durations);

  _gradient.setZero(expected);
  for (int i = 0; i < inner_count; ++i) {
    _gradient(2 * i) = grad_by_points(0, i);
    _gradient(2 * i + 1) = grad_by_points(1, i);
  }

  // 11) 参考吸引项对路标点的直接导数（代价已在步骤 6 累加）。
  if (use_reference) {
    for (int i = 0; i < inner_count; ++i) {
      const Eigen::Vector2d difference =
        problem_.inner.col(i).head<2>() - problem_.reference_points[static_cast<std::size_t>(i)];
      _gradient(2 * i) += 2.0 * config_.w_reference * difference.x();
      _gradient(2 * i + 1) += 2.0 * config_.w_reference * difference.y();
    }
  }

  // 12) 时间变量链式法则：s -> T 的导数是 sigmoid(s)。
  for (int i = 0; i < pieces; ++i) {
    _gradient(2 * inner_count + i) =
      grad_by_durations(i) * stableSigmoid(problem_.time_parameters(i));
  }

  if (!isFinite(cost)) {
    return std::numeric_limits<double>::infinity();
  }
  return cost;
}

Eigen::Vector2d MincoOptimizer::applyFineHeuristic(
  const Eigen::Vector2d & _position, const Eigen::Vector2d & _velocity,
  const Eigen::Vector2d & _gradient, double _distance) const
{
  // 报告式 FINELY 启发式（方案 §6.5）。默认关闭：它修改了梯度方向，
  // 代价与梯度不再是严格导数关系，因此只能在常规软约束稳定后单独评估。
  const double speed = _velocity.norm();
  if (!esdf_ || speed < kTangentSpeedEpsilon) {
    // 低速时方向不定：保留完整梯度，不做切向剔除，也不做零向量归一化。
    return _gradient;
  }
  const Eigen::Vector2d tangent = _velocity / speed;
  const Eigen::Vector2d perpendicular = _gradient - tangent * tangent.dot(_gradient);
  const double perpendicular_norm = perpendicular.norm();
  if (perpendicular_norm < 1.0e-9) {
    return _gradient;
  }
  const Eigen::Vector2d direction = perpendicular / perpendicular_norm;
  const EsdfQueryResult probe = esdf_->query(
    _position.x() + config_.fine_probe_step * direction.x(),
    _position.y() + config_.fine_probe_step * direction.y());
  if (!probe.valid) {
    return _gradient;
  }
  const double slope = (probe.distance - _distance) / config_.fine_probe_step;
  if (!isFinite(slope)) {
    return _gradient;
  }
  const double gain = clampToRange(slope / config_.fine_slope_threshold, 0.0, 1.0);
  // 报告式做法：沿探测方向改善越慢，越要缩小障碍梯度（“violaPos 与梯度大小”一起调整）。
  // 这里按标量 gain 缩放整个梯度，保持梯度方向不变、只改变步长尺度。
  return gain * _gradient;
}

bool MincoOptimizer::extractTrajectory(Trajectory2D & _trajectory) const
{
  const int pieces = problem_.pieces;
  if (pieces < 1) {
    return false;
  }
  const Eigen::MatrixX3d & coeffs = generator_.getCoeffs();
  if (coeffs.rows() != 6 * pieces) {
    return false;
  }
  _trajectory.clear();
  for (int i = 0; i < pieces; ++i) {
    Trajectory2D::Coefficients x{};
    Trajectory2D::Coefficients y{};
    for (int k = 0; k < Trajectory2D::kCoefficientCount; ++k) {
      x[static_cast<std::size_t>(k)] = coeffs(6 * i + k, 0);
      y[static_cast<std::size_t>(k)] = coeffs(6 * i + k, 1);
    }
    if (!_trajectory.addPiece(x, y, problem_.durations(i))) {
      _trajectory.clear();
      return false;
    }
  }
  return true;
}

SolveStatus MincoOptimizer::runStage(
  bool _pre_stage, Eigen::VectorXd & _x, double & _cost, int & _iterations)
{
  problem_.use_time_ratio_penalty = _pre_stage && config_.w_time_ratio > 0.0;
  problem_.use_fine_heuristic = !_pre_stage && config_.enable_report_fine_heuristic;

  lbfgs::lbfgs_parameter_t parameters;
  parameters.mem_size = config_.lbfgs_memory;
  parameters.g_epsilon = config_.gradient_tolerance;
  parameters.past = 0;
  parameters.max_iterations = config_.max_iterations;
  last_iteration_ = 0;
  const auto stage_begin = std::chrono::steady_clock::now();

  // 先用当前状态求一次值，确定参考点与约束快照都合法。
  Eigen::VectorXd gradient;
  double cost = evaluate(_x, gradient, true);
  if (!isFinite(cost)) {
    return SolveStatus::kInvalidInput;
  }

  int status = lbfgs::lbfgs_optimize(
    _x, cost, &MincoOptimizer::evaluateCallback, nullptr, &MincoOptimizer::progressCallback,
    static_cast<void *>(this), parameters);
  _cost = cost;
  _iterations += last_iteration_;

  // 重新在最终自变量上评估，保证 problem_/generator_ 与返回的代价一致。
  double final_cost = evaluate(_x, gradient, false);
  if (!isFinite(final_cost)) {
    return SolveStatus::kNumericalFailure;
  }
  _cost = final_cost;

  last_stage_time_ms_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
                          std::chrono::steady_clock::now() - stage_begin)
                          .count() *
                        1.0e-6;

  if (status == lbfgs::LBFGS_CONVERGENCE || status == lbfgs::LBFGS_STOP) {
    return SolveStatus::kSuccess;
  }
  if (status == lbfgs::LBFGS_CANCELED || status == lbfgs::LBFGSERR_INCREASEGRADIENT) {
    // 起点处梯度方向已不能下降：结果有限，交由验证器判定是否可用。
    return last_iteration_ > 0 ? SolveStatus::kSuccess : SolveStatus::kNumericalFailure;
  }
  if (status == lbfgs::LBFGSERR_MAXIMUMITERATION) {
    // 达到迭代上限但结果有限：按成功处理，由调用方用独立验证器把关。
    return SolveStatus::kSuccess;
  }
  return SolveStatus::kNumericalFailure;
}

bool MincoOptimizer::optimize(
  const TrajectoryInitialGuess & _guess, const Limits2D & _limits, MincoOptimizeResult & _result)
{
  _result = MincoOptimizeResult();
  if (!configured_) {
    _result.status = SolveStatus::kNotInitialized;
    _result.message = "optimizer not configured";
    return false;
  }
  if (!_limits.valid()) {
    _result.status = SolveStatus::kInvalidInput;
    _result.message = "invalid limits";
    return false;
  }

  const int pieces = _guess.pieceCount();
  if (pieces < 1 || _guess.waypoints.size() != static_cast<std::size_t>(pieces) + 1) {
    _result.status = SolveStatus::kInvalidInput;
    _result.message = "initial guess size mismatch";
    return false;
  }
  // 非有限的段时长必须直接拒绝：encodeVariables() 用 std::max(Tmin, T) 做下限保护，
  // 而 std::max(a, NaN) 会返回 a，NaN 时长会被静默替换成 Tmin，掩盖上游的错误输入。
  for (const double duration : _guess.durations) {
    if (!isFinite(duration)) {
      _result.status = SolveStatus::kInvalidInput;
      _result.message = "non-finite piece duration";
      return false;
    }
  }
  if (
    !_guess.head_position.allFinite() || !_guess.head_velocity.allFinite() ||
    !_guess.head_acceleration.allFinite() || !_guess.tail_position.allFinite() ||
    !_guess.tail_velocity.allFinite() || !_guess.tail_acceleration.allFinite()) {
    _result.status = SolveStatus::kInvalidInput;
    _result.message = "non-finite boundary state";
    return false;
  }

  // 软约束阈值统一取自有效约束（方案 §7.2 的“约束一致性”要求）。
  limits_max_speed_sq_ = _limits.max_linear_speed * _limits.max_linear_speed;
  limits_max_accel_sq_ = _limits.max_linear_accel * _limits.max_linear_accel;

  problem_ = Problem();
  problem_.pieces = pieces;
  problem_.head.setZero();
  problem_.head.col(0) << _guess.head_position.x(), _guess.head_position.y(), 0.0;
  problem_.head.col(1) << _guess.head_velocity.x(), _guess.head_velocity.y(), 0.0;
  problem_.head.col(2) << _guess.head_acceleration.x(), _guess.head_acceleration.y(), 0.0;
  problem_.tail.setZero();
  problem_.tail.col(0) << _guess.tail_position.x(), _guess.tail_position.y(), 0.0;
  problem_.tail.col(1) << _guess.tail_velocity.x(), _guess.tail_velocity.y(), 0.0;
  problem_.tail.col(2) << _guess.tail_acceleration.x(), _guess.tail_acceleration.y(), 0.0;
  problem_.limits = &_limits;

  problem_.reference_points.clear();
  if (config_.w_reference > 0.0) {
    problem_.reference_points.reserve(_guess.waypoints.size());
    for (std::size_t i = 1; i + 1 < _guess.waypoints.size(); ++i) {
      problem_.reference_points.push_back(_guess.waypoints[i]);
    }
  }

  generator_.setConditions(problem_.head, problem_.tail, pieces);

  Eigen::VectorXd x = encodeVariables(_guess);
  if (has_warm_start_ && static_cast<int>(warm_durations_.size()) == pieces) {
    // 热启动：用上一帧的内部路标点与段时长覆盖前端初值。
    const int inner_count = pieces - 1;
    if (static_cast<int>(warm_inner_points_.size()) == inner_count) {
      for (int i = 0; i < inner_count; ++i) {
        x(2 * i) = warm_inner_points_[static_cast<std::size_t>(i)].x();
        x(2 * i + 1) = warm_inner_points_[static_cast<std::size_t>(i)].y();
      }
      for (int i = 0; i < pieces; ++i) {
        const double duration = std::max(
          config_.min_piece_duration * (1.0 + 1.0e-6),
          warm_durations_[static_cast<std::size_t>(i)]);
        x(2 * inner_count + i) = softplusInverse(duration - config_.min_piece_duration);
      }
    }
  }

  double cost = 0.0;
  int iterations = 0;
  SolveStatus status = runStage(true, x, cost, iterations);
  _result.pre_time_ms = last_stage_time_ms_;
  if (status != SolveStatus::kSuccess) {
    _result.status = status;
    _result.message = "PRE stage failed";
    _result.iterations = iterations;
    return false;
  }

  if (config_.two_stage) {
    const SolveStatus fine_status = runStage(false, x, cost, iterations);
    _result.fine_time_ms = last_stage_time_ms_;
    if (fine_status != SolveStatus::kSuccess) {
      _result.status = fine_status;
      _result.message = "FINELY stage failed";
      _result.iterations = iterations;
      return false;
    }
  }

  Trajectory2D trajectory;
  if (!extractTrajectory(trajectory)) {
    _result.status = SolveStatus::kNumericalFailure;
    _result.message = "failed to extract trajectory";
    _result.iterations = iterations;
    return false;
  }

  _result.status = SolveStatus::kSuccess;
  _result.trajectory = std::move(trajectory);
  _result.cost = cost;
  _result.iterations = iterations;
  _result.time_optimized = true;
  return true;
}

bool MincoOptimizer::evaluateObjective(
  const TrajectoryInitialGuess & _guess, const Limits2D & _limits, double & _cost,
  Eigen::VectorXd & _gradient, Trajectory2D * _trajectory)
{
  if (!configured_ || !_limits.valid()) {
    return false;
  }
  const int pieces = _guess.pieceCount();
  if (pieces < 1 || _guess.waypoints.size() != static_cast<std::size_t>(pieces) + 1) {
    return false;
  }
  // 非有限段时长同样直接拒绝（原因见 optimize()：std::max(a, NaN) 会静默取 a）。
  for (const double duration : _guess.durations) {
    if (!isFinite(duration)) {
      return false;
    }
  }

  limits_max_speed_sq_ = _limits.max_linear_speed * _limits.max_linear_speed;
  limits_max_accel_sq_ = _limits.max_linear_accel * _limits.max_linear_accel;

  problem_ = Problem();
  problem_.pieces = pieces;
  problem_.head.setZero();
  problem_.head.col(0) << _guess.head_position.x(), _guess.head_position.y(), 0.0;
  problem_.head.col(1) << _guess.head_velocity.x(), _guess.head_velocity.y(), 0.0;
  problem_.head.col(2) << _guess.head_acceleration.x(), _guess.head_acceleration.y(), 0.0;
  problem_.tail.setZero();
  problem_.tail.col(0) << _guess.tail_position.x(), _guess.tail_position.y(), 0.0;
  problem_.tail.col(1) << _guess.tail_velocity.x(), _guess.tail_velocity.y(), 0.0;
  problem_.tail.col(2) << _guess.tail_acceleration.x(), _guess.tail_acceleration.y(), 0.0;
  problem_.limits = &_limits;
  problem_.reference_points.clear();
  if (config_.w_reference > 0.0) {
    for (std::size_t i = 1; i + 1 < _guess.waypoints.size(); ++i) {
      problem_.reference_points.push_back(_guess.waypoints[i]);
    }
  }
  problem_.use_time_ratio_penalty = config_.w_time_ratio > 0.0;
  problem_.use_fine_heuristic = config_.enable_report_fine_heuristic;

  generator_.setConditions(problem_.head, problem_.tail, pieces);
  const Eigen::VectorXd x = encodeVariables(_guess);
  _cost = evaluate(x, _gradient, true);
  if (!isFinite(_cost)) {
    return false;
  }
  if (_trajectory != nullptr) {
    return extractTrajectory(*_trajectory);
  }
  return true;
}

}  // namespace srm27_minco_core
