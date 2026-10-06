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

#include "srm27_minco_core/tracking_reference.hpp"

#include <algorithm>
#include <cmath>

#include "srm27_minco_core/kinematics.hpp"

namespace srm27_minco_core
{

namespace
{

/// \brief 投影粗搜索的采样点数。
constexpr int kCoarseProjectionSamples = 48;

/// \brief 投影局部精化的迭代次数。
constexpr int kProjectionRefinements = 24;

/// \brief 速度视为零的阈值（m/s）。
constexpr double kZeroSpeed = 1.0e-3;

}  // namespace

bool TrackingReferenceConfig::valid(std::string * _reason) const
{
  const auto fail = [&_reason](const char * _message) {
    if (_reason != nullptr) {
      *_reason = _message;
    }
    return false;
  };
  if (!isFinite(projection_backward_window) || projection_backward_window < 0.0) {
    return fail("projection_backward_window must be non-negative");
  }
  if (!isFinite(projection_forward_window) || projection_forward_window < 0.0) {
    return fail("projection_forward_window must be non-negative");
  }
  if (projection_backward_window + projection_forward_window <= 0.0) {
    return fail("at least one projection window must be positive");
  }
  if (!isFinite(max_progress_step) || max_progress_step <= 0.0) {
    return fail("max_progress_step must be positive");
  }
  if (!isFinite(backtrack_limit) || backtrack_limit < 0.0) {
    return fail("backtrack_limit must be non-negative");
  }
  if (!isFinite(goal_slowdown_time) || goal_slowdown_time < 0.0) {
    return fail("goal_slowdown_time must be non-negative");
  }
  return true;
}

bool TrackingReferenceBuilder::configure(
  const TrackingReferenceConfig & _config, std::string * _reason)
{
  configured_ = false;
  if (!_config.valid(_reason)) {
    return false;
  }
  config_ = _config;
  configured_ = true;
  reset();
  return true;
}

void TrackingReferenceBuilder::reset()
{
  progress_ = 0.0;
  has_progress_ = false;
  last_yaw_ = 0.0;
  has_last_yaw_ = false;
}

double TrackingReferenceBuilder::projectProgress(
  const Trajectory2D & _trajectory, const Eigen::Vector2d & _position, double _previous_progress,
  bool _has_previous) const
{
  if (_trajectory.empty() || !_position.allFinite()) {
    return 0.0;
  }
  const double total = _trajectory.totalDuration();

  double search_low = 0.0;
  double search_high = total;
  if (_has_previous) {
    search_low = std::max(0.0, _previous_progress - config_.projection_backward_window);
    search_high = std::min(total, _previous_progress + config_.projection_forward_window);
  }
  if (search_high - search_low < 1.0e-9) {
    return clampToRange(_previous_progress, 0.0, total);
  }

  const auto distanceSquared = [&_trajectory, &_position](double _t) {
    return (_trajectory.positionAt(_t) - _position).squaredNorm();
  };

  // 粗搜索：在上一次进度附近的连续窗口内找最近点，避免跨自交点跳到远处分支。
  double best_time = search_low;
  double best_distance = distanceSquared(search_low);
  for (int i = 1; i <= kCoarseProjectionSamples; ++i) {
    const double alpha = static_cast<double>(i) / static_cast<double>(kCoarseProjectionSamples);
    const double t = search_low + alpha * (search_high - search_low);
    const double distance = distanceSquared(t);
    if (distance < best_distance) {
      best_distance = distance;
      best_time = t;
    }
  }

  // 局部精化：在最佳粗采样点两侧各半个粗步长内做黄金分割式收缩。
  const double coarse_step =
    (search_high - search_low) / static_cast<double>(kCoarseProjectionSamples);
  double low = std::max(search_low, best_time - coarse_step);
  double high = std::min(search_high, best_time + coarse_step);
  for (int i = 0; i < kProjectionRefinements; ++i) {
    const double third = (high - low) / 3.0;
    const double left = low + third;
    const double right = high - third;
    if (distanceSquared(left) < distanceSquared(right)) {
      high = right;
    } else {
      low = left;
    }
  }
  return clampToRange(0.5 * (low + high), 0.0, total);
}

bool TrackingReferenceBuilder::build(
  const Trajectory2D & _trajectory, const State2D & _state, const MpcModel & _model,
  YawReferenceProvider & _yaw_provider, TrackingReferenceResult & _result)
{
  _result = TrackingReferenceResult();
  if (!configured_ || !_model.configured() || _trajectory.empty()) {
    _result.message = "builder, model or trajectory is not available";
    return false;
  }
  if (!_state.valid || !std::isfinite(_state.x) || !std::isfinite(_state.y)) {
    _result.message = "invalid state";
    return false;
  }

  const double total = _trajectory.totalDuration();
  double projected = projectProgress(_trajectory, _state.position(), progress_, has_progress_);
  if (has_progress_) {
    // 限制单周期进度变化：允许有限回退，禁止无限倒退；前进也受限以防止堵转后追赶墙钟进度。
    projected = clampToRange(
      projected, progress_ - config_.backtrack_limit, progress_ + config_.max_progress_step);
  }
  progress_ = clampToRange(projected, 0.0, total);
  has_progress_ = true;

  std::vector<double> sample_times(static_cast<std::size_t>(_model.steps()), 0.0);
  double time = progress_;
  for (int i = 0; i < _model.steps(); ++i) {
    sample_times[static_cast<std::size_t>(i)] = std::min(time, total);
    time += _model.dt();
  }

  if (!assemble(_trajectory, _state, _model, _yaw_provider, sample_times, _result)) {
    return false;
  }
  _result.progress = progress_;
  return true;
}

bool TrackingReferenceBuilder::buildSecondPass(
  const Trajectory2D & _trajectory, const Eigen::MatrixXd & _predicted_velocity,
  const MpcModel & _model, YawReferenceProvider & _yaw_provider, TrackingReferenceResult & _result)
{
  if (!configured_ || !_model.configured() || _trajectory.empty() || !_result.valid) {
    _result.message = "second pass requires a valid first pass result";
    _result.valid = false;
    return false;
  }
  if (!config_.two_pass_reference) {
    // 配置关闭时不做第二次采样：避免“参数说自己没开、代码却仍在重采样”的语义漂移。
    _result.message = "two_pass_reference is disabled in the configuration";
    _result.valid = false;
    return false;
  }
  const int steps = _model.steps();
  if (
    _predicted_velocity.rows() != 2 || _predicted_velocity.cols() != steps ||
    !_predicted_velocity.allFinite()) {
    _result.message = "predicted velocity has wrong shape";
    _result.valid = false;
    return false;
  }

  const double total = _trajectory.totalDuration();
  std::vector<double> sample_times(static_cast<std::size_t>(steps), 0.0);

  // 报告第 24 页公式：alpha_i = max(0, cos(angle between v_pred and v_ref))，
  // tau_{i+1} = tau_i + h * alpha_i。缩短的是**参考轨迹上的采样增量**，物理步长 h 不变。
  const double h = _model.dt();
  double time = progress_;
  for (int i = 0; i < steps; ++i) {
    sample_times[static_cast<std::size_t>(i)] = std::min(time, total);
    const Eigen::Vector2d reference_velocity = _trajectory.velocityAt(time);
    const Eigen::Vector2d predicted_velocity = _predicted_velocity.col(i);
    const double reference_norm = reference_velocity.norm();
    const double predicted_norm = predicted_velocity.norm();
    double alpha = 1.0;
    if (reference_norm > kZeroSpeed && predicted_norm > kZeroSpeed) {
      alpha = clampToRange(
        predicted_velocity.dot(reference_velocity) / (predicted_norm * reference_norm), 0.0, 1.0);
    } else {
      // 起步/终点附近速度接近零：方向不可靠，保持原采样，避免永远不起步。
      alpha = 1.0;
    }
    time += h * alpha;
  }

  const double previous_progress = progress_;
  const bool previous_has = has_progress_;
  if (!assemble(_trajectory, State2D(), _model, _yaw_provider, sample_times, _result)) {
    progress_ = previous_progress;
    has_progress_ = previous_has;
    return false;
  }
  _result.progress = progress_;
  return true;
}

bool TrackingReferenceBuilder::assemble(
  const Trajectory2D & _trajectory, const State2D & _state, const MpcModel & _model,
  YawReferenceProvider & _yaw_provider, const std::vector<double> & _sample_times,
  TrackingReferenceResult & _result)
{
  const int steps = _model.steps();
  const int nz = _model.stateDim();
  const int nu = _model.inputDim();
  if (static_cast<int>(_sample_times.size()) != steps) {
    _result.message = "sample time count mismatch";
    _result.valid = false;
    return false;
  }

  const double total = _trajectory.totalDuration();
  const bool terminal_stop = _trajectory.terminal_is_global_goal;
  const double slowdown = config_.goal_slowdown_time;

  _result.z_ref = Eigen::MatrixXd::Zero(nz, steps);
  _result.u_ref = Eigen::MatrixXd::Zero(nu, steps);
  _result.tangent_angle.assign(static_cast<std::size_t>(steps), 0.0);

  // 航向必须跨周期连续：第二次采样时 `_state` 可能无效，沿用上一次的连续航向。
  double previous_yaw = _state.valid ? _state.yaw : last_yaw_;
  bool yaw_initialized = _state.valid || has_last_yaw_;
  _result.near_terminal = false;

  for (int i = 0; i < steps; ++i) {
    const double t = _sample_times[static_cast<std::size_t>(i)];
    const bool at_end = (total - t) <= 1.0e-9;
    if (at_end) {
      _result.near_terminal = true;
    }

    // 终点前平滑收敛：位置用末点，速度参考按剩余时间线性收敛到零。
    double taper = 1.0;
    if (terminal_stop && slowdown > 0.0) {
      const double remaining = std::max(0.0, total - t);
      taper = clampToRange(remaining / slowdown, 0.0, 1.0);
      if (taper < 1.0) {
        _result.near_terminal = true;
      }
    }

    const Eigen::Vector2d position = at_end ? _trajectory.endPosition() : _trajectory.positionAt(t);
    const Eigen::Vector2d velocity = at_end ? Eigen::Vector2d::Zero() : _trajectory.velocityAt(t);
    const Eigen::Vector2d acceleration =
      at_end ? Eigen::Vector2d::Zero() : _trajectory.accelerationAt(t);

    Eigen::Vector2d tangent = Eigen::Vector2d::Zero();
    const bool tangent_valid = !at_end && _trajectory.tangentAt(t, tangent);
    if (tangent_valid) {
      _result.tangent_angle[static_cast<std::size_t>(i)] = std::atan2(tangent.y(), tangent.x());
    } else if (i > 0) {
      _result.tangent_angle[static_cast<std::size_t>(i)] =
        _result.tangent_angle[static_cast<std::size_t>(i) - 1];
    }

    double yaw_reference = previous_yaw;
    double omega_reference = 0.0;
    if (!yaw_initialized) {
      previous_yaw = tangent_valid ? std::atan2(tangent.y(), tangent.x()) : 0.0;
      yaw_initialized = true;
    }
    if (!_yaw_provider.yawReference(
          t, position, velocity, tangent_valid, tangent, previous_yaw, yaw_reference,
          omega_reference)) {
      _result.message = "yaw reference provider failed";
      _result.valid = false;
      return false;
    }
    // 航向参考必须连续展开，禁止在 2*pi 分支之间跳变。
    yaw_reference = unwrapAngleNear(previous_yaw, yaw_reference);
    previous_yaw = yaw_reference;

    const double omega_scale = at_end ? 0.0 : taper;

    _result.z_ref.block<2, 1>(0, i) = position;
    _result.z_ref(2, i) = yaw_reference;
    _result.u_ref(2, i) = omega_scale * omega_reference;

    if (nz > 3) {
      // 六维加速度模型：决策量是加速度，不是速度。
      _result.z_ref.block<2, 1>(3, i) = taper * velocity;
      _result.z_ref(5, i) = omega_scale * omega_reference;
      _result.u_ref.block<2, 1>(0, i) = taper * acceleration;
      _result.u_ref(2, i) = 0.0;
    } else {
      // 速度层模型：决策量就是平移速度与角速度。
      _result.u_ref.block<2, 1>(0, i) = taper * velocity;
    }
  }

  if (_model.type() == MpcModelType::kAccelerationDoubleIntegrator && steps > 1) {
    for (int i = 0; i + 1 < steps; ++i) {
      _result.u_ref(2, i) = (_result.z_ref(5, i + 1) - _result.z_ref(5, i)) / _model.dt();
    }
    _result.u_ref(2, steps - 1) = _result.u_ref(2, steps - 2);
  }

  last_yaw_ = previous_yaw;
  has_last_yaw_ = true;

  if (!_result.z_ref.allFinite() || !_result.u_ref.allFinite()) {
    _result.message = "non-finite reference";
    _result.valid = false;
    return false;
  }
  _result.valid = true;
  _result.message.clear();
  return true;
}

}  // namespace srm27_minco_core
