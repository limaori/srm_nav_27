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

#include "srm27_minco_core/trajectory_initializer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "gcopter/minco.hpp"
#include "srm27_minco_core/kinematics.hpp"

namespace srm27_minco_core
{

namespace
{

/// \brief 速度视为零的阈值（m/s）。
constexpr double kZeroSpeed = 1.0e-3;

/// \brief 时间轴上的二分查找：返回使 `_times[index] <= _t` 的最大索引。
std::size_t locateTimeIndex(const std::vector<double> & _times, double _t)
{
  if (_times.empty()) {
    return 0;
  }
  const auto it = std::upper_bound(_times.begin(), _times.end(), _t);
  if (it == _times.begin()) {
    return 0;
  }
  const std::size_t index = static_cast<std::size_t>(it - _times.begin()) - 1;
  return std::min(index, _times.size() - 1);
}

}  // namespace

std::vector<Eigen::Vector2d> TrajectoryInitializer::cleanAndResample(
  const std::vector<Eigen::Vector2d> & _polyline, double _duplicate_epsilon, double _step)
{
  std::vector<Eigen::Vector2d> cleaned;
  if (
    _polyline.size() < 2 || !isFinite(_step) || _step <= 0.0 || !isFinite(_duplicate_epsilon) ||
    _duplicate_epsilon < 0.0) {
    return cleaned;
  }

  // 1) 丢弃非有限点与重复点。
  for (const Eigen::Vector2d & point : _polyline) {
    if (!point.allFinite()) {
      continue;
    }
    if (!cleaned.empty() && (point - cleaned.back()).norm() < _duplicate_epsilon) {
      continue;
    }
    cleaned.push_back(point);
  }
  if (cleaned.size() < 2) {
    return {};
  }

  // 2) 丢弃“短小折返/共线抖动”：中间点到其相邻点连线的距离小于阈值时删除该点。
  bool removed = true;
  while (removed && cleaned.size() > 2) {
    removed = false;
    for (std::size_t i = 1; i + 1 < cleaned.size(); ++i) {
      const Eigen::Vector2d a = cleaned[i - 1];
      const Eigen::Vector2d b = cleaned[i];
      const Eigen::Vector2d c = cleaned[i + 1];
      const Eigen::Vector2d ac = c - a;
      const double ac_norm = ac.norm();
      double deviation = 0.0;
      if (ac_norm < _duplicate_epsilon) {
        deviation = (b - a).norm();
      } else {
        const Eigen::Vector2d ab = b - a;
        deviation = std::abs(ac.x() * ab.y() - ac.y() * ab.x()) / ac_norm;
      }
      const double detour = (b - a).norm() + (c - b).norm() - ac_norm;
      if (deviation < _duplicate_epsilon && detour < 2.0 * _duplicate_epsilon) {
        cleaned.erase(cleaned.begin() + static_cast<std::ptrdiff_t>(i));
        removed = true;
        break;
      }
    }
  }
  if (cleaned.size() < 2) {
    return {};
  }

  // 3) 按弧长重采样。
  std::vector<Eigen::Vector2d> resampled;
  resampled.push_back(cleaned.front());
  double carry = 0.0;
  for (std::size_t i = 0; i + 1 < cleaned.size(); ++i) {
    const Eigen::Vector2d a = cleaned[i];
    const Eigen::Vector2d b = cleaned[i + 1];
    const double segment = (b - a).norm();
    if (segment < _duplicate_epsilon) {
      continue;
    }
    const Eigen::Vector2d direction = (b - a) / segment;
    double travelled = carry;
    while (travelled + _step <= segment) {
      travelled += _step;
      resampled.push_back(a + direction * travelled);
    }
    carry = travelled - segment;
  }
  if ((resampled.back() - cleaned.back()).norm() > _duplicate_epsilon) {
    resampled.push_back(cleaned.back());
  }
  if (resampled.size() < 2) {
    return {};
  }
  return resampled;
}

bool TrajectoryInitializer::initialize(
  const std::vector<Eigen::Vector2d> & _polyline, const Eigen::Vector2d & _head_velocity,
  const Config & _config, TrajectoryInitialGuess & _guess, std::string * _reason)
{
  const auto fail = [&_reason](const char * _message) {
    if (_reason != nullptr) {
      *_reason = _message;
    }
    return false;
  };

  if (
    !_config.max_speed || _config.max_speed <= 0.0 || _config.max_accel <= 0.0 ||
    _config.max_brake <= 0.0 || _config.resample_step <= 0.0 || _config.min_piece_duration <= 0.0 ||
    _config.max_piece_duration < _config.min_piece_duration ||
    _config.nominal_piece_duration <= 0.0 || !_head_velocity.allFinite() ||
    !std::isfinite(_config.terminal_speed) || _config.terminal_speed < 0.0) {
    return fail("invalid initializer configuration or head velocity");
  }

  const std::vector<Eigen::Vector2d> points =
    cleanAndResample(_polyline, _config.duplicate_epsilon, _config.resample_step);
  if (points.size() < 2) {
    return fail("polyline too short after cleaning");
  }

  // 每个重采样点到下一个点的距离（除最后一个点外都存在）。
  const std::size_t sample_count = points.size();
  std::vector<double> segment_length(sample_count - 1, 0.0);
  std::vector<double> arc_length(sample_count, 0.0);
  for (std::size_t i = 0; i + 1 < sample_count; ++i) {
    segment_length[i] = (points[i + 1] - points[i]).norm();
    arc_length[i + 1] = arc_length[i] + segment_length[i];
  }
  const double total_length = arc_length.back();
  if (!(total_length > _config.duplicate_epsilon)) {
    return fail("polyline has zero length");
  }

  // 转角降速：用相邻点方向变化估计局部曲率。
  std::vector<double> speed_limit(sample_count, _config.max_speed);
  for (std::size_t i = 0; i < sample_count; ++i) {
    const std::size_t prev = (i == 0) ? 0 : i - 1;
    const std::size_t next = (i + 1 < sample_count) ? i + 1 : sample_count - 1;
    if (prev == next) {
      continue;
    }
    const Eigen::Vector2d d0 = (points[i] - points[prev]);
    const Eigen::Vector2d d1 = (points[next] - points[i]);
    if (d0.norm() < 1.0e-9 || d1.norm() < 1.0e-9) {
      continue;
    }
    const Eigen::Vector2d u0 = d0.normalized();
    const Eigen::Vector2d u1 = d1.normalized();
    const double angle = std::abs(std::atan2(u0.x() * u1.y() - u0.y() * u1.x(), u0.dot(u1)));
    const double blend =
      clampToRange(angle / std::max(1.0e-6, _config.corner_angle_threshold), 0.0, 1.0);
    const double ratio = 1.0 - (1.0 - clampToRange(_config.corner_speed_ratio, 0.05, 1.0)) * blend;
    speed_limit[i] = std::max(kZeroSpeed, _config.max_speed * ratio);
  }

  // 速度剖面：前向加速扫描 + 后向制动扫描。
  std::vector<double> speed(sample_count, _config.max_speed);
  const double head_speed = std::min(_head_velocity.norm(), speed_limit.front());
  speed.front() = head_speed;
  for (std::size_t i = 1; i < sample_count; ++i) {
    const double reachable =
      std::sqrt(speed[i - 1] * speed[i - 1] + 2.0 * _config.max_accel * segment_length[i - 1]);
    speed[i] = std::min(speed_limit[i], reachable);
  }
  const double tail_speed = _config.terminal_is_global_goal
                              ? 0.0
                              : std::clamp(_config.terminal_speed, 0.0, _config.max_speed);
  speed.back() = std::min(speed.back(), tail_speed);
  for (std::size_t i = sample_count - 1; i > 0; --i) {
    const double reachable =
      std::sqrt(speed[i] * speed[i] + 2.0 * _config.max_brake * segment_length[i - 1]);
    speed[i - 1] = std::min(speed[i - 1], reachable);
  }
  // 制动扫描可能把起点速度压到低于实测速度；起点必须与真实状态一致，因此恢复后重扫。
  speed.front() = head_speed;
  for (std::size_t i = 1; i < sample_count; ++i) {
    const double reachable =
      std::sqrt(speed[i - 1] * speed[i - 1] + 2.0 * _config.max_accel * segment_length[i - 1]);
    speed[i] = std::min(speed[i], reachable);
  }

  // 由速度剖面积分时间。两端都接近零的短段使用三角形加减速解，避免除以零。
  std::vector<double> sample_time(sample_count, 0.0);
  for (std::size_t i = 0; i + 1 < sample_count; ++i) {
    const double ds = segment_length[i];
    const double v0 = speed[i];
    const double v1 = speed[i + 1];
    double dt = 0.0;
    if (v0 < kZeroSpeed && v1 < kZeroSpeed) {
      dt = 2.0 * std::sqrt(ds / _config.max_accel);
    } else {
      dt = 2.0 * ds / (v0 + v1);
    }
    if (!isFinite(dt) || dt <= 0.0) {
      dt = ds / std::max(kZeroSpeed, _config.max_speed);
    }
    dt = std::max(dt, ds / _config.max_speed);
    sample_time[i + 1] = sample_time[i] + dt;
  }
  const double total_time = sample_time.back();
  if (!(total_time > 1.0e-3)) {
    return fail("computed total time is degenerate");
  }

  // 选择段数：先按名义段时长，再受最小/最大段时长与上下限约束。
  int pieces = static_cast<int>(std::lround(total_time / _config.nominal_piece_duration));
  pieces = std::max(pieces, static_cast<int>(std::ceil(total_time / _config.max_piece_duration)));
  const int max_by_min_duration =
    static_cast<int>(std::floor(total_time / _config.min_piece_duration));
  if (max_by_min_duration >= 1) {
    pieces = std::min(pieces, max_by_min_duration);
  }
  pieces = std::max(pieces, std::max(1, _config.min_pieces));
  pieces = std::min(pieces, std::max(1, _config.max_pieces));
  if (pieces * 2 > static_cast<int>(sample_count)) {
    pieces = std::max(1, static_cast<int>(sample_count) / 2);
  }

  // 按近似等时间间隔取路标点。
  std::vector<Eigen::Vector2d> waypoints;
  const double piece_duration = total_time / static_cast<double>(pieces);
  waypoints.reserve(static_cast<std::size_t>(pieces) + 1);
  waypoints.push_back(points.front());
  for (int k = 1; k < pieces; ++k) {
    const double target_time = static_cast<double>(k) * piece_duration;
    const std::size_t index = locateTimeIndex(sample_time, target_time);
    const std::size_t next = std::min(index + 1, sample_count - 1);
    const double span = sample_time[next] - sample_time[index];
    double alpha = 0.0;
    if (span > 1.0e-9) {
      alpha = clampToRange((target_time - sample_time[index]) / span, 0.0, 1.0);
    }
    waypoints.push_back(points[index] + (points[next] - points[index]) * alpha);
  }
  waypoints.push_back(points.back());

  // 末端切向：非全局目标时用它给出末端速度方向。
  Eigen::Vector2d tail_direction = points.back() - points[sample_count - 2];
  if (tail_direction.norm() > 1.0e-9) {
    tail_direction.normalize();
  } else {
    tail_direction = Eigen::Vector2d::Zero();
  }

  _guess.waypoints = waypoints;
  _guess.durations.assign(static_cast<std::size_t>(pieces), piece_duration);
  // 逐段时长在最小/最大约束内做一次温和的边界修正，保持总时长不变。
  for (double & duration : _guess.durations) {
    duration = clampToRange(duration, _config.min_piece_duration, _config.max_piece_duration);
  }
  _guess.head_position = points.front();
  _guess.head_velocity = _head_velocity;
  _guess.head_acceleration = Eigen::Vector2d::Zero();
  _guess.tail_position = points.back();
  _guess.tail_velocity = _config.terminal_is_global_goal
                           ? Eigen::Vector2d::Zero()
                           : Eigen::Vector2d(speed.back() * tail_direction);
  _guess.tail_acceleration = Eigen::Vector2d::Zero();
  return true;
}

bool TrajectoryInitializer::buildInitialTrajectory(
  const TrajectoryInitialGuess & _guess, Trajectory2D & _trajectory)
{
  const int pieces = _guess.pieceCount();
  if (pieces < 1 || _guess.waypoints.size() != static_cast<std::size_t>(pieces) + 1) {
    return false;
  }
  for (const double duration : _guess.durations) {
    if (!isFinite(duration) || duration <= 0.0) {
      return false;
    }
  }
  for (const Eigen::Vector2d & point : _guess.waypoints) {
    if (!point.allFinite()) {
      return false;
    }
  }

  Eigen::Matrix3d head = Eigen::Matrix3d::Zero();
  head.col(0) << _guess.head_position.x(), _guess.head_position.y(), 0.0;
  head.col(1) << _guess.head_velocity.x(), _guess.head_velocity.y(), 0.0;
  head.col(2) << _guess.head_acceleration.x(), _guess.head_acceleration.y(), 0.0;

  Eigen::Matrix3d tail = Eigen::Matrix3d::Zero();
  tail.col(0) << _guess.tail_position.x(), _guess.tail_position.y(), 0.0;
  tail.col(1) << _guess.tail_velocity.x(), _guess.tail_velocity.y(), 0.0;
  tail.col(2) << _guess.tail_acceleration.x(), _guess.tail_acceleration.y(), 0.0;

  Eigen::VectorXd durations(pieces);
  for (int i = 0; i < pieces; ++i) {
    durations(i) = _guess.durations[static_cast<std::size_t>(i)];
  }

  Eigen::Matrix3Xd inner(3, std::max(0, pieces - 1));
  for (int i = 0; i < pieces - 1; ++i) {
    const Eigen::Vector2d & point = _guess.waypoints[static_cast<std::size_t>(i) + 1];
    inner.col(i) << point.x(), point.y(), 0.0;
  }

  minco::MINCO_S3NU generator;
  generator.setConditions(head, tail, pieces);
  generator.setParameters(inner, durations);

  // 直接取 MINCO 的系数矩阵：行 6*i+k 是第 i 段的 c_k，顺序已经是 c0..c5，
  // 无需经过 GCOPTER 的 Trajectory 再做一次行序翻转。
  const Eigen::MatrixX3d & coefficients = generator.getCoeffs();
  if (coefficients.rows() != 6 * pieces) {
    return false;
  }

  _trajectory.clear();
  for (int i = 0; i < pieces; ++i) {
    Trajectory2D::Coefficients x{};
    Trajectory2D::Coefficients y{};
    for (int k = 0; k < Trajectory2D::kCoefficientCount; ++k) {
      x[static_cast<std::size_t>(k)] = coefficients(6 * i + k, 0);
      y[static_cast<std::size_t>(k)] = coefficients(6 * i + k, 1);
    }
    if (!_trajectory.addPiece(x, y, _guess.durations[static_cast<std::size_t>(i)])) {
      _trajectory.clear();
      return false;
    }
  }
  return !_trajectory.empty();
}

}  // namespace srm27_minco_core
