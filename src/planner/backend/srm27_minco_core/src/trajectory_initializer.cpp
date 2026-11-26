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

/// \brief 转角速度衰减（与 navi_minco_bit `utils::calCurvatureDecay` 同构）。
///
/// 角度小于死区时不降速；超过饱和值后角速度不再继续下降（避免直角与回头弯的差别被放大）。
/// 衰减形式为 `min_vel + (vmax - min_vel) * exp(-ratio * power)`，`ratio` 取
/// `(min(angle, saturation) - deadzone) / saturation` —— 与参考工程逐字一致（注意分母是
/// `saturation` 而不是 `saturation - deadzone`，这样参数可以直接照抄）。
///
/// \return 该角度下的局部速度上限（m/s），保证不超过 `_vmax`。
double curvatureDecay(
  double _angle, double _vmax, double _deadzone, double _saturation, double _min_vel,
  double _power)
{
  if (!(_angle >= _deadzone)) {
    return _vmax;
  }
  const double saturation = std::max(_saturation, _deadzone + 1.0e-6);
  const double clamped_angle = std::min(_angle, saturation);
  const double ratio = (clamped_angle - _deadzone) / saturation;
  const double min_vel = std::clamp(_min_vel, 0.0, _vmax);
  const double local_vmax = min_vel + (_vmax - min_vel) * std::exp(-ratio * std::max(0.0, _power));
  return std::clamp(local_vmax, min_vel, _vmax);
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

  // 转角降速（2026-11-26 改为 navi_minco_bit 的语义）：
  //
  // 参考工程在**稀疏**路径的顶点上量转向角，再按 `calCurvatureDecay` 求该段的局部速度上限，
  // 并把 `local_vmaxs` 一起交给优化器。本项目输入的是已平滑的密集路径，圆角被展成很多
  // 小转角 —— 逐点夹角只有 1° 左右，旧实现按 `angle / corner_angle_threshold` 线性插值，
  // `blend` 常年贴近 0，所以 `corner_speed_ratio` 改了也不生效（现场实测证实）。
  //
  // 这里用“前后各半窗的弦”的夹角代替顶点夹角：窗口按弧长取固定长度，能把一个圆角
  // 累积的转向角重新量出来，同时对直道上的小幅横向抖动（`improvePathClearance` 的
  // 居中微调）不敏感 —— 那些抖动在 0.4 m 窗口里只有几度，落在死区内。
  double corner_angle_max = 0.0;
  // 转角速度地板：绝对速度与"占上限比例"两种写法，比例优先（见 Config 注释）。
  const double turn_min_vel =
    (_config.min_turn_vel_ratio > 0.0)
      ? clampToRange(_config.min_turn_vel_ratio, 0.0, 1.0) * _config.max_speed
      : _config.min_turn_vel;
  std::vector<double> speed_limit(sample_count, _config.max_speed);
  {
    const double window = std::max(2.0 * _config.resample_step, _config.corner_chord_window);
    const int half =
      std::max(1, static_cast<int>(std::lround(0.5 * window / std::max(1.0e-6, _config.resample_step))));
    const int last = static_cast<int>(sample_count) - 1;
    std::vector<double> heading(sample_count, 0.0);
    std::vector<bool> heading_valid(sample_count, false);
    for (int i = 0; i <= last; ++i) {
      const int a = std::max(0, i - half);
      const int b = std::min(last, i + half);
      if (b <= a) {
        continue;
      }
      const Eigen::Vector2d chord = points[static_cast<std::size_t>(b)] -
                                   points[static_cast<std::size_t>(a)];
      if (chord.norm() < 1.0e-6) {
        continue;
      }
      heading[static_cast<std::size_t>(i)] = std::atan2(chord.y(), chord.x());
      heading_valid[static_cast<std::size_t>(i)] = true;
    }
    for (int i = 0; i <= last; ++i) {
      const std::size_t index = static_cast<std::size_t>(i);
      if (!heading_valid[index]) {
        continue;
      }
      const int prev = std::max(0, i - half);
      const int next = std::min(last, i + half);
      // 用窗口两端的局部切向做差：等价于参考工程在稀疏顶点上量的转角。
      if (!heading_valid[static_cast<std::size_t>(prev)]) {
        continue;
      }
      double delta = heading[index] - heading[static_cast<std::size_t>(prev)];
      delta = std::atan2(std::sin(delta), std::cos(delta));
      const double angle = std::abs(delta);
      if (angle <= _config.turn_angle_deadzone) {
        continue;
      }
      corner_angle_max = std::max(corner_angle_max, angle);
      const double local_vmax = curvatureDecay(
        angle, _config.max_speed, _config.turn_angle_deadzone, _config.turn_angle_saturation,
        turn_min_vel, _config.decay_power);
      speed_limit[index] = std::max(kZeroSpeed, std::min(speed_limit[index], local_vmax));
      // 转向发生在窗口中点附近：把上限也铺到窗口后半段，避免上限只落在一个采样点上，
      // 被后续的等时间取点跳过。
      for (int k = i; k <= next; ++k) {
        speed_limit[static_cast<std::size_t>(k)] =
          std::max(kZeroSpeed, std::min(speed_limit[static_cast<std::size_t>(k)], local_vmax));
      }
    }
  }

  // 逐段上限的可达化（与 navi_minco_bit `utils::VelPropogation` 同序：先后向制动扫描、
  // 再前向加速扫描）。参考工程把 `local_vmaxs` 直接作为优化器的逐段速度约束，而原始
  // 转角上限在弯前一段是“不可达”的 —— 直接拿它当阈值会和起点实测速度自相矛盾
  // （例如车正以 0.5 m/s 进弯，弯所在段的原始上限只有 0.12 m/s，整条轨迹一出发就越界）。
  // 这里把上限本身做成“减速度可达”的剖面，并把起点抬到不低于实测速度。
  const double head_speed = std::min(_head_velocity.norm(), _config.max_speed);
  std::vector<double> limit_profile = speed_limit;
  for (std::size_t i = sample_count - 1; i > 0; --i) {
    const double reachable = std::sqrt(
      limit_profile[i] * limit_profile[i] + 2.0 * _config.max_brake * segment_length[i - 1]);
    limit_profile[i - 1] = std::min(limit_profile[i - 1], reachable);
  }
  limit_profile.front() = std::max(limit_profile.front(), head_speed);
  for (std::size_t i = 1; i < sample_count; ++i) {
    const double reachable = std::sqrt(
      limit_profile[i - 1] * limit_profile[i - 1] + 2.0 * _config.max_accel * segment_length[i - 1]);
    limit_profile[i] = std::min(limit_profile[i], reachable);
  }

  // 速度剖面：前向加速扫描 + 后向制动扫描。
  std::vector<double> speed(sample_count, _config.max_speed);
  const double head_speed_profile = std::min(_head_velocity.norm(), speed_limit.front());
  speed.front() = head_speed_profile;
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
  speed.front() = head_speed_profile;
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
  // 每段起点对应的采样下标，用于统计段内的局部速度上限（见 speed_limits）。
  std::vector<std::size_t> waypoint_sample(static_cast<std::size_t>(pieces) + 1, 0);
  const double piece_duration = total_time / static_cast<double>(pieces);
  waypoints.reserve(static_cast<std::size_t>(pieces) + 1);
  waypoints.push_back(points.front());
  for (int k = 1; k < pieces; ++k) {
    const double target_time = static_cast<double>(k) * piece_duration;
    const std::size_t index = locateTimeIndex(sample_time, target_time);
    waypoint_sample[static_cast<std::size_t>(k)] = index;
    const std::size_t next = std::min(index + 1, sample_count - 1);
    const double span = sample_time[next] - sample_time[index];
    double alpha = 0.0;
    if (span > 1.0e-9) {
      alpha = clampToRange((target_time - sample_time[index]) / span, 0.0, 1.0);
    }
    waypoints.push_back(points[index] + (points[next] - points[index]) * alpha);
  }
  waypoints.push_back(points.back());
  waypoint_sample[static_cast<std::size_t>(pieces)] = sample_count - 1;

  // 末端切向：非全局目标时用它给出末端速度方向。
  Eigen::Vector2d tail_direction = points.back() - points[sample_count - 2];
  if (tail_direction.norm() > 1.0e-9) {
    tail_direction.normalize();
  } else {
    tail_direction = Eigen::Vector2d::Zero();
  }

  _guess.corner_angle_max = corner_angle_max;
  _guess.waypoints = waypoints;
  _guess.durations.assign(static_cast<std::size_t>(pieces), piece_duration);
  // 逐段时长在最小/最大约束内做一次温和的边界修正，保持总时长不变。
  for (double & duration : _guess.durations) {
    duration = clampToRange(duration, _config.min_piece_duration, _config.max_piece_duration);
  }
  // 逐段速度上限 = 段内所有采样点局部上限的最小值。优化器用这个值当该段的速度软约束
  // 阈值，转角降速才会真正留在最优解里，而不是被最优解改回全局上限。
  _guess.speed_limits.assign(static_cast<std::size_t>(pieces), _config.max_speed);
  const bool corner_limited = turn_min_vel < _config.max_speed;
  if (corner_limited) {
    for (int k = 0; k < pieces; ++k) {
      const std::size_t begin = waypoint_sample[static_cast<std::size_t>(k)];
      const std::size_t end = std::max(begin, waypoint_sample[static_cast<std::size_t>(k) + 1]);
      // 段内取**可达化上限**的较大者：一段内的速度可以到该段允许的最高值，
      // 真正需要降速的地方（转角所在段）本身的上限就低。
      double cap = kZeroSpeed;
      for (std::size_t i = begin; i <= end && i < sample_count; ++i) {
        cap = std::max(cap, limit_profile[i]);
      }
      _guess.speed_limits[static_cast<std::size_t>(k)] = std::clamp(cap, kZeroSpeed, _config.max_speed);
    }
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
