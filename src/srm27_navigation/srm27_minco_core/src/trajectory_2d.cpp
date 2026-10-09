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

#include "srm27_minco_core/trajectory_2d.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace srm27_minco_core
{

void Trajectory2D::clear()
{
  x_coefficients_.clear();
  y_coefficients_.clear();
  durations_.clear();
  cumulative_time_.assign(1, 0.0);
  total_duration_ = 0.0;
  clearance_samples.clear();
  narrow_intervals.clear();
}

bool Trajectory2D::addPiece(const Coefficients & _x, const Coefficients & _y, double _duration)
{
  if (!isFinite(_duration) || _duration <= 0.0) {
    return false;
  }
  for (int k = 0; k < kCoefficientCount; ++k) {
    if (!isFinite(_x[static_cast<std::size_t>(k)]) || !isFinite(_y[static_cast<std::size_t>(k)])) {
      return false;
    }
  }
  x_coefficients_.push_back(_x);
  y_coefficients_.push_back(_y);
  durations_.push_back(_duration);
  total_duration_ += _duration;
  cumulative_time_.push_back(total_duration_);
  return true;
}

bool Trajectory2D::addPieceHighOrderFirst(
  const Coefficients & _x_high_first, const Coefficients & _y_high_first, double _duration)
{
  Coefficients x{};
  Coefficients y{};
  for (int k = 0; k < kCoefficientCount; ++k) {
    // 输入索引 0 是最高阶 c5，输出索引 k 是 c_k。
    x[static_cast<std::size_t>(k)] =
      _x_high_first[static_cast<std::size_t>(kCoefficientCount - 1 - k)];
    y[static_cast<std::size_t>(k)] =
      _y_high_first[static_cast<std::size_t>(kCoefficientCount - 1 - k)];
  }
  return addPiece(x, y, _duration);
}

bool Trajectory2D::setDurations(const std::vector<double> & _durations)
{
  if (_durations.size() != durations_.size()) {
    return false;
  }
  double total = 0.0;
  std::vector<double> cumulative;
  cumulative.reserve(_durations.size() + 1);
  cumulative.push_back(0.0);
  for (const double duration : _durations) {
    if (!isFinite(duration) || duration <= 0.0) {
      return false;
    }
    total += duration;
    cumulative.push_back(total);
  }
  durations_ = _durations;
  cumulative_time_ = std::move(cumulative);
  total_duration_ = total;
  return true;
}

double Trajectory2D::clampTime(double _t) const
{
  if (!isFinite(_t)) {
    return 0.0;
  }
  return clampToRange(_t, 0.0, total_duration_);
}

void Trajectory2D::locate(double _t, int & _piece_index, double & _tau) const
{
  if (durations_.empty()) {
    _piece_index = -1;
    _tau = 0.0;
    return;
  }
  const double t = clampTime(_t);
  // 使用上界查找定位：位于段边界时归入后一段；末端归入最后一段。
  const auto it = std::upper_bound(cumulative_time_.begin(), cumulative_time_.end(), t);
  int index = static_cast<int>(it - cumulative_time_.begin()) - 1;
  index = std::max(0, std::min(index, pieceCount() - 1));
  _piece_index = index;
  _tau = t - cumulative_time_[static_cast<std::size_t>(index)];
  _tau = clampToRange(_tau, 0.0, durations_[static_cast<std::size_t>(index)]);
}

Eigen::Vector2d Trajectory2D::piecePosition(int _piece_index, double _tau) const
{
  if (_piece_index < 0 || _piece_index >= pieceCount()) {
    return Eigen::Vector2d::Zero();
  }
  const auto & x = x_coefficients_[static_cast<std::size_t>(_piece_index)];
  const auto & y = y_coefficients_[static_cast<std::size_t>(_piece_index)];
  const double tau = clampToRange(_tau, 0.0, durations_[static_cast<std::size_t>(_piece_index)]);
  double px = 0.0;
  double py = 0.0;
  for (int k = kCoefficientCount - 1; k >= 0; --k) {
    px = px * tau + x[static_cast<std::size_t>(k)];
    py = py * tau + y[static_cast<std::size_t>(k)];
  }
  return Eigen::Vector2d(px, py);
}

Eigen::Vector2d Trajectory2D::pieceVelocity(int _piece_index, double _tau) const
{
  if (_piece_index < 0 || _piece_index >= pieceCount()) {
    return Eigen::Vector2d::Zero();
  }
  const auto & x = x_coefficients_[static_cast<std::size_t>(_piece_index)];
  const auto & y = y_coefficients_[static_cast<std::size_t>(_piece_index)];
  const double tau = clampToRange(_tau, 0.0, durations_[static_cast<std::size_t>(_piece_index)]);
  double vx = 0.0;
  double vy = 0.0;
  for (int k = kCoefficientCount - 1; k >= 1; --k) {
    vx = vx * tau + static_cast<double>(k) * x[static_cast<std::size_t>(k)];
    vy = vy * tau + static_cast<double>(k) * y[static_cast<std::size_t>(k)];
  }
  return Eigen::Vector2d(vx, vy);
}

Eigen::Vector2d Trajectory2D::pieceAcceleration(int _piece_index, double _tau) const
{
  if (_piece_index < 0 || _piece_index >= pieceCount()) {
    return Eigen::Vector2d::Zero();
  }
  const auto & x = x_coefficients_[static_cast<std::size_t>(_piece_index)];
  const auto & y = y_coefficients_[static_cast<std::size_t>(_piece_index)];
  const double tau = clampToRange(_tau, 0.0, durations_[static_cast<std::size_t>(_piece_index)]);
  double ax = 0.0;
  double ay = 0.0;
  for (int k = kCoefficientCount - 1; k >= 2; --k) {
    const double factor = static_cast<double>(k) * static_cast<double>(k - 1);
    ax = ax * tau + factor * x[static_cast<std::size_t>(k)];
    ay = ay * tau + factor * y[static_cast<std::size_t>(k)];
  }
  return Eigen::Vector2d(ax, ay);
}

Eigen::Vector2d Trajectory2D::pieceJerk(int _piece_index, double _tau) const
{
  if (_piece_index < 0 || _piece_index >= pieceCount()) {
    return Eigen::Vector2d::Zero();
  }
  const auto & x = x_coefficients_[static_cast<std::size_t>(_piece_index)];
  const auto & y = y_coefficients_[static_cast<std::size_t>(_piece_index)];
  const double tau = clampToRange(_tau, 0.0, durations_[static_cast<std::size_t>(_piece_index)]);
  double jx = 0.0;
  double jy = 0.0;
  for (int k = kCoefficientCount - 1; k >= 3; --k) {
    const double factor =
      static_cast<double>(k) * static_cast<double>(k - 1) * static_cast<double>(k - 2);
    jx = jx * tau + factor * x[static_cast<std::size_t>(k)];
    jy = jy * tau + factor * y[static_cast<std::size_t>(k)];
  }
  return Eigen::Vector2d(jx, jy);
}

Eigen::Vector2d Trajectory2D::positionAt(double _t) const
{
  int index = 0;
  double tau = 0.0;
  locate(_t, index, tau);
  return piecePosition(index, tau);
}

Eigen::Vector2d Trajectory2D::velocityAt(double _t) const
{
  int index = 0;
  double tau = 0.0;
  locate(_t, index, tau);
  return pieceVelocity(index, tau);
}

Eigen::Vector2d Trajectory2D::accelerationAt(double _t) const
{
  int index = 0;
  double tau = 0.0;
  locate(_t, index, tau);
  return pieceAcceleration(index, tau);
}

Eigen::Vector2d Trajectory2D::jerkAt(double _t) const
{
  int index = 0;
  double tau = 0.0;
  locate(_t, index, tau);
  return pieceJerk(index, tau);
}

std::vector<Eigen::Vector2d> Trajectory2D::samplePositions(double _dt) const
{
  std::vector<Eigen::Vector2d> samples;
  if (durations_.empty() || !isFinite(_dt) || _dt <= 0.0) {
    return samples;
  }
  const int count = static_cast<int>(std::floor(total_duration_ / _dt)) + 1;
  samples.reserve(static_cast<std::size_t>(count) + 1);
  for (int i = 0; i <= count; ++i) {
    samples.push_back(positionAt(static_cast<double>(i) * _dt));
  }
  if ((total_duration_ - static_cast<double>(count) * _dt) > 1.0e-9) {
    samples.push_back(endPosition());
  }
  return samples;
}

bool Trajectory2D::tangentAt(double _t, Eigen::Vector2d & _tangent) const
{
  const Eigen::Vector2d velocity = velocityAt(_t);
  const double speed = velocity.norm();
  if (!isFinite(speed) || speed < 1.0e-6) {
    return false;
  }
  _tangent = velocity / speed;
  return true;
}

void Trajectory2D::continuityResiduals(
  double & _max_pos, double & _max_vel, double & _max_acc) const
{
  _max_pos = 0.0;
  _max_vel = 0.0;
  _max_acc = 0.0;
  for (int i = 0; i + 1 < pieceCount(); ++i) {
    const Eigen::Vector2d pos_left = piecePosition(i, durations_[static_cast<std::size_t>(i)]);
    const Eigen::Vector2d pos_right = piecePosition(i + 1, 0.0);
    const Eigen::Vector2d vel_left = pieceVelocity(i, durations_[static_cast<std::size_t>(i)]);
    const Eigen::Vector2d vel_right = pieceVelocity(i + 1, 0.0);
    const Eigen::Vector2d acc_left = pieceAcceleration(i, durations_[static_cast<std::size_t>(i)]);
    const Eigen::Vector2d acc_right = pieceAcceleration(i + 1, 0.0);
    _max_pos = std::max(_max_pos, (pos_left - pos_right).norm());
    _max_vel = std::max(_max_vel, (vel_left - vel_right).norm());
    _max_acc = std::max(_max_acc, (acc_left - acc_right).norm());
  }
}

bool Trajectory2D::sanityCheck(std::string * _reason) const
{
  const auto fail = [&_reason](const char * _message) {
    if (_reason != nullptr) {
      *_reason = _message;
    }
    return false;
  };

  if (durations_.empty()) {
    return fail("trajectory has no pieces");
  }
  if (x_coefficients_.size() != durations_.size() || y_coefficients_.size() != durations_.size()) {
    return fail("coefficient/piece count mismatch");
  }
  if (!isFinite(total_duration_) || total_duration_ <= 0.0) {
    return fail("non-positive total duration");
  }
  for (std::size_t i = 0; i < durations_.size(); ++i) {
    if (!isFinite(durations_[i]) || durations_[i] <= 0.0) {
      return fail("non-positive piece duration");
    }
    for (int k = 0; k < kCoefficientCount; ++k) {
      if (
        !isFinite(x_coefficients_[i][static_cast<std::size_t>(k)]) ||
        !isFinite(y_coefficients_[i][static_cast<std::size_t>(k)])) {
        return fail("non-finite coefficient");
      }
    }
  }
  double max_pos = 0.0;
  double max_vel = 0.0;
  double max_acc = 0.0;
  continuityResiduals(max_pos, max_vel, max_acc);
  if (max_pos > 1.0e-6 || max_vel > 1.0e-5 || max_acc > 1.0e-4) {
    return fail("piece boundary p/v/a discontinuity");
  }
  if (!isFinite(generated_stamp) || !isFinite(valid_after) || !isFinite(valid_until)) {
    return fail("non-finite time metadata");
  }
  // `valid_until == 0` 表示“未设置有效期”，与 `TrajectoryValidator::validate()` 中
  // “valid_until > 0 才检查过期”的语义保持一致；只有真的设置了有效期时才要求顺序正确。
  if (valid_until > 0.0 && valid_until < valid_after) {
    return fail("valid_until earlier than valid_after");
  }
  return true;
}

}  // namespace srm27_minco_core
