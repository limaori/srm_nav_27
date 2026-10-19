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

#include "srm27_minco_core/recovery_decider.hpp"

#include <cmath>
#include <cstdio>
#include <limits>

namespace srm27_minco_core
{

namespace
{

/// \brief 参考实现在梯度平台分支打的黄字日志。
///
/// 刻意保留成 stderr 而不是引入 ROS 日志：本类在 core 包内**不依赖 rclcpp**，
/// 控制器与行为树插件都要能用。调用方在自己的日志里会同时记录脱困事件。
void logCircularSearch(const double _best_clearance, const double _best_angle_deg)
{
  std::fprintf(
    stderr,
    "\033[33m[MincoRecovery] Flat gradient detected, executing circular search recovery! "
    "Max ESDF: %.4f at angle %.1f deg\033[0m\n",
    _best_clearance, _best_angle_deg);
  std::fflush(stderr);
}

}  // namespace

void RecoveryDecider::configure(const RecoveryConfig & _config)
{
  // 与参考实现一致：非法输入夹到安全值，不抛异常。参数来自 yaml，抛异常会让
  // 整个控制器 configure 失败，代价远大于"用一个保守默认值继续跑"。
  config_.fail_threshold = (_config.fail_threshold > 0) ? _config.fail_threshold : 1;
  config_.cooldown_sec = (std::isfinite(_config.cooldown_sec) && _config.cooldown_sec >= 0.0)
                           ? _config.cooldown_sec
                           : 0.0;
  config_.window_sec = (std::isfinite(_config.window_sec) && _config.window_sec > 0.0)
                         ? _config.window_sec
                         : 0.5;
  config_.escape_speed = (std::isfinite(_config.escape_speed) && _config.escape_speed > 0.0)
                           ? _config.escape_speed
                           : 0.4;
  config_.stuck_clearance =
    (std::isfinite(_config.stuck_clearance) && _config.stuck_clearance > 0.0)
      ? _config.stuck_clearance
      : 0.25;
  config_.escaped_clearance =
    (std::isfinite(_config.escaped_clearance) && _config.escaped_clearance > 0.0)
      ? _config.escaped_clearance
      : 0.40;
  // 成功阈值必须严格大于卡死阈值，否则"进入脱困"与"脱困成功"会在同一个净空上
  // 同时成立，脱困状态一进一出形成高频振荡。这里强制拉开一个栅格量级的间隔。
  if (config_.escaped_clearance <= config_.stuck_clearance) {
    config_.escaped_clearance = config_.stuck_clearance + 0.15;
  }
  config_.search_radius = (std::isfinite(_config.search_radius) && _config.search_radius > 0.0)
                            ? _config.search_radius
                            : 0.4;
  config_.search_samples = (_config.search_samples > 0) ? _config.search_samples : 36;
  config_.gradient_epsilon =
    (std::isfinite(_config.gradient_epsilon) && _config.gradient_epsilon >= 0.0)
      ? _config.gradient_epsilon
      : 1e-3;
}

void RecoveryDecider::reset()
{
  consecutive_failures_ = 0;
  last_failure_sec_ = -1.0;
  last_recovery_start_sec_ = -1.0;
  last_recovery_end_sec_ = -1.0;
  recovery_active_ = false;
  escape_velocity_.setZero();
  last_escape_direction_.setZero();
  entry_clearance_ = -1.0;
  last_trigger_ = "none";
  last_exit_ = "none";
  used_circular_search_ = false;
  circular_best_clearance_ = -1.0;
}

// -----------------------------------------------------------------------------
// 1) 脱困判定
// -----------------------------------------------------------------------------

bool RecoveryDecider::onPlanningFailure(double _now_sec)
{
  if (!isTimeValid(_now_sec)) {
    return false;
  }

  ++consecutive_failures_;
  last_failure_sec_ = _now_sec;

  return shouldStartRecovery(_now_sec);
}

void RecoveryDecider::onPlanningSuccess()
{
  consecutive_failures_ = 0;
  last_failure_sec_ = -1.0;
}

bool RecoveryDecider::shouldStartRecovery(double _now_sec) const
{
  if (!isTimeValid(_now_sec) || recovery_active_) {
    return false;
  }
  if (consecutive_failures_ < config_.fail_threshold) {
    return false;
  }
  // 从未脱困过（-1）时冷却不适用；这与参考实现 `last_recovery_end_time_ < 0.0` 的分支等价，
  // 同时天然覆盖了"时间基准倒流"的异常（结束时刻大于当前时刻 => 冷却不满足）。
  if (last_recovery_end_sec_ < 0.0) {
    return true;
  }
  return (_now_sec - last_recovery_end_sec_) >= config_.cooldown_sec;
}

RecoveryDecision RecoveryDecider::decide(
  double _now_sec, const Eigen::Vector2d & _position, const Esdf2D & _esdf,
  Eigen::Vector2d * _escape_velocity)
{
  if (_escape_velocity != nullptr) {
    _escape_velocity->setZero();
  }
  if (!isTimeValid(_now_sec) || !_position.allFinite()) {
    return RecoveryDecision::kNone;
  }
  if (!onPlanningFailure(_now_sec)) {
    // 门限未到，或已经在一次脱困会话里（此时应由 updateRecovery 推进，而不是重新进入）。
    return RecoveryDecision::kNone;
  }

  // 门限已跨过：开一次脱困会话。**先开会话再算方向**，与参考实现一致
  // （参考实现的 handleReplanFailure 里 onReplanFailure 已经调用了 startRecovery），
  // 这样即使方向算不出来也不会在下一个周期把计数再累加一遍。
  recovery_active_ = true;
  last_recovery_start_sec_ = _now_sec;

  const EsdfQueryResult here = _esdf.query(_position.x(), _position.y());
  entry_clearance_ = here.valid ? here.distance : std::numeric_limits<double>::quiet_NaN();

  Eigen::Vector2d direction = Eigen::Vector2d::Zero();
  double best_clearance = -1.0;
  used_circular_search_ = false;
  circular_best_clearance_ = -1.0;
  if (!planEscapeDirection(_esdf, _position, config_, &direction, &best_clearance)) {
    // 没有方向的脱困等于停车。这里必须让状态"可自愈"：
    //  * 关掉会话（否则 shouldStartRecovery 恒为 false，参考实现在这一分支返回
    //    ENTER_EMER_STOP 且把 recovery_active_ 永久锁死，是同一个坑）；
    //  * **不动失败计数**，让它保持在门限之上，于是冷却一过就能立刻再试一次，
    //    而不是从头再累计 fail_threshold 次失败。
    recovery_active_ = false;
    last_recovery_start_sec_ = -1.0;
    last_recovery_end_sec_ = _now_sec;
    last_trigger_ = "no_escape_direction";
    return RecoveryDecision::kNoEscapeDirection;
  }

  escape_velocity_ = direction * config_.escape_speed;
  last_escape_direction_ = direction;
  last_trigger_ = used_circular_search_ ? "circular_search" : "gradient";
  if (_escape_velocity != nullptr) {
    *_escape_velocity = escape_velocity_;
  }
  return RecoveryDecision::kEscape;
}

// -----------------------------------------------------------------------------
// 2) 脱困方向规划
// -----------------------------------------------------------------------------

bool RecoveryDecider::planEscapeDirection(
  const Esdf2D & _esdf, const Eigen::Vector2d & _position, const RecoveryConfig & _config,
  Eigen::Vector2d * _direction, double * _best_clearance, bool _log_transitions)
{
  if (_direction == nullptr) {
    return false;
  }
  _direction->setZero();
  if (_best_clearance != nullptr) {
    *_best_clearance = -1.0;
  }
  if (!_position.allFinite() || !_esdf.valid()) {
    return false;
  }

  const EsdfQueryResult here = _esdf.query(_position.x(), _position.y());
  if (!here.valid) {
    // 机器人在距离场覆盖范围之外：没有任何可靠的方向信息。
    return false;
  }

  if (here.gradient_valid && here.gradient.allFinite()) {
    const double norm = here.gradient.norm();
    if (std::isfinite(norm) && norm >= _config.gradient_epsilon) {
      *_direction = here.gradient / norm;
      if (_best_clearance != nullptr) {
        *_best_clearance = here.distance;
      }
      return true;
    }
  }

  // 梯度平台（深陷障碍内部时双线性插值会给出零梯度，纯开阔区同理）。
  // 退化为环形搜索：取邻域内净空最大的方向。
  const int samples = (_config.search_samples > 0) ? _config.search_samples : 36;
  const double radius = (_config.search_radius > 0.0) ? _config.search_radius : 0.4;
  const double step = 2.0 * M_PI / static_cast<double>(samples);
  double max_clearance = -std::numeric_limits<double>::max();
  double best_angle = 0.0;
  bool found = false;
  for (int i = 0; i < samples; ++i) {
    const double angle = step * static_cast<double>(i);
    const EsdfQueryResult sample = _esdf.query(
      _position.x() + radius * std::cos(angle), _position.y() + radius * std::sin(angle));
    if (!sample.valid || !std::isfinite(sample.distance)) {
      continue;
    }
    found = true;
    if (sample.distance > max_clearance) {
      max_clearance = sample.distance;
      best_angle = angle;
    }
  }
  if (!found) {
    return false;
  }

  *_direction = Eigen::Vector2d(std::cos(best_angle), std::sin(best_angle));
  if (_best_clearance != nullptr) {
    *_best_clearance = max_clearance;
  }
  if (_log_transitions) {
    logCircularSearch(max_clearance, best_angle * 180.0 / M_PI);
  }
  return true;
}

// -----------------------------------------------------------------------------
// 2b) 脱困路径（沿方向场前推，供可视化）
// -----------------------------------------------------------------------------

bool RecoveryDecider::traceEscapePath(
  const Esdf2D & _esdf, const Eigen::Vector2d & _start, const RecoveryConfig & _config,
  double _step, int _max_steps, std::vector<Eigen::Vector2d> * _path)
{
  if (_path == nullptr) {
    return false;
  }
  _path->clear();
  if (!_start.allFinite() || !_esdf.valid()) {
    return false;
  }
  _path->push_back(_start);

  const double step = (std::isfinite(_step) && _step > 0.0) ? _step : 0.1;
  const int max_steps = (_max_steps > 0) ? _max_steps : 64;

  Eigen::Vector2d current = _start;
  for (int i = 0; i < max_steps; ++i) {
    // 每步都重算方向：这正是"路径可能不是直线"的来源，也是它比一根箭头更有信息量的原因。
    // `_log_transitions = false`：这是可视化用的预演，不该在每个控制周期刷环形搜索日志。
    Eigen::Vector2d direction = Eigen::Vector2d::Zero();
    double best_clearance = -1.0;
    if (!planEscapeDirection(
          _esdf, current, _config, &direction, &best_clearance, /*_log_transitions=*/false)) {
      // 走到方向场给不出方向的区域（地图外 / 全方向采样无效）：路径就到这里。
      return _path->size() > 1;
    }

    current += direction * step;
    if (!current.allFinite()) {
      return _path->size() > 1;
    }
    _path->push_back(current);

    // 已经走出卡死区就收尾：再往前是正常导航该走的路，不属于"脱困路径"。
    const EsdfQueryResult here = _esdf.query(current.x(), current.y());
    if (!here.valid) {
      return true;
    }
    if (std::isfinite(here.distance) && here.distance > _config.escaped_clearance) {
      return true;
    }
  }
  return _path->size() > 1;
}

// -----------------------------------------------------------------------------
// 3) 脱困成功判定
// -----------------------------------------------------------------------------

bool RecoveryDecider::inRecovery(double _now_sec) const
{
  if (!recovery_active_ || !isTimeValid(_now_sec) || last_recovery_start_sec_ < 0.0) {
    return false;
  }
  return (_now_sec - last_recovery_start_sec_) <= config_.window_sec;
}

void RecoveryDecider::finishRecovery(bool _success, double _now_sec)
{
  recovery_active_ = false;
  if (_success) {
    consecutive_failures_ = 0;
    last_failure_sec_ = -1.0;
  }
  if (isTimeValid(_now_sec)) {
    last_recovery_end_sec_ = _now_sec;
  }
}

RecoveryExit RecoveryDecider::updateRecovery(
  double _now_sec, const Eigen::Vector2d & _position, const Esdf2D & _esdf, double _clearance,
  Eigen::Vector2d * _escape_velocity, bool _log_transitions)
{
  if (!recovery_active_) {
    return RecoveryExit::kNone;
  }
  if (!isTimeValid(_now_sec)) {
    return RecoveryExit::kNone;
  }

  // 顺序固定：先判成功，再判超时。
  //
  // 判据是**单帧、单点**的位置判据（与参考实现一致）：只看自身净空，不校验速度、
  // 也不校验"新轨迹是否可解"。超时后回到重规划，由重规划自己证明是否真的脱困。
  if (std::isfinite(_clearance) && _clearance > config_.escaped_clearance) {
    last_exit_ = "escaped";
    finishRecovery(true, _now_sec);
    if (_escape_velocity != nullptr) {
      _escape_velocity->setZero();
    }
    return RecoveryExit::kEscaped;
  }

  if (!inRecovery(_now_sec)) {
    last_exit_ = "timeout";
    finishRecovery(false, _now_sec);
    if (_escape_velocity != nullptr) {
      _escape_velocity->setZero();
    }
    return RecoveryExit::kTimeout;
  }

  // 按**当前位置**重算方向（见头文件说明：不能沿用进入脱困时那一个）。
  // 刻意放在成功/超时判定**之后**：已经该退出的周期没必要再花一次
  // ESDF 环形搜索（最多 36 次查询）——这一点在 20~50 Hz 的控制周期里是有意义的。
  Eigen::Vector2d direction = Eigen::Vector2d::Zero();
  double best_clearance = -1.0;
  if (planEscapeDirection(
        _esdf, _position, config_, &direction, &best_clearance, _log_transitions)) {
    escape_velocity_ = direction * config_.escape_speed;
    last_escape_direction_ = direction;
    const EsdfQueryResult here = _esdf.query(_position.x(), _position.y());
    used_circular_search_ =
      !(here.gradient_valid && here.gradient.allFinite() &&
        here.gradient.norm() >= config_.gradient_epsilon);
    circular_best_clearance_ = best_clearance;
  }
  // 算不出方向时沿用上一周期的速度：这一步不比"原地停住"更差，
  // 而且窗口照常计时、必然超时退出，不会卡死在这里。

  if (_escape_velocity != nullptr) {
    *_escape_velocity = escape_velocity_;
  }
  return RecoveryExit::kNone;
}

// -----------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------

bool RecoveryDecider::isTimeValid(double _now_sec)
{
  return std::isfinite(_now_sec) && _now_sec >= 0.0;
}

const char * toString(RecoveryDecision _decision)
{
  switch (_decision) {
    case RecoveryDecision::kNone:
      return "none";
    case RecoveryDecision::kEscape:
      return "escape";
    case RecoveryDecision::kNoEscapeDirection:
      return "no_escape_direction";
  }
  return "unknown";
}

const char * toString(RecoveryExit _exit)
{
  switch (_exit) {
    case RecoveryExit::kNone:
      return "none";
    case RecoveryExit::kEscaped:
      return "escaped";
    case RecoveryExit::kTimeout:
      return "timeout";
  }
  return "unknown";
}

}  // namespace srm27_minco_core
