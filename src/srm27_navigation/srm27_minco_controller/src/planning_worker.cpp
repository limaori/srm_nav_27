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

#include "srm27_minco_controller/planning_worker.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <sstream>

#include "srm27_minco_controller/local_path.hpp"
#include "srm27_minco_core/kinematics.hpp"

namespace srm27_minco_controller
{

namespace
{

/// \brief 单调时钟（秒），只用于耗时统计。
double steadyNow()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
           std::chrono::steady_clock::now().time_since_epoch())
           .count() *
         1.0e-9;
}

}  // namespace

PlanningWorker::~PlanningWorker() { stop(); }

bool PlanningWorker::start()
{
  if (running_.load()) {
    return true;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    shutdown_ = false;
    has_request_ = false;
    has_result_ = false;
  }
  running_.store(true);
  thread_ = std::thread(&PlanningWorker::run, this);
  return true;
}

void PlanningWorker::stop()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    shutdown_ = true;
    has_request_ = false;
  }
  condition_.notify_all();
  if (thread_.joinable()) {
    thread_.join();
  }
  running_.store(false);
  busy_.store(false);
}

void PlanningWorker::clear()
{
  std::lock_guard<std::mutex> lock(mutex_);
  has_request_ = false;
  has_result_ = false;
  result_ = PlanningResult();
}

bool PlanningWorker::submit(const PlanningRequest & _request)
{
  if (!running_.load()) {
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (shutdown_) {
      return false;
    }
    if (has_request_) {
      // 队列长度取 1：忙时只保留最新请求。
      discarded_stale_.fetch_add(1);
    }
    request_ = _request;
    has_request_ = true;
  }
  condition_.notify_one();
  return true;
}

bool PlanningWorker::takeResult(PlanningResult & _result)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_result_) {
    return false;
  }
  _result = result_;
  has_result_ = false;
  result_ = PlanningResult();
  return true;
}

void PlanningWorker::run()
{
  while (true) {
    PlanningRequest request;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      condition_.wait(lock, [this] { return shutdown_ || has_request_; });
      if (shutdown_) {
        break;
      }
      request = request_;
      has_request_ = false;
      busy_.store(true);
    }

    const PlanningResult result = plan(request);

    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (shutdown_) {
        // 生命周期已停止：不回写任何结果。
        busy_.store(false);
        break;
      }
      result_ = result;
      has_result_ = true;
      busy_.store(false);
    }
  }
}

PlanningResult PlanningWorker::plan(const PlanningRequest & _request)
{
  using srm27_minco_core::MincoOptimizer;
  using srm27_minco_core::MincoOptimizeResult;
  using srm27_minco_core::SolveStatus;
  using srm27_minco_core::Trajectory2D;
  using srm27_minco_core::TrajectoryInitialGuess;
  using srm27_minco_core::TrajectoryInitializer;
  using srm27_minco_core::TrajectoryValidationReport;
  using srm27_minco_core::TrajectoryValidator;

  PlanningResult result;
  result.versions = _request.versions;
  const double begin = steadyNow();

  if (!_request.esdf || !_request.esdf->valid()) {
    result.status = "no_map";
    result.reason = "distance field is not available";
    return result;
  }

  // 1) 局部路径裁剪。
  const double frontend_begin = steadyNow();
  LocalPathConfig path_config;
  path_config.horizon = _request.local_path_horizon;
  path_config.required_clearance =
    _request.validator_config.robot_radius + _request.validator_config.clearance_margin;
  path_config.terminal_speed = _request.initializer_config.terminal_speed;
  path_config.max_speed = _request.limits.max_linear_speed;
  path_config.braking_deceleration = _request.validator_config.braking_deceleration;
  path_config.reaction_latency = _request.validator_config.reaction_latency;
  const LocalPathResult clipped = extractLocalPath(
    _request.path, _request.state.position(), *_request.esdf, path_config,
    _request.terminal_is_global_goal);
  const auto & local_path = clipped.points;
  const bool terminal_reached = clipped.end == LocalPathEnd::kGlobalGoal;
  result.local_path = local_path;
  result.terminal_reason = toString(clipped.end);
  result.terminal_speed = clipped.terminal_speed;
  if (local_path.size() < 2) {
    // 走到这里说明"当前位置已经就是路径终点"或"终点在地图外"。
    // 前者不该报错：由目标检查器和控制器一起完成停车，重复报 no_path 只会制造噪声，
    // 并被上层误判成规划失败。
    const double distance_to_end = _request.path.empty()
                                     ? std::numeric_limits<double>::infinity()
                                     : (_request.path.back() - _request.state.position()).norm();
    const bool at_goal = terminal_reached && distance_to_end <= _request.terminal_reached_radius;
    result.status = at_goal ? "already_at_goal" : "no_path";
    result.reason = at_goal ? "robot is already at the path end"
                            : "local path is empty after clipping: " + result.terminal_reason;
    result.frontend_time_ms = (steadyNow() - frontend_begin) * 1.0e3;
    result.local_path = local_path;
    result.total_time_ms = (steadyNow() - begin) * 1.0e3;
    return result;
  }

  // 2) 前端初值（路标点 + 时间分配）。
  TrajectoryInitializer::Config initializer_config = _request.initializer_config;
  initializer_config.max_speed = _request.limits.max_linear_speed;
  initializer_config.max_accel = _request.limits.max_linear_accel;
  initializer_config.max_brake =
    std::min(_request.limits.max_linear_accel, _request.validator_config.braking_deceleration);
  initializer_config.terminal_is_global_goal = terminal_reached;
  initializer_config.terminal_speed = clipped.terminal_speed;

  TrajectoryInitialGuess guess;
  std::string reason;
  if (!TrajectoryInitializer::initialize(
        local_path, _request.state.velocity, initializer_config, guess, &reason)) {
    result.status = "frontend_failed";
    result.reason = reason;
    result.frontend_time_ms = (steadyNow() - frontend_begin) * 1.0e3;
    result.total_time_ms = (steadyNow() - begin) * 1.0e3;
    return result;
  }
  result.terminal_speed = guess.tail_velocity.norm();
  result.frontend_time_ms = (steadyNow() - frontend_begin) * 1.0e3;

  // 3) MINCO 优化（PRE / FINELY 两阶段）。
  MincoOptimizer optimizer;
  srm27_minco_core::MincoOptimizerConfig minco_config = _request.minco_config;
  if (!optimizer.configure(minco_config, &reason)) {
    result.status = "invalid_config";
    result.reason = reason;
    result.total_time_ms = (steadyNow() - begin) * 1.0e3;
    return result;
  }
  // 距离场以 shared_ptr<const Esdf2D> 传入，工作线程与优化器只读它。
  optimizer.setEsdf(_request.esdf);
  if (_request.use_warm_start) {
    optimizer.setWarmStart(_request.warm_inner_points, _request.warm_durations);
  }

  MincoOptimizeResult optimize_result;
  optimizer.optimize(guess, _request.limits, optimize_result);
  result.pre_time_ms = optimize_result.pre_time_ms;
  result.finely_time_ms = optimize_result.fine_time_ms;
  result.iterations = optimize_result.iterations;
  if (optimize_result.status != SolveStatus::kSuccess) {
    result.status = std::string("optimize_") + srm27_minco_core::toString(optimize_result.status);
    result.reason = optimize_result.message;
    result.total_time_ms = (steadyNow() - begin) * 1.0e3;
    return result;
  }

  // 4) 独立轨迹验证（优化成功 != 轨迹安全）。
  TrajectoryValidator validator;
  // 校验器配置来自请求（由控制器与自身的 validator_ 用同一份配置填入），
  // 只有速度/加速度上限需要跟随本次请求的有效约束。
  srm27_minco_core::TrajectoryValidatorConfig validator_config = _request.validator_config;
  validator_config.max_linear_speed = _request.limits.max_linear_speed;
  validator_config.max_linear_accel = _request.limits.max_linear_accel;
  if (!validator.configure(validator_config, &reason)) {
    result.status = "invalid_validator_config";
    result.reason = reason;
    result.total_time_ms = (steadyNow() - begin) * 1.0e3;
    return result;
  }

  Trajectory2D trajectory = optimize_result.trajectory;

  // 硬保证：MINCO 的速度/加速度是**软约束**，最优点可能轻微越过硬上限；而验证器按硬上限
  // 判定，越限即整条丢弃，现场表现为“反复 recovery、机器人完全不动”。这里做确定性的
  // 时间拉伸修复（同一组路标点与首末边界条件、段时长整体放大 k），把越限压回硬上限以内；
  // 修复后仍会重新计算极值并做完整碰撞与时效校验，不安全就照样丢弃。
  double candidate_speed = 0.0;
  double candidate_accel = 0.0;
  if (validator.computeExtrema(trajectory, candidate_speed, candidate_accel)) {
    for (int attempt = 0; attempt < 2; ++attempt) {
      const double speed_ratio = _request.limits.max_linear_speed > 0.0
                                   ? candidate_speed / _request.limits.max_linear_speed
                                   : 1.0;
      const double accel_ratio = _request.limits.max_linear_accel > 0.0
                                   ? std::sqrt(candidate_accel / _request.limits.max_linear_accel)
                                   : 1.0;
      const double scale = std::max(1.0, std::max(speed_ratio, accel_ratio));
      if (scale <= 1.0 + 1.0e-6) {
        break;
      }
      Trajectory2D repaired;
      if (!optimizer.rescaleDurations(trajectory, scale, repaired)) {
        break;
      }
      trajectory = std::move(repaired);
      ++result.speed_repair_count;
      result.time_scale *= scale;
      if (!validator.computeExtrema(trajectory, candidate_speed, candidate_accel)) {
        break;
      }
    }
  }

  trajectory.versions = _request.versions;
  trajectory.generated_stamp = _request.request_stamp;
  trajectory.valid_after = _request.request_stamp;
  // 必须给出**真正的**有效期：`Trajectory2D::sanityCheck()` 会检查
  // `valid_until >= valid_after`，只把 valid_until 留 0 会让候选轨迹在自检阶段
  // 就被判死（表现为 validation_failed 且说不出具体原因）。
  trajectory.valid_until = _request.request_stamp + _request.validity_window;
  trajectory.terminal_is_global_goal = terminal_reached;
  trajectory.terminal_requires_stop = result.terminal_speed <= 1.0e-6;
  trajectory.terminal_reason = result.terminal_reason;

  const double validation_begin = steadyNow();
  TrajectoryValidationReport report;
  const bool valid =
    validator.validate(trajectory, *_request.esdf, _request.state, _request.request_stamp, report);
  result.validation_time_ms = (steadyNow() - validation_begin) * 1.0e3;
  result.validation = report;
  result.min_clearance = report.min_clearance;
  result.max_speed = report.max_speed;
  result.max_acceleration = report.max_acceleration;

  if (!valid) {
    result.status = "validation_failed";
    // 把"失败点"写清楚：只报一句原因，现场只能猜是几何、动力学还是覆盖不足。
    // 这里带上首次违例的时间与位置（odom 下）以及关键数值，便于直接定位。
    std::ostringstream detail;
    detail.precision(4);
    detail << report.reason << " | terminal=" << result.terminal_reason
           << " terminal_speed=" << result.terminal_speed;
    const double total = trajectory.totalDuration();
    if (report.first_violation_time > 0.0 && report.first_violation_time < total) {
      const Eigen::Vector2d violation_position = trajectory.positionAt(report.first_violation_time);
      detail << " | t=" << report.first_violation_time << "s/ " << total << "s pos=("
             << violation_position.x() << ", " << violation_position.y() << ")";
    }
    detail << " | 最小净空=" << report.min_clearance
           << "m 需要=" << (minco_config.robot_radius + minco_config.clearance_margin) << "m"
           << " 有效前缀=" << report.effective_prefix_duration
           << "s 需要=" << report.required_prefix_duration << "s"
           << " 最大速度=" << report.max_speed << " 最大加速度=" << report.max_acceleration;
    // 覆盖不足是最难归因的一类失败：它取决于**当前速度**（制动要覆盖
    // reaction_latency + v/a_brake）与轨迹的**实际总时长**，两者都不在上面那几个数里。
    // 2026-10-09 实车日志只留下 "有效前缀=0.2611s 需要=0.6s"，无法区分
    // "当前速度太大、制动覆盖不足" 与 "末端没有静止、不能按静止延拓"。
    detail << " | 当前速度=" << (_request.state.valid ? _request.state.velocity.norm() : 0.0)
           << "m/s 制动需覆盖=" << report.required_stop_time << "s"
           << " 轨迹总时长=" << total << "s"
           << " 末端速度=" << trajectory.endVelocity().norm()
           << " 末端加速度=" << trajectory.endAcceleration().norm()
           << " 末端静止=" << (report.ends_at_rest ? "true" : "false");
    result.reason = detail.str();
    result.total_time_ms = (steadyNow() - begin) * 1.0e3;
    return result;
  }

  // 5) 记录净空采样与窄通道区间，供朝向策略与可视化使用。
  const double sample_dt = 0.05;
  for (double t = 0.0; t <= trajectory.totalDuration(); t += sample_dt) {
    const Eigen::Vector2d position = trajectory.positionAt(t);
    const srm27_minco_core::EsdfQueryResult query =
      _request.esdf->query(position.x(), position.y());
    srm27_minco_core::ClearanceSample sample;
    sample.time = t;
    sample.position = position;
    sample.clearance = query.valid ? query.distance : -std::numeric_limits<double>::infinity();
    trajectory.clearance_samples.push_back(sample);
  }

  result.success = true;
  result.status = "success";
  result.trajectory = std::move(trajectory);
  result.total_time_ms = (steadyNow() - begin) * 1.0e3;
  // 端到端时延：从请求提交到结果产出（含队列等待）。
  result.latency_ms = (_request.request_steady > 0.0)
                        ? std::max(0.0, (steadyNow() - _request.request_steady) * 1.0e3)
                        : result.total_time_ms;
  return result;
}

}  // namespace srm27_minco_controller
