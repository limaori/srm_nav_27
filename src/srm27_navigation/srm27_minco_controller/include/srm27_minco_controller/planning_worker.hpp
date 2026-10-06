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

#ifndef SRM27_MINCO_CONTROLLER__PLANNING_WORKER_HPP_
#define SRM27_MINCO_CONTROLLER__PLANNING_WORKER_HPP_

#include <Eigen/Core>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "srm27_minco_core/esdf_2d.hpp"
#include "srm27_minco_core/minco_optimizer.hpp"
#include "srm27_minco_core/trajectory_2d.hpp"
#include "srm27_minco_core/trajectory_initializer.hpp"
#include "srm27_minco_core/trajectory_validator.hpp"
#include "srm27_minco_core/types.hpp"

namespace srm27_minco_controller
{

/// \brief 一次规划任务的输入（不可变快照）。
struct PlanningRequest
{
  /// \brief 版本集合（目标会话 / 路径 / 地图 / 限速）。
  srm27_minco_core::VersionSet versions{};
  /// \brief 局部参考折线（规划坐标系，m）。调用方已完成裁剪与坐标系转换。
  std::vector<Eigen::Vector2d> path{};
  /// \brief 请求时刻的状态（规划坐标系）。
  srm27_minco_core::State2D state{};
  /// \brief 有效约束快照。
  srm27_minco_core::Limits2D limits{};
  /// \brief MINCO 优化器配置快照（避免求解中途参数变化）。
  srm27_minco_core::MincoOptimizerConfig minco_config{};
  /// \brief 前端配置快照。
  srm27_minco_core::TrajectoryInitializer::Config initializer_config{};
  /// \brief 局部终点是否就是全局导航目标。
  bool terminal_is_global_goal{true};
  /// \brief 请求时的 ROS 时间（秒）。
  double request_stamp{0.0};
  /// \brief 请求提交时刻的单调时间（秒），只用于端到端时延统计。
  double request_steady{0.0};
  /// \brief 局部路径截取视野长度（m）。
  double local_path_horizon{2.0};
  /// \brief 地图/距离场快照（由控制线程在锁内复制后建立，工作线程只读）。
  std::shared_ptr<const srm27_minco_core::Esdf2D> esdf{};
  /// \brief 是否使用热启动初值。
  bool use_warm_start{false};
  /// \brief 热启动内部路标点（不含首末点）。
  std::vector<Eigen::Vector2d> warm_inner_points{};
  /// \brief 热启动段时长（s）。
  std::vector<double> warm_durations{};
  /// \brief 规划类型（诊断用）。
  int replan_type{0};
};

/// \brief 一次规划任务的结果。
struct PlanningResult
{
  /// \brief 是否得到一条通过独立验证的轨迹。
  bool success{false};
  /// \brief 版本集合。
  srm27_minco_core::VersionSet versions{};
  /// \brief 候选轨迹（仅在 `success` 为真时可用）。
  srm27_minco_core::Trajectory2D trajectory{};
  /// \brief 验证报告。
  srm27_minco_core::TrajectoryValidationReport validation{};
  /// \brief 求解状态字符串。
  std::string status{"none"};
  /// \brief 失败原因或附加说明。
  std::string reason{};
  /// \brief 各阶段耗时（ms）。
  double esdf_time_ms{0.0};
  double frontend_time_ms{0.0};
  double pre_time_ms{0.0};
  double finely_time_ms{0.0};
  double validation_time_ms{0.0};
  double total_time_ms{0.0};
  /// \brief 优化迭代次数。
  int iterations{0};
  /// \brief 优化后的最小净空与最大速度/加速度。
  double min_clearance{0.0};
  double max_speed{0.0};
  double max_acceleration{0.0};
  /// \brief 端到端时延：从请求到结果产出（ms）。
  double latency_ms{0.0};
};

/// \brief 后台规划线程：MINCO 前端 + 优化 + 独立验证。
///
/// 生命周期（方案 §4.4、§4.5.3）：
///  * `start()` / `stop()` 由插件的 activate / deactivate 驱动；`stop()` 会 join 线程，
///    非分离线程，退出后不再回写任何结果。
///  * 任务队列长度取 1：忙时只保留最新请求，旧请求被替换。
///  * 工作线程**只产出轨迹**，不发布任何速度，也没有自己的定时器。
class PlanningWorker
{
public:
  PlanningWorker() = default;
  ~PlanningWorker();

  /// \brief 启动线程。
  bool start();

  /// \brief 停止并 join 线程；可重复调用。
  void stop();

  /// \brief 是否在运行。
  bool running() const { return running_.load(); }

  /// \brief 提交请求；队列中未处理的请求会被替换。
  /// \return 线程未运行时返回 false。
  bool submit(const PlanningRequest & _request);

  /// \brief 取走最新结果（非阻塞）。
  /// \return 有结果时返回 true，并清空内部结果缓存。
  bool takeResult(PlanningResult & _result);

  /// \brief 丢弃未处理的请求与结果（新目标/取消/失活时调用）。
  void clear();

  /// \brief 已丢弃的“过期结果”计数（诊断用）。
  std::uint64_t discardedStaleCount() const { return discarded_stale_.load(); }

  /// \brief 是否正在执行任务。
  bool busy() const { return busy_.load(); }

private:
  /// \brief 线程主循环。
  void run();

  /// \brief 执行一次规划（纯计算，不触碰 ROS 与锁）。
  static PlanningResult plan(const PlanningRequest & _request);

  /// \brief 从全局/局部折线中截取局部可执行段（方案 §6.3 第一阶段）。
  ///
  /// 规则：从当前投影附近开始，按弧长累计到 `_horizon`，遇到地图外（距离场查询
  /// 无效）立即截断，保证不会把规划段指向地图边界以外。
  static std::vector<Eigen::Vector2d> extractLocalPath(
    const std::vector<Eigen::Vector2d> & _path, const Eigen::Vector2d & _position, double _horizon,
    const srm27_minco_core::Esdf2D & _esdf, bool _terminal_is_global_goal,
    bool * _terminal_reached);

  std::thread thread_{};
  mutable std::mutex mutex_{};
  std::condition_variable condition_{};
  bool has_request_{false};
  bool shutdown_{false};
  PlanningRequest request_{};
  bool has_result_{false};
  PlanningResult result_{};
  std::atomic<bool> running_{false};
  std::atomic<bool> busy_{false};
  std::atomic<std::uint64_t> discarded_stale_{0};
};

}  // namespace srm27_minco_controller

#endif  // SRM27_MINCO_CONTROLLER__PLANNING_WORKER_HPP_
