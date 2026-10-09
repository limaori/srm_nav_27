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

#ifndef SRM27_MINCO_CORE__REPLAN_MANAGER_HPP_
#define SRM27_MINCO_CORE__REPLAN_MANAGER_HPP_

#include <Eigen/Core>
#include <cstdint>
#include <string>
#include <vector>

#include "srm27_minco_core/trajectory_2d.hpp"
#include "srm27_minco_core/types.hpp"

namespace srm27_minco_core
{

/// \brief 重规划类型（方案 §6.7）。
enum class ReplanType {
  /// \brief 不需要重规划，继续执行当前轨迹。
  kNone = 0,
  /// \brief 完整重规划：无轨迹、新目标、失去连通性、或旧轨迹已失效。
  kFull,
  /// \brief 仅优化：路径拓扑不变、只是障碍或跟踪状态小变化，用旧路标与时间热启动。
  kHotStart,
  /// \brief 保留安全前缀：只重规划后半段。
  kPrefixReuse,
};

/// \brief 重规划请求。
struct ReplanRequest
{
  /// \brief 请求所属版本集合。
  VersionSet versions{};
  /// \brief 当前状态采样时间（ROS 秒）。
  double state_stamp{0.0};
  /// \brief 当前 ROS 时间（秒）。
  double now_stamp{0.0};
  /// \brief 机器人是否仍有全局目标（取消后为假）。
  bool has_goal{false};
  /// \brief 局部路径是否可用。
  bool has_path{false};
  /// \brief 上一帧轨迹当前是否仍然经过验证（调用方用最新地图复验后的结果）。
  bool previous_trajectory_reusable{false};
  /// \brief 当前投影进度是否可信（与上一次投影的偏差在允许范围内）。
  bool projection_reliable{true};
  /// \brief 当前状态位置（规划坐标系，m），用于状态跳变检测。
  Eigen::Vector2d state_position{Eigen::Vector2d::Zero()};
  /// \brief `state_position` 是否可用；为假时跳过状态跳变检测。
  bool has_state_position{false};
  /// \brief 当前在已提交轨迹上的进度（s），用于计算“剩余可执行时间”。
  double trajectory_progress{0.0};
};

/// \brief 重规划决策。
struct ReplanDecision
{
  /// \brief 决定的重规划类型。
  ReplanType type{ReplanType::kNone};
  /// \brief 是否允许开始规划任务。
  bool allowed{false};
  /// \brief 是否使用热启动初值。
  bool use_warm_start{false};
  /// \brief 是否保留旧轨迹的安全前缀。
  bool reuse_prefix{false};
  /// \brief 决策原因（用于诊断）。
  std::string reason{};
};

/// \brief 重规划触发与提交策略（纯逻辑，不含线程与 ROS）。
///
/// 负责回答两个问题：
///  1. 本周期要不要重规划、用哪种方式；
///  2. 一个候选结果能不能替换当前轨迹（版本、时间与拼接检查）。
///
/// 本类不是线程安全的：由工作线程单独持有。
class ReplanManager
{
public:
  /// \brief 配置。
  struct Config
  {
    /// \brief 周期性重规划间隔（s）。
    double replan_period{0.10};
    /// \brief 地图版本变化后的最小重规划间隔（s），避免 10 Hz 地图把规划压满。
    double map_change_min_period{0.10};
    /// \brief 轨迹允许的最大年龄（s）。
    double trajectory_max_age{0.30};
    /// \brief 保留前缀的最小时长（s）。
    double prefix_reuse_min_duration{0.20};
    /// \brief 判定“状态跳变/定位不可信”的平移偏差阈值（m）。
    double state_jump_threshold{0.50};
    /// \brief 地图版本相对当前轨迹前进多少代后必须重规划。
    std::uint64_t map_version_replan_delta{1};
  };

  /// \brief 设置配置。
  bool configure(const Config & _config, std::string * _reason = nullptr);

  /// \brief 复位：清除当前轨迹、版本与计时（新目标、取消、失活时调用）。
  void reset();

  /// \brief 记录一次成功提交的轨迹。
  void onPlanCommitted(
    const VersionSet & _versions, const Trajectory2D & _trajectory, double _now_stamp);

  /// \brief 当前是否有已提交并仍可用的轨迹。
  bool hasCommittedTrajectory() const { return has_committed_; }

  /// \brief 已提交轨迹的版本。
  const VersionSet & committedVersions() const { return committed_versions_; }

  /// \brief 已提交轨迹的提交时刻（ROS 秒）。
  double committedStamp() const { return committed_stamp_; }

  /// \brief 评估是否需要重规划以及方式。
  ReplanDecision evaluate(const ReplanRequest & _request) const;

  /// \brief 判断候选轨迹能否替换当前轨迹。
  /// \param _candidate 候选轨迹。
  /// \param _versions 候选所属版本集合。
  /// \param _now_stamp 当前 ROS 时间。
  /// \param _reason 失败原因（可为 nullptr）。
  bool canCommit(
    const Trajectory2D & _candidate, const VersionSet & _versions, double _now_stamp,
    std::string * _reason) const;

private:
  Config config_{};
  bool configured_{false};
  bool has_committed_{false};
  VersionSet committed_versions_{};
  Trajectory2D committed_trajectory_{};
  double committed_stamp_{0.0};
  double last_plan_stamp_{0.0};
  bool has_last_plan_{false};
  std::uint64_t last_planned_map_version_{0};
  bool has_planned_map_version_{false};
};

/// \brief 重规划类型的稳定字符串（诊断用）。
inline const char * toString(const ReplanType _type)
{
  switch (_type) {
    case ReplanType::kNone:
      return "none";
    case ReplanType::kFull:
      return "full";
    case ReplanType::kHotStart:
      return "hot_start";
    case ReplanType::kPrefixReuse:
      return "prefix_reuse";
  }
  return "unknown";
}

}  // namespace srm27_minco_core

#endif  // SRM27_MINCO_CORE__REPLAN_MANAGER_HPP_
