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

#ifndef SRM27_MINCO_CORE__TRAJECTORY_VALIDATOR_HPP_
#define SRM27_MINCO_CORE__TRAJECTORY_VALIDATOR_HPP_

#include <Eigen/Core>
#include <string>

#include "srm27_minco_core/esdf_2d.hpp"
#include "srm27_minco_core/trajectory_2d.hpp"
#include "srm27_minco_core/types.hpp"

namespace srm27_minco_core
{

/// \brief 轨迹验证配置。
///
/// 几何全部使用**原始占用 + 真实包络半径**，不使用 costmap 膨胀半径，
/// 避免机器人半径被重复计入（方案 §6.1、§13）。
struct TrajectoryValidatorConfig
{
  /// \brief 机器人碰撞包络半径（m），来自本车模型。
  double robot_radius{0.33};
  /// \brief 额外净空裕量（m）。
  double clearance_margin{0.05};

  /// \brief 平移合速度硬上限（m/s）。
  double max_linear_speed{0.5};
  /// \brief 平移合加速度硬上限（m/s^2）。
  double max_linear_accel{0.3};
  /// \brief 速度上限的相对余量（0.05 表示允许超过 5% 才判失败）。
  double speed_tolerance_ratio{0.02};
  /// \brief 加速度上限的相对余量。
  double accel_tolerance_ratio{0.02};

  /// \brief 段时长下限（s）。
  double min_piece_duration{0.05};

  /// \brief 碰撞扫描的基础时间步长（s）；实际步长还会按分辨率与最大速度收紧。
  double sample_dt{0.02};
  /// \brief 相邻采样点的最大允许位移（m），防止高速时“跳过”薄障碍。
  double max_sample_spacing{0.02};

  /// \brief 未知区域是否不可通行。
  bool unknown_is_obstacle{true};

  /// \brief 实测可保证的制动减速度（m/s^2），用于计算所需停车时间；必须来自辨识。
  double braking_deceleration{0.3};
  /// \brief 感知与控制总延迟（s）。
  double reaction_latency{0.10};
  /// \brief 可执行前缀必须覆盖的最短时长（s），由调用方按 MPC 窗口设置。
  double required_prefix_duration{0.6};

  /// \brief 地图数据允许的最大年龄（s）。
  double map_timeout{0.30};
  /// \brief 轨迹允许的最大年龄（s）。
  double trajectory_max_age{0.30};

  /// \brief 配置自检。
  bool valid(std::string * _reason = nullptr) const;
};

/// \brief 轨迹验证报告。
struct TrajectoryValidationReport
{
  /// \brief 是否全部通过。
  bool valid{false};
  /// \brief 失败原因（第一处失败）。
  std::string reason{};

  /// \brief 全段最大平移速率（m/s）。
  double max_speed{0.0};
  /// \brief 全段最大平移加速度（m/s^2）。
  double max_acceleration{0.0};
  /// \brief 全段最小净空（m，自由区为正）。
  double min_clearance{0.0};
  /// \brief 最小净空出现的时间（s）。
  double min_clearance_time{0.0};
  /// \brief 首次违反净空要求的时间（s）；无违例时为轨迹总时长。
  double first_violation_time{0.0};
  /// \brief 在最新地图上仍然安全的可执行前缀时长（s）。
  double effective_prefix_duration{0.0};
  /// \brief 需要覆盖的前缀时长（MPC 窗口与停车时间中的较大者，s）。
  double required_prefix_duration{0.0};
  /// \brief 所需停车时间（s），含反应延迟。
  double required_stop_time{0.0};

  bool coefficients_ok{false};
  bool boundary_ok{false};
  bool continuity_ok{false};
  bool dynamics_ok{false};
  bool collision_ok{false};
  bool coverage_ok{false};
  bool timing_ok{false};

  /// \brief 段间残差（m / m/s / m/s^2），用于诊断拼接质量。
  double continuity_max_position{0.0};
  double continuity_max_velocity{0.0};
  double continuity_max_acceleration{0.0};
};

/// \brief 发布前的独立轨迹验证（方案 §6.6）。
///
/// 优化器返回成功不等于轨迹安全：本类做系数/时间有限性、边界与段间 p/v/a 连续性、
/// 速度/加速度极值（多项式求根，保底加密采样）、全段扫掠净空、有效前缀长度与时效
/// 检查。任一失败都应丢弃候选轨迹。
class TrajectoryValidator
{
public:
  /// \brief 配置校验器。
  bool configure(const TrajectoryValidatorConfig & _config, std::string * _reason = nullptr);

  /// \brief 是否已配置。
  bool configured() const { return configured_; }

  /// \brief 当前配置。
  const TrajectoryValidatorConfig & config() const { return config_; }

  /// \brief 完整验证。
  /// \param _trajectory 候选轨迹。
  /// \param _esdf 最新距离场（地图版本用于时效判断）。
  /// \param _state 当前状态（用于计算停车时间与覆盖需求）；可为无效状态。
  /// \param _now_stamp 当前 ROS 时间（秒）。
  /// \param _report 输出报告。
  bool validate(
    const Trajectory2D & _trajectory, const Esdf2D & _esdf, const State2D & _state,
    double _now_stamp, TrajectoryValidationReport & _report) const;

  /// \brief 计算全段速度/加速度极值（多项式求根 + 端点）。
  ///
  /// 每个分量的多项式在段内是低阶多项式，速度模长平方是 8 次多项式，求其导数的
  /// 实根即可得到极值点，再与端点比较；这比纯采样更可靠。
  bool computeExtrema(
    const Trajectory2D & _trajectory, double & _max_speed, double & _max_acceleration) const;

  /// \brief 只在给定时间区间内做碰撞扫描。
  /// \param _from_time 起始时间（s）。
  /// \param _to_time 结束时间（s）。
  /// \param _min_clearance 输出该区间最小净空（m）。
  /// \param _first_violation_time 输出首次违反净空要求的时间（s）；无违例时为 `_to_time`。
  /// \return 区间内全部满足净空要求时返回 true。
  bool checkCollision(
    const Trajectory2D & _trajectory, const Esdf2D & _esdf, double _from_time, double _to_time,
    double & _min_clearance, double & _first_violation_time) const;

private:
  TrajectoryValidatorConfig config_{};
  bool configured_{false};
};

}  // namespace srm27_minco_core

#endif  // SRM27_MINCO_CORE__TRAJECTORY_VALIDATOR_HPP_
