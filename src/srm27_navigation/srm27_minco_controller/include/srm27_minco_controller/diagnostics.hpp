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

#ifndef SRM27_MINCO_CONTROLLER__DIAGNOSTICS_HPP_
#define SRM27_MINCO_CONTROLLER__DIAGNOSTICS_HPP_

#include <cstdint>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <string>

namespace srm27_minco_controller
{

/// \brief 控制器诊断字段（方案 §11.3 要求输出的指标）。
///
/// 所有数值都来自实际执行分支，不根据“有没有抛异常”判断成功；请求命令与最终
/// 经过限幅的命令分别记录。
struct ControllerDiagnostics
{
  // 规划阶段
  std::string planning_result{"idle"};
  std::string terminal_reason{"none"};
  double terminal_speed{0.0};
  double esdf_time_ms{0.0};
  double frontend_time_ms{0.0};
  double pre_time_ms{0.0};
  double finely_time_ms{0.0};
  double validation_time_ms{0.0};
  double planning_time_ms{0.0};

  // 版本与时效
  std::uint64_t trajectory_id{0};
  std::uint64_t goal_epoch{0};
  std::uint64_t map_version{0};
  std::uint64_t path_version{0};
  std::uint64_t limits_version{0};
  double state_age{0.0};
  double map_age{0.0};
  double trajectory_age{0.0};

  // 重规划
  std::string replan_type{"none"};
  std::uint64_t switched_trajectory_count{0};
  std::uint64_t rejected_stale_result_count{0};
  std::uint64_t missed_deadline_count{0};

  // 轨迹质量
  double minimum_clearance{0.0};
  double maximum_speed{0.0};
  double maximum_acceleration{0.0};

  // 跟踪误差
  double cross_track_error{0.0};
  double along_track_error{0.0};
  double projection_progress{0.0};

  // MPC
  double mpc_solve_time_pass1_ms{0.0};
  double mpc_solve_time_pass2_ms{0.0};
  double total_control_time_ms{0.0};
  int qp_status{0};
  int qp_iterations{0};
  double max_constraint_violation{0.0};

  // 命令归属与停止
  std::string yaw_mode{"xy_only"};
  std::string command_owner{"navigation"};
  double command_age{0.0};
  double applied_speed_scale{1.0};
  std::string stop_reason{"none"};
  double stop_request_time{0.0};
  double output_zero_time{0.0};
  double actual_stop_time{0.0};

  // 终点收敛（2026-10-09 实车终点振荡诊断）
  /// \brief 当前位姿到路径末点的距离（m）。
  double terminal_distance{0.0};
  /// \brief 是否处于终点急停（已进入成功区域，正在输出零速等待目标检查器判定停稳）。
  bool terminal_active{false};
  /// \brief 本次生效的终点急停进入阈值（m）。
  double terminal_tolerance{0.0};

  /// \brief 请求命令（车体系，m/s 与 rad/s）。
  double requested_vx{0.0};
  double requested_vy{0.0};
  double requested_wz{0.0};
};

/// \brief 把诊断字段填成 `DiagnosticArray`。
diagnostic_msgs::msg::DiagnosticArray toDiagnosticArray(
  const ControllerDiagnostics & _diagnostics, const std::string & _frame_id, int _level_ok);

}  // namespace srm27_minco_controller

#endif  // SRM27_MINCO_CONTROLLER__DIAGNOSTICS_HPP_
