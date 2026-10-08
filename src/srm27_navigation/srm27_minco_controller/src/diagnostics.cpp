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

#include "srm27_minco_controller/diagnostics.hpp"

#include <sstream>
#include <vector>

namespace srm27_minco_controller
{

namespace
{

/// \brief 追加一个字符串型键值对。
void addString(
  std::vector<diagnostic_msgs::msg::KeyValue> & _values, const std::string & _key,
  const std::string & _value)
{
  diagnostic_msgs::msg::KeyValue entry;
  entry.key = _key;
  entry.value = _value;
  _values.push_back(entry);
}

/// \brief 追加一个数值型键值对（固定 6 位有效数字，避免日志噪声）。
void addNumber(
  std::vector<diagnostic_msgs::msg::KeyValue> & _values, const std::string & _key,
  const double _value)
{
  std::ostringstream stream;
  stream.precision(6);
  stream << _value;
  addString(_values, _key, stream.str());
}

/// \brief 追加一个无符号整数型键值对。
void addCount(
  std::vector<diagnostic_msgs::msg::KeyValue> & _values, const std::string & _key,
  const std::uint64_t _value)
{
  addString(_values, _key, std::to_string(_value));
}

/// \brief 追加一个整型键值对。
void addInteger(
  std::vector<diagnostic_msgs::msg::KeyValue> & _values, const std::string & _key, const int _value)
{
  addString(_values, _key, std::to_string(_value));
}

}  // namespace

diagnostic_msgs::msg::DiagnosticArray toDiagnosticArray(
  const ControllerDiagnostics & _diagnostics, const std::string & _frame_id, int _level_ok)
{
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.frame_id = _frame_id;

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "srm27_minco_controller: MincoMpcController";
  status.hardware_id = "srm27_navigation";
  status.level = static_cast<std::uint8_t>(_level_ok);
  status.message = _diagnostics.planning_result;

  std::vector<diagnostic_msgs::msg::KeyValue> & values = status.values;
  addString(values, "planning_result", _diagnostics.planning_result);
  addString(values, "terminal_reason", _diagnostics.terminal_reason);
  addNumber(values, "terminal_speed", _diagnostics.terminal_speed);
  addNumber(values, "esdf_time_ms", _diagnostics.esdf_time_ms);
  addNumber(values, "frontend_time_ms", _diagnostics.frontend_time_ms);
  addNumber(values, "pre_time_ms", _diagnostics.pre_time_ms);
  addNumber(values, "finely_time_ms", _diagnostics.finely_time_ms);
  addNumber(values, "validation_time_ms", _diagnostics.validation_time_ms);
  addNumber(values, "planning_time_ms", _diagnostics.planning_time_ms);

  addCount(values, "trajectory_id", _diagnostics.trajectory_id);
  addCount(values, "goal_epoch", _diagnostics.goal_epoch);
  addCount(values, "map_version", _diagnostics.map_version);
  addCount(values, "path_version", _diagnostics.path_version);
  addCount(values, "limits_version", _diagnostics.limits_version);
  addNumber(values, "state_age", _diagnostics.state_age);
  addNumber(values, "map_age", _diagnostics.map_age);
  addNumber(values, "trajectory_age", _diagnostics.trajectory_age);

  addString(values, "replan_type", _diagnostics.replan_type);
  addCount(values, "switched_trajectory_count", _diagnostics.switched_trajectory_count);
  addCount(values, "rejected_stale_result_count", _diagnostics.rejected_stale_result_count);
  addCount(values, "missed_deadline_count", _diagnostics.missed_deadline_count);

  addNumber(values, "minimum_clearance", _diagnostics.minimum_clearance);
  addNumber(values, "maximum_speed", _diagnostics.maximum_speed);
  addNumber(values, "maximum_acceleration", _diagnostics.maximum_acceleration);
  addNumber(values, "cross_track_error", _diagnostics.cross_track_error);
  addNumber(values, "along_track_error", _diagnostics.along_track_error);
  addNumber(values, "projection_progress", _diagnostics.projection_progress);

  addNumber(values, "mpc_solve_time_pass1_ms", _diagnostics.mpc_solve_time_pass1_ms);
  addNumber(values, "mpc_solve_time_pass2_ms", _diagnostics.mpc_solve_time_pass2_ms);
  addNumber(values, "total_control_time_ms", _diagnostics.total_control_time_ms);
  addInteger(values, "qp_status", _diagnostics.qp_status);
  addInteger(values, "qp_iterations", _diagnostics.qp_iterations);
  addNumber(values, "max_constraint_violation", _diagnostics.max_constraint_violation);

  addString(values, "yaw_mode", _diagnostics.yaw_mode);
  addString(values, "command_owner", _diagnostics.command_owner);
  addNumber(values, "command_age", _diagnostics.command_age);
  addNumber(values, "applied_speed_scale", _diagnostics.applied_speed_scale);
  addString(values, "stop_reason", _diagnostics.stop_reason);
  addNumber(values, "stop_request_time", _diagnostics.stop_request_time);
  addNumber(values, "output_zero_time", _diagnostics.output_zero_time);
  addNumber(values, "actual_stop_time", _diagnostics.actual_stop_time);

  addNumber(values, "requested_vx", _diagnostics.requested_vx);
  addNumber(values, "requested_vy", _diagnostics.requested_vy);
  addNumber(values, "requested_wz", _diagnostics.requested_wz);

  array.status.push_back(status);
  return array;
}

}  // namespace srm27_minco_controller
