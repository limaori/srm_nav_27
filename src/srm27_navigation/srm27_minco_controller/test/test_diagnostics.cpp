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

#include <gtest/gtest.h>

#include <algorithm>
#include <string>
#include <vector>

#include "srm27_minco_controller/diagnostics.hpp"

/// \file
/// \brief 诊断输出的完整性测试。
///
/// 方案 §11.3 明确要求这些指标必须能输出；这里检查字段确实被写入诊断数组，
/// 防止“文档里有指标、代码里没实现”。

namespace
{

/// \brief 取某个键的值；不存在时返回空字符串。
std::string valueOf(const diagnostic_msgs::msg::DiagnosticArray & _array, const std::string & _key)
{
  for (const diagnostic_msgs::msg::DiagnosticStatus & status : _array.status) {
    for (const diagnostic_msgs::msg::KeyValue & entry : status.values) {
      if (entry.key == _key) {
        return entry.value;
      }
    }
  }
  return std::string();
}

}  // namespace

TEST(DiagnosticsTest, ContainsAllRequiredMetricKeys)
{
  srm27_minco_controller::ControllerDiagnostics diagnostics;
  diagnostics.planning_result = "success";
  diagnostics.terminal_reason = "planning_horizon";
  diagnostics.terminal_speed = 0.8;
  diagnostics.esdf_time_ms = 0.11;
  diagnostics.frontend_time_ms = 0.22;
  diagnostics.pre_time_ms = 1.0;
  diagnostics.finely_time_ms = 2.0;
  diagnostics.validation_time_ms = 0.33;
  diagnostics.planning_time_ms = 3.5;
  diagnostics.trajectory_id = 7;
  diagnostics.goal_epoch = 3;
  diagnostics.map_version = 11;
  diagnostics.path_version = 5;
  diagnostics.limits_version = 2;
  diagnostics.state_age = 0.02;
  diagnostics.map_age = 0.05;
  diagnostics.trajectory_age = 0.07;
  diagnostics.replan_type = "hot_start";
  diagnostics.switched_trajectory_count = 4;
  diagnostics.rejected_stale_result_count = 1;
  diagnostics.missed_deadline_count = 0;
  diagnostics.minimum_clearance = 0.41;
  diagnostics.maximum_speed = 0.49;
  diagnostics.maximum_acceleration = 0.28;
  diagnostics.cross_track_error = 0.03;
  diagnostics.along_track_error = 0.01;
  diagnostics.projection_progress = 1.25;
  diagnostics.mpc_solve_time_pass1_ms = 0.6;
  diagnostics.qp_status = 0;
  diagnostics.qp_iterations = 12;
  diagnostics.max_constraint_violation = 0.0;
  diagnostics.yaw_mode = "xy_only";
  diagnostics.command_owner = "navigation";
  diagnostics.command_age = 0.06;
  diagnostics.applied_speed_scale = 1.0;
  diagnostics.stop_reason = "none";

  const diagnostic_msgs::msg::DiagnosticArray array =
    srm27_minco_controller::toDiagnosticArray(diagnostics, "odom", 0);
  ASSERT_EQ(array.status.size(), 1u);
  EXPECT_EQ(array.header.frame_id, "odom");
  EXPECT_EQ(array.status.front().level, 0);
  EXPECT_EQ(array.status.front().message, "success");
  EXPECT_EQ(valueOf(array, "terminal_reason"), "planning_horizon");
  EXPECT_EQ(valueOf(array, "terminal_speed"), "0.8");

  // 方案 §11.3 列出的指标必须全部存在。
  const std::vector<std::string> required = {
    "planning_result",
    "esdf_time_ms",
    "frontend_time_ms",
    "pre_time_ms",
    "finely_time_ms",
    "validation_time_ms",
    "planning_time_ms",
    "trajectory_id",
    "goal_epoch",
    "map_version",
    "path_version",
    "limits_version",
    "state_age",
    "map_age",
    "trajectory_age",
    "replan_type",
    "switched_trajectory_count",
    "rejected_stale_result_count",
    "missed_deadline_count",
    "minimum_clearance",
    "maximum_speed",
    "maximum_acceleration",
    "cross_track_error",
    "along_track_error",
    "projection_progress",
    "mpc_solve_time_pass1_ms",
    "mpc_solve_time_pass2_ms",
    "total_control_time_ms",
    "qp_status",
    "qp_iterations",
    "max_constraint_violation",
    "yaw_mode",
    "command_owner",
    "command_age",
    "applied_speed_scale",
    "stop_reason",
    "stop_request_time",
    "output_zero_time",
    "actual_stop_time",
    "requested_vx",
    "requested_vy",
    "requested_wz"};
  for (const std::string & key : required) {
    EXPECT_FALSE(valueOf(array, key).empty()) << "缺少诊断字段: " << key;
  }
}

TEST(DiagnosticsTest, FormatsNumbersWithoutLocaleSurprises)
{
  srm27_minco_controller::ControllerDiagnostics diagnostics;
  diagnostics.planning_result = "timeout";
  diagnostics.planning_time_ms = 12.5;
  diagnostics.minimum_clearance = -0.25;

  const diagnostic_msgs::msg::DiagnosticArray array =
    srm27_minco_controller::toDiagnosticArray(diagnostics, "odom", 1);
  EXPECT_EQ(valueOf(array, "planning_result"), "timeout");
  EXPECT_EQ(valueOf(array, "planning_time_ms"), "12.5");
  EXPECT_EQ(valueOf(array, "minimum_clearance"), "-0.25");
  EXPECT_EQ(array.status.front().level, 1);
}

TEST(DiagnosticsTest, CountersAreIntegers)
{
  srm27_minco_controller::ControllerDiagnostics diagnostics;
  diagnostics.trajectory_id = 42;
  diagnostics.qp_iterations = 7;

  const diagnostic_msgs::msg::DiagnosticArray array =
    srm27_minco_controller::toDiagnosticArray(diagnostics, "odom", 0);
  EXPECT_EQ(valueOf(array, "trajectory_id"), "42");
  EXPECT_EQ(valueOf(array, "qp_iterations"), "7");
}
