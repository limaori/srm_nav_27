// Copyright 2027 SRM
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

#ifndef SRM27_BEHAVIOR__PLUGINS__CONDITION__IS_HP_DROPPING_HPP_
#define SRM27_BEHAVIOR__PLUGINS__CONDITION__IS_HP_DROPPING_HPP_

#include <chrono>
#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rm_decision_interfaces/msg/robot_status.hpp"

namespace srm27_behavior
{

/**
 * @brief Detects rapid HP drop from RobotStatus (fallback when armor damage info is unavailable)
 * and optionally holds SUCCESS for hold_duration_ms so evasion branches do not flicker.
 */
class IsHpDroppingCondition : public BT::SimpleConditionNode
{
public:
  IsHpDroppingCondition(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

private:
  BT::NodeStatus checkHpDropping();

  int match_epoch_ = -1;
  rclcpp::Logger logger_ = rclcpp::get_logger("IsHpDroppingCondition");
  int last_hp_ = -1;
  bool has_last_tick_ = false;
  std::chrono::steady_clock::time_point last_tick_time_{};
  bool has_active_hold_ = false;
  std::chrono::steady_clock::time_point hold_until_{};
};

}  // namespace srm27_behavior

#endif  // SRM27_BEHAVIOR__PLUGINS__CONDITION__IS_HP_DROPPING_HPP_
