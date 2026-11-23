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

#ifndef SRM27_BEHAVIOR__PLUGINS__CONDITION__HYSTERESIS_CHECK_HPP_
#define SRM27_BEHAVIOR__PLUGINS__CONDITION__HYSTERESIS_CHECK_HPP_

#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rm_decision_interfaces/msg/robot_status.hpp"

namespace srm27_behavior
{

/**
 * @brief Hysteresis latch condition on RobotStatus (hp / ammo / heat) to avoid boundary
 * oscillation (e.g., enter supply when HP <= 150, keep returning SUCCESS until HP >= 360).
 */
class HysteresisCheckCondition : public BT::SimpleConditionNode
{
public:
  HysteresisCheckCondition(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

private:
  BT::NodeStatus checkHysteresis();

  int match_epoch_ = -1;
  rclcpp::Logger logger_ = rclcpp::get_logger("HysteresisCheckCondition");
  bool latched_ = false;
};

}  // namespace srm27_behavior

#endif  // SRM27_BEHAVIOR__PLUGINS__CONDITION__HYSTERESIS_CHECK_HPP_
