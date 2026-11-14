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

#include "srm27_behavior/plugins/condition/hysteresis_check.hpp"

namespace srm27_behavior
{

HysteresisCheckCondition::HysteresisCheckCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&HysteresisCheckCondition::checkHysteresis, this), config)
{
}

BT::NodeStatus HysteresisCheckCondition::checkHysteresis()
{
  int epoch = 0;
  getInput("match_epoch", epoch);
  if (epoch != match_epoch_) {
    latched_ = false;
    match_epoch_ = epoch;
  }
  auto msg = getInput<rm_decision_interfaces::msg::RobotStatus>("key_port");
  if (!msg) {
    RCLCPP_ERROR(logger_, "RobotStatus message is not available");
    return BT::NodeStatus::FAILURE;
  }

  std::string metric = "hp";
  int trigger_value = 150;
  int exit_value = 360;
  bool trigger_when_below = true;
  bool ignore_zero_hp = true;
  bool exit_allowed = true;

  getInput("metric", metric);
  getInput("trigger_value", trigger_value);
  getInput("exit_value", exit_value);
  getInput("trigger_when_below", trigger_when_below);
  getInput("ignore_zero_hp", ignore_zero_hp);
  getInput("exit_allowed", exit_allowed);

  int value = 0;
  if (ignore_zero_hp && msg->status_valid && msg->current_hp == 0) {
    latched_ = false;
    return BT::NodeStatus::FAILURE;
  }
  if (metric == "hp") {
    if (!msg->status_valid) {
      return BT::NodeStatus::FAILURE;
    }
    value = static_cast<int>(msg->current_hp);
    if (ignore_zero_hp && value <= 0) {
      latched_ = false;
      return BT::NodeStatus::FAILURE;
    }
  } else if (metric == "ammo") {
    if (!msg->ammo_valid) {
      return BT::NodeStatus::FAILURE;
    }
    value = static_cast<int>(msg->projectile_allowance_17mm);
  } else if (metric == "heat") {
    if (!msg->heat_valid) {
      return BT::NodeStatus::FAILURE;
    }
    value = static_cast<int>(msg->shooter_heat);
  } else {
    RCLCPP_ERROR(logger_, "Unsupported metric '%s' (expected hp|ammo|heat)", metric.c_str());
    return BT::NodeStatus::FAILURE;
  }

  if (!latched_) {
    const bool triggered = trigger_when_below ? (value <= trigger_value) : (value >= trigger_value);
    if (triggered) {
      latched_ = true;
      RCLCPP_INFO(
        logger_, "Hysteresis latched on %s=%d (trigger=%d, exit=%d)", metric.c_str(), value,
        trigger_value, exit_value);
    }
  } else {
    const bool exited = trigger_when_below ? (value >= exit_value) : (value <= exit_value);
    if (exited && exit_allowed) {
      latched_ = false;
      RCLCPP_INFO(
        logger_, "Hysteresis released on %s=%d (exit=%d)", metric.c_str(), value, exit_value);
    }
  }

  return latched_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::PortsList HysteresisCheckCondition::providedPorts()
{
  return {
    BT::InputPort<int>("match_epoch", "{@match_epoch}", "Reset history each new match"),
    BT::InputPort<rm_decision_interfaces::msg::RobotStatus>(
      "key_port", "{@referee_robotStatus}", "RobotStatus port on blackboard"),
    BT::InputPort<std::string>("metric", "hp", "Metric to monitor: hp | ammo | heat"),
    BT::InputPort<int>("trigger_value", 150, "Value that latches the condition to SUCCESS"),
    BT::InputPort<int>("exit_value", 360, "Value that releases the latch back to FAILURE"),
    BT::InputPort<bool>(
      "trigger_when_below", true,
      "True: latch when value <= trigger_value and exit when value >= exit_value"),
    BT::InputPort<bool>(
      "ignore_zero_hp", true, "Reset latch when hp == 0 (robot dead)"),
    BT::InputPort<bool>("exit_allowed", true, "Additional evidence required to release the latch"),
  };
}

}  // namespace srm27_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<srm27_behavior::HysteresisCheckCondition>("HysteresisCheck");
}
