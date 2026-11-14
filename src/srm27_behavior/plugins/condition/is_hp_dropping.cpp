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

#include "srm27_behavior/plugins/condition/is_hp_dropping.hpp"

namespace srm27_behavior
{

IsHpDroppingCondition::IsHpDroppingCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsHpDroppingCondition::checkHpDropping, this), config)
{
}

BT::NodeStatus IsHpDroppingCondition::checkHpDropping()
{
  int epoch = 0;
  getInput("match_epoch", epoch);
  if (epoch != match_epoch_) {
    last_hp_ = -1; has_last_tick_ = false; has_active_hold_ = false;
    match_epoch_ = epoch;
  }
  int hp_drop_threshold = 10;
  int hold_duration_ms = 2000;
  int stale_reset_ms = 800;
  getInput("hp_drop_threshold", hp_drop_threshold);
  getInput("hold_duration_ms", hold_duration_ms);
  getInput("stale_reset_ms", stale_reset_ms);

  auto msg = getInput<rm_decision_interfaces::msg::RobotStatus>("key_port");
  if (!msg) {
    RCLCPP_ERROR(logger_, "RobotStatus message is not available");
    return BT::NodeStatus::FAILURE;
  }

  if (!msg->status_valid) {
    last_hp_ = -1;
    has_last_tick_ = false;
    has_active_hold_ = false;
    return BT::NodeStatus::FAILURE;
  }

  const int current_hp = static_cast<int>(msg->current_hp);
  const auto now = std::chrono::steady_clock::now();

  // If this branch was dormant (e.g. preempted by supply recovery or pre-game),
  // re-baseline last_hp_ instead of comparing against a stale HP value.
  if (
    !has_last_tick_ || last_hp_ < 0 ||
    (stale_reset_ms > 0 &&
     std::chrono::duration_cast<std::chrono::milliseconds>(now - last_tick_time_).count() >
       stale_reset_ms))
  {
    last_hp_ = current_hp;
    last_tick_time_ = now;
    has_last_tick_ = true;
    has_active_hold_ = false;
    return BT::NodeStatus::FAILURE;
  }

  last_tick_time_ = now;
  const int drop = last_hp_ - current_hp;
  last_hp_ = current_hp;

  if (current_hp == 0 || drop < 0) {
    has_active_hold_ = false;
    return BT::NodeStatus::FAILURE;
  }

  if (drop >= hp_drop_threshold && hp_drop_threshold > 0) {
    RCLCPP_INFO(
      logger_, "HP dropped by %d (threshold=%d, current_hp=%d)", drop, hp_drop_threshold,
      current_hp);
    if (hold_duration_ms > 0) {
      has_active_hold_ = true;
      hold_until_ = now + std::chrono::milliseconds(hold_duration_ms);
    }
    return BT::NodeStatus::SUCCESS;
  }

  if (has_active_hold_) {
    if (now < hold_until_) {
      return BT::NodeStatus::SUCCESS;
    }
    has_active_hold_ = false;
  }

  return BT::NodeStatus::FAILURE;
}

BT::PortsList IsHpDroppingCondition::providedPorts()
{
  return {
    BT::InputPort<int>("match_epoch", "{@match_epoch}", "Reset history each new match"),
    BT::InputPort<rm_decision_interfaces::msg::RobotStatus>(
      "key_port", "{@referee_robotStatus}", "RobotStatus port on blackboard"),
    BT::InputPort<int>(
      "hp_drop_threshold", 10, "Minimum HP drop between ticks to trigger evasion"),
    BT::InputPort<int>(
      "hold_duration_ms", 2000, "Duration (ms) to hold SUCCESS after an HP drop event"),
    BT::InputPort<int>(
      "stale_reset_ms", 800,
      "Re-baseline last_hp if gap between ticks exceeds this threshold (ms)"),
  };
}

}  // namespace srm27_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<srm27_behavior::IsHpDroppingCondition>("IsHpDropping");
}
