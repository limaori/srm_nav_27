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

#include "srm27_behavior/plugins/action/set_sentry_posture.hpp"

#include <algorithm>

namespace srm27_behavior
{

SetSentryPostureAction::SetSentryPostureAction(
  const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params)
: RosTopicPubNode<std_msgs::msg::UInt8>(name, config, params)
{
}

BT::PortsList SetSentryPostureAction::providedPorts()
{
  return providedBasicPorts({
    BT::InputPort<std::string>(
      "posture", "MOVE", "Desired RMUC posture: MOVE|ATTACK|DEFENSE or 0|1|2"),
    BT::InputPort<std::string>(
      "fallback_posture", "",
      "Optional fallback posture when desired posture approaches 180s degradation cap"),
    BT::InputPort<double>("cooldown_sec", 5.0, "Minimum interval between posture switches (s)"),
    BT::InputPort<double>(
      "degrade_limit_sec", 175.0,
      "Cumulative seconds threshold before triggering fallback_posture (<=0 disables)"),
    BT::InputPort<bool>(
      "reset", false, "Reset posture cooldown and cumulative usage counters to MOVE (0)"),
    BT::InputPort<std::shared_ptr<SentryPostureState>>(
      "state_port", "{@sentry_posture_state}", "Shared posture state tracker on blackboard"),
    BT::OutputPort<int>(
      "active_posture", "{@current_posture}", "Currently active posture ID (0|1|2)"),
  });
}

bool SetSentryPostureAction::setMessage(std_msgs::msg::UInt8 & msg)
{
  std::shared_ptr<SentryPostureState> state;
  if (!getInput("state_port", state) || !state) {
    state = std::make_shared<SentryPostureState>();
    config().blackboard->set("sentry_posture_state", state);
  }

  bool reset = false;
  getInput("reset", reset);
  if (reset) {
    state->reset();
    setOutput("active_posture", 0);
    msg.data = 0;
    return true;
  }

  std::string posture_str = "MOVE";
  std::string fallback_str;
  double cooldown_sec = 5.0;
  double degrade_limit_sec = 175.0;
  getInput("posture", posture_str);
  getInput("fallback_posture", fallback_str);
  getInput("cooldown_sec", cooldown_sec);
  getInput("degrade_limit_sec", degrade_limit_sec);

  uint8_t desired = 0;
  if (!parseSentryPosture(posture_str, desired)) {
    RCLCPP_ERROR(
      node_->get_logger(), "SetSentryPosture: invalid posture '%s'", posture_str.c_str());
    return false;
  }

  const rclcpp::Time now = node_->now();
  std::lock_guard<std::mutex> lock(state->mutex);

  if (state->has_last_tick && now.get_clock_type() == state->last_tick_time.get_clock_type()) {
    const double dt = (now - state->last_tick_time).seconds();
    if (dt > 0.0 && state->active_posture < 3) {
      state->used_sec[state->active_posture] += dt;
    }
  }
  state->last_tick_time = now;
  state->has_last_tick = true;

  uint8_t target = desired;
  uint8_t fallback = desired;
  if (
    degrade_limit_sec > 0.0 && state->used_sec[desired] >= degrade_limit_sec &&
    !fallback_str.empty() && parseSentryPosture(fallback_str, fallback) && fallback != desired &&
    (state->used_sec[fallback] < degrade_limit_sec || state->active_posture == fallback))
  {
    target = fallback;
  }

  if (target != state->active_posture) {
    bool cooldown_elapsed = !state->has_switched || cooldown_sec <= 0.0;
    if (!cooldown_elapsed && now.get_clock_type() == state->last_switch_time.get_clock_type()) {
      const double since_switch = (now - state->last_switch_time).seconds();
      if (since_switch < 0.0) {
        state->last_switch_time = now;
      } else if (since_switch >= cooldown_sec) {
        cooldown_elapsed = true;
      }
    }

    if (cooldown_elapsed) {
      const uint8_t prev = state->active_posture;
      state->active_posture = target;
      state->last_switch_time = now;
      state->has_switched = true;
      RCLCPP_INFO(
        node_->get_logger(),
        "Sentry posture %s -> %s (used s: MOVE=%.1f ATTACK=%.1f DEFENSE=%.1f)",
        sentryPostureName(prev), sentryPostureName(target), state->used_sec[0], state->used_sec[1],
        state->used_sec[2]);
    }
  }

  setOutput("active_posture", static_cast<int>(state->active_posture));
  msg.data = state->active_posture;
  return true;
}

}  // namespace srm27_behavior

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(srm27_behavior::SetSentryPostureAction, "SetSentryPosture");
