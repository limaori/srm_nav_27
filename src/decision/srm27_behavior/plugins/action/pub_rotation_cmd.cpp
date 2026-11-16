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

#include "srm27_behavior/plugins/action/pub_rotation_cmd.hpp"

#include <memory>
#include <chrono>
#include <cmath>
#include <limits>

#include "srm27_behavior/navigation_spin_tracker.hpp"

namespace srm27_behavior
{

PublishRotationCmdAction::PublishRotationCmdAction(
  const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params)
: RosTopicPubNode<geometry_msgs::msg::Twist>(name, config, params)
{
}

BT::PortsList PublishRotationCmdAction::providedPorts()
{
  return providedBasicPorts({
    BT::InputPort<double>("spin_speed", 0.0, "Angular Z velocity on rotation_cmd (rad/s)"),
    BT::InputPort<bool>("randomize", false, "Use positive nonperiodic spin for HP-drop evasion"),
    BT::InputPort<std::string>("spin_mode", "", "Optional label recorded in behavior/spin_state"),
    BT::InputPort<std::shared_ptr<NavigationSpinTracker>>(
      "tracker_port", "{@nav_spin_tracker}", "Shared NavigationSpinTracker on blackboard"),
  });
}

bool PublishRotationCmdAction::setMessage(geometry_msgs::msg::Twist & msg)
{
  double spin_speed = 0.0;
  std::string spin_mode;
  getInput("spin_speed", spin_speed);
  getInput("spin_mode", spin_mode);
  const bool randomize = getInput<bool>("randomize").value();
  if (!std::isfinite(spin_speed) || spin_speed < 0.0) {
    RCLCPP_ERROR(node_->get_logger(), "Spin speed must be finite and nonnegative");
    return false;
  }

  std::shared_ptr<NavigationSpinTracker> tracker;
  if (getInput("tracker_port", tracker) && tracker) {
    std::lock_guard<std::mutex> lock(tracker->mutex);
    const double now = std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
    if (randomize || tracker->evasion_smoothing) {
      int epoch = 0;
      (void)config().blackboard->rootBlackboard()->get("match_epoch", epoch);
      const double request = spin_speed;
      spin_speed = tracker->spin_reference.sample(now, request, randomize,
        std::numeric_limits<double>::infinity(), epoch, tracker->spin_seed,
        tracker->spin_max_accel, tracker->spin_max_jerk, tracker->evasion_min_speed);
      tracker->evasion_smoothing = randomize || !tracker->spin_reference.settled(request);
    } else {
      // 保留普通恒速请求，并作为受击平滑变速的起点。
      tracker->spin_reference.reset(spin_speed);
    }
    tracker->last_cmd_wz = spin_speed;
    if (!spin_mode.empty()) {
      tracker->last_spin_mode = spin_mode;
    }
  } else if (randomize) {
    RCLCPP_ERROR(node_->get_logger(), "Random spin requires the shared navigation spin tracker");
    return false;
  }

  msg.linear.x = 0.0;
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = spin_speed;
  return true;
}

}  // namespace srm27_behavior

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(srm27_behavior::PublishRotationCmdAction, "PublishRotationCmd");
