// Copyright 2025 Lihan Chen
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

#include "srm27_behavior/plugins/action/pub_nav2_goal.hpp"

#include <memory>

#include "srm27_behavior/custom_types.hpp"
#include "srm27_behavior/navigation_spin_tracker.hpp"
#include "srm27_behavior/waypoint_utils.hpp"

namespace srm27_behavior
{

PubNav2GoalAction::PubNav2GoalAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: RosTopicPubNode<geometry_msgs::msg::PoseStamped>(name, conf, params)
{
}

bool PubNav2GoalAction::setMessage(geometry_msgs::msg::PoseStamped & msg)
{
  std::string waypoint_name;
  std::string waypoints_file = "params/waypoints.yaml";
  getInput("waypoint_name", waypoint_name);
  getInput("waypoints_file", waypoints_file);

  geometry_msgs::msg::PoseStamped resolved_goal;
  if (!waypoint_name.empty()) {
    if (!loadNamedWaypoint(waypoints_file, waypoint_name, resolved_goal, "map")) {
      RCLCPP_ERROR(
        logger(), "PubNav2Goal: failed to load waypoint_name '%s' from '%s'",
        waypoint_name.c_str(), waypoints_file.c_str());
      return false;
    }
  } else {
    auto goal = getInput<geometry_msgs::msg::PoseStamped>("goal");
    if (!goal) {
      RCLCPP_ERROR(logger(), "Goal pose is not available");
      return false;
    }
    resolved_goal = *goal;
  }

  msg.header.stamp = now();
  msg.header.frame_id = resolved_goal.header.frame_id.empty() ? "map" : resolved_goal.header.frame_id;
  msg.pose = resolved_goal.pose;

  std::shared_ptr<NavigationSpinTracker> tracker;
  if (getInput("tracker_port", tracker) && tracker) {
    tracker->setNewGoal(msg);
  }
  return true;
}

BT::PortsList PubNav2GoalAction::providedPorts()
{
  BT::PortsList additional_ports = {
    BT::InputPort<geometry_msgs::msg::PoseStamped>(
      "goal", "0;0;0", "Expected goal pose that send to nav2. Fill with format `x;y;yaw`"),
    BT::InputPort<std::string>(
      "waypoint_name", "", "Optional named waypoint from waypoints_file (overrides inline goal)"),
    BT::InputPort<std::string>(
      "waypoints_file", "params/waypoints.yaml", "Named waypoints YAML path"),
    BT::InputPort<std::shared_ptr<NavigationSpinTracker>>(
      "tracker_port", "{@nav_spin_tracker}", "Shared NavigationSpinTracker on blackboard"),
  };
  return providedBasicPorts(additional_ports);
}

}  // namespace srm27_behavior

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(srm27_behavior::PubNav2GoalAction, "PubNav2Goal");
