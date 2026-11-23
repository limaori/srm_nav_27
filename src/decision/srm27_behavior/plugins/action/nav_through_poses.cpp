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

#include "srm27_behavior/plugins/action/nav_through_poses.hpp"

#include <memory>
#include <vector>

#include "srm27_behavior/navigation_spin_tracker.hpp"
#include "srm27_behavior/waypoint_utils.hpp"

namespace srm27_behavior
{

NavigateThroughPosesAction::NavigateThroughPosesAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: RosActionNode<nav2_msgs::action::NavigateThroughPoses>(name, conf, params)
{
}

BT::NodeStatus NavigateThroughPosesAction::tick()
{
  const auto status = BT::RosActionNode<nav2_msgs::action::NavigateThroughPoses>::tick();
  std::shared_ptr<NavigationSpinTracker> tracker;
  if (getInput("tracker_port", tracker) && tracker) {
    std::lock_guard<std::mutex> lock(tracker->mutex);
    tracker->navigation_ready = isGoalAccepted() && status == BT::NodeStatus::RUNNING;
    tracker->syncToBlackboard();
  }
  return status;
}

bool NavigateThroughPosesAction::setGoal(Goal & goal)
{
  std::string frame_id = "map";
  std::string waypoint_file;
  std::string waypoint_group;
  std::string waypoints_config = "params/waypoints.yaml";
  std::string goals_str;

  getInput("frame_id", frame_id);
  getInput("waypoint_file", waypoint_file);
  getInput("waypoint_group", waypoint_group);
  getInput("waypoints_config", waypoints_config);
  getInput("goals", goals_str);

  std::vector<geometry_msgs::msg::PoseStamped> waypoints;
  if (!waypoint_group.empty()) {
    if (!loadNamedWaypointGroup(waypoints_config, waypoint_group, waypoints, frame_id)) {
      RCLCPP_ERROR(
        logger(), "NavigateThroughPoses: failed to load waypoint_group '%s' from '%s'",
        waypoint_group.c_str(), waypoints_config.c_str());
      return false;
    }
  } else if (!waypoint_file.empty()) {
    if (!loadWaypointsFromFile(waypoint_file, waypoints, frame_id)) {
      RCLCPP_ERROR(
        logger(), "NavigateThroughPoses: failed to load waypoint_file '%s'",
        waypoint_file.c_str());
      return false;
    }
  } else if (!parseInlineWaypoints(goals_str, waypoints, frame_id)) {
    RCLCPP_ERROR(
      logger(), "NavigateThroughPoses: neither valid 'waypoint_group', 'waypoint_file' nor 'goals' provided");
    return false;
  }

  const auto stamp = now();
  for (auto & wp : waypoints) {
    if (wp.header.frame_id.empty()) {
      wp.header.frame_id = frame_id;
    }
    wp.header.stamp = stamp;
  }

  goal.poses = waypoints;
  std::shared_ptr<NavigationSpinTracker> tracker;
  if (getInput("tracker_port", tracker) && tracker && !goal.poses.empty()) {
    tracker->setNewGoal(goal.poses.back());
  }
  RCLCPP_INFO(logger(), "NavigateThroughPoses: sending %zu waypoints", goal.poses.size());
  return true;
}

BT::NodeStatus NavigateThroughPosesAction::onResultReceived(const WrappedResult & wr)
{
  if (wr.code == rclcpp_action::ResultCode::SUCCEEDED) {
    std::shared_ptr<NavigationSpinTracker> tracker;
    if (getInput("tracker_port", tracker) && tracker) {
      if (!tracker->markGoalFinished()) {
        config().blackboard->rootBlackboard()->set("nav_fault", std::string("success_outside_tolerance"));
        return BT::NodeStatus::FAILURE;
      }
    }
    RCLCPP_INFO(logger(), "NavigateThroughPoses: completed");
    return BT::NodeStatus::SUCCESS;
  }
  RCLCPP_WARN(
    logger(), "NavigateThroughPoses: ended with result code %d", static_cast<int>(wr.code));
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus NavigateThroughPosesAction::onFeedback(
  const std::shared_ptr<const nav2_msgs::action::NavigateThroughPoses::Feedback> feedback)
{
  setOutput("remaining_waypoints", static_cast<int>(feedback->number_of_poses_remaining));
  return BT::NodeStatus::RUNNING;
}

void NavigateThroughPosesAction::halt()
{
  BT::RosActionNode<nav2_msgs::action::NavigateThroughPoses>::halt();
}

void NavigateThroughPosesAction::onHalt()
{
  std::shared_ptr<NavigationSpinTracker> tracker;
  if (getInput("tracker_port", tracker) && tracker) { tracker->clearGoal(); }
}

BT::NodeStatus NavigateThroughPosesAction::onFailure(BT::ActionNodeErrorCode error)
{
  RCLCPP_ERROR(
    logger(), "NavigateThroughPosesAction failed with error: %s", BT::toStr(error));
  return BT::NodeStatus::FAILURE;
}

BT::PortsList NavigateThroughPosesAction::providedPorts()
{
  return providedBasicPorts({
    BT::InputPort<std::string>(
      "waypoint_group", "", "Named group in waypoints_config (params/waypoints.yaml)"),
    BT::InputPort<std::string>(
      "waypoints_config", "{@waypoints_file}", "Named waypoints YAML path"),
    BT::InputPort<std::string>(
      "waypoint_file", "", "Mission YAML (missions/*.yaml) or CSV file path"),
    BT::InputPort<std::string>(
      "goals", "", "Inline waypoints: 'x1;y1;yaw1|x2;y2;yaw2|...'"),
    BT::InputPort<std::string>("frame_id", "map", "Waypoints frame_id"),
    BT::InputPort<std::shared_ptr<NavigationSpinTracker>>(
      "tracker_port", "{@nav_spin_tracker}", "Shared NavigationSpinTracker on blackboard"),
    BT::OutputPort<int>("remaining_waypoints", "Remaining waypoints from Nav2 feedback"),
  });
}

}  // namespace srm27_behavior

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(srm27_behavior::NavigateThroughPosesAction, "NavigateThroughPoses");
