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

#include "srm27_behavior/plugins/action/send_nav2_goal.hpp"

#include "srm27_behavior/custom_types.hpp"
#include "srm27_behavior/navigation_spin_tracker.hpp"
#include "srm27_behavior/waypoint_utils.hpp"

namespace srm27_behavior
{

SendNav2GoalAction::SendNav2GoalAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: RosActionNode<nav2_msgs::action::NavigateToPose>(name, conf, params)
{
}

BT::NodeStatus SendNav2GoalAction::tick()
{
  const auto status = BT::RosActionNode<nav2_msgs::action::NavigateToPose>::tick();
  std::shared_ptr<NavigationSpinTracker> tracker;
  if (getInput("tracker_port", tracker) && tracker) {
    std::lock_guard<std::mutex> lock(tracker->mutex);
    tracker->navigation_ready = isGoalAccepted() && status == BT::NodeStatus::RUNNING;
    tracker->syncToBlackboard();
  }
  return status;
}

bool SendNav2GoalAction::setGoal(nav2_msgs::action::NavigateToPose::Goal & goal)
{
  std::string waypoint_name;
  std::string waypoints_file;
  getInput("waypoint_name", waypoint_name);
  getInput("waypoints_file", waypoints_file);
  if (!waypoint_name.empty()) {
    if (!loadNamedWaypoint(waypoints_file, waypoint_name, goal.pose)) {
      RCLCPP_ERROR(logger(), "Unknown waypoint: %s", waypoint_name.c_str());
      return false;
    }
  } else {
    auto pose = getInput<geometry_msgs::msg::PoseStamped>("goal");
    if (!pose) { return false; }
    goal.pose = *pose;
  }
  if (goal.pose.header.frame_id.empty()) { goal.pose.header.frame_id = "map"; }
  goal.pose.header.stamp = now();
  std::shared_ptr<NavigationSpinTracker> tracker;
  if (getInput("tracker_port", tracker) && tracker) { tracker->setNewGoal(goal.pose); }

  return true;
}

BT::NodeStatus SendNav2GoalAction::onResultReceived(const WrappedResult & wr)
{
  switch (wr.code) {
    case rclcpp_action::ResultCode::SUCCEEDED: {
      std::shared_ptr<NavigationSpinTracker> tracker;
      if (getInput("tracker_port", tracker) && tracker && !tracker->markGoalFinished()) {
        config().blackboard->rootBlackboard()->set("nav_fault", std::string("success_outside_tolerance"));
        return BT::NodeStatus::FAILURE;
      }
      RCLCPP_INFO(logger(), "Navigation succeeded!");
      return BT::NodeStatus::SUCCESS;
    }

    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(logger(), "Navigation aborted by server");
      return BT::NodeStatus::FAILURE;

    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(logger(), "Navigation canceled");
      return BT::NodeStatus::FAILURE;

    default:
      RCLCPP_ERROR(logger(), "Unknown navigation result code: %d", static_cast<int>(wr.code));
      return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus SendNav2GoalAction::onFeedback(
  const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
{
  RCLCPP_DEBUG(logger(), "Distance remaining: %f", feedback->distance_remaining);
  return BT::NodeStatus::RUNNING;
}

void SendNav2GoalAction::halt()
{
  BT::RosActionNode<nav2_msgs::action::NavigateToPose>::halt();
}

void SendNav2GoalAction::onHalt()
{
  std::shared_ptr<NavigationSpinTracker> tracker;
  if (getInput("tracker_port", tracker) && tracker) { tracker->clearGoal(); }
}

BT::NodeStatus SendNav2GoalAction::onFailure(BT::ActionNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "SendNav2GoalAction failed: %s", BT::toStr(error));
  return BT::NodeStatus::FAILURE;
}

BT::PortsList SendNav2GoalAction::providedPorts()
{
  BT::PortsList additional_ports = {
    BT::InputPort<std::string>("waypoint_name", "", "Named destination"),
    BT::InputPort<std::string>("waypoints_file", "{@waypoints_file}", "Waypoint YAML"),
    BT::InputPort<std::shared_ptr<NavigationSpinTracker>>(
      "tracker_port", "{@nav_spin_tracker}", "Navigation tracker"),
    BT::InputPort<geometry_msgs::msg::PoseStamped>(
      "goal", "0;0;0", "Expected goal pose that send to nav2. Fill with format `x;y;yaw`"),
  };
  return providedBasicPorts(additional_ports);
}

}  // namespace srm27_behavior

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(srm27_behavior::SendNav2GoalAction, "SendNav2Goal");
