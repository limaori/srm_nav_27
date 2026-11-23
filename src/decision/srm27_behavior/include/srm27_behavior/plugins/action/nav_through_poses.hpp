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

#ifndef SRM27_BEHAVIOR__PLUGINS__ACTION__NAV_THROUGH_POSES_HPP_
#define SRM27_BEHAVIOR__PLUGINS__ACTION__NAV_THROUGH_POSES_HPP_

#include <string>

#include "behaviortree_ros2/bt_action_node.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"

namespace srm27_behavior
{

/**
 * @brief Sends a multi-waypoint list (from inline string or YAML/CSV mission file) to
 * Nav2's NavigateThroughPoses action server for smooth continuous patrolling.
 */
class NavigateThroughPosesAction
: public BT::RosActionNode<nav2_msgs::action::NavigateThroughPoses>
{
public:
  NavigateThroughPosesAction(
    const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params);

  bool setGoal(Goal & goal) override;
  BT::NodeStatus tick() override;

  BT::NodeStatus onResultReceived(const WrappedResult & wr) override;

  BT::NodeStatus onFeedback(
    const std::shared_ptr<const nav2_msgs::action::NavigateThroughPoses::Feedback> feedback)
    override;

  void halt() override;

  void onHalt() override;

  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;

  static BT::PortsList providedPorts();
};

}  // namespace srm27_behavior

#endif  // SRM27_BEHAVIOR__PLUGINS__ACTION__NAV_THROUGH_POSES_HPP_
