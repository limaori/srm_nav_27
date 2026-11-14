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

#ifndef SRM27_BEHAVIOR__PLUGINS__ACTION__PUB_ROTATION_CMD_HPP_
#define SRM27_BEHAVIOR__PLUGINS__ACTION__PUB_ROTATION_CMD_HPP_

#include <string>

#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace srm27_behavior
{

/**
 * @brief Single-tick publisher of chassis spin request (Twist.angular.z) to MINCO rotation_cmd.
 * Uses RosTopicPubNode so it returns SUCCESS in one tick without halting sibling actions in
 * ReactiveSequence.
 */
class PublishRotationCmdAction : public BT::RosTopicPubNode<geometry_msgs::msg::Twist>
{
public:
  PublishRotationCmdAction(
    const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  bool setMessage(geometry_msgs::msg::Twist & msg) override;
};

}  // namespace srm27_behavior

#endif  // SRM27_BEHAVIOR__PLUGINS__ACTION__PUB_ROTATION_CMD_HPP_
