// Copyright 2027 SRM
// SPDX-License-Identifier: Apache-2.0

#ifndef SRM27_BEHAVIOR_PLUGINS__ACTION__PUBLISH_HANDOVER_FLAG_HPP_
#define SRM27_BEHAVIOR_PLUGINS__ACTION__PUBLISH_HANDOVER_FLAG_HPP_

#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include "std_msgs/msg/u_int8.hpp"

namespace srm27_behavior
{

class PublishHandoverFlagAction : public BT::RosTopicPubNode<std_msgs::msg::UInt8>
{
public:
  using RosTopicPubNode::RosTopicPubNode;

  static BT::PortsList providedPorts();
  bool setMessage(std_msgs::msg::UInt8 & msg) override;
};

}  // namespace srm27_behavior

#endif  // SRM27_BEHAVIOR_PLUGINS__ACTION__PUBLISH_HANDOVER_FLAG_HPP_
