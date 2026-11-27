// Copyright 2027 SRM
// SPDX-License-Identifier: Apache-2.0

#include "srm27_behavior_plugins/action/publish_handover_flag.hpp"

#include <cstdint>
#include <limits>

namespace srm27_behavior
{

BT::PortsList PublishHandoverFlagAction::providedPorts()
{
  return {
    BT::InputPort<std::string>("topic_name", "decision/handover_flag", "Handover flag topic"),
    BT::InputPort<unsigned>("flag_value", 1u, "UInt8 handover flag (0-255)"),
  };
}

bool PublishHandoverFlagAction::setMessage(std_msgs::msg::UInt8 & msg)
{
  const auto flag = getInput<unsigned>("flag_value");
  if (!flag || *flag > std::numeric_limits<uint8_t>::max()) {
    RCLCPP_ERROR(node_->get_logger(), "PublishHandoverFlag: flag_value must be in [0, 255]");
    return false;
  }
  auto * bb = config().blackboard->rootBlackboard();
  bb->set("motion_allowed", false);
  bb->set("handover_active", true);
  msg.data = static_cast<uint8_t>(*flag);
  RCLCPP_INFO(
    node_->get_logger(), "Publishing handover flag %u on '%s'", *flag, prev_topic_name_.c_str());
  return true;
}

}  // namespace srm27_behavior

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(srm27_behavior::PublishHandoverFlagAction, "PublishHandoverFlag");
