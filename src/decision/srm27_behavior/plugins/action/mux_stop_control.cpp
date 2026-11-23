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

#include "srm27_behavior/plugins/action/mux_stop_control.hpp"

namespace srm27_behavior
{

MuxStopControlAction::MuxStopControlAction(
  const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params)
: BT::RosServiceNode<std_srvs::srv::Trigger>(name, config, params)
{
}

BT::PortsList MuxStopControlAction::providedPorts()
{
  return providedBasicPorts({
    BT::InputPort<bool>(
      "ignore_unreachable", false,
      "Return SUCCESS when srm_cmd_mux service is absent (e.g. standalone BT test)"),
  });
}

bool MuxStopControlAction::setRequest(Request::SharedPtr & request)
{
  (void)request;
  return true;
}

BT::NodeStatus MuxStopControlAction::onResponseReceived(const Response::SharedPtr & response)
{
  if (response && response->success) {
    RCLCPP_INFO(
      logger(), "Mux service [%s] succeeded: %s", service_name_.c_str(),
      response->message.c_str());
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_WARN(
    logger(), "Mux service [%s] rejected: %s", service_name_.c_str(),
    response ? response->message.c_str() : "null response");
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus MuxStopControlAction::onFailure(BT::ServiceNodeErrorCode error)
{
  bool ignore_unreachable = false;
  getInput("ignore_unreachable", ignore_unreachable);

  if (error == BT::SERVICE_UNREACHABLE && ignore_unreachable) {
    RCLCPP_WARN_THROTTLE(
      logger(), *node_.lock()->get_clock(), 5000,
      "Mux service [%s] unreachable (ignored)", service_name_.c_str());
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_ERROR(
    logger(), "Mux service [%s] failed with error: %s", service_name_.c_str(), BT::toStr(error));
  return BT::NodeStatus::FAILURE;
}

}  // namespace srm27_behavior

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(srm27_behavior::MuxStopControlAction, "MuxStopControl");
