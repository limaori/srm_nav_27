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

#ifndef SRM27_BEHAVIOR__PLUGINS__ACTION__MUX_STOP_CONTROL_HPP_
#define SRM27_BEHAVIOR__PLUGINS__ACTION__MUX_STOP_CONTROL_HPP_

#include <string>

#include "behaviortree_ros2/bt_service_node.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace srm27_behavior
{

/**
 * @brief Trigger srm_cmd_mux stop_all / resume_all service from Behavior Tree.
 */
class MuxStopControlAction : public BT::RosServiceNode<std_srvs::srv::Trigger>
{
public:
  MuxStopControlAction(
    const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  bool setRequest(Request::SharedPtr & request) override;

  BT::NodeStatus onResponseReceived(const Response::SharedPtr & response) override;

  BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override;
};

}  // namespace srm27_behavior

#endif  // SRM27_BEHAVIOR__PLUGINS__ACTION__MUX_STOP_CONTROL_HPP_
