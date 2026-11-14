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

#ifndef SRM27_BEHAVIOR__PLUGINS__ACTION__SET_SENTRY_POSTURE_HPP_
#define SRM27_BEHAVIOR__PLUGINS__ACTION__SET_SENTRY_POSTURE_HPP_

#include <memory>
#include <string>

#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include "srm27_behavior/sentry_posture_state.hpp"
#include "std_msgs/msg/u_int8.hpp"

namespace srm27_behavior
{

/**
 * @brief Single-tick action publishing RMUC 2027 sentry posture (0=MOVE, 1=ATTACK, 2=DEFENSE)
 * on sentry/desired_posture while enforcing the 5s switch cooldown and 180s degradation budget.
 */
class SetSentryPostureAction : public BT::RosTopicPubNode<std_msgs::msg::UInt8>
{
public:
  SetSentryPostureAction(
    const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  bool setMessage(std_msgs::msg::UInt8 & msg) override;
};

}  // namespace srm27_behavior

#endif  // SRM27_BEHAVIOR__PLUGINS__ACTION__SET_SENTRY_POSTURE_HPP_
