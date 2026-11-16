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

#ifndef SRM27_BEHAVIOR__PLUGINS__CONDITION__IS_IN_SEMANTIC_ZONE_HPP_
#define SRM27_BEHAVIOR__PLUGINS__CONDITION__IS_IN_SEMANTIC_ZONE_HPP_

#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "srm27_behavior/semantic_map.hpp"

namespace srm27_behavior
{

/**
 * @brief Checks whether the robot's current pose falls inside a named/typed semantic polygon
 * zone loaded from YAML and outputs zone attributes (e.g., max_spin).
 */
class IsInSemanticZoneCondition : public BT::SimpleConditionNode
{
public:
  IsInSemanticZoneCondition(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

private:
  BT::NodeStatus checkZone();

  rclcpp::Logger logger_ = rclcpp::get_logger("IsInSemanticZoneCondition");
  SemanticMap semantic_map_;
  std::string loaded_map_file_;
};

}  // namespace srm27_behavior

#endif  // SRM27_BEHAVIOR__PLUGINS__CONDITION__IS_IN_SEMANTIC_ZONE_HPP_
