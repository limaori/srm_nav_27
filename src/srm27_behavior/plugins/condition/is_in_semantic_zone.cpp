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

#include "srm27_behavior/plugins/condition/is_in_semantic_zone.hpp"

namespace srm27_behavior
{

IsInSemanticZoneCondition::IsInSemanticZoneCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsInSemanticZoneCondition::checkZone, this), config)
{
}

BT::NodeStatus IsInSemanticZoneCondition::checkZone()
{
  std::string map_file;
  std::string zone_code;
  std::string zone_type;
  getInput("map_file", map_file);
  getInput("zone_code", zone_code);
  getInput("zone_type", zone_type);

  if (map_file.empty()) {
    RCLCPP_ERROR(logger_, "IsInSemanticZone: 'map_file' is empty");
    return BT::NodeStatus::FAILURE;
  }

  if (map_file != loaded_map_file_) {
    if (!semantic_map_.loadFromYaml(map_file)) {
      RCLCPP_ERROR(logger_, "IsInSemanticZone: failed to load '%s'", map_file.c_str());
      loaded_map_file_.clear();
      return BT::NodeStatus::FAILURE;
    }
    loaded_map_file_ = map_file;
    RCLCPP_INFO(
      logger_, "Loaded semantic map '%s' (%zu zones)", map_file.c_str(), semantic_map_.size());
  }

  double x = 0.0;
  double y = 0.0;
  bool has_pose = false;

  if (auto map_pose = getInput<geometry_msgs::msg::PoseStamped>("map_pose_port")) {
    if (map_pose->header.frame_id == semantic_map_.frameId()) {
      x = map_pose->pose.position.x;
      y = map_pose->pose.position.y;
      has_pose = true;
    }
  }

  if (!has_pose) {
    if (auto odom = getInput<nav_msgs::msg::Odometry>("odom_port")) {
      if (odom->header.frame_id == semantic_map_.frameId()) {
        x = odom->pose.pose.position.x;
        y = odom->pose.pose.position.y;
        has_pose = true;
      }
    }
  }

  if (!has_pose) {
    return BT::NodeStatus::FAILURE;
  }

  const SemanticZone * matched = semantic_map_.findZoneAt(x, y, zone_code, zone_type);
  if (!matched) {
    setOutput("matched_zone", std::string(""));
    return BT::NodeStatus::FAILURE;
  }

  setOutput("matched_zone", matched->code);
  setOutput("zone_max_spin", matched->max_spin);
  return BT::NodeStatus::SUCCESS;
}

BT::PortsList IsInSemanticZoneCondition::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "map_file", "params/rmuc_semantic_map.yaml", "Semantic map YAML file path"),
    BT::InputPort<std::string>("zone_code", "", "Exact zone code to match (optional)"),
    BT::InputPort<std::string>(
      "zone_type", "", "Zone type to match, e.g. slope/tunnel/fortress/center (optional)"),
    BT::InputPort<geometry_msgs::msg::PoseStamped>(
      "map_pose_port", "{@nav_mapPose}", "Robot pose in map frame from TF"),
    BT::InputPort<nav_msgs::msg::Odometry>(
      "odom_port", "{@nav_odometry}", "Fallback odometry pose when map TF is absent"),
    BT::OutputPort<std::string>("matched_zone", "{matched_zone}", "Matched zone code"),
    BT::OutputPort<double>("zone_max_spin", "{zone_max_spin}", "Max allowed spin speed in zone"),
  };
}

}  // namespace srm27_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<srm27_behavior::IsInSemanticZoneCondition>("IsInSemanticZone");
}
