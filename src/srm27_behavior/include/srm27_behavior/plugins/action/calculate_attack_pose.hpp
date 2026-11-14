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

#ifndef SRM27_BEHAVIOR__PLUGINS__ACTION__CALCULATE_ATTACK_POSE_HPP_
#define SRM27_BEHAVIOR__PLUGINS__ACTION__CALCULATE_ATTACK_POSE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace srm27_behavior
{

/**
 * @brief Samples candidate attack poses on a ring around enemy_pose, filters against
 * global_costmap, and outputs the nearest collision-free pose facing the enemy.
 */
class CalculateAttackPoseAction : public BT::RosTopicPubNode<visualization_msgs::msg::MarkerArray>
{
public:
  CalculateAttackPoseAction(
    const std::string & instance_name, const BT::NodeConfig & conf,
    const BT::RosNodeParams & params);

  bool setMessage(visualization_msgs::msg::MarkerArray & msg) override;

  static BT::PortsList providedPorts();

private:
  std::vector<geometry_msgs::msg::Point> generateCandidatePoints(
    const geometry_msgs::msg::Point & enemy_position, double attack_radius, int num_sectors) const;

  std::vector<geometry_msgs::msg::Point> filterFeasiblePoints(
    const std::vector<geometry_msgs::msg::Point> & candidates,
    const nav_msgs::msg::OccupancyGrid & costmap, int cost_threshold) const;

  geometry_msgs::msg::Point selectBestPoint(
    const std::vector<geometry_msgs::msg::Point> & feasible_points,
    const geometry_msgs::msg::Point & robot_position) const;

  geometry_msgs::msg::PoseStamped createAttackPose(
    const geometry_msgs::msg::Point & attack_point,
    const geometry_msgs::msg::PointStamped & enemy_position) const;

  void createVisualizationMarkers(
    visualization_msgs::msg::MarkerArray & msg,
    const geometry_msgs::msg::Point & enemy_position,
    const std::vector<geometry_msgs::msg::Point> & candidates,
    const std::vector<geometry_msgs::msg::Point> & feasible_points,
    const geometry_msgs::msg::Point & best_point,
    const geometry_msgs::msg::Point & robot_position,
    const nav_msgs::msg::OccupancyGrid & costmap,
    double attack_radius) const;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace srm27_behavior

#endif  // SRM27_BEHAVIOR__PLUGINS__ACTION__CALCULATE_ATTACK_POSE_HPP_
