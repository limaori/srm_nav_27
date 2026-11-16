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

#include "srm27_behavior/plugins/action/calculate_attack_pose.hpp"

#include <algorithm>
#include <cmath>

#include "nav2_util/robot_utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace srm27_behavior
{

CalculateAttackPoseAction::CalculateAttackPoseAction(
  const std::string & instance_name, const BT::NodeConfig & conf,
  const BT::RosNodeParams & params)
: BT::RosTopicPubNode<visualization_msgs::msg::MarkerArray>(instance_name, conf, params)
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

BT::PortsList CalculateAttackPoseAction::providedPorts()
{
  return providedBasicPorts({
    BT::InputPort<nav_msgs::msg::OccupancyGrid>(
      "costmap_port", "{@nav_globalCostmap}", "GlobalCostmap port on blackboard"),
    BT::InputPort<geometry_msgs::msg::PoseStamped>(
      "enemy_pose_port", "{@enemy_pose}", "Enemy pose port on blackboard"),
    BT::InputPort<nav_msgs::msg::Odometry>(
      "odom_port", "{@nav_odometry}", "Fallback odometry port when TF is unavailable"),
    BT::InputPort<double>("attack_radius", 2.5, "Desired engagement radius around enemy (m)"),
    BT::InputPort<int>("num_sectors", 36, "Number of ring candidate samples"),
    BT::InputPort<int>("cost_threshold", 50, "Maximum acceptable occupancy cost (0-100)"),
    BT::InputPort<std::string>("robot_base_frame", "base_link", "Robot base TF frame"),
    BT::InputPort<double>("transform_tolerance", 0.5, "TF lookup timeout (s)"),
    BT::InputPort<double>(
      "max_target_age_sec", 1.5, "Expire enemy_pose older than this age (<=0 disables check)"),
    BT::InputPort<bool>("visualize", false, "Publish RViz MarkerArray"),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>(
      "goal", "{attack_pose}", "Computed attack PoseStamped"),
  });
}

bool CalculateAttackPoseAction::setMessage(visualization_msgs::msg::MarkerArray & msg)
{
  msg.markers.clear();

  auto global_costmap = getInput<nav_msgs::msg::OccupancyGrid>("costmap_port");
  auto enemy_pose = getInput<geometry_msgs::msg::PoseStamped>("enemy_pose_port");
  if (!global_costmap || !enemy_pose) {
    return false;
  }

  if (global_costmap->data.empty() || global_costmap->info.resolution <= 0.0f) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 3000,
      "CalculateAttackPose: global_costmap is empty or invalid");
    return false;
  }

  double attack_radius = 2.5;
  int num_sectors = 36;
  int cost_threshold = 50;
  std::string robot_base_frame = "base_link";
  double transform_tolerance = 0.5;
  double max_target_age_sec = 1.5;
  bool visualize = false;

  getInput("attack_radius", attack_radius);
  getInput("num_sectors", num_sectors);
  getInput("cost_threshold", cost_threshold);
  getInput("robot_base_frame", robot_base_frame);
  getInput("transform_tolerance", transform_tolerance);
  getInput("max_target_age_sec", max_target_age_sec);
  getInput("visualize", visualize);

  if (num_sectors <= 0 || num_sectors > 720 || !std::isfinite(attack_radius) ||
      attack_radius <= 0.0 || !std::isfinite(enemy_pose->pose.position.x) ||
      !std::isfinite(enemy_pose->pose.position.y)) {
    return false;
  }

  const auto & stamp = enemy_pose->header.stamp;
  if (max_target_age_sec > 0.0) {
    if (stamp.sec == 0 && stamp.nanosec == 0) { return false; }
    const double age = (node_->now() - rclcpp::Time(stamp)).seconds();
    if (age > max_target_age_sec || age < -1.0) {
      return false;
    }
  }

  const std::string costmap_frame =
    global_costmap->header.frame_id.empty() ? "map" : global_costmap->header.frame_id;

  geometry_msgs::msg::PointStamped enemy_in;
  enemy_in.header = enemy_pose->header;
  if (enemy_in.header.frame_id.empty()) {
    enemy_in.header.frame_id = costmap_frame;
  }
  enemy_in.point = enemy_pose->pose.position;

  geometry_msgs::msg::PointStamped enemy_on_costmap;
  if (enemy_in.header.frame_id == costmap_frame) {
    enemy_on_costmap = enemy_in;
  } else {
    try {
      enemy_on_costmap = tf_buffer_->transform(
        enemy_in, costmap_frame, tf2::durationFromSec(transform_tolerance));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 3000,
        "CalculateAttackPose: failed to transform enemy_pose (%s -> %s): %s",
        enemy_in.header.frame_id.c_str(), costmap_frame.c_str(), ex.what());
      return false;
    }
  }

  const auto candidates =
    generateCandidatePoints(enemy_on_costmap.point, attack_radius, num_sectors);
  const auto feasible_points =
    filterFeasiblePoints(candidates, *global_costmap, cost_threshold);
  if (feasible_points.empty()) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 3000,
      "CalculateAttackPose: no feasible attack points found around enemy");
    return false;
  }

  geometry_msgs::msg::PoseStamped robot_pose;
  if (!nav2_util::getCurrentPose(
        robot_pose, *tf_buffer_, costmap_frame, robot_base_frame, transform_tolerance))
  {
    auto odom = getInput<nav_msgs::msg::Odometry>("odom_port");
    if (!odom || odom->header.frame_id != costmap_frame) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 3000,
        "CalculateAttackPose: failed to get robot pose via TF or odometry");
      return false;
    }
    robot_pose.header = odom->header;
    robot_pose.pose = odom->pose.pose;
  }

  const auto best_point = selectBestPoint(feasible_points, robot_pose.pose.position);
  const auto attack_pose = createAttackPose(best_point, enemy_on_costmap);
  setOutput("goal", attack_pose);

  if (visualize) {
    createVisualizationMarkers(
      msg, enemy_on_costmap.point, candidates, feasible_points, best_point,
      robot_pose.pose.position, *global_costmap, attack_radius);
  }
  return true;
}

std::vector<geometry_msgs::msg::Point> CalculateAttackPoseAction::generateCandidatePoints(
  const geometry_msgs::msg::Point & enemy_position, double attack_radius, int num_sectors) const
{
  std::vector<geometry_msgs::msg::Point> candidates;
  candidates.reserve(static_cast<size_t>(num_sectors));
  for (int i = 0; i < num_sectors; ++i) {
    const double angle = i * 2.0 * M_PI / num_sectors;
    geometry_msgs::msg::Point p;
    p.x = enemy_position.x + attack_radius * std::cos(angle);
    p.y = enemy_position.y + attack_radius * std::sin(angle);
    p.z = 0.0;
    candidates.push_back(p);
  }
  return candidates;
}

std::vector<geometry_msgs::msg::Point> CalculateAttackPoseAction::filterFeasiblePoints(
  const std::vector<geometry_msgs::msg::Point> & candidates,
  const nav_msgs::msg::OccupancyGrid & costmap, int cost_threshold) const
{
  std::vector<geometry_msgs::msg::Point> feasible;
  const auto & info = costmap.info;
  if (!std::isfinite(info.resolution) || info.resolution <= 0.0 ||
      !std::isfinite(info.origin.position.x) || !std::isfinite(info.origin.position.y) ||
      costmap.data.size() != static_cast<size_t>(info.width) * info.height) {
    return feasible;
  }
  // shortcut: Nav2 costmaps have axis-aligned origins; reject rotated custom grids.
  if (std::abs(info.origin.orientation.z) > 1e-6 ||
      std::abs(info.origin.orientation.x) > 1e-6 ||
      std::abs(info.origin.orientation.y) > 1e-6) { return feasible; }
  for (const auto & p : candidates) {
    const double cell_x = std::floor((p.x - info.origin.position.x) / info.resolution);
    const double cell_y = std::floor((p.y - info.origin.position.y) / info.resolution);
    if (!std::isfinite(cell_x) || !std::isfinite(cell_y) ||
        cell_x < 0 || cell_x >= info.width || cell_y < 0 || cell_y >= info.height)
    {
      continue;
    }
    const size_t idx = static_cast<size_t>(cell_y) * info.width + static_cast<size_t>(cell_x);
    if (idx >= costmap.data.size()) {
      continue;
    }
    const int8_t cost = costmap.data[idx];
    if (cost >= 0 && cost <= cost_threshold) {
      feasible.push_back(p);
    }
  }
  return feasible;
}

geometry_msgs::msg::Point CalculateAttackPoseAction::selectBestPoint(
  const std::vector<geometry_msgs::msg::Point> & feasible_points,
  const geometry_msgs::msg::Point & robot_position) const
{
  return *std::min_element(
    feasible_points.begin(), feasible_points.end(),
    [&](const auto & a, const auto & b) {
      const double da = std::hypot(a.x - robot_position.x, a.y - robot_position.y);
      const double db = std::hypot(b.x - robot_position.x, b.y - robot_position.y);
      return da < db;
    });
}

geometry_msgs::msg::PoseStamped CalculateAttackPoseAction::createAttackPose(
  const geometry_msgs::msg::Point & attack_point,
  const geometry_msgs::msg::PointStamped & enemy_position) const
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = enemy_position.header.frame_id;
  pose.header.stamp = node_->now();
  pose.pose.position = attack_point;

  const double dx = enemy_position.point.x - attack_point.x;
  const double dy = enemy_position.point.y - attack_point.y;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, std::atan2(dy, dx));
  pose.pose.orientation = tf2::toMsg(q);
  return pose;
}

void CalculateAttackPoseAction::createVisualizationMarkers(
  visualization_msgs::msg::MarkerArray & msg,
  const geometry_msgs::msg::Point & enemy_position,
  const std::vector<geometry_msgs::msg::Point> & candidates,
  const std::vector<geometry_msgs::msg::Point> & feasible_points,
  const geometry_msgs::msg::Point & best_point,
  const geometry_msgs::msg::Point & robot_position,
  const nav_msgs::msg::OccupancyGrid & costmap,
  double attack_radius) const
{
  (void)robot_position;
  const std::string frame = costmap.header.frame_id.empty() ? "map" : costmap.header.frame_id;

  visualization_msgs::msg::Marker enemy_marker;
  enemy_marker.header.frame_id = frame;
  enemy_marker.ns = "enemy";
  enemy_marker.id = 0;
  enemy_marker.type = visualization_msgs::msg::Marker::SPHERE;
  enemy_marker.pose.position = enemy_position;
  enemy_marker.scale.x = enemy_marker.scale.y = enemy_marker.scale.z = 0.3;
  enemy_marker.color.b = 1.0f;
  enemy_marker.color.a = 1.0f;
  msg.markers.push_back(enemy_marker);

  visualization_msgs::msg::Marker best_marker;
  best_marker.header.frame_id = frame;
  best_marker.ns = "best";
  best_marker.id = 0;
  best_marker.type = visualization_msgs::msg::Marker::SPHERE;
  best_marker.pose.position = best_point;
  best_marker.scale.x = best_marker.scale.y = best_marker.scale.z = 0.4;
  best_marker.color.g = 1.0f;
  best_marker.color.a = 1.0f;
  msg.markers.push_back(best_marker);

  for (size_t i = 0; i < candidates.size(); ++i) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame;
    marker.ns = "candidates";
    marker.id = static_cast<int>(i);
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.pose.position = candidates[i];
    marker.scale.x = marker.scale.y = marker.scale.z = 0.18;
    const bool ok = std::any_of(
      feasible_points.begin(), feasible_points.end(),
      [&](const auto & p) { return p.x == candidates[i].x && p.y == candidates[i].y; });
    marker.color.r = ok ? 0.0f : 1.0f;
    marker.color.g = ok ? 1.0f : 0.0f;
    marker.color.a = ok ? 0.8f : 0.25f;
    msg.markers.push_back(marker);
  }

  visualization_msgs::msg::Marker circle;
  circle.header.frame_id = frame;
  circle.ns = "range";
  circle.id = 0;
  circle.type = visualization_msgs::msg::Marker::LINE_STRIP;
  circle.pose.position.z = 0.05;
  circle.scale.x = 0.04;
  circle.color.b = 1.0f;
  circle.color.a = 0.5f;
  constexpr int kSteps = 36;
  for (int i = 0; i <= kSteps; ++i) {
    const double angle = i * 2.0 * M_PI / kSteps;
    geometry_msgs::msg::Point pt;
    pt.x = enemy_position.x + attack_radius * std::cos(angle);
    pt.y = enemy_position.y + attack_radius * std::sin(angle);
    circle.points.push_back(pt);
  }
  msg.markers.push_back(circle);
}

}  // namespace srm27_behavior

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(srm27_behavior::CalculateAttackPoseAction, "CalculateAttackPose");
