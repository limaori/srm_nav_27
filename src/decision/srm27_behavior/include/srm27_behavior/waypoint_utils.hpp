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

#ifndef SRM27_BEHAVIOR__WAYPOINT_UTILS_HPP_
#define SRM27_BEHAVIOR__WAYPOINT_UTILS_HPP_

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace srm27_behavior
{

inline std::string resolveBehaviorFilePath(const std::string & path_str)
{
  if (path_str.empty()) {
    return "";
  }
  std::filesystem::path p(path_str);
  if (p.is_absolute() || std::filesystem::exists(p)) {
    return p.string();
  }
  try {
    const auto pkg_share = ament_index_cpp::get_package_share_directory("srm27_behavior");
    const auto candidate = std::filesystem::path(pkg_share) / p;
    if (std::filesystem::exists(candidate)) {
      return candidate.string();
    }
  } catch (const std::exception &) {
  }
  return path_str;
}

inline geometry_msgs::msg::PoseStamped makePoseStamped(
  double x, double y, double yaw, const std::string & frame_id)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = frame_id;
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  pose.pose.orientation = tf2::toMsg(q);
  return pose;
}

/// Parse inline goals formatted as "x1;y1;yaw1|x2;y2;yaw2|..."
inline bool parseInlineWaypoints(
  const std::string & goals_str,
  std::vector<geometry_msgs::msg::PoseStamped> & waypoints,
  const std::string & frame_id = "map")
{
  waypoints.clear();
  if (goals_str.empty()) {
    return false;
  }

  std::istringstream list_stream(goals_str);
  std::string item;
  while (std::getline(list_stream, item, '|')) {
    std::istringstream coord_stream(item);
    std::string token;
    std::vector<double> vals;
    try {
      while (std::getline(coord_stream, token, ';')) {
        size_t consumed = 0;
        const double value = std::stod(token, &consumed);
        if (token.find_first_not_of(" \t\r\n", consumed) != std::string::npos || !std::isfinite(value)) {
          waypoints.clear();
          return false;
        }
        vals.push_back(value);
      }
    } catch (const std::exception &) {
      waypoints.clear();
      return false;
    }
    if ((vals.size() != 2 && vals.size() != 3) || item.back() == ';') {
      waypoints.clear();
      return false;
    }
    waypoints.push_back(makePoseStamped(vals[0], vals[1], vals.size() == 3 ? vals[2] : 0.0, frame_id));
  }
  if (goals_str.back() == '|') { waypoints.clear(); }
  return !waypoints.empty();
}

/// Load waypoints from mission YAML (missions/*.yaml format) or CSV (id,x,y,z,qx,qy,qz,qw).
inline bool loadWaypointsFromFile(
  const std::string & raw_path,
  std::vector<geometry_msgs::msg::PoseStamped> & waypoints,
  const std::string & default_frame_id = "map")
{
  waypoints.clear();
  const std::string resolved = resolveBehaviorFilePath(raw_path);
  if (resolved.empty() || !std::filesystem::exists(resolved)) {
    return false;
  }

  const auto ext = std::filesystem::path(resolved).extension().string();
  if (ext == ".yaml" || ext == ".yml") {
    try {
      const YAML::Node root = YAML::LoadFile(resolved);
      const std::string frame =
        root["frame_id"] ? root["frame_id"].as<std::string>(default_frame_id) : default_frame_id;
      const auto nodes = root["waypoints"];
      if (!nodes || !nodes.IsSequence()) {
        return false;
      }
      for (const auto & wp : nodes) {
        if (!wp["x"] || !wp["y"]) { waypoints.clear(); return false; }
        const double x = wp["x"].as<double>();
        const double y = wp["y"].as<double>();
        const double yaw = wp["yaw"] ? wp["yaw"].as<double>(0.0) : 0.0;
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw)) {
          waypoints.clear(); return false;
        }
        waypoints.push_back(makePoseStamped(x, y, yaw, frame));
      }
      return !waypoints.empty();
    } catch (const std::exception &) {
      waypoints.clear(); return false;
    }
  }

  // Fallback to CSV: id,pose_x,pose_y,pose_z,rot_x,rot_y,rot_z,rot_w
  std::ifstream file(resolved);
  if (!file.is_open()) {
    return false;
  }
  auto number = [](const std::string & token) {
    size_t consumed = 0;
    const double value = std::stod(token, &consumed);
    if (token.find_first_not_of(" \t\r\n", consumed) != std::string::npos || !std::isfinite(value)) {
      throw std::invalid_argument("Invalid waypoint number");
    }
    return value;
  };
  std::string line;
  std::getline(file, line);  // skip header
  while (std::getline(file, line)) {
    if (line.empty()) {
      continue;
    }
    std::istringstream ss(line);
    std::string token;
    std::vector<std::string> tokens;
    while (std::getline(ss, token, ',')) {
      tokens.push_back(token);
    }
    if (tokens.size() != 8) { waypoints.clear(); return false; }
    try {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = default_frame_id;
      pose.pose.position.x = number(tokens[1]);
      pose.pose.position.y = number(tokens[2]);
      pose.pose.position.z = number(tokens[3]);
      pose.pose.orientation.x = number(tokens[4]);
      pose.pose.orientation.y = number(tokens[5]);
      pose.pose.orientation.z = number(tokens[6]);
      pose.pose.orientation.w = number(tokens[7]);
      const auto & q = pose.pose.orientation;
      const auto & p = pose.pose.position;
      const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
          !std::isfinite(norm) || std::abs(norm - 1.0) > 0.01) {
        waypoints.clear(); return false;
      }
      waypoints.push_back(pose);
    } catch (const std::exception &) {
      waypoints.clear(); return false;
    }
  }
  return !waypoints.empty();
}

inline bool parseSinglePoseYaml(
  const YAML::Node & wp, geometry_msgs::msg::PoseStamped & out, const std::string & frame_id)
{
  if (!wp || !wp["x"] || !wp["y"]) {
    return false;
  }
  const double x = wp["x"].as<double>();
  const double y = wp["y"].as<double>();
  const double yaw = wp["yaw"] ? wp["yaw"].as<double>(0.0) : 0.0;
  const std::string frame = wp["frame_id"] ? wp["frame_id"].as<std::string>(frame_id) : frame_id;
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw) || frame.empty()) {
    return false;
  }
  out = makePoseStamped(x, y, yaw, frame);
  return true;
}

/// Load a single named waypoint from params/waypoints.yaml (`named_waypoints.<name>`).
inline bool loadNamedWaypoint(
  const std::string & raw_path,
  const std::string & waypoint_name,
  geometry_msgs::msg::PoseStamped & pose_out,
  const std::string & default_frame_id = "map")
{
  if (waypoint_name.empty()) {
    return false;
  }
  const std::string resolved = resolveBehaviorFilePath(raw_path);
  if (resolved.empty() || !std::filesystem::exists(resolved)) {
    return false;
  }
  try {
    const YAML::Node root = YAML::LoadFile(resolved);
    const std::string frame =
      root["frame_id"] ? root["frame_id"].as<std::string>(default_frame_id) : default_frame_id;
    const auto named = root["named_waypoints"];
    if (!named || !named[waypoint_name]) {
      return false;
    }
    return parseSinglePoseYaml(named[waypoint_name], pose_out, frame);
  } catch (const std::exception &) {
    return false;
  }
}

/// Load a named waypoint group from params/waypoints.yaml (`waypoint_groups.<group_name>`).
inline bool loadNamedWaypointGroup(
  const std::string & raw_path,
  const std::string & group_name,
  std::vector<geometry_msgs::msg::PoseStamped> & waypoints,
  const std::string & default_frame_id = "map")
{
  waypoints.clear();
  if (group_name.empty()) {
    return false;
  }
  const std::string resolved = resolveBehaviorFilePath(raw_path);
  if (resolved.empty() || !std::filesystem::exists(resolved)) {
    return false;
  }
  try {
    const YAML::Node root = YAML::LoadFile(resolved);
    const std::string frame =
      root["frame_id"] ? root["frame_id"].as<std::string>(default_frame_id) : default_frame_id;
    const auto groups = root["waypoint_groups"];
    if (!groups || !groups[group_name] || !groups[group_name].IsSequence()) {
      return false;
    }
    for (const auto & item : groups[group_name]) {
      geometry_msgs::msg::PoseStamped pose;
      if (!parseSinglePoseYaml(item, pose, frame)) { waypoints.clear(); return false; }
      waypoints.push_back(pose);
    }
    return !waypoints.empty();
  } catch (const std::exception &) {
    waypoints.clear(); return false;
  }
}

}  // namespace srm27_behavior

#endif  // SRM27_BEHAVIOR__WAYPOINT_UTILS_HPP_
