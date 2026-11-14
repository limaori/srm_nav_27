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

#ifndef SRM27_BEHAVIOR__NAVIGATION_SPIN_TRACKER_HPP_
#define SRM27_BEHAVIOR__NAVIGATION_SPIN_TRACKER_HPP_

#include <algorithm>
#include <cmath>
#include <mutex>
#include <sstream>
#include <string>

#include "behaviortree_cpp/blackboard.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace srm27_behavior
{

/**
 * @brief Tracks active navigation target distance with enter/leave hysteresis to coordinate
 * slow spin while moving and fast spin upon arrival.
 */
struct NavigationSpinTracker
{
  std::mutex mutex;

  // Dynamic tunable parameters (hot-reloaded from ROS 2 params)
  double moving_speed = 2.5;
  double arrived_speed = 7.0;
  double supply_speed = 2.0;
  double evasion_speed = 8.0;
  double arrive_tolerance = 0.35;
  double leave_tolerance = 0.55;
  double completion_tolerance = 0.8;

  // Runtime goal & robot pose state
  bool has_active_goal = false;
  bool nav_goal_finished = false;
  bool navigation_ready = false;
  bool nav_arrived = false;
  bool has_robot_pose = false;
  double dist_to_goal = 0.0;
  double last_cmd_wz = 0.0;
  std::string last_spin_mode = "INIT";
  geometry_msgs::msg::PoseStamped active_goal;
  geometry_msgs::msg::PoseStamped completion_pose;
  geometry_msgs::msg::PoseStamped last_robot_pose;
  BT::Blackboard::Ptr global_bb;

  void syncToBlackboard(const BT::Blackboard::Ptr & bb = nullptr)
  {
    const auto target = bb ? bb : global_bb;
    if (!target) {
      return;
    }
    target->set("has_active_goal", has_active_goal);
    target->set("nav_goal_finished", nav_goal_finished);
    target->set("navigation_ready", navigation_ready);
    target->set("nav_arrived", nav_arrived);
    target->set("dist_to_goal", dist_to_goal);
    target->set("spin_moving_speed", moving_speed);
    target->set("spin_arrived_speed", arrived_speed);
    target->set("spin_supply_speed", supply_speed);
    target->set("spin_evasion_speed", evasion_speed);
    if (has_active_goal) {
      target->set("active_goal", active_goal);
    }
  }

  void evaluateLocked()
  {
    if (!has_active_goal) {
      nav_arrived = false;
      dist_to_goal = 0.0;
      return;
    }

    if (has_robot_pose && active_goal.header.frame_id == last_robot_pose.header.frame_id) {
      dist_to_goal = std::hypot(
        active_goal.pose.position.x - last_robot_pose.pose.position.x,
        active_goal.pose.position.y - last_robot_pose.pose.position.y);
      // Nav2 may finish relative to a grid-snapped endpoint; use its accepted stopping pose as an anchor.
      const bool near_completion = nav_goal_finished &&
        completion_pose.header.frame_id == active_goal.header.frame_id &&
        std::hypot(completion_pose.pose.position.x - last_robot_pose.pose.position.x,
                   completion_pose.pose.position.y - last_robot_pose.pose.position.y) <= leave_tolerance;
      if (dist_to_goal <= arrive_tolerance || near_completion) {
        nav_arrived = true;
      } else if (dist_to_goal > std::max(leave_tolerance, arrive_tolerance)) {
        nav_arrived = false;
      }
    } else {
      nav_arrived = false;
    }
  }

  void setNewGoal(const geometry_msgs::msg::PoseStamped & goal, const BT::Blackboard::Ptr & bb = nullptr)
  {
    std::lock_guard<std::mutex> lock(mutex);
    const double delta = has_active_goal
                           ? std::hypot(
                               goal.pose.position.x - active_goal.pose.position.x,
                               goal.pose.position.y - active_goal.pose.position.y)
                           : 1e9;
    // Deduplicate repeated identical goal dispatches so hovering at arrived goal stays ARRIVED.
    if (!has_active_goal || delta > 0.05 || goal.header.frame_id != active_goal.header.frame_id) {
      active_goal = goal;
      has_active_goal = true;
      nav_goal_finished = false;
      nav_arrived = false;
    }
    nav_goal_finished = false;
    evaluateLocked();
    syncToBlackboard(bb);
  }

  void clearGoal()
  {
    std::lock_guard<std::mutex> lock(mutex);
    has_active_goal = false;
    nav_goal_finished = false;
    navigation_ready = false;
    nav_arrived = false;
    dist_to_goal = 0.0;
    syncToBlackboard();
  }

  bool markGoalFinished(const BT::Blackboard::Ptr & bb = nullptr)
  {
    std::lock_guard<std::mutex> lock(mutex);
    // Allow normal planner endpoint/stop tolerances, never anchor arbitrary distant "success".
    if (!has_robot_pose || active_goal.header.frame_id != last_robot_pose.header.frame_id ||
        dist_to_goal > completion_tolerance) { return false; }
    nav_goal_finished = true;
    completion_pose = last_robot_pose;
    evaluateLocked();
    syncToBlackboard(bb);
    return true;
  }

  void updateRobotPose(
    const geometry_msgs::msg::PoseStamped & pose, const BT::Blackboard::Ptr & bb = nullptr)
  {
    std::lock_guard<std::mutex> lock(mutex);
    last_robot_pose = pose;
    has_robot_pose = true;
    evaluateLocked();
    syncToBlackboard(bb);
  }

  std::string formatDebugState(const std::string & spin_mode, double cmd_wz)
  {
    std::lock_guard<std::mutex> lock(mutex);
    std::ostringstream oss;
    oss.setf(std::ios::fixed);
    oss.precision(2);
    oss << "mode=" << (spin_mode.empty() ? "UNKNOWN" : spin_mode)
        << " arrived=" << (nav_arrived ? 1 : 0)
        << " has_goal=" << (has_active_goal ? 1 : 0)
        << " dist=" << dist_to_goal
        << " tol=[" << arrive_tolerance << "," << leave_tolerance << "]"
        << " wz=" << cmd_wz;
    return oss.str();
  }
};

}  // namespace srm27_behavior

#endif  // SRM27_BEHAVIOR__NAVIGATION_SPIN_TRACKER_HPP_
