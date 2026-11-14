// Copyright 2025 Lihan Chen
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

#include "srm27_behavior/srm27_behavior_server.hpp"
#include "srm27_behavior/sentry_posture_state.hpp"

#include <cmath>
#include <type_traits>


#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rm_decision_interfaces/msg/ally_robot_hp.hpp"
#include "rm_decision_interfaces/msg/buff.hpp"
#include "rm_decision_interfaces/msg/event_data.hpp"
#include "rm_decision_interfaces/msg/game_status.hpp"
#include "rm_decision_interfaces/msg/ground_robot_position.hpp"
#include "rm_decision_interfaces/msg/projectile_allowance.hpp"
#include "rm_decision_interfaces/msg/rfid_status.hpp"
#include "rm_decision_interfaces/msg/robot_status.hpp"
#include "rm_decision_interfaces/msg/switch_position.hpp"
#include "rm_decision_interfaces/msg/sefdefined.hpp"
namespace srm27_behavior
{
template <typename T, typename Hook>
void SentryBehaviorServer::subscribe(
  const std::string & topic, const std::string & bb_key, const rclcpp::QoS & qos, Hook && hook)
{
  auto sub = node()->create_subscription<T>(
    topic, qos, [this, bb_key, hook = std::forward<Hook>(hook)](const typename T::SharedPtr msg) {
      if (!bb_key.empty()) {
        globalBlackboard()->set(bb_key, *msg);
        globalBlackboard()->set(bb_key + "_received", std::chrono::steady_clock::now());
      }
      if constexpr (!std::is_same_v<std::decay_t<Hook>, std::nullptr_t>) {
        hook(*msg);
      }
    });
  subscriptions_.push_back(sub);
}

void SentryBehaviorServer::initRefereeInputs()
{
  subscribe<rm_decision_interfaces::msg::EventData>(
    "referee/event_data", "referee_eventData", rclcpp::QoS(10),
    [this](const rm_decision_interfaces::msg::EventData & m) {
      auto bb = globalBlackboard();
      bb->set("event_valid", true);
      bb->set("center_status", static_cast<int>(m.center_gain_point_status));
      bb->set("fortress_status", static_cast<int>(m.fortress_gain_point_status));
      bb->set("central_highland_status", static_cast<int>(m.central_highland_status));
      bb->set("big_energy_status", static_cast<int>(m.big_energy_status));
    });

  subscribe<rm_decision_interfaces::msg::AllyRobotHP>(
    "referee/ally_robot_hp", "referee_allyRobotHP", rclcpp::QoS(10),
    [this](const rm_decision_interfaces::msg::AllyRobotHP & m) {
      auto bb = globalBlackboard();
      bb->set("ally_hp_valid", true);
      bb->set("ally_outpost_hp", static_cast<int>(m.ally_outpost_hp));
      bb->set("ally_base_hp", static_cast<int>(m.ally_base_hp));
      bb->set("ally_sentry_hp", static_cast<int>(m.ally_7_robot_hp));
    });

  subscribe<rm_decision_interfaces::msg::GameStatus>(
    "referee/game_status", "referee_gameStatus", rclcpp::QoS(10),
    [this](const rm_decision_interfaces::msg::GameStatus & m) {
      auto bb = globalBlackboard();
      if (m.game_progress == 4 &&
          (previous_progress_ != 4 || static_cast<int>(m.stage_remain_time) > previous_remain_ + 5)) {
        bb->set("match_epoch", ++match_epoch_);
        bb->get<std::shared_ptr<SentryPostureState>>("sentry_posture_state")->reset();
        bb->set("needs_supply", false);
        previous_hp_ = -1;
        spin_tracker_->clearGoal();
      }
      previous_progress_ = m.game_progress;
      previous_remain_ = m.stage_remain_time;
      bb->set("game_valid", true);
      bb->set("game_type", static_cast<int>(m.game_type));
      bb->set("game_progress", static_cast<int>(m.game_progress));
      bb->set("remain_time", static_cast<int>(m.stage_remain_time));
    });

  subscribe<rm_decision_interfaces::msg::GroundRobotPosition>(
    "referee/ground_robot_position", "referee_groundRobotPosition");

  subscribe<rm_decision_interfaces::msg::RfidStatus>(
    "referee/rfid_status", "referee_rfidStatus", rclcpp::QoS(10),
    [this](const rm_decision_interfaces::msg::RfidStatus & m) {
      auto bb = globalBlackboard();
      const bool supply =
        (m.friendly_supply_zone_non_overlap == m.DETECTED) ||
        (m.friendly_supply_zone_overlap == m.DETECTED);
      bb->set("rfid_supply", supply);
      if (supply && previous_hp_ > 0) { bb->set("needs_supply", false); }
      bb->set("rfid_center", m.center_gain_point_rmul == m.DETECTED);
      bb->set("rfid_fortress", m.friendly_fortress_gain == m.DETECTED);
    });

  subscribe<rm_decision_interfaces::msg::Sefdefined>(
    node()->declare_parameter("status_topic", std::string("/srm/sefdefined")),
    "referee_identity", rclcpp::QoS(10),
    [this](const rm_decision_interfaces::msg::Sefdefined & m) {
      auto bb = globalBlackboard();
      const bool valid = (m.robot_id == 7 || m.robot_id == 107) &&
        m.maximum_hp > 0 && m.current_hp <= m.maximum_hp;
      if (valid) {
        if (previous_hp_ == 0 && m.current_hp > 0) { bb->set("needs_supply", true); }
        previous_hp_ = m.current_hp;
        if (m.current_hp > 0 && bb->get<bool>("rfid_supply")) { bb->set("needs_supply", false); }
      }
      bb->set("robot_id", static_cast<int>(m.robot_id));
      bb->set("status_valid", valid);
      bb->set("chassis_enabled", m.power_management_chassis_output != 0);
      bb->set("hp", static_cast<int>(m.current_hp));
      bb->set("max_hp", static_cast<int>(m.maximum_hp));
      rm_decision_interfaces::msg::RobotStatus status;
      (void)bb->get("referee_robotStatus", status);
      status.robot_id = m.robot_id;
      status.current_hp = m.current_hp;
      status.maximum_hp = m.maximum_hp;
      status.status_valid = valid;
      bb->set("referee_robotStatus", status);
    });

  // 聚合状态仍供热量/受击插件读取，但它的到达不能证明身份、HP 或弹量新鲜。
  subscribe<rm_decision_interfaces::msg::RobotStatus>(
    "referee/robot_status", "referee_aggregate", rclcpp::QoS(10),
    [this](const rm_decision_interfaces::msg::RobotStatus & m) {
      auto bb = globalBlackboard();
      rm_decision_interfaces::msg::RobotStatus status;
      (void)bb->get("referee_robotStatus", status);
      status.shooter_heat = m.shooter_heat;
      status.heat_valid = m.heat_valid;
      status.damage_info_valid = m.damage_info_valid;
      status.hp_deduction_reason = m.hp_deduction_reason;
      bb->set("referee_robotStatus", status);
    });

  subscribe<rm_decision_interfaces::msg::ProjectileAllowance>(
    "referee/projectile_allowance", "referee_projectileAllowance", rclcpp::QoS(10),
    [this](const rm_decision_interfaces::msg::ProjectileAllowance & m) {
      auto bb = globalBlackboard();
      bb->set("ammo_valid", true);
      bb->set("ammo", static_cast<int>(m.projectile_allowance_17mm));
      bb->set("gold", static_cast<int>(m.remaining_gold_coin));
      rm_decision_interfaces::msg::RobotStatus status;
      if (bb->get("referee_robotStatus", status)) {
        status.projectile_allowance_17mm = m.projectile_allowance_17mm;
        status.remaining_gold_coin = m.remaining_gold_coin;
        status.ammo_valid = true;
        bb->set("referee_robotStatus", status);
      }
    });

  subscribe<rm_decision_interfaces::msg::SwitchPosition>(
    "referee/switch_position", "referee_switchPosition", rclcpp::QoS(10),
    [this](const rm_decision_interfaces::msg::SwitchPosition & m) {
      auto bb = globalBlackboard();
      bb->set("switch_valid", true);
      bb->set("switch_position", static_cast<int>(m.switch_position));
    });

  subscribe<rm_decision_interfaces::msg::Buff>("referee/buff", "referee_buff");

  subscribe<nav_msgs::msg::Odometry>(
    "odometry", "nav_odometry", rclcpp::QoS(10),
    [this](const nav_msgs::msg::Odometry & m) {
      const double age = (node()->now() - rclcpp::Time(m.header.stamp)).seconds();
      if (m.header.frame_id == map_frame_ && age >= 0.0 && age < input_timeout_sec_ &&
          std::isfinite(m.pose.pose.position.x) && std::isfinite(m.pose.pose.position.y)) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = m.header;
        pose.pose = m.pose.pose;
        globalBlackboard()->set("nav_mapPose", pose);
        const auto stamp = rclcpp::Time(pose.header.stamp).nanoseconds();
        if (stamp != last_pose_stamp_) {
          last_pose_stamp_ = stamp;
          globalBlackboard()->set("nav_mapPose_received", std::chrono::steady_clock::now());
        }
        spin_tracker_->updateRobotPose(pose);
      }
    });

  subscribe<geometry_msgs::msg::PoseStamped>(
    "enemy_pose", "enemy_pose", rclcpp::SensorDataQoS());

  auto costmap_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  subscribe<nav_msgs::msg::OccupancyGrid>(
    "global_costmap/costmap", "nav_globalCostmap", costmap_qos);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node()->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  map_pose_timer_ = node()->create_wall_timer(
    std::chrono::milliseconds(100), [this]() {
      if (!tf_buffer_->canTransform(map_frame_, robot_base_frame_, tf2::TimePointZero)) {
        refreshInputState();
        return;
      }
      geometry_msgs::msg::PoseStamped pose;
      if (nav2_util::getCurrentPose(pose, *tf_buffer_, map_frame_, robot_base_frame_, 0.1)) {
        const double age = (node()->now() - rclcpp::Time(pose.header.stamp)).seconds();
        if (age >= 0.0 && age < input_timeout_sec_ &&
            std::isfinite(pose.pose.position.x) && std::isfinite(pose.pose.position.y)) {
          globalBlackboard()->set("nav_mapPose", pose);
          const auto stamp = rclcpp::Time(pose.header.stamp).nanoseconds();
          if (stamp != last_pose_stamp_) {
            last_pose_stamp_ = stamp;
            globalBlackboard()->set("nav_mapPose_received", std::chrono::steady_clock::now());
          }
          spin_tracker_->updateRobotPose(pose);
        }
      }
      refreshInputState();
    });
}

void SentryBehaviorServer::refreshInputState()
{
  auto bb = globalBlackboard();
  const auto now = std::chrono::steady_clock::now();
  bool ready = true;
  for (const auto & key : {"referee_gameStatus", "referee_identity", "nav_mapPose"}) {
    std::chrono::steady_clock::time_point received;
    ready = bb->get(std::string(key) + "_received", received) &&
            std::chrono::duration<double>(now - received).count() < input_timeout_sec_ && ready;
  }
  bb->set("inputs_ready", ready);
  std::chrono::steady_clock::time_point ammo_received;
  const bool ammo_valid = bb->get("referee_projectileAllowance_received", ammo_received) &&
    std::chrono::duration<double>(now - ammo_received).count() < input_timeout_sec_;
  bb->set("ammo_valid", ammo_valid);
  rm_decision_interfaces::msg::RobotStatus status;
  if (bb->get("referee_robotStatus", status)) {
    status.ammo_valid = ammo_valid;
    bb->set("referee_robotStatus", status);
  }
  rm_decision_interfaces::msg::EventData events;
  std::chrono::steady_clock::time_point event_received;
  const bool arena_valid = bb->get("referee_eventData", events) && events.battlefield_valid &&
    bb->get("referee_eventData_received", event_received) &&
    std::chrono::duration<double>(now - event_received).count() < input_timeout_sec_;
  bb->set("arena_valid", arena_valid);
  const size_t own = bb->get<int>("robot_id") == 107 ? 1 : 0, enemy = 1 - own;
  bb->set("own_outpost_control", arena_valid ? static_cast<int>(events.outpost_control[own]) : -1);
  bb->set("enemy_outpost_control", arena_valid ? static_cast<int>(events.outpost_control[enemy]) : -1);
  bb->set("own_base_control", arena_valid ? static_cast<int>(events.base_control[own]) : -1);
  bb->set("enemy_base_control", arena_valid ? static_cast<int>(events.base_control[enemy]) : -1);
  bb->set("own_highland_activation", arena_valid ? static_cast<int>(events.highland_activation[own]) : -1);
  bb->set("own_outpost_activation", arena_valid ? static_cast<int>(events.outpost_activation[own]) : -1);
  bb->set("own_fortress_activation", arena_valid ? static_cast<int>(events.fortress_activation[own]) : -1);
  bb->set("own_base_activation", arena_valid ? static_cast<int>(events.base_activation[own]) : -1);
  rm_decision_interfaces::msg::AllyRobotHP hp;
  std::chrono::steady_clock::time_point hp_received;
  const bool score_valid = bb->get("referee_allyRobotHP", hp) && hp.control_score_valid &&
    bb->get("referee_allyRobotHP_received", hp_received) &&
    std::chrono::duration<double>(now - hp_received).count() < input_timeout_sec_;
  bb->set("score_valid", score_valid);
  bb->set("control_lead", score_valid ? static_cast<int>(hp.ally_control_degree) - hp.enemy_control_degree : 0);
  if (score_valid) {
    bb->set("ally_outpost_hp", static_cast<int>(own == 0 ? hp.red_outpost_hp : hp.blue_outpost_hp));
  }
  for (const auto & pair : std::vector<std::pair<std::string, std::string>>{
      {"referee_allyRobotHP", "ally_hp_valid"}, {"referee_eventData", "event_valid"},
      {"referee_switchPosition", "switch_valid"}}) {
    std::chrono::steady_clock::time_point received;
    bb->set(pair.second, bb->get(pair.first + "_received", received) &&
        std::chrono::duration<double>(now - received).count() < input_timeout_sec_);
  }
  std::chrono::steady_clock::time_point rfid_received;
  if (!bb->get("referee_rfidStatus_received", rfid_received) ||
      std::chrono::duration<double>(now - rfid_received).count() >= input_timeout_sec_) {
    bb->set("rfid_supply", false);
    bb->set("rfid_center", false);
    bb->set("referee_rfidStatus", rm_decision_interfaces::msg::RfidStatus{});
  }
}

}  // namespace srm27_behavior
