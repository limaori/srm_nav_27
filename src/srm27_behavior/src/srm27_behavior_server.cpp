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
namespace srm27_behavior
{

void SentryBehaviorServer::initBlackboardDefaults()
{
  auto bb = globalBlackboard();
  bb->set("motion_allowed", false);
  bb->set("guard_ready", false);
  bb->set("decision_mode", std::string("wait"));
  bb->set("decision_reason", std::string("initializing"));
  bb->set("nav_fault", std::string(""));
  bb->set("robot_id", 0);
  bb->set("chassis_enabled", false);
  bb->set("inputs_ready", false);
  bb->set("needs_supply", false);
  bb->set("supply_cooling", false);
  bb->set("match_epoch", 0);
  bb->set<BT::TreeNode *>("navigation_owner", nullptr);
  bb->set("sentry_posture_state", std::make_shared<SentryPostureState>());
  bb->set("current_posture", 0);

  // Flat referee scalar defaults so XML ScriptCondition never fails on unset keys
  bb->set("game_valid", false);
  bb->set("game_type", 0);
  bb->set("game_progress", 0);
  bb->set("remain_time", 0);

  bb->set("status_valid", false);
  bb->set("heat_valid", false);
  bb->set("ammo_valid", false);
  bb->set("hp", 0);
  bb->set("max_hp", 400);
  bb->set("heat", 0);
  bb->set("ammo", 0);
  bb->set("gold", 0);

  bb->set("ally_hp_valid", false);
  bb->set("ally_outpost_hp", 0);
  bb->set("ally_base_hp", 0);
  bb->set("ally_sentry_hp", 0);

  bb->set("arena_valid", false);
  bb->set("score_valid", false);
  bb->set("control_lead", 0);
  for (const auto key : {"own_outpost_control", "enemy_outpost_control", "own_base_control",
      "enemy_base_control", "own_highland_activation", "own_outpost_activation",
      "own_fortress_activation", "own_base_activation"}) { bb->set(key, -1); }
  bb->set("event_valid", false);
  bb->set("center_status", -1);
  bb->set("fortress_status", -1);
  bb->set("central_highland_status", -1);
  bb->set("big_energy_status", -1);

  bb->set("switch_valid", false);
  bb->set("switch_position", -1);

  bb->set("rfid_supply", false);
  bb->set("rfid_center", false);
  bb->set("rfid_fortress", false);

  bb->set("nav_spin_tracker", spin_tracker_);
  spin_tracker_->syncToBlackboard();
}

void SentryBehaviorServer::initDynamicParameters()
{
  node()->declare_parameter("spin.moving_speed", spin_tracker_->moving_speed);
  node()->declare_parameter("spin.arrived_speed", spin_tracker_->arrived_speed);
  node()->declare_parameter("spin.supply_speed", spin_tracker_->supply_speed);
  node()->declare_parameter("spin.evasion_speed", spin_tracker_->evasion_speed);
  node()->declare_parameter("spin.arrive_tolerance", spin_tracker_->arrive_tolerance);
  node()->declare_parameter("spin.leave_tolerance", spin_tracker_->leave_tolerance);

  {
    std::lock_guard<std::mutex> lock(spin_tracker_->mutex);
    node()->get_parameter("spin.moving_speed", spin_tracker_->moving_speed);
    node()->get_parameter("spin.arrived_speed", spin_tracker_->arrived_speed);
    node()->get_parameter("spin.supply_speed", spin_tracker_->supply_speed);
    node()->get_parameter("spin.evasion_speed", spin_tracker_->evasion_speed);
    node()->get_parameter("spin.arrive_tolerance", spin_tracker_->arrive_tolerance);
    node()->get_parameter("spin.leave_tolerance", spin_tracker_->leave_tolerance);
    spin_tracker_->evaluateLocked();
    spin_tracker_->syncToBlackboard();
  }

  auto validate = [](double moving, double arrived, double supply, double evasion,
                     double enter, double leave) {
    return std::isfinite(moving) && std::isfinite(arrived) && std::isfinite(supply) &&
           std::isfinite(evasion) && std::isfinite(enter) && std::isfinite(leave) &&
           enter >= 0.05 && leave >= enter;
  };
  if (!validate(spin_tracker_->moving_speed, spin_tracker_->arrived_speed,
                spin_tracker_->supply_speed, spin_tracker_->evasion_speed,
                spin_tracker_->arrive_tolerance, spin_tracker_->leave_tolerance)) {
    throw std::invalid_argument("Invalid spin speeds or arrival tolerances");
  }
  param_cb_handle_ = node()->add_on_set_parameters_callback(
    [this, validate](const std::vector<rclcpp::Parameter> & params) {
      rcl_interfaces::msg::SetParametersResult result;
      const auto values = node()->get_parameters({"spin.moving_speed", "spin.arrived_speed",
        "spin.supply_speed", "spin.evasion_speed", "spin.arrive_tolerance", "spin.leave_tolerance"});
      double moving = values[0].as_double(), arrived = values[1].as_double();
      double supply = values[2].as_double(), evasion = values[3].as_double();
      double enter = values[4].as_double(), leave = values[5].as_double();
      for (const auto & p : params) {
        const auto & n = p.get_name();
        if (n == "spin.moving_speed") { moving = p.as_double(); }
        else if (n == "spin.arrived_speed") { arrived = p.as_double(); }
        else if (n == "spin.supply_speed") { supply = p.as_double(); }
        else if (n == "spin.evasion_speed") { evasion = p.as_double(); }
        else if (n == "spin.arrive_tolerance") { enter = p.as_double(); }
        else if (n == "spin.leave_tolerance") { leave = p.as_double(); }
      }
      result.successful = validate(moving, arrived, supply, evasion, enter, leave);
      if (!result.successful) {
        result.reason = "Speeds must be finite; 0.05 <= arrive_tolerance <= leave_tolerance";
        return result;
      }
      // Apply only committed parameters in the tick loop; another callback may still reject this batch.
      return result;
    });
}

void SentryBehaviorServer::publishSpinDebugState(bool force)
{
  if (!spin_state_pub_) {
    return;
  }
  std::string mode;
  double wz = 0.0;
  {
    std::lock_guard<std::mutex> lock(spin_tracker_->mutex);
    mode = spin_tracker_->last_spin_mode;
    wz = spin_tracker_->last_cmd_wz;
  }
  const std::string line = spin_tracker_->formatDebugState(mode, wz);
  if (force || line != last_spin_debug_) {
    last_spin_debug_ = line;
    std_msgs::msg::String msg;
    msg.data = line;
    spin_state_pub_->publish(msg);
  }
}

SentryBehaviorServer::SentryBehaviorServer(const rclcpp::NodeOptions & options)
: TreeExecutionServer(options)
{
  node()->declare_parameter("use_cout_logger", false);
  node()->get_parameter("use_cout_logger", use_cout_logger_);
  node()->declare_parameter("map_frame", std::string("map"));
  node()->get_parameter("map_frame", map_frame_);
  node()->declare_parameter("robot_base_frame", std::string("base_link"));
  node()->get_parameter("robot_base_frame", robot_base_frame_);

  spin_tracker_ = std::make_shared<NavigationSpinTracker>();
  spin_tracker_->completion_tolerance = node()->declare_parameter("navigation_completion_tolerance", 0.8);
  if (!std::isfinite(spin_tracker_->completion_tolerance) || spin_tracker_->completion_tolerance <= 0) {
    throw std::invalid_argument("navigation_completion_tolerance must be positive and finite");
  }
  spin_tracker_->global_bb = globalBlackboard();
  spin_state_pub_ = node()->create_publisher<std_msgs::msg::String>("behavior/spin_state", 10);

  rotation_pub_ = node()->create_publisher<geometry_msgs::msg::Twist>("rotation_cmd", 1);
  hold_spin_pub_ = node()->create_publisher<std_msgs::msg::Float64>("behavior/hold_spin", 1);
  permit_pub_ = node()->create_publisher<std_msgs::msg::Bool>("decision/motion_permit", 1);
  status_pub_ = node()->create_publisher<std_msgs::msg::String>("decision/status", 10);
  guard_client_ = node()->create_client<rcl_interfaces::srv::SetParameters>("srm_cmd_mux/set_parameters");
  initBlackboardDefaults();
  initDynamicParameters();
  input_timeout_sec_ = node()->declare_parameter("input_timeout_sec", 2.0);
  if (!std::isfinite(input_timeout_sec_) || input_timeout_sec_ <= 0.0) {
    throw std::invalid_argument("input_timeout_sec must be positive and finite");
  }
  auto bb = globalBlackboard();
  bb->set("dry_run", node()->declare_parameter("dry_run", true));
  bb->set("waypoints_calibrated", node()->declare_parameter("waypoints_calibrated", false));
  bb->set("uc_resupply_enabled", node()->declare_parameter("uc_resupply_enabled", true));
  bb->set("uc_enemy_base_enabled", node()->declare_parameter("uc_enemy_base_enabled", false));
  for (const auto & entry : std::vector<std::pair<std::string, std::string>>{
      {"waypoints_file", "params/waypoints.yaml"},
      {"ul_control_waypoint", "rmul_center"},
      {"ul_semantic_map_file", "params/rmul_semantic_map.yaml"},
      {"uc_semantic_map_file", "params/rmuc_semantic_map.yaml"}}) {
    bb->set(entry.first, node()->declare_parameter(entry.first, entry.second));
  }
  for (const auto & setting : std::vector<std::pair<std::string, int>>{
      {"ul_hp_trigger", 150}, {"ul_hp_exit", 360}, {"uc_hp_trigger", 140},
      {"uc_hp_exit", 360}, {"uc_ammo_trigger", 30}, {"uc_ammo_exit", 150},
      {"uc_base_defense_hp", 2000}, {"uc_early_remain", 240},
      {"uc_attack_hp", 250}, {"uc_endgame_remain", 60}}) {
    globalBlackboard()->set(setting.first, static_cast<int>(node()->declare_parameter(setting.first, setting.second)));
  }

  for (const auto & setting : std::vector<std::pair<std::string, double>>{
      {"supply_timeout", 25.0}, {"supply_retry", 15.0}, {"navigation_timeout", 45.0}}) {
    const double value = node()->declare_parameter(setting.first, setting.second);
    if (!std::isfinite(value) || value <= 0) { throw std::invalid_argument(setting.first + " must be positive"); }
    bb->set(setting.first, value);
  }
  initRefereeInputs();
}

void SentryBehaviorServer::stopOutputs()
{
  spin_tracker_->clearGoal();
  if (!rclcpp::ok() || globalBlackboard()->get<bool>("dry_run")) { return; }
  rotation_pub_->publish(geometry_msgs::msg::Twist{});
  hold_spin_pub_->publish(std_msgs::msg::Float64{});
  if (!globalBlackboard()->get<bool>("dry_run")) {
    permit_pub_->publish(std_msgs::msg::Bool{});
  }
  globalBlackboard()->set("motion_allowed", false);
}

bool SentryBehaviorServer::onGoalReceived(
  const std::string & tree_name, const std::string & payload)
{
  RCLCPP_INFO(
    node()->get_logger(), "onGoalReceived with tree name '%s' with payload '%s'", tree_name.c_str(),
    payload.c_str());
  return true;
}

void SentryBehaviorServer::onTreeCreated(BT::Tree & tree)
{
  globalBlackboard()->set<BT::TreeNode *>("navigation_owner", nullptr);
  if (use_cout_logger_) {
    logger_cout_ = std::make_shared<BT::StdCoutLogger>(tree);
  }
  tick_count_ = 0;
  publishSpinDebugState(true);
}

std::optional<BT::NodeStatus> SentryBehaviorServer::onLoopAfterTick(BT::NodeStatus /*status*/)
{
  ++tick_count_;
  auto bb = globalBlackboard();
  const bool dry_run = bb->get<bool>("dry_run");
  if (!dry_run && !bb->get<bool>("guard_ready")) {
    if (guard_future_.valid()) {
      if (guard_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        const auto response = guard_future_.get();
        bb->set("guard_ready", response && response->results.size() == 1 && response->results[0].successful);
        guard_future_ = {};
      }
    } else if (guard_client_->service_is_ready()) {
      auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
      request->parameters.push_back(rclcpp::Parameter("decision_required", true).to_parameter_msg());
      guard_future_ = guard_client_->async_send_request(request).future.share();
    }
  }
  {
    const auto values = node()->get_parameters({"spin.moving_speed", "spin.arrived_speed",
      "spin.supply_speed", "spin.evasion_speed", "spin.arrive_tolerance", "spin.leave_tolerance"});
    std::lock_guard<std::mutex> lock(spin_tracker_->mutex);
    spin_tracker_->moving_speed = values[0].as_double();
    spin_tracker_->arrived_speed = values[1].as_double();
    spin_tracker_->supply_speed = values[2].as_double();
    spin_tracker_->evasion_speed = values[3].as_double();
    spin_tracker_->arrive_tolerance = values[4].as_double();
    spin_tracker_->leave_tolerance = values[5].as_double();
    spin_tracker_->evaluateLocked();
    spin_tracker_->syncToBlackboard();
  }
  std_msgs::msg::Float64 hold;
  bool stationary = false;
  const bool enabled = !dry_run && bb->get<bool>("guard_ready") && bb->get<bool>("motion_allowed") &&
    bb->get<bool>("inputs_ready") && bb->get<bool>("status_valid") && bb->get<bool>("chassis_enabled") &&
    bb->get<int>("hp") > 0 && bb->get<int>("game_progress") == 4 && bb->get<int>("remain_time") > 0;
  if (enabled) {
    std::lock_guard<std::mutex> lock(spin_tracker_->mutex);
    const bool reached = spin_tracker_->nav_goal_finished && spin_tracker_->nav_arrived;
    const bool supplying = spin_tracker_->last_spin_mode == "SUPPLY_HOLD" &&
      !spin_tracker_->has_active_goal && bb->get<bool>("rfid_supply");
    if ((reached || supplying) && spin_tracker_->last_spin_mode != "MATCH_STOP") {
      stationary = true;
      hold.data = spin_tracker_->last_cmd_wz;
    }
  }
  const bool permit = enabled && (bb->get<bool>("navigation_ready") || stationary);
  if (!dry_run) {
    std_msgs::msg::Bool allowed;
    allowed.data = permit;
    permit_pub_->publish(allowed);
    hold_spin_pub_->publish(hold);
  }
  std::ostringstream json;
  json << std::boolalpha << "{\"mode\":\"" << bb->get<std::string>("decision_mode")
       << "\",\"reason\":\"" << bb->get<std::string>("decision_reason")
       << "\",\"dry_run\":" << dry_run << ",\"motion_permit\":" << permit
       << ",\"guard_ready\":" << bb->get<bool>("guard_ready")
       << ",\"inputs_ready\":" << bb->get<bool>("inputs_ready")
       << ",\"arrived\":" << bb->get<bool>("nav_arrived")
       << ",\"nav_fault\":\"" << bb->get<std::string>("nav_fault") << "\""
       << ",\"hp\":" << bb->get<int>("hp") << ",\"ammo\":" << bb->get<int>("ammo")
       << ",\"ammo_valid\":" << bb->get<bool>("ammo_valid")
       << ",\"arena_valid\":" << bb->get<bool>("arena_valid")
       << ",\"score_valid\":" << bb->get<bool>("score_valid")
       << ",\"control_lead\":" << bb->get<int>("control_lead")
       << ",\"own_outpost_control\":" << bb->get<int>("own_outpost_control")
       << ",\"enemy_outpost_control\":" << bb->get<int>("enemy_outpost_control")
       << ",\"waypoints_calibrated\":" << bb->get<bool>("waypoints_calibrated")
       << ",\"posture_transport\":\"unavailable\"}";
  std_msgs::msg::String report;
  report.data = json.str();
  status_pub_->publish(report);
  publishSpinDebugState(false);
  return std::nullopt;
}

std::optional<std::string> SentryBehaviorServer::onTreeExecutionCompleted(
  BT::NodeStatus status, bool was_cancelled)
{
  RCLCPP_INFO(
    node()->get_logger(), "onTreeExecutionCompleted with status=%d (canceled=%d) after %d ticks",
    static_cast<int>(status), was_cancelled, tick_count_);
  stopOutputs();
  logger_cout_.reset();
  std::string result = treeName() +
                       " tree completed with status=" + std::to_string(static_cast<int>(status)) +
                       " after " + std::to_string(tick_count_) + " ticks";
  return result;
}

}  // namespace srm27_behavior

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto action_server = std::make_shared<srm27_behavior::SentryBehaviorServer>(options);

  RCLCPP_INFO(action_server->node()->get_logger(), "Starting SentryBehaviorServer");

  rclcpp::executors::MultiThreadedExecutor exec(
    rclcpp::ExecutorOptions(), 2, false, std::chrono::milliseconds(250));
  exec.add_node(action_server->node());
  exec.spin();
  exec.remove_node(action_server->node());

  rclcpp::shutdown();
}
