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

#ifndef SRM27_BEHAVIOR__SRM27_BEHAVIOR_SERVER_HPP_
#define SRM27_BEHAVIOR__SRM27_BEHAVIOR_SERVER_HPP_

#include <behaviortree_cpp/loggers/bt_cout_logger.h>

#include <memory>
#include <string>
#include <vector>

#include "behaviortree_ros2/tree_execution_server.hpp"
#include "rclcpp/rclcpp.hpp"
#include "srm27_behavior/navigation_spin_tracker.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace srm27_behavior
{

class SentryBehaviorServer : public BT::TreeExecutionServer
{
public:
  explicit SentryBehaviorServer(const rclcpp::NodeOptions & options);
  ~SentryBehaviorServer() override { stopExecution(); }

  /**
   * @brief Callback invoked when a goal is received and before the tree is created.
   * If it returns false, the goal will be rejected.
  */
  bool onGoalReceived(const std::string & tree_name, const std::string & payload) override;

  /**
   * @brief Callback invoked after the tree is created.
   * It can be used, for instance, to initialize a logger or the global blackboard.
   *
   * @param tree The tree that was created
  */
  void onTreeCreated(BT::Tree & tree) override;

  /**
   * @brief onLoopAfterTick invoked at each loop, after tree.tickOnce().
   * If it returns a valid NodeStatus, the tree will stop and return that status.
   * Return std::nullopt to continue the execution.
   *
   * @param status The status of the tree after the last tick
  */
  std::optional<BT::NodeStatus> onLoopAfterTick(BT::NodeStatus status) override;

  /**
   * @brief onTreeExecutionCompleted is a callback invoked after the tree execution is completed,
   * i.e. if it returned SUCCESS/FAILURE or if the action was cancelled by the Action Client.
   *
   * @param status The status of the tree after the last tick
   * @param was_cancelled True if the action was cancelled by the Action Client
   *
   * @return if not std::nullopt, the string will be sent as [return_message] to the Action Client.
  */
  std::optional<std::string> onTreeExecutionCompleted(
    BT::NodeStatus status, bool was_cancelled) override;

private:
  template <typename T, typename Hook = std::nullptr_t>
  void subscribe(
    const std::string & topic, const std::string & bb_key,
    const rclcpp::QoS & qos = rclcpp::QoS(10), Hook && hook = nullptr);

  void initBlackboardDefaults();
  void initRefereeInputs();
  void refreshInputState();
  void stopOutputs();
  void initDynamicParameters();
  void publishSpinDebugState(bool force = false);

  std::vector<std::shared_ptr<rclcpp::SubscriptionBase>> subscriptions_;
  std::shared_ptr<NavigationSpinTracker> spin_tracker_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr spin_state_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
  std::string last_spin_debug_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr rotation_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr hold_spin_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr permit_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr guard_client_;
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedFuture guard_future_;
  int64_t last_pose_stamp_ = -1;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr map_pose_timer_;
  std::string map_frame_ = "map";
  std::string robot_base_frame_ = "base_link";
  std::shared_ptr<BT::StdCoutLogger> logger_cout_;
  uint32_t tick_count_ = 0;
  bool use_cout_logger_;
  double input_timeout_sec_ = 2.0;
  int match_epoch_ = 0;
  int previous_progress_ = -1;
  int previous_remain_ = 0;
  int previous_hp_ = -1;
};

}  // namespace srm27_behavior

#endif  // SRM27_BEHAVIOR__SRM27_BEHAVIOR_SERVER_HPP_
