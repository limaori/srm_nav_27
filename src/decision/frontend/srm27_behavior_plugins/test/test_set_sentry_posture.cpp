// Copyright 2027 SRM. Licensed under the Apache License, Version 2.0.
//
// SetSentryPosture 插件的行为测试（原 test_decision_core 的 Posture 用例）。
// 2026-11-24 决策模块拆包时从 core 测试移到插件包：该用例需要真正的 BT 节点，
// 而 core 包只保留无插件的共享状态/算法。

#include <gtest/gtest.h>

#include <memory>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "rclcpp/rclcpp.hpp"
#include "srm27_behavior_core/sentry_posture_state.hpp"
#include "srm27_behavior_plugins/action/set_sentry_posture.hpp"

using namespace srm27_behavior;

TEST(SetSentryPostureActionTest, PostureCooldownAndUncappedCumulativeTime)
{
  rclcpp::init(0, nullptr);
  {
    auto node = std::make_shared<rclcpp::Node>("posture_budget_test");
    auto bb = BT::Blackboard::create();
    auto state = std::make_shared<SentryPostureState>();
    bb->set("sentry_posture_state", state);
    bb->set("current_posture", 0);
    BT::RosNodeParams params;
    params.nh = node;
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<SetSentryPostureAction>("SetSentryPosture", params);
    auto tree = factory.createTreeFromText(
      R"(<root BTCPP_format="4"><BehaviorTree ID="Budget">
        <SetSentryPosture posture="MOVE" fallback_posture="DEFENSE"
                          topic_name="posture_request"/>
      </BehaviorTree></root>)", bb);
    state->used_sec[0] = 170.;
    state->has_last_tick = true;
    state->last_tick_time = node->now() - rclcpp::Duration::from_seconds(15.);
    state->has_switched = true;
    state->last_switch_time = node->now() - rclcpp::Duration::from_seconds(4.);
    tree.tickExactlyOnce();
    EXPECT_GT(state->used_sec[0], 184.);
    EXPECT_EQ(bb->get<int>("current_posture"), 0);  // 5s cooldown still active.
    state->last_switch_time = node->now() - rclcpp::Duration::from_seconds(6.);
    tree.tickExactlyOnce();
    EXPECT_EQ(bb->get<int>("current_posture"), 2);  // Budget fallback after cooldown.
    state->reset();
    EXPECT_EQ(state->used_sec[0], 0.);
    EXPECT_FALSE(state->has_switched);
  }
  rclcpp::shutdown();
}
