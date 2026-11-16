// Copyright 2027 SRM. Licensed under the Apache License, Version 2.0.
#include <fstream>
#include <gtest/gtest.h>
#include "srm27_behavior/navigation_spin_tracker.hpp"
#include "srm27_behavior/plugins/action/set_sentry_posture.hpp"
#include "srm27_behavior/semantic_map.hpp"
#include "srm27_behavior/waypoint_utils.hpp"

using namespace srm27_behavior;

TEST(DecisionCore, ArrivalHysteresisCompletionAndFrameChanges)
{
  NavigationSpinTracker tracker;
  tracker.setNewGoal(makePoseStamped(1., 0., 0., "map"));
  tracker.updateRobotPose(makePoseStamped(0., 0., 0., "odom"));
  EXPECT_FALSE(tracker.nav_arrived);
  tracker.markGoalFinished();
  EXPECT_FALSE(tracker.nav_arrived);
  tracker.updateRobotPose(makePoseStamped(0.7, 0., 0., "map"));
  EXPECT_TRUE(tracker.nav_arrived);
  tracker.updateRobotPose(makePoseStamped(0.55, 0., 0., "map"));
  EXPECT_TRUE(tracker.nav_arrived);
  tracker.updateRobotPose(makePoseStamped(0.4, 0., 0., "map"));
  EXPECT_FALSE(tracker.nav_arrived);
  tracker.setNewGoal(makePoseStamped(1., 0., 0., "odom"));
  EXPECT_EQ(tracker.active_goal.header.frame_id, "odom");
  EXPECT_FALSE(tracker.nav_arrived);
  tracker.clearGoal();
  EXPECT_FALSE(tracker.has_active_goal);
  EXPECT_FALSE(tracker.nav_goal_finished);
}

TEST(DecisionCore, InvalidInlineRouteNeverExecutesAPartialMission)
{
  std::vector<geometry_msgs::msg::PoseStamped> poses;
  for (const auto & input : {"", "1;2|bad", "1;2|3;nan", "1;2|3;4junk",
                            "1;2|", "1;2;", "1;2;3;4", "|1;2", "1;2|3;;4"}) {
    EXPECT_FALSE(parseInlineWaypoints(input, poses)) << input;
    EXPECT_TRUE(poses.empty()) << input;
  }
  ASSERT_TRUE(parseInlineWaypoints("1; 2 |3;4;0.5 ", poses));
  ASSERT_EQ(poses.size(), 2u);
  EXPECT_DOUBLE_EQ(poses[1].pose.position.x, 3.);
}

TEST(DecisionCore, InvalidYamlRouteAndNonFinitePointAreRejected)
{
  const auto file = std::filesystem::path(::testing::TempDir()) / "srm27-invalid-waypoints.yaml";
  {
    std::ofstream output(file);
    output << "frame_id: map\nnamed_waypoints:\n  bad: {x: .nan, y: 1}\n"
              "waypoint_groups:\n  bad: [{x: 1, y: 2}, {x: 3}]\n";
  }
  geometry_msgs::msg::PoseStamped pose;
  EXPECT_FALSE(loadNamedWaypoint(file.string(), "bad", pose));
  std::vector<geometry_msgs::msg::PoseStamped> poses;
  EXPECT_FALSE(loadNamedWaypointGroup(file.string(), "bad", poses));
  EXPECT_TRUE(poses.empty());
  std::filesystem::remove(file);
  const auto csv = std::filesystem::path(::testing::TempDir()) / "srm27-invalid-waypoints.csv";
  {
    std::ofstream output(csv);
    output << "id,x,y,z,qx,qy,qz,qw\nfirst,1,2,0,0,0,0,1\nbad,3junk,4,0,0,0,0,1\n";
  }
  EXPECT_FALSE(loadWaypointsFromFile(csv.string(), poses));
  EXPECT_TRUE(poses.empty());
  {
    std::ofstream output(csv);
    output << "id,x,y,z,qx,qy,qz,qw\r\nfirst,1,2,0,0,0,0,1\r\n";
  }
  EXPECT_TRUE(loadWaypointsFromFile(csv.string(), poses));
  EXPECT_EQ(poses.size(), 1u);
  std::filesystem::remove(csv);
}

TEST(DecisionCore, PostureCooldownAndUncappedCumulativeTime)
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

TEST(DecisionCore, Nav2CompletionUsesStoppingPoseAndStillDetectsDrift)
{
  NavigationSpinTracker tracker;
  tracker.setNewGoal(makePoseStamped(10., 0., 0., "map"));
  tracker.updateRobotPose(makePoseStamped(9.3, 0., 0., "map"));
  EXPECT_FALSE(tracker.nav_arrived);
  tracker.markGoalFinished();
  EXPECT_TRUE(tracker.nav_arrived);  // Navigation's accepted tolerance can differ from our enter radius.
  tracker.updateRobotPose(makePoseStamped(9.2, 0., 0., "map"));
  EXPECT_TRUE(tracker.nav_arrived);
  tracker.updateRobotPose(makePoseStamped(8.7, 0., 0., "map"));
  EXPECT_FALSE(tracker.nav_arrived);
}


TEST(DecisionCore, DistantNavigationSuccessNeverCreatesStationarySpin)
{
  NavigationSpinTracker tracker;
  tracker.setNewGoal(makePoseStamped(10., 0., 0., "map"));
  tracker.updateRobotPose(makePoseStamped(0., 0., 0., "map"));
  EXPECT_FALSE(tracker.markGoalFinished());
  EXPECT_FALSE(tracker.nav_goal_finished);
  EXPECT_FALSE(tracker.nav_arrived);
  tracker.updateRobotPose(makePoseStamped(9.3, 0., 0., "map"));
  tracker.arrive_tolerance = tracker.leave_tolerance = .05;
  EXPECT_TRUE(tracker.markGoalFinished());  // Spin tuning must not change Nav2 completion acceptance.
}

TEST(DecisionCore, SpinReferenceIsBoundedSmoothReproducibleAndResets)
{
  SpinReference a, b;
  double prev = 0.0, prev_accel = 0.0;
  constexpr double dt = 0.001;
  double low = 8.0, high = 0.0;
  for (int i = 0; i < 20000; ++i) {
    const double now = i * dt;
    const double request = i < 7000 ? 7.0 : (i < 14000 ? 2.5 : 8.0);
    const bool variable = i < 7000 || i >= 14000;
    const double v = a.sample(now, request, variable, 8.0, 1, 42, 8.0, 40.0, 3.0);
    EXPECT_DOUBLE_EQ(v, b.sample(now, request, variable, 8.0, 1, 42, 8.0, 40.0, 3.0));
    EXPECT_GE(v, 0.0);
    EXPECT_LE(v, 8.0);
    const double accel = (v - prev) / dt;
    if (i > 1) {
      EXPECT_LE(std::abs(accel), 8.001);
      EXPECT_LE(std::abs((accel - prev_accel) / dt), 40.02);
    }
    if (i > 3000 && i < 7000) { low = std::min(low, v); high = std::max(high, v); }
    prev = v;
    prev_accel = accel;
  }
  EXPECT_GT(high - low, 0.2);
  EXPECT_LE(a.sample(20.0, 8.0, true, 2.0, 1, 42, 8.0, 40.0, 3.0), 2.0);
  EXPECT_DOUBLE_EQ(a.sample(20.001, 0.0, true, 8.0, 1, 42, 8.0, 40.0, 3.0), 0.0);
  EXPECT_DOUBLE_EQ(a.sample(20.002, 7.0, true, 8.0, 2, 42, 8.0, 40.0, 3.0), 0.0);
  EXPECT_DOUBLE_EQ(a.sample(22.0, 7.0, true, 8.0, 2, 42, 8.0, 40.0, 3.0), 0.0);
  EXPECT_DOUBLE_EQ(a.sample(22.001, NAN, true, 8.0, 2, 42, 8.0, 40.0, 3.0), 0.0);
}

TEST(DecisionCore, RandomSpinMinimumCanChangeWithoutResettingTheReference)
{
  SpinReference reference;
  reference.reset(7.0);
  double previous = 7.0, low = 8.0;
  for (int i = 0; i < 2000; ++i) {
    const double minimum = i < 500 ? 7.0 : 3.0;
    const double value = reference.sample(i * .01, 8.0, true, 8.0, 1, 42, 8.0, 40.0, minimum);
    EXPECT_LE(std::abs(value - previous), .080001);
    EXPECT_GE(value, minimum);
    EXPECT_LE(value, 8.0);
    if (i >= 500) { low = std::min(low, value); }
    previous = value;
  }
  EXPECT_LT(low, 5.0);
}

TEST(DecisionCore, RandomSpinStartsFromPreviousSpeedAndRejectsInvalidInputs)
{
  SpinReference reference;
  reference.reset(2.5);
  EXPECT_DOUBLE_EQ(reference.sample(0.0, 8.0, true, 8.0, 1, 42, 8.0, 40.0, 3.0), 2.5);
  double previous = 2.5;
  for (int i = 1; i <= 200; ++i) {
    const double value = reference.sample(i * .01, 8.0, true, 8.0, 1, 42, 8.0, 40.0, 3.0);
    EXPECT_GT(value, 0.0);
    EXPECT_LE(std::abs(value - previous), .080001);
    previous = value;
  }
  EXPECT_DOUBLE_EQ(reference.sample(2.01, -8.0, true, 8.0, 1, 42, 8.0, 40.0, 3.0), 0.0);
  EXPECT_DOUBLE_EQ(reference.sample(2.02, 8.0, true, 8.0, 1, 42, 0.0, 40.0, 3.0), 0.0);
  EXPECT_DOUBLE_EQ(reference.sample(2.03, 8.0, true, 8.0, 1, 42, 8.0, NAN, 3.0), 0.0);
}

TEST(DecisionCore, RandomSpinHasNoFixedPeriodAndCanReturnToConstantSpeed)
{
  SpinReference reference;
  std::vector<double> samples;
  double now = 0.0;
  for (int i = 0; i < 12000; ++i) {
    now = i * .01;
    samples.push_back(reference.sample(now, 8.0, true, 8.0, 1, 42, 8.0, 40.0, 3.0));
  }
  // 排除 0.2~5 秒的重复周期；这不代表对任意预测器都不可预测。
  for (int lag = 20; lag <= 500; ++lag) {
    double squared_error = 0.0;
    for (size_t i = 1000; i + lag < samples.size(); ++i) {
      squared_error += std::pow(samples[i + lag] - samples[i], 2);
    }
    EXPECT_GT(squared_error / (samples.size() - 1000 - lag), .001);
  }
  for (int i = 1; i <= 500; ++i) {
    reference.sample(now + i * .01, 2.5, false, 8.0, 1, 42, 8.0, 40.0, 3.0);
  }
  EXPECT_TRUE(reference.settled(2.5));
}
