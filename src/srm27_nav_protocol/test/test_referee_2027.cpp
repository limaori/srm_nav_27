// Copyright 2027 SRM. Licensed under the Apache License, Version 2.0.
#include <gtest/gtest.h>
#include "referee_2027.hpp"
using namespace srm27_nav_protocol;

TEST(Referee2027, FieldStatesKeepRedBlueAndUnknownActivationSeparate)
{
  ReceiveEventData2027 p{};
  p.data.event_data = (1u << 28) | (2u << 30) | (4u << 5) | (3u << 25) | (2u << 20);
  p.data.event_data_2 = (2u << 2) | (321u << 4) | (6u << 13) | (3u << 24);
  const auto m = decodeEvent2027(p);
  EXPECT_TRUE(m.battlefield_valid);
  EXPECT_EQ(m.outpost_control[0], 1);
  EXPECT_EQ(m.outpost_control[1], 2);
  EXPECT_EQ(m.base_control[0], 0);
  EXPECT_EQ(m.base_control[1], 2);
  EXPECT_EQ(m.fortress_activation[0], 4);  // unavailable is not enemy activation
  EXPECT_EQ(m.outpost_activation[1], 3);
  EXPECT_EQ(m.highland_activation[1], 2);
  EXPECT_EQ(m.dart_last_hit_time, 321);
  EXPECT_EQ(m.dart_last_hit_target, 6);
  EXPECT_EQ(m.center_gain_point_status, 3);
  p.data.event_data |= 3u << 28;
  EXPECT_FALSE(decodeEvent2027(p).battlefield_valid);
}

TEST(Referee2027, HpAndControlScoresUse2027Offsets)
{
  ReceiveAllRobotHp2027 p{};
  p.data.ally_3_robot_hp = 180;
  p.data.ally_7_robot_hp = 380;
  p.data.red_outpost_hp = 0;
  p.data.blue_outpost_hp = 1400;
  p.data.ally_base_hp = 4000;
  p.data.enemy_base_hp = 3000;
  p.data.ally_control_degree = 123;
  p.data.enemy_control_degree = 456;
  const auto m = decodeHp2027(p);
  EXPECT_TRUE(m.control_score_valid);
  EXPECT_EQ(m.ally_3_robot_hp, 180);
  EXPECT_EQ(m.ally_7_robot_hp, 380);
  EXPECT_EQ(m.red_outpost_hp, 0);
  EXPECT_EQ(m.blue_outpost_hp, 1400);
  EXPECT_EQ(m.ally_base_hp, 4000);
  EXPECT_EQ(m.enemy_base_hp, 3000);
  EXPECT_EQ(m.ally_control_degree, 123);
  EXPECT_EQ(m.enemy_control_degree, 456);
}
