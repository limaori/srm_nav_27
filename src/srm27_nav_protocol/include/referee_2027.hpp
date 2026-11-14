// Copyright 2027 SRM. Licensed under the Apache License, Version 2.0.
#pragma once
#include "packet_typedef.hpp"
#include "rm_decision_interfaces/msg/ally_robot_hp.hpp"
#include "rm_decision_interfaces/msg/event_data.hpp"

namespace srm27_nav_protocol
{
// 通信协议 V1.0.0 (20261029), 表 1-9；R/B 是场地标识，不是己方/敌方。
inline rm_decision_interfaces::msg::EventData decodeEvent2027(const ReceiveEventData2027 & packet)
{
  rm_decision_interfaces::msg::EventData m;
  const auto a = packet.data.event_data, b = packet.data.event_data_2;
  m.supply_zone_non_overlap = a & 1;
  m.small_energy_status = (a >> 1) & 3;
  m.big_energy_status = (a >> 3) & 3;
  m.fortress_activation = {{static_cast<uint8_t>((a >> 5) & 7), static_cast<uint8_t>((a >> 8) & 7)}};
  m.base_activation = {{static_cast<uint8_t>((a >> 11) & 7), static_cast<uint8_t>((a >> 14) & 7)}};
  m.highland_activation = {{static_cast<uint8_t>((a >> 18) & 3), static_cast<uint8_t>((a >> 20) & 3)}};
  m.outpost_activation = {{static_cast<uint8_t>((a >> 22) & 7), static_cast<uint8_t>((a >> 25) & 7)}};
  m.outpost_control = {{static_cast<uint8_t>((a >> 28) & 3), static_cast<uint8_t>((a >> 30) & 3)}};
  m.base_control = {{static_cast<uint8_t>(b & 3), static_cast<uint8_t>((b >> 2) & 3)}};
  m.battlefield_valid = m.outpost_control[0] <= 2 && m.outpost_control[1] <= 2 &&
    m.base_control[0] <= 2 && m.base_control[1] <= 2;
  m.dart_last_hit_time = (b >> 4) & 0x1ff;
  m.dart_last_hit_target = (b >> 13) & 7;
  m.center_gain_point_status = (b >> 24) & 3;
  return m;
}

// 表 1-8：控制度相对己方；前哨 HP 按 R/B 编号，由决策在原始身份新鲜时映射。
inline rm_decision_interfaces::msg::AllyRobotHP decodeHp2027(const ReceiveAllRobotHp2027 & packet)
{
  rm_decision_interfaces::msg::AllyRobotHP m;
  const auto & d = packet.data;
  m.ally_1_robot_hp = d.ally_1_robot_hp;
  m.ally_3_robot_hp = d.ally_3_robot_hp;
  m.ally_4_robot_hp = d.ally_4_robot_hp;
  m.ally_7_robot_hp = d.ally_7_robot_hp;
  m.ally_base_hp = d.ally_base_hp;
  m.red_outpost_hp = d.red_outpost_hp;
  m.blue_outpost_hp = d.blue_outpost_hp;
  m.enemy_base_hp = d.enemy_base_hp;
  m.ally_control_degree = d.ally_control_degree;
  m.enemy_control_degree = d.enemy_control_degree;
  m.control_score_valid = true;
  return m;
}
}  // namespace srm27_nav_protocol
