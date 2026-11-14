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

#ifndef SRM27_BEHAVIOR__SENTRY_POSTURE_STATE_HPP_
#define SRM27_BEHAVIOR__SENTRY_POSTURE_STATE_HPP_

#include <array>
#include <cctype>
#include <cstdint>
#include <mutex>
#include <string>

#include "rclcpp/time.hpp"

namespace srm27_behavior
{

enum class SentryPosture : uint8_t
{
  MOVE = 0,
  ATTACK = 1,
  DEFENSE = 2,
};

inline bool parseSentryPosture(const std::string & raw, uint8_t & out)
{
  std::string norm;
  norm.reserve(raw.size());
  for (unsigned char c : raw) {
    if (!std::isspace(c)) {
      norm.push_back(static_cast<char>(std::toupper(c)));
    }
  }
  if (norm == "0" || norm == "MOVE") {
    out = static_cast<uint8_t>(SentryPosture::MOVE);
    return true;
  }
  if (norm == "1" || norm == "ATTACK") {
    out = static_cast<uint8_t>(SentryPosture::ATTACK);
    return true;
  }
  if (norm == "2" || norm == "DEFENSE") {
    out = static_cast<uint8_t>(SentryPosture::DEFENSE);
    return true;
  }
  return false;
}

inline const char * sentryPostureName(uint8_t posture)
{
  switch (posture) {
    case 0:
      return "MOVE";
    case 1:
      return "ATTACK";
    case 2:
      return "DEFENSE";
    default:
      return "UNKNOWN";
  }
}

/**
 * @brief Shared posture cooldown & cumulative 3-minute budget tracker across all BT subtrees
 *        per RMUC 2027 rule section 5.7.2.
 */
struct SentryPostureState
{
  std::mutex mutex;
  uint8_t active_posture = static_cast<uint8_t>(SentryPosture::MOVE);
  bool has_last_tick = false;
  bool has_switched = false;
  rclcpp::Time last_tick_time{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_switch_time{0, 0, RCL_ROS_TIME};
  std::array<double, 3> used_sec{{0.0, 0.0, 0.0}};

  void reset()
  {
    std::lock_guard<std::mutex> lock(mutex);
    active_posture = static_cast<uint8_t>(SentryPosture::MOVE);
    has_last_tick = false;
    has_switched = false;
    used_sec = {{0.0, 0.0, 0.0}};
  }
};

}  // namespace srm27_behavior

#endif  // SRM27_BEHAVIOR__SENTRY_POSTURE_STATE_HPP_
