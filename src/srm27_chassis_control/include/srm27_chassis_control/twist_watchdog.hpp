// Copyright 2026 SRM
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

#ifndef SRM27_CHASSIS_CONTROL__TWIST_WATCHDOG_HPP_
#define SRM27_CHASSIS_CONTROL__TWIST_WATCHDOG_HPP_

#include <chrono>

namespace srm27_chassis_control
{
/// \brief 速度输入超时检测。
///
/// 接收侧记录最近一次收到消息的时间，守护线程用单调时钟判断是否超时，
/// 超时后把对应速度清零。时间点由调用方传入，便于单元测试。
class TwistWatchdog
{
public:
  using TimePoint = std::chrono::steady_clock::time_point;

  explicit TwistWatchdog(double _timeoutSeconds = 0.1);

  /// \brief 设置超时时间（秒）。非有限值或负值按 0（立即超时）处理。
  void setTimeout(double _timeoutSeconds);

  /// \brief 当前超时时间（秒）。
  double timeout() const;

  /// \brief 清空输入与计时，回到"从未收到输入"的状态。
  void reset();

  /// \brief 记录一次输入。
  void markInput(TimePoint _now);

  /// \brief 是否收到过至少一次输入。
  bool hasInput() const;

  /// \brief 在给定时刻是否已经超时。
  ///
  /// 从未收到输入时返回 true：没有输入就不应该继续按旧速度运动。
  bool expired(TimePoint _now) const;

private:
  double timeout_{0.1};
  bool received_{false};
  TimePoint last_{};
};

/// \brief 比较两个时间点的间隔（秒）。
double elapsedSeconds(TwistWatchdog::TimePoint _from, TwistWatchdog::TimePoint _to);
}  // namespace srm27_chassis_control

#endif  // SRM27_CHASSIS_CONTROL__TWIST_WATCHDOG_HPP_
