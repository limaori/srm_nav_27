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

#include "srm27_chassis_control/twist_watchdog.hpp"

#include <cmath>

namespace srm27_chassis_control
{
TwistWatchdog::TwistWatchdog(double _timeoutSeconds)
{
  this->setTimeout(_timeoutSeconds);
}

void TwistWatchdog::setTimeout(double _timeoutSeconds)
{
  if (!std::isfinite(_timeoutSeconds) || _timeoutSeconds < 0.0) {
    this->timeout_ = 0.0;
    return;
  }
  this->timeout_ = _timeoutSeconds;
}

double TwistWatchdog::timeout() const
{
  return this->timeout_;
}

void TwistWatchdog::reset()
{
  this->received_ = false;
  this->last_ = TimePoint{};
}

void TwistWatchdog::markInput(TimePoint _now)
{
  this->received_ = true;
  this->last_ = _now;
}

bool TwistWatchdog::hasInput() const
{
  return this->received_;
}

bool TwistWatchdog::expired(TimePoint _now) const
{
  if (!this->received_) {
    return true;
  }
  if (this->timeout_ <= 0.0) {
    return true;
  }
  return elapsedSeconds(this->last_, _now) > this->timeout_;
}

double elapsedSeconds(TwistWatchdog::TimePoint _from, TwistWatchdog::TimePoint _to)
{
  const std::chrono::duration<double> delta = _to - _from;
  return delta.count();
}
}  // namespace srm27_chassis_control
