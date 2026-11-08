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

#include "fake_vel_transform/rate_integration.hpp"

#include <cmath>

namespace fake_vel_transform
{
double integrateMeasuredRate(
  const std::vector<RateSample> & samples, double from, double to, double stale_timeout,
  bool & partial)
{
  partial = false;
  if (!(to > from)) {
    return 0.0;
  }
  if (samples.empty()) {
    partial = true;
    return 0.0;
  }
  const double timeout = (std::isfinite(stale_timeout) && stale_timeout > 0.0) ? stale_timeout : 0.0;

  // 断流：最新样本比区间右端老超过门限 -> 推算项按 0（方案 §4.4.2）。
  if ((to - samples.back().stamp) > timeout) {
    partial = true;
    return 0.0;
  }

  double cursor = from;
  double rate = 0.0;
  bool have_rate = false;
  if (samples.front().stamp <= from) {
    for (const auto & sample : samples) {
      if (sample.stamp > from) {
        break;
      }
      rate = sample.rate;
      have_rate = true;
    }
  } else {
    partial = true;
    cursor = samples.front().stamp;
    rate = samples.front().rate;
    have_rate = true;
  }
  if (!have_rate) {
    partial = true;
    return 0.0;
  }

  double integral = 0.0;
  for (const auto & sample : samples) {
    if (sample.stamp <= cursor) {
      continue;
    }
    if (sample.stamp >= to) {
      break;
    }
    integral += rate * (sample.stamp - cursor);
    cursor = sample.stamp;
    rate = sample.rate;
  }
  if (to > cursor) {
    // 尾段按零阶保持。断流早在上面的门限判断里就整段返回 0 了，所以能走到这里的
    // 尾段一定在门限之内（`to - samples.back().stamp <= timeout`）。
    integral += rate * (to - cursor);
  }
  return integral;
}
}  // namespace fake_vel_transform
