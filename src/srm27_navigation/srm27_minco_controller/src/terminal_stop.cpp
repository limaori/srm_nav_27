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

#include "srm27_minco_controller/terminal_stop.hpp"

#include <algorithm>
#include <cmath>

namespace srm27_minco_controller
{

void TerminalStop::configure(const Config & _config)
{
  config_ = _config;
  // 负值没有意义，且会让状态机在阈值附近抖动。
  if (!std::isfinite(config_.tolerance) || config_.tolerance < 0.0) {
    config_.tolerance = 0.0;
  }
  if (!std::isfinite(config_.fallback_tolerance) || config_.fallback_tolerance < 0.0) {
    config_.fallback_tolerance = 0.0;
  }
  if (!std::isfinite(config_.exit_margin) || config_.exit_margin < 0.0) {
    config_.exit_margin = 0.0;
  }
  reset();
}

void TerminalStop::reset()
{
  active_ = false;
  entry_tolerance_ = 0.0;
  exit_tolerance_ = 0.0;
}

bool TerminalStop::update(double _distance, double _goal_tolerance)
{
  // 解析本次生效的进入阈值：显式配置 > 目标检查器 > 兜底。
  double tolerance = config_.tolerance;
  if (!(tolerance > 0.0)) {
    const bool usable = std::isfinite(_goal_tolerance) && _goal_tolerance > 0.0;
    tolerance = usable ? _goal_tolerance : config_.fallback_tolerance;
  }
  entry_tolerance_ = std::max(0.0, tolerance);
  exit_tolerance_ = entry_tolerance_ + config_.exit_margin;

  if (!config_.enabled || !(entry_tolerance_ > 0.0)) {
    active_ = false;
    return false;
  }

  // 距离不可判定时保持现状：不主动进入，也不解除已有急停 ——
  // 解除必须有明确依据，否则一次坏样本就会让车重新开始追末点。
  if (std::isfinite(_distance)) {
    if (active_) {
      if (_distance > exit_tolerance_) {
        active_ = false;
      }
    } else if (_distance <= entry_tolerance_) {
      // 只看位置，不看速度：真正的停车剖面本来就会在容差内以较高速度进来。
      active_ = true;
    }
  }
  return active_;
}

}  // namespace srm27_minco_controller
