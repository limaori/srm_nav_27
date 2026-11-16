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

#ifndef FAKE_VEL_TRANSFORM__RATE_INTEGRATION_HPP_
#define FAKE_VEL_TRANSFORM__RATE_INTEGRATION_HPP_

#include <vector>

namespace fake_vel_transform
{
/// \brief 一条实测转速样本（秒，rad/s）。样本必须按时间**递增**排列。
struct RateSample
{
  double stamp{0.0};
  double rate{0.0};
};

/// \brief 对分段常值（零阶保持）的**实测转速**在 `[from, to]` 上积分。
///
/// 与"积分下发指令"的区别（方案 §4.4.2）：样本是测量值，不是要执行的命令，
/// 因此：
///   * 最新样本比 `to` 老超过 `stale_timeout` -> 视为断流，直接返回 0（不瞎猜）；
///   * 区间左端没有覆盖（最早样本晚于 `from`）-> 从最早样本开始积分并置 `partial`；
///   * 尾段只保持到"最后一条样本 + `stale_timeout`"，再往后按 0 处理。
/// 这两种降级都只是"少算"，不会把断流放大成一个虚假的转角。
///
/// \param[in]  samples 按时间递增的样本（可为空）。
/// \param[in]  from    积分左端（秒）。
/// \param[in]  to      积分右端（秒）。
/// \param[in]  stale_timeout 允许的最大断流（秒）；<= 0 时任何尾段都按断流处理。
/// \param[out] partial 是否有任何一段没被样本覆盖（诊断用）。
/// \return 积分结果（rad）。
double integrateMeasuredRate(
  const std::vector<RateSample> & samples, double from, double to, double stale_timeout,
  bool & partial);
}  // namespace fake_vel_transform

#endif  // FAKE_VEL_TRANSFORM__RATE_INTEGRATION_HPP_
