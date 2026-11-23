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

#ifndef SRM27_MINCO_CONTROLLER__TERMINAL_STOP_HPP_
#define SRM27_MINCO_CONTROLLER__TERMINAL_STOP_HPP_

namespace srm27_minco_controller
{

/// \brief 在目标成功区域内锁定零平移命令，避免 MPC 反复追踪精确末点。
///
/// 使用与 Nav2 目标检查器一致的实测位姿；MPC 的延迟外推状态不能用于提前锁停。
/// 进入只检查距离，接管即输出零速。停车后的滑行仍取决于执行端制动能力。
/// 滞回必须限制在目标检查器的 xy 成功圈内：漂移/滑行到圈外时重新跟踪，
/// 不能在“检查器拒绝到达、控制器又永久给零”的状态里死锁。
class TerminalStop
{
public:
  struct Config
  {
    /// \brief 是否启用终点急停。
    bool enabled{true};
    /// \brief 固定进入阈值（m）。<=0 表示使用目标检查器给出的 xy 容差。
    double tolerance{0.0};
    /// \brief 目标检查器容差不可用时的兜底进入阈值（m）。
    double fallback_tolerance{0.20};
    /// \brief 退出滞回（m），退出半径不得超过目标检查器的 xy 成功半径。
    double exit_margin{0.15};
  };

  void configure(const Config & _config);
  void reset();

  /// \brief 用当前位姿到路径末点的距离更新急停判定。
  ///
  /// \param _distance Nav2 实测位姿到路径末点的距离（m），不使用 MPC 外推位姿。
  /// \param _goal_tolerance 目标检查器的 xy 容差（m）；<=0 或非有限表示不可用。
  /// \return 是否处于急停状态（true 时调用方必须输出零速，不得再跟踪轨迹）。
  bool update(double _distance, double _goal_tolerance);

  bool active() const { return active_; }
  /// \brief 本次生效的进入阈值（m），供日志与诊断使用。
  double entryTolerance() const { return entry_tolerance_; }
  /// \brief 本次生效的退出阈值（m）。
  double exitTolerance() const { return exit_tolerance_; }

private:
  Config config_{};
  bool active_{false};
  double entry_tolerance_{0.0};
  double exit_tolerance_{0.0};
};

}  // namespace srm27_minco_controller

#endif  // SRM27_MINCO_CONTROLLER__TERMINAL_STOP_HPP_
