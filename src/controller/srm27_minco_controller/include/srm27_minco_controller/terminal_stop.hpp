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

/// \brief 终点急停：一进入目标检查器的成功区域，就**立即输出零速**，不再追踪末点。
///
/// 为什么需要它（2026-10-09 实车终点振荡，`log_diag/real_goal_20261009_140024/`）：
///
///  * MPC 的代价里一直带着"到路径末点的位置误差"（`mpc.q_position`）。即使车已经进入
///    `StoppedGoalChecker` 的 xy 容差，控制器也会继续把车往**精确末点**上修；
///  * 一旦冲过末点就反向修正，再冲、再修正 —— 现场表现就是终点附近来回走；
///  * 而"目标检查器允许在容差内判成功"与"控制器仍要求精确末点"这两件事本来是矛盾的。
///
/// 本类用最简单的方式把矛盾消掉：**进容差就停，不再有任何后段控制逻辑**。
///
///  1. **进入条件只看位置**。轨迹是停在末点上的停车剖面，车进入容差圈时的速度本来就是
///     `sqrt(2*a*tol)`（实车约 1.5 m/s，几乎满速）；任何"先慢下来再接管"的门槛都等于永不接管。
///  2. **接管后立即给零**，并按给定减速度把命令收掉由底盘自己完成（急刹）。
///     好处是**控制器再也不可能主动把车带出容差**：它不再输出任何朝末点或背向末点的速度。
///  3. **不追踪、不修正**：接管期间既不追末点，也不为"冲过头"做反向修正。
///     这正是振荡的直接来源，去掉它之后振荡在逻辑上不可能出现。
///
/// 代价与边界（必须如实记录，不能当成"没有代价"）：
///  * 急刹的停车距离完全由**底盘自己**决定（电机反接/摩擦/打滑），控制器不再限速；
///    所以"停在哪里"取决于底盘的减速能力，取决于 §5 的制动辨识。
///  * 车在距末点 `tol` 处给零，之后滑行 `v²/(2*a_chassis)`。因为容差对末点是**对称的**，
///    只要 `v²/(2*a_chassis) <= 2·tol`，停车点就仍落在末点 ±`tol` 内 —— 仍然判成功。
///    以实车 `v_max=1.5`、`tol=0.40` 代入，条件是 `a_chassis >= 1.41 m/s²`。
///  * 想减小急刹的冲击，正确做法是**把 `tolerance` 调小**（进入越晚、速度越低），
///    而不是重新引入减速曲线：`tol=0.15` 时进入速度约 0.95 m/s，而不是 1.5 m/s。
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
    /// \brief 退出滞回（m）：距离超过 entry + exit_margin 才解除急停。
    double exit_margin{0.15};
  };

  void configure(const Config & _config);
  void reset();

  /// \brief 用当前位姿到路径末点的距离更新急停判定。
  ///
  /// \param _distance 当前位姿到路径末点的距离（m）；非有限值表示无法判定。
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
