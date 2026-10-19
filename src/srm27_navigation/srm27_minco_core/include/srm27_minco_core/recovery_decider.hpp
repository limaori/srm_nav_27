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

#ifndef SRM27_MINCO_CORE__RECOVERY_DECIDER_HPP_
#define SRM27_MINCO_CORE__RECOVERY_DECIDER_HPP_

#include <Eigen/Core>
#include <cstdint>
#include <string>
#include <vector>

#include "srm27_minco_core/esdf_2d.hpp"

namespace srm27_minco_core
{

/// \brief 一次脱困决策的结果。
enum class RecoveryDecision {
  /// \brief 什么都没发生：门限未到、正处于脱困会话中，或输入非法。
  kNone = 0,
  /// \brief 进入脱困：`_escape_velocity` 已填入世界坐标系下的逃逸速度（m/s）。
  kEscape,
  /// \brief 无法算出可用的逃逸方向（距离场无效 / 全部采样非有限）。
  ///
  /// 调用方**不得**据此进入脱困状态：没有方向的脱困等于原地不动，
  /// 会把一次可恢复的卡死变成永久卡死。保留目标、继续重试是唯一安全的处置。
  kNoEscapeDirection,
};

/// \brief 脱困决策的转出原因（写出到诊断，不影响行为）。
enum class RecoveryExit {
  /// \brief 未退出。
  kNone = 0,
  /// \brief 净空恢复到 `escaped_clearance` 以上，判为脱困成功。
  kEscaped,
  /// \brief 超过 `window_sec` 仍未恢复净空，判为脱困超时。
  kTimeout,
};

/// \brief 脱困决策的可调参数。
///
/// 默认值与 `navi_minco_bit/src/navigation/minco_planner` 的
/// `MincoPlanner.recovery_server.*` 默认值一致，便于逐项对照迁移前的行为。
struct RecoveryConfig
{
  /// \brief 连续规划失败多少次才允许进入脱困。
  int fail_threshold{3};
  /// \brief 两次脱困之间的最小间隔（s）。
  double cooldown_sec{2.0};
  /// \brief 一次脱困会话的最长持续时间（s）；窗口内没恢复净空即判超时。
  double window_sec{1.5};
  /// \brief 逃逸速度（m/s）。
  double escape_speed{0.4};

  /// \brief 卡死分诊阈值（m）：自身净空低于该值才算"被卡住"。
  ///
  /// 高于该值的规划失败按"前方路障"处理（直接绕路），不计入失败计数、不触发脱困。
  /// 这是脱困语义上的关键分水岭。
  double stuck_clearance{0.25};
  /// \brief 脱困成功判据（m）：自身净空恢复到该值以上即判成功。
  ///
  /// 必须大于 `stuck_clearance`，否则会在刚进入脱困时立刻判成功、
  /// 出现"进入-退出"高频振荡。
  double escaped_clearance{0.40};

  /// \brief 梯度退化时的环形搜索半径（m）。
  ///
  /// 应略大于机器人的膨胀半径：太小会搜到自身所在栅格、太大则方向分辨率不足。
  double search_radius{0.4};
  /// \brief 环形搜索的采样数量（每 360/num_samples 度一个采样点）。
  int search_samples{36};
  /// \brief 梯度范数低于该值时判为"梯度平台"，改用环形搜索。
  double gradient_epsilon{1e-3};
};

/// \brief 脱困判定 / 脱困方向规划 / 脱困成功判定。
///
/// 从 `navi_minco_bit` 的 `RecoverServer`（`recovery_behaivor.{hpp,cpp}`）迁移而来，
/// 保持原有的**判定语义**与默认阈值，但去掉了对 `geometry_msgs` 与 ROS 的依赖：
/// 位姿换成裸 `(x, y)`，距离场换成 `Esdf2D` 的解析查询（参考实现用 4 次有限差分近似
/// 梯度，这里直接取双线性插值的解析梯度，数值更准且只查 1 次）。
///
/// 三类职责：
///  1. **脱困判定**：连续失败计数 + 阈值 + 冷却 + 卡死分诊（`decide()`）。
///  2. **脱困方向规划**：ESDF 梯度上升；梯度退化时退化为环形搜索取最开阔方向。
///  3. **脱困成功判定**：脱困期间净空恢复到 `escaped_clearance` 以上即成功，
///     超过 `window_sec` 未恢复即超时（`updateRecovery()`）。
///
/// 本类**不做任何 I/O**：日志写 stderr，速度由调用方执行（控制器返回 Twist，
/// 或行为树的脱困行为发布 cmd_vel）。
class RecoveryDecider
{
public:
  RecoveryDecider() = default;

  /// \brief 设置参数；非法值被夹到安全默认值而不是抛异常（与参考实现一致）。
  void configure(const RecoveryConfig & _config);
  const RecoveryConfig & config() const { return config_; }

  /// \brief 清空全部运行期状态（新目标、取消、失活、定位重置）。
  void reset();

  // === 1) 脱困判定 ===

  /// \brief 汇报一次规划失败。
  /// \return 计数已累计并跨过门限时返回 true（此后 `inRecovery()` 为真）。
  bool onPlanningFailure(double _now_sec);

  /// \brief 汇报一次规划成功；清零失败计数。
  void onPlanningSuccess();

  /// \brief 判断此刻是否允许进入脱困（不含卡死分诊）。
  bool shouldStartRecovery(double _now_sec) const;

  /// \brief 脱困门限判定 + 脱困方向规划，一次调用完成。
  ///
  /// \param _now_sec 当前 ROS 时间（s）；非有限或为负时直接返回 `kNone`。
  /// \param _position 机器人当前位置（规划坐标系，m）。
  /// \param _esdf 距离场；空或无效时返回 `kNoEscapeDirection`。
  /// \param _escape_velocity 输出：世界坐标系下的逃逸速度（m/s），仅在 `kEscape` 时有效。
  RecoveryDecision decide(
    double _now_sec, const Eigen::Vector2d & _position, const Esdf2D & _esdf,
    Eigen::Vector2d * _escape_velocity);

  /// \brief 规划坐标系下的逃逸方向（单位向量），仅在 `kEscape` 后可读。
  const Eigen::Vector2d & escapeDirection() const { return last_escape_direction_; }

  /// \brief 沿脱困方向场前推出一条**脱困路径**（供 RViz 用绿线展示）。
  ///
  /// 与"逃逸方向"不是一回事：方向只是当前这一周期该往哪走，而路径要回答
  /// "如果一直按每周期重算的方向走，车会走出什么轨迹"。因为方向随位置重算
  /// （见 `updateRecovery`），这条路**通常不是直线** —— 这正是现场最需要看到的东西：
  /// 直线撞墙而折线绕开，说明环形搜索/梯度在起作用；两条都是直线撞墙，说明
  /// 距离场里确实没有出口。
  ///
  /// 实现就是**沿方向场欧拉积分**：每步取当前点的 `planEscapeDirection`，沿该方向
  /// 前进 `_step`，直到净空超过 `escaped_clearance`（判为走出卡死区）、离开地图、
  /// 或达到步数上限。
  ///
  /// \param _esdf 距离场。
  /// \param _start 起点（通常是机器人当前位置，规划坐标系，m）。
  /// \param _config 参数；步长取 `min(_step, escape_speed * window_sec)` 的语义由调用方控制。
  /// \param _step 积分步长（m），非正时取 0.1 m。
  /// \param _max_steps 步数上限，非正时取 64。
  /// \param _path 输出：至少含起点。无有效方向时返回只含起点的路径。
  /// \return 路径是否至少走出了第一步（即"是否有可执行的脱困路径"）。
  static bool traceEscapePath(
    const Esdf2D & _esdf, const Eigen::Vector2d & _start, const RecoveryConfig & _config,
    double _step, int _max_steps, std::vector<Eigen::Vector2d> * _path);

  // === 3) 脱困成功判定 ===

  /// \brief 脱困会话是否仍在时间窗口内。
  bool inRecovery(double _now_sec) const;

  /// \brief 是否处于脱困会话中（不看窗口）。
  bool isRecovering() const { return recovery_active_; }

  /// \brief 结束脱困会话并记录结束时刻（冷却参考点）。
  /// \param _success 成功时清零失败计数。
  /// \param _now_sec 当前 ROS 时间（s）；非法时只清标志、不打冷却点。
  void finishRecovery(bool _success, double _now_sec);

  /// \brief 脱困期间每周期调用：判成功 / 判超时，并**按当前位置重算**逃逸方向。
  ///
  /// **必须每周期重算方向**，不能沿用进入脱困时的那一个：参考实现的 FSM 在
  /// `RECOVERING` 里每次 tick 都调用 `calculateEscapeVelocity`，方向随车移动而更新。
  /// 方向冻结的后果是"环形搜索选出的那个最开阔方向只能推一步"——车往前挪之后
  /// 那个方向可能已经不再最开阔，而脱困要的恰恰是朝最开阔处持续收敛。
  ///
  /// \param _now_sec 当前 ROS 时间（s）。
  /// \param _position 机器人当前位置（规划坐标系，m）。
  /// \param _esdf 距离场；无效时无法给方向，本周期不输出速度（但窗口继续走）。
  /// \param _clearance 当前自身净空（m）；非有限时视为"未成功"，只按时间推进。
  /// \param _escape_velocity 输出：仍在脱困时填入本周期应执行的逃逸速度。
  /// \param _log_transitions 是否打印方向分支日志（进入脱困那一周期打一次即可）。
  /// \return `kNone` 表示"继续脱困"，否则表示已退出且退出原因即为返回值。
  RecoveryExit updateRecovery(
    double _now_sec, const Eigen::Vector2d & _position, const Esdf2D & _esdf, double _clearance,
    Eigen::Vector2d * _escape_velocity, bool _log_transitions = false);

  // === 状态查询（诊断） ===
  int consecutiveFailures() const { return consecutive_failures_; }
  double lastRecoveryStartSec() const { return last_recovery_start_sec_; }
  double lastRecoveryEndSec() const { return last_recovery_end_sec_; }
  /// \brief 最近一次进入脱困的原因字符串（诊断）。
  const std::string & lastTrigger() const { return last_trigger_; }
  /// \brief 最近一次退出的原因字符串（诊断）。
  const std::string & lastExit() const { return last_exit_; }
  /// \brief 进入脱困时自身净空（m）；未进入过为 +inf。
  double entryClearance() const { return entry_clearance_; }
  /// \brief 进入脱困时是否走了环形搜索分支（诊断：梯度是否退化）。
  bool usedCircularSearch() const { return used_circular_search_; }
  /// \brief 最近一次环形搜索找到的最大净空（m）；未搜过为 -inf。
  double circularSearchBestClearance() const { return circular_best_clearance_; }

  // === 2) 脱困方向规划（纯函数，供行为树脱困行为复用） ===

  /// \brief 由距离场规划脱困方向（单位向量，规划坐标系）。
  ///
  /// 参考实现的三段逻辑：
  ///  1. 距离场不可用 -> 失败；
  ///  2. 梯度范数 >= `gradient_epsilon` -> 沿梯度上升（远离障碍）；
  ///  3. 否则环形搜索 `search_samples` 个方向、半径 `search_radius`，取净空最大者。
  ///
  /// \param _esdf 距离场。
  /// \param _position 机器人当前位置（规划坐标系，m）。
  /// \param _config 参数。
  /// \param _direction 输出：单位方向向量。
  /// \param _best_clearance 输出：方向选取依据（梯度分支填起点净空，环形分支填环形最大值）。
  /// \param _log_transitions 是否打印"梯度退化、转环形搜索"日志。
  /// \return 无可用方向时返回 false。
  static bool planEscapeDirection(
    const Esdf2D & _esdf, const Eigen::Vector2d & _position, const RecoveryConfig & _config,
    Eigen::Vector2d * _direction, double * _best_clearance, bool _log_transitions = true);

private:
  /// \brief 时间是否可用（有限且非负）。
  static bool isTimeValid(double _now_sec);

  RecoveryConfig config_{};

  // 失败与脱困运行期状态
  int consecutive_failures_{0};
  double last_failure_sec_{-1.0};
  double last_recovery_start_sec_{-1.0};
  double last_recovery_end_sec_{-1.0};
  bool recovery_active_{false};

  // 脱困会话缓存
  Eigen::Vector2d escape_velocity_{Eigen::Vector2d::Zero()};
  Eigen::Vector2d last_escape_direction_{Eigen::Vector2d::Zero()};
  double entry_clearance_{-1.0};

  // 诊断
  std::string last_trigger_{"none"};
  std::string last_exit_{"none"};
  bool used_circular_search_{false};
  double circular_best_clearance_{-1.0};
};

/// \brief 枚举转字符串（诊断用）。
const char * toString(RecoveryDecision _decision);
/// \brief 枚举转字符串（诊断用）。
const char * toString(RecoveryExit _exit);

}  // namespace srm27_minco_core

#endif  // SRM27_MINCO_CORE__RECOVERY_DECIDER_HPP_
