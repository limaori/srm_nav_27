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

#ifndef SRM27_MINCO_CORE__TRAJECTORY_INITIALIZER_HPP_
#define SRM27_MINCO_CORE__TRAJECTORY_INITIALIZER_HPP_

#include <Eigen/Core>
#include <string>
#include <vector>

#include "srm27_minco_core/trajectory_2d.hpp"
#include "srm27_minco_core/types.hpp"

namespace srm27_minco_core
{

/// \brief 前端初值：路标点与段时长。
struct TrajectoryInitialGuess
{
  /// \brief 路标点（`odom` 系），第 0 个是起点，最后一个是局部终点。
  std::vector<Eigen::Vector2d> waypoints;
  /// \brief 各段时长（s），长度 = waypoints.size() - 1。
  std::vector<double> durations;
  /// \brief 各段允许的最高速度（m/s），长度 = durations.size()。
  ///
  /// 对应 navi_minco_bit `MincoPlanner::PTAllocation()` 传给优化器的 `local_vmaxs`：
  /// 转角越尖，该段上限越低。为空表示不额外限制（只用场景速度上限）。
  /// 优化器把它当作该段的速度软约束阈值，否则“转角降速”只存在于初值里，
  /// 最优点会把速度重新改回全局上限 —— 这也是旧实现 `corner_speed_ratio` 失效的原因之一。
  std::vector<double> speed_limits;
  /// \brief 本次折线上用弦长窗口量到的最大转向角（rad）。
  ///
  /// 诊断用：把它和 `turn_angle_deadzone` / `turn_angle_saturation` 一起看，就能判断
  /// “转角降速到底有没有机会生效”。密集平滑路径上这个角度通常远小于稀疏折线的顶点角。
  double corner_angle_max{0.0};
  /// \brief 起点状态（p/v/a）。
  Eigen::Vector2d head_position{Eigen::Vector2d::Zero()};
  Eigen::Vector2d head_velocity{Eigen::Vector2d::Zero()};
  Eigen::Vector2d head_acceleration{Eigen::Vector2d::Zero()};
  /// \brief 终点状态（p/v/a）。
  Eigen::Vector2d tail_position{Eigen::Vector2d::Zero()};
  Eigen::Vector2d tail_velocity{Eigen::Vector2d::Zero()};
  Eigen::Vector2d tail_acceleration{Eigen::Vector2d::Zero()};

  /// \brief 段数。
  int pieceCount() const { return static_cast<int>(durations.size()); }
};

/// \brief 由折线路径生成 MINCO 初值（方案 §6.3）。
///
/// 步骤：去除重复点/短小折返 -> 弧长重采样 -> 转角降速 -> 前向加速扫描/后向制动扫描 ->
/// 由速度剖面积分时长 -> 按近似等时间间隔取路标点并限制最小/最大段时长。
///
/// 本类不做碰撞检查，也不输出可执行轨迹；它只产生“形状 + 时间”初值，供 MINCO 优化。
class TrajectoryInitializer
{
public:
  /// \brief 配置。
  struct Config
  {
    /// \brief 场景速度上限（m/s）。
    double max_speed{0.5};
    /// \brief 场景加速度上限（m/s^2）。
    double max_accel{0.3};
    /// \brief 制动减速度上限（m/s^2）；必须来自实测可保证的制动能力。
    double max_brake{0.3};
    /// \brief 弧长重采样步长（m）。
    double resample_step{0.10};
    /// \brief 相邻输入点距离小于该值时视为重复点并被丢弃（m）。
    double duplicate_epsilon{1.0e-3};
    /// \brief 段时长下限（s）。
    double min_piece_duration{0.05};
    /// \brief 段时长上限（s）。
    double max_piece_duration{1.0};
    /// \brief 名义段时长（s），用于决定路标点数量。
    double nominal_piece_duration{0.30};
    /// \brief 转角处允许的最低速度比例（0..1]。
    ///
    /// \deprecated 旧实现的“按逐点方向变化线性插值降速”。因为重采样步长只有 0.1 m，
    /// 平滑后的圆角每点方向变化约 1°，`blend` 常年贴近 0，参数实际上不生效。
    /// 现在改用下面的 `turn_angle_*` / `min_turn_vel` / `decay_power`（与
    /// navi_minco_bit 的 `calCurvatureDecay` 同构），本字段只为兼容旧配置保留，不再读取。
    double corner_speed_ratio{0.35};
    /// \brief 旧实现的转角判定阈值（rad），同样不再读取（见 `corner_speed_ratio`）。
    double corner_angle_threshold{0.35};
    /// \brief 转角降速的角度死区（rad）：小于该角度视为直道，不降速。
    double turn_angle_deadzone{0.10};
    /// \brief 转角降速的角度饱和值（rad）：超过该角度按饱和值计算（角度再大速度也不再降）。
    double turn_angle_saturation{0.80};
    /// \brief 最尖转角处的速度地板（m/s）。等于 `max_speed` 表示关闭转角降速。
    ///
    /// 注意这是**绝对速度**（与 navi_minco_bit 的 `min_turn_vel` 语义一致），必须按本车
    /// 速度上限等比换算：参考工程 max_vel=3.0 / min_turn_vel=0.7（23%），
    /// 本车 max_linear_speed=0.5 时对应 0.12 m/s 左右。
    double min_turn_vel{0.12};
    /// \brief 转角降速的指数衰减系数（无量纲），越大越早降到地板速度。
    double decay_power{1.6};
    /// \brief 转角速度地板占场景速度上限的比例（0~1]；> 0 时覆盖 `min_turn_vel`。
    ///
    /// 这是本项目相对 navi_minco_bit 的一处**扩展**：参考工程的 `min_turn_vel` 是绝对速度
    /// （0.7 m/s / max_vel 3.0 = 23%），而本仓库会在 0.5~2.6 m/s 之间切换速度档
    /// （`speed_limits.yaml`）。用绝对值时换档就会失效（例如 2.6 m/s 档下 0.12 m/s 等于让车
    /// 在弯里停住）。取比例后与速度档解耦，`0.23` 即参考工程的等效值。
    double min_turn_vel_ratio{0.0};
    /// \brief 估计转角用的弦长窗口（m）。
    ///
    /// 参考工程直接在**稀疏**路径的顶点上量夹角（那里的顶点就是真转角）。本项目输入是
    /// 已平滑的密集路径，圆角被展成多段小角度，逐点夹角接近 0。这里用“前后各半窗的弦”
    /// 的夹角替代顶点夹角：窗口越大越接近真实转角，越小越接近局部曲率。
    double corner_chord_window{0.40};
    /// \brief 局部终点是否等于全局目标；为真时末端速度为 0。
    bool terminal_is_global_goal{true};
    /// \brief 末端不是全局目标时保留的末端速度（m/s）。
    double terminal_speed{0.0};
    /// \brief 最小可行路标点数（段数不少于该值）。
    int min_pieces{2};
    /// \brief 最大路标点段数，避免长走廊产生过多变量。
    int max_pieces{12};
  };

  /// \brief 由折线生成初值。
  /// \param _polyline 输入折线（`odom` 系，m），至少两个点。
  /// \param _head_velocity 当前估计的起点速度（`odom` 系，m/s）。
  /// \param _config 配置。
  /// \param _guess 输出初值。
  /// \param _reason 失败原因（可为 nullptr）。
  /// \return 成功返回 true。
  static bool initialize(
    const std::vector<Eigen::Vector2d> & _polyline, const Eigen::Vector2d & _head_velocity,
    const Config & _config, TrajectoryInitialGuess & _guess, std::string * _reason);

  /// \brief 只做折线清理与弧长重采样（供单元测试与调试使用）。
  /// \return 清洗后的折线；输入非法时返回空。
  static std::vector<Eigen::Vector2d> cleanAndResample(
    const std::vector<Eigen::Vector2d> & _polyline, double _duplicate_epsilon, double _step);

  /// \brief 由路标点与时长构造一段段五次多项式（MINCO 条件），用于“形状预优化”前的初始轨迹。
  ///
  /// 该轨迹只作为初值，**不得直接执行**（可能碰撞、可能超动力学约束）。
  /// \return 构造失败返回 false。
  static bool buildInitialTrajectory(
    const TrajectoryInitialGuess & _guess, Trajectory2D & _trajectory);
};

}  // namespace srm27_minco_core

#endif  // SRM27_MINCO_CORE__TRAJECTORY_INITIALIZER_HPP_
