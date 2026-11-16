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
    double corner_speed_ratio{0.35};
    /// \brief 判定转角的速度方向变化阈值（rad）。
    double corner_angle_threshold{0.35};
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
