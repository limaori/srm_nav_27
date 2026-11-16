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

#ifndef SRM27_MINCO_CORE__TRAJECTORY_2D_HPP_
#define SRM27_MINCO_CORE__TRAJECTORY_2D_HPP_

#include <Eigen/Core>
#include <array>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "srm27_minco_core/types.hpp"

namespace srm27_minco_core
{

/// \brief 轨迹上的净空采样点，用于通道宽度判定与朝向策略。
struct ClearanceSample
{
  /// \brief 相对轨迹起点的时间（s）。
  double time{0.0};
  /// \brief `odom` 系位置（m）。
  Eigen::Vector2d position{Eigen::Vector2d::Zero()};
  /// \brief 该点处的保守净空（m），自由区为正。
  double clearance{0.0};
};

/// \brief 带时间信息的二维连续轨迹（分段五次多项式）。
///
/// 表示约定（方案 §5.3）：
///  * 第 i 段在局部时间 `tau in [0, T_i]` 上定义：
///    `p(tau) = c0 + c1*tau + c2*tau^2 + c3*tau^3 + c4*tau^4 + c5*tau^5`。
///  * **对外系数顺序固定为 c0 -> c5**（`xCoefficients()[i][k]` 即 c_k）。
///    上游 GCOPTER `Trajectory<5>` 的 `CoefficientMat` 存的是高阶在前，
///    适配器（`fromGcopterCoeffs`）显式翻转，并有往返测试覆盖。
///  * 位置、速度、加速度、jerk 都从同一条多项式解析求值；采样点只用于显示。
class Trajectory2D
{
public:
  /// \brief 多项式阶数（五次）。
  static constexpr int kPolynomialOrder = 5;
  /// \brief 每段系数个数（c0..c5）。
  static constexpr int kCoefficientCount = kPolynomialOrder + 1;

  /// \brief 单段系数：索引 k 对应 tau^k。
  using Coefficients = std::array<double, kCoefficientCount>;

  Trajectory2D() = default;

  /// \brief 清空轨迹（保留版本等元数据之外的几何内容）。
  void clear();

  /// \brief 预留段数。
  void reserve(std::size_t _pieces)
  {
    x_coefficients_.reserve(_pieces);
    y_coefficients_.reserve(_pieces);
    durations_.reserve(_pieces);
  }

  /// \brief 追加一段；`_duration` 必须为正且有限。
  /// \return 参数非法时返回 false，轨迹不变。
  bool addPiece(const Coefficients & _x, const Coefficients & _y, double _duration);

  /// \brief 按“高阶在前”的顺序追加一段（GCOPTER `CoefficientMat` 行序）。
  /// \param _x_high_first `[c5, c4, c3, c2, c1, c0]`。
  bool addPieceHighOrderFirst(
    const Coefficients & _x_high_first, const Coefficients & _y_high_first, double _duration);

  /// \brief 是否为空。
  bool empty() const { return durations_.empty(); }
  /// \brief 段数。
  int pieceCount() const { return static_cast<int>(durations_.size()); }
  /// \brief 轨迹总时长（s）。
  double totalDuration() const { return total_duration_; }
  /// \brief 各段时长（s）。
  const std::vector<double> & durations() const { return durations_; }
  /// \brief x 方向系数（c0..c5）。
  const std::vector<Coefficients> & xCoefficients() const { return x_coefficients_; }
  /// \brief y 方向系数（c0..c5）。
  const std::vector<Coefficients> & yCoefficients() const { return y_coefficients_; }

  /// \brief 按给定段时长重建时间轴（用于从“路标点 + 时长”重建轨迹）。
  /// \return 时长数量与段数不一致或存在非正时长时返回 false。
  bool setDurations(const std::vector<double> & _durations);

  /// \brief 把绝对轨迹时间裁剪到 [0, totalDuration]。
  double clampTime(double _t) const;

  /// \brief 定位 `_t` 所在段，并返回段内局部时间。
  /// \param _t 绝对时间（s），自动裁剪到有效范围。
  /// \param _piece_index 输出段索引。
  /// \param _tau 输出段内局部时间（s）。
  void locate(double _t, int & _piece_index, double & _tau) const;

  /// \brief 段内求值：`_tau` 会被裁剪到 [0, T_i]。
  Eigen::Vector2d piecePosition(int _piece_index, double _tau) const;
  Eigen::Vector2d pieceVelocity(int _piece_index, double _tau) const;
  Eigen::Vector2d pieceAcceleration(int _piece_index, double _tau) const;
  Eigen::Vector2d pieceJerk(int _piece_index, double _tau) const;

  /// \brief `odom` 系位置（m）。
  Eigen::Vector2d positionAt(double _t) const;
  /// \brief `odom` 系速度（m/s）。
  Eigen::Vector2d velocityAt(double _t) const;
  /// \brief `odom` 系加速度（m/s^2）。
  Eigen::Vector2d accelerationAt(double _t) const;
  /// \brief `odom` 系 jerk（m/s^3）。
  Eigen::Vector2d jerkAt(double _t) const;

  /// \brief 起点状态。
  Eigen::Vector2d startPosition() const { return positionAt(0.0); }
  /// \brief 终点位置。
  Eigen::Vector2d endPosition() const { return positionAt(total_duration_); }
  /// \brief 起点速度。
  Eigen::Vector2d startVelocity() const { return velocityAt(0.0); }
  /// \brief 终点速度。
  Eigen::Vector2d endVelocity() const { return velocityAt(total_duration_); }
  /// \brief 起点加速度。
  Eigen::Vector2d startAcceleration() const { return accelerationAt(0.0); }
  /// \brief 终点加速度。
  Eigen::Vector2d endAcceleration() const { return accelerationAt(total_duration_); }

  /// \brief 以固定时间间隔采样位置。
  std::vector<Eigen::Vector2d> samplePositions(double _dt) const;

  /// \brief 轨迹切线方向（单位向量）；速度接近零时返回 false 且不写入输出。
  bool tangentAt(double _t, Eigen::Vector2d & _tangent) const;

  /// \brief 系数、时长与连续性的基本自检。
  /// \param _reason 失败原因（可为 nullptr）。
  bool sanityCheck(std::string * _reason) const;

  /// \brief 段间位置/速度/加速度最大偏差（m、m/s、m/s^2），用于诊断拼接质量。
  void continuityResiduals(double & _max_pos, double & _max_vel, double & _max_acc) const;

  /// \brief 轨迹所属版本集合。
  VersionSet versions{};
  /// \brief 生成时刻（ROS 时间秒）。
  double generated_stamp{0.0};
  /// \brief 该轨迹允许被执行的起始时间（ROS 时间秒），用于“未来切换时刻”。
  double valid_after{0.0};
  /// \brief 该轨迹的有效期上限（ROS 时间秒）。
  double valid_until{0.0};
  /// \brief 终点是否为最终导航目标（否则只是局部终点）。
  bool terminal_is_global_goal{false};
  /// \brief Map/obstacle boundaries also require stopping, without becoming navigation goals.
  bool terminal_requires_stop{false};
  std::string terminal_reason{"unspecified"};
  /// \brief 新轨迹切换/拼接时保留的旧轨迹前缀时长（s）。
  double committed_prefix_duration{0.0};
  /// \brief 净空采样。
  std::vector<ClearanceSample> clearance_samples{};
  /// \brief 判定为窄通道的区间 `[起点时间, 终点时间]`（s）。
  std::vector<std::pair<double, double>> narrow_intervals{};

private:
  std::vector<Coefficients> x_coefficients_;
  std::vector<Coefficients> y_coefficients_;
  std::vector<double> durations_;
  /// \brief 各段累计起始时间，长度 = 段数 + 1，`cumulative_time_[0] == 0`。
  std::vector<double> cumulative_time_{0.0};
  double total_duration_{0.0};
};

}  // namespace srm27_minco_core

#endif  // SRM27_MINCO_CORE__TRAJECTORY_2D_HPP_
