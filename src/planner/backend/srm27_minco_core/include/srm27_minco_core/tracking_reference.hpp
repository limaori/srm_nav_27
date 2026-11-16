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

#ifndef SRM27_MINCO_CORE__TRACKING_REFERENCE_HPP_
#define SRM27_MINCO_CORE__TRACKING_REFERENCE_HPP_

#include <Eigen/Core>
#include <string>
#include <vector>

#include "srm27_minco_core/mpc_model.hpp"
#include "srm27_minco_core/trajectory_2d.hpp"
#include "srm27_minco_core/types.hpp"

namespace srm27_minco_core
{

/// \brief 航向参考提供者：把轨迹与策略模式映射为航向/角速度参考。
///
/// core 不认识 Nav2 与自转仲裁，策略由 ROS 适配层实现本接口（方案 §4.5.1）。
class YawReferenceProvider
{
public:
  virtual ~YawReferenceProvider() = default;

  /// \brief 为一个预测时刻给出航向与角速度参考。
  /// \param _trajectory_time 该点在轨迹上的时间（s，绝对进度）。
  /// \param _position 轨迹位置（m，odom 系）。
  /// \param _velocity 轨迹速度（m/s，odom 系）。
  /// \param _tangent_valid 切线是否可用（低速时可能不可用）。
  /// \param _tangent 单位切线方向；`_tangent_valid` 为假时内容无意义。
  /// \param _previous_yaw 上一个采样点的连续展开航向（rad）。
  /// \param _yaw 输出航向参考（rad，连续展开，不做 2*pi 折叠）。
  /// \param _omega 输出角速度参考（rad/s）。
  /// \return 成功返回 true。
  virtual bool yawReference(
    double _trajectory_time, const Eigen::Vector2d & _position, const Eigen::Vector2d & _velocity,
    bool _tangent_valid, const Eigen::Vector2d & _tangent, double _previous_yaw, double & _yaw,
    double & _omega) = 0;
};

/// \brief 跟踪参考构造配置。
struct TrackingReferenceConfig
{
  /// \brief 进度投影向后搜索窗口（s），用于吸收跟踪滞后。
  double projection_backward_window{0.30};
  /// \brief 进度投影向前搜索窗口（s）。
  double projection_forward_window{1.00};
  /// \brief 单个控制周期允许的最大进度前进量（s）。
  double max_progress_step{0.60};
  /// \brief 单个控制周期允许的最大进度回退量（s），用于恢复误差但防止无限倒退。
  double backtrack_limit{0.05};
  /// \brief 接近终点前开始平滑收敛的时长（s）。
  double goal_slowdown_time{0.40};
  /// \brief 是否启用报告式第二次采样（alpha 缩放）。
  bool two_pass_reference{false};

  /// \brief 配置自检。
  bool valid(std::string * _reason = nullptr) const;
};

/// \brief 跟踪参考构造结果。
struct TrackingReferenceResult
{
  /// \brief 是否可用。
  bool valid{false};
  /// \brief 本次使用的投影进度 `tau0`（s）。
  double progress{0.0};
  /// \brief 参考状态，形状 `(nz, N)`。
  Eigen::MatrixXd z_ref{};
  /// \brief 参考输入，形状 `(nu, N)`。
  Eigen::MatrixXd u_ref{};
  /// \brief 逐步切线角（rad），用于各向异性权重。
  std::vector<double> tangent_angle{};
  /// \brief 是否已进入终点收敛区。
  bool near_terminal{false};
  /// \brief 说明信息。
  std::string message{};
};

/// \brief 由 MINCO 轨迹与当前状态构造 MPC 参考（方案 §5.4、§7.3）。
///
/// 关键点：
///  * 使用独立的 `progress`（轨迹进度）而不是墙钟时间；每周期只在上一次进度附近的
///    连续窗口内搜索投影点，跨自交点时不会跳到远处另一段。
///  * 允许有限回退恢复误差，但禁止每次重规划无限倒退。
///  * 第二次采样的缩放量只作用在**参考轨迹的采样增量**上，物理预测步长 h 不变。
///
/// 本类不是线程安全的：一个实例只能由一个控制线程使用。
class TrackingReferenceBuilder
{
public:
  /// \brief 配置。
  bool configure(const TrackingReferenceConfig & _config, std::string * _reason = nullptr);

  /// \brief 清空进度（新目标、取消、轨迹作废时调用）。
  void reset();

  /// \brief 当前轨迹进度（s）。
  double progress() const { return progress_; }

  /// \brief 是否存在有效的上一次进度。
  bool hasProgress() const { return has_progress_; }

  /// \brief 第一次参考构造。
  bool build(
    const Trajectory2D & _trajectory, const State2D & _state, const MpcModel & _model,
    YawReferenceProvider & _yaw_provider, TrackingReferenceResult & _result);

  /// \brief 第二次参考构造：用第一次预测速度做 alpha 缩放重采样。
  /// \param _predicted_velocity 第一次求解的预测平移速度，形状 `(2, N)`。
  bool buildSecondPass(
    const Trajectory2D & _trajectory, const Eigen::MatrixXd & _predicted_velocity,
    const MpcModel & _model, YawReferenceProvider & _yaw_provider,
    TrackingReferenceResult & _result);

  /// \brief 在指定进度附近把状态投影到轨迹上（对外暴露以便单元测试）。
  /// \param _previous_progress 上一次进度（s）。
  /// \param _has_previous 是否存在上一次进度。
  /// \return 投影得到的进度；轨迹为空时返回 0。
  double projectProgress(
    const Trajectory2D & _trajectory, const Eigen::Vector2d & _position, double _previous_progress,
    bool _has_previous) const;

private:
  /// \brief 按采样进度列表组装参考。
  bool assemble(
    const Trajectory2D & _trajectory, const State2D & _state, const MpcModel & _model,
    YawReferenceProvider & _yaw_provider, const std::vector<double> & _sample_times,
    TrackingReferenceResult & _result);

  TrackingReferenceConfig config_{};
  bool configured_{false};
  double progress_{0.0};
  bool has_progress_{false};
  /// \brief 上一次使用的连续航向，保证第二次采样与跨周期一致在同一 2*pi 分支。
  double last_yaw_{0.0};
  bool has_last_yaw_{false};
};

}  // namespace srm27_minco_core

#endif  // SRM27_MINCO_CORE__TRACKING_REFERENCE_HPP_
