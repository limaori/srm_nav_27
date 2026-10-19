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

#ifndef SRM27_MINCO_CONTROLLER__YAW_POLICY_HPP_
#define SRM27_MINCO_CONTROLLER__YAW_POLICY_HPP_

#include <string>

#include "srm27_minco_core/tracking_reference.hpp"

namespace srm27_minco_controller
{

/// \brief 航向参考模式（方案 §7.3、§8.1、§8.2）。
enum class YawMode {
  /// \brief 阶段一：MPC 只控平移，角速度上下限为 0，自转由独立链路负责。
  kXyOnly = 0,
  /// \brief 跟踪轨迹切线（开阔区前进时保持车头朝前）。
  kFollowTangent = 1,
  /// \brief 按外部请求的期望角速度连续积分航向（窄通道自转/到点自转）。
  kSpin = 2,
  /// \brief 窄通道内保持接近当前姿态的正/反切向。
  kNarrowTrack = 3,
};

/// \brief 航向策略：把轨迹与模式映射为 MPC 的航向/角速度参考。
///
/// 本类只做参考生成，不直接发布速度，也不与独立自转链路叠加角速度（方案 §8.1）。
class YawPolicy : public srm27_minco_core::YawReferenceProvider
{
public:
  /// \brief 配置。
  struct Config
  {
    /// \brief 模式。
    YawMode mode{YawMode::kXyOnly};
    /// \brief 到达目标后是否停止自转参考（避免长期 SPIN 使停止条件永不满足）。
    bool stop_rotation_on_goal{true};
    /// \brief 有效角速度上限（rad/s）；`kXyOnly` 下强制为 0。
    double max_angular_speed{0.3};
    /// \brief 有效角加速度上限（rad/s^2）。
    double max_angular_accel{0.5};
    /// \brief 窄通道判定的净空阈值（m），来自地图分辨率与真实外形评估。
    double narrow_clearance_threshold{0.60};
    /// \brief 是否允许倒向行驶（负向切向）；全向底盘默认为真。
    bool allow_reverse{true};
  };

  YawPolicy() = default;

  /// \brief 设置配置。
  bool configure(const Config & _config, std::string * _reason = nullptr);

  /// \brief 复位：清空连续航向与跟踪状态。
  void reset(double _yaw);

  /// \brief 每个控制周期把当前连续航向同步进来（`kXyOnly` 用它作为恒定参考）。
  void setCurrentYaw(double _yaw) { current_yaw_ = _yaw; }

  /// \brief 设置外部自转请求（`kSpin` 使用）。
  ///
  /// 调用方必须每个控制周期在请求仍然新鲜时调用一次；请求过期、导航取消或失活时
  /// 调用 `clearSpinRequest()`。这样不存在“请求超时后仍沿用旧自转”的路径。
  /// \param _omega 期望角速度（rad/s）。
  void setSpinRequest(double _omega);

  /// \brief 撤销自转请求（导航取消/失活/请求过期）。
  void clearSpinRequest();

  /// \brief 当前是否有有效自转请求。
  bool spinRequestActive() const { return spin_request_active_; }

  /// \brief 更新窄通道判定结果（由控制器根据轨迹净空与通道几何给出）。
  void setNarrowTrackActive(bool _active) { narrow_track_active_ = _active; }

  /// \brief 当前是否处于窄通道跟踪模式。
  bool narrowTrackActive() const { return narrow_track_active_; }

  /// \brief MPC 可用的角速度上限（`kXyOnly` 为 0，即 MPC 不许转动）。
  double angularSpeedLimit() const;

  /// \brief MPC 可用的角加速度上限。
  double angularAccelLimit() const;

  /// \brief 当前模式。
  YawMode mode() const { return config_.mode; }

  /// \brief 模式名（诊断用）。
  const char * modeName() const;

  /// \brief 实现 `srm27_minco_core::YawReferenceProvider`。
  bool yawReference(
    double _trajectory_time, const Eigen::Vector2d & _position, const Eigen::Vector2d & _velocity,
    bool _tangent_valid, const Eigen::Vector2d & _tangent, double _previous_yaw, double & _yaw,
    double & _omega) override;

private:
  Config config_{};
  double current_yaw_{0.0};
  double spin_omega_{0.0};
  bool spin_request_active_{false};
  bool narrow_track_active_{false};
  /// \brief 上一次 FOLLOW/NARROW 模式给出的切向参考，用于估计角速度参考。
  double last_tangent_time_{0.0};
  double last_tangent_yaw_{0.0};
  bool has_last_tangent_{false};
  bool configured_{false};
};

/// \brief 由字符串解析航向模式；无法识别时返回 false。
bool parseYawMode(const std::string & _text, YawMode & _mode);

/// \brief 航向模式转字符串（参数回读用）。
const char * toString(YawMode _mode);

}  // namespace srm27_minco_controller

#endif  // SRM27_MINCO_CONTROLLER__YAW_POLICY_HPP_
