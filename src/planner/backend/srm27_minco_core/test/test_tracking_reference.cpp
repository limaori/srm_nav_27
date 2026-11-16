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

/// \file
/// \brief 跟踪参考构造（进度投影、限幅、终点收敛、第二次采样 alpha）的单元测试。
///
/// 覆盖方案 §5.4（轨迹进度与投影）与 §7.3（参考构造、终点收敛、two-pass 采样）：
///  * 进度用独立变量表达，只在窗口内搜索投影，自交/回折时不跳到远处分支；
///  * 单周期进度变化受 `max_progress_step` 与 `backtrack_limit` 限制；
///  * 终点前速度参考按剩余时间收敛到零；
///  * 第二次采样的 alpha 只缩放参考采样增量，物理步长不变；
///  * 航向参考连续展开，不出现 2*pi 跳变。

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>

#include "srm27_minco_core/mpc_model.hpp"
#include "srm27_minco_core/tracking_reference.hpp"
#include "srm27_minco_core/trajectory_2d.hpp"
#include "srm27_minco_core/types.hpp"

namespace
{

/// \brief 圆周率常量，避免依赖未标准化的 M_PI 宏。
const double kPi = std::acos(-1.0);

/// \brief 预测步数（与产品默认一致）。
const int kSteps = 30;

/// \brief 速度层 MPC 模型配置：`z = [x, y, yaw]`，`u = [vx, vy, omega]`。
srm27_minco_core::MpcModel::Config trackingModelConfig()
{
  srm27_minco_core::MpcModel::Config config;
  config.type = srm27_minco_core::MpcModelType::kVelocityIntegrator;
  config.prediction_steps = kSteps;
  config.prediction_dt = 0.02;
  return config;
}

/// \brief 默认跟踪参考配置（窗口 0.30/1.00 s、前进 0.60 s、回退 0.05 s）。
srm27_minco_core::TrackingReferenceConfig trackingConfig()
{
  srm27_minco_core::TrackingReferenceConfig config;
  config.projection_backward_window = 0.30;
  config.projection_forward_window = 1.00;
  config.max_progress_step = 0.60;
  config.backtrack_limit = 0.05;
  config.goal_slowdown_time = 0.40;
  config.two_pass_reference = true;
  return config;
}

/// \brief 构造沿 `+x` 的常速直线轨迹。
/// \param _pieces 段数。
/// \param _seconds_per_piece 每段时长（s）。
/// \param _speed 常速（m/s）。
/// \return 从 (0,0) 出发、总长 `_pieces*_seconds_per_piece*_speed` 的轨迹。
srm27_minco_core::Trajectory2D makeStraightTrajectory(
  const int _pieces, const double _seconds_per_piece, const double _speed)
{
  srm27_minco_core::Trajectory2D trajectory;
  double x = 0.0;
  for (int i = 0; i < _pieces; ++i) {
    srm27_minco_core::Trajectory2D::Coefficients cx{};
    srm27_minco_core::Trajectory2D::Coefficients cy{};
    cx[0] = x;
    cx[1] = _speed;
    EXPECT_TRUE(trajectory.addPiece(cx, cy, _seconds_per_piece));
    x += _speed * _seconds_per_piece;
  }
  return trajectory;
}

/// \brief 构造“去程 + 回程”的回折轨迹：去程沿 `+x` 到 `_length`，回程沿 `-x` 返回。
///
/// 回程在 y 上偏移 `_offset`，因此两条腿在 y 方向很近但轨迹进度相差很远，
/// 用于验证投影不会从近处分支跳到远处分支。
srm27_minco_core::Trajectory2D makeHairpinTrajectory(
  const double _length, const double _speed, const double _offset)
{
  srm27_minco_core::Trajectory2D trajectory;
  const double duration = _length / _speed;

  srm27_minco_core::Trajectory2D::Coefficients cx{};
  srm27_minco_core::Trajectory2D::Coefficients cy{};
  cx[0] = 0.0;
  cx[1] = _speed;
  EXPECT_TRUE(trajectory.addPiece(cx, cy, duration));

  cx[0] = _length;
  cx[1] = -_speed;
  cy[0] = _offset;
  EXPECT_TRUE(trajectory.addPiece(cx, cy, duration));
  return trajectory;
}

/// \brief 构造切线角跨越 ±pi 的轨迹：沿 `-x` 前进，y 先增后减。
///
/// 两段的切线角分别落在 `+pi` 与 `-pi` 邻域，若不展开航向就会出现约 2*pi 的跳变。
srm27_minco_core::Trajectory2D makeBranchCutTrajectory()
{
  srm27_minco_core::Trajectory2D trajectory;

  srm27_minco_core::Trajectory2D::Coefficients cx{};
  srm27_minco_core::Trajectory2D::Coefficients cy{};
  cx[0] = 1.0;
  cx[1] = -1.0;
  cy[0] = -0.05;
  cy[1] = 0.1;
  EXPECT_TRUE(trajectory.addPiece(cx, cy, 1.0));

  cx[0] = 0.0;
  cx[1] = -1.0;
  cy[0] = 0.05;
  cy[1] = -0.1;
  EXPECT_TRUE(trajectory.addPiece(cx, cy, 1.0));
  return trajectory;
}

/// \brief 构造有效状态。
srm27_minco_core::State2D makeState(
  const double _x, const double _y, const double _yaw = 0.0, const bool _valid = true)
{
  srm27_minco_core::State2D state;
  state.valid = _valid;
  state.x = _x;
  state.y = _y;
  state.yaw = _yaw;
  state.sample_stamp = 1.0;
  state.received_stamp = 1.0;
  return state;
}

/// \brief FOLLOW 模式：航向参考取轨迹切线角，角速度参考恒为 0。
class TangentYawProvider : public srm27_minco_core::YawReferenceProvider
{
public:
  bool yawReference(
    double _trajectory_time, const Eigen::Vector2d & _position, const Eigen::Vector2d & _velocity,
    bool _tangent_valid, const Eigen::Vector2d & _tangent, double _previous_yaw, double & _yaw,
    double & _omega) override
  {
    (void)_trajectory_time;
    (void)_position;
    (void)_velocity;
    // 切线不可用（低速）时保持上一次连续航向，避免参考抖动。
    _yaw = _tangent_valid ? std::atan2(_tangent.y(), _tangent.x()) : _previous_yaw;
    _omega = 0.0;
    return true;
  }
};

/// \brief xy_only 场景：航向固定为构造值，角速度参考恒为 0。
class FixedYawProvider : public srm27_minco_core::YawReferenceProvider
{
public:
  explicit FixedYawProvider(const double _yaw) : yaw_(_yaw) {}

  bool yawReference(
    double _trajectory_time, const Eigen::Vector2d & _position, const Eigen::Vector2d & _velocity,
    bool _tangent_valid, const Eigen::Vector2d & _tangent, double _previous_yaw, double & _yaw,
    double & _omega) override
  {
    (void)_trajectory_time;
    (void)_position;
    (void)_velocity;
    (void)_tangent_valid;
    (void)_tangent;
    (void)_previous_yaw;
    _yaw = yaw_;
    _omega = 0.0;
    return true;
  }

private:
  double yaw_{0.0};
};

/// \brief 打印实测值，便于在测试日志中直接核对报告数据。
void printMeasurement(const std::string & _name, const double _value)
{
  std::cout << "[MEASURE] " << _name << " = " << _value << std::endl;
}

}  // namespace

// ---------------------------------------------------------------------------
// 配置与输入校验
// ---------------------------------------------------------------------------

TEST(TrackingReferenceConfigTest, Configure_RejectsInvalidParameters)
{
  srm27_minco_core::TrackingReferenceBuilder builder;
  std::string reason;

  srm27_minco_core::TrackingReferenceConfig config = trackingConfig();
  config.projection_backward_window = -0.1;
  EXPECT_FALSE(builder.configure(config, &reason));
  EXPECT_FALSE(reason.empty());

  config = trackingConfig();
  config.projection_forward_window = -0.5;
  EXPECT_FALSE(builder.configure(config));

  config = trackingConfig();
  config.projection_backward_window = 0.0;
  config.projection_forward_window = 0.0;
  EXPECT_FALSE(builder.configure(config));

  config = trackingConfig();
  config.max_progress_step = 0.0;
  EXPECT_FALSE(builder.configure(config));

  config = trackingConfig();
  config.backtrack_limit = -0.01;
  EXPECT_FALSE(builder.configure(config));

  config = trackingConfig();
  config.goal_slowdown_time = -0.1;
  EXPECT_FALSE(builder.configure(config));

  config = trackingConfig();
  config.projection_backward_window = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(builder.configure(config));

  EXPECT_FALSE(builder.hasProgress());
}

TEST(TrackingReferenceBuilderTest, Build_RejectsEmptyTrajectoryOrInvalidState)
{
  const srm27_minco_core::Trajectory2D trajectory = makeStraightTrajectory(3, 2.0, 0.5);
  srm27_minco_core::MpcModel model;
  ASSERT_TRUE(model.configure(trackingModelConfig()));
  TangentYawProvider provider;

  // 未 configure 的构造器不可用。
  srm27_minco_core::TrackingReferenceBuilder unconfigured;
  srm27_minco_core::TrackingReferenceResult result;
  EXPECT_FALSE(unconfigured.build(trajectory, makeState(0.5, 0.0), model, provider, result));

  srm27_minco_core::TrackingReferenceBuilder builder;
  ASSERT_TRUE(builder.configure(trackingConfig()));

  // 轨迹为空。
  const srm27_minco_core::Trajectory2D empty;
  EXPECT_FALSE(builder.build(empty, makeState(0.5, 0.0), model, provider, result));
  EXPECT_FALSE(result.valid);

  // 状态无效 / 位置非有限。
  EXPECT_FALSE(builder.build(trajectory, makeState(0.5, 0.0, 0.0, false), model, provider, result));
  const srm27_minco_core::State2D nan_state =
    makeState(std::numeric_limits<double>::quiet_NaN(), 0.0);
  EXPECT_FALSE(builder.build(trajectory, nan_state, model, provider, result));
  EXPECT_FALSE(builder.hasProgress());
}

// ---------------------------------------------------------------------------
// 直线轨迹投影、进度单调性与限幅
// ---------------------------------------------------------------------------

TEST(TrackingReferenceBuilderTest, Build_StraightLineProducesProjectedReference)
{
  const srm27_minco_core::Trajectory2D trajectory = makeStraightTrajectory(3, 2.0, 0.5);
  srm27_minco_core::MpcModel model;
  ASSERT_TRUE(model.configure(trackingModelConfig()));
  TangentYawProvider provider;
  srm27_minco_core::TrackingReferenceBuilder builder;
  ASSERT_TRUE(builder.configure(trackingConfig()));

  srm27_minco_core::TrackingReferenceResult result;
  ASSERT_TRUE(builder.build(trajectory, makeState(0.5, 0.0), model, provider, result));
  EXPECT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(trajectory.totalDuration(), 6.0);

  // 进度落在 [0, 总时长] 内，且状态在 x=0.5 处对应进度约 1.0 s。
  EXPECT_GE(result.progress, 0.0);
  EXPECT_LE(result.progress, trajectory.totalDuration());
  EXPECT_NEAR(result.progress, 1.0, 1e-3);
  printMeasurement("straight_line_progress", result.progress);

  // 维度与类型约定：速度层 3xN。
  ASSERT_EQ(result.z_ref.rows(), 3);
  ASSERT_EQ(result.z_ref.cols(), kSteps);
  ASSERT_EQ(result.u_ref.rows(), 3);
  ASSERT_EQ(result.u_ref.cols(), kSteps);
  ASSERT_EQ(result.tangent_angle.size(), static_cast<std::size_t>(kSteps));

  // 第一个位置参考就是状态在轨迹上的投影点。
  EXPECT_NEAR(result.z_ref(0, 0), 0.5, 1e-3);
  EXPECT_NEAR(result.z_ref(1, 0), 0.0, 1e-6);
  EXPECT_NEAR(result.z_ref(2, 0), 0.0, 1e-6);

  // 速度层模型的 u_ref 就是轨迹速度参考；终点不是全局目标时不做收敛。
  for (int i = 0; i < kSteps; ++i) {
    EXPECT_NEAR(result.u_ref(0, i), 0.5, 1e-6) << "i = " << i;
    EXPECT_NEAR(result.u_ref(1, i), 0.0, 1e-6) << "i = " << i;
    EXPECT_NEAR(result.u_ref(2, i), 0.0, 1e-6) << "i = " << i;
    EXPECT_NEAR(result.tangent_angle[static_cast<std::size_t>(i)], 0.0, 1e-9) << "i = " << i;
  }
  EXPECT_FALSE(result.near_terminal);
  EXPECT_TRUE(builder.hasProgress());
  EXPECT_NEAR(builder.progress(), result.progress, 1e-12);
}

TEST(TrackingReferenceBuilderTest, Build_ProgressIsMonotonicAlongForwardMotion)
{
  const srm27_minco_core::Trajectory2D trajectory = makeStraightTrajectory(3, 2.0, 0.5);
  srm27_minco_core::MpcModel model;
  ASSERT_TRUE(model.configure(trackingModelConfig()));
  TangentYawProvider provider;
  srm27_minco_core::TrackingReferenceBuilder builder;
  ASSERT_TRUE(builder.configure(trackingConfig()));

  srm27_minco_core::TrackingReferenceResult result;
  double previous_progress = -1.0;
  for (int k = 0; k <= 20; ++k) {
    const double x = 0.2 + 0.05 * static_cast<double>(k);  // 每次前进 0.05 m（进度 +0.1 s）
    ASSERT_TRUE(builder.build(trajectory, makeState(x, 0.0), model, provider, result));
    if (previous_progress >= 0.0) {
      EXPECT_GE(result.progress, previous_progress - 1e-9) << "k = " << k;
    }
    previous_progress = result.progress;
  }
  printMeasurement("monotonic_final_progress", previous_progress);
  EXPECT_GT(previous_progress, 2.0);
}

TEST(TrackingReferenceBuilderTest, Build_ClampsForwardProgressJump)
{
  const srm27_minco_core::Trajectory2D trajectory = makeStraightTrajectory(3, 2.0, 0.5);
  srm27_minco_core::MpcModel model;
  ASSERT_TRUE(model.configure(trackingModelConfig()));
  TangentYawProvider provider;
  const srm27_minco_core::TrackingReferenceConfig config = trackingConfig();
  srm27_minco_core::TrackingReferenceBuilder builder;
  ASSERT_TRUE(builder.configure(config));

  srm27_minco_core::TrackingReferenceResult result;
  ASSERT_TRUE(builder.build(trajectory, makeState(1.0, 0.0), model, provider, result));
  const double before = result.progress;

  // 状态突然跳到轨迹很靠后的位置（x=2.95 对应进度 5.9 s）。
  ASSERT_TRUE(builder.build(trajectory, makeState(2.95, 0.0), model, provider, result));
  const double increase = result.progress - before;
  printMeasurement("forward_jump_progress_increase", increase);
  EXPECT_GE(increase, 0.0);
  EXPECT_LE(increase, config.max_progress_step + 1e-9);
  // 不可能一步从进度约 2 s 直接跳到终点。
  EXPECT_LT(result.progress, trajectory.totalDuration());
}

TEST(TrackingReferenceBuilderTest, Build_ClampsBackwardProgressJump)
{
  const srm27_minco_core::Trajectory2D trajectory = makeStraightTrajectory(3, 2.0, 0.5);
  srm27_minco_core::MpcModel model;
  ASSERT_TRUE(model.configure(trackingModelConfig()));
  TangentYawProvider provider;
  const srm27_minco_core::TrackingReferenceConfig config = trackingConfig();
  srm27_minco_core::TrackingReferenceBuilder builder;
  ASSERT_TRUE(builder.configure(config));

  srm27_minco_core::TrackingReferenceResult result;
  ASSERT_TRUE(builder.build(trajectory, makeState(1.0, 0.0), model, provider, result));
  const double before = result.progress;

  // 状态突然倒退到起点，投影会落到窗口下边界，但回退量必须被限幅。
  ASSERT_TRUE(builder.build(trajectory, makeState(0.0, 0.0), model, provider, result));
  const double decrease = before - result.progress;
  printMeasurement("backward_jump_progress_decrease", decrease);
  EXPECT_GE(decrease, 0.0);
  EXPECT_LE(decrease, config.backtrack_limit + 1e-9);
  EXPECT_GT(result.progress, 0.0);
}

TEST(TrackingReferenceBuilderTest, ProjectProgress_RespectsSearchWindow)
{
  const srm27_minco_core::Trajectory2D trajectory = makeStraightTrajectory(3, 2.0, 0.5);
  const srm27_minco_core::TrackingReferenceConfig config = trackingConfig();
  srm27_minco_core::TrackingReferenceBuilder builder;
  ASSERT_TRUE(builder.configure(config));

  // x = 2.5 m 对应进度 5.0 s。
  const Eigen::Vector2d far_away(2.5, 0.0);
  EXPECT_NEAR(builder.projectProgress(trajectory, far_away, 0.0, false), 5.0, 1e-3);
  // 上一次进度为 1.0 s 时向前窗口只有 1.0 s，投影被限制在 2.0 s 附近。
  const double windowed = builder.projectProgress(trajectory, far_away, 1.0, true);
  printMeasurement("windowed_projection", windowed);
  EXPECT_LE(windowed, 1.0 + config.projection_forward_window + 1e-6);
  EXPECT_GT(windowed, 1.0);
  // 空轨迹返回 0，不抛异常。
  const srm27_minco_core::Trajectory2D empty;
  EXPECT_DOUBLE_EQ(builder.projectProgress(empty, Eigen::Vector2d(1.0, 0.0), 1.0, true), 0.0);
}

TEST(TrackingReferenceBuilderTest, Build_HairpinProjectionStaysOnNearbyBranch)
{
  // 去程 3 m / 6 s，回程 3 m / 6 s（y 偏移 0.05 m），总时长 12 s。
  const srm27_minco_core::Trajectory2D trajectory = makeHairpinTrajectory(3.0, 0.5, 0.05);
  ASSERT_DOUBLE_EQ(trajectory.totalDuration(), 12.0);
  // 回程在 t = 9 s 处正好经过 (1.5, 0.05)，与去程 t = 3 s 处的 (1.5, 0) 只差 5 cm。
  EXPECT_NEAR(trajectory.positionAt(9.0).x(), 1.5, 1e-9);
  EXPECT_NEAR(trajectory.positionAt(9.0).y(), 0.05, 1e-9);

  srm27_minco_core::MpcModel model;
  ASSERT_TRUE(model.configure(trackingModelConfig()));
  TangentYawProvider provider;
  srm27_minco_core::TrackingReferenceConfig config = trackingConfig();
  config.projection_forward_window = 0.50;
  srm27_minco_core::TrackingReferenceBuilder builder;
  ASSERT_TRUE(builder.configure(config));

  srm27_minco_core::TrackingReferenceResult result;
  ASSERT_TRUE(builder.build(trajectory, makeState(1.5, 0.0), model, provider, result));
  const double before = result.progress;
  EXPECT_NEAR(before, 3.0, 1e-3);

  // 状态移到回程腿上（全局最近点其实在 t = 9 s），但窗口 + 限幅必须挡住这次“跨分支跳跃”。
  ASSERT_TRUE(builder.build(trajectory, makeState(1.5, 0.05), model, provider, result));
  const double increase = result.progress - before;
  printMeasurement("hairpin_progress_increase", increase);
  EXPECT_LE(increase, config.max_progress_step + 1e-9);
  EXPECT_LE(increase, config.projection_forward_window + 1e-6);
  EXPECT_LT(result.progress, 6.0);  // 绝不能跳到回程分支
}

// ---------------------------------------------------------------------------
// 终点收敛与第二次采样
// ---------------------------------------------------------------------------

TEST(TrackingReferenceBuilderTest, Build_TerminalTaperConvergesToZeroVelocity)
{
  const srm27_minco_core::Trajectory2D straight = makeStraightTrajectory(3, 2.0, 0.5);
  srm27_minco_core::Trajectory2D trajectory = straight;
  trajectory.terminal_is_global_goal = true;

  srm27_minco_core::MpcModel model;
  ASSERT_TRUE(model.configure(trackingModelConfig()));
  TangentYawProvider provider;
  const srm27_minco_core::TrackingReferenceConfig config = trackingConfig();
  srm27_minco_core::TrackingReferenceBuilder builder;
  ASSERT_TRUE(builder.configure(config));

  // 状态放在接近终点处：x = 2.9 m → 进度 5.8 s，剩余 0.2 s < goal_slowdown_time(0.4)。
  srm27_minco_core::TrackingReferenceResult result;
  ASSERT_TRUE(builder.build(trajectory, makeState(2.9, 0.0), model, provider, result));
  EXPECT_TRUE(result.near_terminal);
  EXPECT_NEAR(result.progress, 5.8, 1e-3);

  double previous_magnitude = std::numeric_limits<double>::infinity();
  for (int i = 0; i < kSteps; ++i) {
    const double magnitude = result.u_ref.col(i).head<2>().norm();
    EXPECT_LE(magnitude, previous_magnitude + 1e-12) << "i = " << i;
    previous_magnitude = magnitude;
  }
  printMeasurement("terminal_tail_first_speed_ref", result.u_ref.col(0).head<2>().norm());
  printMeasurement("terminal_tail_last_speed_ref", result.u_ref.col(kSteps - 1).head<2>().norm());
  EXPECT_GT(result.u_ref.col(0).head<2>().norm(), 0.0);
  EXPECT_NEAR(result.u_ref.col(kSteps - 1).head<2>().norm(), 0.0, 1e-12);
  // 采样时间被裁剪到总时长，位置参考落在终点。
  EXPECT_NEAR(result.z_ref(0, kSteps - 1), trajectory.endPosition().x(), 1e-9);
  EXPECT_NEAR(result.z_ref(1, kSteps - 1), trajectory.endPosition().y(), 1e-9);
}

TEST(TrackingReferenceBuilderTest, MapBoundaryStopUsesTerminalTaperWithoutBeingGoal)
{
  auto trajectory = makeStraightTrajectory(3, 2.0, 0.5);
  trajectory.terminal_requires_stop = true;
  ASSERT_FALSE(trajectory.terminal_is_global_goal);
  srm27_minco_core::MpcModel model;
  ASSERT_TRUE(model.configure(trackingModelConfig()));
  TangentYawProvider provider;
  srm27_minco_core::TrackingReferenceBuilder builder;
  ASSERT_TRUE(builder.configure(trackingConfig()));
  srm27_minco_core::TrackingReferenceResult result;
  ASSERT_TRUE(builder.build(trajectory, makeState(2.9, 0), model, provider, result));
  EXPECT_TRUE(result.near_terminal);
  EXPECT_LT(result.u_ref.col(0).head<2>().norm(), 0.5);
  EXPECT_NEAR(result.u_ref.col(kSteps - 1).head<2>().norm(), 0.0, 1.0e-12);
}

TEST(TrackingReferenceBuilderTest, Build_SecondPassMatchesFirstPassWhenAligned)
{
  const srm27_minco_core::Trajectory2D trajectory = makeStraightTrajectory(3, 2.0, 0.5);
  srm27_minco_core::MpcModel model;
  ASSERT_TRUE(model.configure(trackingModelConfig()));
  TangentYawProvider provider;
  srm27_minco_core::TrackingReferenceBuilder builder;
  ASSERT_TRUE(builder.configure(trackingConfig()));

  srm27_minco_core::TrackingReferenceResult result;
  ASSERT_TRUE(builder.build(trajectory, makeState(1.0, 0.0), model, provider, result));
  const Eigen::MatrixXd first_z_ref = result.z_ref;
  const double first_progress = result.progress;

  // 预测速度与参考速度同向同速 → alpha = 1，采样增量与第一次完全一致。
  Eigen::MatrixXd predicted_velocity(2, kSteps);
  for (int i = 0; i < kSteps; ++i) {
    predicted_velocity.col(i) << 0.5, 0.0;
  }
  ASSERT_TRUE(builder.buildSecondPass(trajectory, predicted_velocity, model, provider, result));
  EXPECT_TRUE(result.valid);
  const double difference = (result.z_ref - first_z_ref).cwiseAbs().maxCoeff();
  printMeasurement("second_pass_aligned_max_difference", difference);
  EXPECT_LT(difference, 1e-9);
  EXPECT_DOUBLE_EQ(result.progress, first_progress);

  // 采样跨度约等于一个预测视界内的轨迹弧长：29 * h * 0.5 = 0.29 m。
  const double span = result.z_ref(0, kSteps - 1) - result.z_ref(0, 0);
  printMeasurement("second_pass_aligned_span", span);
  EXPECT_NEAR(span, static_cast<double>(kSteps - 1) * model.dt() * 0.5, 1e-6);
}

TEST(TrackingReferenceBuilderTest, Build_SecondPassScalesSamplingByAlpha)
{
  const srm27_minco_core::Trajectory2D trajectory = makeStraightTrajectory(3, 2.0, 0.5);
  srm27_minco_core::MpcModel model;
  ASSERT_TRUE(model.configure(trackingModelConfig()));
  TangentYawProvider provider;
  srm27_minco_core::TrackingReferenceBuilder builder;
  ASSERT_TRUE(builder.configure(trackingConfig()));

  srm27_minco_core::TrackingReferenceResult result;
  ASSERT_TRUE(builder.build(trajectory, makeState(1.0, 0.0), model, provider, result));

  // 预测速度与参考成 60° → alpha = cos(60°) = 0.5，采样跨度应约为一半。
  Eigen::MatrixXd predicted_velocity(2, kSteps);
  for (int i = 0; i < kSteps; ++i) {
    predicted_velocity.col(i) << 0.5 * std::cos(kPi / 3.0), 0.5 * std::sin(kPi / 3.0);
  }
  ASSERT_TRUE(builder.buildSecondPass(trajectory, predicted_velocity, model, provider, result));
  const double span = result.z_ref(0, kSteps - 1) - result.z_ref(0, 0);
  printMeasurement("second_pass_alpha_half_span", span);
  EXPECT_NEAR(span, 0.5 * static_cast<double>(kSteps - 1) * model.dt() * 0.5, 1e-6);
}

TEST(TrackingReferenceBuilderTest, Build_SecondPassStallsWhenPredictedVelocityReversed)
{
  const srm27_minco_core::Trajectory2D trajectory = makeStraightTrajectory(3, 2.0, 0.5);
  srm27_minco_core::MpcModel model;
  ASSERT_TRUE(model.configure(trackingModelConfig()));
  TangentYawProvider provider;
  srm27_minco_core::TrackingReferenceBuilder builder;
  ASSERT_TRUE(builder.configure(trackingConfig()));

  srm27_minco_core::TrackingReferenceResult result;
  ASSERT_TRUE(builder.build(trajectory, makeState(1.0, 0.0), model, provider, result));
  const double first_progress = result.progress;

  // 预测速度与参考反向 → cos = -1，被裁剪为 alpha = 0，采样时间不前进。
  Eigen::MatrixXd predicted_velocity(2, kSteps);
  for (int i = 0; i < kSteps; ++i) {
    predicted_velocity.col(i) << -0.5, 0.0;
  }
  ASSERT_TRUE(builder.buildSecondPass(trajectory, predicted_velocity, model, provider, result));
  EXPECT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(result.progress, first_progress);

  const double advance =
    (result.z_ref.col(kSteps - 1).head<2>() - result.z_ref.col(0).head<2>()).norm();
  printMeasurement("second_pass_reversed_advance", advance);
  EXPECT_LT(advance, 1e-9);
  // 所有采样点仍停在进度 2.0 s 处，即 x = 1.0 m（状态当前的投影位置）。
  EXPECT_NEAR(result.z_ref(0, 0), 1.0, 1e-3);
  EXPECT_NEAR(result.z_ref(0, kSteps - 1), 1.0, 1e-3);
}

TEST(TrackingReferenceBuilderTest, Build_SecondPassRejectsWrongShapeOrMissingFirstPass)
{
  const srm27_minco_core::Trajectory2D trajectory = makeStraightTrajectory(3, 2.0, 0.5);
  srm27_minco_core::MpcModel model;
  ASSERT_TRUE(model.configure(trackingModelConfig()));
  TangentYawProvider provider;
  srm27_minco_core::TrackingReferenceBuilder builder;
  ASSERT_TRUE(builder.configure(trackingConfig()));

  // 没有成功的第一次采样：直接拒绝。
  srm27_minco_core::TrackingReferenceResult fresh;
  Eigen::MatrixXd predicted_velocity = Eigen::MatrixXd::Zero(2, kSteps);
  EXPECT_FALSE(builder.buildSecondPass(trajectory, predicted_velocity, model, provider, fresh));

  srm27_minco_core::TrackingReferenceResult result;
  ASSERT_TRUE(builder.build(trajectory, makeState(1.0, 0.0), model, provider, result));

  // 维度错误（行数、列数）都必须返回 false。
  Eigen::MatrixXd wrong_rows = Eigen::MatrixXd::Zero(3, kSteps);
  EXPECT_FALSE(builder.buildSecondPass(trajectory, wrong_rows, model, provider, result));
  EXPECT_FALSE(result.valid);

  ASSERT_TRUE(builder.build(trajectory, makeState(1.0, 0.0), model, provider, result));
  Eigen::MatrixXd wrong_cols = Eigen::MatrixXd::Zero(2, kSteps + 1);
  EXPECT_FALSE(builder.buildSecondPass(trajectory, wrong_cols, model, provider, result));

  // 非有限预测速度同样拒绝。
  ASSERT_TRUE(builder.build(trajectory, makeState(1.0, 0.0), model, provider, result));
  Eigen::MatrixXd nan_velocity = Eigen::MatrixXd::Zero(2, kSteps);
  nan_velocity(0, 5) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(builder.buildSecondPass(trajectory, nan_velocity, model, provider, result));
}

// ---------------------------------------------------------------------------
// 航向参考：xy_only 固定航向与 ±pi 分支连续性
// ---------------------------------------------------------------------------

TEST(TrackingReferenceBuilderTest, Build_FixedHeadingProviderKeepsConstantYaw)
{
  const srm27_minco_core::Trajectory2D trajectory = makeStraightTrajectory(3, 2.0, 0.5);
  srm27_minco_core::MpcModel model;
  ASSERT_TRUE(model.configure(trackingModelConfig()));
  FixedYawProvider provider(1.234);
  srm27_minco_core::TrackingReferenceBuilder builder;
  ASSERT_TRUE(builder.configure(trackingConfig()));

  srm27_minco_core::TrackingReferenceResult result;
  ASSERT_TRUE(builder.build(trajectory, makeState(1.0, 0.0, 0.0), model, provider, result));
  for (int i = 0; i < kSteps; ++i) {
    EXPECT_NEAR(result.z_ref(2, i), 1.234, 1e-12) << "i = " << i;
    EXPECT_NEAR(result.u_ref(2, i), 0.0, 1e-12) << "i = " << i;
  }
  // 位置与平移速度参考仍来自轨迹本身。
  EXPECT_NEAR(result.u_ref(0, 0), 0.5, 1e-6);
  EXPECT_NEAR(result.z_ref(0, 0), 1.0, 1e-3);
}

TEST(TrackingReferenceBuilderTest, Build_YawReferenceStaysContinuousAcrossBranchCut)
{
  const srm27_minco_core::Trajectory2D trajectory = makeBranchCutTrajectory();
  ASSERT_DOUBLE_EQ(trajectory.totalDuration(), 2.0);
  srm27_minco_core::MpcModel model;
  ASSERT_TRUE(model.configure(trackingModelConfig()));
  TangentYawProvider provider;
  srm27_minco_core::TrackingReferenceBuilder builder;
  ASSERT_TRUE(builder.configure(trackingConfig()));

  // 状态放在 t = 0.9 s 处：预测视界 0.6 s 会跨过 t = 1.0 s 的 ±pi 分支切点。
  srm27_minco_core::TrackingReferenceResult result;
  ASSERT_TRUE(builder.build(trajectory, makeState(0.1, 0.04, kPi), model, provider, result));
  EXPECT_NEAR(result.progress, 0.9, 1e-3);

  const double expected_first = std::atan2(0.1, -1.0);  // ≈ +3.0419 rad
  EXPECT_NEAR(result.z_ref(2, 0), expected_first, 1e-3);

  double max_difference = 0.0;
  for (int i = 0; i + 1 < kSteps; ++i) {
    const double difference = std::abs(result.z_ref(2, i + 1) - result.z_ref(2, i));
    max_difference = std::max(max_difference, difference);
    EXPECT_LT(difference, kPi) << "i = " << i;
  }
  printMeasurement("branch_cut_max_yaw_step", max_difference);
  printMeasurement("branch_cut_last_yaw", result.z_ref(2, kSteps - 1));
  // 未展开的实现会在此处跳回 -3.0419；展开后应继续增大并越过 +pi。
  EXPECT_GT(result.z_ref(2, kSteps - 1), kPi);
  EXPECT_LT(result.z_ref(2, kSteps - 1), expected_first + 2.0 * kPi);
}
