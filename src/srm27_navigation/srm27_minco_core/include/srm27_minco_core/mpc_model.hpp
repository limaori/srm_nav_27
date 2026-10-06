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

#ifndef SRM27_MINCO_CORE__MPC_MODEL_HPP_
#define SRM27_MINCO_CORE__MPC_MODEL_HPP_

#include <Eigen/Core>
#include <string>

namespace srm27_minco_core
{

/// \brief MPC 预测模型类型。
///
/// 两种模型的**决策量与物理含义不同**，必须显式选择，不能只改参数名：
///  * `kVelocityIntegrator`：`z = [x, y, yaw]`，`u = [vx, vy, omega]`（都在 odom 系），
///    `z(k+1) = z(k) + h u(k)`。与本车速度接口匹配，是 P2 基线（方案 §7.1）。
///  * `kAccelerationDoubleIntegrator`：`z = [x, y, yaw, vx, vy, omega]`，
///    `u = [ax, ay, alpha]`，零阶保持双积分离散化。用于报告六维模型的对照实现。
enum class MpcModelType {
  kVelocityIntegrator = 0,
  kAccelerationDoubleIntegrator = 1,
};

/// \brief MPC 预测模型的离散状态空间与凝聚矩阵。
///
/// 只构建 `(A, B)` 的凝聚形式 `Z = Sx z0 + Su U`；矩阵在 `configure()` 时预分配，
/// 每个控制周期只做矩阵乘法（方案 §7.2）。
class MpcModel
{
public:
  /// \brief 模型配置。
  struct Config
  {
    /// \brief 模型类型。
    MpcModelType type{MpcModelType::kVelocityIntegrator};
    /// \brief 预测步数 N，必须为正。
    int prediction_steps{30};
    /// \brief 预测步长 h（s），必须为正。
    double prediction_dt{0.02};
  };

  MpcModel() = default;

  /// \brief 配置模型并构建凝聚矩阵。
  bool configure(const Config & _config, std::string * _reason = nullptr);

  /// \brief 是否已完成配置。
  bool configured() const { return configured_; }

  MpcModelType type() const { return config_.type; }
  int stateDim() const { return state_dim_; }
  int inputDim() const { return input_dim_; }
  int steps() const { return config_.prediction_steps; }
  double dt() const { return config_.prediction_dt; }
  int decisionDim() const { return input_dim_ * config_.prediction_steps; }

  /// \brief 状态索引：位置 x/y 在状态向量中的下标（两种模型都相同）。
  static int positionIndex() { return 0; }
  /// \brief 航向在状态向量中的下标。
  static int yawIndex() { return 2; }
  /// \brief 平移速度在状态向量中的下标；速度层模型没有速度状态时返回 -1。
  int velocityIndex() const
  {
    return config_.type == MpcModelType::kAccelerationDoubleIntegrator ? 3 : -1;
  }
  /// \brief 角速度在状态向量中的下标；速度层模型没有该状态时返回 -1。
  int omegaIndex() const
  {
    return config_.type == MpcModelType::kAccelerationDoubleIntegrator ? 5 : -1;
  }

  /// \brief 凝聚状态矩阵 `Sx`，形状 `(nz*N, nz)`。
  const Eigen::MatrixXd & sx() const { return sx_; }
  /// \brief 凝聚输入矩阵 `Su`，形状 `(nz*N, nu*N)`。
  const Eigen::MatrixXd & su() const { return su_; }

  /// \brief 由初值与输入序列预测状态轨迹。
  /// \param _z0 初值，长度 `stateDim()`。
  /// \param _u 输入序列，按步优先排列，长度 `decisionDim()`。
  /// \return 形状 `(nz, N+1)` 的状态轨迹（含初值）；参数非法时返回空矩阵。
  Eigen::MatrixXd predict(const Eigen::VectorXd & _z0, const Eigen::VectorXd & _u) const;

  /// \brief 单步矩阵 A。
  const Eigen::MatrixXd & a() const { return a_; }
  /// \brief 单步矩阵 B。
  const Eigen::MatrixXd & b() const { return b_; }

private:
  Config config_{};
  int state_dim_{0};
  int input_dim_{0};
  bool configured_{false};
  Eigen::MatrixXd a_{};
  Eigen::MatrixXd b_{};
  Eigen::MatrixXd sx_{};
  Eigen::MatrixXd su_{};
};

}  // namespace srm27_minco_core

#endif  // SRM27_MINCO_CORE__MPC_MODEL_HPP_
