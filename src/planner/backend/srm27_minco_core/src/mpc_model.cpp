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

#include "srm27_minco_core/mpc_model.hpp"

#include <cmath>

#include "srm27_minco_core/kinematics.hpp"

namespace srm27_minco_core
{

bool MpcModel::configure(const Config & _config, std::string * _reason)
{
  const auto fail = [&_reason](const char * _message) {
    if (_reason != nullptr) {
      *_reason = _message;
    }
    return false;
  };

  configured_ = false;
  if (_config.prediction_steps < 1) {
    return fail("prediction_steps must be at least 1");
  }
  if (!isFinite(_config.prediction_dt) || _config.prediction_dt <= 0.0) {
    return fail("prediction_dt must be positive");
  }

  config_ = _config;
  const double h = _config.prediction_dt;

  switch (_config.type) {
    case MpcModelType::kVelocityIntegrator:
      state_dim_ = 3;
      input_dim_ = 3;
      a_ = Eigen::MatrixXd::Identity(3, 3);
      b_ = h * Eigen::MatrixXd::Identity(3, 3);
      break;
    case MpcModelType::kAccelerationDoubleIntegrator:
      state_dim_ = 6;
      input_dim_ = 3;
      a_ = Eigen::MatrixXd::Zero(6, 6);
      a_.block<3, 3>(0, 0) = Eigen::MatrixXd::Identity(3, 3);
      a_.block<3, 3>(0, 3) = h * Eigen::MatrixXd::Identity(3, 3);
      a_.block<3, 3>(3, 3) = Eigen::MatrixXd::Identity(3, 3);
      b_ = Eigen::MatrixXd::Zero(6, 3);
      b_.block<3, 3>(0, 0) = 0.5 * h * h * Eigen::MatrixXd::Identity(3, 3);
      b_.block<3, 3>(3, 0) = h * Eigen::MatrixXd::Identity(3, 3);
      break;
    default:
      return fail("unknown MPC model type");
  }

  const int steps = _config.prediction_steps;
  const int n = state_dim_ * steps;
  const int m = input_dim_ * steps;
  sx_ = Eigen::MatrixXd::Zero(n, state_dim_);
  su_ = Eigen::MatrixXd::Zero(n, m);

  // Z_i = A^(i+1) z0 + Σ_{j<=i} A^(i-j) B u_j，与模型无关的通用凝聚构造。
  Eigen::MatrixXd a_power = Eigen::MatrixXd::Identity(state_dim_, state_dim_);
  for (int i = 0; i < steps; ++i) {
    a_power = a_power * a_;
    sx_.block(i * state_dim_, 0, state_dim_, state_dim_) = a_power;
    Eigen::MatrixXd a_power_inner = Eigen::MatrixXd::Identity(state_dim_, state_dim_);
    for (int j = i; j >= 0; --j) {
      // a_power_inner = A^(i-j)
      su_.block(i * state_dim_, j * input_dim_, state_dim_, input_dim_) = a_power_inner * b_;
      a_power_inner = a_power_inner * a_;
    }
  }

  configured_ = true;
  return true;
}

Eigen::MatrixXd MpcModel::predict(const Eigen::VectorXd & _z0, const Eigen::VectorXd & _u) const
{
  if (!configured_ || _z0.size() != state_dim_ || _u.size() != decisionDim()) {
    return Eigen::MatrixXd();
  }
  Eigen::MatrixXd trajectory(state_dim_, steps() + 1);
  trajectory.col(0) = _z0;
  for (int i = 0; i < steps(); ++i) {
    trajectory.col(i + 1) = a_ * trajectory.col(i) + b_ * _u.segment(i * input_dim_, input_dim_);
  }
  return trajectory;
}

}  // namespace srm27_minco_core
