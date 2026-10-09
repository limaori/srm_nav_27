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

#include "srm27_minco_core/mpc_solver.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <qpOASES.hpp>

#include "srm27_minco_core/kinematics.hpp"

namespace srm27_minco_core
{

namespace
{

/// \brief 约束类型标记。
enum ConstraintKind {
  kSpeedPolygon = 0,
  kInputChange = 1,
};

/// \brief 圆周率。不依赖 M_PI，避免不同标准库宏开关带来的可移植性问题。
constexpr double kPi = 3.14159265358979323846;

}  // namespace

MpcSolver::MpcSolver() = default;

MpcSolver::~MpcSolver() = default;

bool MpcSolverConfig::valid(std::string * _reason) const
{
  const auto fail = [&_reason](const char * _message) {
    if (_reason != nullptr) {
      *_reason = _message;
    }
    return false;
  };

  const double weights[] = {q_position, q_yaw,          q_velocity,    q_omega,      r_translation,
                            r_angular,  r_input_change, q_along_track, q_cross_track};
  for (const double weight : weights) {
    if (!isFinite(weight) || weight < 0.0) {
      return fail("MPC weights must be finite and non-negative");
    }
  }
  if (q_position <= 0.0 || r_translation <= 0.0 || r_angular <= 0.0) {
    return fail("position and input weights must be positive");
  }
  if (speed_polygon_sides < 4 || (speed_polygon_sides % 2) != 0) {
    return fail("speed_polygon_sides must be an even number of at least 4");
  }
  if (!isFinite(max_linear_accel) || max_linear_accel <= 0.0) {
    return fail("max_linear_accel must be positive");
  }
  if (!isFinite(max_angular_accel) || max_angular_accel <= 0.0) {
    return fail("max_angular_accel must be positive");
  }
  if (
    !isFinite(max_linear_jerk) || max_linear_jerk <= 0.0 || !isFinite(max_angular_jerk) ||
    max_angular_jerk <= 0.0) {
    return fail("jerk limits must be positive");
  }
  if (max_working_set_recalculations < 1) {
    return fail("max_working_set_recalculations must be at least 1");
  }
  if (!isFinite(qp_deadline_ms) || qp_deadline_ms <= 0.0) {
    return fail("qp_deadline_ms must be positive");
  }
  if (use_tangent_normal_weight && (q_along_track <= 0.0 || q_cross_track <= 0.0)) {
    return fail("tangent/normal weights must be positive when enabled");
  }
  return true;
}

Eigen::Matrix2d MpcSolver::planarWeight(double _tangent_angle, double _q_along, double _q_cross)
{
  const Eigen::Vector2d tangent(std::cos(_tangent_angle), std::sin(_tangent_angle));
  const Eigen::Vector2d normal(-std::sin(_tangent_angle), std::cos(_tangent_angle));
  return _q_along * tangent * tangent.transpose() + _q_cross * normal * normal.transpose();
}

bool MpcSolver::configure(
  const MpcModel::Config & _model, const MpcSolverConfig & _config, std::string * _reason)
{
  configured_ = false;
  if (!_config.valid(_reason)) {
    return false;
  }
  if (!model_.configure(_model, _reason)) {
    return false;
  }

  config_ = _config;
  n_ = model_.decisionDim();
  m_ = 0;

  const int steps = model_.steps();
  const int nz = model_.stateDim();
  const int nu = model_.inputDim();

  // Rbar：输入权重（分块对角）。
  rbar_ = Eigen::MatrixXd::Zero(n_, n_);
  for (int i = 0; i < steps; ++i) {
    Eigen::MatrixXd block = Eigen::MatrixXd::Zero(nu, nu);
    block(0, 0) = config_.r_translation;
    block(1, 1) = config_.r_translation;
    block(2, 2) = config_.r_angular;
    rbar_.block(i * nu, i * nu, nu, nu) = block;
  }

  // Sbar：输入变化权重（分块对角）。
  sbar_ = Eigen::MatrixXd::Zero(n_, n_);
  if (config_.r_input_change > 0.0) {
    for (int i = 0; i < steps; ++i) {
      sbar_.block(i * nu, i * nu, nu, nu) =
        config_.r_input_change * Eigen::MatrixXd::Identity(nu, nu);
    }
  }

  // Ddu：输入差分算子，(Ddu U)_i = U_i - U_{i-1}，其中 U_{-1} 由 d_prev 给出。
  ddu_ = Eigen::MatrixXd::Zero(n_, n_);
  for (int i = 0; i < steps; ++i) {
    ddu_.block(i * nu, i * nu, nu, nu) = Eigen::MatrixXd::Identity(nu, nu);
    if (i > 0) {
      ddu_.block(i * nu, (i - 1) * nu, nu, nu) = -Eigen::MatrixXd::Identity(nu, nu);
    }
  }
  d_prev_ = Eigen::VectorXd::Zero(n_);
  h_ = Eigen::MatrixXd::Zero(n_, n_);
  g_ = Eigen::VectorXd::Zero(n_);
  h_row_major_.resize(n_, n_);
  lb_ = Eigen::VectorXd::Zero(n_);
  ub_ = Eigen::VectorXd::Zero(n_);
  qbar_ = Eigen::MatrixXd::Zero(nz * steps, nz * steps);

  if (!setLimits(limits_, _reason)) {
    // 默认约束非法时使用保守默认值并明确报告。
    Limits2D fallback;
    if (!setLimits(fallback, _reason)) {
      return false;
    }
  }

  configured_ = true;
  return true;
}

bool MpcSolver::setLimits(const Limits2D & _limits, std::string * _reason)
{
  if (!_limits.valid()) {
    if (_reason != nullptr) {
      *_reason = "invalid limits";
    }
    return false;
  }
  limits_ = _limits;
  if (configured_) {
    // 限速变化：约束上下界在每次 solve 时重建，这里只需要重新计算多边形内接半径。
    const double v_max = limits_.max_linear_speed;
    const double inner = v_max * std::cos(kPi / static_cast<double>(config_.speed_polygon_sides));
    if (!isFinite(inner) || inner <= 0.0) {
      if (_reason != nullptr) {
        *_reason = "speed polygon inradius is degenerate";
      }
      return false;
    }
  }
  return true;
}

void MpcSolver::reset()
{
  // 清掉跨周期的输入基准，并让下一次求解重新冷启动。
  d_prev_.setZero();
  persistent_problem_.reset();
  persistent_constraint_count_ = -1;
  persistent_initialized_ = false;
  last_solve_used_hot_start_ = false;
}

void MpcSolver::buildQbar(const MpcReference & _reference)
{
  const int steps = model_.steps();
  const int nz = model_.stateDim();
  qbar_.setZero();
  for (int i = 0; i < steps; ++i) {
    Eigen::MatrixXd block = Eigen::MatrixXd::Zero(nz, nz);
    if (
      config_.use_tangent_normal_weight &&
      static_cast<int>(_reference.tangent_angle.size()) == steps) {
      block.block<2, 2>(0, 0) = planarWeight(
        _reference.tangent_angle[static_cast<std::size_t>(i)], config_.q_along_track,
        config_.q_cross_track);
    } else {
      block(0, 0) = config_.q_position;
      block(1, 1) = config_.q_position;
    }
    block(2, 2) = config_.q_yaw;
    if (nz > 3) {
      block(3, 3) = config_.q_velocity;
      block(4, 4) = config_.q_velocity;
      block(5, 5) = config_.q_omega;
    }
    qbar_.block(i * nz, i * nz, nz, nz) = block;
  }
}

bool MpcSolver::buildConstraints(const MpcInitialState & _state, std::string * _reason)
{
  const int steps = model_.steps();
  const int nu = model_.inputDim();
  const bool velocity_model = model_.type() == MpcModelType::kVelocityIntegrator;

  // 1) 变量上下界（分量盒约束）。
  lb_.setConstant(-std::numeric_limits<double>::infinity());
  ub_.setConstant(std::numeric_limits<double>::infinity());
  for (int i = 0; i < steps; ++i) {
    if (velocity_model) {
      lb_(i * nu + 0) = -limits_.max_linear_speed;
      ub_(i * nu + 0) = limits_.max_linear_speed;
      lb_(i * nu + 1) = -limits_.max_linear_speed;
      ub_(i * nu + 1) = limits_.max_linear_speed;
      lb_(i * nu + 2) = -limits_.max_angular_speed;
      ub_(i * nu + 2) = limits_.max_angular_speed;
    } else {
      lb_(i * nu + 0) = -config_.max_linear_accel;
      ub_(i * nu + 0) = config_.max_linear_accel;
      lb_(i * nu + 1) = -config_.max_linear_accel;
      ub_(i * nu + 1) = config_.max_linear_accel;
      lb_(i * nu + 2) = -config_.max_angular_accel;
      ub_(i * nu + 2) = config_.max_angular_accel;
    }
  }

  // 2) 统计一般约束行数。
  const int polygon_rows = velocity_model ? config_.speed_polygon_sides * steps : 0;
  // 输入差分：每一步每个分量两条不等式。
  //
  // 行数**恒定**（不受 `has_previous_input` 影响）是刻意的：qpOASES 的持久求解器在约束
  // 矩阵行数变化时必须重建并冷启动，而每次提交新轨迹都会清掉上一条命令基准；行数恒定
  // 才能让热启动在整段任务里持续生效。没有上一条命令时把首步差分限值放宽到全速域，
  // 等价于“不约束首步”，语义与该约束本来要表达的内容一致（方案 §3.5 修正项 3）。
  const int change_rows = config_.enforce_input_change ? 2 * nu * steps : 0;

  m_ = polygon_rows + change_rows;
  if (m_ < 0) {
    m_ = 0;
  }
  a_row_major_.resize(m_, n_);
  a_row_major_.setZero();
  lb_a_ = Eigen::VectorXd::Zero(m_);
  ub_a_ = Eigen::VectorXd::Zero(m_);
  constraint_step_.assign(static_cast<std::size_t>(m_), -1);
  constraint_kind_.assign(static_cast<std::size_t>(m_), -1);

  int row = 0;

  // 3) 合速度的多边形内接近似（只对速度层模型，因为只有它的决策量是速度）。
  if (velocity_model) {
    const int sides = config_.speed_polygon_sides;
    const double inner = limits_.max_linear_speed * std::cos(kPi / static_cast<double>(sides));
    if (!isFinite(inner) || inner < 0.0) {
      if (_reason != nullptr) {
        *_reason = "speed polygon inradius is invalid";
      }
      return false;
    }
    for (int i = 0; i < steps; ++i) {
      for (int j = 0; j < sides; ++j) {
        const double angle = 2.0 * kPi * static_cast<double>(j) / static_cast<double>(sides);
        a_row_major_(row, i * nu + 0) = std::cos(angle);
        a_row_major_(row, i * nu + 1) = std::sin(angle);
        lb_a_(row) = -inner;
        ub_a_(row) = inner;
        constraint_step_[static_cast<std::size_t>(row)] = i;
        constraint_kind_[static_cast<std::size_t>(row)] = kSpeedPolygon;
        ++row;
      }
    }
  }

  // 4) 输入差分约束。
  if (config_.enforce_input_change) {
    for (int i = 0; i < steps; ++i) {
      const bool first_without_reference = (i == 0) && !_state.has_previous_input;
      // 首步用真实控制间隔，后续步用模型步长（方案 §3.5 修正项 3）。
      double interval = model_.dt();
      if (i == 0) {
        interval = _state.control_interval;
      }
      if (!isFinite(interval) || interval <= 0.0) {
        if (_reason != nullptr) {
          *_reason = "invalid control interval for input difference constraint";
        }
        return false;
      }
      double limit_translation = 0.0;
      double limit_angular = 0.0;
      if (first_without_reference) {
        // 没有上一条执行命令：首步没有可比较的基准，按全速域放宽（等价于不约束）。
        limit_translation = 2.0 * limits_.max_linear_speed;
        limit_angular = 2.0 * limits_.max_angular_speed;
      } else if (velocity_model) {
        limit_translation = config_.max_linear_accel * interval;
        limit_angular = config_.max_angular_accel * interval;
      } else {
        limit_translation = config_.max_linear_jerk * interval;
        limit_angular = config_.max_angular_jerk * interval;
      }

      for (int c = 0; c < nu; ++c) {
        const double limit = (c == 2) ? limit_angular : limit_translation;
        // +e_c^T (U_i - U_{i-1}) <= limit
        a_row_major_(row, i * nu + c) += 1.0;
        if (i > 0) {
          a_row_major_(row, (i - 1) * nu + c) += -1.0;
        }
        // 首步 -limit <= U_0 - base <= limit，即 base-limit <= U_0 <= base+limit。
        // base 的符号不能取反，否则非零速度下首步会被强迫反向。
        double base = 0.0;
        if (i == 0) {
          base = _state.has_previous_input ? _state.previous_applied_input(c) : 0.0;
        }
        lb_a_(row) = -limit + base;
        ub_a_(row) = limit + base;
        constraint_step_[static_cast<std::size_t>(row)] = i;
        constraint_kind_[static_cast<std::size_t>(row)] = kInputChange;
        ++row;

        // -e_c^T (U_i - U_{i-1}) <= limit
        a_row_major_(row, i * nu + c) += -1.0;
        if (i > 0) {
          a_row_major_(row, (i - 1) * nu + c) += 1.0;
        }
        lb_a_(row) = -limit - base;
        ub_a_(row) = limit - base;
        constraint_step_[static_cast<std::size_t>(row)] = i;
        constraint_kind_[static_cast<std::size_t>(row)] = kInputChange;
        ++row;
      }
    }
  }

  if (row != m_) {
    if (_reason != nullptr) {
      *_reason = "constraint row count mismatch";
    }
    return false;
  }
  return true;
}

double MpcSolver::computeConstraintViolation(const Eigen::VectorXd & _u) const
{
  double violation = 0.0;
  for (int i = 0; i < n_; ++i) {
    violation = std::max(violation, lb_(i) - _u(i));
    violation = std::max(violation, _u(i) - ub_(i));
  }
  if (m_ > 0) {
    const Eigen::VectorXd values = a_row_major_ * _u;
    for (int i = 0; i < m_; ++i) {
      violation = std::max(violation, lb_a_(i) - values(i));
      violation = std::max(violation, values(i) - ub_a_(i));
    }
  }
  return std::max(0.0, violation);
}

bool MpcSolver::warmUp()
{
  if (!configured_) {
    return false;
  }
  MpcInitialState state;
  state.z0 = Eigen::VectorXd::Zero(model_.stateDim());
  state.has_previous_input = false;
  state.control_interval = model_.dt();
  MpcReference reference;
  reference.z_ref = Eigen::MatrixXd::Zero(model_.stateDim(), model_.steps());
  reference.u_ref = Eigen::MatrixXd::Zero(model_.inputDim(), model_.steps());
  MpcSolution solution;
  const bool ok = solve(state, reference, solution);
  // 预热失败（例如边界处退化）不是错误：真实求解时会重新尝试。
  return ok;
}

bool MpcSolver::solve(
  const MpcInitialState & _state, const MpcReference & _reference, MpcSolution & _solution)
{
  _solution = MpcSolution();
  if (!configured_) {
    _solution.status = SolveStatus::kNotInitialized;
    _solution.message = "solver not configured";
    return false;
  }

  const int steps = model_.steps();
  const int nz = model_.stateDim();
  const int nu = model_.inputDim();

  if (_state.z0.size() != nz || !_state.z0.allFinite()) {
    _solution.status = SolveStatus::kInvalidInput;
    _solution.message = "invalid initial state";
    return false;
  }
  if (
    _reference.z_ref.rows() != nz || _reference.z_ref.cols() != steps ||
    _reference.u_ref.rows() != nu || _reference.u_ref.cols() != steps ||
    !_reference.z_ref.allFinite() || !_reference.u_ref.allFinite()) {
    _solution.status = SolveStatus::kInvalidInput;
    _solution.message = "invalid reference dimensions or non-finite reference";
    return false;
  }
  if (
    _state.has_previous_input &&
    (_state.previous_applied_input.size() != nu || !_state.previous_applied_input.allFinite())) {
    _solution.status = SolveStatus::kInvalidInput;
    _solution.message = "invalid previous applied input";
    return false;
  }

  if (!buildConstraints(_state, &_solution.message)) {
    _solution.status = SolveStatus::kInvalidInput;
    return false;
  }

  buildQbar(_reference);

  const Eigen::VectorXd z_ref_stacked =
    Eigen::Map<const Eigen::VectorXd>(_reference.z_ref.data(), _reference.z_ref.size());
  const Eigen::VectorXd u_ref_stacked =
    Eigen::Map<const Eigen::VectorXd>(_reference.u_ref.data(), _reference.u_ref.size());

  d_prev_.setZero();
  if (_state.has_previous_input) {
    d_prev_.head(nu) = _state.previous_applied_input;
  }

  // H = 2 (Su^T Qbar Su + Rbar + Ddu^T Sbar Ddu)
  h_ = 2.0 * (model_.su().transpose() * qbar_ * model_.su() + rbar_);
  if (config_.r_input_change > 0.0) {
    h_ += 2.0 * (ddu_.transpose() * sbar_ * ddu_);
  }
  // g = 2 (Su^T Qbar (Sx z0 - Zref) - Rbar Uref - Ddu^T Sbar d_prev)
  g_ = 2.0 * (model_.su().transpose() * qbar_ * (model_.sx() * _state.z0 - z_ref_stacked) -
              rbar_ * u_ref_stacked);
  if (config_.r_input_change > 0.0) {
    g_ -= 2.0 * (ddu_.transpose() * sbar_ * d_prev_);
  }

  if (!h_.allFinite() || !g_.allFinite()) {
    _solution.status = SolveStatus::kNumericalFailure;
    _solution.message = "non-finite QP data";
    return false;
  }
  // qpOASES 默认按行优先读取矩阵。
  h_row_major_ = h_;

  const auto start_time = std::chrono::steady_clock::now();
  qpOASES::int_t nwsr = config_.max_working_set_recalculations;
  qpOASES::returnValue status = qpOASES::RET_INIT_FAILED;
  bool used_hot_start = false;
  // 单独持有“本次实际使用”的求解器，冷启动路径下局部对象出作用域后仍能取解。
  qpOASES::QProblem * active_problem = nullptr;
  std::unique_ptr<qpOASES::QProblem> cold_problem;

  if (config_.use_hot_start) {
    // H 与 Ac 在 configure() 之后固定不变，每周期只有 g 与上下界变化，正是 hotstart() 的场景。
    if (!persistent_problem_ || persistent_constraint_count_ != m_) {
      persistent_problem_ = std::make_unique<qpOASES::QProblem>(n_, m_);
      persistent_problem_->setPrintLevel(qpOASES::PL_NONE);
      persistent_constraint_count_ = m_;
      persistent_initialized_ = false;
    }
    if (persistent_initialized_) {
      status = persistent_problem_->hotstart(
        g_.data(), lb_.data(), ub_.data(), (m_ > 0) ? lb_a_.data() : nullptr,
        (m_ > 0) ? ub_a_.data() : nullptr, nwsr);
      used_hot_start = (status == qpOASES::SUCCESSFUL_RETURN);
    }
    if (!used_hot_start) {
      // 冷启动（首次、约束行数变化、或热启动数值失败）：正确性不变，只影响耗时。
      persistent_problem_ = std::make_unique<qpOASES::QProblem>(n_, m_);
      persistent_problem_->setPrintLevel(qpOASES::PL_NONE);
      persistent_constraint_count_ = m_;
      nwsr = config_.max_working_set_recalculations;
      status = persistent_problem_->init(
        h_row_major_.data(), g_.data(), (m_ > 0) ? a_row_major_.data() : nullptr, lb_.data(),
        ub_.data(), (m_ > 0) ? lb_a_.data() : nullptr, (m_ > 0) ? ub_a_.data() : nullptr, nwsr);
      persistent_initialized_ = (status == qpOASES::SUCCESSFUL_RETURN);
    }
    active_problem = persistent_problem_.get();
  } else {
    // 关闭热启动：每周期新建求解器，行为最容易核验（方案 §3.5 修正项 6 的首期做法）。
    cold_problem = std::make_unique<qpOASES::QProblem>(n_, m_);
    cold_problem->setPrintLevel(qpOASES::PL_NONE);
    status = cold_problem->init(
      h_row_major_.data(), g_.data(), (m_ > 0) ? a_row_major_.data() : nullptr, lb_.data(),
      ub_.data(), (m_ > 0) ? lb_a_.data() : nullptr, (m_ > 0) ? ub_a_.data() : nullptr, nwsr);
    active_problem = cold_problem.get();
  }

  const auto end_time = std::chrono::steady_clock::now();
  _solution.solve_time_ms =
    std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count() * 1.0e-6;
  _solution.iterations = static_cast<int>(nwsr);
  last_solve_used_hot_start_ = used_hot_start;

  if (status != qpOASES::SUCCESSFUL_RETURN) {
    switch (status) {
      case qpOASES::RET_INIT_FAILED_INFEASIBILITY:
        _solution.status = SolveStatus::kInfeasible;
        break;
      case qpOASES::RET_MAX_NWSR_REACHED:
        _solution.status = SolveStatus::kTimeout;
        break;
      default:
        _solution.status = SolveStatus::kNumericalFailure;
        break;
    }
    _solution.message = "qpOASES failed, returnValue=" + std::to_string(static_cast<int>(status));
    return false;
  }

  // 超过预算只做标记：解本身已经通过有限性与硬约束校验，丢弃它只会让机器人无故停下。
  _solution.deadline_exceeded = _solution.solve_time_ms > config_.qp_deadline_ms;

  Eigen::VectorXd u = Eigen::VectorXd::Zero(n_);
  active_problem->getPrimalSolution(u.data());
  if (!u.allFinite()) {
    _solution.status = SolveStatus::kNumericalFailure;
    _solution.message = "non-finite QP solution";
    return false;
  }

  _solution.max_constraint_violation = computeConstraintViolation(u);
  if (_solution.max_constraint_violation > 1.0e-6) {
    _solution.status = SolveStatus::kNumericalFailure;
    _solution.message = "solution violates constraints";
    return false;
  }

  _solution.u.resize(nu, steps);
  for (int i = 0; i < steps; ++i) {
    _solution.u.col(i) = u.segment(i * nu, nu);
  }
  _solution.z = model_.predict(_state.z0, u);
  if (_solution.z.cols() != steps + 1) {
    _solution.status = SolveStatus::kNumericalFailure;
    _solution.message = "failed to predict state trajectory";
    return false;
  }

  // 预测的最大平移速率：速度层模型看决策量，六维模型看状态里的速度分量。
  double max_speed = 0.0;
  for (int i = 0; i < steps; ++i) {
    if (model_.type() == MpcModelType::kVelocityIntegrator) {
      max_speed = std::max(max_speed, _solution.u.col(i).head<2>().norm());
    } else {
      max_speed = std::max(max_speed, _solution.z.block<2, 1>(3, i + 1).norm());
    }
  }
  _solution.predicted_max_speed = max_speed;

  _solution.status = SolveStatus::kSuccess;
  _solution.message.clear();
  return true;
}

bool MpcSolver::commandAt(
  const MpcSolution & _solution, double _lookahead, Eigen::VectorXd & _command) const
{
  if (!_solution.usable() || !isFinite(_lookahead) || _lookahead < 0.0) {
    return false;
  }
  const int steps = model_.steps();
  const double dt = model_.dt();

  // 命令时间落在两个离散预测点之间时做线性插值。
  const double index = _lookahead / dt;
  const int lower = static_cast<int>(std::floor(index));
  const double alpha = index - static_cast<double>(lower);

  const auto input_at = [&](int _step) {
    Eigen::VectorXd value;
    if (model_.type() == MpcModelType::kVelocityIntegrator) {
      const int clamped = std::min(std::max(_step, 0), steps - 1);
      value = _solution.u.col(clamped);
    } else {
      // 六维模型的决策量是加速度，这里返回对应时刻的预测速度。
      const int last_column = static_cast<int>(_solution.z.cols()) - 1;
      const int clamped = std::min(std::max(_step + 1, 0), last_column);
      value = _solution.z.block<3, 1>(3, clamped);
    }
    return value;
  };

  const Eigen::VectorXd lower_value = input_at(lower);
  if (alpha <= 1.0e-9) {
    _command = lower_value;
  } else {
    const Eigen::VectorXd upper_value = input_at(lower + 1);
    _command = (1.0 - alpha) * lower_value + alpha * upper_value;
  }
  if (_command.size() != 3 || !_command.allFinite()) {
    _command.resize(0);
    return false;
  }
  return true;
}

}  // namespace srm27_minco_core
