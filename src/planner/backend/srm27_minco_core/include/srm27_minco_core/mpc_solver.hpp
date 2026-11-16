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

#ifndef SRM27_MINCO_CORE__MPC_SOLVER_HPP_
#define SRM27_MINCO_CORE__MPC_SOLVER_HPP_

#include <Eigen/Core>
#include <memory>
#include <string>
#include <vector>

#include "srm27_minco_core/mpc_model.hpp"
#include "srm27_minco_core/types.hpp"

// 只前向声明 qpOASES 的求解器类型，避免把 QP 求解器头文件泄漏到本包公开接口。
// 代价是析构函数必须在 .cpp 中显式定义（pimpl 惯用法）。
namespace qpOASES
{
class QProblem;
}  // namespace qpOASES

namespace srm27_minco_core
{

/// \brief MPC 求解器配置。
///
/// 所有权重都需要本项目按场景标定；单位写在字段注释里。所有矩阵在 `configure()`
/// 时预分配，控制热路径上不做动态分配（方案 §4.5.3）。
struct MpcSolverConfig
{
  /// \brief 位置误差权重（1/m^2）。
  double q_position{20.0};
  /// \brief 航向误差权重（1/rad^2）。
  double q_yaw{5.0};
  /// \brief 平移速度误差权重（1/(m/s)^2），仅六维加速度模型使用。
  double q_velocity{1.0};
  /// \brief 角速度误差权重（1/(rad/s)^2），仅六维加速度模型使用。
  double q_omega{0.5};
  /// \brief 平移输入权重（1/(m/s)^2 或 1/(m/s^2)^2）。
  double r_translation{1.0};
  /// \brief 自转输入权重。
  double r_angular{1.0};
  /// \brief 输入变化惩罚权重；0 表示关闭。
  double r_input_change{0.0};

  /// \brief 是否按轨迹切线构造横纵向不同的位置权重（方案 §3.5 修正项 2）。
  ///
  /// 权重矩阵为 `Qxy = q_along * t t^T + q_cross * n n^T`，交叉项为
  /// `(q_along - q_cross) * cos(theta) * sin(theta)`；**不能**把符号写反。
  bool use_tangent_normal_weight{false};
  /// \brief 沿轨迹切线方向的位置权重（1/m^2）。
  double q_along_track{20.0};
  /// \brief 垂直轨迹切线方向的位置权重（1/m^2）。
  double q_cross_track{40.0};

  /// \brief 合速度约束的保守内接正多边形边数 K（方案 §7.2）。
  ///
  /// `||v|| <= v_max` 是二阶锥约束，不能直接交给 qpOASES；用
  /// `n_j^T v <= v_max * cos(pi/K)` 的 K 条半平面近似，K 必须为 >= 4 的偶数。
  int speed_polygon_sides{8};

  /// \brief 是否施加输入差分（速度层模型即加速度）约束。
  bool enforce_input_change{true};
  /// \brief 平移加速度上限（m/s^2），速度层模型用它约束速度差分。
  double max_linear_accel{0.3};
  /// \brief 角加速度上限（rad/s^2），速度层模型用它约束角速度差分。
  double max_angular_accel{0.5};
  /// \brief 平移 jerk 上限（m/s^3），仅六维加速度模型用它约束加速度差分。
  double max_linear_jerk{5.0};
  /// \brief 角 jerk 上限（rad/s^3），仅六维加速度模型使用。
  double max_angular_jerk{10.0};

  /// \brief 是否复用持久求解器并热启动。
  ///
  /// 目标函数与约束矩阵在 `configure()` 之后固定不变，每周期只有梯度与上下界变化，
  /// 正是 qpOASES `hotstart()` 的适用场景。默认开启：实测 N=30、K=8、含输入差分约束时，
  /// 每周期新建 `QProblem` 冷启动约 7.5 ms（P99 约 9 ms），热启动可降到亚毫秒量级；
  /// 关闭后回到“每周期冷启动”的可核验行为（方案 §3.5 修正项 6）。
  ///
  /// 热启动失败（数值退化、工作集变化过大）时会自动回退到一次冷启动，不影响正确性。
  bool use_hot_start{true};

  /// \brief 单次求解允许的工作集重算次数上限。
  int max_working_set_recalculations{200};
  /// \brief QP 求解时间预算（ms）；超过后结果按超时标记。
  double qp_deadline_ms{5.0};

  /// \brief 配置自检。
  bool valid(std::string * _reason = nullptr) const;
};

/// \brief 一次 MPC 求解的输入状态。
struct MpcInitialState
{
  /// \brief 状态初值，长度 `MpcModel::stateDim()`。
  Eigen::VectorXd z0{};
  /// \brief 上一周期**实际执行**的输入（经过限幅/死区后的命令），长度 `inputDim`。
  Eigen::VectorXd previous_applied_input{};
  /// \brief 是否存在上一周期执行命令；为假时首步不做差分约束。
  bool has_previous_input{false};
  /// \brief 本周期与上一周期之间的**真实**控制间隔（s），用于首步差分约束。
  double control_interval{0.02};
};

/// \brief MPC 参考。
struct MpcReference
{
  /// \brief 参考状态，形状 `(nz, N)`。
  Eigen::MatrixXd z_ref{};
  /// \brief 参考输入，形状 `(nu, N)`。
  Eigen::MatrixXd u_ref{};
  /// \brief 可选的逐步轨迹切线角（rad，长度 N），用于横纵向权重。
  std::vector<double> tangent_angle{};
};

/// \brief MPC 求解结果。
struct MpcSolution
{
  /// \brief 状态。
  SolveStatus status{SolveStatus::kNotInitialized};
  /// \brief 输入序列，形状 `(nu, N)`。
  Eigen::MatrixXd u{};
  /// \brief 预测状态轨迹（含初值），形状 `(nz, N+1)`。
  Eigen::MatrixXd z{};
  /// \brief 求解耗时（ms）。
  double solve_time_ms{0.0};
  /// \brief 工作集重算次数。
  int iterations{0};
  /// \brief 最大约束违背量（越界单位为 m/s 或 rad/s 等原始单位）。
  double max_constraint_violation{0.0};
  /// \brief 预测的最大平移速率（m/s）。
  double predicted_max_speed{0.0};
  /// \brief 本次求解是否超过 `qp_deadline_ms`。
  ///
  /// 超时**不影响解的有效性**：解仍然通过了有限性与硬约束校验，只是说明本周期算得慢了。
  /// 调用方据此计入 `missed_deadline_count`，而不是丢弃一个合法解、让机器人无故停下。
  bool deadline_exceeded{false};
  /// \brief 说明信息（失败原因）。
  std::string message{};

  /// \brief 是否可直接使用。
  bool usable() const { return status == SolveStatus::kSuccess; }
};

/// \brief 基于 qpOASES 的凝聚 QP 全向 MPC 求解器（速度层 + 六维加速度层）。
///
/// 目标（方案 §7.1–§7.2）：
/// ```text
/// J = (Z - Zref)^T Qbar (Z - Zref)
///   + (U - Uref)^T Rbar (U - Uref)
///   + (Ddu U - d_prev)^T Sbar (Ddu U - d_prev)
/// H = 2 (Su^T Qbar Su + Rbar + Ddu^T Sbar Ddu)
/// g = 2 (Su^T Qbar (Sx z0 - Zref) - Rbar Uref - Ddu^T Sbar d_prev)
/// ```
/// 两边的系数 2 一致；约束以 `lbA <= Ac U <= ubA` 与 `lb <= U <= ub` 表达。
///
/// 本类不是线程安全的：一个实例只能由一个线程使用。
class MpcSolver
{
public:
  MpcSolver();
  /// \brief 析构（在 .cpp 中定义，以配合 qpOASES 的前向声明）。
  ~MpcSolver();
  MpcSolver(const MpcSolver &) = delete;
  MpcSolver & operator=(const MpcSolver &) = delete;

  /// \brief 配置模型与权重，并预分配矩阵。
  bool configure(
    const MpcModel::Config & _model, const MpcSolverConfig & _config,
    std::string * _reason = nullptr);

  /// \brief 设置有效约束（合速度上限、角速度上限、加速度上限）。
  bool setLimits(const Limits2D & _limits, std::string * _reason = nullptr);

  /// \brief 是否已配置。
  bool configured() const { return configured_; }

  /// \brief 预测模型。
  const MpcModel & model() const { return model_; }

  /// \brief 当前有效约束。
  const Limits2D & limits() const { return limits_; }

  /// \brief 求解一次 QP。
  bool solve(
    const MpcInitialState & _state, const MpcReference & _reference, MpcSolution & _solution);

  /// \brief 返回执行时刻 `_lookahead` 秒后的预测平移/自转速度（车体系转换由调用方完成）。
  ///
  /// 命令时间落在两个离散预测点之间时做线性插值，不四舍五入跳到任意预测步。
  /// \return 长度 3 的向量 `[vx, vy, omega]`；不可用时返回 false。
  bool commandAt(
    const MpcSolution & _solution, double _lookahead, Eigen::VectorXd & _command) const;

  /// \brief 平面各向异性权重矩阵 `Qxy = q_along * t t^T + q_cross * n n^T`。
  /// \param _tangent_angle 轨迹切线角（rad）。
  static Eigen::Matrix2d planarWeight(double _tangent_angle, double _q_along, double _q_cross);

  /// \brief 清空求解器内部缓存（新目标/取消/失活时调用）；下一次求解会冷启动。
  void reset();

  /// \brief 预热：用一次哑求解完成冷启动（H 分解与首个工作集），避免第一次真实求解
  /// 落在控制周期里（冷启动约 8~15 ms）。应在 `activate()` 之类非热路径上调用。
  /// \return 预热成功返回 true；失败不影响后续真实求解（只是没有预热）。
  bool warmUp();

  /// \brief 上一次求解是否走了热启动路径（诊断用）。
  bool lastSolveUsedHotStart() const { return last_solve_used_hot_start_; }

private:
  /// \brief 组装 Qbar（`nz*N x nz*N`）。
  void buildQbar(const MpcReference & _reference);
  /// \brief 组装约束矩阵与上下界。
  bool buildConstraints(const MpcInitialState & _state, std::string * _reason);
  /// \brief 计算解的最大约束违背量。
  double computeConstraintViolation(const Eigen::VectorXd & _u) const;

  MpcModel model_{};
  MpcSolverConfig config_{};
  Limits2D limits_{};
  bool configured_{false};

  int n_{0};
  int m_{0};

  Eigen::MatrixXd qbar_{};
  Eigen::MatrixXd rbar_{};
  Eigen::MatrixXd sbar_{};
  Eigen::MatrixXd ddu_{};
  Eigen::MatrixXd h_{};
  Eigen::VectorXd g_{};
  Eigen::VectorXd d_prev_{};

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> h_row_major_{};
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> a_row_major_{};
  Eigen::VectorXd lb_{};
  Eigen::VectorXd ub_{};
  Eigen::VectorXd lb_a_{};
  Eigen::VectorXd ub_a_{};
  /// \brief 每行约束所属的“步索引”，仅用于诊断。
  std::vector<int> constraint_step_{};
  /// \brief 约束类型标记，仅用于诊断。
  std::vector<int> constraint_kind_{};

  /// \brief 持久求解器（启用热启动时使用）。
  std::unique_ptr<qpOASES::QProblem> persistent_problem_{};
  /// \brief 持久求解器当前缓存的约束行数；行数变化时必须重建。
  int persistent_constraint_count_{-1};
  /// \brief 持久求解器是否已用当前 H/A 成功初始化。
  bool persistent_initialized_{false};
  /// \brief 上一次求解是否使用了热启动。
  bool last_solve_used_hot_start_{false};
};

}  // namespace srm27_minco_core

#endif  // SRM27_MINCO_CORE__MPC_SOLVER_HPP_
