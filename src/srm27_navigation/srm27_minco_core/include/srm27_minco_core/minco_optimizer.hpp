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

#ifndef SRM27_MINCO_CORE__MINCO_OPTIMIZER_HPP_
#define SRM27_MINCO_CORE__MINCO_OPTIMIZER_HPP_

#include <Eigen/Core>
#include <memory>
#include <string>
#include <vector>

#include "gcopter/minco.hpp"
#include "srm27_minco_core/esdf_2d.hpp"
#include "srm27_minco_core/trajectory_2d.hpp"
#include "srm27_minco_core/trajectory_initializer.hpp"
#include "srm27_minco_core/types.hpp"

namespace srm27_minco_core
{

/// \brief MINCO 优化器配置。
///
/// 权重都没有“通用最优值”，必须由本项目按场景标定；单位写在字段注释里。
struct MincoOptimizerConfig
{
  /// \brief jerk 能量权重（1/(m^2/s^5)，即代价单位 = 权重 * ∫||jerk||²dt）。
  double w_jerk{1.0};
  /// \brief 总时长权重（代价单位 = 权重 * 秒）。
  double w_time{10.0};
  /// \brief 障碍软约束权重（代价单位 = 权重 * 米）。
  double w_obstacle{100.0};
  /// \brief 速度软约束权重（代价单位 = 权重 * (m/s)^2）。
  double w_velocity{20.0};
  /// \brief 加速度软约束权重（代价单位 = 权重 * (m/s^2)^2）。
  double w_acceleration{2.0};
  /// \brief 路标点参考吸引权重（代价单位 = 权重 * m^2）；0 表示关闭。
  double w_reference{0.0};
  /// \brief PRE 阶段段时长比例惩罚权重；0 表示关闭。
  double w_time_ratio{1.0};

  /// \brief 软 hinge 的平滑系数（1/单位），越大越接近硬约束。
  double soft_hinge_beta{20.0};

  /// \brief 机器人碰撞包络半径（m），来自本车模型，不使用 costmap 的膨胀半径。
  double robot_radius{0.33};
  /// \brief 额外净空裕量（m），叠加在包络半径之上形成障碍软约束的安全距离。
  double clearance_margin{0.05};

  /// \brief 每段数值积分采样点数（midpoint 规则）。
  int samples_per_piece{8};

  /// \brief 段时长下限（s），通过 `T = Tmin + softplus(s)` 保证严格为正。
  double min_piece_duration{0.05};
  /// \brief 段时长上限（s），仅用于判定结果是否合理，不参与梯度。
  double max_piece_duration{1.0};

  /// \brief L-BFGS 最大迭代次数。
  int max_iterations{150};
  /// \brief L-BFGS 梯度收敛阈值。
  double gradient_tolerance{1.0e-4};
  /// \brief L-BFGS 历史长度。
  int lbfgs_memory{10};

  /// \brief 是否执行 PRE / FINELY 两阶段优化（方案 §6.5）。
  bool two_stage{true};
  /// \brief PRE 阶段允许的段时长与平均值之比下限。
  double pre_time_ratio_min{0.90};
  /// \brief PRE 阶段允许的段时长与平均值之比上限。
  double pre_time_ratio_max{1.10};

  /// \brief 是否启用报告中科大式的 FINELY 通道启发式（切向梯度剔除 + 方向探测）。
  ///
  /// 默认关闭：该启发式会修改梯度方向，代价与梯度不再是严格导数关系，
  /// 只能在常规 ESDF 软约束稳定后再单独评估（方案 §6.5）。
  bool enable_report_fine_heuristic{false};
  /// \brief 启发式的斜率阈值（报告为约 0.5）。
  double fine_slope_threshold{0.5};
  /// \brief 启发式探测步长（m），应随地图分辨率与插值有效邻域限定。
  double fine_probe_step{0.10};

  /// \brief 参数自检。
  bool valid(std::string * _reason = nullptr) const;
};

/// \brief MINCO 优化结果。
struct MincoOptimizeResult
{
  /// \brief 求解状态。
  SolveStatus status{SolveStatus::kNotInitialized};
  /// \brief 优化后的轨迹（仅当 `status == kSuccess` 时可用）。
  Trajectory2D trajectory{};
  /// \brief 最终代价。
  double cost{0.0};
  /// \brief 总迭代次数（两阶段求和）。
  int iterations{0};
  /// \brief PRE 阶段耗时（ms）。
  double pre_time_ms{0.0};
  /// \brief FINELY 阶段耗时（ms）；`two_stage` 为假时该阶段不执行，保持 0。
  double fine_time_ms{0.0};
  /// \brief 失败的阶段名或说明。
  std::string message{};
  /// \brief 是否实际做了时间优化（时间被固定时可用于区分失败原因）。
  bool time_optimized{false};
};

/// \brief 基于 GCOPTER MINCO 核心的二维地面轨迹优化器。
///
/// 设计约束（方案 §4.5、§6.4–§6.5）：
///  * 只暴露 x/y 与段时长；z 及其导数固定为 0，不搬入多旋翼的重力/推力约束。
///  * 内部使用 `MINCO_S3NU`（最小 jerk、五次多项式）；外部统一 `Trajectory2D`
///    的 c0..c5 系数顺序。
///  * 目标函数包含 jerk 能量、总时长、障碍软约束、速度/加速度软约束与可选参考吸引，
///    按段内 midpoint 数值积分求和，并**完整**回传路标点与段时长的导数（含积分权重
///    与采样时刻对时长的依赖）。
///  * 本类不做碰撞与动力学可行性判定，也不输出可执行轨迹；发布前必须经过
///    `TrajectoryValidator`。
///
/// 本类不是线程安全的：一个实例只能由一个线程使用。
class MincoOptimizer
{
public:
  MincoOptimizer() = default;

  /// \brief 设置配置。
  bool configure(const MincoOptimizerConfig & _config, std::string * _reason = nullptr);

  /// \brief 设置障碍距离场；优化器只读取它，不持有所有权。
  void setEsdf(std::shared_ptr<const Esdf2D> _esdf) { esdf_ = std::move(_esdf); }

  /// \brief 用上一帧的轨迹提供热启动初值（内部路标点 + 段时长）。
  /// \param _inner_points 内部路标点（不含首末点），数量必须是段数 - 1。
  /// \param _durations 段时长。
  /// \return 数量自洽时返回 true。
  bool setWarmStart(
    const std::vector<Eigen::Vector2d> & _inner_points, const std::vector<double> & _durations);

  /// \brief 清除热启动缓存。
  void clearWarmStart();

  /// \brief 优化初值。
  /// \param _guess 前端初值（路标点 + 时长 + 首末 p/v/a）。
  /// \param _limits 速度/加速度有效约束，用于把软约束阈值设为统一值。
  /// \param _result 输出结果。
  /// \return 成功返回 true 且 `_result.status == kSuccess`。
  bool optimize(
    const TrajectoryInitialGuess & _guess, const Limits2D & _limits, MincoOptimizeResult & _result);

  /// \brief 直接评估目标函数与解析梯度（供数值梯度检查使用）。
  ///
  /// 输入必须是完整的初值结构；输出 `_gradient` 的顺序与内部 L-BFGS 变量一致：
  /// 前 `2*(M-1)` 个是内部路标点的 x/y，随后 `M` 个是时间参数 `s`（`T = Tmin + softplus(s)`）。
  /// \return 失败返回 false。
  bool evaluateObjective(
    const TrajectoryInitialGuess & _guess, const Limits2D & _limits, double & _cost,
    Eigen::VectorXd & _gradient, Trajectory2D * _trajectory = nullptr);

  /// \brief 把“路标点 + 时长”编码为内部变量（供测试构造自变量）。
  Eigen::VectorXd encodeVariables(const TrajectoryInitialGuess & _guess) const;

private:
  /// \brief 目标函数内部状态。
  struct Problem
  {
    int pieces{0};
    Eigen::Matrix3d head{Eigen::Matrix3d::Zero()};
    Eigen::Matrix3d tail{Eigen::Matrix3d::Zero()};
    Eigen::Matrix3Xd inner{Eigen::MatrixXd::Zero(3, 0)};
    Eigen::VectorXd durations{Eigen::VectorXd::Zero(0)};
    Eigen::VectorXd time_parameters{Eigen::VectorXd::Zero(0)};
    std::vector<Eigen::Vector2d> reference_points{};
    const Limits2D * limits{nullptr};
    bool use_time_ratio_penalty{false};
    bool use_fine_heuristic{false};
  };

  /// \brief 评估一次目标函数：填充 `generator_` 与 `problem_`。
  double evaluate(const Eigen::VectorXd & _x, Eigen::VectorXd & _gradient, bool _need_gradient);

  /// \brief 由当前 `generator_` 生成 `Trajectory2D`。
  bool extractTrajectory(Trajectory2D & _trajectory) const;

  /// \brief 运行一次 L-BFGS。
  SolveStatus runStage(bool _pre_stage, Eigen::VectorXd & _x, double & _cost, int & _iterations);

  /// \brief 报告式 FINELY 通道启发式：按切向梯度剔除与方向探测缩放障碍梯度。
  ///
  /// 该函数只在 `enable_report_fine_heuristic` 为真时被调用，且会破坏
  /// “代价与梯度严格一致”的关系，属于**启发式**分支（方案 §6.5）。
  Eigen::Vector2d applyFineHeuristic(
    const Eigen::Vector2d & _position, const Eigen::Vector2d & _velocity,
    const Eigen::Vector2d & _gradient, double _distance) const;

  /// \brief L-BFGS 代价/梯度回调。
  static double evaluateCallback(
    void * _instance, const Eigen::VectorXd & _x, Eigen::VectorXd & _g);

  /// \brief L-BFGS 迭代进度回调，用于统计实际迭代次数。
  static int progressCallback(
    void * _instance, const Eigen::VectorXd & _x, const Eigen::VectorXd & _g, double _fx,
    double _step, int _k, int _ls);

  /// \brief 安全距离（包络半径 + 裕量，m）。
  double safetyDistance() const { return config_.robot_radius + config_.clearance_margin; }

  MincoOptimizerConfig config_{};
  std::shared_ptr<const Esdf2D> esdf_{};
  minco::MINCO_S3NU generator_{};
  Problem problem_{};
  std::vector<Eigen::Vector2d> warm_inner_points_{};
  std::vector<double> warm_durations_{};
  bool has_warm_start_{false};
  bool configured_{false};
  /// \brief 单次评估使用的约束快照（由 optimize/evaluateObjective 写入）。
  double limits_max_speed_sq_{0.0};
  double limits_max_accel_sq_{0.0};
  /// \brief 上一次 L-BFGS 实际执行的迭代次数。
  int last_iteration_{0};
  /// \brief 上一次 `runStage` 的耗时（ms），由 `optimize` 汇总到分项字段。
  double last_stage_time_ms_{0.0};
};

}  // namespace srm27_minco_core

#endif  // SRM27_MINCO_CORE__MINCO_OPTIMIZER_HPP_
