/**
 * @file pose_graph.hpp
 * @brief 声明位姿图更新、观测线性化和状态查询接口。
 */
#pragma once

#include <gtsam/base/make_shared.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "backend/factors.hpp"
#include "backend/measurement.hpp"
#include "backend/types.hpp"

namespace backend {
/// 增量位姿图的数值求解失败异常。
class NumericalError final : public std::runtime_error {
public:
    /// 保存线性系统求解失败的具体原因。
    explicit NumericalError(const std::string& reason) : std::runtime_error(reason) {}
};

/// 管理增量位姿图的因子、状态估计和协方差查询。
class PoseGraph {
public:
    /// 构造增量位姿图并初始化固定窗口与噪声配置。
    explicit PoseGraph(const PoseGraphConfig& config = PoseGraphConfig());

    /// 清空当前图并应用新配置。
    void reset(const PoseGraphConfig& config);

    /// 使用配置的先验方差添加位姿先验，并登记节点初值和时间戳。
    void addPrior(int index, const Eigen::Matrix4d& pose,
                  double timestamp = std::numeric_limits<double>::quiet_NaN());
    /// 使用给定协方差添加位姿先验，并登记节点初值和时间戳。
    /// 协方差采用该位姿的局部右扰动坐标，顺序为 [平移, 旋转]。
    void addPrior(int index, const Eigen::Matrix4d& pose, const PoseCovariance6d& covariance,
                  double timestamp = std::numeric_limits<double>::quiet_NaN());

    /// 添加前端相对位姿观测，以 initial_to_pose 登记目标节点初值。
    /// 协方差采用测量的局部右扰动坐标，顺序为 [平移, 旋转]。
    void addOdometryMeasurement(int from_index, int to_index,
                                const Eigen::Matrix4d& measured_between,
                                const Eigen::Matrix4d& initial_to_pose,
                                const PoseCovariance6d& covariance,
                                double timestamp = std::numeric_limits<double>::quiet_NaN());

    /// 添加跨前端重置的相对位姿约束，并登记新代次的目标节点。
    /// 协方差采用测量的局部右扰动坐标，顺序为 [平移, 旋转]。
    void addResetContinuityMeasurement(int from_index, int to_index,
                                       const Eigen::Matrix4d& measured_between,
                                       const Eigen::Matrix4d& initial_to_pose,
                                       const PoseCovariance6d& covariance,
                                       double timestamp = std::numeric_limits<double>::quiet_NaN());

    /// 基于前端重力方向添加横滚、俯仰约束，标准差单位为 rad。
    void addGravityAlignment(int index, const Eigen::Matrix4d& T_odom_sensor,
                             double standard_deviation_rad);

    /// 为已有节点添加完整协方差的绝对位姿观测。
    /// 协方差采用测量的局部右扰动坐标，顺序为 [平移, 旋转]。
    void addAbsolutePose(int index, const Eigen::Matrix4d& measured_pose,
                         const PoseCovariance6d& covariance);

    /// 在已提交图上计算绝对位姿观测的白化残差与雅可比。
    /// 平方根信息采用测量的局部右扰动坐标，列顺序为 [平移, 旋转]。
    InnovationTerm linearizeAbsolutePose(int index, const Eigen::Matrix4d& measured_pose,
                                         const PoseSqrtInformation& sqrt_information) const;
    /// 添加可降秩的绝对位姿因子，平方根信息采用测量局部右扰动的 [平移, 旋转] 列序。
    void addAbsolutePoseInformation(int index, const Eigen::Matrix4d& measured_pose,
                                    const PoseSqrtInformation& sqrt_information);
    /// 在已提交图上计算相对位姿观测的白化残差与雅可比。
    /// 平方根信息采用测量的局部右扰动坐标，列顺序为 [平移, 旋转]。
    InnovationTerm linearizeRelativePose(int from_index, int to_index,
                                         const Eigen::Matrix4d& measured_between,
                                         const PoseSqrtInformation& sqrt_information) const;
    /// 添加可降秩的相对位姿回环因子，并按配置应用鲁棒核。
    /// 平方根信息采用测量的局部右扰动坐标，列顺序为 [平移, 旋转]。
    void addLoopInformation(int from_index, int to_index, const Eigen::Matrix4d& measured_between,
                            const PoseSqrtInformation& sqrt_information);

    /// 初始化共享的 UWB 到 map 空间对齐状态，并添加外部先验。
    /// 状态包含水平平移与航向，use_z=true 时加入高度；协方差采用世界加性
    /// [平移, 固定轴旋转] 顺序，提取对应状态分量。
    void addUwbAlignmentPrior(const Eigen::Matrix4d& T_map_uwb, const PoseCovariance6d& covariance,
                              bool use_z = true);

    /// 登记水平对齐与可选高度的共享初值，随后与对应观测因子一同提交。
    void addUwbAlignmentInitialValue(const Eigen::Matrix4d& T_map_uwb, bool use_z = true);

    /// 添加共享 UWB 时间偏置的先验及初值，单位为 s。
    /// 运动时刻 = 消息时间戳 + 时间偏置。
    void addUwbTimeOffsetPrior(double initial_seconds, double standard_deviation_seconds);

    /// 添加 UWB 天线位置观测；use_z=false 使用 XY，true 使用 XYZ。
    void addUwbPosition(int index, const UwbMeasurement& measurement, bool use_z = true);
    /// 在已提交图上计算 UWB 白化残差及位姿、共享时空状态的雅可比。
    InnovationTerm linearizeUwbPosition(int index, const UwbMeasurement& measurement,
                                        bool use_z = true) const;

    /// 基于更新前图协方差计算一批观测的联合及逐项创新检验结果。
    /// 调用时待提交因子与初值为空，各观测项来自同一图版本。
    InnovationBatchAssessment assessInnovationBatch(const std::vector<InnovationTerm>& terms) const;

    /// 提交待处理因子和初值，执行增量求解。
    /// 含全局因子时返回全部活动位姿，否则返回本批新增位姿。
    /// 线性系统求解失败时清空图并抛出 NumericalError。
    std::vector<std::pair<int, Eigen::Matrix4d>> update(bool global_factor_added);

    /// 判断指定关键帧是否仍有可查询的图估计。
    bool hasEstimate(int index) const;
    /// 查询指定关键帧的当前图位姿，状态不存在时返回空值。
    std::optional<Eigen::Matrix4d> estimate(int index) const;
    /// 判断共享 UWB 空间对齐状态是否已初始化且可查询。
    bool hasUwbAlignment() const;
    /// 返回 UWB 坐标系到 map 系的当前变换。
    std::optional<Eigen::Matrix4d> uwbAlignmentEstimate() const;
    /// 返回空间对齐的世界加性边缘协方差，顺序为 [平移, 固定轴旋转]。
    /// 未参与估计的分量行列为零。
    std::optional<PoseCovariance6d> uwbAlignmentMarginalCovariance() const;
    /// 判断共享 UWB 时间偏置状态是否可查询。
    bool hasUwbTimeOffset() const;
    /// 返回共享 UWB 时间偏置估计，单位为秒。
    std::optional<double> uwbTimeOffsetEstimate() const;
    /// 返回共享 UWB 时间偏置的边缘方差，单位为秒平方。
    std::optional<double> uwbTimeOffsetMarginalVariance() const;
    /// 返回位姿局部右扰动坐标下的边缘协方差，顺序为 [平移, 旋转]。
    std::optional<PoseCovariance6d> marginalCovariance(int index) const;

private:
    /// 检查位置、连杆臂及共享时空状态，返回不可用原因。
    std::string uwbInputError(const UwbMeasurement& measurement, bool use_z) const;
    /// 按高度、空间对齐和时间支撑组合构造对应的 UWB 因子。
    gtsam::NoiseModelFactor::shared_ptr makeUwbFactor(int index, const UwbMeasurement& measurement,
                                                      bool use_z,
                                                      const gtsam::SharedNoiseModel& noise) const;
    /// 将后端齐次位姿转换为 GTSAM Pose3。
    static gtsam::Pose3 toGtsam(const Eigen::Matrix4d& pose);
    /// 将 GTSAM Pose3 转换为后端齐次位姿。
    static Eigen::Matrix4d fromGtsam(const gtsam::Pose3& pose);
    /// 返回共享 UWB 平面对齐状态的图变量键。
    static gtsam::Key uwbAlignmentKey();
    /// 返回共享 UWB 高度对齐状态的图变量键。
    static gtsam::Key uwbAlignmentZKey();
    /// 返回共享 UWB 时间偏置状态的图变量键。
    static gtsam::Key uwbTimeOffsetKey();
    /// 登记新位姿初值、时间戳和待提交的状态编号。
    void recordNewState(int index, const Eigen::Matrix4d& pose, double timestamp);
    /// 更新固定滞后窗口并边缘化过期轨迹位姿。
    void updateWindow();
    /// 按给定键顺序和变量切空间维数查询联合边缘协方差。
    std::optional<Eigen::MatrixXd> jointMarginalCovariance(
        const std::vector<gtsam::Key>& keys, const std::vector<Eigen::Index>& dimensions) const;
    /// 返回当前活动窗口内所有位姿的估计。
    std::vector<std::pair<int, Eigen::Matrix4d>> activeEstimates() const;

    /// 位姿图的噪声、鲁棒核与窗口配置。
    PoseGraphConfig config_;
    /// 当前增量求解器。
    std::unique_ptr<gtsam::ISAM2> isam_;
    /// 本批待提交的观测因子。
    gtsam::NonlinearFactorGraph pending_graph_;
    /// 本批新图变量的初值。
    gtsam::Values pending_values_;
    /// 固定滞后窗口中的轨迹节点键及其单调图时间，s。
    std::deque<std::pair<gtsam::Key, double>> pose_timestamps_;
    /// 本批新增的轨迹节点编号。
    std::vector<int> pending_new_indices_;
    /// 是否已登记共享空间对齐初值。
    bool uwb_alignment_initialized_ = false;
    /// 空间对齐状态是否包含高度变量。
    bool uwb_alignment_use_z_ = false;
    /// 二维对齐模式沿用的固定高度，m。
    double uwb_alignment_fixed_z_ = 0.0;
    /// 是否已登记共享时间偏置初值。
    bool uwb_time_offset_initialized_ = false;
    /// 输入时间转换为单调图时间所使用的偏移，s。
    double timestamp_offset_ = 0.0;
    /// 最近一次登记的有效输入时间戳，s。
    double last_input_timestamp_ = std::numeric_limits<double>::quiet_NaN();
    /// 最近一次登记的单调图时间，s。
    double latest_graph_timestamp_ = std::numeric_limits<double>::quiet_NaN();
};

}  // namespace backend
