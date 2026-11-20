/**
 * @file factors.hpp
 * @brief 定义图观测输入、UWB 时间关联数据与创新检验结果。
 */
#pragma once
#include <gtsam/inference/Key.h>

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "backend/config.hpp"
#include "backend/measurement.hpp"

namespace backend {
/// 单项或联合观测的创新检验结果。
struct InnovationAssessment {
    /// 检验结果是否有效。
    bool valid = false;
    /// 卡方检验的自由度。
    int degrees_of_freedom = 0;
    /// 归一化创新平方（NIS）。
    double nis = std::numeric_limits<double>::infinity();
    /// 卡方分布的上尾概率。
    double upper_tail_probability = 0.0;
    /// 检验结果无效时的原因。
    std::string invalid_reason;
};

/// 外部观测在已提交图预测点处的白化残差与雅可比。
struct InnovationTerm {
    /// 观测项是否已成功线性化与白化。
    bool valid = false;
    /// 观测涉及的图变量键，与雅可比块顺序一致。
    std::vector<gtsam::Key> keys;
    /// 白化残差，对应的测量噪声协方差为单位阵。
    Eigen::VectorXd whitened_residual;
    /// 各图变量的白化雅可比；列数为其切空间维数：Pose3 为 6、Pose2 为 3、标量为 1。
    std::vector<Eigen::MatrixXd> whitened_jacobians;
    /// 观测项无效时的原因。
    std::string invalid_reason;
};

/// UWB 时间插值使用的前端里程计样本。
struct TimedOdometryPose {
    /// 里程计采样时刻，s。
    double timestamp = 0.0;
    /// 传感器在 odom 坐标系中的位姿。
    Eigen::Matrix4d T_odom_sensor = Eigen::Matrix4d::Identity();
};

/// UWB 时间偏置求值使用的局部轨迹与天线外参。
struct UwbTimeOffsetSupport {
    /// UWB 原始消息时间戳，s。
    double measurement_timestamp = 0.0;
    /// 关联关键帧的传感器在 odom 系中的位姿。
    Eigen::Matrix4d keyframe_odom_pose = Eigen::Matrix4d::Identity();
    /// 天线相位中心在传感器坐标系中的位置，m。
    Eigen::Vector3d antenna_position_sensor = Eigen::Vector3d::Zero();
    /// 按时间递增排列的里程计样本。
    std::vector<TimedOdometryPose> odometry;

    /// 在消息时间戳加偏置的时刻，计算关键帧传感器系中的天线位置及可选速度。
    /// 区间外按最近区间外推；输入或计算结果无效时返回 false。
    bool evaluate(double time_offset_seconds, Eigen::Vector3d* antenna_in_keyframe,
                  Eigen::Vector3d* velocity_in_keyframe = nullptr) const;
};

/// UWB 位置因子的观测、协方差与时空对齐输入。
struct UwbMeasurement {
    /// 天线位置观测，m；固定对齐时为 map 系，在线对齐时为 UWB 原生系。
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    /// 测量时刻的天线在关联关键帧传感器系中的位置，m。
    Eigen::Vector3d antenna_in_keyframe = Eigen::Vector3d::Zero();
    /// 位置观测协方差，与 position 使用相同坐标系，m²。
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Identity();
    /// 是否在因子中联合估计 UWB 到 map 的坐标对齐。
    bool with_alignment = false;
    /// 在线时间偏置计算数据；空值时使用 antenna_in_keyframe。
    std::optional<UwbTimeOffsetSupport> time_support = std::nullopt;
    /// UWB 原始消息时间戳，s。
    double timestamp = 0.0;
};

/// 同一批观测基于更新前图协方差得到的创新检验结果。
struct InnovationBatchAssessment {
    /// 全部观测的联合检验结果。
    InnovationAssessment joint;
    /// 各观测的逐项检验结果，顺序与输入观测项一致。
    std::vector<InnovationAssessment> individual;
};

/// 将逐分量白化残差的 RMS 阈值换算为 GTSAM 块鲁棒核的残差范数阈值。
inline double robustBlockKernelSize(const double per_component_rms, const std::size_t dimension) {
    return per_component_rms * std::sqrt(static_cast<double>(std::max<std::size_t>(1, dimension)));
}

}  // namespace backend
