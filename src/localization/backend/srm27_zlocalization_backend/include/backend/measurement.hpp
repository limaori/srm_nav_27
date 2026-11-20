/**
 * @file measurement.hpp
 * @brief 定义位姿观测的平方根信息及其校验、转换接口。
 */
#pragma once

#include <Eigen/Core>
#include <limits>
#include <optional>
#include <string>

#include "backend/covariance.hpp"

namespace backend {

/// 位姿观测的 r × 6 平方根信息矩阵，列顺序为 [平移, 旋转]。
/// 行对应独立的白化残差方向，r 为观测秩。
using PoseSqrtInformation = Eigen::Matrix<double, Eigen::Dynamic, 6, Eigen::RowMajor>;

/// 位姿观测信息及其验收结果。
struct PoseMeasurementInformation {
    /// 白化残差对应的平方根信息矩阵。
    PoseSqrtInformation sqrt_information;
    /// 有效观测秩。
    int rank = 0;
    /// 白化时使用的残差方差。
    double residual_variance = std::numeric_limits<double>::infinity();
    /// 观测信息是否通过验收。
    bool valid = false;
    /// 观测信息无效时的原因。
    std::string invalid_reason;
};

/// 验证平方根信息的维度、有限性与白化残差行的独立性。
bool validatePoseSqrtInformation(const PoseSqrtInformation& sqrt_information,
                                 std::string* invalid_reason = nullptr);

/// 在同一观测切空间中叠加附加位姿协方差，返回更新后的平方根信息。
/// 输入或计算结果无效时返回 nullopt。
std::optional<PoseSqrtInformation> addPoseMeasurementCovariance(
    const PoseSqrtInformation& sqrt_information, const PoseCovariance6d& additional_covariance);

/// 将位姿观测的平方根信息从参考系左扰动转换为位姿局部右扰动。
PoseSqrtInformation leftToRightPoseSqrtInformation(
    const Eigen::Matrix4d& pose, const PoseSqrtInformation& left_sqrt_information);

/// 将列顺序从后端 [平移, 旋转] 重排为 GTSAM Pose3 的 [旋转, 平移]。
Eigen::MatrixXd backendToGtsamPoseSqrtInformation(
    const PoseSqrtInformation& backend_sqrt_information);

/// 将满秩平方根信息转换为同一切空间的位姿协方差。
/// 观测未满秩或转换失败时返回 nullopt。
std::optional<PoseCovariance6d> poseCovarianceFromSqrtInformation(
    const PoseSqrtInformation& sqrt_information);

}  // namespace backend
