/**
 * @file measurement.cpp
 * @brief 验收并转换降秩位姿测量信息，保留可观方向与完整耦合。
 */

#include "backend/measurement.hpp"

#include <Eigen/Dense>

namespace backend {

/// 验证平方根信息的维度、有限性与白化残差行的独立性。
bool validatePoseSqrtInformation(const PoseSqrtInformation& sqrt_information,
                                 std::string* invalid_reason) {
    /// 保存平方根信息验收失败原因并返回无效标志。
    const auto reject = [&](const char* reason) {
        if (invalid_reason != nullptr) {
            *invalid_reason = reason;
        }
        return false;
    };
    if (sqrt_information.cols() != 6 || sqrt_information.rows() <= 0 ||
        sqrt_information.rows() > 6) {
        return reject("sqrt-information must be r x 6 with 1 <= r <= 6");
    }
    if (!sqrt_information.allFinite()) {
        return reject("sqrt-information contains a non-finite value");
    }
    const Eigen::MatrixXd gram = sqrt_information * sqrt_information.transpose();
    const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(0.5 * (gram + gram.transpose()));
    if (solver.info() != Eigen::Success || solver.eigenvalues().minCoeff() <= 1.0e-12) {
        return reject("sqrt-information rows are not independent");
    }
    return true;
}

/// 给已白化的可能降秩位姿观测叠加同一切空间中的共模协方差。
/// 若 L 为原平方根信息、C 为同一切空间中的附加位姿协方差，则新信息为
/// L^T (I + L C L^T)^-1 L。该形式保留原信息的零空间与可观方向耦合。
std::optional<PoseSqrtInformation> addPoseMeasurementCovariance(
    const PoseSqrtInformation& sqrt_information, const PoseCovariance6d& additional_covariance) {
    if (!validatePoseSqrtInformation(sqrt_information) || !additional_covariance.allFinite()) {
        return std::nullopt;
    }
    const PoseCovariance6d covariance =
        0.5 * (additional_covariance + additional_covariance.transpose());
    const Eigen::SelfAdjointEigenSolver<PoseCovariance6d> covariance_solver(covariance);
    if (covariance_solver.info() != Eigen::Success ||
        covariance_solver.eigenvalues().minCoeff() < -1.0e-12) {
        return std::nullopt;
    }

    Eigen::MatrixXd residual_covariance =
        Eigen::MatrixXd::Identity(sqrt_information.rows(), sqrt_information.rows()) +
        sqrt_information * covariance * sqrt_information.transpose();
    residual_covariance = 0.5 * (residual_covariance + residual_covariance.transpose());
    const Eigen::LLT<Eigen::MatrixXd> solver(residual_covariance);
    if (solver.info() != Eigen::Success) {
        return std::nullopt;
    }
    const Eigen::MatrixXd inverse_lower = solver.matrixL().solve(
        Eigen::MatrixXd::Identity(sqrt_information.rows(), sqrt_information.rows()));
    PoseSqrtInformation result = inverse_lower * sqrt_information;
    if (!validatePoseSqrtInformation(result)) {
        return std::nullopt;
    }
    return result;
}

/// 配准更新在 target 坐标系左乘，而 GTSAM Pose3 因子对测量位姿使用局部右扰动。
/// 当 delta_left = Ad(T) delta_right 时，L_right = L_left Ad(T)。
PoseSqrtInformation leftToRightPoseSqrtInformation(
    const Eigen::Matrix4d& pose, const PoseSqrtInformation& left_sqrt_information) {
    return left_sqrt_information * poseAdjoint(pose);
}

/// 把列从后端 [平移, 旋转] 重排为 GTSAM Pose3 [旋转,
/// 平移]；残差行空间与白化不变。
Eigen::MatrixXd backendToGtsamPoseSqrtInformation(
    const PoseSqrtInformation& backend_sqrt_information) {
    Eigen::MatrixXd gtsam_sqrt_information(backend_sqrt_information.rows(), 6);
    gtsam_sqrt_information.leftCols<3>() = backend_sqrt_information.rightCols<3>();
    gtsam_sqrt_information.rightCols<3>() = backend_sqrt_information.leftCols<3>();
    return gtsam_sqrt_information;
}

/// 将满秩平方根信息转换为位姿协方差；观测未满秩或转换失败时返回 nullopt。
std::optional<PoseCovariance6d> poseCovarianceFromSqrtInformation(
    const PoseSqrtInformation& sqrt_information) {
    if (sqrt_information.rows() != 6 || !validatePoseSqrtInformation(sqrt_information)) {
        return std::nullopt;
    }
    const PoseCovariance6d information = sqrt_information.transpose() * sqrt_information;
    Eigen::LDLT<PoseCovariance6d> solver(0.5 * (information + information.transpose()));
    if (solver.info() != Eigen::Success || (solver.vectorD().array() <= 1.0e-12).any()) {
        return std::nullopt;
    }
    PoseCovariance6d covariance = solver.solve(PoseCovariance6d::Identity());
    covariance = 0.5 * (covariance + covariance.transpose());
    PoseCovariance6d normalized;
    if (!covariance.allFinite() || !validateAndRegularizePoseCovariance(covariance, &normalized)) {
        return std::nullopt;
    }
    return normalized;
}

}  // namespace backend
