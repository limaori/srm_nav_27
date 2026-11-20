/**
 * @file graph_noise.cpp
 * @brief 实现测量协方差验收、鲁棒噪声和创新白化。
 */

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "backend/covariance.hpp"
#include "backend/logger.hpp"
#include "noise.hpp"

namespace backend::detail {

/// 将具名先验方差转换为 GTSAM 的旋转、平移顺序。
Eigen::Matrix<double, 6, 1> priorVariancesGtsamOrder(const PoseGraphConfig& config) {
    Eigen::Matrix<double, 6, 1> variances;
    variances << config.prior_rotation_variances, config.prior_translation_variances;
    return variances;
}

/// 将位姿协方差转换为 GTSAM 的旋转、平移顺序。
gtsam::Matrix rosToGtsamOrder(const PoseCovariance6d& covariance) {
    gtsam::Matrix out = gtsam::Matrix::Zero(6, 6);
    out.block<3, 3>(0, 0) = covariance.block<3, 3>(3, 3);
    out.block<3, 3>(0, 3) = covariance.block<3, 3>(3, 0);
    out.block<3, 3>(3, 0) = covariance.block<3, 3>(0, 3);
    out.block<3, 3>(3, 3) = covariance.block<3, 3>(0, 0);
    return out;
}

/// 验证平面对齐变换并提取其 SE(2) 状态。
gtsam::Pose2 planarAlignmentFromMatrix(const Eigen::Matrix4d& T_map_uwb) {
    if (!T_map_uwb.allFinite() ||
        !T_map_uwb.row(3).isApprox(Eigen::RowVector4d(0.0, 0.0, 0.0, 1.0), 1.0e-9)) {
        throw std::invalid_argument("UWB alignment transform is non-finite or non-homogeneous");
    }
    const Eigen::Matrix3d rotation = T_map_uwb.block<3, 3>(0, 0);
    if (!(rotation.transpose() * rotation).isApprox(Eigen::Matrix3d::Identity(), 1.0e-9) ||
        std::abs(rotation.determinant() - 1.0) > 1.0e-9) {
        throw std::invalid_argument("UWB alignment transform does not contain a valid rotation");
    }
    if ((rotation * Eigen::Vector3d::UnitZ() - Eigen::Vector3d::UnitZ()).norm() > 1.0e-9) {
        throw std::invalid_argument(
            "UWB alignment is SE(2)+optional-Z and cannot contain roll/pitch");
    }
    return gtsam::Pose2(T_map_uwb(0, 3), T_map_uwb(1, 3), yawFromRotation(rotation));
}

/// 将世界加性对齐协方差转换到共享状态的局部坐标。
gtsam::Matrix alignmentPriorCovarianceLocal(const PoseCovariance6d& world_covariance,
                                            const gtsam::Pose2& planar_alignment,
                                            const bool use_z) {
    const std::vector<int> indices =
        use_z ? std::vector<int>{0, 1, 5, 2} : std::vector<int>{0, 1, 5};
    const Eigen::Index dimension = static_cast<Eigen::Index>(indices.size());
    gtsam::Matrix reduced = gtsam::Matrix::Zero(dimension, dimension);
    if (!world_covariance.allFinite()) {
        reduced.setConstant(std::numeric_limits<double>::quiet_NaN());
        return reduced;
    }
    for (Eigen::Index row = 0; row < dimension; ++row) {
        for (Eigen::Index col = 0; col < dimension; ++col) {
            reduced(row, col) = world_covariance(indices[static_cast<std::size_t>(row)],
                                                 indices[static_cast<std::size_t>(col)]);
        }
    }
    gtsam::Matrix J = gtsam::Matrix::Identity(dimension, dimension);
    J.topLeftCorner<2, 2>() = planar_alignment.rotation().matrix().transpose();
    const gtsam::Matrix local = J * reduced * J.transpose();
    return 0.5 * (local + local.transpose());
}

/// 验收测量协方差，无效时使用显式保守回退噪声。
gtsam::SharedNoiseModel covarianceNoiseOrFallback(const gtsam::Matrix& covariance,
                                                  const gtsam::Matrix& fallback,
                                                  const char* context,
                                                  const bool zero_means_missing) {
    const bool missing =
        zero_means_missing && covariance.allFinite() && covariance.cwiseAbs().maxCoeff() <= 1.0e-15;
    Eigen::MatrixXd normalized;
    std::string invalid_reason;
    bool regularized = false;
    if (missing ||
        !validateAndRegularizeCovariance(covariance, &normalized, &invalid_reason, &regularized)) {
        if (!missing) {
            BACKEND_WARN_STREAM(context << " covariance rejected (" << invalid_reason
                                        << "); using conservative fallback");
        }
        normalized = fallback;
    } else if (regularized) {
        BACKEND_DEBUG_STREAM(context << " covariance received conservative numerical "
                                        "regularization");
    }
    return gtsam::noiseModel::Gaussian::Covariance(normalized);
}

/// 转换位姿协方差顺序并构造带保守回退的噪声模型。
gtsam::SharedNoiseModel poseNoise(const PoseCovariance6d& covariance,
                                  const Eigen::Matrix<double, 6, 1>& fallback_gtsam_variances,
                                  const char* context) {
    return covarianceNoiseOrFallback(rosToGtsamOrder(covariance),
                                     fallback_gtsam_variances.asDiagonal(), context,
                                     /*zero_means_missing=*/true);
}

/// 严格验收协方差并构造噪声模型，无效输入抛出异常。
gtsam::SharedNoiseModel strictCovarianceNoise(const gtsam::Matrix& covariance,
                                              const char* context) {
    Eigen::MatrixXd normalized;
    std::string invalid_reason;
    bool regularized = false;
    if (!validateAndRegularizeCovariance(covariance, &normalized, &invalid_reason, &regularized)) {
        throw std::invalid_argument(std::string(context) +
                                    " covariance rejected: " + invalid_reason);
    }
    if (regularized) {
        BACKEND_DEBUG_STREAM(context << " covariance received conservative numerical "
                                        "regularization");
    }
    return gtsam::noiseModel::Gaussian::Covariance(normalized);
}

/// 按残差维数和配置构造块鲁棒噪声模型。
gtsam::SharedNoiseModel robustNoise(const gtsam::SharedNoiseModel& base,
                                    const PoseGraphConfig& config) {
    if (!config.use_robust_kernel) {
        return base;
    }

    // GTSAM 默认块重加权使用完整白化残差范数；把 RMS 阈值乘以维数平方根，
    // 使同一配置值在二维、三维和六维因子中具有一致统计含义。
    const double block_size = robustBlockKernelSize(config.robust_kernel_size, base->dim());
    const std::string kernel = config.robust_kernel;
    if (kernel == "cauchy") {
        return gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Cauchy::Create(block_size), base);
    }
    if (kernel == "huber") {
        return gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Huber::Create(block_size), base);
    }
    return base;
}

/// 构造包含拒绝原因的无效创新项。
InnovationTerm invalidInnovationTerm(const std::string& reason) {
    InnovationTerm term;
    term.invalid_reason = reason;
    return term;
}

/// 按测量协方差白化残差和各图变量的 Jacobian。
InnovationTerm makeWhitenedInnovationTerm(std::vector<gtsam::Key> keys,
                                          const Eigen::VectorXd& residual,
                                          std::vector<Eigen::MatrixXd> jacobians,
                                          const Eigen::MatrixXd& measurement_covariance,
                                          const std::string& covariance_context) {
    if (keys.empty() || keys.size() != jacobians.size() || residual.size() <= 0 ||
        !residual.allFinite()) {
        return invalidInnovationTerm("innovation term dimensions or residual are invalid");
    }
    for (std::size_t index = 0; index < jacobians.size(); ++index) {
        if (jacobians[index].rows() != residual.size() || jacobians[index].cols() <= 0 ||
            !jacobians[index].allFinite()) {
            return invalidInnovationTerm("innovation term Jacobian is invalid");
        }
        if (std::find(keys.begin(), keys.begin() + static_cast<std::ptrdiff_t>(index),
                      keys[index]) != keys.begin() + static_cast<std::ptrdiff_t>(index)) {
            return invalidInnovationTerm("innovation term contains a duplicate graph key");
        }
    }

    Eigen::MatrixXd normalized_covariance;
    std::string covariance_error;
    if (!validateAndRegularizeCovariance(measurement_covariance, &normalized_covariance,
                                         &covariance_error)) {
        return invalidInnovationTerm(covariance_context +
                                     " covariance rejected: " + covariance_error);
    }
    Eigen::LLT<Eigen::MatrixXd> solver(normalized_covariance);
    if (solver.info() != Eigen::Success) {
        return invalidInnovationTerm(covariance_context + " covariance whitening failed");
    }

    InnovationTerm term;
    term.keys = std::move(keys);
    term.whitened_residual = solver.matrixL().solve(residual);
    term.whitened_jacobians.reserve(jacobians.size());
    for (const Eigen::MatrixXd& jacobian : jacobians) {
        term.whitened_jacobians.push_back(solver.matrixL().solve(jacobian));
    }
    term.valid = term.whitened_residual.allFinite();
    for (const Eigen::MatrixXd& jacobian : term.whitened_jacobians) {
        term.valid = term.valid && jacobian.allFinite();
    }
    if (!term.valid) {
        term.invalid_reason = covariance_context + " whitening produced non-finite values";
    }
    return term;
}

}  // namespace backend::detail
