/**
 * @file covariance.cpp
 * @brief 验收并传播六维位姿协方差，保持坐标顺序与完整耦合。
 */

#include "backend/covariance.hpp"

#include <Eigen/Dense>
#include <algorithm>

#include "backend/config.hpp"
#include "backend/math.hpp"

namespace backend {

/// 计算单段前端里程计增量的独立过程协方差。
PoseCovariance6d OdometryProcessNoiseConfig::covariance(const double delta_time,
                                                        const double translation_distance,
                                                        const double rotation_angle) const {
    constexpr double kVarianceFloor = 1.0e-12;
    const double dt = std::isfinite(delta_time) ? std::max(0.0, delta_time) : 0.0;
    const double distance =
        std::isfinite(translation_distance) ? std::max(0.0, translation_distance) : 0.0;
    const double angle = std::isfinite(rotation_angle) ? std::max(0.0, rotation_angle) : 0.0;
    const double translation_variance =
        std::max(kVarianceFloor,
                 translation_variance_per_second * dt + translation_variance_per_meter * distance);
    const double rotation_variance = std::max(
        kVarianceFloor, rotation_variance_per_second * dt + rotation_variance_per_radian * angle);

    PoseCovariance6d result = PoseCovariance6d::Zero();
    result.diagonal().head<3>().setConstant(translation_variance);
    result.diagonal().tail<3>().setConstant(rotation_variance);
    return result;
}

/// 校验协方差且绝不增加其信息量。明显非对称、非有限或不定的输入会被拒绝；只允许
/// 对称化舍入误差、抬升舍入量级负特征值以及增大过小的正特征值。
bool validateAndRegularizeCovariance(const Eigen::MatrixXd& input, Eigen::MatrixXd* output,
                                     std::string* invalid_reason, bool* regularized,
                                     const CovarianceValidationOptions& options) {
    /// 保存协方差验收失败原因并返回无效标志。
    const auto reject = [&](const std::string& reason) {
        if (invalid_reason != nullptr) {
            *invalid_reason = reason;
        }
        if (regularized != nullptr) {
            *regularized = false;
        }
        return false;
    };
    if (output == nullptr || input.rows() == 0 || input.rows() != input.cols()) {
        return reject("covariance must be a non-empty square matrix");
    }
    if (!input.allFinite()) {
        return reject("covariance contains a non-finite value");
    }

    const double magnitude = std::max(1.0e-30, input.cwiseAbs().maxCoeff());
    const double asymmetry = (input - input.transpose()).cwiseAbs().maxCoeff();
    const double symmetry_tolerance =
        options.absolute_symmetry_tolerance + options.relative_symmetry_tolerance * magnitude;
    if (asymmetry > symmetry_tolerance) {
        return reject("covariance is materially asymmetric");
    }

    Eigen::MatrixXd symmetric = 0.5 * (input + input.transpose());
    if ((symmetric.diagonal().array() > options.maximum_diagonal).any()) {
        return reject("covariance diagonal exceeds the supported range");
    }
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(symmetric);
    if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite()) {
        return reject("covariance eigendecomposition failed");
    }

    Eigen::VectorXd eigenvalues = solver.eigenvalues();
    const double largest = eigenvalues.maxCoeff();
    const double smallest = eigenvalues.minCoeff();
    if (!(largest > 0.0)) {
        return reject("covariance carries no positive uncertainty");
    }
    const double negative_tolerance = options.absolute_negative_eigenvalue_tolerance +
                                      options.relative_negative_eigenvalue_tolerance * largest;
    if (smallest < -negative_tolerance) {
        return reject("covariance is materially indefinite");
    }
    if (smallest == 0.0) {
        return reject("covariance is singular");
    }

    const double condition_floor = largest / std::max(1.0, options.maximum_condition_number);
    const double conservative_floor = std::max(options.minimum_eigenvalue, condition_floor);
    bool changed = asymmetry > 0.0;
    for (Eigen::Index index = 0; index < eigenvalues.size(); ++index) {
        if (eigenvalues(index) < conservative_floor) {
            eigenvalues(index) = conservative_floor;
            changed = true;
        }
    }
    *output = solver.eigenvectors() * eigenvalues.asDiagonal() * solver.eigenvectors().transpose();
    *output = 0.5 * (*output + output->transpose());
    if (!output->allFinite()) {
        return reject("covariance regularization produced a non-finite value");
    }
    if (invalid_reason != nullptr) {
        invalid_reason->clear();
    }
    if (regularized != nullptr) {
        *regularized = changed;
    }
    return true;
}

/// 验收六维位姿协方差并保守正则化，保持平移、旋转及交叉块。
bool validateAndRegularizePoseCovariance(const PoseCovariance6d& input, PoseCovariance6d* output,
                                         std::string* invalid_reason, bool* regularized) {
    if (output == nullptr) {
        if (invalid_reason != nullptr) {
            *invalid_reason = "output covariance pointer is null";
        }
        if (regularized != nullptr) {
            *regularized = false;
        }
        return false;
    }
    Eigen::MatrixXd normalized;
    if (!validateAndRegularizeCovariance(input, &normalized, invalid_reason, regularized)) {
        return false;
    }
    *output = normalized;
    return true;
}

/// 将 ROS 行主序数组恢复为平移、旋转顺序的六维协方差。
PoseCovariance6d poseCovarianceFromRos(const std::array<double, 36>& covariance) {
    PoseCovariance6d out;
    for (int r = 0; r < 6; ++r) {
        for (int c = 0; c < 6; ++c) {
            out(r, c) = covariance[static_cast<std::size_t>(r * 6 + c)];
        }
    }
    return out;
}

/// 返回缺少有效前端协方差时的保守位姿不确定性。
PoseCovariance6d fallbackFrontendPoseCovariance() {
    PoseCovariance6d covariance = PoseCovariance6d::Zero();
    constexpr double kTranslationStd = 0.05;      // m
    constexpr double kRotationStd = kPi / 180.0;  // rad
    for (int i = 0; i < 3; ++i) {
        covariance(i, i) = kTranslationStd * kTranslationStd;
    }
    for (int i = 3; i < 6; ++i) {
        covariance(i, i) = kRotationStd * kRotationStd;
    }
    return covariance;
}

/// 旋转位姿协方差的平移、旋转及交叉块。
PoseCovariance6d rotatePoseCovariance(const PoseCovariance6d& covariance,
                                      const Eigen::Matrix3d& rotation) {
    PoseCovariance6d J = PoseCovariance6d::Identity();
    J.block<3, 3>(0, 0) = rotation;
    J.block<3, 3>(3, 3) = rotation;
    return J * covariance * J.transpose();
}

/// 把 ROS 绝对位姿协方差转换为 GTSAM Pose3 使用的局部右扰动切空间。ROS 在位姿
/// header 坐标系表达位置加性误差和固定轴姿态误差，并非完整 SE(3) 左扰动，
/// 因此绝对平移不得引入杆臂项。
PoseCovariance6d worldAdditiveToLocalPoseCovariance(
    const Eigen::Matrix4d& world_pose, const PoseCovariance6d& world_additive_covariance) {
    PoseCovariance6d J = PoseCovariance6d::Zero();
    const Eigen::Matrix3d rotation_inverse = world_pose.block<3, 3>(0, 0).transpose();
    J.block<3, 3>(0, 0) = rotation_inverse;
    J.block<3, 3>(3, 3) = rotation_inverse;
    const PoseCovariance6d converted = J * world_additive_covariance * J.transpose();
    return 0.5 * (converted + converted.transpose());
}

/// 把 GTSAM Pose3 局部右扰动边缘协方差转换为位姿 world/header 坐标系中的 ROS
/// 位置加性误差与固定轴姿态协方差。
PoseCovariance6d localToWorldAdditivePoseCovariance(const Eigen::Matrix4d& world_pose,
                                                    const PoseCovariance6d& local_covariance) {
    PoseCovariance6d J = PoseCovariance6d::Zero();
    const Eigen::Matrix3d rotation = world_pose.block<3, 3>(0, 0);
    J.block<3, 3>(0, 0) = rotation;
    J.block<3, 3>(3, 3) = rotation;
    const PoseCovariance6d converted = J * local_covariance * J.transpose();
    return 0.5 * (converted + converted.transpose());
}

/// 将姿态与位置的不确定性传播到刚性连杆臂端点。
PoseCovariance6d propagateLeverArmCovariance(const PoseCovariance6d& covariance,
                                             const Eigen::Vector3d& lever_arm) {
    PoseCovariance6d J = PoseCovariance6d::Identity();
    J.block<3, 3>(0, 3) = -skewMatrix(lever_arm);
    return J * covariance * J.transpose();
}

/// 把重力对齐坐标变换的 [平移, 旋转] 协方差传播到 `position_alignment` 处位姿的
/// 世界加性协方差。旋转不确定度通过完整 roll/pitch/yaw 杆臂 Jacobian 影响位置；
/// 位姿偏离对齐原点时只保留 yaw 会低估位置不确定度。
PoseCovariance6d alignmentToPoseCovariance(const PoseCovariance6d& alignment_covariance,
                                           const Eigen::Matrix3d& R_world_alignment,
                                           const Eigen::Vector3d& position_alignment) {
    PoseCovariance6d J = PoseCovariance6d::Identity();
    J.block<3, 3>(0, 3) = -skewMatrix(R_world_alignment * position_alignment);
    const PoseCovariance6d propagated = J * alignment_covariance * J.transpose();
    return 0.5 * (propagated + propagated.transpose());
}

/// 后端切空间顺序 [平移, 旋转] 下的 SE(3) Adjoint。
PoseCovariance6d poseAdjoint(const Eigen::Matrix4d& transform) {
    PoseCovariance6d adjoint = PoseCovariance6d::Zero();
    const Eigen::Matrix3d rotation = transform.block<3, 3>(0, 0);
    const Eigen::Vector3d translation = transform.block<3, 1>(0, 3);
    adjoint.block<3, 3>(0, 0) = rotation;
    adjoint.block<3, 3>(0, 3) = skewMatrix(translation) * rotation;
    adjoint.block<3, 3>(3, 3) = rotation;
    return adjoint;
}

/// 把 `Exp(delta_left) * pose` 的协方差转换为 GTSAM 使用的局部右扰动
/// `pose * Exp(delta_right)`。
PoseCovariance6d leftToRightPoseCovariance(const Eigen::Matrix4d& pose,
                                           const PoseCovariance6d& covariance) {
    const PoseCovariance6d adjoint_inverse = poseAdjoint(pose.inverse());
    const PoseCovariance6d converted = adjoint_inverse * covariance * adjoint_inverse.transpose();
    return 0.5 * (converted + converted.transpose());
}

/// 将世界系位姿边缘协方差通过局部右扰动协方差描述的相对运动传播。结果是
/// `world_from * from_to` 在 ROS 位姿约定下的世界系位置加性与固定轴旋转协方差。
PoseCovariance6d propagatePoseWithRelativeCovariance(
    const Eigen::Matrix4d& world_from, const PoseCovariance6d& from_world_covariance,
    const Eigen::Matrix4d& world_to, const PoseCovariance6d& relative_local_covariance) {
    const Eigen::Vector3d baseline = world_to.block<3, 1>(0, 3) - world_from.block<3, 1>(0, 3);
    const Eigen::Matrix3d R_world_to = world_to.block<3, 3>(0, 0);

    PoseCovariance6d J_from = PoseCovariance6d::Identity();
    J_from.block<3, 3>(0, 3) = -skewMatrix(baseline);
    PoseCovariance6d J_relative = PoseCovariance6d::Zero();
    J_relative.block<3, 3>(0, 0) = R_world_to;
    J_relative.block<3, 3>(3, 3) = R_world_to;

    const PoseCovariance6d covariance =
        J_from * from_world_covariance * J_from.transpose() +
        J_relative * relative_local_covariance * J_relative.transpose();
    return 0.5 * (covariance + covariance.transpose());
}

/// 将六维协方差写为 ROS 行主序数组。
void writePoseCovariance(const PoseCovariance6d& covariance, std::array<double, 36>* out) {
    if (out == nullptr) {
        return;
    }
    for (int r = 0; r < 6; ++r) {
        for (int c = 0; c < 6; ++c) {
            (*out)[static_cast<std::size_t>(r * 6 + c)] = covariance(r, c);
        }
    }
}

/// 施加配置的平移与旋转方差下限。
void applyPoseCovarianceFloor(PoseCovariance6d* covariance, const double translation_std,
                              const double rotation_std) {
    if (covariance == nullptr) {
        return;
    }
    const double translation_var = translation_std * translation_std;
    const double rotation_var = rotation_std * rotation_std;
    for (int i = 0; i < 3; ++i) {
        (*covariance)(i, i) = std::max((*covariance)(i, i), translation_var);
    }
    for (int i = 3; i < 6; ++i) {
        (*covariance)(i, i) = std::max((*covariance)(i, i), rotation_var);
    }
}

}  // namespace backend
