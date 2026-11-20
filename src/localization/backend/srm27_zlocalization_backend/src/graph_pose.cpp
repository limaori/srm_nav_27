/**
 * @file graph_pose.cpp
 * @brief 实现先验、里程计、重力、绝对位姿和回环因子边界。
 */

#include <cmath>
#include <memory>
#include <stdexcept>

#include "backend/pose_graph.hpp"
#include "gravity.hpp"
#include "noise.hpp"

namespace backend {

using detail::GravityAlignmentFactor;
using detail::invalidInnovationTerm;
using detail::poseNoise;
using detail::priorVariancesGtsamOrder;
using detail::robustNoise;
using detail::rosToGtsamOrder;
using detail::strictCovarianceNoise;

namespace {

class ProjectedPosePriorFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
public:
    using gtsam::NoiseModelFactor1<gtsam::Pose3>::evaluateError;

    /// 构造按有效平方根信息投影的绝对位姿因子。
    ProjectedPosePriorFactor(const gtsam::Key key, const gtsam::Pose3& measurement,
                             const Eigen::MatrixXd& sqrt_information)
        : gtsam::NoiseModelFactor1<gtsam::Pose3>(
              gtsam::noiseModel::Unit::Create(static_cast<std::size_t>(sqrt_information.rows())),
              key),
          measurement_(measurement),
          sqrt_information_(sqrt_information) {}

    /// 计算测量残差，并在请求时写出各状态的解析 Jacobian。
    gtsam::Vector evaluateError(const gtsam::Pose3& pose, boost::optional<gtsam::Matrix&> jacobian =
                                                              boost::none) const override {
        if (jacobian) {
            gtsam::Matrix66 H_local;
            const gtsam::Vector6 local = measurement_.localCoordinates(pose, boost::none, H_local);
            *jacobian = sqrt_information_ * H_local;
            return sqrt_information_ * local;
        }
        return sqrt_information_ * measurement_.localCoordinates(pose);
    }

private:
    gtsam::Pose3 measurement_;
    Eigen::MatrixXd sqrt_information_;
};

class ProjectedPoseBetweenFactor : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3> {
public:
    using gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>::evaluateError;

    /// 构造按有效平方根信息投影的相对位姿因子。
    ProjectedPoseBetweenFactor(const gtsam::Key from_key, const gtsam::Key to_key,
                               const gtsam::Pose3& measurement,
                               const Eigen::MatrixXd& sqrt_information,
                               const gtsam::SharedNoiseModel& noise_model)
        : gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>(noise_model, from_key, to_key),
          measurement_(measurement),
          sqrt_information_(sqrt_information) {}

    /// 计算测量残差，并在请求时写出各状态的解析 Jacobian。
    gtsam::Vector evaluateError(
        const gtsam::Pose3& from, const gtsam::Pose3& to,
        boost::optional<gtsam::Matrix&> from_jacobian = boost::none,
        boost::optional<gtsam::Matrix&> to_jacobian = boost::none) const override {
        gtsam::Matrix66 H_from;
        gtsam::Matrix66 H_to;
        const gtsam::Pose3 predicted = from.between(to, H_from, H_to);
        gtsam::Matrix66 H_local;
        const gtsam::Vector6 local = measurement_.localCoordinates(predicted, boost::none, H_local);
        if (from_jacobian) {
            *from_jacobian = sqrt_information_ * H_local * H_from;
        }
        if (to_jacobian) {
            *to_jacobian = sqrt_information_ * H_local * H_to;
        }
        return sqrt_information_ * local;
    }

private:
    gtsam::Pose3 measurement_;
    Eigen::MatrixXd sqrt_information_;
};

/// 构造带里程计保守回退噪声的相对位姿因子。
gtsam::BetweenFactor<gtsam::Pose3> odometryFactor(const PoseGraphConfig& config, int from_index,
                                                  int to_index,
                                                  const Eigen::Matrix4d& measured_between,
                                                  const PoseCovariance6d& covariance,
                                                  const char* context) {
    const PoseCovariance6d fallback_covariance = config.odom_noise.covariance(1.0, 0.0, 0.0);
    Eigen::Matrix<double, 6, 1> fallback_gtsam_variances;
    fallback_gtsam_variances << fallback_covariance.diagonal().tail<3>(),
        fallback_covariance.diagonal().head<3>();
    return gtsam::BetweenFactor<gtsam::Pose3>(
        from_index, to_index,
        gtsam::Pose3(
            gtsam::Rot3(measured_between.block<3, 3>(0, 0)),
            gtsam::Point3(measured_between(0, 3), measured_between(1, 3), measured_between(2, 3))),
        poseNoise(covariance, fallback_gtsam_variances, context));
}

}  // namespace

/// 添加位姿先验并登记首个状态及其时间戳。
void PoseGraph::addPrior(const int index, const Eigen::Matrix4d& pose, const double timestamp) {
    auto noise = gtsam::noiseModel::Diagonal::Variances(priorVariancesGtsamOrder(config_));
    pending_graph_.add(gtsam::PriorFactor<gtsam::Pose3>(index, toGtsam(pose), noise));
    recordNewState(index, pose, timestamp);
}

/// 添加位姿先验并登记首个状态及其时间戳。
void PoseGraph::addPrior(const int index, const Eigen::Matrix4d& pose,
                         const PoseCovariance6d& covariance, const double timestamp) {
    pending_graph_.add(gtsam::PriorFactor<gtsam::Pose3>(
        index, toGtsam(pose),
        poseNoise(covariance, priorVariancesGtsamOrder(config_), "pose prior")));
    recordNewState(index, pose, timestamp);
}

/// 添加同代里程计相对位姿测量并登记目标状态。
void PoseGraph::addOdometryMeasurement(const int from_index, const int to_index,
                                       const Eigen::Matrix4d& measured_between,
                                       const Eigen::Matrix4d& initial_to_pose,
                                       const PoseCovariance6d& covariance, const double timestamp) {
    pending_graph_.add(
        odometryFactor(config_, from_index, to_index, measured_between, covariance, "odometry"));
    recordNewState(to_index, initial_to_pose, timestamp);
}

/// 添加跨重置连续性测量并登记新代目标状态。
void PoseGraph::addResetContinuityMeasurement(const int from_index, const int to_index,
                                              const Eigen::Matrix4d& measured_between,
                                              const Eigen::Matrix4d& initial_to_pose,
                                              const PoseCovariance6d& covariance,
                                              const double timestamp) {
    pending_graph_.add(odometryFactor(config_, from_index, to_index, measured_between, covariance,
                                      "reset continuity"));
    recordNewState(to_index, initial_to_pose, timestamp);
}

/// 验证里程计旋转并添加只约束倾斜方向的重力因子。
void PoseGraph::addGravityAlignment(const int index, const Eigen::Matrix4d& T_odom_sensor,
                                    const double standard_deviation_rad) {
    if (!T_odom_sensor.allFinite() || !std::isfinite(standard_deviation_rad) ||
        standard_deviation_rad <= 0.0) {
        throw std::invalid_argument("gravity alignment pose/noise is invalid");
    }
    const Eigen::Matrix3d rotation = T_odom_sensor.block<3, 3>(0, 0);
    const Eigen::Matrix3d orthogonality =
        rotation.transpose() * rotation - Eigen::Matrix3d::Identity();
    if (!orthogonality.allFinite() || orthogonality.norm() > 1.0e-5 ||
        std::abs(rotation.determinant() - 1.0) > 1.0e-5) {
        throw std::invalid_argument("gravity alignment odometry rotation is not in SO(3)");
    }
    pending_graph_.add(gtsam::make_shared<GravityAlignmentFactor>(
        index, gtsam::Rot3(rotation),
        gtsam::noiseModel::Isotropic::Sigma(3, standard_deviation_rad)));
}

/// 添加已经通过完整位姿验收的全局锚定观测。
void PoseGraph::addAbsolutePose(const int index, const Eigen::Matrix4d& measured_pose,
                                const PoseCovariance6d& covariance) {
    // 完整位姿恢复观测已通过假设验收，此处直接按其协方差添加先验因子。
    const auto noise = strictCovarianceNoise(rosToGtsamOrder(covariance), "scan-to-map absolute");
    pending_graph_.add(gtsam::PriorFactor<gtsam::Pose3>(index, toGtsam(measured_pose), noise));
}

/// 在已提交预测图上白化并线性化绝对位姿观测。
InnovationTerm PoseGraph::linearizeAbsolutePose(const int index,
                                                const Eigen::Matrix4d& measured_pose,
                                                const PoseSqrtInformation& sqrt_information) const {
    std::string information_error;
    if (!measured_pose.allFinite() ||
        !validatePoseSqrtInformation(sqrt_information, &information_error)) {
        return invalidInnovationTerm(information_error.empty()
                                         ? "absolute pose measurement is non-finite"
                                         : information_error);
    }
    if (pending_graph_.size() != 0U || !pending_values_.empty()) {
        return invalidInnovationTerm("innovation assessment requires a committed prediction graph");
    }
    if (!isam_->valueExists(index)) {
        return invalidInnovationTerm("pose index is unavailable for innovation assessment");
    }

    const Eigen::MatrixXd L = backendToGtsamPoseSqrtInformation(sqrt_information);
    const ProjectedPosePriorFactor factor(index, toGtsam(measured_pose), L);
    gtsam::Matrix G;
    const gtsam::Vector whitened_innovation =
        factor.evaluateError(isam_->calculateEstimate<gtsam::Pose3>(index), G);
    if (!whitened_innovation.allFinite() || !G.allFinite() || G.rows() != sqrt_information.rows() ||
        G.cols() != 6) {
        return invalidInnovationTerm("innovation residual/Jacobian is invalid");
    }

    InnovationTerm term;
    term.valid = true;
    term.keys = {static_cast<gtsam::Key>(index)};
    term.whitened_residual = whitened_innovation;
    term.whitened_jacobians = {G};
    return term;
}

/// 添加允许降秩的绝对位姿平方根信息因子。
void PoseGraph::addAbsolutePoseInformation(const int index, const Eigen::Matrix4d& measured_pose,
                                           const PoseSqrtInformation& sqrt_information) {
    std::string invalid_reason;
    if (!measured_pose.allFinite() ||
        !validatePoseSqrtInformation(sqrt_information, &invalid_reason)) {
        throw std::invalid_argument(invalid_reason.empty()
                                        ? "absolute pose information measurement is non-finite"
                                        : invalid_reason);
    }
    pending_graph_.add(gtsam::make_shared<ProjectedPosePriorFactor>(
        index, toGtsam(measured_pose), backendToGtsamPoseSqrtInformation(sqrt_information)));
}

/// 在已提交预测图上白化并线性化相对位姿观测。
InnovationTerm PoseGraph::linearizeRelativePose(const int from_index, const int to_index,
                                                const Eigen::Matrix4d& measured_between,
                                                const PoseSqrtInformation& sqrt_information) const {
    std::string information_error;
    if (!measured_between.allFinite() ||
        !validatePoseSqrtInformation(sqrt_information, &information_error)) {
        return invalidInnovationTerm(information_error.empty()
                                         ? "relative pose measurement is non-finite"
                                         : information_error);
    }
    if (pending_graph_.size() != 0U || !pending_values_.empty()) {
        return invalidInnovationTerm("innovation assessment requires a committed prediction graph");
    }
    if (from_index == to_index || !isam_->valueExists(from_index) ||
        !isam_->valueExists(to_index)) {
        return invalidInnovationTerm("relative pose indices are duplicate or unavailable");
    }

    const Eigen::MatrixXd L = backendToGtsamPoseSqrtInformation(sqrt_information);
    const auto unit = gtsam::noiseModel::Unit::Create(static_cast<std::size_t>(L.rows()));
    const ProjectedPoseBetweenFactor factor(from_index, to_index, toGtsam(measured_between), L,
                                            unit);
    gtsam::Matrix H_from;
    gtsam::Matrix H_to;
    const gtsam::Vector residual =
        factor.evaluateError(isam_->calculateEstimate<gtsam::Pose3>(from_index),
                             isam_->calculateEstimate<gtsam::Pose3>(to_index), H_from, H_to);
    if (!residual.allFinite() || !H_from.allFinite() || !H_to.allFinite()) {
        return invalidInnovationTerm("relative innovation residual/Jacobian is invalid");
    }
    InnovationTerm term;
    term.valid = true;
    term.keys = {static_cast<gtsam::Key>(from_index), static_cast<gtsam::Key>(to_index)};
    term.whitened_residual = residual;
    term.whitened_jacobians = {H_from, H_to};
    return term;
}

/// 添加保留可观方向和完整耦合的鲁棒回环因子。
void PoseGraph::addLoopInformation(const int from_index, const int to_index,
                                   const Eigen::Matrix4d& measured_between,
                                   const PoseSqrtInformation& sqrt_information) {
    std::string invalid_reason;
    if (!measured_between.allFinite() ||
        !validatePoseSqrtInformation(sqrt_information, &invalid_reason)) {
        throw std::invalid_argument(
            invalid_reason.empty() ? "loop information measurement is non-finite" : invalid_reason);
    }
    const Eigen::MatrixXd L = backendToGtsamPoseSqrtInformation(sqrt_information);
    const auto noise =
        robustNoise(gtsam::noiseModel::Unit::Create(static_cast<std::size_t>(L.rows())), config_);
    pending_graph_.add(gtsam::make_shared<ProjectedPoseBetweenFactor>(
        from_index, to_index, toGtsam(measured_between), L, noise));
}

}  // namespace backend
