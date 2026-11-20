/**
 * @file graph_calibration.cpp
 * @brief 实现 UWB 坐标对齐和共享时间偏置状态的初始化。
 */

#include <cmath>
#include <memory>
#include <stdexcept>

#include "backend/pose_graph.hpp"
#include "noise.hpp"

namespace backend {

using detail::alignmentPriorCovarianceLocal;
using detail::covarianceNoiseOrFallback;
using detail::planarAlignmentFromMatrix;
namespace {

class UwbAlignmentPriorFactor : public gtsam::NoiseModelFactor2<gtsam::Pose2, double> {
public:
    using gtsam::NoiseModelFactor2<gtsam::Pose2, double>::evaluateError;

    /// 构造保留平面对齐与高度相关性的联合先验因子。
    UwbAlignmentPriorFactor(const gtsam::Key planar_key, const gtsam::Key z_key,
                            const gtsam::Pose2& planar_measurement, const double z_measurement,
                            const gtsam::SharedNoiseModel& noise_model)
        : gtsam::NoiseModelFactor2<gtsam::Pose2, double>(noise_model, planar_key, z_key),
          planar_measurement_(planar_measurement),
          z_measurement_(z_measurement) {}

    /// 计算测量残差，并在请求时写出各状态的解析 Jacobian。
    gtsam::Vector evaluateError(
        const gtsam::Pose2& planar, const double& z,
        boost::optional<gtsam::Matrix&> planar_jacobian = boost::none,
        boost::optional<gtsam::Matrix&> z_jacobian = boost::none) const override {
        gtsam::Matrix33 H_local;
        const gtsam::Vector3 planar_error =
            planar_measurement_.localCoordinates(planar, boost::none, H_local);
        if (planar_jacobian) {
            planar_jacobian->setZero(4, 3);
            planar_jacobian->topRows<3>() = H_local;
        }
        if (z_jacobian) {
            z_jacobian->setZero(4, 1);
            (*z_jacobian)(3, 0) = 1.0;
        }
        gtsam::Vector4 error;
        error.head<3>() = planar_error;
        error(3) = z - z_measurement_;
        return error;
    }

private:
    gtsam::Pose2 planar_measurement_;
    double z_measurement_ = 0.0;
};

}  // namespace

/// 添加共享空间对齐先验及其初始值，拒绝重复初始化。
void PoseGraph::addUwbAlignmentPrior(const Eigen::Matrix4d& T_map_uwb,
                                     const PoseCovariance6d& covariance, const bool use_z) {
    if (uwb_alignment_initialized_) {
        throw std::logic_error("UWB alignment prior already exists");
    }
    const gtsam::Pose2 planar = planarAlignmentFromMatrix(T_map_uwb);
    const gtsam::Key planar_key = uwbAlignmentKey();
    const gtsam::Matrix local_covariance = alignmentPriorCovarianceLocal(covariance, planar, use_z);
    const auto noise = covarianceNoiseOrFallback(
        local_covariance, gtsam::Matrix::Identity(use_z ? 4 : 3, use_z ? 4 : 3),
        "UWB alignment prior");
    if (use_z) {
        const gtsam::Key z_key = uwbAlignmentZKey();
        pending_graph_.add(gtsam::make_shared<UwbAlignmentPriorFactor>(planar_key, z_key, planar,
                                                                       T_map_uwb(2, 3), noise));
        pending_values_.insert(z_key, T_map_uwb(2, 3));
    } else {
        pending_graph_.add(gtsam::PriorFactor<gtsam::Pose2>(planar_key, planar, noise));
    }
    pending_values_.insert(planar_key, planar);
    uwb_alignment_initialized_ = true;
    uwb_alignment_use_z_ = use_z;
    uwb_alignment_fixed_z_ = T_map_uwb(2, 3);
}

/// 添加共享空间对齐初始值而不注入额外先验信息。
void PoseGraph::addUwbAlignmentInitialValue(const Eigen::Matrix4d& T_map_uwb, const bool use_z) {
    if (uwb_alignment_initialized_) {
        throw std::logic_error("UWB alignment state already exists");
    }
    const gtsam::Pose2 planar = planarAlignmentFromMatrix(T_map_uwb);
    const gtsam::Key planar_key = uwbAlignmentKey();
    pending_values_.insert(planar_key, planar);
    if (use_z) {
        pending_values_.insert(uwbAlignmentZKey(), T_map_uwb(2, 3));
    }
    uwb_alignment_initialized_ = true;
    uwb_alignment_use_z_ = use_z;
    uwb_alignment_fixed_z_ = T_map_uwb(2, 3);
}

/// 添加以秒为单位的共享时间偏置先验及初始值。
void PoseGraph::addUwbTimeOffsetPrior(const double initial_seconds,
                                      const double standard_deviation_seconds) {
    if (uwb_time_offset_initialized_) {
        throw std::logic_error("UWB time-offset prior already exists");
    }
    if (!std::isfinite(initial_seconds) || !std::isfinite(standard_deviation_seconds) ||
        standard_deviation_seconds <= 0.0) {
        throw std::invalid_argument("UWB time-offset prior mean/noise is invalid");
    }
    const gtsam::Key key = uwbTimeOffsetKey();
    pending_graph_.add(gtsam::PriorFactor<double>(
        key, initial_seconds, gtsam::noiseModel::Isotropic::Sigma(1, standard_deviation_seconds)));
    pending_values_.insert(key, initial_seconds);
    uwb_time_offset_initialized_ = true;
}

}  // namespace backend
