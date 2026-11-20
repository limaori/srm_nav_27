/**
 * @file uwb_timing.cpp
 * @brief 构造连续时间天线观测及其不确定度。
 */

#include <algorithm>
#include <cmath>
#include <utility>

#include "backend/covariance.hpp"
#include "backend/math.hpp"
#include "backend/uwb_fusion.hpp"

namespace backend {

/// 在时间偏移后的测量时刻求取关键帧系天线位置及速度。
bool UwbTimeOffsetSupport::evaluate(const double time_offset_seconds,
                                    Eigen::Vector3d* antenna_in_keyframe,
                                    Eigen::Vector3d* velocity_in_keyframe) const {
    if (antenna_in_keyframe == nullptr || !std::isfinite(measurement_timestamp) ||
        !std::isfinite(time_offset_seconds) || !keyframe_odom_pose.allFinite() ||
        !antenna_position_sensor.allFinite() || odometry.size() < 2U) {
        return false;
    }
    for (std::size_t index = 0; index < odometry.size(); ++index) {
        if (!std::isfinite(odometry[index].timestamp) ||
            !odometry[index].T_odom_sensor.allFinite() ||
            (index > 0U && odometry[index].timestamp <= odometry[index - 1U].timestamp)) {
            return false;
        }
    }

    const double query_timestamp = measurement_timestamp + time_offset_seconds;
    if (!std::isfinite(query_timestamp)) {
        return false;
    }
    const auto right =
        std::lower_bound(odometry.begin(), odometry.end(), query_timestamp,
                         [](const TimedOdometryPose& sample, const double timestamp) {
                             return sample.timestamp < timestamp;
                         });
    std::size_t right_index = 1U;
    if (right == odometry.begin()) {
        right_index = 1U;
    } else if (right == odometry.end()) {
        right_index = odometry.size() - 1U;
    } else {
        right_index = static_cast<std::size_t>(std::distance(odometry.begin(), right));
    }
    const TimedOdometryPose& left_sample = odometry[right_index - 1U];
    const TimedOdometryPose& right_sample = odometry[right_index];
    const double span = right_sample.timestamp - left_sample.timestamp;
    if (!std::isfinite(span) || span <= 1.0e-9) {
        return false;
    }
    const double alpha = (query_timestamp - left_sample.timestamp) / span;

    Eigen::Quaterniond left_rotation(left_sample.T_odom_sensor.block<3, 3>(0, 0));
    Eigen::Quaterniond right_rotation(right_sample.T_odom_sensor.block<3, 3>(0, 0));
    Eigen::Quaterniond keyframe_rotation(keyframe_odom_pose.block<3, 3>(0, 0));
    if (!left_rotation.coeffs().allFinite() || !right_rotation.coeffs().allFinite() ||
        !keyframe_rotation.coeffs().allFinite() || left_rotation.norm() <= 1.0e-12 ||
        right_rotation.norm() <= 1.0e-12 || keyframe_rotation.norm() <= 1.0e-12) {
        return false;
    }
    left_rotation.normalize();
    right_rotation.normalize();
    keyframe_rotation.normalize();
    if (left_rotation.dot(right_rotation) < 0.0) {
        right_rotation.coeffs() *= -1.0;
    }
    Eigen::Quaterniond relative_rotation = left_rotation.conjugate() * right_rotation;
    relative_rotation.normalize();
    Eigen::AngleAxisd relative_angle_axis(relative_rotation);
    double angle = relative_angle_axis.angle();
    Eigen::Vector3d axis = relative_angle_axis.axis();
    if (!std::isfinite(angle) || !axis.allFinite()) {
        return false;
    }
    if (std::abs(angle) <= 1.0e-12) {
        angle = 0.0;
        axis = Eigen::Vector3d::UnitX();
    }
    const Eigen::Matrix3d rotation_odom_sensor =
        left_rotation.toRotationMatrix() *
        Eigen::AngleAxisd(alpha * angle, axis).toRotationMatrix();
    const Eigen::Vector3d position_left = left_sample.T_odom_sensor.block<3, 1>(0, 3);
    const Eigen::Vector3d position_right = right_sample.T_odom_sensor.block<3, 1>(0, 3);
    const Eigen::Vector3d position_odom_sensor =
        position_left + alpha * (position_right - position_left);
    const Eigen::Vector3d angular_velocity_sensor = axis * (angle / span);
    const Eigen::Vector3d velocity_odom_antenna =
        (position_right - position_left) / span +
        rotation_odom_sensor * angular_velocity_sensor.cross(antenna_position_sensor);
    const Eigen::Vector3d position_odom_antenna =
        position_odom_sensor + rotation_odom_sensor * antenna_position_sensor;

    const Eigen::Matrix3d R_keyframe_odom = keyframe_rotation.toRotationMatrix().transpose();
    *antenna_in_keyframe =
        R_keyframe_odom * (position_odom_antenna - keyframe_odom_pose.block<3, 1>(0, 3));
    if (velocity_in_keyframe != nullptr) {
        *velocity_in_keyframe = R_keyframe_odom * velocity_odom_antenna;
    }
    return antenna_in_keyframe->allFinite() &&
           (velocity_in_keyframe == nullptr || velocity_in_keyframe->allFinite());
}

/// 在物理测量时刻查询里程计并构造天线位置与速度支撑。
std::optional<UwbFactorBuilder::TimingProjection> UwbFactorBuilder::projectMeasurementTime(
    const UwbFactorBuildContext& context) const {
    const double motion_timestamp =
        context.observation.timestamp + context.mode.time_offset_seconds;
    const double keyframe_time_offset = context.keyframe.timestamp - motion_timestamp;
    if (!std::isfinite(motion_timestamp) || !std::isfinite(keyframe_time_offset)) {
        return std::nullopt;
    }

    const std::optional<Eigen::Matrix4d> timed_odom_pose =
        context.odometry.poseAt(motion_timestamp, config_.odom_sync_tolerance);
    if (!timed_odom_pose.has_value()) {
        return std::nullopt;
    }

    TimingProjection projection;
    projection.keyframe_time_offset = keyframe_time_offset;
    projection.T_keyframe_measurement = context.keyframe.odom_pose.inverse() * *timed_odom_pose;
    projection.antenna_in_keyframe =
        projection.T_keyframe_measurement.block<3, 3>(0, 0) * config_.antenna_position_sensor +
        projection.T_keyframe_measurement.block<3, 1>(0, 3);

    const bool model_timing_jitter = config_.time_jitter_std_seconds > 0.0;
    if (!context.mode.estimate_time_offset && !model_timing_jitter) {
        return projection;
    }
    if (context.odometry.size() < 2U ||
        (context.mode.estimate_time_offset && !context.mode.time_offset_state_available)) {
        return std::nullopt;
    }

    UwbTimeOffsetSupport support;
    support.measurement_timestamp = context.observation.timestamp;
    support.keyframe_odom_pose = context.keyframe.odom_pose;
    support.antenna_position_sensor = config_.antenna_position_sensor;
    support.odometry.reserve(context.odometry.size());
    for (const auto& [timestamp, pose] : context.odometry.samples()) {
        support.odometry.push_back({timestamp, pose});
    }

    Eigen::Vector3d antenna_from_support;
    Eigen::Vector3d velocity_in_keyframe = Eigen::Vector3d::Zero();
    if (!support.evaluate(context.mode.time_offset_seconds, &antenna_from_support,
                          model_timing_jitter ? &velocity_in_keyframe : nullptr)) {
        return std::nullopt;
    }
    if (context.mode.estimate_time_offset) {
        projection.antenna_in_keyframe = antenna_from_support;
        projection.time_support = std::move(support);
    }
    if (model_timing_jitter) {
        projection.residual_velocity =
            context.keyframe.optimized_pose.block<3, 3>(0, 0) * velocity_in_keyframe;
        if (context.mode.with_alignment) {
            projection.residual_velocity =
                context.T_map_uwb.block<3, 3>(0, 0).transpose() * projection.residual_velocity;
        }
        if (!projection.residual_velocity.allFinite()) {
            return std::nullopt;
        }
    }
    return projection;
}

/// 把关键帧与测量时刻间的过程噪声保守传播为位置方差。
std::optional<double> UwbFactorBuilder::propagatedPositionVariance(
    const TimingProjection& projection) const {
    const double propagation_time = std::abs(projection.keyframe_time_offset);
    const double propagation_distance = projection.T_keyframe_measurement.block<3, 1>(0, 3).norm();
    const double propagation_rotation = rotationAngleFromMatrix(projection.T_keyframe_measurement);
    const PoseCovariance6d covariance = config_.odometry_noise.covariance(
        propagation_time, propagation_distance, propagation_rotation);
    const double translation_variance = covariance.diagonal().head<3>().maxCoeff();
    const double rotation_variance = covariance.diagonal().tail<3>().maxCoeff();
    const double result =
        translation_variance + rotation_variance * config_.antenna_position_sensor.squaredNorm();
    if (!std::isfinite(result) || result < 0.0) {
        return std::nullopt;
    }
    return result;
}

/// 结合时间关联、天线杆臂及噪声传播构造 UWB 因子输入。
std::optional<UwbFactorInput> UwbFactorBuilder::build(const UwbFactorBuildContext& context) const {
    const std::optional<TimingProjection> projection = projectMeasurementTime(context);
    if (!projection.has_value()) {
        return std::nullopt;
    }

    UwbFactorInput input;
    input.timestamp = context.observation.timestamp;
    input.antenna_in_keyframe = projection->antenna_in_keyframe;
    input.time_support = projection->time_support;
    input.with_alignment = context.mode.with_alignment;
    if (input.with_alignment) {
        input.position = context.observation.position_uwb;
        input.covariance = config_.position_covariance;
    } else {
        input.position = positionMap(context.observation, context.T_map_uwb);
        input.covariance = positionCovarianceMap(context.T_map_uwb);
    }

    const double jitter_variance =
        config_.time_jitter_std_seconds * config_.time_jitter_std_seconds;
    if (!std::isfinite(jitter_variance)) {
        return std::nullopt;
    }
    input.covariance +=
        jitter_variance * projection->residual_velocity * projection->residual_velocity.transpose();

    const std::optional<double> propagation_variance = propagatedPositionVariance(*projection);
    if (!propagation_variance.has_value()) {
        return std::nullopt;
    }
    input.covariance.diagonal().array() += *propagation_variance;
    return input;
}

/// 将原始 UWB 天线位置变换到地图坐标系。
Eigen::Vector3d UwbFactorBuilder::positionMap(const UwbObservation& observation,
                                              const Eigen::Matrix4d& T_map_uwb) const {
    return T_map_uwb.block<3, 3>(0, 0) * observation.position_uwb + T_map_uwb.block<3, 1>(0, 3);
}

/// 将 UWB 解算坐标系中的位置协方差旋转到地图坐标系。
Eigen::Matrix3d UwbFactorBuilder::positionCovarianceMap(const Eigen::Matrix4d& T_map_uwb) const {
    const Eigen::Matrix3d R_map_uwb = T_map_uwb.block<3, 3>(0, 0);
    return R_map_uwb * config_.position_covariance * R_map_uwb.transpose();
}
}  // namespace backend
