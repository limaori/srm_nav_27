/**
 * @file body_calibration.cpp
 * @brief 累计同步姿态，分阶段估计机体旋转外参与平面杆臂。
 */
#include "body_calibration.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

#include "backend/math.hpp"

namespace backend::tools {

/// 绑定已校验的采样策略和目标机体朝向，建立空标定状态。
BodyCalibration::BodyCalibration(const BodyCalibrationConfig& config) : config_(config) {
}

/// 记录同步水位并按采样周期推进当前阶段，返回需要展示的事件。
BodyCalibration::Update BodyCalibration::addPose(double stamp,
                                                 const Eigen::Matrix4d& t_map_sensor) {
    received_synced_pose_ = true;
    last_synced_stamp_ = stamp;
    if (last_processed_stamp_ >= 0.0 && stamp - last_processed_stamp_ < config_.sample_period)
        return Update::None;
    last_processed_stamp_ = stamp;
    return phase_ == Phase::Angle ? collectAngleSample(stamp, t_map_sensor)
                                  : collectOffsetSample(stamp, t_map_sensor);
}

/// 在目标机体朝向下累计旋转样本，满足条件后切换到杆臂阶段。
BodyCalibration::Update BodyCalibration::collectAngleSample(double stamp,
                                                            const Eigen::Matrix4d& t_map_sensor) {
    if (angle_first_stamp_ < 0.0)
        angle_first_stamp_ = stamp;
    const Eigen::Matrix3d r_map_sensor = t_map_sensor.block<3, 3>(0, 0);
    const Eigen::Matrix3d r_body_sensor = config_.target_body_rotation.transpose() * r_map_sensor;
    Eigen::Quaterniond q_body_sensor(r_body_sensor);
    q_body_sensor.normalize();
    if (!have_angle_ref_) {
        angle_ref_q_ = q_body_sensor;
        have_angle_ref_ = true;
    }
    Eigen::Vector4d coeffs = q_body_sensor.coeffs();
    if (angle_ref_q_.coeffs().dot(coeffs) < 0.0)
        coeffs = -coeffs;
    angle_quat_sum_ += coeffs;
    angle_samples_ += 1;
    const double elapsed = stamp - angle_first_stamp_;
    if (!(angle_samples_ >= config_.angle_min_samples && elapsed >= config_.angle_sample_sec))
        return Update::None;
    if (angle_quat_sum_.norm() <= 1e-9)
        return Update::AngleRejected;

    Eigen::Quaterniond q_avg;
    q_avg.coeffs() = angle_quat_sum_.normalized();
    q_avg.normalize();
    r_body_sensor_ = q_avg.toRotationMatrix();
    phase_ = Phase::Offset;
    offset_samples_.clear();
    last_offset_print_stamp_ = -1.0;
    return Update::AngleCompleted;
}

/// 在机体原地转动时累计轨迹样本，并按配置周期返回输出事件。
BodyCalibration::Update BodyCalibration::collectOffsetSample(double stamp,
                                                             const Eigen::Matrix4d& t_map_sensor) {
    const Eigen::Matrix3d r_map_sensor = t_map_sensor.block<3, 3>(0, 0);
    const Eigen::Matrix3d r_map_body = r_map_sensor * r_body_sensor_.transpose();
    OffsetSample sample;
    sample.p_map_sensor = t_map_sensor.block<2, 1>(0, 3);
    sample.r_map_body_xy = r_map_body.block<2, 2>(0, 0);
    sample.body_yaw = backend::yawFromRotation(r_map_body);
    if (offset_samples_.empty()) {
        sample.body_yaw_unwrapped = sample.body_yaw;
    } else {
        const auto& prev = offset_samples_.back();
        sample.body_yaw_unwrapped =
            prev.body_yaw_unwrapped + backend::normalizeAngle(sample.body_yaw - prev.body_yaw);
    }
    offset_samples_.push_back(sample);
    if (last_offset_print_stamp_ < 0.0 ||
        stamp - last_offset_print_stamp_ >= config_.print_period) {
        last_offset_print_stamp_ = stamp;
        return Update::OffsetReady;
    }
    return Update::None;
}

/// 返回是否已收到同步位姿。
bool BodyCalibration::receivedPose() const {
    return received_synced_pose_;
}
/// 返回当前标定阶段。
BodyCalibration::Phase BodyCalibration::phase() const {
    return phase_;
}
/// 返回角度阶段已接纳的样本数量。
int BodyCalibration::angleSamples() const {
    return angle_samples_;
}
/// 返回角度阶段从首样本到最近同步位姿的时长。
double BodyCalibration::angleElapsed() const {
    return angle_first_stamp_ >= 0.0 ? std::max(0.0, last_synced_stamp_ - angle_first_stamp_) : 0.0;
}
/// 返回已经完成标定的传感器到机体旋转。
const Eigen::Matrix3d& BodyCalibration::bodyRotation() const {
    return r_body_sensor_;
}
/// 返回杆臂阶段已接纳的样本数量。
std::size_t BodyCalibration::offsetSamples() const {
    return offset_samples_.size();
}

/// 由二维旋转轨迹最小二乘估计车体旋转中心和传感器杆臂。
bool BodyCalibration::estimateOffset(OffsetEstimate* estimate, std::string* reason) const {
    if (static_cast<int>(offset_samples_.size()) < config_.offset_min_samples) {
        if (reason) {
            *reason = "waiting for more samples";
        }
        return false;
    }

    double min_yaw = std::numeric_limits<double>::infinity();
    double max_yaw = -std::numeric_limits<double>::infinity();
    for (const auto& sample : offset_samples_) {
        min_yaw = std::min(min_yaw, sample.body_yaw_unwrapped);
        max_yaw = std::max(max_yaw, sample.body_yaw_unwrapped);
    }
    const double yaw_span_deg = backend::radiansToDegrees(max_yaw - min_yaw);
    if (yaw_span_deg < config_.offset_min_yaw_span_deg) {
        if (reason) {
            std::ostringstream oss;
            oss << "waiting for larger yaw span (" << yaw_span_deg << "/"
                << config_.offset_min_yaw_span_deg << " deg)";
            *reason = oss.str();
        }
        return false;
    }

    // 以 p_sensor = center - R_body * offset 联合拟合平面旋转中心和杆臂。
    const int n = static_cast<int>(offset_samples_.size());
    Eigen::MatrixXd design(2 * n, 4);
    Eigen::VectorXd b(2 * n);
    for (int i = 0; i < n; ++i) {
        const auto& sample = offset_samples_[i];
        const Eigen::Matrix2d& rotation = sample.r_map_body_xy;
        design(2 * i, 0) = 1.0;
        design(2 * i, 1) = 0.0;
        design(2 * i, 2) = -rotation(0, 0);
        design(2 * i, 3) = -rotation(0, 1);
        b(2 * i) = sample.p_map_sensor.x();

        design(2 * i + 1, 0) = 0.0;
        design(2 * i + 1, 1) = 1.0;
        design(2 * i + 1, 2) = -rotation(1, 0);
        design(2 * i + 1, 3) = -rotation(1, 1);
        b(2 * i + 1) = sample.p_map_sensor.y();
    }

    const Eigen::Vector4d sol = design.colPivHouseholderQr().solve(b);
    if (!sol.allFinite()) {
        if (reason) {
            *reason = "least-squares solve failed";
        }
        return false;
    }

    const Eigen::Vector2d center_map(sol.x(), sol.y());
    const Eigen::Vector2d body_offset(sol.z(), sol.w());

    double residual_sum_sq = 0.0;
    double radius_sum = 0.0;
    std::vector<Eigen::Vector2d> per_sample_offsets;
    per_sample_offsets.reserve(offset_samples_.size());
    for (const auto& sample : offset_samples_) {
        const Eigen::Vector2d predicted_sensor = center_map - sample.r_map_body_xy * body_offset;
        const Eigen::Vector2d residual = predicted_sensor - sample.p_map_sensor;
        residual_sum_sq += residual.squaredNorm();
        radius_sum += (sample.p_map_sensor - center_map).norm();

        const Eigen::FullPivLU<Eigen::Matrix2d> lu(sample.r_map_body_xy);
        if (lu.isInvertible()) {
            per_sample_offsets.push_back(lu.solve(center_map - sample.p_map_sensor));
        }
    }

    // 用可逆姿态样本的逐样本杆臂离散程度报告估计稳定性。
    Eigen::Vector2d offset_std = Eigen::Vector2d::Zero();
    if (!per_sample_offsets.empty()) {
        Eigen::Vector2d mean = Eigen::Vector2d::Zero();
        for (const auto& offset : per_sample_offsets) {
            mean += offset;
        }
        mean /= static_cast<double>(per_sample_offsets.size());
        Eigen::Vector2d variance = Eigen::Vector2d::Zero();
        for (const auto& offset : per_sample_offsets) {
            const Eigen::Vector2d diff = offset - mean;
            variance += diff.cwiseProduct(diff);
        }
        variance /= static_cast<double>(per_sample_offsets.size());
        offset_std.x() = std::sqrt(std::max(0.0, variance.x()));
        offset_std.y() = std::sqrt(std::max(0.0, variance.y()));
    }

    estimate->center_map = center_map;
    estimate->body_offset = body_offset;
    estimate->body_offset_std = offset_std;
    estimate->yaw_span_deg = yaw_span_deg;
    estimate->radius_mean = radius_sum / static_cast<double>(n);
    estimate->residual_rms = std::sqrt(residual_sum_sq / static_cast<double>(n));
    return true;
}

}  // namespace backend::tools
