/**
 * @file math.hpp
 * @brief 后端共用的角度和 SE(3) 数学工具。
 */
#pragma once

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <limits>

namespace backend {

/// 返回满足 skewMatrix(a)b=a×b 的反对称矩阵。
inline Eigen::Matrix3d skewMatrix(const Eigen::Vector3d& v) {
    Eigen::Matrix3d skew;
    skew << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
    return skew;
}

inline constexpr double kPi = 3.14159265358979323846;

/// 将角度值从度转换为弧度。
inline double degreesToRadians(const double deg) {
    return deg * kPi / 180.0;
}

/// 将角度值从弧度转换为度。
inline double radiansToDegrees(const double rad) {
    return rad * 180.0 / kPi;
}

/// 将弧度角归一化到 [-π, π]，保留两个端点的符号。
inline double normalizeAngle(const double angle) {
    if (!std::isfinite(angle))
        return std::numeric_limits<double>::quiet_NaN();
    double normalized = std::fmod(angle, 2.0 * kPi);
    if (normalized > kPi) {
        normalized -= 2.0 * kPi;
    }
    if (normalized < -kPi) {
        normalized += 2.0 * kPi;
    }
    return normalized;
}

/// 将度数角归一化到 [-180, 180)。
inline double normalizeDegrees(double deg) {
    if (!std::isfinite(deg))
        return std::numeric_limits<double>::quiet_NaN();
    deg = std::fmod(deg, 360.0);
    if (deg >= 180.0) {
        deg -= 360.0;
    }
    if (deg < -180.0) {
        deg += 360.0;
    }
    return deg;
}

/// 按 ZYX 顺序将度数欧拉角转换为旋转矩阵。
inline Eigen::Matrix3d rpyDegreesToRotation(const double roll_deg, const double pitch_deg,
                                            const double yaw_deg) {
    const Eigen::AngleAxisd roll(degreesToRadians(roll_deg), Eigen::Vector3d::UnitX());
    const Eigen::AngleAxisd pitch(degreesToRadians(pitch_deg), Eigen::Vector3d::UnitY());
    const Eigen::AngleAxisd yaw(degreesToRadians(yaw_deg), Eigen::Vector3d::UnitZ());
    return (yaw * pitch * roll).toRotationMatrix();
}

/// 从旋转矩阵提取 ZYX 欧拉角，奇异位置将偏航置零。
inline Eigen::Vector3d rpyFromRotation(const Eigen::Matrix3d& rotation) {
    const double pitch_arg = std::clamp(-rotation(2, 0), -1.0, 1.0);
    const double pitch = std::asin(pitch_arg);
    double roll = 0.0;
    double yaw = 0.0;
    if (std::abs(std::cos(pitch)) > 1e-8) {
        roll = std::atan2(rotation(2, 1), rotation(2, 2));
        yaw = std::atan2(rotation(1, 0), rotation(0, 0));
    } else {
        roll = std::atan2(-rotation(1, 2), rotation(1, 1));
        yaw = 0.0;
    }
    return Eigen::Vector3d(roll, pitch, yaw);
}

/// 返回旋转矩阵的 ZYX 偏航角，单位为弧度。
inline double yawFromRotation(const Eigen::Matrix3d& rotation) {
    return std::atan2(rotation(1, 0), rotation(0, 0));
}

/// 返回齐次变换中旋转部分的 ZYX 偏航角。
inline double yawFromMatrix(const Eigen::Matrix4d& transform) {
    return std::atan2(transform(1, 0), transform(0, 0));
}

/// 返回横滚与俯仰绝对值的较大者，单位为弧度。
inline double maxRollPitchFromRotation(const Eigen::Matrix3d& rotation) {
    const double roll = std::atan2(rotation(2, 1), rotation(2, 2));
    const double pitch = std::asin(std::clamp(-rotation(2, 0), -1.0, 1.0));
    return std::max(std::abs(roll), std::abs(pitch));
}

/// 返回相对旋转的主支转角，单位为弧度。
inline double rotationAngleFromMatrix(const Eigen::Matrix4d& transform) {
    const Eigen::Matrix3d rot = transform.block<3, 3>(0, 0);
    const double cos_angle = std::clamp((rot.trace() - 1.0) * 0.5, -1.0, 1.0);
    return std::acos(cos_angle);
}

/// 由偏航角和平移构造齐次位姿初值。
inline Eigen::Matrix4d yawGuess(const double yaw_rad, const Eigen::Vector3d& translation) {
    Eigen::Matrix4d guess = Eigen::Matrix4d::Identity();
    const double c = std::cos(yaw_rad);
    const double s = std::sin(yaw_rad);
    guess(0, 0) = c;
    guess(0, 1) = -s;
    guess(1, 0) = s;
    guess(1, 1) = c;
    guess.block<3, 1>(0, 3) = translation;
    return guess;
}

}  // namespace backend
