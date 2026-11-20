/**
 * @file gravity.hpp
 * @brief 定义只约束倾斜方向的重力因子，供图估计与解析导数验证共用。
 */
#pragma once

#include "backend/pose_graph.hpp"

namespace backend::detail {

class GravityAlignmentFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
public:
    using gtsam::NoiseModelFactor1<gtsam::Pose3>::evaluateError;

    /// 构造只约束倾斜方向、不提供全局偏航信息的重力因子。
    GravityAlignmentFactor(const gtsam::Key key, const gtsam::Rot3& R_odom_sensor,
                           const gtsam::SharedNoiseModel& noise_model)
        : gtsam::NoiseModelFactor1<gtsam::Pose3>(noise_model, key), R_odom_sensor_(R_odom_sensor) {}

    /// 计算测量残差，并在请求时写出各状态的解析 Jacobian。
    gtsam::Vector evaluateError(
        const gtsam::Pose3& T_map_sensor,
        boost::optional<gtsam::Matrix&> jacobian = boost::none) const override {
        gtsam::Matrix33 H_rotation;
        const gtsam::Vector3 gravity_sensor =
            T_map_sensor.rotation().unrotate(gtsam::Vector3::UnitZ(), H_rotation);
        if (jacobian) {
            *jacobian = gtsam::Matrix::Zero(3, 6);
            jacobian->leftCols<3>() = R_odom_sensor_.matrix() * H_rotation;
        }
        // C^T ez - ez；三维方向差的旋转 Jacobian 秩为 2，完全不含全局 yaw 信息。
        return R_odom_sensor_.rotate(gravity_sensor) - gtsam::Vector3::UnitZ();
    }

private:
    gtsam::Rot3 R_odom_sensor_;
};

}  // namespace backend::detail
