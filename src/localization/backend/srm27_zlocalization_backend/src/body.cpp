/**
 * @file body.cpp
 * @brief 使用固定外参转换传感器位姿与车体位姿。
 */

#include "backend/body.hpp"

#include "backend/math.hpp"

namespace backend {

/// 设置车体系偏移，并由度数欧拉角预计算传感器到车体旋转。
void BodyTransform::configure(const Eigen::Vector3d& offset, const Eigen::Vector3d& rpy_degrees) {
    offset_ = offset;
    sensor_to_body_rotation_ =
        rpyDegreesToRotation(rpy_degrees.x(), rpy_degrees.y(), rpy_degrees.z());
}

/// 将父坐标系中的传感器位姿转换为车体位姿。
Eigen::Matrix4d BodyTransform::bodyPoseFromSensorPose(
    const Eigen::Matrix4d& T_parent_sensor) const {
    Eigen::Matrix4d T_parent_body = T_parent_sensor;
    const Eigen::Matrix3d R_parent_sensor = T_parent_sensor.block<3, 3>(0, 0);
    const Eigen::Matrix3d R_parent_body = R_parent_sensor * sensor_to_body_rotation_.transpose();
    T_parent_body.block<3, 3>(0, 0) = R_parent_body;

    if (!offset_.isZero(1e-12)) {
        T_parent_body.block<3, 1>(0, 3) += R_parent_body * offset_;
    }
    return T_parent_body;
}

/// 将父坐标系中的车体位姿转换为传感器位姿。
Eigen::Matrix4d BodyTransform::sensorPoseFromBodyPose(const Eigen::Matrix4d& T_parent_body) const {
    Eigen::Matrix4d T_parent_sensor = T_parent_body;
    const Eigen::Matrix3d R_parent_body = T_parent_body.block<3, 3>(0, 0);
    // bodyPoseFromSensorPose() 的逆变换：R_parent_sensor = R_parent_body * R_body_sensor，
    // p_parent_sensor = p_parent_body - R_parent_body * offset_body。
    const Eigen::Matrix3d R_parent_sensor = R_parent_body * sensor_to_body_rotation_;
    T_parent_sensor.block<3, 3>(0, 0) = R_parent_sensor;

    if (!offset_.isZero(1e-12)) {
        T_parent_sensor.block<3, 1>(0, 3) -= R_parent_body * offset_;
    }
    return T_parent_sensor;
}

}  // namespace backend
