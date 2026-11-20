/**
 * @file body.hpp
 * @brief 定义传感器与车体固定外参及双向位姿转换。
 */
#pragma once

#include <Eigen/Dense>

namespace backend {

class BodyTransform {
public:
    /// 设置车体系偏移，并由度数欧拉角预计算传感器到车体旋转。
    void configure(const Eigen::Vector3d& offset, const Eigen::Vector3d& rpy_degrees);

    /// 将父坐标系中的传感器位姿转换为车体位姿。
    Eigen::Matrix4d bodyPoseFromSensorPose(const Eigen::Matrix4d& T_parent_sensor) const;
    /// 将父坐标系中的车体位姿转换为传感器位姿。
    Eigen::Matrix4d sensorPoseFromBodyPose(const Eigen::Matrix4d& T_parent_body) const;

    /// 返回传感器原点到车体原点的车体系偏移。
    const Eigen::Vector3d& offset() const { return offset_; }

private:
    Eigen::Vector3d offset_ = Eigen::Vector3d::Zero();
    Eigen::Matrix3d sensor_to_body_rotation_ = Eigen::Matrix3d::Identity();
};

}  // namespace backend
