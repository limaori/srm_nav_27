/**
 * @file body_calibration.hpp
 * @brief 管理机体旋转与平面杆臂标定的样本、阶段和估计结果。
 */
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstddef>
#include <string>
#include <vector>

namespace backend::tools {

struct BodyCalibrationConfig {
    double sample_period = 0.05;
    double print_period = 1.0;
    double angle_sample_sec = 5.0;
    int angle_min_samples = 80;
    int offset_min_samples = 80;
    double offset_min_yaw_span_deg = 180.0;
    Eigen::Matrix3d target_body_rotation = Eigen::Matrix3d::Identity();
};

struct OffsetEstimate {
    Eigen::Vector2d center_map = Eigen::Vector2d::Zero();
    Eigen::Vector2d body_offset = Eigen::Vector2d::Zero();
    Eigen::Vector2d body_offset_std = Eigen::Vector2d::Zero();
    double yaw_span_deg = 0.0;
    double radius_mean = 0.0;
    double residual_rms = 0.0;
};

/// 只接收同步传感器位姿；通信、参数校验和结果展示由工具入口负责。
class BodyCalibration {
public:
    enum class Phase { Angle, Offset };
    enum class Update { None, AngleRejected, AngleCompleted, OffsetReady };

    /// 绑定已校验的采样策略和目标机体朝向，建立空标定状态。
    explicit BodyCalibration(const BodyCalibrationConfig& config);
    /// 记录同步水位并按采样周期推进当前阶段，返回需要展示的事件。
    Update addPose(double stamp, const Eigen::Matrix4d& t_map_sensor);
    /// 返回是否已收到同步位姿。
    bool receivedPose() const;
    /// 返回当前标定阶段。
    Phase phase() const;
    /// 返回角度阶段已接纳的样本数量。
    int angleSamples() const;
    /// 返回角度阶段从首样本到最近同步位姿的时长。
    double angleElapsed() const;
    /// 返回已经完成标定的传感器到机体旋转。
    const Eigen::Matrix3d& bodyRotation() const;
    /// 返回杆臂阶段已接纳的样本数量。
    std::size_t offsetSamples() const;
    /// 由二维旋转轨迹最小二乘估计车体旋转中心和传感器杆臂。
    bool estimateOffset(OffsetEstimate* estimate, std::string* reason) const;

private:
    struct OffsetSample {
        Eigen::Vector2d p_map_sensor = Eigen::Vector2d::Zero();
        Eigen::Matrix2d r_map_body_xy = Eigen::Matrix2d::Identity();
        double body_yaw = 0.0;
        double body_yaw_unwrapped = 0.0;
    };

    /// 在目标机体朝向下累计旋转样本，满足条件后切换到杆臂阶段。
    Update collectAngleSample(double stamp, const Eigen::Matrix4d& t_map_sensor);
    /// 在机体原地转动时累计轨迹样本，并按配置周期返回输出事件。
    Update collectOffsetSample(double stamp, const Eigen::Matrix4d& t_map_sensor);

    BodyCalibrationConfig config_;
    bool received_synced_pose_ = false;
    double last_synced_stamp_ = -1.0;
    double last_processed_stamp_ = -1.0;
    Phase phase_ = Phase::Angle;
    bool have_angle_ref_ = false;
    Eigen::Quaterniond angle_ref_q_ = Eigen::Quaterniond::Identity();
    Eigen::Vector4d angle_quat_sum_ = Eigen::Vector4d::Zero();
    int angle_samples_ = 0;
    double angle_first_stamp_ = -1.0;
    Eigen::Matrix3d r_body_sensor_ = Eigen::Matrix3d::Identity();
    std::vector<OffsetSample> offset_samples_;
    double last_offset_print_stamp_ = -1.0;
};

}  // namespace backend::tools
