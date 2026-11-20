/**
 * @file calibrate.cpp
 * @brief 采集时间对齐的定位位姿，分阶段估计机体旋转外参与平面杆臂。
 */

#include <tf2/exceptions.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <builtin_interfaces/msg/time.hpp>
#include <chrono>
#include <cmath>
#include <functional>
#include <geometry_msgs/msg/pose.hpp>
#include <iostream>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include "backend/math.hpp"
#include "body_calibration.hpp"

namespace backend::tools {
namespace {

/// 将 ROS 秒和纳秒时间字段转换为浮点秒。
double stampToSec(const builtin_interfaces::msg::Time& stamp) {
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

/// 归一化消息四元数并构造齐次传感器位姿。
Eigen::Matrix4d poseToMatrix(const geometry_msgs::msg::Pose& pose) {
    Eigen::Matrix4d out = Eigen::Matrix4d::Identity();
    Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y,
                         pose.orientation.z);
    q.normalize();
    out.block<3, 3>(0, 0) = q.toRotationMatrix();
    out(0, 3) = pose.position.x;
    out(1, 3) = pose.position.y;
    out(2, 3) = pose.position.z;
    return out;
}

/// 检查齐次位姿矩阵的全部元素是否有限。
bool matrixFinite(const Eigen::Matrix4d& m) {
    return m.array().isFinite().all();
}

struct PoseSample {
    double stamp = 0.0;
    rclcpp::Time ros_stamp{0, 0, RCL_ROS_TIME};
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
};

}  // namespace

class BodyCalibrationNode : public rclcpp::Node {
public:
    /// 读取采样策略，建立 TF/里程计输入并启动两阶段标定。
    BodyCalibrationNode() : Node("body_calibration_node") {
        map_frame_ = declare_parameter<std::string>("map_frame", "map");
        odom_frame_ = declare_parameter<std::string>("odom_frame", "camera_init");
        odom_topic_ = declare_parameter<std::string>("odom_topic", "/Odometry");
        tf_lookup_timeout_ = declare_parameter<double>("max_sync_dt", 0.08);
        calibration_config_.sample_period = declare_parameter<double>("sample_period", 0.05);
        calibration_config_.print_period = declare_parameter<double>("print_period", 1.0);
        calibration_config_.angle_sample_sec = declare_parameter<double>("angle_sample_sec", 5.0);
        calibration_config_.angle_min_samples = declare_parameter<int>("angle_min_samples", 80);
        calibration_config_.offset_min_samples = declare_parameter<int>("offset_min_samples", 80);
        calibration_config_.offset_min_yaw_span_deg =
            declare_parameter<double>("offset_min_yaw_span_deg", 180.0);
        offset_radius_warn_m_ = declare_parameter<double>("offset_radius_warn_m", 0.02);
        target_roll_deg_ = declare_parameter<double>("target_roll_deg", 0.0);
        target_pitch_deg_ = declare_parameter<double>("target_pitch_deg", 0.0);
        target_yaw_deg_ = declare_parameter<double>("target_yaw_deg", 0.0);
        validateParameters();

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true);

        const auto qos = rclcpp::SensorDataQoS();
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, qos,
            std::bind(&BodyCalibrationNode::odomCallback, this, std::placeholders::_1));
        timer_ = create_wall_timer(std::chrono::seconds(1),
                                   std::bind(&BodyCalibrationNode::timerCallback, this));

        calibration_config_.target_body_rotation =
            backend::rpyDegreesToRotation(target_roll_deg_, target_pitch_deg_, target_yaw_deg_);
        calibration_ = std::make_unique<BodyCalibration>(calibration_config_);

        RCLCPP_INFO_STREAM(
            get_logger(),
            "Body extrinsic calibration started. raw T_map_sensor is composed "
                << "from TF " << map_frame_ << "->" << odom_frame_ << " and " << odom_topic_
                << " (frontend odom->sensor), tf_lookup_timeout=" << tf_lookup_timeout_ << "s.");
        RCLCPP_INFO_STREAM(get_logger(),
                           "Stage 1: keep the robot body facing map +X. The result printed "
                           "below is body.rotation (R_body_sensor).");
    }

private:
    /// 校验采样时序、样本数量及标定几何阈值，拒绝不可用配置。
    void validateParameters() const {
        /// 判断配置是否为有限的非负数。
        const auto nonnegative = [](double value) { return std::isfinite(value) && value >= 0; };
        if (!nonnegative(tf_lookup_timeout_) || !std::isfinite(calibration_config_.sample_period) ||
            calibration_config_.sample_period <= 0 ||
            !std::isfinite(calibration_config_.print_period) ||
            calibration_config_.print_period <= 0 ||
            !nonnegative(calibration_config_.angle_sample_sec))
            throw std::invalid_argument("calibration timing parameters are invalid");
        if (calibration_config_.angle_min_samples < 1 || calibration_config_.offset_min_samples < 2)
            throw std::invalid_argument(
                "calibration requires at least 1 angle and 2 offset samples");
        if (!std::isfinite(calibration_config_.offset_min_yaw_span_deg) ||
            calibration_config_.offset_min_yaw_span_deg <= 0 ||
            !nonnegative(offset_radius_warn_m_) || !std::isfinite(target_roll_deg_) ||
            !std::isfinite(target_pitch_deg_) || !std::isfinite(target_yaw_deg_))
            throw std::invalid_argument("calibration geometric parameters are invalid");
        if (map_frame_.empty() || odom_frame_.empty() || odom_topic_.empty())
            throw std::invalid_argument("calibration frame and topic names must not be empty");
    }

    /// 将同步时刻的前端里程计与 map->odom TF 合成为原始传感器位姿。
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        PoseSample sample;
        sample.stamp = stampToSec(msg->header.stamp);
        if (sample.stamp <= 0.0) {
            sample.stamp = now().seconds();
            sample.ros_stamp = now();
        } else {
            sample.ros_stamp = rclcpp::Time(msg->header.stamp);
        }
        sample.pose = poseToMatrix(msg->pose.pose);
        if (!matrixFinite(sample.pose)) {
            return;
        }

        const auto timeout = rclcpp::Duration::from_seconds(std::max(0.0, tf_lookup_timeout_));
        try {
            const auto tf_msg =
                tf_buffer_->lookupTransform(map_frame_, odom_frame_, sample.ros_stamp, timeout);
            const Eigen::Matrix4d t_map_odom = tf2::transformToEigen(tf_msg).matrix();
            if (!matrixFinite(t_map_odom)) {
                return;
            }
            const Eigen::Matrix4d t_map_sensor = t_map_odom * sample.pose;
            processRawSensorPose(sample.stamp, t_map_sensor);
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *get_clock(), 5000,
                                        "Waiting for TF " << map_frame_ << "->" << odom_frame_
                                                          << " at odom stamp " << sample.stamp
                                                          << ": " << ex.what());
        }
    }

    /// 将同步位姿交给标定对象，并展示阶段完成或定期估计事件。
    void processRawSensorPose(double stamp, const Eigen::Matrix4d& t_map_sensor) {
        switch (calibration_->addPose(stamp, t_map_sensor)) {
            case BodyCalibration::Update::AngleRejected:
                RCLCPP_ERROR(get_logger(), "Angle calibration failed: empty sum");
                break;
            case BodyCalibration::Update::AngleCompleted:
                printAngleEstimate();
                break;
            case BodyCalibration::Update::OffsetReady:
                printOffsetEstimate(false);
                break;
            case BodyCalibration::Update::None:
                break;
        }
    }

    /// 展示已完成的机体旋转标定，并提示进入杆臂采集阶段。
    void printAngleEstimate() {
        const Eigen::Vector3d rpy = backend::rpyFromRotation(calibration_->bodyRotation());
        const double roll_deg = backend::radiansToDegrees(rpy.x());
        const double pitch_deg = backend::radiansToDegrees(rpy.y());
        const double yaw_deg = backend::normalizeDegrees(backend::radiansToDegrees(rpy.z()));
        const double yaw_negative_wrap = yaw_deg > 0.0 ? yaw_deg - 360.0 : yaw_deg;

        RCLCPP_INFO_STREAM(
            get_logger(), "\n========== body_rotation estimate ==========\n"
                              << "body:\n"
                              << "  rotation:\n"
                              << "    roll_deg: " << roll_deg << "\n"
                              << "    pitch_deg: " << pitch_deg << "\n"
                              << "    yaw_deg: " << yaw_deg << "\n"
                              << "# equivalent yaw_deg_negative_wrap: " << yaw_negative_wrap << "\n"
                              << "# convention: R_body_sensor = Rz(yaw)*Ry(pitch)*Rx(roll)\n"
                              << "# backend: R_map_body = R_map_sensor * R_body_sensor^T\n"
                              << "============================================");
        RCLCPP_INFO_STREAM(get_logger(),
                           "Stage 2: rotate the robot around its own center. I will print "
                           "body.offset.x/y continuously.");
    }

    /// 输出当前杆臂估计及其覆盖范围和残差诊断。
    void printOffsetEstimate(const bool force) {
        OffsetEstimate estimate;
        std::string reason;
        if (!calibration_->estimateOffset(&estimate, &reason)) {
            if (force || !reason.empty()) {
                RCLCPP_INFO_STREAM(get_logger(),
                                   "Offset calibration "
                                       << reason << ": samples=" << calibration_->offsetSamples());
            }
            return;
        }

        std::ostringstream warn;
        if (estimate.radius_mean < offset_radius_warn_m_) {
            warn << "\n# warning: fitted radius is only " << estimate.radius_mean
                 << " m; offset may be near zero or rotation motion is too "
                    "small.";
        }

        RCLCPP_INFO_STREAM(get_logger(), "\n========== body_offset estimate ==========\n"
                                             << "body:\n"
                                             << "  offset:\n"
                                             << "    x: " << estimate.body_offset.x() << "\n"
                                             << "    y: " << estimate.body_offset.y() << "\n"
                                             << "    z: 0.0\n"
                                             << "# samples: " << calibration_->offsetSamples()
                                             << ", yaw_span_deg: " << estimate.yaw_span_deg
                                             << ", radius_mean: " << estimate.radius_mean
                                             << ", residual_rms: " << estimate.residual_rms << "\n"
                                             << "# offset_std_x: " << estimate.body_offset_std.x()
                                             << ", offset_std_y: " << estimate.body_offset_std.y()
                                             << "\n"
                                             << "# center_map_x: " << estimate.center_map.x()
                                             << ", center_map_y: " << estimate.center_map.y()
                                             << "\n# fit model: p_map_sensor.xy = center_map - "
                                                "R_map_body.xy * body_offset.xy"
                                             << warn.str() << "\n"
                                             << "==========================================");
    }

    /// 周期报告采样进度或最新杆臂估计。
    void timerCallback() {
        if (!calibration_->receivedPose()) {
            RCLCPP_WARN_STREAM_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "Waiting for odom plus TF: tf=" << map_frame_ << "->" << odom_frame_
                                                << ", odom_topic=" << odom_topic_);
            return;
        }

        if (calibration_->phase() == BodyCalibration::Phase::Angle) {
            const double elapsed = calibration_->angleElapsed();
            RCLCPP_INFO_STREAM(get_logger(), "Angle calibration progress: samples="
                                                 << calibration_->angleSamples() << "/"
                                                 << calibration_config_.angle_min_samples
                                                 << ", elapsed=" << elapsed << "/"
                                                 << calibration_config_.angle_sample_sec << "s");
        } else {
            printOffsetEstimate(false);
        }
    }

    std::string map_frame_;
    std::string odom_frame_;
    std::string odom_topic_;
    double tf_lookup_timeout_ = 0.08;
    double offset_radius_warn_m_ = 0.02;
    double target_roll_deg_ = 0.0;
    double target_pitch_deg_ = 0.0;
    double target_yaw_deg_ = 0.0;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    BodyCalibrationConfig calibration_config_;
    std::unique_ptr<BodyCalibration> calibration_;
};

}  // namespace backend::tools

/// 启动机体外参标定节点并执行 ROS 回调，退出时关闭 ROS。
int main(int argc, char** argv) {
    try {
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<backend::tools::BodyCalibrationNode>());
        rclcpp::shutdown();
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Body calibration failed: " << error.what() << '\n';
        if (rclcpp::ok())
            rclcpp::shutdown();
        return 1;
    }
}
