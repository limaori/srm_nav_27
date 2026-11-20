// 用已知外参与整圈解析轨迹验证两阶段标定及不足支撑时的拒绝。
#include "body_calibration.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace backend::tools {
namespace {
/// 以已知旋转外参、车体中心和水平杆臂构造两阶段标定场景。
class BodyCalibrationScenario : public ::testing::Test {
protected:
    BodyCalibrationConfig config;
    const Eigen::Matrix3d rotation = (Eigen::AngleAxisd(.2, Eigen::Vector3d::UnitX()) *
                                      Eigen::AngleAxisd(-.1, Eigen::Vector3d::UnitY()))
                                         .toRotationMatrix();
    const Eigen::Vector2d center{3, -2}, offset{.25, -.12};

    /// 设置采样条件和用于旋转标定的已知车体姿态。
    void SetUp() override {
        config.sample_period = .01;
        config.angle_sample_sec = .4;
        config.angle_min_samples = 5;
        config.offset_min_samples = 12;
        config.offset_min_yaw_span_deg = 180;
        config.target_body_rotation = (Eigen::AngleAxisd(.7, Eigen::Vector3d::UnitZ()) *
                                       Eigen::AngleAxisd(.15, Eigen::Vector3d::UnitY()))
                                          .toRotationMatrix();
    }
    /// 计算车体绕固定中心旋转时的传感器真值位姿。
    Eigen::Matrix4d sensorPose(double yaw) const {
        Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
        const Eigen::Matrix3d body =
            Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        pose.block<3, 3>(0, 0) = body * rotation;
        pose.block<2, 1>(0, 3) = center - body.topLeftCorner<2, 2>() * offset;
        return pose;
    }
    /// 计算已知车体姿态下用于角度标定的传感器位姿。
    Eigen::Matrix4d anglePose() const {
        Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
        pose.block<3, 3>(0, 0) = config.target_body_rotation * rotation;
        pose.block<3, 1>(0, 3) =
            Eigen::Vector3d(center.x(), center.y(), 0) -
            config.target_body_rotation * Eigen::Vector3d(offset.x(), offset.y(), 0);
        return pose;
    }

    /// 完成静态角度采样，核对外参并进入水平偏移标定阶段。
    void calibrateRotation(BodyCalibration& calibration) const {
        BodyCalibration::Update event = BodyCalibration::Update::None;
        for (int index = 0; index < 6; ++index)
            event = calibration.addPose(1 + .1 * index, anglePose());
        EXPECT_EQ(calibration.phase(), BodyCalibration::Phase::Offset);
        EXPECT_TRUE(event == BodyCalibration::Update::OffsetReady ||
                    event == BodyCalibration::Update::AngleCompleted);
        EXPECT_LT((calibration.bodyRotation() - rotation).norm(), 1e-12);
    }
};

/// 通过已知车体姿态与整圈解析运动恢复旋转外参及水平杆臂。
TEST_F(BodyCalibrationScenario, RotationAndLeverArmRecoverKnownPhysicalGeometry) {
    BodyCalibration calibration(config);
    EXPECT_FALSE(calibration.receivedPose());
    calibrateRotation(calibration);
    for (int index = 0; index < 36; ++index)
        calibration.addPose(2 + .1 * index, sensorPose(index * 2 * std::acos(-1.) / 36));
    OffsetEstimate estimate;
    ASSERT_TRUE(calibration.estimateOffset(&estimate, nullptr));
    EXPECT_LT((estimate.center_map - center).norm(), 1e-12);
    EXPECT_LT((estimate.body_offset - offset).norm(), 1e-12);
    EXPECT_LT(estimate.body_offset_std.norm(), 1e-12);
    EXPECT_NEAR(estimate.radius_mean, offset.norm(), 1e-12);
    EXPECT_LT(estimate.residual_rms, 1e-12);
    EXPECT_GT(estimate.yaw_span_deg, 300);
}

/// 重复朝向和过密采样不能替代水平偏移标定所需的航向覆盖。
TEST_F(BodyCalibrationScenario, RepeatedHeadingAndOversamplingDoNotProvideYawExcitation) {
    BodyCalibration calibration(config);
    calibration.addPose(1, anglePose());
    calibration.addPose(1.001, anglePose());
    EXPECT_EQ(calibration.angleSamples(), 1);
    for (int index = 1; index <= 5; ++index)
        calibration.addPose(1 + .1 * index, anglePose());
    OffsetEstimate estimate;
    std::string reason;
    EXPECT_FALSE(calibration.estimateOffset(&estimate, &reason));
    for (int index = 0; index < 20; ++index)
        calibration.addPose(2 + .1 * index, sensorPose(0));
    EXPECT_FALSE(calibration.estimateOffset(&estimate, &reason));
    EXPECT_FALSE(reason.empty());
    EXPECT_GE(calibration.offsetSamples(), 20U);
}
}  // namespace
}  // namespace backend::tools
