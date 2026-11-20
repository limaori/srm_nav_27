// 覆盖二维/三维、固定/在线空间对齐及固定/在线时间偏置的真实图因子。
#include <gtest/gtest.h>

#include <limits>
#include <tuple>

#include "backend/pose_graph.hpp"
#include "geometry.hpp"

namespace backend {
namespace {
/// 独立几何判据使用的位姿、空间对齐和时间偏置。
struct CalibrationState {
    Eigen::Matrix4d pose = testing::translation(2, -1, 3);
    gtsam::Pose2 planar{1, -2, .4};
    double height = .5;
    double time_offset = .12;
};

/// 将平面对齐和高度参数组合成 UWB 到地图的变换。
Eigen::Matrix4d alignmentPose(const CalibrationState& state) {
    auto result = testing::translation(state.planar.x(), state.planar.y(), state.height);
    result.block<3, 3>(0, 0) =
        Eigen::AngleAxisd(state.planar.theta(), Eigen::Vector3d::UnitZ()).toRotationMatrix();
    return result;
}

/// 遍历观测维度、空间标定和时间标定三组开关的因子夹具。
class UwbFactor : public ::testing::TestWithParam<std::tuple<bool, bool, bool>> {
protected:
    bool use_z = std::get<0>(GetParam());
    bool aligned = std::get<1>(GetParam());
    bool timed = std::get<2>(GetParam());
    CalibrationState nominal;
    UwbMeasurement measurement;

    /// 用解析运动生成带已知残差及相关噪声的位置观测。
    void SetUp() override {
        nominal.pose.block<3, 3>(0, 0) = (Eigen::AngleAxisd(.3, Eigen::Vector3d::UnitZ()) *
                                          Eigen::AngleAxisd(-.2, Eigen::Vector3d::UnitY()))
                                             .toRotationMatrix();
        measurement.with_alignment = aligned;
        measurement.antenna_in_keyframe = Eigen::Vector3d(.3, -.2, .1);
        measurement.covariance.diagonal() << .09, .16, .25;
        measurement.covariance(0, 1) = measurement.covariance(1, 0) = .015;
        Eigen::Vector3d antenna = measurement.antenna_in_keyframe;
        if (timed) {
            UwbTimeOffsetSupport support;
            support.measurement_timestamp = 11;
            support.antenna_position_sensor = antenna;
            auto end = testing::translation(1, .4, .1);
            end.block<3, 3>(0, 0) =
                Eigen::AngleAxisd(.4, Eigen::Vector3d::UnitZ()).toRotationMatrix();
            support.odometry = {{10, Eigen::Matrix4d::Identity()}, {12, end}};
            measurement.time_support = support;
            // 区间恒速平移与恒角速度运动的独立解析位置。
            const double elapsed = 1 + nominal.time_offset;
            antenna = Eigen::AngleAxisd(.2 * elapsed, Eigen::Vector3d::UnitZ()) * antenna +
                      elapsed * Eigen::Vector3d(.5, .2, .05);
        }
        measurement.position =
            nominal.pose.block<3, 3>(0, 0) * antenna + nominal.pose.block<3, 1>(0, 3);
        if (aligned) {
            const Eigen::Matrix4d transform = alignmentPose(nominal);
            measurement.position = transform.block<3, 3>(0, 0).transpose() *
                                   (measurement.position - transform.block<3, 1>(0, 3));
        }
        measurement.position += Eigen::Vector3d(.12, -.08, .05);
    }

    /// 按测试组合初始化位姿及共享标定状态，并提交预测图。
    void initialize(PoseGraph& graph, const CalibrationState& state) const {
        graph.addPrior(0, state.pose, PoseCovariance6d::Identity() * .1, 1);
        if (aligned)
            graph.addUwbAlignmentPrior(alignmentPose(state), PoseCovariance6d::Identity() * .1,
                                       use_z);
        if (timed)
            graph.addUwbTimeOffsetPrior(state.time_offset, .2);
        graph.update(false);
    }
    /// 在给定独立状态处取得真实因子的白化残差和雅可比。
    InnovationTerm linearize(const CalibrationState& state) const {
        PoseGraph graph;
        initialize(graph, state);
        return graph.linearizeUwbPosition(0, measurement, use_z);
    }
    /// 在对应图变量的原生切空间中施加单轴扰动，供中心差分使用。
    CalibrationState perturb(std::size_t block, int axis, double step) const {
        auto state = nominal;
        if (block == 0) {
            gtsam::Vector6 delta = gtsam::Vector6::Zero();
            delta[axis] = step;
            state.pose = gtsam::Pose3(state.pose).retract(delta).matrix();
        } else if (aligned && block == 1) {
            gtsam::Vector3 delta = gtsam::Vector3::Zero();
            delta[axis] = step;
            state.planar = state.planar.retract(delta);
        } else if (aligned && use_z && block == 2) {
            state.height += step;
        } else {
            state.time_offset += step;
        }
        return state;
    }
};

/// 在八种 UWB 配置下，用独立几何和中心差分核对残差及全部状态雅可比。
TEST_P(UwbFactor, WhitenedResidualAndAllStateJacobiansMatchIndependentGeometry) {
    const auto term = linearize(nominal);
    ASSERT_TRUE(term.valid) << term.invalid_reason;
    const int dimension = use_z ? 3 : 2;
    ASSERT_EQ(term.whitened_residual.size(), dimension);
    const Eigen::MatrixXd covariance = measurement.covariance.topLeftCorner(dimension, dimension);
    const Eigen::Vector3d offset(-.12, .08, -.05);
    const Eigen::VectorXd expected = covariance.llt().matrixL().solve(offset.head(dimension));
    EXPECT_LT((term.whitened_residual - expected).norm(), 1e-10);
    ASSERT_EQ(term.keys.size(), 1U + (aligned ? (use_z ? 2U : 1U) : 0U) + (timed ? 1U : 0U));
    constexpr double step = 1e-6;
    for (std::size_t block = 0; block < term.keys.size(); ++block)
        for (int axis = 0; axis < term.whitened_jacobians[block].cols(); ++axis) {
            SCOPED_TRACE(::testing::Message() << "block=" << block << " axis=" << axis);
            const auto plus = linearize(perturb(block, axis, step));
            const auto minus = linearize(perturb(block, axis, -step));
            ASSERT_TRUE(plus.valid);
            ASSERT_TRUE(minus.valid);
            const Eigen::VectorXd derivative =
                (plus.whitened_residual - minus.whitened_residual) / (2 * step);
            EXPECT_LT((derivative - term.whitened_jacobians[block].col(axis)).norm(), 2e-6);
        }
}

/// 同一 UWB 观测经创新评估和真实图更新后，残差模型与状态修正保持一致。
TEST_P(UwbFactor, SameModelIsUsedByInnovationAssessmentAndGraphUpdate) {
    PoseGraph graph;
    initialize(graph, nominal);
    const auto before = graph.linearizeUwbPosition(0, measurement, use_z);
    const auto assessment = graph.assessInnovationBatch({before});
    ASSERT_TRUE(assessment.joint.valid);
    EXPECT_EQ(assessment.joint.degrees_of_freedom, use_z ? 3 : 2);
    graph.addUwbPosition(0, measurement, use_z);
    EXPECT_FALSE(graph.linearizeUwbPosition(0, measurement, use_z).valid);
    graph.update(true);
    const auto after = graph.linearizeUwbPosition(0, measurement, use_z);
    ASSERT_TRUE(after.valid);
    EXPECT_LT(after.whitened_residual.norm(), before.whitened_residual.norm());
    if (aligned) {
        const auto covariance = graph.uwbAlignmentMarginalCovariance();
        ASSERT_TRUE(covariance);
        EXPECT_TRUE(covariance->allFinite());
        EXPECT_DOUBLE_EQ((*covariance)(3, 3), 0);
        EXPECT_DOUBLE_EQ((*covariance)(4, 4), 0);
        if (!use_z) {
            EXPECT_DOUBLE_EQ((*covariance)(2, 2), 0);
        }
    }
    if (timed) {
        ASSERT_TRUE(graph.uwbTimeOffsetMarginalVariance());
        EXPECT_LE(*graph.uwbTimeOffsetMarginalVariance(), .04 + 1e-10);
    }
}
INSTANTIATE_TEST_SUITE_P(MeasurementModes, UwbFactor,
                         ::testing::Combine(::testing::Bool(), ::testing::Bool(),
                                            ::testing::Bool()));

/// 不可观标定导致数值求解失败时清空图，随后允许建立新的图会话。
TEST(UwbFactorValidation, UnobservableCalibrationClearsFailedGraphAndAllowsNewSession) {
    PoseGraph graph;
    graph.addPrior(0, Eigen::Matrix4d::Identity(), 1);
    graph.addUwbAlignmentInitialValue(Eigen::Matrix4d::Identity(), false);
    UwbMeasurement measurement;
    measurement.with_alignment = true;
    graph.addUwbPosition(0, measurement, false);
    // 单个原点位置只有两个约束，不能确定平面对齐的三个自由度。
    EXPECT_THROW(graph.update(true), NumericalError);
    EXPECT_FALSE(graph.hasEstimate(0));
    EXPECT_FALSE(graph.hasUwbAlignment());
    graph.addPrior(1, Eigen::Matrix4d::Identity(), 2);
    EXPECT_NO_THROW(graph.update(false));
    EXPECT_TRUE(graph.hasEstimate(1));
}

/// 拒绝缺少标定状态或时间支撑无效的 UWB 输入，避免留下部分因子。
TEST(UwbFactorValidation, RejectsMissingCalibrationOrInvalidSupportWithoutPartialInsertion) {
    PoseGraph graph;
    graph.addPrior(0, Eigen::Matrix4d::Identity(), 1);
    graph.update(false);
    UwbMeasurement measurement;
    measurement.with_alignment = true;
    EXPECT_FALSE(graph.linearizeUwbPosition(0, measurement).valid);
    EXPECT_THROW(graph.addUwbPosition(0, measurement), std::invalid_argument);
    measurement.with_alignment = false;
    measurement.time_support = UwbTimeOffsetSupport{};
    EXPECT_FALSE(graph.linearizeUwbPosition(0, measurement).valid);
    EXPECT_THROW(graph.addUwbPosition(0, measurement), std::invalid_argument);
    measurement.time_support.reset();
    measurement.position.z() = std::numeric_limits<double>::quiet_NaN();
    EXPECT_TRUE(graph.linearizeUwbPosition(0, measurement, false).valid);
    EXPECT_FALSE(graph.linearizeUwbPosition(0, measurement, true).valid);
    measurement.position.setZero();
    EXPECT_TRUE(
        graph.assessInnovationBatch({graph.linearizeUwbPosition(0, measurement)}).joint.valid);
}
}  // namespace
}  // namespace backend
