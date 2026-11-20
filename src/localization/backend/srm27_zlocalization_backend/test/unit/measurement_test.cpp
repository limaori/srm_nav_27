// 统计合同以矩阵恒等式及可观零空间为判据。
#include "backend/measurement.hpp"

#include <gtest/gtest.h>

#include <Eigen/Eigenvalues>
#include <limits>

#include "backend/covariance.hpp"
#include "geometry.hpp"

namespace backend {
namespace {
/// 核对协方差坐标往返与 ROS 数组编码，保留交叉项且不引入绝对平移杆臂。
TEST(PoseCovariance, CoordinateRoundtripPreservesCrossTermsAndIgnoresAbsoluteTranslation) {
    PoseCovariance6d basis = PoseCovariance6d::Identity();
    basis(0, 4) = .3;
    const PoseCovariance6d covariance = basis * basis.transpose();
    Eigen::Matrix4d pose = testing::translation(1000, -2000, 3000);
    pose.block<3, 3>(0, 0) = Eigen::AngleAxisd(.7, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const auto local = worldAdditiveToLocalPoseCovariance(pose, covariance);
    EXPECT_LT((localToWorldAdditivePoseCovariance(pose, local) - covariance).norm(), 1e-12);
    pose.block<3, 1>(0, 3).setZero();
    EXPECT_LT((worldAdditiveToLocalPoseCovariance(pose, covariance) - local).norm(), 1e-12);
    std::array<double, 36> wire{};
    writePoseCovariance(covariance, &wire);
    EXPECT_EQ(wire[4], covariance(0, 4));
    EXPECT_LT((poseCovarianceFromRos(wire) - covariance).norm(), 1e-12);
}

/// 数值正则化只能增加协方差，实质负方差和非有限输入应被拒绝。
TEST(PoseCovariance, RegularizationCannotIncreaseConfidence) {
    PoseCovariance6d input = PoseCovariance6d::Identity();
    input(0, 0) = 1e-14;
    PoseCovariance6d output;
    bool regularized = false;
    ASSERT_TRUE(validateAndRegularizePoseCovariance(input, &output, nullptr, &regularized));
    EXPECT_TRUE(regularized);
    const Eigen::SelfAdjointEigenSolver<PoseCovariance6d> spectrum(output - input);
    EXPECT_GE(spectrum.eigenvalues().minCoeff(), -1e-12);
    input(0, 0) = -.1;
    EXPECT_FALSE(validateAndRegularizePoseCovariance(input, &output));
    input(0, 0) = std::numeric_limits<double>::infinity();
    EXPECT_FALSE(validateAndRegularizePoseCovariance(input, &output));
}

/// 附加共模噪声增加可观方向方差，同时保持原观测零空间。
TEST(PoseInformation, CommonNoisePreservesNullspaceAndAddsObservableVariance) {
    PoseSqrtInformation information = PoseSqrtInformation::Zero(2, 6);
    information(0, 0) = 2;
    information(1, 4) = 4;
    ASSERT_TRUE(validatePoseSqrtInformation(information));
    EXPECT_FALSE(poseCovarianceFromSqrtInformation(information));
    const auto bounded =
        addPoseMeasurementCovariance(information, PoseCovariance6d::Identity() * .5);
    ASSERT_TRUE(bounded);
    const PoseCovariance6d precision = bounded->transpose() * *bounded;
    EXPECT_NEAR(precision(0, 0), 1 / (.25 + .5), 1e-12);
    EXPECT_NEAR(precision(4, 4), 1 / (.0625 + .5), 1e-12);
    EXPECT_DOUBLE_EQ(precision(1, 1), 0);
    EXPECT_DOUBLE_EQ(precision(5, 5), 0);
}

/// 检查后端到 GTSAM 的列序转换，并拒绝线性相关的观测行。
TEST(PoseInformation, RankAndColumnOrderAreExplicit) {
    PoseSqrtInformation information = PoseSqrtInformation::Zero(2, 6);
    information(0, 0) = 2;
    information(1, 3) = 3;
    const auto gtsam = backendToGtsamPoseSqrtInformation(information);
    EXPECT_DOUBLE_EQ(gtsam(0, 3), 2);
    EXPECT_DOUBLE_EQ(gtsam(1, 0), 3);
    information.row(1) = information.row(0);
    EXPECT_FALSE(validatePoseSqrtInformation(information));
}

/// 由已知满秩平方根信息恢复解析协方差。
TEST(PoseInformation, FullRankConversionRecoversCovariance) {
    const PoseSqrtInformation information = PoseCovariance6d::Identity() * 2;
    const auto covariance = poseCovarianceFromSqrtInformation(information);
    ASSERT_TRUE(covariance);
    EXPECT_LT((*covariance - PoseCovariance6d::Identity() * .25).norm(), 1e-12);
}
}  // namespace
}  // namespace backend
