// 用可解析的线性高斯图验证共享变量相关性和联合创新验收边界。
#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "backend/pose_graph.hpp"
#include "geometry.hpp"

namespace backend {
namespace {
/// 具有共享位置先验的两节点线性高斯图夹具。
class InnovationGraph : public ::testing::Test {
protected:
    PoseGraph graph;
    PoseSqrtInformation x_information = PoseSqrtInformation::Zero(1, 6);

    /// 建立已提交的位姿预测图与仅约束 X 方向的观测信息。
    void SetUp() override {
        x_information(0, 0) = 10;
        graph.addPrior(0, Eigen::Matrix4d::Identity(), PoseCovariance6d::Identity() * .25, 1);
        graph.addOdometryMeasurement(0, 1, Eigen::Matrix4d::Identity(), Eigen::Matrix4d::Identity(),
                                     PoseCovariance6d::Identity() * .04, 2);
        graph.update(false);
    }
    /// 在当前图预测处线性化指定节点的 X 方向观测。
    InnovationTerm observe(int index, double x) const {
        return graph.linearizeAbsolutePose(index, testing::translation(x), x_information);
    }
};

/// 用共享先验的两节点图验证联合创新包含节点间交叉协方差。
TEST_F(InnovationGraph, SharedPriorProducesCorrelatedJointInnovation) {
    const auto first = observe(0, .2);
    const auto second = observe(1, -.2);
    const auto batch = graph.assessInnovationBatch({first, second});
    ASSERT_TRUE(batch.joint.valid);
    ASSERT_EQ(batch.individual.size(), 2U);
    // 两个位置共享首帧先验：P00=.25，P11=.25+.04，P01=.25；测量方差均为.01。
    Eigen::Matrix2d covariance;
    covariance << .26, .25, .25, .30;
    const Eigen::Vector2d residual(-.2, .2);
    const double expected = residual.dot(covariance.ldlt().solve(residual));
    EXPECT_NEAR(batch.joint.nis, expected, 1e-10);
    EXPECT_EQ(batch.joint.degrees_of_freedom, 2);
    EXPECT_NEAR(batch.joint.upper_tail_probability, std::exp(-expected / 2), 1e-12);
    EXPECT_GT(batch.joint.nis, batch.individual[0].nis + batch.individual[1].nis);
    const auto reversed = graph.assessInnovationBatch({second, first});
    ASSERT_TRUE(reversed.joint.valid);
    EXPECT_NEAR(reversed.joint.nis, expected, 1e-10);
    EXPECT_NEAR(reversed.individual[0].nis, batch.individual[1].nis, 1e-12);
}

/// 相对位姿创新消去两节点共享的绝对位姿不确定度。
TEST_F(InnovationGraph, RelativeMeasurementCancelsCommonPoseUncertainty) {
    const auto term = graph.linearizeRelativePose(0, 1, testing::translation(.1), x_information);
    const auto batch = graph.assessInnovationBatch({term});
    ASSERT_TRUE(batch.joint.valid);
    EXPECT_EQ(batch.joint.degrees_of_freedom, 1);
    EXPECT_NEAR(batch.joint.nis, .1 * .1 / (.04 + .01), 1e-10);
    ASSERT_EQ(batch.individual.size(), 1U);
    EXPECT_DOUBLE_EQ(batch.individual[0].nis, batch.joint.nis);
}

/// 互不相连的图分量，其联合创新能量等于各分量能量之和。
TEST_F(InnovationGraph, IndependentTreesHaveAdditiveInnovationEnergy) {
    graph.addPrior(2, Eigen::Matrix4d::Identity(), PoseCovariance6d::Identity() * .5, 3);
    graph.update(false);
    const auto batch = graph.assessInnovationBatch({observe(0, .2), observe(2, .3)});
    ASSERT_TRUE(batch.joint.valid);
    ASSERT_EQ(batch.individual.size(), 2U);
    EXPECT_NEAR(batch.joint.nis, .04 / .26 + .09 / .51, 1e-10);
    EXPECT_NEAR(batch.joint.nis, batch.individual[0].nis + batch.individual[1].nis, 1e-10);
}

/// 存在待提交因子时拒绝创新评估，提交后恢复正常查询。
TEST_F(InnovationGraph, PendingFactorsCannotBeAssessedAsCommittedPrediction) {
    const auto term = observe(0, .2);
    graph.addAbsolutePoseInformation(0, testing::translation(.1), x_information);
    EXPECT_FALSE(graph.linearizeAbsolutePose(0, testing::translation(.1), x_information).valid);
    EXPECT_FALSE(graph.assessInnovationBatch({term}).joint.valid);
    graph.update(true);
    EXPECT_TRUE(graph.assessInnovationBatch({observe(0, .1)}).joint.valid);
}

/// 拒绝维度、图键或数值无效的观测项，并保持已有图估计。
TEST_F(InnovationGraph, MalformedTermsAreRejectedWithoutChangingTheGraph) {
    const auto pose = graph.estimate(0);
    const auto covariance = graph.marginalCovariance(0);
    ASSERT_TRUE(pose);
    ASSERT_TRUE(covariance);
    EXPECT_FALSE(graph.assessInnovationBatch({}).joint.valid);
    for (int invalid_case = 0; invalid_case < 8; ++invalid_case) {
        SCOPED_TRACE(invalid_case);
        auto term = observe(0, .2);
        switch (invalid_case) {
            case 0:
                term.valid = false;
                break;
            case 1:
                term.keys.clear();
                break;
            case 2:
                term.whitened_jacobians.clear();
                break;
            case 3:
                term.whitened_residual[0] = std::numeric_limits<double>::quiet_NaN();
                break;
            case 4:
                term.keys[0] = 999;
                break;
            case 5:
                term.whitened_jacobians[0] = Eigen::MatrixXd::Zero(1, 5);
                break;
            case 6:
                term.keys.push_back(term.keys.front());
                term.whitened_jacobians.push_back(term.whitened_jacobians.front());
                break;
            case 7:
                term.whitened_jacobians[0](0, 0) = std::numeric_limits<double>::infinity();
                break;
        }
        const auto result = graph.assessInnovationBatch({term});
        EXPECT_FALSE(result.joint.valid);
        EXPECT_FALSE(result.joint.invalid_reason.empty());
    }
    const auto after_pose = graph.estimate(0);
    const auto after_covariance = graph.marginalCovariance(0);
    ASSERT_TRUE(after_pose);
    ASSERT_TRUE(after_covariance);
    EXPECT_LT((*after_pose - *pose).norm(), 1e-12);
    EXPECT_LT((*after_covariance - *covariance).norm(), 1e-12);
}

/// 过期图键的创新查询失败，活动图键仍可用于观测评估。
TEST_F(InnovationGraph, MarginalizedKeyIsUnavailableWithoutInvalidatingLiveKeys) {
    PoseGraphConfig config;
    config.fixed_lag_seconds = 1;
    graph.reset(config);
    graph.addPrior(0, Eigen::Matrix4d::Identity(), 1);
    graph.update(false);
    for (int index = 1; index <= 4; ++index) {
        graph.addOdometryMeasurement(index - 1, index, Eigen::Matrix4d::Identity(),
                                     Eigen::Matrix4d::Identity(),
                                     PoseCovariance6d::Identity() * .04, index + 1);
        graph.update(false);
    }
    EXPECT_FALSE(observe(0, .1).valid);
    EXPECT_TRUE(graph.assessInnovationBatch({observe(4, .1)}).joint.valid);
}
}  // namespace
}  // namespace backend
