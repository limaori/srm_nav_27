// 使用解析运动及线性高斯后验检验传播、更新和独立预测器的组合。
#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

#include "zlio/filter.hpp"
#include "zlio/odometry.hpp"
#include "zlio/prediction.hpp"
#include "zlio/state.hpp"

namespace zlio {
namespace {
/// 构造与重力平衡的静止惯性采样。
Imu stationaryImu(Time time) {
    Imu imu;
    imu.time = time;
    imu.acceleration.z() = 9.81;
    return imu;
}

TEST(InertialPropagation, StationarySpecificForceBalancesGravity) {
    Estimate estimate;
    estimate.state.time = nanoseconds(1);
    estimate.covariance *= .01;
    const auto segment = propagateInertial(estimate, stationaryImu(nanoseconds(1)),
                                           stationaryImu(nanoseconds(1.01)), Vec3(0, 0, -9.81), {});
    EXPECT_EQ(estimate.state.time, nanoseconds(1.01));
    EXPECT_LT(estimate.state.pose.position.norm(), 1e-12);
    EXPECT_LT(estimate.state.velocity.norm(), 1e-12);
    EXPECT_LT(segment.at(nanoseconds(1.005)).position.norm(), 1e-12);
    EXPECT_TRUE(numericallyPositiveCovariance(estimate.covariance));
    EXPECT_LT((estimate.covariance - estimate.covariance.transpose()).norm(), 1e-12);
}

TEST(InertialTrajectory, ConstantAccelerationMatchesAnalyticPositionAndVelocity) {
    State state;
    state.time = nanoseconds(1);
    state.velocity.x() = 2;
    Imu left = stationaryImu(state.time);
    left.acceleration.x() = 3;
    Imu right = left;
    right.time = nanoseconds(1.2);
    const ImuSegment segment(state, left, right, Vec3(0, 0, -9.81));
    EXPECT_NEAR(segment.end().pose.position.x(), 2 * .2 + .5 * 3 * .2 * .2, 1e-12);
    EXPECT_NEAR(segment.end().velocity.x(), 2 + 3 * .2, 1e-12);
    EXPECT_NEAR(segment.at(nanoseconds(1.1)).position.x(), .215, 1e-12);
    EXPECT_THROW(segment.at(state.time - 1), std::exception);
    EXPECT_THROW(segment.at(right.time + 1), std::exception);
}

TEST(ErrorStateFilter, RepeatedLinearizationDoesNotCountPriorMultipleTimes) {
    ErrorStateFilter filter;
    filter.reset({}, Mat15::Identity());
    // x ~ N(0,1)，一次 x=2、方差 1/4 的观测给出均值 1.6、方差 .2。
    const auto result = filter.update(
        [](const State& state, const Mat15&) {
            Linearization observation;
            observation.information(0, 0) = 4;
            observation.gradient[0] = 4 * (state.pose.position.x() - 2);
            observation.count = 1;
            return observation;
        },
        8, 1e-10);
    EXPECT_TRUE(result.accepted);
    EXPECT_NEAR(filter.state().pose.position.x(), 1.6, 1e-10);
    EXPECT_NEAR(filter.covariance()(0, 0), .2, 1e-10);
    // SD 图表会耦合平移与姿态；未观测的姿态方差仍保持先验。
    EXPECT_NEAR(filter.covariance()(5, 5), 1, 1e-10);
}

TEST(ErrorStateFilter, EmptyObservationPreservesPrior) {
    ErrorStateFilter filter;
    State state;
    state.pose.position = Vec3(1, 2, 3);
    filter.reset(state, Mat15::Identity());
    const auto result = filter.update([](const State&, const Mat15&) { return Linearization{}; });
    EXPECT_FALSE(result.accepted);
    EXPECT_LT((filter.state().pose.position - state.pose.position).norm(), 1e-12);
    EXPECT_LT((filter.covariance() - Mat15::Identity()).norm(), 1e-12);
}

TEST(OutputPredictor, DelayedPosteriorRepropagatesWithoutChangingCallerEstimate) {
    OutputPredictor predictor(Vec3(0, 0, -9.81), {});
    for (int index = 0; index <= 4; ++index)
        EXPECT_FALSE(predictor.addImu(stationaryImu(nanoseconds(1 + .01 * index))));
    Estimate posterior;
    posterior.state.time = nanoseconds(1.02);
    posterior.state.velocity.x() = 2;
    ASSERT_TRUE(predictor.correct(posterior));
    EXPECT_FALSE(predictor.correct(posterior));
    const auto predicted = predictor.addImu(stationaryImu(nanoseconds(1.05)));
    ASSERT_TRUE(predicted);
    EXPECT_EQ(predicted->state.time, nanoseconds(1.05));
    EXPECT_NEAR(predicted->state.pose.position.x(), .06, 1e-10);
    EXPECT_DOUBLE_EQ(posterior.state.pose.position.x(), 0);
    predictor.reset(posterior);
    EXPECT_TRUE(predictor.initialized());
}

/// 重置预检拒绝已经过期的后验，同时允许保留历史中的精确端点。
TEST(OutputPredictor, ResetEligibilityTracksRetainedHistory) {
    OutputPredictor predictor(Vec3(0, 0, -9.81), {}, {.03, 65536});
    EXPECT_FALSE(predictor.canReset(nanoseconds(1)));
    predictor.addImu(stationaryImu(nanoseconds(1)));
    Estimate posterior;
    posterior.state.time = nanoseconds(1);
    ASSERT_TRUE(predictor.correct(posterior));
    for (int step = 1; step <= 5; ++step)
        predictor.addImu(stationaryImu(nanoseconds(1) + step * nanoseconds(.01)));
    EXPECT_FALSE(predictor.canReset(posterior.state.time));
    EXPECT_TRUE(predictor.canReset(nanoseconds(1.02)));
    EXPECT_FALSE(predictor.canReset(nanoseconds(1.05) + 1));
}

/// 预测器不能把真实断流解释成一个跨越多秒的测量区间。
TEST(OutputPredictor, RejectsMissingMeasurementsBeforePropagation) {
    OutputPredictor predictor(Vec3(0, 0, -9.81), {});
    predictor.addImu(stationaryImu(nanoseconds(1)));
    Estimate posterior;
    posterior.state.time = nanoseconds(1);
    posterior.state.velocity.x() = 2;
    ASSERT_TRUE(predictor.correct(posterior));
    EXPECT_THROW(predictor.addImu(stationaryImu(nanoseconds(11))), InputDiscontinuity);
    const auto valid = predictor.addImu(stationaryImu(nanoseconds(1.01)));
    ASSERT_TRUE(valid);
    EXPECT_NEAR(valid->state.pose.position.x(), .02, 1e-12);
}

/// 重复及被拒绝的回退采样不产生预测输出，也不改变后续均值和协方差。
TEST(OutputPredictor, DuplicateAndRewoundSamplesDoNotAlterAcceptedPredictions) {
    OutputPredictor predictor(Vec3(0, 0, -9.81), {});
    OutputPredictor reference(Vec3(0, 0, -9.81), {});
    const auto first = stationaryImu(nanoseconds(1));
    predictor.addImu(first);
    reference.addImu(first);
    Estimate posterior;
    posterior.state.time = first.time;
    posterior.state.velocity = Vec3(2, -1, .5);
    ASSERT_TRUE(predictor.correct(posterior));
    ASSERT_TRUE(reference.correct(posterior));
    for (int step = 1; step <= 3; ++step) {
        const auto next = stationaryImu(first.time + step * nanoseconds(.01));
        auto duplicate = stationaryImu(next.time - nanoseconds(.01));
        duplicate.acceleration = Vec3(100, -50, 200);
        duplicate.angular_velocity = Vec3(1, 2, 3);
        EXPECT_FALSE(predictor.addImu(duplicate));
        EXPECT_THROW(predictor.addImu(stationaryImu(duplicate.time - 1)), InputDiscontinuity);
        auto invalid = duplicate;
        invalid.acceleration.x() = std::numeric_limits<double>::quiet_NaN();
        EXPECT_THROW(predictor.addImu(invalid), std::invalid_argument);
        const auto actual = predictor.addImu(next);
        const auto expected = reference.addImu(next);
        ASSERT_TRUE(actual);
        ASSERT_TRUE(expected);
        EXPECT_EQ(actual->state.time, expected->state.time);
        EXPECT_LT(difference(actual->state, expected->state).norm(), 1e-12);
        EXPECT_LT((actual->covariance - expected->covariance).norm(), 1e-12);
    }
}

/// 里程计入口与预测入口使用相同的重复、回退和无效测量分类。
TEST(OdometryInput, DuplicateSamplesDoNotAdvanceInitialization) {
    Odometry odometry(Config{});
    const auto first = stationaryImu(nanoseconds(1));
    odometry.addImu(first);
    const auto status = odometry.initializationStatus();
    EXPECT_NO_THROW(odometry.addImu(first));
    EXPECT_FALSE(odometry.initialized());
    EXPECT_EQ(odometry.initializationStatus().state, status.state);
    EXPECT_EQ(odometry.initializationStatus().reason, status.reason);
    EXPECT_THROW(odometry.addImu(stationaryImu(first.time - 1)), InputDiscontinuity);
    auto invalid = first;
    invalid.angular_velocity.z() = std::numeric_limits<double>::infinity();
    EXPECT_THROW(odometry.addImu(invalid), std::invalid_argument);
    EXPECT_NO_THROW(odometry.addImu(stationaryImu(first.time + nanoseconds(.01))));
}
}  // namespace
}  // namespace zlio
