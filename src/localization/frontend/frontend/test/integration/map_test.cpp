/**
 * @file map_test.cpp
 * @brief 用解析平面场景验证支撑几何、点与平面不确定性以及地图缓存。
 */
#include "zlio/map.hpp"

#include <gtest/gtest.h>

#include <cmath>

#include "covariance.hpp"
#include "moments.hpp"
#include "zlio/state.hpp"

namespace zlio {
namespace {
/// 构造具有各向同性独立噪声的均匀水平平面。
std::vector<MapPoint> horizontalPlane() {
    std::vector<MapPoint> points;
    for (int x = 1; x <= 8; ++x)
        for (int y = 1; y <= 8; ++y)
            points.push_back({Vec3(.05 * x, .05 * y, .2), Mat3::Identity() * 1e-5, {}});
    return points;
}

/// 射线径向方差与切向角度噪声遵循各自物理尺度。
TEST(PointUncertainty, RayNoiseHasRadialAndTangentialPhysicalScales) {
    const RayMeasurementCovariance noise(.02, .05);
    const Mat3 covariance = noise(Vec3(10, 0, 0));
    EXPECT_NEAR(covariance(0, 0), .02 * .02, 1e-12);
    const double tangent_sigma = 10 * std::sin(.05 * std::acos(-1.) / 180);
    EXPECT_NEAR(covariance(1, 1), tangent_sigma * tangent_sigma, 1e-12);
    EXPECT_NEAR(covariance(2, 2), covariance(1, 1), 1e-12);
}

/// 点边缘协方差仅计入一次共享位姿噪声，复制后保留有效所有权。
TEST(PointUncertainty, SharedPoseNoiseIsAddedOnceToMarginal) {
    const Vec3 point(1, 2, 3);
    const Mat6 pose_covariance = Mat6::Identity() * .03;
    MapPoint mapped{point, Mat3::Identity() * .01, MapPoseCovariance(pose_covariance)};
    const auto J = pointPoseJacobian(point);
    const Mat3 expected = mapped.independent_covariance + J * pose_covariance * J.transpose();
    EXPECT_LT((mapped.marginalCovariance() - expected).norm(), 1e-12);
    auto copy = mapped;
    mapped.shared_pose_covariance.reset();
    EXPECT_LT((copy.marginalCovariance() - expected).norm(), 1e-12);
}

/// 平面谱判据区分面状、线状与体状支撑。
TEST(PlaneGeometry, RejectsLineAndVolumetricSupport) {
    EXPECT_TRUE(planarSpectrum(Vec3(.0001, .1, .2), .01, .2, .1));
    EXPECT_FALSE(planarSpectrum(Vec3(.0001, .0002, .2), .01, .2, .1));
    EXPECT_FALSE(planarSpectrum(Vec3(.1, .15, .2), .01, .2, .1));
}

/// 重复及共线支撑在两种噪声统计路径下均不能定义平面法向。
TEST(PlaneStatistics, RepeatedAndCollinearSupportsCannotDefineANormal) {
    // 非零局部中心使重复坐标的二阶矩存在消减误差，不只检查恰好为零的散布。
    const Vec3 center(3.989, -2.02329, -1.85588);
    const Vec3 origin(3.96875, -2.03125, -1.84375);
    const Mat3 rotation = Eigen::AngleAxisd(.7, Vec3(1, 2, 3).normalized()).toRotationMatrix();
    constexpr int kRepetitions = 20;
    constexpr int kHalfLineSize = 3;
    constexpr double kPointVariance = 1e-5;
    for (double spread : {0., 1e-9, .01}) {
        SCOPED_TRACE(spread);
        plane::MomentStatistics moments;
        for (int repeat = 0; repeat < kRepetitions; ++repeat)
            for (int index = -kHalfLineSize; index <= kHalfLineSize; ++index) {
                const MapPoint point{center + rotation * Vec3(spread * index, 0, 0),
                                     Mat3::Identity() * kPointVariance,
                                     {}};
                moments.add(point, origin);
            }
        EXPECT_FALSE(moments.fit(origin).valid);
        const Mat3 sensor_sum =
            Mat3::Identity() * (kRepetitions * (2 * kHalfLineSize + 1)) * kPointVariance;
        EXPECT_FALSE(moments.fit(origin, .01, &sensor_sum).valid);
    }
}

/// 不同尺度的旋转平面保留解析法向、中心与点面预测方差。
TEST(PlaneStatistics, ResolvedPlanarSupportPreservesAnalyticPredictionVariance) {
    const Vec3 center(.25, -.5, 2);
    const Mat3 rotation = Eigen::AngleAxisd(.7, Vec3(1, 2, 3).normalized()).toRotationMatrix();
    constexpr double kPointVariance = 1e-6;
    // 5×5 对称格的两条切向散布均为 2*spacing²，解析预测方差不依赖平面朝向。
    for (double spacing : {1e-5, .001, .1}) {
        SCOPED_TRACE(spacing);
        plane::MomentStatistics moments;
        for (int x = -2; x <= 2; ++x)
            for (int y = -2; y <= 2; ++y)
                moments.add(MapPoint{center + rotation * Vec3(spacing * x, spacing * y, 0),
                                     Mat3::Identity() * kPointVariance,
                                     {}},
                            center);
        const auto fitted = moments.fit(center);
        ASSERT_TRUE(fitted.valid);
        EXPECT_NEAR(std::abs(fitted.normal.dot(rotation.col(2))), 1, 1e-12);
        EXPECT_LT((fitted.center - center).norm(), 1e-12);
        Vec3 jacobian;
        const Vec3 query = center + rotation * Vec3(3 * spacing, spacing, 0);
        const double expected = kPointVariance / 25 * (1 + (3 * 3 + 1) / 2.);
        EXPECT_NEAR(fitted.variance(query, jacobian), expected, expected * 1e-8);
    }
}

/// 地图匹配恢复平面几何，缓存不得跨清空操作或不同地图实例复用。
TEST(VoxelMap, MatchesPlaneAndInvalidatesCacheAcrossClearAndOtherMaps) {
    VoxelMap map;
    VoxelMap::QueryCache cache;
    EXPECT_FALSE(map.match(Vec3(.2, .2, .21), Mat3::Identity() * .001, &cache));
    map.insert(horizontalPlane());
    const auto match = map.match(Vec3(.2, .2, .21), Mat3::Identity() * .001, &cache);
    ASSERT_TRUE(match);
    EXPECT_NEAR(std::abs(match->normal.z()), 1, 1e-10);
    EXPECT_NEAR(std::abs(match->residual), .01, 1e-10);
    const auto moved = map.match(Vec3(.2, .2, .22), Mat3::Identity() * .001, &cache);
    ASSERT_TRUE(moved);
    EXPECT_NEAR(std::abs(moved->residual), .02, 1e-10);
    VoxelMap other;
    EXPECT_FALSE(other.match(Vec3(.2, .2, .21), Mat3::Zero(), &cache));
    map.clear();
    EXPECT_EQ(map.size(), 0U);
    EXPECT_FALSE(map.match(Vec3(.2, .2, .21), Mat3::Identity() * .001, &cache));
    map.insert(horizontalPlane());
    EXPECT_TRUE(map.match(Vec3(.2, .2, .21), Mat3::Identity() * .001, &cache));
}
}  // namespace
}  // namespace zlio
