// 配置验收与初始化边界独立于具体 YAML 排版。
#include "zlio/config.hpp"

#include <gtest/gtest.h>

#include <functional>
#include <limits>

#include "zlio/initialization.hpp"
#include "zlio/odometry.hpp"

namespace zlio {
namespace {
TEST(Configuration, RepositoryConfigurationIsValidAndExtrinsicIsNormalized) {
    const Config config = loadConfig(ZLIO_TEST_CONFIG);
    EXPECT_NO_THROW(validateConfig(config));
    EXPECT_NEAR(config.body_from_lidar.rotation.norm(), 1, 1e-12);
    Config scaled = config;
    // 仅允许修正小幅标定舍入误差；明显错误的旋转应被拒绝。
    scaled.body_from_lidar.rotation.coeffs() *= 1.001;
    const Config normalized = normalizeConfig(scaled);
    EXPECT_NEAR(normalized.body_from_lidar.rotation.norm(), 1, 1e-12);
    EXPECT_LT((normalized.body_from_lidar.rotation.toRotationMatrix() -
               config.body_from_lidar.rotation.toRotationMatrix())
                  .norm(),
              1e-12);
}

using InvalidConfig = std::function<void(Config&)>;
class ConfigurationDomain : public ::testing::TestWithParam<InvalidConfig> {};
TEST_P(ConfigurationDomain, RejectsInvalidPhysicalOrResourceDomain) {
    Config config;
    GetParam()(config);
    EXPECT_THROW(validateConfig(config), std::invalid_argument);
}
INSTANTIATE_TEST_SUITE_P(
    InvalidInputs, ConfigurationDomain,
    ::testing::Values(
        InvalidConfig([](Config& c) { c.gravity = std::numeric_limits<double>::quiet_NaN(); }),
        InvalidConfig([](Config& c) { c.maximum_range = c.minimum_range; }),
        InvalidConfig([](Config& c) { c.map.voxel_size = 0; }),
        InvalidConfig([](Config& c) { c.map.maximum_roots = 0; }),
        InvalidConfig([](Config& c) { c.association_threads = 0; }),
        InvalidConfig([](Config& c) { c.window.minimum_interval = c.window.maximum_interval + 1; }),
        InvalidConfig([](Config& c) { c.noise.gyro_sample_variance = -1; }),
        InvalidConfig([](Config& c) { c.body_from_lidar.rotation.coeffs().setZero(); })));

TEST(InitializationBoundary, ConfiguredPriorRetainsBlockUnitsAndAnisotropy) {
    InitialUncertainty uncertainty;
    for (std::size_t block = 0; block < uncertainty.variance.size(); ++block)
        uncertainty.variance[block] = Vec3(1, 2, 3) * static_cast<double>(block + 1);
    const Mat15 covariance = initialCovariance(uncertainty);
    for (int axis = 0; axis < 15; ++axis)
        EXPECT_DOUBLE_EQ(covariance(axis, axis), (axis / 3 + 1) * (axis % 3 + 1));
    EXPECT_DOUBLE_EQ((covariance - covariance.diagonal().asDiagonal().toDenseMatrix()).norm(), 0);
}

TEST(InitializationBoundary, EmptySessionCannotPublishOrWarmResetAnEstimate) {
    Odometry odometry({});
    EXPECT_FALSE(odometry.initialized());
    EXPECT_FALSE(odometry.resetWarm());
    EXPECT_TRUE(odometry.processAvailable(true).empty());
    EXPECT_EQ(odometry.initializationStatus().state, InitializationState::Collecting);
    EXPECT_THROW(odometry.setOutputResolution(-1), std::invalid_argument);
    EXPECT_FALSE(odometry.initialized());
}
}  // namespace
}  // namespace zlio
