// 两种真实描述子共享候选资格、近期排除和持久化合同，不要求算法评分相同。
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <memory>

#include "backend/gprobe.hpp"
#include "backend/scan_context.hpp"
#include "scene.hpp"
#include "temporary_directory.hpp"

namespace backend {
namespace {
/// 为 Scan Context 与 G-PROBE 提供相同点云和候选索引场景。
class PlaceRetrieval : public ::testing::TestWithParam<bool> {
protected:
    Cloud::Ptr cloud = testing::roomCloud();
    std::unique_ptr<PlaceDescriptor> descriptor;

    /// 按测试参数构造描述子，并建立四个已知位置的关键帧。
    void SetUp() override {
        if (GetParam()) {
            auto value = std::make_unique<GProbeDescriptor>();
            GProbeDescriptorConfig config;
            config.query_fov_deg = 360;
            config.query_fov_center_deg = 0;
            config.exclude_recent = 2;
            value->configure(config);
            descriptor = std::move(value);
        } else {
            auto value = std::make_unique<ScanContextDescriptor>();
            ScanContextDescriptorConfig config;
            config.exclude_recent = 2;
            config.augment_count = 1;
            value->configure(config);
            descriptor = std::move(value);
        }
        for (int index = 0; index < 4; ++index)
            descriptor->addKeyframe(*cloud, Eigen::Vector3d(index * 10, 0, 0));
    }
};

/// 检查显式候选范围、返回数量上限及不同描述子的航向结果。
TEST_P(PlaceRetrieval, LocalizationHonorsExplicitEligibilityAndResultBudget) {
    const auto hits = descriptor->queryLocalizationCandidates(*cloud, {1}, 1);
    ASSERT_EQ(hits.size(), 1U);
    EXPECT_EQ(hits[0].index, 1);
    EXPECT_TRUE(std::isfinite(hits[0].descriptor_distance));
    EXPECT_TRUE(std::isfinite(hits[0].descriptor_score));
    EXPECT_LE(descriptor->queryLocalizationCandidates(*cloud, {}, 2).size(), 2U);
    EXPECT_TRUE(descriptor->queryLocalizationCandidates(*cloud, {-1, 999}, 2).empty());
    EXPECT_TRUE(descriptor->queryLocalizationCandidates(Cloud{}, {}, 2).empty());
    if (GetParam()) {
        EXPECT_FALSE(hits[0].yaw_seed_rad);
    } else {
        ASSERT_TRUE(hits[0].yaw_seed_rad);
        EXPECT_NEAR(*hits[0].yaw_seed_rad, 0, 1e-8);
    }
}

/// 回环检索应排除当前帧、近期帧和无效索引，即使它们出现在候选白名单中。
TEST_P(PlaceRetrieval, LoopEligibilityNeverReintroducesRecentOrCurrentFrames) {
    const auto hits = descriptor->detectLatestCandidatesWithin({0, 2, 3, -1, 999});
    ASSERT_EQ(hits.size(), 1U);
    EXPECT_EQ(hits[0].index, 0);
    EXPECT_TRUE(descriptor->detectLatestCandidatesWithin({}).empty());
    EXPECT_TRUE(descriptor->detectLatestCandidatesWithin({2, 3}).empty());
    const auto global = descriptor->detectLatestCandidates();
    ASSERT_FALSE(global.empty());
    for (const auto& hit : global) {
        EXPECT_GE(hit.index, 0);
        EXPECT_LT(hit.index, 2);
    }
}

/// 保存并重载描述子后保持检索结果，清空原索引不影响已加载副本。
TEST_P(PlaceRetrieval, SavedIndexPreservesRetrievalAndIndependentEmptyCopy) {
    testing::TemporaryDirectory temporary;
    const auto expected = descriptor->queryLocalizationCandidates(*cloud, {1}, 1);
    ASSERT_EQ(expected.size(), 1U);
    ASSERT_TRUE(descriptor->save(temporary.path().string()));
    auto loaded = descriptor->createEmpty();
    EXPECT_TRUE(loaded->empty());
    EXPECT_EQ(loaded->enabled(), descriptor->enabled());
    ASSERT_TRUE(loaded->load(temporary.path().string()));
    EXPECT_EQ(loaded->size(), descriptor->size());
    const auto actual = loaded->queryLocalizationCandidates(*cloud, {1}, 1);
    ASSERT_EQ(actual.size(), expected.size());
    EXPECT_EQ(actual[0].index, expected[0].index);
    EXPECT_NEAR(actual[0].descriptor_score, expected[0].descriptor_score, 1e-6);
    descriptor->clear();
    EXPECT_TRUE(descriptor->empty());
    EXPECT_FALSE(loaded->empty());
    EXPECT_FALSE(loaded->queryLocalizationCandidates(*cloud, {1}, 1).empty());
}

/// 拒绝无效索引和非有限位置更新，原描述子索引仍可检索。
TEST_P(PlaceRetrieval, InvalidPositionUpdatePreservesRetrievableIndex) {
    EXPECT_THROW(descriptor->updateKeyframePosition(999, Eigen::Vector3d::Zero()),
                 std::out_of_range);
    EXPECT_THROW(descriptor->updateKeyframePosition(
                     0, Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())),
                 std::invalid_argument);
    descriptor->updateKeyframePosition(0, Eigen::Vector3d(100, 2, 0));
    const auto hits = descriptor->queryLocalizationCandidates(*cloud, {0}, 1);
    ASSERT_EQ(hits.size(), 1U);
    EXPECT_EQ(hits[0].index, 0);
}
INSTANTIATE_TEST_SUITE_P(ScanContextAndGProbe, PlaceRetrieval, ::testing::Bool());
}  // namespace
}  // namespace backend
