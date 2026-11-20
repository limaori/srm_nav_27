// 清理工具的几何性质与完整历史语义，独立于前端估计和在线图。
#include "cleanup/cleanup.hpp"

#include <gtest/gtest.h>

#include <limits>

#include "cleanup/geometry.hpp"
#include "geometry.hpp"

namespace backend::tools::cleanup {
namespace {
/// 构造先前近处占据、随后远处密集射线穿越的走廊扫描。
std::vector<Keyframe> corridor() {
    std::vector<Keyframe> frames;
    for (int index = 0; index < 6; ++index) {
        Keyframe frame;
        frame.index = index;
        frame.timestamp = 10 + index;
        frame.epoch = 3;
        Cloud::Ptr cloud(new Cloud);
        // 自由确认需要完整邻域证据，单条射线不足以证明三维空间已清空。
        const int radius = index == 0 ? 0 : 15;
        for (int y = -radius; y <= radius; ++y)
            for (int z = -radius; z <= radius; ++z) {
                PointType point;
                point.x = index == 0 ? 2.05F : 6.05F;
                point.y = .05F + .2F * y;
                point.z = .05F + .2F * z;
                point.intensity = static_cast<float>(index * 1000 + cloud->size());
                cloud->push_back(point);
            }
        frame.cloud = CloudBlock(cloud);
        frames.push_back(frame);
    }
    return frames;
}

/// 有符号体素坐标跨越页边界时，编码和解码应保持原坐标。
TEST(CleanupGrid, PageEncodingRoundtripsAcrossSignedBoundaries) {
    for (int x = -17; x <= 17; ++x)
        for (int y = -9; y <= 9; ++y)
            for (int z = -9; z <= 9; ++z) {
                const Index cell(x, y, z);
                ASSERT_LT(Grid::offset(cell), Grid::kCellsPerPage);
                ASSERT_TRUE(Grid::coordinate(Grid::page(cell), Grid::offset(cell)) == cell);
            }
    const Grid grid(.2);
    EXPECT_EQ(grid.cell(Eigen::Vector3d(-.01, .01, .21)), Index(-1, 0, 1));
}

class AxisRay : public ::testing::TestWithParam<int> {};
/// 用解析轴向射线检查穿越格序列，并排除回波端点所在格。
TEST_P(AxisRay, VisitsAnalyticCellsAndExcludesEndpointCell) {
    const Grid grid(1);
    const int direction = GetParam();
    Eigen::Vector3d origin(.0, .25, .25), end(3.25 * direction, .25, .25);
    Ray ray(origin, end, grid);
    std::vector<int> visited;
    do {
        ASSERT_LT(visited.size(), 10U) << "ray must terminate";
        visited.push_back(ray.index().x());
        EXPECT_EQ(ray.index().y(), 0);
        EXPECT_EQ(ray.index().z(), 0);
    } while (ray.advance());
    const std::vector<int> expected =
        direction == 1 ? std::vector<int>{0, 1, 2} : std::vector<int>{0, -1, -2, -3};
    EXPECT_EQ(visited, expected);
}
INSTANTIATE_TEST_SUITE_P(SignedBoundary, AxisRay, ::testing::Values(-1, 1));

/// 后续自由空间证据清理早期动态回波，同时保留输入快照与静态表面。
TEST(CleanupPipeline, LaterFreeSpaceRemovesEarlierReturnWithoutChangingInput) {
    const auto input = corridor();
    Config config;
    config.lidar_origin = Eigen::Vector3d(1, .05, .05);
    const auto result = cleanFrames(input, config);
    EXPECT_EQ(result.statistics.input_points, 1U + 5U * 31U * 31U);
    EXPECT_EQ(result.statistics.removed_points, 1U);
    ASSERT_EQ(result.frames.size(), 5U);
    EXPECT_EQ(result.source_indices, (std::vector<int>{1, 2, 3, 4, 5}));
    for (std::size_t index = 0; index < result.frames.size(); ++index) {
        const auto& source = input[index + 1];
        const auto& output = result.frames[index];
        EXPECT_EQ(output.index, static_cast<int>(index));
        EXPECT_EQ(output.epoch, source.epoch);
        EXPECT_DOUBLE_EQ(output.timestamp, source.timestamp);
        EXPECT_LT((output.optimized_pose - source.optimized_pose).norm(), 1e-12);
        EXPECT_LT((output.odom_pose - source.odom_pose).norm(), 1e-12);
        EXPECT_FLOAT_EQ(output.cloud.load()->front().intensity,
                        source.cloud.load()->front().intensity);
    }
    EXPECT_FLOAT_EQ(input.front().cloud.load()->front().x, 2.05F);
    EXPECT_EQ(input.front().cloud.pointCount(), 1U);
}

/// 单条射线缺少邻域证据时，应保留周围未知格中的回波。
TEST(CleanupPipeline, SingleRayDoesNotProveUnobservedNeighborCellsAreFree) {
    auto frames = corridor();
    for (std::size_t index = 1; index < frames.size(); ++index) {
        Cloud::Ptr cloud(new Cloud);
        PointType point;
        point.x = 6.05F;
        point.y = .05F;
        point.z = .05F;
        point.intensity = static_cast<float>(index);
        cloud->push_back(point);
        frames[index].cloud = CloudBlock(cloud);
    }
    const auto result = cleanFrames(frames, {});
    EXPECT_EQ(result.statistics.removed_points, 0U);
    EXPECT_EQ(result.frames.size(), frames.size());
}

/// 改变线程数或施加整数格平移后，逐点清理分类保持一致。
TEST(CleanupPipeline, ParallelismAndIntegerGridTranslationPreserveClassification) {
    auto frames = corridor();
    Config config;
    config.threads = 1;
    const auto serial = cleanFrames(frames, config);
    config.threads = 8;
    const auto parallel = cleanFrames(frames, config);
    EXPECT_EQ(parallel.source_indices, serial.source_indices);
    EXPECT_EQ(parallel.statistics.removed_points, serial.statistics.removed_points);
    for (auto& frame : frames)
        frame.optimized_pose = testing::translation(-4, 2, -2);
    const auto translated = cleanFrames(frames, config);
    EXPECT_EQ(translated.source_indices, serial.source_indices);
    EXPECT_EQ(translated.statistics.removed_points, serial.statistics.removed_points);
}

/// 无效配置和清理中断应报告失败，并保留输入关键帧几何。
TEST(CleanupPipeline, InvalidConfigurationAndInterruptedWorkCannotMutateInput) {
    const auto input = corridor();
    Config config;
    config.voxel_resolution = 0;
    EXPECT_THROW(cleanFrames(input, config), std::invalid_argument);
    config = {};
    config.lidar_origin.x() = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(validateConfig(config), std::invalid_argument);
    config = {};
    EXPECT_THROW(cleanFrames(input, config,
                             [](const char*, std::size_t, std::size_t) {
                                 throw std::runtime_error("cancelled by consumer");
                             }),
                 std::runtime_error);
    EXPECT_EQ(input.size(), 6U);
    EXPECT_EQ(input[0].cloud.pointCount(), 1U);
    EXPECT_NO_THROW(cleanFrames(input, config));
}
}  // namespace
}  // namespace backend::tools::cleanup
