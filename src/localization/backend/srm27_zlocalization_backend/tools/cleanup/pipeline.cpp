/**
 * @file pipeline.cpp
 * @brief 对固定地图快照准备观测、运行清理、保留原点属性并写出统计。
 */
#include <pcl/point_types.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <stdexcept>
#include <utility>

#include "cleanup.hpp"
#include "visibility.hpp"

namespace backend::tools::cleanup {
namespace {
/// 平移到雷达原点并物化为单精度，使入图与最终回查使用相同测量坐标。
pcl::PointXYZ lidarPoint(const PointType& point, const Eigen::Vector3d& origin) {
    return {static_cast<float>(point.x - origin.x()), static_cast<float>(point.y - origin.y()),
            static_cast<float>(point.z - origin.z())};
}

/// 将关键帧位姿的平移原点移至雷达发射位置。
Eigen::Isometry3d lidarPose(const Keyframe& frame, const Eigen::Vector3d& origin) {
    Eigen::Isometry3d pose(frame.optimized_pose);
    pose.translate(origin);
    return pose;
}

/// 校验各帧位姿及点坐标的整数格定义域，遇到无效输入时抛出异常。
void validateFrames(const std::vector<Keyframe>& frames, const Config& config) {
    const double coordinate_limit = (static_cast<double>(std::numeric_limits<int>::max()) - 1024) *
                                    config.voxel_resolution / Grid::kSubcellSide;
    for (const auto& frame : frames) {
        const auto& transform = frame.optimized_pose;
        const Eigen::Matrix3d rotation = transform.block<3, 3>(0, 0);
        if (!std::isfinite(frame.timestamp) || !transform.allFinite() ||
            !transform.row(3).isApprox(Eigen::RowVector4d(0, 0, 0, 1), 1e-8) ||
            !(rotation.transpose() * rotation).isApprox(Eigen::Matrix3d::Identity(), 1e-6) ||
            std::abs(rotation.determinant() - 1) > 1e-6)
            throw std::invalid_argument("map cleanup requires finite rigid keyframe poses");
        const auto cloud = frame.cloud.load();
        if (!cloud || cloud->empty())
            throw std::invalid_argument("map cleanup received an empty keyframe");
        const auto pose = lidarPose(frame, config.lidar_origin);
        if (!pose.translation().allFinite() ||
            pose.translation().cwiseAbs().maxCoeff() >= coordinate_limit)
            throw std::invalid_argument("map cleanup origin exceeds the integer map domain");
        for (const auto& point : *cloud) {
            const auto lidar = lidarPoint(point, config.lidar_origin);
            const Eigen::Vector3d local(lidar.x, lidar.y, lidar.z);
            const Eigen::Vector3d world = pose * local;
            if (!world.allFinite() || world.cwiseAbs().maxCoeff() >= coordinate_limit)
                throw std::invalid_argument("map cleanup point exceeds the integer map domain");
        }
    }
}
}  // namespace

/// 检查清理分辨率、射线长度、计数、并行度及雷达原点的有效范围。
void validateConfig(const Config& config) {
    if (!std::isfinite(config.voxel_resolution) || config.voxel_resolution < 0.05 ||
        config.voxel_resolution > 2 || !std::isfinite(config.raycast_range) ||
        config.raycast_range < config.voxel_resolution || config.raycast_range > 200 ||
        config.free_observations < 1 || config.free_observations > 1000 || config.threads < 1 ||
        config.threads > 32 || !config.lidar_origin.allFinite())
        throw std::invalid_argument("invalid offline map cleanup configuration");
}

/// 在完整观测序列上建立最终标签，再按原始点属性生成清理后的关键帧。
Result cleanFrames(const std::vector<Keyframe>& frames, const Config& config,
                   const Progress& progress) {
    validateConfig(config);
    if (frames.empty() ||
        frames.size() >= static_cast<std::size_t>(std::numeric_limits<int>::max()))
        throw std::invalid_argument("map cleanup requires a nonempty, bounded frame sequence");
    const auto start = std::chrono::steady_clock::now();
    validateFrames(frames, config);
    const Grid grid(config.voxel_resolution);
    VisibilityMap map(config);
    // 按观测顺序更新可见性状态，后续观测可修正历史分类。
    for (std::size_t i = 0; i < frames.size(); ++i) {
        const auto cloud = frames[i].cloud.load();
        const auto pose = lidarPose(frames[i], config.lidar_origin);
        std::vector<Eigen::Vector3d> points;
        points.reserve(cloud->size());
        for (const auto& point : *cloud) {
            const auto lidar = lidarPoint(point, config.lidar_origin);
            points.push_back(pose * Eigen::Vector3d(lidar.x, lidar.y, lidar.z));
        }
        Scan scan(std::move(points), grid);
        map.integrate(scan, pose.translation(), static_cast<std::uint32_t>(i));
        if (progress)
            progress("integrate", i + 1, frames.size());
    }
    // 使用最终标签回查同一输入快照，省略空帧并记录原始编号。
    Result result;
    result.statistics.input_frames = frames.size();
    result.frames.reserve(frames.size());
    result.source_indices.reserve(frames.size());
    for (std::size_t i = 0; i < frames.size(); ++i) {
        const auto& frame = frames[i];
        const auto cloud = frame.cloud.load();
        auto filtered = Cloud::Ptr(new Cloud);
        filtered->reserve(cloud->size());
        const auto pose = lidarPose(frame, config.lidar_origin);
        for (const auto& point : *cloud) {
            const auto lidar = lidarPoint(point, config.lidar_origin);
            const auto label = map.classify(pose * Eigen::Vector3d(lidar.x, lidar.y, lidar.z));
            ++result.statistics.input_points;
            if (label == Label::Dynamic) {
                ++result.statistics.removed_points;
            } else {
                filtered->push_back(point);
                result.statistics.unclassified_points += label == Label::Unknown;
            }
        }
        if (!filtered->empty()) {
            Keyframe output = frame;
            output.index = static_cast<int>(result.frames.size());
            output.cloud = CloudBlock(filtered);
            result.frames.push_back(std::move(output));
            result.source_indices.push_back(frame.index);
        }
        if (progress)
            progress("filter", i + 1, frames.size());
    }
    if (result.frames.empty())
        throw std::runtime_error("map cleanup removed every keyframe; refusing an empty map");
    result.statistics.output_frames = result.frames.size();
    result.statistics.elapsed_seconds =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    return result;
}

/// 写出清理配置、统计量和输出帧到原始帧的编号映射。
void saveReport(const std::string& directory, const Config& config, const Result& result) {
    std::ofstream out(std::filesystem::path(directory) / "cleanup.txt");
    const auto& stats = result.statistics;
    out << std::setprecision(17)
        << "algorithm FreeDOM\nupstream dcfd5690cafddf121a80bd8ea477807d9656748e\n"
        << "voxel_resolution " << config.voxel_resolution << "\nraycast_range "
        << config.raycast_range << "\nfree_observations " << config.free_observations
        << "\nthreads " << config.threads << "\nlidar_origin " << config.lidar_origin.transpose()
        << "\ninput_frames " << stats.input_frames << "\noutput_frames " << stats.output_frames
        << "\ninput_points " << stats.input_points << "\nremoved_points " << stats.removed_points
        << "\nunclassified_points " << stats.unclassified_points << "\nelapsed_seconds "
        << stats.elapsed_seconds << "\n# output_index source_index\n";
    for (std::size_t i = 0; i < result.source_indices.size(); ++i)
        out << i << ' ' << result.source_indices[i] << '\n';
    out.flush();
    if (!out)
        throw std::runtime_error("cannot write map cleanup report");
    out.close();
    if (!out)
        throw std::runtime_error("cannot close map cleanup report");
}
}  // namespace backend::tools::cleanup
