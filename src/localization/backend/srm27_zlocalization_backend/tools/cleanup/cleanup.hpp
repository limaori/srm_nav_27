/**
 * @file cleanup.hpp
 * @brief 独立地图工具使用的配置、结果及固定快照清理入口。
 */
#pragma once

#include <cstddef>
#include <functional>
#include <string>
#include <vector>

#include "backend/types.hpp"

namespace backend::tools::cleanup {
struct Config {
    /// 自由体素边长，单位为米。
    double voxel_resolution = 0.2;
    /// 用于累计自由证据的射线长度上限，单位为米。
    double raycast_range = 20.0;
    /// 将体素确认为自由空间所需的观测帧数。
    int free_observations = 3;
    /// 共享调度器允许同时执行的最大任务数。
    int threads = 4;
    /// 雷达原点在关键帧点云坐标系中的位置，单位为米。
    Eigen::Vector3d lidar_origin = Eigen::Vector3d::Zero();
};
struct Statistics {
    std::size_t input_frames = 0, output_frames = 0;
    std::size_t input_points = 0, removed_points = 0, unclassified_points = 0;
    double elapsed_seconds = 0;
};
struct Result {
    std::vector<Keyframe> frames;
    /// 按输出帧顺序保存其原始关键帧编号。
    std::vector<int> source_indices;
    Statistics statistics;
};
using Progress = std::function<void(const char*, std::size_t, std::size_t)>;

/// 验证有限参数和资源配置，不创建检测状态。
void validateConfig(const Config& config);
/// 整组观测完成后按最终占据标签筛选原始点，保留位姿与点属性。
Result cleanFrames(const std::vector<Keyframe>& frames, const Config& config,
                   const Progress& progress = {});
/// 保存配置、数量及导出帧到原帧的映射；调用方负责目录事务。
void saveReport(const std::string& directory, const Config& config, const Result& result);
}  // namespace backend::tools::cleanup
