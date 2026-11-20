/**
 * @file build.hpp
 * @brief 声明全局 PCD 采样参数、局部扫描结果及地图包输出接口。
 */
#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>
#include <string>
#include <vector>

#include "backend/gprobe.hpp"
#include "backend/scan_context.hpp"
#include "backend/types.hpp"

namespace backend::tools::build {

struct BuildOptions {
    std::string input_pcd;
    std::string out_samples;
    std::string out_bundle;

    double x_min = 0.0;
    double x_max = 28.0;
    double y_min = 0.0;
    double y_max = 15.0;
    // 雷达离地高度：既写关键帧 z，也作为 Scan Context 的默认 lidar_height。
    // 必须与 config/backend.yaml 的 scan_context.lidar_height 一致（本车 URDF 0.346388 → 0.3464），
    // 否则建出来的地图包会在 backend 启动时被几何配置校验拒绝（storage.cpp 的 1e-6 比较）。
    double sensor_z = 0.3464;
    double query_step = 2.0;
    double bundle_step = 1.5;
    std::vector<double> query_yaws_deg{0.0, 90.0, 180.0, -90.0};
    std::vector<double> bundle_yaws_deg{0.0};
    int max_samples = 0;
    int max_keyframes = 0;

    double map_voxel = 0.40;
    double scan_voxel = 0.40;
    double min_distance = 0.5;
    double max_distance = 80.0;
    double fov_down_deg = -7.0;
    double fov_up_deg = 52.0;
    int azimuth_bins = 1440;
    int elevation_bins = 160;
    bool occlusion = false;
    bool write_map_pcd = true;
    ScanContextDescriptorConfig scan_context_config;
    GProbeDescriptorConfig gprobe_config;
    bool scan_context_lidar_height_explicit = false;
    bool help = false;

    /// 设置全局 PCD 采样工具采用的默认描述子配置。
    BuildOptions();
};

struct GeneratedScan {
    int index = -1;
    double stamp = 0.0;
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    Cloud::Ptr cloud{new Cloud};
};

/// 输出命令行参数说明。
void printUsage(const char* program);

/// 解析并校验采样、扫描模型和描述子参数。
bool parseArgs(int argc, char** argv, BuildOptions& args);

/// 创建空输出目录，并拒绝覆盖已有内容。
bool ensureEmptyOutputDir(const std::string& path);

/// 按平面网格和航向集合生成传感器位姿。
std::vector<Eigen::Matrix4d> makePoseGrid(const BuildOptions& args, double step,
                                          const std::vector<double>& yaws, int max_count);

/// 加载作为扫描真值的全局 PCD。
pcl::PointCloud<pcl::PointXYZ>::Ptr loadReferenceMap(const std::string& path);

/// 体素化全局参考点云。
pcl::PointCloud<pcl::PointXYZ>::Ptr voxelizeReference(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                                                      double leaf);

/// 输出点云规模和轴对齐包围盒。
void printBounds(const std::string& label, const pcl::PointCloud<pcl::PointXYZ>& cloud);

/// 在给定传感器位姿上模拟局部扫描序列。
std::vector<GeneratedScan> generateScans(const pcl::PointCloud<pcl::PointXYZ>& reference,
                                         const std::vector<Eigen::Matrix4d>& poses,
                                         const BuildOptions& args, double stamp_base);

/// 写出用于离线验证的扫描点云和位姿表。
bool writeSamples(const std::string& directory, const std::vector<GeneratedScan>& scans);

/// 写出可由后端直接加载的地图包及可选描述子和全局点云。
bool writeBundle(const std::string& directory, const std::vector<GeneratedScan>& scans,
                 bool write_map_pcd, double map_voxel,
                 const ScanContextDescriptorConfig& scan_context_config,
                 const GProbeDescriptorConfig& gprobe_config);

}  // namespace backend::tools::build
