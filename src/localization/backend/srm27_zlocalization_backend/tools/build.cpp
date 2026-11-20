/**
 * @file build.cpp
 * @brief 解析采样参数并生成点云样本与地图包。
 */

#include "build.hpp"

#include <algorithm>
#include <exception>
#include <filesystem>
#include <functional>
#include <initializer_list>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "arguments.hpp"

namespace fs = std::filesystem;

namespace backend::tools::build {
namespace {

using ArgumentTable = cli::Arguments;

/// 解析逗号分隔的角度列表。
std::vector<double> parseDegreesList(const std::string& value) {
    std::vector<double> out;
    std::stringstream stream(value);
    std::string token;
    while (std::getline(stream, token, ',')) {
        if (!token.empty()) {
            out.push_back(std::stod(token));
        }
    }
    return out;
}

/// 注册采样范围、扫描模型和输出选项。
void registerSamplingOptions(ArgumentTable* table, BuildOptions* args) {
    table->value({"--input", "--map"}, args->input_pcd);
    table->value({"--out-samples"}, args->out_samples);
    table->value({"--out-bundle"}, args->out_bundle);
    table->value({"--x-min"}, args->x_min);
    table->value({"--x-max"}, args->x_max);
    table->value({"--y-min"}, args->y_min);
    table->value({"--y-max"}, args->y_max);
    table->value({"--sensor-z"}, args->sensor_z);
    table->value({"--query-step"}, args->query_step);
    table->value({"--bundle-step"}, args->bundle_step);
    table->add({"--query-yaws"}, [args](const std::string& value) {
        args->query_yaws_deg = parseDegreesList(value);
        return true;
    });
    table->add({"--bundle-yaws"}, [args](const std::string& value) {
        args->bundle_yaws_deg = parseDegreesList(value);
        return true;
    });
    table->value({"--max-samples"}, args->max_samples);
    table->value({"--max-keyframes"}, args->max_keyframes);
    table->value({"--map-voxel"}, args->map_voxel);
    table->value({"--scan-voxel"}, args->scan_voxel);
    table->value({"--min-distance"}, args->min_distance);
    table->value({"--max-distance"}, args->max_distance);
    table->value({"--fov-down"}, args->fov_down_deg);
    table->value({"--fov-up"}, args->fov_up_deg);
    table->value({"--azimuth-bins"}, args->azimuth_bins);
    table->value({"--elevation-bins"}, args->elevation_bins);
    table->value({"--occlusion"}, args->occlusion);
    table->value({"--write-map-pcd"}, args->write_map_pcd);
    // 一个描述子类型选项同时确定两种可选描述子的启用组合。
    table->add({"--descriptor-type"}, [args](const std::string& value) {
        if (value == "scan_context") {
            args->scan_context_config.enabled = true;
            args->gprobe_config.enabled = false;
        } else if (value == "gprobe") {
            args->scan_context_config.enabled = false;
            args->gprobe_config.enabled = true;
        } else if (value == "both") {
            args->scan_context_config.enabled = true;
            args->gprobe_config.enabled = true;
        } else {
            std::cerr << "Unsupported descriptor type: " << value << "\n";
            return false;
        }
        return true;
    });
}

/// 注册 Scan Context 几何和检索参数。
void registerScanContextOptions(ArgumentTable* table, BuildOptions* args) {
    auto& config = args->scan_context_config;
    table->value({"--sc-enabled"}, config.enabled);
    table->value({"--sc-rings"}, config.num_rings);
    table->value({"--sc-sectors"}, config.num_sectors);
    table->value({"--sc-max-radius"}, config.max_radius);
    table->value({"--sc-downsample"}, config.downsample);
    // 记录显式高度覆盖，使默认高度仍可跟随模拟传感器位置。
    table->add({"--sc-lidar-height"}, [args](const std::string& value) {
        args->scan_context_config.lidar_height = std::stod(value);
        args->scan_context_lidar_height_explicit = true;
        return true;
    });
    table->value({"--sc-min-bin-points"}, config.min_bin_points);
    table->value({"--sc-augment-count"}, config.augment_count);
    table->value({"--sc-lateral-shift"}, config.lateral_shift);
    table->value({"--sc-search-ratio"}, config.search_ratio);
    table->value({"--sc-exclude-recent"}, config.exclude_recent);
    table->value({"--sc-candidates"}, config.candidates);
    table->value({"--sc-distance-threshold"}, config.distance_threshold);
    table->value({"--sc-level"}, config.level_with_odom_roll_pitch);
}

/// 注册 G-PROBE 几何、概率模型和检索参数。
void registerGProbeOptions(ArgumentTable* table, BuildOptions* args) {
    auto& config = args->gprobe_config;
    table->value({"--gprobe-enabled"}, config.enabled);
    table->value({"--gprobe-rings"}, config.num_rings);
    table->value({"--gprobe-sectors"}, config.num_sectors);
    table->value({"--gprobe-max-range"}, config.max_range);
    table->value({"--gprobe-voxel-size"}, config.voxel_size);
    table->value({"--gprobe-sigma-t"}, config.sigma_t);
    table->value({"--gprobe-query-fov"}, config.query_fov_deg);
    table->value({"--gprobe-query-fov-center"}, config.query_fov_center_deg);
    table->value({"--gprobe-branch-top-k"}, config.branch_top_k);
    table->value({"--gprobe-verification-top-k"}, config.verification_top_k);
    table->value({"--gprobe-spatial-nms-radius"}, config.spatial_nms_radius);
    table->value({"--gprobe-heading-window"}, config.heading_window_deg);
    table->value({"--gprobe-column-trim"}, config.column_trim_ratio);
    table->value({"--gprobe-sgrt-kappa"}, config.sgrt_kappa);
    table->value({"--gprobe-level"}, config.level_with_odom_roll_pitch);
    table->value({"--gprobe-exclude-recent"}, config.exclude_recent);
    table->value({"--gprobe-candidates"}, config.candidates);
}

}  // namespace

/// 设置全局 PCD 采样工具采用的默认描述子配置。
BuildOptions::BuildOptions() {
    gprobe_config.enabled = false;
    scan_context_config.enabled = true;
    scan_context_config.level_with_odom_roll_pitch = true;
    scan_context_config.num_rings = 30;
    scan_context_config.num_sectors = 90;
    scan_context_config.max_radius = 80.0;
    scan_context_config.downsample = 0.6;
    scan_context_config.lidar_height = sensor_z;
    scan_context_config.min_bin_points = 1;
    scan_context_config.augment_count = 3;
    scan_context_config.lateral_shift = 2.0;
    scan_context_config.search_ratio = 1.0;
    scan_context_config.exclude_recent = 15;
    scan_context_config.candidates = 15;
    scan_context_config.distance_threshold = 0.6;
}

/// 输出扫描采样、模拟几何、描述子及目录写出选项。
void printUsage(const char* program) {
    const std::string tool_name =
        program != nullptr ? fs::path(program).filename().string() : "build_pcd_map_bundle";
    std::cout << "Usage:\n"
              << "  " << tool_name
              << " --input <global_map.pcd> "
                 "[--out-samples dir] [--out-bundle dir] [options]\n\n"
              << "Pose grid options:\n"
              << "  --x-min m --x-max m --y-min m --y-max m --sensor-z m\n"
              << "  --query-step m --bundle-step m\n"
              << "  --query-yaws deg[,deg...] --bundle-yaws deg[,deg...]\n"
              << "  --max-samples n --max-keyframes n\n\n"
              << "Scan model options:\n"
              << "  --map-voxel m --scan-voxel m\n"
              << "  --min-distance m --max-distance m\n"
              << "  --fov-down deg --fov-up deg\n"
              << "  --azimuth-bins n --elevation-bins n --occlusion true|false\n\n"
              << "Bundle output options:\n"
              << "  --write-map-pcd true|false\n\n"
              << "Descriptor options:\n"
              << "  --descriptor-type scan_context|gprobe|both (default: "
                 "scan_context)\n"
              << "  --sc-enabled true|false --sc-rings n --sc-sectors n\n"
              << "  --sc-max-radius m --sc-downsample m --sc-lidar-height m\n"
              << "  --sc-min-bin-points n --sc-augment-count n\n"
              << "  --sc-lateral-shift m --sc-search-ratio r --sc-level true|false\n"
              << "  --sc-exclude-recent n --sc-candidates n "
                 "--sc-distance-threshold d\n"
              << "  --gprobe-enabled true|false --gprobe-rings n "
                 "--gprobe-sectors n\n"
              << "  --gprobe-max-range m --gprobe-voxel-size m "
                 "--gprobe-sigma-t m\n"
              << "  --gprobe-query-fov deg --gprobe-query-fov-center deg\n"
              << "  --gprobe-branch-top-k n --gprobe-verification-top-k n\n"
              << "  --gprobe-spatial-nms-radius m\n"
              << "  --gprobe-heading-window deg --gprobe-column-trim ratio\n"
              << "  --gprobe-sgrt-kappa value --gprobe-level true|false\n"
              << "  --gprobe-exclude-recent n --gprobe-candidates n\n\n"
              << "Output directories must be absent or empty; existing data is never "
                 "deleted.\n";
}

/// 注册采样和描述子选项，校验必需路径并规范化样本数量与雷达高度。
bool parseArgs(const int argc, char** argv, BuildOptions& args) {
    ArgumentTable table;
    registerSamplingOptions(&table, &args);
    registerScanContextOptions(&table, &args);
    registerGProbeOptions(&table, &args);
    if (!table.parse(argc, argv, args.help)) {
        return false;
    }
    args.max_samples = std::max(0, args.max_samples);
    args.max_keyframes = std::max(0, args.max_keyframes);
    if (!args.scan_context_lidar_height_explicit) {
        args.scan_context_config.lidar_height = args.sensor_z;
    }
    return args.help ||
           (!args.input_pcd.empty() && (!args.out_samples.empty() || !args.out_bundle.empty()));
}

namespace {
/// 验证输出与扫描几何后生成地图包和独立样本，并返回过程退出码。
int run(const BuildOptions& args) {
    if (!ensureEmptyOutputDir(args.out_samples) || !ensureEmptyOutputDir(args.out_bundle)) {
        return 1;
    }
    if (args.x_max < args.x_min || args.y_max < args.y_min ||
        args.max_distance <= args.min_distance || args.fov_up_deg <= args.fov_down_deg ||
        args.azimuth_bins <= 0 || args.elevation_bins <= 0) {
        std::cerr << "Invalid simulation geometry arguments\n";
        return 1;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr reference_raw = loadReferenceMap(args.input_pcd);
    if (!reference_raw || reference_raw->empty()) {
        std::cerr << "Failed to load reference PCD " << args.input_pcd << "\n";
        return 1;
    }
    printBounds("reference_raw", *reference_raw);
    pcl::PointCloud<pcl::PointXYZ>::Ptr reference =
        voxelizeReference(*reference_raw, args.map_voxel);
    printBounds("reference_voxelized", *reference);

    if (!args.out_bundle.empty()) {
        const std::vector<Eigen::Matrix4d> bundle_poses =
            makePoseGrid(args, args.bundle_step, args.bundle_yaws_deg, args.max_keyframes);
        std::cout << "Bundle pose count=" << bundle_poses.size() << "\n";
        std::vector<GeneratedScan> keyframe_scans =
            generateScans(*reference, bundle_poses, args, 1000.0);
        if (!writeBundle(args.out_bundle, keyframe_scans, args.write_map_pcd, args.map_voxel,
                         args.scan_context_config, args.gprobe_config)) {
            return 1;
        }
    }

    if (!args.out_samples.empty()) {
        const std::vector<Eigen::Matrix4d> sample_poses =
            makePoseGrid(args, args.query_step, args.query_yaws_deg, args.max_samples);
        std::cout << "Sample pose count=" << sample_poses.size() << "\n";
        std::vector<GeneratedScan> sample_scans =
            generateScans(*reference, sample_poses, args, 2000.0);
        if (!writeSamples(args.out_samples, sample_scans)) {
            return 1;
        }
    }

    return 0;
}
}  // namespace

}  // namespace backend::tools::build

/// 解析采样配置并生成地图关键帧与独立扫描样本，返回操作退出码。
int main(int argc, char** argv) {
    try {
        backend::tools::build::BuildOptions args;
        if (!backend::tools::build::parseArgs(argc, argv, args)) {
            backend::tools::build::printUsage(argv[0]);
            return 1;
        }
        if (args.help) {
            backend::tools::build::printUsage(argv[0]);
            return 0;
        }
        return backend::tools::build::run(args);
    } catch (const std::exception& error) {
        std::cerr << "Map generation failed: " << error.what() << "\n";
        return 1;
    }
}
