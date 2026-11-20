/**
 * @file filter.cpp
 * @brief 按参考地图过滤关键帧并重建完整地图包。
 */

#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <exception>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "arguments.hpp"
#include "backend/directory.hpp"
#include "backend/geometry.hpp"
#include "backend/keyframe_map.hpp"
#include "bundle.hpp"

namespace backend::tools::filter {

/// 参考地图过滤与未采样参考图导出的命令行参数。
struct FilterOptions {
    std::string input_dir;
    std::string output_dir;
    std::string reference_map;
    std::string export_reference_map;
    double keep_distance = 0.2;
    double match_epsilon = 1e-5;
    double map_voxel = 0.15;
    int min_points_per_keyframe = 20;
    bool exact = false;
    bool help = false;
};

/// 输出地图过滤、参考图导出及精确匹配模式的命令行说明。
void printUsage() {
    std::cout << "Usage:\n"
              << "  filter_map_bundle --in <raw_bundle_dir> --out "
                 "<clean_bundle_dir>\n"
              << "    --reference-map <cleaned_map.pcd> [--keep-distance meters]\n"
              << "    [--exact] [--match-epsilon meters]\n"
              << "    [--map-voxel meters] [--min-points-per-keyframe n]\n\n"
              << "  filter_map_bundle --in <raw_bundle_dir>\n"
              << "    --export-reference-map <unvoxelized_map.pcd>\n\n"
              << "Workflow:\n"
              << "  1. Save a mapping bundle with map.pcd enabled.\n"
              << "  2. Edit/crop map.pcd in CloudCompare and save it as "
                 "cleaned_map.pcd.\n"
              << "  3. Run this tool. It keeps only keyframe points that are close "
                 "to the\n"
              << "     cleaned global map, then rebuilds the bundle descriptor.\n\n"
              << "Exact mode:\n"
              << "  --exact uses --match-epsilon instead of --keep-distance. It only "
                 "makes\n"
              << "  sense when cleaned_map.pcd was exported from an unvoxelized "
                 "global map\n"
              << "  whose points are the original transformed keyframe points.\n";
}

/// 解析参考图导出或地图过滤参数，并校验所选模式的必需输入。
bool parseArgs(const int argc, char** argv, FilterOptions& args) {
    cli::Arguments options;
    options.value({"--in"}, args.input_dir);
    options.value({"--out"}, args.output_dir);
    options.value({"--reference-map"}, args.reference_map);
    options.value({"--export-reference-map"}, args.export_reference_map);
    options.value({"--keep-distance"}, args.keep_distance);
    options.value({"--match-epsilon"}, args.match_epsilon);
    options.value({"--map-voxel"}, args.map_voxel);
    options.value({"--min-points-per-keyframe"}, args.min_points_per_keyframe);
    options.flag({"--exact"}, args.exact);
    if (!options.parse(argc, argv, args.help))
        return false;

    if (args.help) {
        return true;
    }
    if (!args.export_reference_map.empty()) {
        return !args.input_dir.empty();
    }
    return !args.input_dir.empty() && !args.output_dir.empty() && !args.reference_map.empty() &&
           args.keep_distance > 0.0 && args.match_epsilon >= 0.0 &&
           args.min_points_per_keyframe >= 0;
}

namespace {

namespace fs = std::filesystem;

/// 确认路径为目录，缺失时创建其父级及目标目录。
bool ensureDirectory(const fs::path& path) {
    std::error_code error;
    if (fs::exists(path, error)) {
        return fs::is_directory(path, error);
    }
    return fs::create_directories(path, error);
}

/// 创建输出父目录并写出指定分辨率的全局地图 PCD。
bool saveGlobalMapFile(const std::string& path, const std::vector<Keyframe>& keyframes,
                       const double voxel_resolution) {
    const fs::path output_path(path);
    if (output_path.has_parent_path() && !ensureDirectory(output_path.parent_path())) {
        std::cerr << "Cannot create output directory for " << output_path << "\n";
        return false;
    }
    const Cloud::Ptr map = KeyframeMap::buildOptimizedMapFromSnapshot(keyframes, voxel_resolution);
    if (pcl::io::savePCDFileBinary(output_path.string(), *map) != 0) {
        std::cerr << "Failed to write " << output_path << "\n";
        return false;
    }
    std::cout << "Wrote " << output_path << " (" << map->size() << " points)\n";
    return true;
}

/// 将全局地图写入地图包约定的 map.pcd 路径。
bool saveGlobalMap(const std::string& directory, const std::vector<Keyframe>& keyframes,
                   const double voxel_resolution) {
    return saveGlobalMapFile((fs::path(directory) / "map.pcd").string(), keyframes,
                             voxel_resolution);
}

/// 保留靠近参考地图的关键帧局部点，并剔除点数不足的帧及重排索引。
std::vector<Keyframe> filterKeyframesByReferenceMap(const std::vector<Keyframe>& input,
                                                    const Cloud::ConstPtr& reference_map,
                                                    const double match_distance,
                                                    const int min_points_per_keyframe,
                                                    const bool exact_mode) {
    pcl::KdTreeFLANN<PointType> tree;
    tree.setInputCloud(reference_map);
    const float max_distance2 = static_cast<float>(match_distance * match_distance);

    std::vector<Keyframe> output;
    output.reserve(input.size());
    std::size_t input_points = 0;
    std::size_t kept_points = 0;
    int dropped_keyframes = 0;
    for (const Keyframe& keyframe : input) {
        const auto geometry_cloud = keyframe.cloud.load();
        if (!geometry_cloud) {
            ++dropped_keyframes;
            continue;
        }
        Cloud::Ptr filtered(new Cloud);
        filtered->reserve(geometry_cloud->size());
        input_points += geometry_cloud->size();

        for (const PointType& point : geometry_cloud->points) {
            if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
                continue;
            }
            const Eigen::Vector4d local(point.x, point.y, point.z, 1.0);
            const Eigen::Vector4d map = keyframe.optimized_pose * local;
            PointType query;
            query.x = static_cast<float>(map.x());
            query.y = static_cast<float>(map.y());
            query.z = static_cast<float>(map.z());

            std::vector<int> indices(1);
            std::vector<float> distances2(1);
            if (tree.nearestKSearch(query, 1, indices, distances2) > 0 &&
                distances2[0] <= max_distance2) {
                filtered->push_back(point);
            }
        }

        kept_points += filtered->size();
        if (static_cast<int>(filtered->size()) < min_points_per_keyframe) {
            ++dropped_keyframes;
            continue;
        }
        Keyframe kept = keyframe;
        kept.index = static_cast<int>(output.size());
        kept.cloud = CloudBlock(filtered);
        output.push_back(kept);
    }

    std::cout << "Filtered keyframes: " << output.size() << " / " << input.size() << " kept, "
              << dropped_keyframes << " dropped\n";
    std::cout << "Filtered points: " << kept_points << " / " << input_points << " kept ("
              << (exact_mode ? "match_epsilon" : "keep_distance") << "=" << match_distance
              << " m)\n";
    return output;
}

/// 读取非空参考地图点云并报告点数，读取失败时返回失败状态。
bool loadReferenceMap(const std::string& path, Cloud::Ptr* reference) {
    if (reference == nullptr) {
        return false;
    }
    reference->reset(new Cloud);
    if (pcl::io::loadPCDFile<PointType>(path, **reference) != 0 || (*reference)->empty()) {
        std::cerr << "Failed to read non-empty reference map: " << path << "\n";
        return false;
    }
    std::cout << "Loaded reference map " << path << " with " << (*reference)->size() << " points\n";
    return true;
}

}  // namespace

/// 执行未降采样参考图导出，或过滤几何并事务提交完整地图包。
int run(const FilterOptions& args) {
    KeyframeMap manager;
    if (!manager.load(args.input_dir)) {
        std::cerr << "Failed to load input bundle: " << args.input_dir << "\n";
        return 1;
    }
    const std::vector<Keyframe> input_keyframes = manager.snapshot();
    if (!args.export_reference_map.empty()) {
        return saveGlobalMapFile(args.export_reference_map, input_keyframes,
                                 /*voxel_resolution=*/0.0)
                   ? 0
                   : 1;
    }
    validateOutputLocation(args.input_dir, args.output_dir);

    DirectoryTransaction output(args.output_dir);
    std::string output_error;
    if (!output.prepare(&output_error)) {
        std::cerr << "Failed to prepare output directory " << args.output_dir << ": "
                  << output_error << "\n";
        return 1;
    }
    Cloud::Ptr reference;
    const auto descriptors = loadDescriptorConfigs(args.input_dir);
    if (descriptors.empty())
        throw std::runtime_error("input bundle has no supported descriptor file");
    if (!loadReferenceMap(args.reference_map, &reference))
        return 1;

    const std::vector<Keyframe> filtered = filterKeyframesByReferenceMap(
        input_keyframes, reference, args.exact ? args.match_epsilon : args.keep_distance,
        args.min_points_per_keyframe, args.exact);
    if (filtered.empty()) {
        std::cerr << "Filtering removed all keyframes; increase --keep-distance "
                     "or check the reference map frame.\n";
        return 1;
    }
    writeStagedBundle(output.stagingPath().string(), filtered, descriptors);
    if (!saveGlobalMap(output.stagingPath().string(), filtered, args.map_voxel)) {
        return 1;
    }
    if (!output.commit(&output_error)) {
        std::cerr << "Failed to commit output directory " << args.output_dir << ": " << output_error
                  << "\n";
        return 1;
    }
    std::cout << "Wrote cleaned bundle to " << args.output_dir << "\n";
    return 0;
}

}  // namespace backend::tools::filter

/// 解析过滤选项，执行地图工作流并返回对应进程退出码。
int main(int argc, char** argv) {
    try {
        backend::tools::filter::FilterOptions args;
        if (!backend::tools::filter::parseArgs(argc, argv, args)) {
            backend::tools::filter::printUsage();
            return 1;
        }
        if (args.help) {
            backend::tools::filter::printUsage();
            return 0;
        }
        return backend::tools::filter::run(args);
    } catch (const std::exception& error) {
        std::cerr << "Map filtering failed: " << error.what() << '\n';
        return 1;
    }
}
