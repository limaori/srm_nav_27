/**
 * @file cleanup.cpp
 * @brief 从真实关键帧地图包独立执行动态清理并提交完整的新地图包。
 */
#include "cleanup/cleanup.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>

#include "arguments.hpp"
#include "backend/directory.hpp"
#include "backend/keyframe_map.hpp"
#include "backend/map_bundle.hpp"
#include "bundle.hpp"

namespace backend::tools::cleanup {
namespace {
/// 独立地图清理的输入输出路径和算法选项。
struct CleanupOptions {
    std::string input, output;
    Config cleanup;
    double map_voxel = 0.15;
    bool help = false;
};

/// 输出地图包、雷达原点及清理参数的命令行说明。
void printUsage() {
    std::cout
        << "Usage:\n  clean_map_bundle --in <raw_bundle> --out <clean_bundle> [options]\n\n"
        << "Input requires recorded keyframe clouds and sensor poses, not a fused PCD.\n"
        << "Options:\n"
        << "  --lidar-origin x y z   LiDAR origin in the keyframe cloud frame (default: 0 0 0)\n"
        << "  --voxel-resolution m  Free-space cell size (default: 0.2)\n"
        << "  --raycast-range m     Evidence range, without cropping output (default: 20)\n"
        << "  --free-observations n Free-space confirmation count (default: 3)\n"
        << "  --threads n           CPU workers (default: 4)\n"
        << "  --map-voxel m         Global PCD resolution; 0 keeps all points (default: 0.15)\n"
        << "Output must be absent or empty, outside the input directory.\n";
}

/// 解析清理参数并检查必需的输入输出路径。
bool parseArgs(int argc, char** argv, CleanupOptions& args) {
    cli::Arguments options;
    options.value({"--in", "--input"}, args.input);
    options.value({"--out", "--output"}, args.output);
    options.value({"--voxel-resolution"}, args.cleanup.voxel_resolution);
    options.value({"--raycast-range"}, args.cleanup.raycast_range);
    options.value({"--free-observations"}, args.cleanup.free_observations);
    options.value({"--threads"}, args.cleanup.threads);
    options.value({"--map-voxel"}, args.map_voxel);
    options.sequence({"--lidar-origin"},
                     {&args.cleanup.lidar_origin.x(), &args.cleanup.lidar_origin.y(),
                      &args.cleanup.lidar_origin.z()},
                     {});
    return options.parse(argc, argv, args.help) &&
           (args.help || (!args.input.empty() && !args.output.empty()));
}

/// 校验清理数值配置及输出目录与输入地图包的隔离关系。
void validateOptions(const CleanupOptions& args) {
    validateConfig(args.cleanup);
    if (!std::isfinite(args.map_voxel) || args.map_voxel < 0)
        throw std::invalid_argument("map voxel resolution must be finite and nonnegative");
    validateOutputLocation(args.input, args.output);
}

/// 清理固定地图快照，并在全部派生产物写出后提交新地图包。
int run(const CleanupOptions& args) {
    validateOptions(args);
    DirectoryTransaction output(args.output);
    std::string error;
    if (!output.prepare(&error))
        throw std::runtime_error("cannot prepare output: " + error);
    KeyframeMap map;
    if (!map.load(args.input))
        throw std::runtime_error("cannot load input keyframes");
    const auto descriptors = loadDescriptorConfigs(args.input);
    const auto result = cleanFrames(
        map.snapshot(), args.cleanup, [](const char* phase, std::size_t done, std::size_t total) {
            if (done == 1 || done == total || done % 100 == 0)
                std::cout << phase << ' ' << done << '/' << total << '\n';
        });
    const auto staging = output.stagingPath().string();
    writeStagedBundle(staging, result.frames, descriptors);
    if (!saveMapCloud((output.stagingPath() / "map.pcd").string(), result.frames, args.map_voxel))
        throw std::runtime_error("cannot write cleaned map bundle");
    saveReport(staging, args.cleanup, result);
    if (!output.commit(&error))
        throw std::runtime_error("cannot commit output: " + error);
    std::cout << "Wrote cleaned bundle to " << args.output << ": "
              << result.statistics.removed_points << '/' << result.statistics.input_points
              << " points removed, " << result.statistics.unclassified_points
              << " unknown retained\n";
    return 0;
}
}  // namespace
}  // namespace backend::tools::cleanup

/// 处理帮助请求、执行清理并将异常转换为失败退出码。
int main(int argc, char** argv) {
    try {
        backend::tools::cleanup::CleanupOptions args;
        if (!backend::tools::cleanup::parseArgs(argc, argv, args)) {
            backend::tools::cleanup::printUsage();
            return 1;
        }
        if (args.help) {
            backend::tools::cleanup::printUsage();
            return 0;
        }
        return backend::tools::cleanup::run(args);
    } catch (const std::exception& error) {
        std::cerr << "Map cleanup failed: " << error.what() << '\n';
        return 1;
    }
}
