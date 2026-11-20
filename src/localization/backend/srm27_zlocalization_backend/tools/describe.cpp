/**
 * @file describe.cpp
 * @brief 读取地图包并按指定配置重建 Scan Context 描述子。
 */

#include <iostream>
#include <string>

#include "arguments.hpp"
#include "backend/geometry.hpp"
#include "backend/keyframe_map.hpp"
#include "backend/scan_context.hpp"

namespace backend::tools::describe {
namespace {

struct DescribeOptions {
    std::string bundle_dir;
    ScanContextDescriptorConfig config;
    bool help = false;
};

/// 输出地图包位置及 Scan Context 重建配置的命令行说明。
void printUsage() {
    std::cout << "Usage:\n"
              << "  rebuild_scan_context_descriptors --bundle <map_bundle_dir> "
                 "[options]\n\n"
              << "Options:\n"
              << "  --enabled true|false --level true|false\n"
              << "  --num-rings n --num-sectors n --max-radius m\n"
              << "  --downsample m --lidar-height m --min-bin-points n\n"
              << "  --augment-count n --lateral-shift m --search-ratio r\n"
              << "  --exclude-recent n --candidates n --distance-threshold d\n";
}

/// 读取地图包目录与 Scan Context 配置，拒绝未知或缺失选项。
bool parseArgs(int argc, char** argv, DescribeOptions& args) {
    cli::Arguments options;
    options.value({"--bundle"}, args.bundle_dir);
    options.value({"--enabled"}, args.config.enabled);
    options.value({"--level"}, args.config.level_with_odom_roll_pitch);
    options.value({"--num-rings"}, args.config.num_rings);
    options.value({"--num-sectors"}, args.config.num_sectors);
    options.value({"--max-radius"}, args.config.max_radius);
    options.value({"--downsample"}, args.config.downsample);
    options.value({"--lidar-height"}, args.config.lidar_height);
    options.value({"--min-bin-points"}, args.config.min_bin_points);
    options.value({"--augment-count"}, args.config.augment_count);
    options.value({"--lateral-shift"}, args.config.lateral_shift);
    options.value({"--search-ratio"}, args.config.search_ratio);
    options.value({"--exclude-recent"}, args.config.exclude_recent);
    options.value({"--candidates"}, args.config.candidates);
    options.value({"--distance-threshold"}, args.config.distance_threshold);
    return options.parse(argc, argv, args.help) && (args.help || !args.bundle_dir.empty());
}

/// 读取固定关键帧快照并按指定配置重建描述子。
int run(const DescribeOptions& args) {
    backend::KeyframeMap keyframes;
    if (!keyframes.load(args.bundle_dir)) {
        std::cerr << "Failed to load keyframes from " << args.bundle_dir << "\n";
        return 1;
    }

    backend::ScanContextDescriptor descriptor;
    descriptor.configure(args.config);
    backend::rebuildMapDescriptors(descriptor, keyframes.snapshot());

    if (!descriptor.save(args.bundle_dir)) {
        std::cerr << "Failed to write sc_descriptors.bin under " << args.bundle_dir << "\n";
        return 1;
    }

    std::cout << "Wrote " << descriptor.size() << " Scan Context descriptors to " << args.bundle_dir
              << "\n";
    return 0;
}

}  // namespace
}  // namespace backend::tools::describe

/// 读取关键帧并重建 Scan Context 描述子，保存失败时返回非零退出码。
int main(int argc, char** argv) {
    try {
        backend::tools::describe::DescribeOptions args;
        if (!backend::tools::describe::parseArgs(argc, argv, args)) {
            backend::tools::describe::printUsage();
            return 1;
        }
        if (args.help) {
            backend::tools::describe::printUsage();
            return 0;
        }

        return backend::tools::describe::run(args);
    } catch (const std::exception& error) {
        std::cerr << "Failed to rebuild Scan Context descriptors: " << error.what() << '\n';
        return 1;
    }
}
