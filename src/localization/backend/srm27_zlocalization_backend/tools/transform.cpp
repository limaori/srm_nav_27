/**
 * @file transform.cpp
 * @brief 解析坐标变换并重建对应地图包。
 */

#include "transform.hpp"

#include <pcl/io/pcd_io.h>

#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "arguments.hpp"
#include "backend/directory.hpp"
#include "backend/geometry.hpp"
#include "backend/map_bundle.hpp"
#include "backend/sampling.hpp"
#include "bundle.hpp"

namespace backend::tools::transform {

/// 输出基准点、显式位姿及目标地图配准三种模式的命令行说明。
void printUsage() {
    std::cout << "Usage:\n"
              << "  transform_map_bundle --in <bundle_dir> --out <bundle_dir> \\\n"
              << "    --origin x y z --x-axis x y z --y-axis x y z --z-axis x y z\n"
              << "  transform_map_bundle --in <bundle_dir> --out <bundle_dir> \\\n"
              << "    --target-map <correct_frame.pcd> [--source-map <source.pcd>] "
                 "\\\n"
              << "    [--align-voxel 0.2] [--align-max-correspondence 3.0] \\\n"
              << "    [--align-rmse-threshold 0.3] [--align-min-overlap 0.4] \\\n"
              << "    [--initial-yaw-deg 180]\n"
              << "  transform_map_bundle --in <bundle_dir> --out <bundle_dir> \\\n"
              << "    --translation tx ty tz --quaternion qx qy qz qw\n\n"
              << "Four-point mode:\n"
              << "  The four points are given in the old map frame.\n"
              << "  After transformation, origin becomes the new (0,0,0),\n"
              << "  and the x/y/z-axis points define the positive new axes.\n\n"
              << "Manual transform mode:\n"
              << "  Applies the exact source->target transform from translation and\n"
              << "  quaternion. --tx/--ty/--tz and --qx/--qy/--qz/--qw are also\n"
              << "  accepted as scalar aliases.\n\n"
              << "Target-map mode:\n"
              << "  The input bundle map is registered to the target PCD, then the\n"
              << "  estimated rigid transform is applied to keyframes, map.pcd,\n"
              << "  and the bundle descriptor.\n"
              << "  One prepared adaptive point-to-plane kernel is used at every\n"
              << "  resolution. --align-voxel 0 keeps the final pass at full point\n"
              << "  resolution. Use --initial-yaw-deg to skip the full yaw sweep.\n";
}

namespace {
/// 绑定地图路径、显式位姿、配准选项及成组坐标。
void registerTransformOptions(cli::Arguments& options, TransformOptions* args) {
    options.value({"--in"}, args->input_dir);
    options.value({"--out"}, args->output_dir);
    options.value({"--tx"}, args->manual_translation.x(), {&args->has_tx});
    options.value({"--ty"}, args->manual_translation.y(), {&args->has_ty});
    options.value({"--tz"}, args->manual_translation.z(), {&args->has_tz});
    options.value({"--qx"}, args->manual_qx, {&args->has_qx});
    options.value({"--qy"}, args->manual_qy, {&args->has_qy});
    options.value({"--qz"}, args->manual_qz, {&args->has_qz});
    options.value({"--qw"}, args->manual_qw, {&args->has_qw});
    options.value({"--target-map"}, args->target_map);
    options.value({"--source-map"}, args->source_map, {&args->has_alignment_options});
    options.value({"--align-voxel"}, args->alignment_voxel_resolution,
                  {&args->has_alignment_options});
    options.value({"--align-max-correspondence"}, args->alignment_max_correspondence_distance,
                  {&args->has_alignment_options});
    options.value({"--align-overlap-distance"}, args->alignment_overlap_distance,
                  {&args->has_alignment_options});
    options.value({"--align-min-overlap"}, args->alignment_min_overlap,
                  {&args->has_alignment_options});
    options.value({"--align-rmse-threshold"}, args->alignment_rmse_threshold,
                  {&args->has_alignment_options});
    options.value({"--align-max-iterations"}, args->alignment_max_iterations,
                  {&args->has_alignment_options});
    options.value({"--yaw-search-step-deg"}, args->alignment_yaw_search_step_deg,
                  {&args->has_alignment_options});
    options.value({"--initial-yaw-deg"}, args->initial_yaw_deg,
                  {&args->has_initial_yaw, &args->has_alignment_options});
    options.sequence({"--origin"}, {&args->origin.x(), &args->origin.y(), &args->origin.z()},
                     {&args->has_origin});
    options.sequence({"--x-axis"},
                     {&args->x_axis_point.x(), &args->x_axis_point.y(), &args->x_axis_point.z()},
                     {&args->has_x_axis});
    options.sequence({"--y-axis"},
                     {&args->y_axis_point.x(), &args->y_axis_point.y(), &args->y_axis_point.z()},
                     {&args->has_y_axis});
    options.sequence({"--z-axis"},
                     {&args->z_axis_point.x(), &args->z_axis_point.y(), &args->z_axis_point.z()},
                     {&args->has_z_axis});
    options.sequence({"--translation", "--t"},
                     {&args->manual_translation.x(), &args->manual_translation.y(),
                      &args->manual_translation.z()},
                     {&args->has_manual_translation, &args->has_tx, &args->has_ty, &args->has_tz});
    options.sequence(
        {"--quaternion", "--q"},
        {&args->manual_qx, &args->manual_qy, &args->manual_qz, &args->manual_qw},
        {&args->has_manual_quaternion, &args->has_qx, &args->has_qy, &args->has_qz, &args->has_qw});
    options.flag({"--verbose-alignment"}, args->verbose_alignment, &args->has_alignment_options);
}

/// 验证地图配准的分辨率、距离、重叠率、迭代和偏航数值域。
bool validAlignmentOptions(const TransformOptions& args) {
    return args.alignment_voxel_resolution >= 0.0 &&
           std::isfinite(args.alignment_voxel_resolution) &&
           args.alignment_max_correspondence_distance > 0.0 &&
           std::isfinite(args.alignment_max_correspondence_distance) &&
           args.alignment_overlap_distance > 0.0 &&
           std::isfinite(args.alignment_overlap_distance) && args.alignment_min_overlap >= 0.0 &&
           args.alignment_min_overlap <= 1.0 && std::isfinite(args.alignment_min_overlap) &&
           args.alignment_max_iterations > 0 && args.alignment_rmse_threshold > 0.0 &&
           std::isfinite(args.alignment_rmse_threshold) &&
           args.alignment_yaw_search_step_deg >= 0.0 &&
           std::isfinite(args.alignment_yaw_search_step_deg) &&
           (!args.has_initial_yaw || std::isfinite(args.initial_yaw_deg));
}

/// 检查必需目录、位姿完整性及三种互斥变换模式。
bool validateTransformMode(TransformOptions* args) {
    if (args->input_dir.empty() || args->output_dir.empty()) {
        return false;
    }

    const bool has_frame_args =
        args->has_origin || args->has_x_axis || args->has_y_axis || args->has_z_axis;
    args->has_manual_translation = args->has_tx && args->has_ty && args->has_tz;
    args->has_manual_quaternion = args->has_qx && args->has_qy && args->has_qz && args->has_qw;
    const bool has_partial_manual_translation =
        (args->has_tx || args->has_ty || args->has_tz) && !args->has_manual_translation;
    const bool has_partial_manual_quaternion =
        (args->has_qx || args->has_qy || args->has_qz || args->has_qw) &&
        !args->has_manual_quaternion;
    const bool has_manual_args = args->has_manual_translation || args->has_manual_quaternion ||
                                 has_partial_manual_translation || has_partial_manual_quaternion;

    if (has_partial_manual_translation) {
        std::cerr << "Manual transform mode needs all of --tx --ty --tz, "
                     "or --translation x y z\n";
        return false;
    }
    if (has_partial_manual_quaternion) {
        std::cerr << "Manual transform mode needs all of --qx --qy --qz --qw, "
                     "or --quaternion qx qy qz qw\n";
        return false;
    }
    if (args->has_manual_translation != args->has_manual_quaternion) {
        std::cerr << "Manual transform mode needs both translation and "
                     "quaternion\n";
        return false;
    }

    if (!args->target_map.empty()) {
        if (has_frame_args || has_manual_args) {
            std::cerr << "--target-map cannot be combined with "
                         "four-point or manual transform arguments\n";
            return false;
        }
        return validAlignmentOptions(*args);
    }

    if (args->has_alignment_options) {
        std::cerr << "Alignment options require --target-map\n";
        return false;
    }

    if (has_manual_args) {
        if (has_frame_args) {
            std::cerr << "Manual transform arguments cannot be combined with "
                         "--origin/--x-axis/--y-axis/--z-axis\n";
            return false;
        }
        return true;
    }

    return args->has_origin && args->has_x_axis && args->has_y_axis && args->has_z_axis;
}
}  // namespace

/// 解析变换参数并检验模式互斥、显式位姿完整性及配准数值范围。
bool parseArgs(int argc, char** argv, TransformOptions& args) {
    cli::Arguments options;
    registerTransformOptions(options, &args);
    if (!options.parse(argc, argv, args.help))
        return false;
    return args.help || validateTransformMode(&args);
}

namespace fs = std::filesystem;

/// 检查文件名是否以给定后缀结尾。
bool hasSuffix(const std::string& value, const std::string& suffix) {
    return value.size() >= suffix.size() &&
           value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0;
}

/// 确认路径为目录，缺失时创建其父级及目标目录。
bool ensureDirectory(const fs::path& path) {
    std::error_code ec;
    if (fs::exists(path, ec)) {
        return fs::is_directory(path, ec);
    }
    return fs::create_directories(path, ec);
}

/// 复制无需重建的附属文件，跳过关键帧点云、位姿索引及描述子产物。
bool copyPassthroughFiles(const fs::path& input_dir, const fs::path& output_dir) {
    std::error_code ec;
    for (fs::recursive_directory_iterator it(input_dir, ec), end; it != end; it.increment(ec)) {
        if (ec) {
            std::cerr << "Directory traversal failed: " << ec.message() << "\n";
            return false;
        }

        const fs::path rel = fs::relative(it->path(), input_dir, ec);
        if (ec) {
            std::cerr << "Path relative() failed: " << ec.message() << "\n";
            return false;
        }
        if (rel.empty()) {
            continue;
        }

        const fs::path out_path = output_dir / rel;
        if (it->is_directory()) {
            if (rel == "clouds") {
                it.disable_recursion_pending();
                continue;
            }
            if (!ensureDirectory(out_path)) {
                std::cerr << "Failed to create directory: " << out_path << "\n";
                return false;
            }
            continue;
        }

        const std::string filename = rel.filename().string();
        if (filename == "keyframes.txt" || hasSuffix(filename, "_descriptors.bin") ||
            filename == "map.pcd") {
            continue;
        }
        if (rel.parent_path() == "clouds") {
            continue;
        }

        if (!ensureDirectory(out_path.parent_path())) {
            std::cerr << "Failed to create parent directory: " << out_path.parent_path() << "\n";
            return false;
        }
        fs::copy_file(it->path(), out_path, fs::copy_options::overwrite_existing, ec);
        if (ec) {
            std::cerr << "Failed to copy file " << it->path() << ": " << ec.message() << "\n";
            return false;
        }
    }
    return true;
}

/// 在全局地图 PCD 存在时应用同一刚体变换并写入输出目录。
bool transformGlobalMapIfPresent(const std::string& input_dir, const std::string& output_dir,
                                 const Eigen::Matrix4d& t_new_old) {
    const fs::path in_path = fs::path(input_dir) / "map.pcd";
    if (!fs::exists(in_path)) {
        return true;
    }

    Cloud map;
    if (pcl::io::loadPCDFile<PointType>(in_path.string(), map) != 0) {
        std::cerr << "Failed to read " << in_path << "\n";
        return false;
    }
    Cloud::Ptr transformed = transformCloud(map, t_new_old);
    const fs::path out_path = fs::path(output_dir) / "map.pcd";
    if (pcl::io::savePCDFileBinary(out_path.string(), *transformed) != 0) {
        std::cerr << "Failed to write " << out_path << "\n";
        return false;
    }
    return true;
}

namespace {
/// 在独占目录事务中写出变换后的关键帧、描述子、附属文件和可选全局地图。
int writeTransformedBundle(const TransformOptions& args, const std::vector<Keyframe>& keyframes,
                           const Eigen::Matrix4d& t_new_old, const DescriptorConfigs& configs) {
    backend::DirectoryTransaction output(args.output_dir);
    std::string output_error;
    if (!output.prepare(&output_error)) {
        std::cerr << "Failed to prepare output directory " << args.output_dir << ": "
                  << output_error << "\n";
        return 1;
    }
    const std::string staged_output = output.stagingPath().string();
    if (!copyPassthroughFiles(args.input_dir, staged_output)) {
        return 1;
    }
    writeStagedBundle(staged_output, keyframes, configs);
    if (!transformGlobalMapIfPresent(args.input_dir, staged_output, t_new_old)) {
        return 1;
    }
    if (!output.commit(&output_error)) {
        std::cerr << "Failed to commit output directory " << args.output_dir << ": " << output_error
                  << "\n";
        return 1;
    }

    std::cout << "Bundle transformed successfully.\n";
    std::cout << "Input : " << args.input_dir << "\n";
    std::cout << "Output: " << args.output_dir << "\n";
    return 0;
}

/// 校验输入输出位置并变换一致关键帧快照，随后提交完整地图包。
int run(const TransformOptions& args) {
    validateOutputLocation(args.input_dir, args.output_dir);
    const auto configs = loadDescriptorConfigs(args.input_dir);
    if (configs.empty())
        throw std::runtime_error("input bundle has no supported descriptor file");

    backend::KeyframeMap manager;
    if (!manager.load(args.input_dir)) {
        std::cerr << "Failed to load keyframes from " << args.input_dir << "\n";
        return 1;
    }

    std::vector<backend::Keyframe> keyframes = manager.snapshot();
    Eigen::Matrix4d t_new_old = Eigen::Matrix4d::Identity();
    if (!args.target_map.empty()) {
        t_new_old = buildTargetMapTransform(args, keyframes);
    } else if (args.has_manual_translation && args.has_manual_quaternion) {
        t_new_old = buildManualTransform(args);
    } else {
        t_new_old = buildNewFromOldTransform(args);
    }
    printTransformSummary(t_new_old, "Transform source -> target:");
    for (auto& kf : keyframes) {
        kf.odom_pose = t_new_old * kf.odom_pose;
        kf.optimized_pose = t_new_old * kf.optimized_pose;
    }

    return writeTransformedBundle(args, keyframes, t_new_old, configs);
}
}  // namespace

}  // namespace backend::tools::transform

/// 构造地图刚体变换并在目录事务中保存全部变换产物。
int main(int argc, char** argv) {
    try {
        backend::tools::transform::TransformOptions args;
        if (!backend::tools::transform::parseArgs(argc, argv, args)) {
            backend::tools::transform::printUsage();
            return 1;
        }
        if (args.help) {
            backend::tools::transform::printUsage();
            return 0;
        }
        return backend::tools::transform::run(args);
    } catch (const std::exception& error) {
        std::cerr << "Map transformation failed: " << error.what() << '\n';
        return 1;
    }
}
