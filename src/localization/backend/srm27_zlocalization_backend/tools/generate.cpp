/**
 * @file generate.cpp
 * @brief 模拟局部激光扫描并保存样本或完整地图包。
 */

#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <vector>

#include "backend/directory.hpp"
#include "backend/geometry.hpp"
#include "backend/gprobe.hpp"
#include "backend/keyframe_map.hpp"
#include "backend/map_bundle.hpp"
#include "backend/math.hpp"
#include "backend/sampling.hpp"
#include "backend/scan_context.hpp"
#include "build.hpp"
#include "bundle.hpp"

namespace fs = std::filesystem;

namespace backend::tools::build {

/// 创建缺失的输出目录，拒绝非目录路径及已有内容。
bool ensureEmptyOutputDir(const std::string& path) {
    if (path.empty()) {
        return true;
    }
    const fs::path dir(path);
    if (fs::exists(dir) && !fs::is_directory(dir)) {
        std::cerr << dir << " exists and is not a directory\n";
        return false;
    }
    if (fs::exists(dir) && !fs::is_empty(dir)) {
        std::cerr << dir << " is not empty; refusing to overwrite\n";
        return false;
    }
    std::error_code ec;
    fs::create_directories(dir, ec);
    if (ec) {
        std::cerr << "Failed to create " << dir << ": " << ec.message() << "\n";
        return false;
    }
    return true;
}

/// 按地图系位置和以度表示的偏航角构造传感器位姿。
Eigen::Matrix4d poseFromXYYaw(const double x, const double y, const double z,
                              const double yaw_deg) {
    Eigen::Matrix4d pose = yawGuess(degreesToRadians(yaw_deg), Eigen::Vector3d(x, y, z));
    return pose;
}

/// 按平面网格次序和偏航集合生成有界传感器位姿序列。
std::vector<Eigen::Matrix4d> makePoseGrid(const BuildOptions& args, const double step,
                                          const std::vector<double>& yaws, const int max_count) {
    std::vector<Eigen::Matrix4d> poses;
    if (step <= 0.0 || yaws.empty()) {
        return poses;
    }
    for (double y = args.y_min; y <= args.y_max + 1.0e-9; y += step) {
        for (double x = args.x_min; x <= args.x_max + 1.0e-9; x += step) {
            for (const double yaw : yaws) {
                poses.push_back(poseFromXYYaw(x, y, args.sensor_z, yaw));
                if (max_count > 0 && static_cast<int>(poses.size()) >= max_count) {
                    return poses;
                }
            }
        }
    }
    return poses;
}

namespace {
/// 使用 PCL 体素质心规则采样指定点类型的点云。
template <class Point>
typename pcl::PointCloud<Point>::Ptr voxelizePoints(const pcl::PointCloud<Point>& cloud,
                                                    double leaf) {
    typename pcl::PointCloud<Point>::Ptr output(new pcl::PointCloud<Point>);
    if (cloud.empty())
        return output;
    if (leaf <= 0) {
        *output = cloud;
        return output;
    }
    pcl::VoxelGrid<Point> filter;
    filter.setLeafSize(static_cast<float>(leaf), static_cast<float>(leaf),
                       static_cast<float>(leaf));
    filter.setInputCloud(cloud.makeShared());
    filter.filter(*output);
    return output;
}
}  // namespace

/// 体素化带反射强度的局部扫描。
Cloud::Ptr voxelizeCloud(const Cloud& cloud, double leaf) {
    return voxelizePoints(cloud, leaf);
}

/// 读取作为扫描真值的全局 XYZ 点云，文件加载失败时返回空指针。
pcl::PointCloud<pcl::PointXYZ>::Ptr loadReferenceMap(const std::string& path) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, *cloud) != 0) {
        return {};
    }
    return cloud;
}

/// 体素化全局参考几何。
pcl::PointCloud<pcl::PointXYZ>::Ptr voxelizeReference(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                                                      double leaf) {
    return voxelizePoints(cloud, leaf);
}

/// 打印点云点数及三轴坐标范围。
void printBounds(const std::string& label, const pcl::PointCloud<pcl::PointXYZ>& cloud) {
    if (cloud.empty()) {
        std::cout << label << ": empty\n";
        return;
    }
    pcl::PointXYZ min_pt;
    pcl::PointXYZ max_pt;
    pcl::getMinMax3D(cloud, min_pt, max_pt);
    std::cout << label << ": points=" << cloud.size() << ", x=[" << min_pt.x << ", " << max_pt.x
              << "], y=[" << min_pt.y << ", " << max_pt.y << "], z=[" << min_pt.z << ", "
              << max_pt.z << "]\n";
}

/// 按距离、垂直视场及可选角度遮挡模型模拟传感器局部扫描。
Cloud::Ptr simulateScan(const pcl::PointCloud<pcl::PointXYZ>& global_map,
                        const Eigen::Matrix4d& t_map_sensor, const BuildOptions& args) {
    struct Bin {
        float range = std::numeric_limits<float>::infinity();
        PointType point;
    };

    const double min_r2 = args.min_distance * args.min_distance;
    const double max_r2 = args.max_distance * args.max_distance;
    const double fov_down = degreesToRadians(args.fov_down_deg);
    const double fov_up = degreesToRadians(args.fov_up_deg);
    const double fov_span = fov_up - fov_down;
    const Eigen::Matrix3d r_ms = t_map_sensor.block<3, 3>(0, 0);
    const Eigen::Matrix3d r_sm = r_ms.transpose();
    const Eigen::Vector3d t_ms = t_map_sensor.block<3, 1>(0, 3);

    Cloud raw;
    std::vector<Bin> bins;
    if (args.occlusion) {
        bins.resize(static_cast<std::size_t>(args.azimuth_bins * args.elevation_bins));
    }

    for (const auto& p : global_map.points) {
        const Eigen::Vector3d pm(p.x, p.y, p.z);
        const Eigen::Vector3d ps = r_sm * (pm - t_ms);
        const double r2 = ps.squaredNorm();
        if (r2 < min_r2 || r2 > max_r2) {
            continue;
        }
        const double xy = std::hypot(ps.x(), ps.y());
        const double elev = std::atan2(ps.z(), xy);
        if (elev < fov_down || elev > fov_up) {
            continue;
        }
        const double range = std::sqrt(r2);
        PointType out;
        out.x = static_cast<float>(ps.x());
        out.y = static_cast<float>(ps.y());
        out.z = static_cast<float>(ps.z());
        out.intensity = static_cast<float>(range);

        // 关闭遮挡时保留全部视场内点，开启时仅保留每个角度格的最近回波。
        if (!args.occlusion) {
            raw.push_back(out);
            continue;
        }

        double az = std::atan2(ps.y(), ps.x());
        if (az < -kPi) {
            az += 2.0 * kPi;
        }
        if (az >= kPi) {
            az -= 2.0 * kPi;
        }
        int az_idx = static_cast<int>(std::floor((az + kPi) / (2.0 * kPi) * args.azimuth_bins));
        int el_idx =
            static_cast<int>(std::floor((elev - fov_down) / fov_span * args.elevation_bins));
        az_idx = std::clamp(az_idx, 0, args.azimuth_bins - 1);
        el_idx = std::clamp(el_idx, 0, args.elevation_bins - 1);
        const std::size_t bin_idx = static_cast<std::size_t>(el_idx * args.azimuth_bins + az_idx);
        if (range < bins[bin_idx].range) {
            bins[bin_idx].range = static_cast<float>(range);
            bins[bin_idx].point = out;
        }
    }

    if (args.occlusion) {
        raw.reserve(bins.size());
        for (const auto& bin : bins) {
            if (std::isfinite(bin.range)) {
                raw.push_back(bin.point);
            }
        }
    }

    return voxelizeCloud(raw, args.scan_voxel);
}

/// 写出模拟扫描 PCD 和位姿 CSV，并检查文本关闭结果。
bool writeSamples(const std::string& directory, const std::vector<GeneratedScan>& scans) {
    if (directory.empty()) {
        return true;
    }
    const fs::path root(directory);
    const fs::path clouds = root / "clouds";
    std::error_code ec;
    fs::create_directories(clouds, ec);
    if (ec) {
        std::cerr << "Failed to create " << clouds << ": " << ec.message() << "\n";
        return false;
    }

    std::ofstream poses(root / "poses.csv");
    if (!poses.is_open()) {
        std::cerr << "Failed to write " << (root / "poses.csv") << "\n";
        return false;
    }
    poses << "index,stamp,map_tx,map_ty,map_tz,map_qx,map_qy,map_qz,map_qw,"
             "odom_tx,odom_ty,odom_tz,odom_qx,odom_qy,odom_qz,odom_qw,"
             "points\n";
    poses << std::fixed << std::setprecision(9);

    for (const auto& scan : scans) {
        const Eigen::Quaterniond q(scan.pose.block<3, 3>(0, 0));
        const Eigen::Vector3d t = scan.pose.block<3, 1>(0, 3);
        poses << scan.index << "," << scan.stamp << "," << t.x() << "," << t.y() << "," << t.z()
              << "," << q.x() << "," << q.y() << "," << q.z() << "," << q.w() << "," << t.x() << ","
              << t.y() << "," << t.z() << "," << q.x() << "," << q.y() << "," << q.z() << ","
              << q.w() << "," << (scan.cloud ? scan.cloud->size() : 0) << "\n";

        std::ostringstream name;
        name << std::setw(6) << std::setfill('0') << scan.index << ".pcd";
        const fs::path cloud_path = clouds / name.str();
        if (!scan.cloud || pcl::io::savePCDFileBinary(cloud_path.string(), *scan.cloud) != 0) {
            std::cerr << "Failed to write " << cloud_path << "\n";
            return false;
        }
    }
    poses.close();
    if (!poses) {
        std::cerr << "Failed to write " << (root / "poses.csv") << "\n";
        return false;
    }
    return true;
}

/// 在独占暂存目录写出关键帧、描述子及可选全局点云后提交地图包。
bool writeBundle(const std::string& directory, const std::vector<GeneratedScan>& scans,
                 const bool write_map_pcd, const double map_voxel,
                 const ScanContextDescriptorConfig& scan_context_config,
                 const GProbeDescriptorConfig& gprobe_config) {
    if (directory.empty()) {
        return true;
    }

    DirectoryTransaction output(directory);
    std::string output_error;
    if (!output.prepare(&output_error)) {
        std::cerr << "Cannot prepare map bundle: " << output_error << "\n";
        return false;
    }
    const std::string staged = output.stagingPath().string();
    /// 提交已经完整写入的地图包并报告目录提交失败原因。
    const auto commit = [&]() {
        if (output.commit(&output_error))
            return true;
        std::cerr << "Cannot commit map bundle: " << output_error << "\n";
        return false;
    };
    KeyframeMap manager;
    for (const auto& scan : scans) {
        if (!scan.cloud || scan.cloud->empty()) {
            std::cerr << "Refusing to write empty keyframe scan " << scan.index << "\n";
            return false;
        }
        FrontendFrame frame;
        frame.timestamp = scan.stamp;
        frame.odom_pose = scan.pose;
        frame.cloud_local = scan.cloud;
        manager.add(frame);
    }
    const std::vector<Keyframe> keyframes = manager.snapshot();
    DescriptorConfigs descriptors;
    if (scan_context_config.enabled)
        descriptors.scan_context = scan_context_config;
    if (gprobe_config.enabled)
        descriptors.gprobe = gprobe_config;
    writeStagedBundle(staged, keyframes, descriptors);

    if (!write_map_pcd) {
        return commit();
    }
    const fs::path map_path = fs::path(staged) / "map.pcd";
    if (!saveMapCloud(map_path.string(), keyframes, map_voxel)) {
        std::cerr << "Failed to write " << map_path << "\n";
        return false;
    }
    return commit();
}

/// 为给定位姿序列分配索引和时刻，并逐帧模拟扫描及汇总点数。
std::vector<GeneratedScan> generateScans(const pcl::PointCloud<pcl::PointXYZ>& reference,
                                         const std::vector<Eigen::Matrix4d>& poses,
                                         const BuildOptions& args, const double stamp_base) {
    std::vector<GeneratedScan> scans;
    scans.reserve(poses.size());
    std::size_t total_points = 0;
    for (std::size_t i = 0; i < poses.size(); ++i) {
        GeneratedScan scan;
        scan.index = static_cast<int>(i);
        scan.stamp = stamp_base + static_cast<double>(i) * 0.1;
        scan.pose = poses[i];
        scan.cloud = simulateScan(reference, scan.pose, args);
        total_points += scan.cloud ? scan.cloud->size() : 0;
        scans.push_back(std::move(scan));
    }
    const double mean_points =
        scans.empty() ? 0.0 : static_cast<double>(total_points) / static_cast<double>(scans.size());
    std::cout << "Generated " << scans.size() << " scans, mean_points=" << mean_points << "\n";
    return scans;
}

}  // namespace backend::tools::build
