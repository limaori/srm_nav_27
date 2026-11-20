/**
 * @file map_bundle.cpp
 * @brief 读写地图包并提交一致快照的几何与描述子。
 */

#include "backend/map_bundle.hpp"

#include <pcl/io/pcd_io.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

#include "backend/cloud_storage.hpp"
#include "backend/descriptor.hpp"
#include "backend/directory.hpp"
#include "backend/keyframe_map.hpp"
#include "backend/logger.hpp"

namespace backend {
/// 重建并写出地图包的地点描述子。
bool saveMapDescriptor(const std::string& directory, const std::vector<Keyframe>& frames,
                       PlaceDescriptor& descriptor) {
    rebuildMapDescriptors(descriptor, frames);
    return descriptor.save(directory);
}
/// 构造优化位姿下的全局点云并保存为 PCD。
bool saveMapCloud(const std::string& path, const std::vector<Keyframe>& frames, double voxel) {
    const auto cloud = KeyframeMap::buildOptimizedMapFromSnapshot(frames, voxel);
    return pcl::io::savePCDFileBinary(path, *cloud) == 0;
}
/// 从轨迹取得一致快照，完成全部写入后替换目标地图。
bool saveMapBundle(const std::string& directory, const KeyframeMap& trajectory,
                   const PlaceDescriptor& descriptor, const BundleOptions& options) {
    try {
        const auto frames = trajectory.snapshot();
        if (frames.empty())
            return false;
        DirectoryTransaction output(directory, DirectoryTransaction::Policy::Replace);
        std::string error;
        if (!output.prepare(&error)) {
            BACKEND_ERROR_STREAM("map save failed: " << error);
            return false;
        }
        const auto staged = output.stagingPath().string();
        auto saved_descriptor = descriptor.createEmpty();
        if (!KeyframeMap::saveSnapshot(staged, frames) ||
            (saved_descriptor->enabled() &&
             !saveMapDescriptor(staged, frames, *saved_descriptor)) ||
            (options.global_cloud && !saveMapCloud((output.stagingPath() / "map.pcd").string(),
                                                   frames, options.voxel_resolution)))
            return false;
        if (!output.commit(&error)) {
            BACKEND_ERROR_STREAM("map save failed: " << error);
            return false;
        }
        return true;
    } catch (const std::exception& error) {
        BACKEND_ERROR_STREAM("map save failed: " << error.what());
        return false;
    }
}

namespace {
/// 构造地图包中以六位关键帧索引命名的 PCD 文件路径。
std::string cloudPath(const std::string& root, int index) {
    std::ostringstream path;
    path << root << "/clouds/" << std::setw(6) << std::setfill('0') << index << ".pcd";
    return path.str();
}
/// 读取位姿索引，检查格式与数值，并按关键帧编号排序。
std::vector<Keyframe> readKeyframeIndex(const std::string& directory) {
    std::ifstream in(std::filesystem::path(directory) / "keyframes.txt");
    if (!in)
        throw std::runtime_error("cannot open keyframes.txt");
    std::vector<Keyframe> frames;
    std::string line;
    while (std::getline(in, line)) {
        const auto first = line.find_first_not_of(" \t\r");
        if (first == std::string::npos)
            continue;
        if (line[first] == '#') {
            if (line.compare(first, 13, "# backend_map") == 0) {
                std::istringstream version(line.substr(first + 13));
                int value = 0;
                if (!(version >> value) || value != 1)
                    throw std::runtime_error("unsupported map version");
            }
            continue;
        }
        Keyframe frame;
        Eigen::Vector3d p;
        double qx, qy, qz, qw;
        std::istringstream fields(line);
        if (!(fields >> frame.index >> frame.timestamp >> p.x() >> p.y() >> p.z() >> qx >> qy >>
              qz >> qw))
            throw std::runtime_error("malformed keyframe row");
        std::string extra;
        if (fields >> extra)
            throw std::runtime_error("unexpected keyframe columns");
        Eigen::Quaterniond q(qw, qx, qy, qz);
        if (frame.index < 0 || !std::isfinite(frame.timestamp) || !p.allFinite() ||
            !q.coeffs().allFinite() || !std::isfinite(q.norm()) || q.squaredNorm() <= 1e-12)
            throw std::runtime_error("invalid keyframe pose");
        q.normalize();
        frame.optimized_pose.block<3, 3>(0, 0) = q.toRotationMatrix();
        frame.optimized_pose.block<3, 1>(0, 3) = p;
        frame.odom_pose = frame.optimized_pose;
        frames.push_back(std::move(frame));
    }
    if (in.bad() || frames.empty())
        throw std::runtime_error("unreadable or empty map index");
    std::sort(frames.begin(), frames.end(),
              [](const Keyframe& a, const Keyframe& b) { return a.index < b.index; });
    return frames;
}
}  // namespace

/// 捕获一致轨迹快照并写出关键帧索引和局部点云。
bool KeyframeMap::save(const std::string& directory) const {
    return saveSnapshot(directory, snapshot());
}

/// 校验连续关键帧索引并写出位姿表及压缩或普通二进制 PCD。
bool KeyframeMap::saveSnapshot(const std::string& directory, const std::vector<Keyframe>& frames) {
    try {
        if (frames.empty())
            throw std::runtime_error("empty map bundle");
        for (std::size_t i = 0; i < frames.size(); ++i) {
            const auto& frame = frames[i];
            if (frame.index != static_cast<int>(i) || !std::isfinite(frame.timestamp) ||
                !frame.optimized_pose.allFinite() || !frame.cloud || frame.cloud.pointCount() == 0)
                throw std::runtime_error("invalid keyframe " + std::to_string(frame.index));
        }
        std::filesystem::create_directories(std::filesystem::path(directory) / "clouds");
        std::ofstream out(std::filesystem::path(directory) / "keyframes.txt");
        if (!out)
            throw std::runtime_error("cannot open keyframes.txt");
        out << "# backend_map 1\n# index timestamp tx ty tz qx qy qz qw\n" << std::setprecision(17);
        for (const auto& frame : frames) {
            const Eigen::Quaterniond q(frame.optimized_pose.block<3, 3>(0, 0));
            const Eigen::Vector3d p = frame.optimized_pose.block<3, 1>(0, 3);
            out << frame.index << ' ' << frame.timestamp << ' ' << p.x() << ' ' << p.y() << ' '
                << p.z() << ' ' << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w() << '\n';
            const auto cloud = frame.cloud.load();
            if (!cloud || cloud->empty() ||
                pcl::io::savePCDFileBinaryCompressed(cloudPath(directory, frame.index), *cloud) !=
                    0)
                throw std::runtime_error("cannot write keyframe cloud");
            // LZF 对高熵几何可能膨胀；自动保留较小的标准 PCD 编码。
            const std::string path = cloudPath(directory, frame.index);
            const std::size_t binary_bytes =
                pcl::PCDWriter::generateHeader<PointType>(*cloud).size() +
                std::string("DATA binary\n").size() + cloud->size() * 4U * sizeof(float);
            if (std::filesystem::file_size(path) >= binary_bytes &&
                pcl::io::savePCDFileBinary(path, *cloud) != 0)
                throw std::runtime_error("cannot write binary keyframe cloud");
        }
        out.flush();
        if (!out)
            throw std::runtime_error("cannot flush keyframes.txt");
        out.close();
        if (!out)
            throw std::runtime_error("cannot close keyframes.txt");
        return true;
    } catch (const std::exception& error) {
        BACKEND_ERROR_STREAM("map save failed: " << error.what());
        return false;
    }
}

/// 先完整验证地图索引与磁盘几何，再在锁内替换当前轨迹及缓存后备。
bool KeyframeMap::load(const std::string& directory, std::size_t cache_bytes) {
    try {
        auto frames = readKeyframeIndex(directory);
        std::vector<std::string> paths;
        paths.reserve(frames.size());
        for (std::size_t i = 0; i < frames.size(); ++i) {
            if (frames[i].index != static_cast<int>(i))
                throw std::runtime_error("non-contiguous map index");
            paths.push_back(cloudPath(directory, frames[i].index));
        }
        auto storage = CloudStorage::openValidated(paths, cache_bytes);
        for (std::size_t i = 0; i < frames.size(); ++i)
            frames[i].cloud = storage->block(i);
        {
            std::lock_guard<std::mutex> lock(mutex_);
            keyframes_ = std::move(frames);
            storage_ = std::move(storage);
            first_index_ = 0;
            next_index_ = static_cast<int>(keyframes_.size());
            changedLocked();
        }
        BACKEND_INFO_STREAM("loaded validated map: keyframes="
                            << paths.size() << ", cloud_cache_bytes=" << cache_bytes);
        return true;
    } catch (const std::exception& error) {
        BACKEND_ERROR_STREAM("map load failed: " << error.what());
        return false;  // 验证失败不替换已有地图或其缓存/快照。
    }
}
}  // namespace backend
