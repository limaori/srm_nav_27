/**
 * @file cloud_storage.cpp
 * @brief 校验不可变关键帧几何的磁盘后备，并提供按需加载的只读租约。
 */

#include "backend/cloud_storage.hpp"

#include <pcl/io/pcd_io.h>

#include <array>
#include <cstring>
#include <filesystem>
#include <stdexcept>

#include "backend/sampling.hpp"

namespace backend {
namespace {
/// 读取非空关键帧 PCD 并移除非有限几何，无有效点时抛出异常。
Cloud::ConstPtr readCloud(const std::string& path) {
    Cloud::Ptr cloud(new Cloud);
    if (pcl::io::loadPCDFile<PointType>(path, *cloud) != 0 || cloud->empty())
        throw std::runtime_error("invalid or empty keyframe cloud: " + path);
    if (!cloud->is_dense || std::any_of(cloud->begin(), cloud->end(),
                                        [](const PointType& p) { return !pointIsFinite(p); })) {
        cloud->is_dense = false;
        cloud = finiteCloudCopy(*cloud);
        if (cloud->empty())
            throw std::runtime_error("keyframe has no finite geometry: " + path);
    }
    return cloud;
}

/// 计算点坐标与反射强度的内容摘要，用于检测后备几何变化。
std::uint64_t checksum(const Cloud& cloud) {
    std::uint64_t hash = 14695981039346656037ULL;
    for (const auto& p : cloud) {
        const std::array<float, 4> values{p.x, p.y, p.z, p.intensity};
        for (float v : values) {
            std::uint32_t bits;
            std::memcpy(&bits, &v, sizeof(bits));
            for (int byte = 0; byte < 4; ++byte) {
                hash ^= (bits >> (8 * byte)) & 0xffU;
                hash *= 1099511628211ULL;
            }
        }
    }
    return hash;
}
}  // namespace

struct CloudStorage::Impl {
    struct File {
        std::string path;
        std::uintmax_t bytes = 0;
        std::filesystem::file_time_type modified;
        std::size_t points = 0;
        std::uint64_t checksum = 0;
        /// 比较磁盘文件的长度和修改时间是否仍与验收记录一致。
        bool unchanged() const {
            return std::filesystem::file_size(path) == bytes &&
                   std::filesystem::last_write_time(path) == modified;
        }
    };
    /// 按指定字节预算初始化关键帧几何缓存。
    explicit Impl(std::size_t limit) : cache(limit) {}
    std::vector<File> files;
    CloudCache<std::size_t> cache;
};

/// 创建拥有独立点云缓存的磁盘后备存储。
CloudStorage::CloudStorage(std::size_t bytes) : impl_(new Impl(bytes)) {
}
/// 释放文件元数据和存储自身持有的几何缓存。
CloudStorage::~CloudStorage() = default;

/// 逐块验证文件及几何内容，全部成功后返回可共享的磁盘后备。
std::shared_ptr<CloudStorage> CloudStorage::openValidated(const std::vector<std::string>& paths,
                                                          std::size_t cache_bytes) {
    auto storage = std::shared_ptr<CloudStorage>(new CloudStorage(cache_bytes));
    storage->impl_->files.reserve(paths.size());
    for (const auto& path : paths) {
        Impl::File file;
        file.path = std::filesystem::canonical(path).string();
        file.bytes = std::filesystem::file_size(file.path);
        file.modified = std::filesystem::last_write_time(file.path);
        const auto cloud = readCloud(file.path);
        file.points = cloud->size();
        file.checksum = checksum(*cloud);
        if (!file.unchanged())
            throw std::runtime_error("map changed during validation: " + path);
        storage->impl_->files.push_back(std::move(file));
        // 单块临时数据随迭代释放；元数据发布前已完整校验全部点云。
    }
    return storage;
}

/// 为指定磁盘块构造保有存储所有权的轻量几何句柄。
CloudBlock CloudStorage::block(std::size_t index) const {
    if (index >= impl_->files.size())
        throw std::out_of_range("cloud block index is out of range");
    CloudBlock block;
    block.storage_ = shared_from_this();
    block.index_ = index;
    return block;
}

/// 合并同块并发加载，并在读取前后验证磁盘文件及几何内容仍保持不变。
Cloud::ConstPtr CloudStorage::load(std::size_t index) const {
    const auto& file = impl_->files.at(index);
    // 在缓存锁外读取文件，并核对预先验收的元数据和几何摘要。
    return impl_->cache.get(index, [&]() {
        if (!file.unchanged())
            throw std::runtime_error("immutable map backing changed: " + file.path);
        const auto cloud = readCloud(file.path);
        if (!file.unchanged() || cloud->size() != file.points || checksum(*cloud) != file.checksum)
            throw std::runtime_error("immutable map geometry changed: " + file.path);
        return cloud;
    });
}

/// 返回预先验收的点数，无需加载磁盘几何。
std::size_t CloudStorage::pointCount(std::size_t index) const {
    return impl_->files.at(index).points;
}
/// 返回当前缓存占用及累计命中、加载和逐出统计。
CloudCacheStats CloudStorage::stats() const {
    return impl_->cache.stats();
}
/// 返回常驻点云或从磁盘后备加载具有所有权的只读几何租约。
Cloud::ConstPtr CloudBlock::load() const {
    return resident_ ? resident_ : (storage_ ? storage_->load(index_) : Cloud::ConstPtr{});
}
/// 返回常驻或磁盘后备块的点数，空句柄返回零。
std::size_t CloudBlock::pointCount() const {
    return resident_ ? resident_->size() : (storage_ ? storage_->pointCount(index_) : 0);
}
}  // namespace backend
