/**
 * @file cloud_storage.hpp
 * @brief 校验不可变关键帧几何的磁盘后备，并提供按需加载的只读租约。
 */
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "backend/cache.hpp"

namespace backend {
/// 地图几何的磁盘后备与缓存。工厂逐块完整校验后才发布，保留全有或全无语义。
class CloudStorage : public std::enable_shared_from_this<CloudStorage> {
public:
    /// 逐块验证文件及几何内容，全部成功后返回可共享的磁盘后备。
    static std::shared_ptr<CloudStorage> openValidated(const std::vector<std::string>& paths,
                                                       std::size_t cache_bytes);
    /// 释放文件元数据及存储自身持有的缓存。
    ~CloudStorage();
    /// 为指定磁盘块构造保有存储所有权的轻量几何句柄。
    CloudBlock block(std::size_t index) const;
    /// 返回拥有所有权的只读点云；磁盘后备加载需再次校验内容身份。
    Cloud::ConstPtr load(std::size_t index) const;
    /// 返回预先验收的点数，无需加载磁盘几何。
    std::size_t pointCount(std::size_t index) const;
    /// 返回当前缓存占用及累计命中、加载和逐出统计。
    CloudCacheStats stats() const;

private:
    struct Impl;
    /// 创建拥有独立点云缓存的磁盘后备存储。
    explicit CloudStorage(std::size_t cache_bytes);
    std::unique_ptr<Impl> impl_;
};
}  // namespace backend
