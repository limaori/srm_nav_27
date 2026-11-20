/**
 * @file cloud.hpp
 * @brief 定义后端点类型、点云别名与不可变几何租约句柄。
 */
#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstddef>
#include <memory>
#include <utility>

namespace backend {
using PointType = pcl::PointXYZI;
using Cloud = pcl::PointCloud<PointType>;

/// 记录只读几何缓存的占用、加载和逐出统计。
struct CloudCacheStats {
    std::size_t bytes = 0;
    std::size_t peak_bytes = 0;
    std::size_t entries = 0;
    std::size_t hits = 0;
    std::size_t loads = 0;
    std::size_t waits = 0;
    std::size_t evictions = 0;
};

class CloudStorage;

/// 一个不可变关键帧几何块。复制句柄不加载点；load() 返回有所有权的只读租约。
class CloudBlock {
public:
    /// 构造不持有几何的空句柄。
    CloudBlock() = default;
    /// 保存常驻只读点云的共享所有权。
    explicit CloudBlock(Cloud::ConstPtr resident) : resident_(std::move(resident)) {}
    /// 返回拥有所有权的只读点云；磁盘后备加载需再次校验内容身份。
    Cloud::ConstPtr load() const;
    /// 返回几何点数，空句柄返回零；不加载磁盘点云。
    std::size_t pointCount() const;
    /// 返回句柄是否持有常驻点云或已验收的磁盘后备。
    explicit operator bool() const { return resident_ != nullptr || storage_ != nullptr; }

private:
    friend class CloudStorage;
    Cloud::ConstPtr resident_;
    std::shared_ptr<const CloudStorage> storage_;
    std::size_t index_ = 0;
};

}  // namespace backend
