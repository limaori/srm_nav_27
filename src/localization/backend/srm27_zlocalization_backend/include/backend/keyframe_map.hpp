/**
 * @file keyframe_map.hpp
 * @brief 维护关键帧序列、优化位姿和不可变空间索引快照。
 */
#pragma once

#include <cmath>
#include <cstddef>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "backend/config.hpp"
#include "backend/types.hpp"

namespace backend {
class CloudStorage;

class KeyframeMap {
public:
    /// 创建具有独立全局版本身份的空关键帧序列。
    KeyframeMap();
    /// 清空运行关键帧及空间索引，并从零开始新轨迹编号。
    void clear();
    /// 查询当前关键帧序列是否为空。
    bool empty() const;
    /// 返回当前保留的关键帧记录数量。
    std::size_t size() const;
    /// 返回当前关键帧元数据及优化位姿的版本身份。
    std::uint64_t revision() const;
    /// 捕获磁盘后备所有权后，在轨迹锁外读取其缓存统计。
    CloudCacheStats storageStats() const;

    /// 判断当前帧是否应创建新关键帧。
    bool shouldCreateKeyframe(const FrontendFrame& frame, const KeyframePolicy& policy) const;

    /// 添加一帧关键帧，返回新建的 Keyframe 副本。
    Keyframe add(const FrontendFrame& frame);

    /// 获取全部关键帧的快照（线程安全拷贝）。
    std::vector<Keyframe> snapshot() const;
    /// 返回同一轨迹锁内捕获的全部优化位置。
    std::vector<Eigen::Vector3d> positions() const;

    /// 按索引获取单个关键帧。
    bool get(int index, Keyframe* out) const;

    /// 获取最新关键帧。
    Keyframe latest() const;

    /// 返回下一次 add() 使用的单调递增索引；定位轨迹裁剪后仍保持连续语义。
    int nextIndex() const;

    /// 仅保留最新记录且不复用图索引。
    /// 建图保留完整历史；先验图定位可只留一个运行时锚点，有限活跃历史由固定滞后
    /// 平滑器管理。
    void retainLatest(std::size_t max_count);

    /// 只更新发生变化的图状态；固定滞后定位把已边缘化历史帧冻结在末次估计。
    void updateOptimizedPoses(const std::vector<std::pair<int, Eigen::Matrix4d>>& poses);

    /// 按优化位姿的欧氏距离查找空间邻居。max_count=0 表示不限制数量，
    /// max_radius<=0 表示不限制半径。结果按距离、再按 index 稳定排序。
    std::vector<int> nearestKeyframeIndices(const Eigen::Vector3d& position,
                                            std::size_t max_count = 0,
                                            double max_radius = 0.0) const;

    /// 以 center_index 的优化位姿为中心选择空间邻居，并把点云拼到该中心
    /// 关键帧坐标系。max_keyframes 至少按 1 处理，因此中心帧始终在合法结果中。
    Cloud::Ptr buildSpatialSubmap(int center_index, std::size_t max_keyframes,
                                  double max_radius = 0.0) const;

    /// 构建所有关键帧的全局优化地图。
    Cloud::Ptr buildOptimizedMap(double voxel_resolution = 0.0) const;
    /// 逐块投影几何并按指定体素质心规则生成确定排序的全局点云。
    static Cloud::Ptr buildOptimizedMapFromSnapshot(const std::vector<Keyframe>& keyframes,
                                                    double voxel_resolution = 0.0);

    /// 保存关键帧数据到磁盘目录：
    ///   directory/keyframes.txt       每帧位姿+时间戳
    ///   directory/clouds/NNNNNN.pcd   每帧 sensor 坐标系点云
    bool save(const std::string& directory) const;
    /// 写出调用方提供的一致轨迹快照；磁盘操作不占用轨迹状态锁。
    static bool saveSnapshot(const std::string& directory, const std::vector<Keyframe>& keyframes);

    /// 校验并安装关键帧元数据；几何保留磁盘后备并按缓存预算加载。
    bool load(const std::string& directory, std::size_t cache_bytes = 64U * 1024U * 1024U);

private:
    mutable std::mutex mutex_;
    std::vector<Keyframe> keyframes_;
    int first_index_ = 0;
    int next_index_ = 0;
    struct SpatialIndex;
    /// 在锁外构建位置索引，并仅在轨迹版本未变时缓存结果。
    std::shared_ptr<const SpatialIndex> spatialIndex() const;
    /// 在持锁状态推进轨迹版本并使旧空间索引失效。
    void changedLocked();
    std::uint64_t revision_ = 0;
    std::shared_ptr<CloudStorage> storage_;
    mutable std::shared_ptr<const SpatialIndex> spatial_index_;
};

}  // namespace backend
