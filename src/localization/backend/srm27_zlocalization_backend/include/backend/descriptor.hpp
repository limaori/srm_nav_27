/**
 * @file descriptor.hpp
 * @brief 定义全局地点描述子的公共检索接口。
 */
#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "backend/types.hpp"

namespace backend {

class PlaceDescriptor {
public:
    /// 通过描述子策略接口释放实现资源。
    virtual ~PlaceDescriptor() = default;

    /// 返回描述子算法的稳定名称。
    virtual const char* name() const = 0;
    /// 0 表示未提供版本，此时调用方不能缓存空检索结果。
    virtual std::uint64_t revision() const { return 0; }
    /// 创建保留当前配置且没有关键帧的描述子实例。
    virtual std::unique_ptr<PlaceDescriptor> createEmpty() const = 0;
    /// 清空关键帧描述子并推进内容修订号。
    virtual void clear() = 0;
    /// 构造并保存关键帧描述子，更新内容修订号。
    virtual void addKeyframe(const Cloud& cloud_local) = 0;
    /// 构造并保存关键帧描述子，更新内容修订号。
    virtual void addKeyframe(const Cloud& cloud_local, const Eigen::Vector3d& position) {
        (void)position;
        addKeyframe(cloud_local);
    }
    /// 更新检索所用的地图位置并推进内容修订号。
    virtual void updateKeyframePosition(int index, const Eigen::Vector3d& position) = 0;
    /// 召回最新关键帧的历史回环候选并排除近期帧。
    virtual std::vector<LoopCandidate> detectLatestCandidates() const = 0;
    /// 在符合资格的历史索引中召回最新关键帧的候选。
    virtual std::vector<LoopCandidate> detectLatestCandidatesWithin(
        const std::vector<int>& eligible_indices) const = 0;
    /// 召回外部查询点云的恢复候选，保留描述子自身排序。
    virtual std::vector<LoopCandidate> queryLocalizationCandidates(
        const Cloud& query_cloud, const std::vector<int>& allowed_indices,
        int max_results) const = 0;

    /// 返回当前保存的数据条目数。
    virtual std::size_t size() const = 0;
    /// 判断当前索引是否没有条目。
    virtual bool empty() const = 0;
    /// 返回描述子检索是否启用。
    virtual bool enabled() const = 0;
    /// 返回描述子是否使用前端横滚和俯仰整平点云。
    virtual bool levelWithOdomRollPitch() const = 0;
    /// 保存当前描述子配置及数据，失败时返回 false。
    virtual bool save(const std::string& directory) const = 0;
    /// 加载与当前几何配置兼容的描述子数据，失败时返回 false。
    virtual bool load(const std::string& directory) = 0;

protected:
    /// 分配进程内单调递增的描述子内容修订号。
    static std::uint64_t nextRevision() {
        static std::atomic<std::uint64_t> revision{1};
        return revision.fetch_add(1, std::memory_order_relaxed);
    }
};

/// 在线描述子使用不会随图优化改变的前端整平参考。
void addKeyframeDescriptor(PlaceDescriptor& descriptor, const Keyframe& keyframe);
/// 导出/重建地图描述子使用持久化的 optimized_pose，与加载后的整平参考一致。
void rebuildMapDescriptors(PlaceDescriptor& descriptor, const std::vector<Keyframe>& keyframes);

}  // namespace backend
