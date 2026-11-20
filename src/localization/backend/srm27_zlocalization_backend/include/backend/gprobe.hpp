/**
 * @file gprobe.hpp
 * @brief 定义无需学习的 G-PROBE 跨视场地点描述子。
 */
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "backend/descriptor.hpp"

namespace backend {

struct GProbeDescriptorConfig {
    bool enabled = true;
    bool level_with_odom_roll_pitch = true;

    // G-PROBE 基础几何与不确定度模型。
    int num_rings = 40;
    int num_sectors = 60;
    double max_range = 80.0;
    double voxel_size = 0.5;
    double sigma_t = 2.0;

    // 传感器视场几何；Odin1 累积 SLAM 点云约覆盖 105°。
    double query_fov_deg = 105.0;
    double query_fov_center_deg = 7.5;

    // G-PROBE 跨视场检索与验证策略。
    int branch_top_k = 15;
    int verification_top_k = 60;
    double spatial_nms_radius = 2.0;
    double heading_window_deg = 50.0;
    double column_trim_ratio = 0.1;
    double sgrt_kappa = 10.0;

    int exclude_recent = 15;
    int candidates = 30;
};

class GProbeDescriptor final : public PlaceDescriptor {
public:
    /// 分配 G-PROBE 描述子索引的独占实现。
    GProbeDescriptor();
    /// 释放 G-PROBE 描述子索引及其存储。
    ~GProbeDescriptor() override;
    /// 转移 G-PROBE 描述子索引的独占所有权。
    GProbeDescriptor(GProbeDescriptor&&) noexcept;
    /// 转移描述子索引的独占所有权。
    GProbeDescriptor& operator=(GProbeDescriptor&&) noexcept;

    /// 禁止复制持有独占索引的描述子对象。
    GProbeDescriptor(const GProbeDescriptor&) = delete;
    /// 禁止以复制赋值共享描述子索引。
    GProbeDescriptor& operator=(const GProbeDescriptor&) = delete;

    /// 规范化描述子几何与检索配置，并推进内容修订号。
    void configure(const GProbeDescriptorConfig& config);
    /// 返回当前生效的算法配置。
    const GProbeDescriptorConfig& config() const;

    /// 返回当前内容修订号，用于检索缓存身份判断。
    std::uint64_t revision() const override;
    /// 返回描述子算法的稳定名称。
    const char* name() const override { return "G-PROBE"; }
    /// 创建保留当前配置且没有关键帧的描述子实例。
    std::unique_ptr<PlaceDescriptor> createEmpty() const override;
    /// 清空关键帧描述子并推进内容修订号。
    void clear() override;
    /// 构造并保存关键帧描述子，更新内容修订号。
    void addKeyframe(const Cloud& cloud_local) override;
    /// 构造并保存关键帧描述子，更新内容修订号。
    void addKeyframe(const Cloud& cloud_local, const Eigen::Vector3d& position) override;
    /// 更新检索所用的地图位置并推进内容修订号。
    void updateKeyframePosition(int index, const Eigen::Vector3d& position) override;

    /// 召回最新关键帧的历史回环候选并排除近期帧。
    std::vector<LoopCandidate> detectLatestCandidates() const override;
    /// 在符合资格的历史索引中召回最新关键帧的候选。
    std::vector<LoopCandidate> detectLatestCandidatesWithin(
        const std::vector<int>& eligible_indices) const override;
    /// 召回外部查询点云的恢复候选，保留描述子自身排序。
    std::vector<LoopCandidate> queryLocalizationCandidates(const Cloud& query_cloud,
                                                           const std::vector<int>& allowed_indices,
                                                           int max_results) const override;

    /// 返回当前保存的数据条目数。
    std::size_t size() const override;
    /// 判断当前索引是否没有条目。
    bool empty() const override;
    /// 返回描述子检索是否启用。
    bool enabled() const override;
    /// 返回描述子是否使用前端横滚和俯仰整平点云。
    bool levelWithOdomRollPitch() const override;

    /// 保存当前描述子配置及数据，失败时返回 false。
    bool save(const std::string& directory) const override;
    /// 加载与当前几何配置兼容的描述子数据，失败时返回 false。
    bool load(const std::string& directory) override;
    /// 仅读取持久化描述子配置，不加载关键帧数据。
    static bool loadConfig(const std::string& directory, GProbeDescriptorConfig* config);
    /// 返回版本化描述子数据的固定文件名。
    static const char* fileName() { return "gprobe_descriptors.bin"; }

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace backend
