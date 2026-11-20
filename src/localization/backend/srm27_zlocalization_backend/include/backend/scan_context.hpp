/**
 * @file scan_context.hpp
 * @brief 定义地点召回后提供 yaw 初值的 Scan Context 描述子。
 */
#pragma once

#include <Eigen/Dense>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "backend/descriptor.hpp"

namespace backend {
struct ScanContextDescriptorConfig {
    bool enabled = true;
    bool level_with_odom_roll_pitch = true;
    int num_rings = 30;
    int num_sectors = 90;
    double max_radius = 80.0;
    double downsample = 0.6;
    // 与 config/backend.yaml 的 scan_context.lidar_height、建包工具的 --sensor-z 默认
    // 保持同一个值（URDF 0.346388 → 0.3464）。三者不一致时，重建/新建出来的描述子
    // 会被 storage.cpp 的几何校验拒绝。
    double lidar_height = 0.3464;
    int min_bin_points = 1;
    int augment_count = 3;
    double lateral_shift = 2.0;
    double search_ratio = 1.0;
    int exclude_recent = 15;
    int candidates = 15;
    double distance_threshold = 0.6;
};

struct ScanContextYawEstimate {
    double yaw_rad = 0.0;
    double descriptor_distance = std::numeric_limits<double>::max();
    int yaw_shift = 0;
    int variant_index = -1;
};

class ScanContextDescriptor final : public PlaceDescriptor {
public:
    struct Descriptor {
        Eigen::MatrixXf image;
        Eigen::VectorXf sector_key;
    };

    /// 规范化描述子几何与检索配置，并推进内容修订号。
    void configure(const ScanContextDescriptorConfig& config);
    /// 返回当前内容修订号，用于检索缓存身份判断。
    std::uint64_t revision() const override { return revision_; }
    /// 返回描述子算法的稳定名称。
    const char* name() const override { return "Scan Context"; }
    /// 创建保留当前配置且没有关键帧的描述子实例。
    std::unique_ptr<PlaceDescriptor> createEmpty() const override;
    /// 清空关键帧描述子并推进内容修订号。
    void clear() override;

    /// 构造并保存关键帧描述子，更新内容修订号。
    void addKeyframe(const Cloud& cloud_local) override;
    /// 校验关键帧索引和位置；Scan Context 编码不依赖地图位置。
    void updateKeyframePosition(int index, const Eigen::Vector3d& position) override;

    /// 召回最新关键帧的历史回环候选并排除近期帧。
    std::vector<LoopCandidate> detectLatestCandidates() const override;
    /// 在符合资格的历史索引中召回最新关键帧的候选。
    std::vector<LoopCandidate> detectLatestCandidatesWithin(
        const std::vector<int>& eligible_indices) const override;

    /// 返回当前保存的数据条目数。
    std::size_t size() const override;
    /// 判断当前索引是否没有条目。
    bool empty() const override;
    /// 返回描述子检索是否启用。
    bool enabled() const override;
    /// 返回当前生效的算法配置。
    const ScanContextDescriptorConfig& config() const;
    /// 返回描述子是否使用前端横滚和俯仰整平点云。
    bool levelWithOdomRollPitch() const override;

    /// 查询两帧或外部点云之间的有符号偏航初值。
    std::optional<ScanContextYawEstimate> estimateYaw(const Cloud& query_cloud,
                                                      int keyframe_index) const;
    /// 在指定存储索引中检索外部点云并按距离与编号排序。
    std::vector<LoopCandidate> queryAgainstStoredCandidates(const Cloud& query_cloud,
                                                            const std::vector<int>& allowed_indices,
                                                            int max_results) const;
    /// 召回外部查询点云的恢复候选，保留描述子自身排序。
    std::vector<LoopCandidate> queryLocalizationCandidates(const Cloud& query_cloud,
                                                           const std::vector<int>& allowed_indices,
                                                           int max_results) const override;

    /// 保存当前描述子配置及数据，失败时返回 false。
    bool save(const std::string& directory) const override;
    /// 加载与当前几何配置兼容的描述子数据，失败时返回 false。
    bool load(const std::string& directory) override;
    /// 仅读取持久化描述子配置，不加载关键帧数据。
    static bool loadConfig(const std::string& directory, ScanContextDescriptorConfig* config);

private:
    std::uint64_t revision_ = nextRevision();
    using DescriptorVariants = std::vector<Descriptor>;

    /// 将点云按极坐标网格编码为高度描述子。
    Descriptor makeDescriptor(const Cloud& cloud_local, const Eigen::Vector2d& root_shift) const;
    /// 将首点采样后的点集编码为指定平移下的高度描述子。
    Descriptor makeSampledDescriptor(const Cloud& sampled, const Eigen::Vector2d& root_shift) const;
    /// 构造原始及平移增广的描述子集合。
    DescriptorVariants makeDescriptorVariants(const Cloud& cloud_local) const;
    /// 从最佳循环列对齐估计源到目标的有符号偏航角。
    std::optional<ScanContextYawEstimate> estimateYawFromDescriptor(const Descriptor& query,
                                                                    int keyframe_index) const;
    /// 构造满足时间资格及指定索引范围的检索集合。
    std::vector<int> candidateIndexSet(const std::vector<int>& allowed_indices) const;
    /// 按有效列计算两个高度描述子的距离。
    std::pair<double, int> descriptorDistance(const Descriptor& query,
                                              const Descriptor& candidate) const;

    ScanContextDescriptorConfig config_;
    std::vector<DescriptorVariants> descriptors_;
};

}  // namespace backend
