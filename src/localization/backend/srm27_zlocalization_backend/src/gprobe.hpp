/**
 * @file gprobe.hpp
 * @brief 声明 G-PROBE 私有索引及分阶段检索流程。
 */
#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "backend/gprobe.hpp"

namespace backend::gprobe_detail {

inline constexpr std::uint32_t kFileMagic = 0x47505242;  // GPRB
inline constexpr std::uint32_t kFileVersion = 2;
inline constexpr double kEpsilon = 1.0e-9;
inline constexpr double kBernoulliEpsilon = 1.0e-6;
inline constexpr double kGaussianTruncate = 4.0;
inline constexpr int kMapBranchCount = 6;

using Mask = std::vector<std::uint8_t>;

struct Branch {
    std::vector<float> grid;
    std::vector<float> mu;
    std::vector<float> sigma;
};

struct FrameDescriptor {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    std::array<Branch, kMapBranchCount> map_branches;
    Branch query_branch;
};

struct PairGeometry {
    double center_deg = 0.0;
    Mask mask;
};

struct VoxelKey {
    std::int32_t x = 0;
    std::int32_t y = 0;
    std::int32_t z = 0;

    /// 比较三个体素整数坐标是否一致。
    bool operator==(const VoxelKey& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct VoxelKeyHash {
    /// 混合三个体素整数坐标生成哈希值。
    std::size_t operator()(const VoxelKey& key) const {
        std::size_t seed = std::hash<std::int32_t>{}(key.x);
        seed ^= std::hash<std::int32_t>{}(key.y) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
        seed ^= std::hash<std::int32_t>{}(key.z) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
        return seed;
    }
};

struct Nomination {
    int pair = -1;
    double similarity = 0.0;
    double fov_weight = 0.0;
};

struct ScoredCandidate {
    int index = -1;
    double score = 0.0;
};

/// 将角度约束到 [-180, 180) 度。
double wrapDegrees(double degrees);

/// 对离散环形索引执行非负取模。
int wrapIndex(int value, int size);

/// 计算循环平移后目标列对应的源列。
int rolledSourceIndex(int destination, int shift, int sectors);

/// 统计角度掩码中的有效列数。
int countMask(const Mask& mask);

/// 按若干视场中心和宽度生成环形角度掩码。
Mask angleMask(int sectors, const std::vector<double>& centers_deg, double width_deg);

/// 生成六个双视场地图分支的几何定义。
std::array<PairGeometry, kMapBranchCount> makeMapPairs(int sectors);

/// 从点云构造一个带不确定度的 G-PROBE 分支。
std::vector<PointType> downsampleFirstPoint(const Cloud& cloud, double voxel_size);
/// 从首点体素采样结果构造占据概率及其不确定度。
Branch makeSampledBranch(const std::vector<PointType>& points, const Mask& mask,
                         const GProbeDescriptorConfig& config);
/// 从原始点云构造单个视场的概率描述子分支。
Branch makeBranch(const Cloud& cloud, const Mask& mask, const GProbeDescriptorConfig& config);

/// 构造分支粗检索所用的归一化径向键。
std::vector<double> normalizedKey(const Branch& branch, const Mask& mask, int shift,
                                  const GProbeDescriptorConfig& config);

/// 计算两个等长向量的点积。
double dotProduct(const std::vector<double>& lhs, const std::vector<double>& rhs);

/// 计算指定航向平移下两个分支的概率一致性得分。
double branchScore(const Branch& query, const Branch& database, const Mask& query_mask,
                   const Mask& database_mask, int shift, const GProbeDescriptorConfig& config);

/// 计算指定航向平移下两个高度网格的相关量。
double correlationForShift(const Branch& query, const Branch& database, const Mask& query_overlap,
                           const Mask& database_overlap, int shift,
                           const GProbeDescriptorConfig& config);

/// 写出一组 G-PROBE 描述子及其几何配置。
bool saveBundle(const std::string& directory, const GProbeDescriptorConfig& config,
                const std::vector<FrameDescriptor>& frames);

/// 读取描述子，并拒绝与当前几何配置不兼容的数据。
bool loadBundle(const std::string& directory, const GProbeDescriptorConfig& expected_config,
                std::vector<FrameDescriptor>* frames);

/// 只读取持久化文件中的 G-PROBE 配置。
bool loadBundleConfig(const std::string& directory, GProbeDescriptorConfig* config);

}  // namespace backend::gprobe_detail

namespace backend {

struct GProbeDescriptor::Impl {
    std::uint64_t revision = GProbeDescriptor::nextRevision();
    using Branch = gprobe_detail::Branch;
    using FrameDescriptor = gprobe_detail::FrameDescriptor;
    using HeadingPair = gprobe_detail::PairGeometry;
    using Mask = gprobe_detail::Mask;
    using Nomination = gprobe_detail::Nomination;
    using ScoredCandidate = gprobe_detail::ScoredCandidate;
    using PairSet = std::array<HeadingPair, gprobe_detail::kMapBranchCount>;

    struct HeadingModel {
        std::array<int, gprobe_detail::kMapBranchCount> hypotheses{};
        std::array<int, gprobe_detail::kMapBranchCount> hypothesis_sizes{};
    };

    struct PairEvaluation {
        std::array<double, gprobe_detail::kMapBranchCount> scores{};
        std::array<bool, gprobe_detail::kMapBranchCount> nominated{};
    };

    GProbeDescriptorConfig config;
    std::vector<FrameDescriptor> frames;

    /// 生成查询传感器的角度掩码。
    Mask queryMask() const;

    /// 构造一帧地图分支和查询分支描述子。
    FrameDescriptor makeFrame(const Cloud& cloud, const Eigen::Vector3d& position) const;

    /// 规范化外部候选索引；空集合表示允许全部帧。
    std::vector<int> candidateIndices(const std::vector<int>& allowed_indices) const;

    /// 对已排序候选执行基于地图位置的非极大值抑制。
    std::vector<int> selectDiverse(const std::vector<std::pair<int, double>>& ranked,
                                   int count) const;

    /// 以每个地图视场分支的粗键提名空间分散的候选帧。
    std::vector<std::vector<Nomination>> buildNominations(const Branch& query,
                                                          const std::vector<int>& indices,
                                                          const Mask& query_mask,
                                                          const PairSet& pairs) const;

    /// 按视场支持度和粗键相似度选择需要精验证的候选。
    std::vector<int> prefilterCandidates(
        const std::vector<int>& indices,
        const std::vector<std::vector<Nomination>>& nominations) const;

    /// 将六个地图分支归并成两个正交航向轴假设。
    HeadingModel makeHeadingModel(const PairSet& pairs) const;

    /// 在提名分支的航向窗口内精搜，并计算每个分支的概率得分。
    PairEvaluation evaluatePairs(int index, const Branch& query, const Mask& query_mask,
                                 const PairSet& pairs,
                                 const std::vector<std::vector<Nomination>>& nominations) const;

    /// 计算两个正交航向轴中主导轴的 S-GRT 概率。
    double dominantAxisProbability(const PairEvaluation& evaluation,
                                   const HeadingModel& model) const;

    /// 将一个精验证候选归约为最终地点一致性得分。
    std::optional<ScoredCandidate> verifyCandidate(
        int index, const Branch& query, const Mask& query_mask, const PairSet& pairs,
        const std::vector<std::vector<Nomination>>& nominations,
        const HeadingModel& heading_model) const;

    /// 执行粗提名、有限精验证和确定性排序的完整检索流程。
    std::vector<ScoredCandidate> scoreQuery(const Branch& query,
                                            const std::vector<int>& allowed_indices) const;
};

}  // namespace backend
