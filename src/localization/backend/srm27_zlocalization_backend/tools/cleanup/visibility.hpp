/**
 * @file visibility.hpp
 * @brief 定义实测回波、自由证据和历史占据状态。
 * Derived from FreeDOM, Copyright (c) 2025 Chen Li.
 */
#pragma once

#include <array>
#include <bitset>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <unordered_map>
#include <vector>

#include "cleanup.hpp"
#include "geometry.hpp"

namespace backend::tools::cleanup {
/// 分类等级按动态证据强度递增，静态观测在子格融合时优先保留。
enum class Level : std::uint8_t { Static, Aggressive, Moderate, Conservative };
enum class Label { Unknown, Static, Dynamic };

struct ScanCell {
    Index index;
    Eigen::Vector3d center;
    std::vector<std::size_t> points;
    Level level = Level::Static;
};

/// 单帧世界坐标回波，输入顺序固定，几何代表点与原始点索引分开保存。
struct Scan {
    std::vector<Eigen::Vector3d> points;
    std::vector<ScanCell> cells;
    std::unordered_map<Index, std::size_t, IndexHash> lookup;
    std::unordered_map<Index, std::bitset<Grid::kCellsPerPage>, IndexHash> occupied;
    /// 按固定点序构建体素代表点、原始点索引及占据页。
    Scan(std::vector<Eigen::Vector3d> world_points, const Grid& grid);
    /// 查询本帧已占据的自由体素，缺失时返回空指针。
    ScanCell* find(const Index& index);
};

/// 独占一次清理的自由证据与历史占据状态。
class VisibilityMap {
public:
    /// 根据配置创建独立的可见性地图。
    explicit VisibilityMap(const Config& config);
    /// 依次分类当前扫描、更新自由证据、清理历史并融合当前占据。
    void integrate(Scan& scan, const Eigen::Vector3d& origin, std::uint32_t sequence);
    /// 根据最终占据子格查询原始点的分类。
    Label classify(const Eigen::Vector3d& point) const;

private:
    struct Evidence {
        std::uint32_t free_count = 0, occupied_count = 0;
        bool free = false;
    };
    struct FreePage {
        std::array<Evidence, Grid::kCellsPerPage> cells{};
        std::size_t confirmed = 0;
    };
    static constexpr std::uint32_t kRecoveryObservations = 20;
    static constexpr std::uint32_t kNoFrame = std::numeric_limits<std::uint32_t>::max();
    struct Contribution {
        std::uint32_t frame = kNoFrame;
        Level level = Level::Static;
    };
    struct Surface {
        std::array<Contribution, Grid::kSubcellsPerCell> samples{};
        /// 判断该体素是否保存了指定帧的占据贡献。
        bool contains(std::uint32_t frame) const;
    };
    struct SurfacePage {
        std::array<std::unique_ptr<Surface>, Grid::kCellsPerPage> cells{};
    };
    using Levels = std::unordered_map<Index, Level, IndexHash>;
    using Traversed = std::unordered_map<Index, std::bitset<Grid::kCellsPerPage>, IndexHash>;

    /// 查询自由证据，未分配页时返回空指针。
    Evidence* evidence(const Index& cell);
    /// 只读查询自由证据，未分配页时返回空指针。
    const Evidence* evidence(const Index& cell) const;
    /// 查询历史占据体素，未分配时返回空指针。
    Surface* surface(const Index& cell);
    /// 只读查询历史占据体素，未分配时返回空指针。
    const Surface* surface(const Index& cell) const;
    /// 撤销体素的自由确认状态并同步页内计数。
    void revoke(const Index& cell);
    /// 按历史自由证据分类本帧并处理占据恢复。
    void classifyScan(Scan& scan);
    /// 累计每帧射线穿越并返回新确认的自由体素。
    std::vector<Index> estimateFreeSpace(const Scan& scan, const Eigen::Vector3d& origin);
    /// 将新增自由证据传播到同帧历史占据邻域。
    void refineHistory(const std::vector<Index>& newly_free);
    /// 按静态优先规则融合本帧的占据子格贡献。
    void storeScan(const Scan& scan, std::uint32_t sequence);

    const Config config_;
    const Grid grid_;
    std::unordered_map<Index, FreePage, IndexHash> free_;
    std::unordered_map<Index, SurfacePage, IndexHash> surfaces_;
};
}  // namespace backend::tools::cleanup
