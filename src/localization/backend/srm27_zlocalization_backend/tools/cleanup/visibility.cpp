/**
 * @file visibility.cpp
 * @brief 依次完成当前观测分类、自由证据更新、历史清理和占据积分。
 * Derived from FreeDOM, Copyright (c) 2025 Chen Li.
 */
#include "visibility.hpp"

#include <utility>

#include "parallel.hpp"

namespace backend::tools::cleanup {
namespace {
/// 每一层只从该层开始前的种子扩展；接收器负责拒绝已访问或不相容的格子。
template <class Accept>
std::vector<Index> expand(const std::vector<Index>& seeds, Accept accept) {
    std::vector<Index> accepted;
    for (const auto& seed : seeds)
        for (const auto& offset : neighbors()) {
            const Index index = seed + offset;
            if (accept(index))
                accepted.push_back(index);
        }
    return accepted;
}
}  // namespace

/// 按输入点序构建体素中心、点索引及页内占据位图。
Scan::Scan(std::vector<Eigen::Vector3d> world_points, const Grid& grid)
    : points(std::move(world_points)) {
    cells.reserve(points.size());
    lookup.reserve(points.size());
    for (std::size_t i = 0; i < points.size(); ++i) {
        const Index index = grid.cell(points[i]);
        const auto [found, inserted] = lookup.emplace(index, cells.size());
        if (inserted) {
            cells.push_back({index, points[i], {i}, Level::Static});
            occupied[Grid::page(index)].set(Grid::offset(index));
        } else {
            auto& cell = cells[found->second];
            const double count = static_cast<double>(cell.points.size());
            // 按原始点顺序递推体素中心，使并行任务划分不影响归约结果。
            cell.center = (count / (count + 1)) * cell.center + (1 / (count + 1)) * points[i];
            cell.points.push_back(i);
        }
    }
}

/// 通过体素坐标查询本帧对应的占据单元。
ScanCell* Scan::find(const Index& index) {
    const auto found = lookup.find(index);
    return found == lookup.end() ? nullptr : &cells[found->second];
}

/// 保存清理配置并初始化实例网格尺度。
VisibilityMap::VisibilityMap(const Config& config)
    : config_(config), grid_(config.voxel_resolution) {
}

/// 查询已分配页中的自由证据。
VisibilityMap::Evidence* VisibilityMap::evidence(const Index& cell) {
    const auto page = free_.find(Grid::page(cell));
    return page == free_.end() ? nullptr : &page->second.cells[Grid::offset(cell)];
}
/// 只读查询已分配页中的自由证据。
const VisibilityMap::Evidence* VisibilityMap::evidence(const Index& cell) const {
    const auto page = free_.find(Grid::page(cell));
    return page == free_.end() ? nullptr : &page->second.cells[Grid::offset(cell)];
}
/// 查询已分配的历史占据体素。
VisibilityMap::Surface* VisibilityMap::surface(const Index& cell) {
    const auto page = surfaces_.find(Grid::page(cell));
    return page == surfaces_.end() ? nullptr : page->second.cells[Grid::offset(cell)].get();
}
/// 只读查询已分配的历史占据体素。
const VisibilityMap::Surface* VisibilityMap::surface(const Index& cell) const {
    const auto page = surfaces_.find(Grid::page(cell));
    return page == surfaces_.end() ? nullptr : page->second.cells[Grid::offset(cell)].get();
}
/// 检查子格是否保存了给定帧的占据贡献。
bool VisibilityMap::Surface::contains(std::uint32_t frame) const {
    return std::any_of(samples.begin(), samples.end(),
                       [frame](const Contribution& sample) { return sample.frame == frame; });
}

/// 撤销已确认自由格，并同步所属页的自由格数量。
void VisibilityMap::revoke(const Index& cell) {
    auto page = free_.find(Grid::page(cell));
    if (page == free_.end())
        return;
    auto& evidence = page->second.cells[Grid::offset(cell)];
    if (evidence.free) {
        evidence.free = false;
        --page->second.confirmed;
    }
}

/// 命中先查询已有自由状态，再处理占据恢复和有限邻域扩展。
void VisibilityMap::classifyScan(Scan& scan) {
    std::vector<Index> seeds, recover;
    for (auto& cell : scan.cells) {
        auto* entry = evidence(cell.index);
        if (!entry)
            continue;
        ++entry->occupied_count;
        entry->free_count = 0;
        if (entry->free) {
            cell.level = Level::Conservative;
            seeds.push_back(cell.index);
        }
        if (entry->occupied_count >= kRecoveryObservations) {
            revoke(cell.index);
            recover.push_back(cell.index);
        }
    }
    /// 从当前层种子扩展一次邻域并设置动态等级。
    const auto grow = [&](const std::vector<Index>& input, Level level) {
        return expand(input, [&](const Index& index) {
            auto* cell = scan.find(index);
            if (!cell || cell->level >= level)
                return false;
            cell->level = level;
            return true;
        });
    };
    const auto moderate = grow(seeds, Level::Moderate);
    const auto aggressive = grow(moderate, Level::Aggressive);
    grow(aggressive, Level::Aggressive);
    for (const auto& cell : recover)
        for (const auto& offset : neighbors())
            revoke(cell + offset);
}

/// 每帧每格只计一次穿越；先更新所有计数，再统一确认 27 邻域。
std::vector<Index> VisibilityMap::estimateFreeSpace(const Scan& scan,
                                                    const Eigen::Vector3d& origin) {
    const std::size_t workers =
        std::min(scan.cells.size(), static_cast<std::size_t>(config_.threads));
    std::vector<Traversed> partial(workers);
    detail::parallelFor(workers, workers, [&](std::size_t worker) {
        auto& visited = partial[worker];
        for (std::size_t i = worker * scan.cells.size() / workers;
             i < (worker + 1) * scan.cells.size() / workers; ++i) {
            Eigen::Vector3d endpoint = scan.cells[i].center;
            const Eigen::Vector3d delta = endpoint - origin;
            if (delta.squaredNorm() > config_.raycast_range * config_.raycast_range)
                endpoint = origin + delta * (config_.raycast_range / delta.norm());
            Ray ray(origin, endpoint, grid_);
            Index previous_page = Index::Zero();
            bool cached = false;
            const std::bitset<Grid::kCellsPerPage>* occupied = nullptr;
            std::bitset<Grid::kCellsPerPage>* crossed = nullptr;
            do {
                const auto& cell = ray.index();
                const Index page = Grid::page(cell);
                if (!cached || page != previous_page) {
                    previous_page = page;
                    cached = true;
                    const auto hit = scan.occupied.find(page);
                    occupied = hit == scan.occupied.end() ? nullptr : &hit->second;
                    const auto traversed = visited.find(page);
                    crossed = traversed == visited.end() ? nullptr : &traversed->second;
                }
                const auto offset = Grid::offset(cell);
                if (occupied && (*occupied)[offset])
                    break;
                // 不能为只命中、没有穿越的页建立自由历史。
                if (!crossed)
                    crossed = &visited[page];
                crossed->set(offset);
            } while (ray.advance());
        }
    });
    Traversed traversed;
    for (const auto& part : partial)
        for (const auto& [index, bits] : part)
            traversed[index] |= bits;
    struct Update {
        Index index;
        const std::bitset<Grid::kCellsPerPage>* bits;
        FreePage* page;
        std::vector<std::size_t> candidates;
        std::vector<Index> confirmed;
    };
    std::vector<Update> updates;
    updates.reserve(traversed.size());
    for (const auto& [index, bits] : traversed) {
        auto& page = free_[index];
        if (page.confirmed != page.cells.size())
            updates.push_back({index, &bits, &page, {}, {}});
    }
    detail::parallelFor(updates.size(), config_.threads, [&](std::size_t i) {
        auto& update = updates[i];
        for (std::size_t offset = 0; offset < Grid::kCellsPerPage; ++offset) {
            if (!(*update.bits)[offset])
                continue;
            auto& cell = update.page->cells[offset];
            ++cell.free_count;
            cell.occupied_count = 0;
            if (!cell.free && cell.free_count >= static_cast<unsigned>(config_.free_observations))
                update.candidates.push_back(offset);
        }
    });
    detail::parallelFor(updates.size(), config_.threads, [&](std::size_t i) {
        auto& update = updates[i];
        for (const auto offset : update.candidates) {
            const Index index = Grid::coordinate(update.index, offset);
            const bool observed =
                std::all_of(neighbors().begin(), neighbors().end(), [&](const Index& delta) {
                    const auto* neighbor = evidence(index + delta);
                    return neighbor &&
                           neighbor->free_count >= static_cast<unsigned>(config_.free_observations);
                });
            if (observed) {
                update.page->cells[offset].free = true;
                ++update.page->confirmed;
                update.confirmed.push_back(index);
            }
        }
    });
    std::vector<Index> result;
    for (const auto& update : updates)
        result.insert(result.end(), update.confirmed.begin(), update.confirmed.end());
    return result;
}

/// 只扩展到同一历史观测拥有的占据格，避免混淆跨时刻的邻近表面。
void VisibilityMap::refineHistory(const std::vector<Index>& newly_free) {
    std::unordered_map<std::uint32_t, Levels> observations;
    for (const auto& index : newly_free) {
        const auto* cell = surface(index);
        if (!cell)
            continue;
        for (const auto& sample : cell->samples)
            if (sample.frame != kNoFrame)
                observations[sample.frame][index] = Level::Conservative;
    }
    for (auto& [frame, levels] : observations) {
        std::vector<Index> seeds;
        seeds.reserve(levels.size());
        for (const auto& [index, level] : levels) {
            (void)level;
            seeds.push_back(index);
        }
        /// 仅将同一历史帧的未访问占据格加入下一层。
        const auto grow = [&](const std::vector<Index>& input, Level level) {
            return expand(input, [&](const Index& index) {
                if (levels.count(index))
                    return false;
                const auto* cell = surface(index);
                if (!cell || !cell->contains(frame))
                    return false;
                levels.emplace(index, level);
                return true;
            });
        };
        const auto moderate = grow(seeds, Level::Moderate);
        // 历史扩展依次经过中等与激进两层邻域，层内仅使用扩展前的种子。
        grow(moderate, Level::Aggressive);
        for (const auto& [index, level] : levels)
            for (auto& sample : surface(index)->samples)
                if (sample.frame == frame && sample.level < level)
                    sample.level = level;
    }
}

/// 同一子格保留静态程度更高的观测；同一帧内的重复点不重复融合。
void VisibilityMap::storeScan(const Scan& scan, std::uint32_t sequence) {
    for (const auto& cell : scan.cells) {
        auto& surface = surfaces_[Grid::page(cell.index)].cells[Grid::offset(cell.index)];
        if (!surface)
            surface = std::make_unique<Surface>();
        for (const auto point : cell.points) {
            auto& sample = surface->samples[Grid::subOffset(grid_.subcell(scan.points[point]))];
            if (sample.frame == kNoFrame || (sample.frame != sequence && sample.level > cell.level))
                sample = {sequence, cell.level};
        }
    }
}

/// 按当前帧分类、自由证据、历史修正和占据融合的顺序更新地图。
void VisibilityMap::integrate(Scan& scan, const Eigen::Vector3d& origin, std::uint32_t sequence) {
    classifyScan(scan);
    refineHistory(estimateFreeSpace(scan, origin));
    storeScan(scan, sequence);
}

/// 回查最终占据子格；未知保留，不以缺测作为删除依据。
Label VisibilityMap::classify(const Eigen::Vector3d& point) const {
    const auto* cell = surface(grid_.cell(point));
    if (!cell)
        return Label::Unknown;
    const auto& sample = cell->samples[Grid::subOffset(grid_.subcell(point))];
    if (sample.frame == kNoFrame)
        return Label::Unknown;
    return sample.level == Level::Static ? Label::Static : Label::Dynamic;
}
}  // namespace backend::tools::cleanup
