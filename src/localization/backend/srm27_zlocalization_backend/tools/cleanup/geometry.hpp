/**
 * @file geometry.hpp
 * @brief 离线清理的分层整数格及有明确端点约定的射线遍历。
 * Derived from FreeDOM, Copyright (c) 2025 Chen Li.
 */
#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace backend::tools::cleanup {
using Index = Eigen::Vector3i;

struct IndexHash {
    /// 对有符号格坐标按无符号整数运算生成哈希值。
    std::size_t operator()(const Index& index) const {
        return static_cast<std::uint32_t>(index.x()) * 73856093U ^
               static_cast<std::uint32_t>(index.y()) * 19349663U ^
               static_cast<std::uint32_t>(index.z()) * 83492791U;
    }
};

/// 自由体素分成 4³ 占据子格，8³ 自由体素组成一页；分辨率由实例持有。
struct Grid {
    static constexpr int kPageBits = 3;
    static constexpr int kSubcellBits = 2;
    static constexpr int kPageSide = 1 << kPageBits;
    static constexpr int kSubcellSide = 1 << kSubcellBits;
    static constexpr std::size_t kCellsPerPage = kPageSide * kPageSide * kPageSide;
    static constexpr std::size_t kSubcellsPerCell = kSubcellSide * kSubcellSide * kSubcellSide;
    const double resolution;
    const double inverse;
    const double sub_inverse;
    /// 根据自由体素边长设置体素与占据子格的坐标尺度。
    explicit Grid(double size)
        : resolution(size), inverse(1 / size), sub_inverse(kSubcellSide / size) {}

    /// 将世界坐标向下取整为自由体素坐标。
    Index cell(const Eigen::Vector3d& point) const {
        return (point * inverse).array().floor().cast<int>();
    }
    /// 将世界坐标向下取整为占据子格坐标。
    Index subcell(const Eigen::Vector3d& point) const {
        return (point * sub_inverse).array().floor().cast<int>();
    }
    /// 负坐标同样向下取整，避免 C++ 整数除法朝零截断改变格子归属。
    static Index page(const Index& cell) {
        Index result;
        for (int axis = 0; axis < 3; ++axis) {
            const std::int64_t value = cell[axis];
            result[axis] = static_cast<int>(value >= 0 ? value / kPageSide
                                                       : -((-value + kPageSide - 1) / kPageSide));
        }
        return result;
    }
    /// 将体素坐标编码为所在页内的连续偏移。
    static std::size_t offset(const Index& cell) {
        constexpr unsigned mask = kPageSide - 1;
        return ((static_cast<unsigned>(cell.x()) & mask) << (2 * kPageBits)) |
               ((static_cast<unsigned>(cell.y()) & mask) << kPageBits) |
               (static_cast<unsigned>(cell.z()) & mask);
    }
    /// 将子格坐标编码为所属自由体素内的连续偏移。
    static std::size_t subOffset(const Index& subcell) {
        constexpr unsigned mask = kSubcellSide - 1;
        return ((static_cast<unsigned>(subcell.x()) & mask) << (2 * kSubcellBits)) |
               ((static_cast<unsigned>(subcell.y()) & mask) << kSubcellBits) |
               (static_cast<unsigned>(subcell.z()) & mask);
    }
    /// 根据页坐标和页内偏移恢复自由体素坐标。
    static Index coordinate(const Index& page, std::size_t offset) {
        constexpr unsigned mask = kPageSide - 1;
        return page * kPageSide + Index(static_cast<int>(offset >> (2 * kPageBits)),
                                        static_cast<int>((offset >> kPageBits) & mask),
                                        static_cast<int>(offset & mask));
    }
};

/// 按面、边、角的固定顺序返回 26 邻域偏移，中心格由调用方单独处理。
inline const std::array<Index, 26>& neighbors() {
    static const std::array<Index, 26> offsets{
        Index(-1, 0, 0),  Index(1, 0, 0),   Index(0, -1, 0),  Index(0, 1, 0),    Index(0, 0, -1),
        Index(0, 0, 1),   Index(-1, -1, 0), Index(-1, 1, 0),  Index(1, -1, 0),   Index(1, 1, 0),
        Index(0, -1, -1), Index(0, 1, -1),  Index(0, -1, 1),  Index(0, 1, 1),    Index(-1, 0, -1),
        Index(1, 0, -1),  Index(-1, 0, 1),  Index(1, 0, 1),   Index(-1, -1, -1), Index(-1, 1, -1),
        Index(-1, -1, 1), Index(-1, 1, 1),  Index(1, -1, -1), Index(1, 1, -1),   Index(1, -1, 1),
        Index(1, 1, 1)};
    return offsets;
}

/// DDA 包含起始格，排除末端格；格边界正向走向下一平面，负向可立即跨格。
class Ray {
public:
    /// 初始化三个坐标方向的跨面时刻和步长。
    Ray(const Eigen::Vector3d& origin, const Eigen::Vector3d& end, const Grid& grid)
        : index_(grid.cell(origin)) {
        const Eigen::Vector3d velocity = end - origin;
        for (int axis = 0; axis < 3; ++axis) {
            const double speed = velocity[axis];
            step_[axis] = speed > 0 ? 1 : speed < 0 ? -1 : 0;
            stride_[axis] = speed == 0 ? 1 : grid.resolution / std::abs(speed);
            const double boundary =
                (static_cast<double>(index_[axis]) + (speed > 0 ? 1 : 0)) * grid.resolution;
            next_[axis] = speed == 0 ? 1 : std::max(0.0, (boundary - origin[axis]) / speed);
        }
    }
    /// 返回当前访问的自由体素坐标。
    const Index& index() const { return index_; }
    /// 跨越最近的网格平面，返回后续是否仍有位于射线末端之前的格子。
    bool advance() {
        // 同时跨面时按 Z、Y、X 优先级访问边界。
        const int axis = next_[0] < next_[1] && next_[0] < next_[2] ? 0
                         : next_[1] < next_[2]                      ? 1
                                                                    : 2;
        index_[axis] += step_[axis];
        next_[axis] += stride_[axis];
        return next_[0] < 1 || next_[1] < 1 || next_[2] < 1;
    }

private:
    Index index_, step_;
    Eigen::Vector3d stride_, next_;
};
}  // namespace backend::tools::cleanup
