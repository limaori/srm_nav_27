/**
 * @file geometry.cpp
 * @brief 构造 G-PROBE 概率网格并计算跨视场几何得分。
 */

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <unordered_set>

#include "backend/math.hpp"
#include "gprobe.hpp"

namespace backend::gprobe_detail {

/// 将有限角度通过周期取模映射到 [-180, 180) 度。
double wrapDegrees(const double degrees) {
    double value = std::fmod(degrees + 180.0, 360.0);
    if (value < 0.0) {
        value += 360.0;
    }
    return value - 180.0;
}

/// 将离散环形索引映射到非负余数区间。
int wrapIndex(const int value, const int size) {
    const int remainder = value % size;
    return remainder < 0 ? remainder + size : remainder;
}

/// 计算循环列移后对应的源列索引。
int rolledSourceIndex(const int destination, const int shift, const int sectors) {
    return wrapIndex(destination - shift, sectors);
}

/// 统计视场掩码中的有效列数。
int countMask(const Mask& mask) {
    return static_cast<int>(std::count(mask.begin(), mask.end(), 1U));
}

/// 根据视场中心和宽度构造离散角度掩码。
Mask angleMask(const int sectors, const std::vector<double>& centers_deg, const double width_deg) {
    Mask mask(static_cast<std::size_t>(sectors), 0U);
    for (int sector = 0; sector < sectors; ++sector) {
        const double angle =
            -180.0 + (static_cast<double>(sector) + 0.5) * 360.0 / static_cast<double>(sectors);
        for (const double center : centers_deg) {
            if (std::abs(wrapDegrees(angle - center)) <= 0.5 * width_deg + 1.0e-9) {
                mask[static_cast<std::size_t>(sector)] = 1U;
                break;
            }
        }
    }
    return mask;
}

/// 计算两个角度的圆周均值，保持环绕边界连续。
double circularMeanDegrees(const double first, const double second) {
    const double a = degreesToRadians(first);
    const double b = degreesToRadians(second);
    const double x = std::cos(a) + std::cos(b);
    const double y = std::sin(a) + std::sin(b);
    if (std::hypot(x, y) < 1.0e-9) {
        return first;
    }
    return radiansToDegrees(std::atan2(y, x));
}

/// 构造六个双视场地图分支的几何掩码。
std::array<PairGeometry, kMapBranchCount> makeMapPairs(const int sectors) {
    constexpr std::array<double, 4> centers{-135.0, -45.0, 45.0, 135.0};
    std::array<PairGeometry, kMapBranchCount> pairs;
    int out = 0;
    for (int first = 0; first < 4; ++first) {
        for (int second = first + 1; second < 4; ++second) {
            PairGeometry& pair = pairs[static_cast<std::size_t>(out++)];
            pair.center_deg = circularMeanDegrees(centers[static_cast<std::size_t>(first)],
                                                  centers[static_cast<std::size_t>(second)]);
            pair.mask = angleMask(sectors,
                                  {centers[static_cast<std::size_t>(first)],
                                   centers[static_cast<std::size_t>(second)]},
                                  90.0);
        }
    }
    return pairs;
}

/// 按标准差及有限截断半径构造归一化高斯核。
std::vector<double> gaussianKernel(const double sigma) {
    if (sigma <= 0.0) {
        return {1.0};
    }
    const int radius = std::max(1, static_cast<int>(std::floor(kGaussianTruncate * sigma + 0.5)));
    std::vector<double> kernel(static_cast<std::size_t>(2 * radius + 1));
    double sum = 0.0;
    for (int offset = -radius; offset <= radius; ++offset) {
        const double value =
            std::exp(-0.5 * static_cast<double>(offset * offset) / (sigma * sigma));
        kernel[static_cast<std::size_t>(offset + radius)] = value;
        sum += value;
    }
    for (double& value : kernel) {
        value /= sum;
    }
    return kernel;
}

/// 对角度维执行循环边界的一维高斯卷积。
void blurCircular(const std::vector<double>& input, const std::vector<double>& kernel,
                  std::vector<double>* output) {
    const int size = static_cast<int>(input.size());
    const int radius = static_cast<int>(kernel.size() / 2U);
    output->assign(input.size(), 0.0);
    for (int index = 0; index < size; ++index) {
        double value = 0.0;
        for (int offset = -radius; offset <= radius; ++offset) {
            value += kernel[static_cast<std::size_t>(offset + radius)] *
                     input[static_cast<std::size_t>(wrapIndex(index + offset, size))];
        }
        (*output)[static_cast<std::size_t>(index)] = value;
    }
}

/// 对径向维执行常数边界的一维高斯卷积。
void blurConstant(const std::vector<double>& input, const std::vector<double>& kernel,
                  std::vector<double>* output) {
    const int size = static_cast<int>(input.size());
    const int radius = static_cast<int>(kernel.size() / 2U);
    output->assign(input.size(), 0.0);
    for (int index = 0; index < size; ++index) {
        double value = 0.0;
        for (int offset = -radius; offset <= radius; ++offset) {
            const int source = index + offset;
            if (source < 0 || source >= size) {
                continue;
            }
            value += kernel[static_cast<std::size_t>(offset + radius)] *
                     input[static_cast<std::size_t>(source)];
        }
        (*output)[static_cast<std::size_t>(index)] = value;
    }
}

/// 按体素保留首个有限点，保持描述子专用的采样顺序。
std::vector<PointType> downsampleFirstPoint(const Cloud& cloud, const double voxel_size) {
    std::vector<PointType> points;
    points.reserve(cloud.size());
    if (voxel_size <= 0.0) {
        for (const auto& point : cloud.points) {
            if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
                points.push_back(point);
            }
        }
        return points;
    }

    std::unordered_set<VoxelKey, VoxelKeyHash> occupied;
    occupied.reserve(cloud.size());
    for (const auto& point : cloud.points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
            continue;
        }
        const VoxelKey key{static_cast<std::int32_t>(std::floor(point.x / voxel_size)),
                           static_cast<std::int32_t>(std::floor(point.y / voxel_size)),
                           static_cast<std::int32_t>(std::floor(point.z / voxel_size))};
        if (occupied.insert(key).second) {
            points.push_back(point);
        }
    }
    return points;
}

/// 计算点所属的角度扇区并约束索引范围。
int pointSector(const PointType& point, const int sectors) {
    const double angle = std::atan2(static_cast<double>(point.y), static_cast<double>(point.x));
    const double scaled = (angle + kPi) / (2.0 * kPi) * sectors;
    return std::clamp(static_cast<int>(std::floor(scaled)), 0, sectors - 1);
}

/// 从原始点云构造单个视场的概率描述子分支。
Branch makeBranch(const Cloud& cloud, const Mask& mask, const GProbeDescriptorConfig& config) {
    return makeSampledBranch(downsampleFirstPoint(cloud, config.voxel_size), mask, config);
}

/// 从首点体素采样结果构造占据概率及其不确定度。
Branch makeSampledBranch(const std::vector<PointType>& points, const Mask& mask,
                         const GProbeDescriptorConfig& config) {
    const int rings = config.num_rings;
    const int sectors = config.num_sectors;
    const std::size_t cell_count = static_cast<std::size_t>(rings * sectors);
    Branch branch;
    branch.grid.assign(cell_count, 0.0F);
    branch.mu.assign(cell_count, 0.0F);
    branch.sigma.assign(cell_count, 0.0F);
    std::vector<std::uint8_t> occupancy(cell_count, 0U);
    std::vector<float> maximum_height(cell_count, -std::numeric_limits<float>::infinity());

    for (const auto& point : points) {
        const int sector = pointSector(point, sectors);
        if (mask[static_cast<std::size_t>(sector)] == 0U) {
            continue;
        }
        const double radius =
            std::hypot(static_cast<double>(point.x), static_cast<double>(point.y));
        if (radius > config.max_range) {
            continue;
        }
        const int ring =
            std::clamp(static_cast<int>(radius / config.max_range * rings), 0, rings - 1);
        const std::size_t cell = static_cast<std::size_t>(ring * sectors + sector);
        occupancy[cell] = 1U;
        maximum_height[cell] = std::max(maximum_height[cell], point.z);
    }

    for (std::size_t cell = 0; cell < cell_count; ++cell) {
        if (occupancy[cell] != 0U) {
            branch.grid[cell] = maximum_height[cell];
        }
    }

    const double ring_width = config.max_range / rings;
    const double sector_width = 2.0 * kPi / sectors;
    std::vector<double> angular_input(static_cast<std::size_t>(sectors));
    std::vector<double> angular_output;
    std::vector<double> blurred(cell_count, 0.0);
    for (int ring = 0; ring < rings; ++ring) {
        double density = 0.0;
        for (int sector = 0; sector < sectors; ++sector) {
            const std::size_t cell = static_cast<std::size_t>(ring * sectors + sector);
            angular_input[static_cast<std::size_t>(sector)] = occupancy[cell] != 0U ? 1.0 : 0.0;
            density += angular_input[static_cast<std::size_t>(sector)];
        }
        density = std::max(density / sectors, 0.01);
        const double radius = (static_cast<double>(ring) + 0.5) * ring_width;
        const double sigma_effective = config.sigma_t * std::sqrt(density);
        const double sigma_theta = std::max((sigma_effective / radius) / sector_width, 0.1);
        blurCircular(angular_input, gaussianKernel(sigma_theta), &angular_output);
        for (int sector = 0; sector < sectors; ++sector) {
            blurred[static_cast<std::size_t>(ring * sectors + sector)] =
                angular_output[static_cast<std::size_t>(sector)];
        }
    }

    const std::vector<double> radial_kernel = gaussianKernel(config.sigma_t / ring_width);
    std::vector<double> radial_input(static_cast<std::size_t>(rings));
    std::vector<double> radial_output;
    for (int sector = 0; sector < sectors; ++sector) {
        for (int ring = 0; ring < rings; ++ring) {
            radial_input[static_cast<std::size_t>(ring)] =
                blurred[static_cast<std::size_t>(ring * sectors + sector)];
        }
        blurConstant(radial_input, radial_kernel, &radial_output);
        for (int ring = 0; ring < rings; ++ring) {
            const std::size_t cell = static_cast<std::size_t>(ring * sectors + sector);
            const double mu = std::clamp(radial_output[static_cast<std::size_t>(ring)], 0.0, 1.0);
            branch.mu[cell] = static_cast<float>(mu);
            branch.sigma[cell] = static_cast<float>(std::sqrt(mu * (1.0 - mu)));
        }
    }

    for (int sector = 0; sector < sectors; ++sector) {
        if (mask[static_cast<std::size_t>(sector)] != 0U) {
            continue;
        }
        for (int ring = 0; ring < rings; ++ring) {
            const std::size_t cell = static_cast<std::size_t>(ring * sectors + sector);
            branch.grid[cell] = 0.0F;
            branch.mu[cell] = 0.0F;
            branch.sigma[cell] = 0.0F;
        }
    }
    return branch;
}

/// 提取并归一化分支的径向粗检索键。
std::vector<double> normalizedKey(const Branch& branch, const Mask& mask, const int shift,
                                  const GProbeDescriptorConfig& config) {
    std::vector<double> key(static_cast<std::size_t>(2 * config.num_rings), 0.0);
    for (int ring = 0; ring < config.num_rings; ++ring) {
        double mu_sum = 0.0;
        double height_sum = 0.0;
        for (int sector = 0; sector < config.num_sectors; ++sector) {
            if (mask[static_cast<std::size_t>(sector)] == 0U) {
                continue;
            }
            const int source = rolledSourceIndex(sector, shift, config.num_sectors);
            const std::size_t cell = static_cast<std::size_t>(ring * config.num_sectors + source);
            mu_sum += branch.mu[cell];
            height_sum += branch.grid[cell];
        }
        key[static_cast<std::size_t>(ring)] = mu_sum / config.num_sectors;
        key[static_cast<std::size_t>(config.num_rings + ring)] = height_sum / config.num_sectors;
    }
    const double norm = std::sqrt(std::inner_product(key.begin(), key.end(), key.begin(), 0.0));
    if (norm > kEpsilon) {
        for (double& value : key) {
            value /= norm;
        }
    }
    return key;
}

/// 计算等长粗检索键的内积相似度。
double dotProduct(const std::vector<double>& lhs, const std::vector<double>& rhs) {
    return std::inner_product(lhs.begin(), lhs.end(), rhs.begin(), 0.0);
}

/// 将概率限制在可计算对数的 Bernoulli 开区间内。
double clampBernoulli(const double mu, const double sigma) {
    return std::clamp(mu * (1.0 - sigma) + 0.5 * sigma, kBernoulliEpsilon, 1.0 - kBernoulliEpsilon);
}

/// 计算两个 Bernoulli 分布的对称 KL 距离。
double symmetricBernoulliKl(const double first, const double second) {
    const double first_to_second =
        first * std::log(first / second) + (1.0 - first) * std::log((1.0 - first) / (1.0 - second));
    const double second_to_first = second * std::log(second / first) +
                                   (1.0 - second) * std::log((1.0 - second) / (1.0 - first));
    return 0.5 * (first_to_second + second_to_first);
}

/// 按共同视场和列修剪策略计算分支验证得分。
double branchScore(const Branch& query, const Branch& database, const Mask& query_mask,
                   const Mask& database_mask, const int shift,
                   const GProbeDescriptorConfig& config) {
    struct JointCell {
        double query_height = 0.0;
        double database_height = 0.0;
        double weight = 0.0;
    };
    struct ColumnDivergence {
        double mean = 0.0;
        int count = 0;
    };

    const int query_mask_count = countMask(query_mask);
    const int database_mask_count = countMask(database_mask);
    int overlap_count = 0;
    std::vector<JointCell> joint_height;
    std::vector<ColumnDivergence> columns;
    columns.reserve(static_cast<std::size_t>(config.num_sectors));

    for (int sector = 0; sector < config.num_sectors; ++sector) {
        const int database_sector = rolledSourceIndex(sector, shift, config.num_sectors);
        if (query_mask[static_cast<std::size_t>(sector)] == 0U ||
            database_mask[static_cast<std::size_t>(database_sector)] == 0U) {
            continue;
        }
        ++overlap_count;
        double divergence_sum = 0.0;
        int divergence_count = 0;
        for (int ring = 0; ring < config.num_rings; ++ring) {
            const std::size_t query_cell =
                static_cast<std::size_t>(ring * config.num_sectors + sector);
            const std::size_t database_cell =
                static_cast<std::size_t>(ring * config.num_sectors + database_sector);
            const double query_mu = query.mu[query_cell];
            const double database_mu = database.mu[database_cell];
            const double query_p = clampBernoulli(query_mu, query.sigma[query_cell]);
            const double database_p = clampBernoulli(database_mu, database.sigma[database_cell]);
            if (query.grid[query_cell] != 0.0F && database.grid[database_cell] != 0.0F) {
                joint_height.push_back(
                    {query.grid[query_cell], database.grid[database_cell], query_p * database_p});
            }
            if (query_mu + database_mu > 0.1) {
                divergence_sum += symmetricBernoulliKl(query_p, database_p);
                ++divergence_count;
            }
        }
        if (divergence_count > 0) {
            columns.push_back({divergence_sum / divergence_count, divergence_count});
        }
    }

    if (overlap_count == 0 || columns.empty()) {
        return 0.0;
    }
    const double fov_weight = static_cast<double>(overlap_count) /
                              std::max(1, std::min(query_mask_count, database_mask_count));

    double height_correlation = 0.0;
    if (joint_height.size() >= 5U) {
        double weight_sum = 0.0;
        double query_mean = 0.0;
        double database_mean = 0.0;
        for (const JointCell& cell : joint_height) {
            weight_sum += cell.weight;
            query_mean += cell.weight * cell.query_height;
            database_mean += cell.weight * cell.database_height;
        }
        if (weight_sum > 1.0e-12) {
            query_mean /= weight_sum;
            database_mean /= weight_sum;
            double numerator = 0.0;
            double query_norm = 0.0;
            double database_norm = 0.0;
            for (const JointCell& cell : joint_height) {
                const double query_centered = cell.query_height - query_mean;
                const double database_centered = cell.database_height - database_mean;
                numerator += cell.weight * query_centered * database_centered;
                query_norm += cell.weight * query_centered * query_centered;
                database_norm += cell.weight * database_centered * database_centered;
            }
            const double denominator = std::sqrt(query_norm * database_norm);
            height_correlation =
                denominator <= 1.0e-12 ? 1.0 : std::max(0.0, numerator / denominator);
        }
    }

    std::sort(columns.begin(), columns.end(),
              [](const ColumnDivergence& lhs, const ColumnDivergence& rhs) {
                  return lhs.mean < rhs.mean;
              });
    int drop = static_cast<int>(std::floor(columns.size() * config.column_trim_ratio + 0.5));
    drop = std::min(drop, static_cast<int>(columns.size()) - 1);
    const int keep = static_cast<int>(columns.size()) - drop;
    double divergence_sum = 0.0;
    int divergence_cells = 0;
    for (int index = 0; index < keep; ++index) {
        const ColumnDivergence& column = columns[static_cast<std::size_t>(index)];
        divergence_sum += column.mean * column.count;
        divergence_cells += column.count;
    }
    const double bkl = divergence_cells > 0 ? std::exp(-divergence_sum / divergence_cells) : 0.0;
    return fov_weight * height_correlation * bkl;
}

/// 计算指定环形列移下的分支相关性。
double correlationForShift(const Branch& query, const Branch& database, const Mask& query_overlap,
                           const Mask& database_overlap, const int shift,
                           const GProbeDescriptorConfig& config) {
    double correlation = 0.0;
    for (int ring = 0; ring < config.num_rings; ++ring) {
        for (int database_sector = 0; database_sector < config.num_sectors; ++database_sector) {
            if (database_overlap[static_cast<std::size_t>(database_sector)] == 0U) {
                continue;
            }
            const int query_sector = wrapIndex(database_sector + shift, config.num_sectors);
            if (query_overlap[static_cast<std::size_t>(query_sector)] == 0U) {
                continue;
            }
            const std::size_t database_cell =
                static_cast<std::size_t>(ring * config.num_sectors + database_sector);
            const std::size_t query_cell =
                static_cast<std::size_t>(ring * config.num_sectors + query_sector);
            correlation += database.grid[database_cell] * query.grid[query_cell];
        }
    }
    return correlation;
}

}  // namespace backend::gprobe_detail
