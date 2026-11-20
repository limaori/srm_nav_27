/**
 * @file registration.cpp
 * @brief 配置点到平面校正器并维护细、粗两级目标平面缓存。
 */

#include "backend/registration.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "backend/logger.hpp"
#include "backend/sampling.hpp"
#include "registration.hpp"

namespace backend {
/// 创建具有字节和条目预算的准备结果缓存。
Registration::TargetCache::TargetCache(std::size_t bytes, std::size_t entries)
    : impl_(std::make_unique<Impl>(bytes, entries)) {
}

/// 释放缓存拥有的准备结果。
Registration::TargetCache::~TargetCache() = default;

/// 清空缓存并使尚未完成的旧代加载失效。
void Registration::TargetCache::clear() {
    impl_->cache.clear();
}

/// 返回当前准备结果缓存统计。
CloudCacheStats Registration::TargetCache::stats() const {
    return impl_->cache.stats();
}

namespace {

/// 返回有限输入，非有限时使用指定默认值。
double finiteOr(const double value, const double fallback) {
    return std::isfinite(value) ? value : fallback;
}

}  // namespace

/// 构造点面校正器并关联可选的只读目标准备缓存。
Registration::Registration(const RegistrationConfig& config, TargetCache* target_cache)
    : target_cache_(target_cache) {
    reconfigure(config);
}

/// 规范化当前配置并清理依赖旧配置的准备状态。
void Registration::reconfigure(const RegistrationConfig& config) {
    config_ = config;
    const RegistrationConfig defaults;
    config_.voxel_resolution =
        std::max(0.0, finiteOr(config_.voxel_resolution, defaults.voxel_resolution));
    config_.max_iterations = std::max(1, config_.max_iterations);
    config_.max_correspondence_distance = std::max(
        1.0e-3,
        finiteOr(config_.max_correspondence_distance, defaults.max_correspondence_distance));
    config_.plane_knn = std::max(3, config_.plane_knn);
    config_.max_plane_eigenvalue_ratio = std::clamp(
        finiteOr(config_.max_plane_eigenvalue_ratio, defaults.max_plane_eigenvalue_ratio), 0.0,
        1.0);
    config_.min_plane_middle_eigenvalue_ratio =
        std::clamp(finiteOr(config_.min_plane_middle_eigenvalue_ratio,
                            defaults.min_plane_middle_eigenvalue_ratio),
                   0.0, 1.0);
    config_.max_plane_neighbor_distance =
        finiteOr(config_.max_plane_neighbor_distance, defaults.max_plane_neighbor_distance);
    config_.min_correspondences = std::max(1, config_.min_correspondences);
    config_.min_correspondence_ratio = std::clamp(
        finiteOr(config_.min_correspondence_ratio, defaults.min_correspondence_ratio), 0.0, 1.0);
    config_.coarse_trigger_min_correspondence_ratio =
        std::clamp(finiteOr(config_.coarse_trigger_min_correspondence_ratio,
                            defaults.coarse_trigger_min_correspondence_ratio),
                   0.0, config_.min_correspondence_ratio);
    config_.max_rmse = std::max(0.0, finiteOr(config_.max_rmse, defaults.max_rmse));
    config_.huber_delta = std::max(0.0, finiteOr(config_.huber_delta, defaults.huber_delta));
    config_.residual_std_floor =
        std::max(1.0e-4, finiteOr(config_.residual_std_floor, defaults.residual_std_floor));
    config_.measurement_translation_std_floor =
        std::max(0.0, finiteOr(config_.measurement_translation_std_floor,
                               defaults.measurement_translation_std_floor));
    config_.measurement_rotation_std_floor_rad =
        std::max(0.0, finiteOr(config_.measurement_rotation_std_floor_rad,
                               defaults.measurement_rotation_std_floor_rad));
    config_.min_coverage_sectors = std::clamp(config_.min_coverage_sectors, 0, 36);
    config_.degeneracy_condition_threshold = std::max(
        1.0 + 1.0e-6,
        finiteOr(config_.degeneracy_condition_threshold, defaults.degeneracy_condition_threshold));
    invalidateCache();
}

/// 返回当前已经完成平面准备的细层目标点云。
Cloud::ConstPtr Registration::cachedTarget() const {
    return fine_level_ ? Cloud::ConstPtr(fine_level_->target) : Cloud::ConstPtr();
}

/// 按指定尺度准备目标点云、近邻索引和可信局部平面。
bool Registration::prepareLevel(const Cloud::ConstPtr& target, const double scale,
                                PreparedLevel* level) const {
    if (level == nullptr || !target || target->empty()) {
        return false;
    }

    const double voxel = scale > 1.0 ? config_.voxel_resolution * scale : 0.0;
    level->source = target;
    level->target = voxel > 0.0 ? Cloud::ConstPtr(voxelize(*target, voxel)) : target;
    if (!level->target || level->target->empty()) {
        return false;
    }
    level->max_correspondence_distance = config_.max_correspondence_distance * scale;
    const double max_plane_neighbor_distance = config_.max_plane_neighbor_distance > 0.0
                                                   ? config_.max_plane_neighbor_distance * scale
                                                   : config_.max_plane_neighbor_distance;
    level->huber_delta = config_.huber_delta * scale;
    level->min_correspondences =
        scale > 1.0 ? std::max(12, std::min(config_.min_correspondences,
                                            static_cast<int>(std::ceil(config_.min_correspondences /
                                                                       (scale * scale)))))
                    : config_.min_correspondences;
    level->tree.setInputCloud(level->target);
    level->planes.assign(level->target->size(), PreparedPlane{});

    int valid_planes = 0;
    level->tree.query(
        level->target->size(), config_.plane_knn,
        [&](std::size_t i) {
            const auto& p = (*level->target)[i];
            return Eigen::Vector3d(p.x, p.y, p.z);
        },
        /// 从邻域协方差拟合局部平面，并验收谱形状与空间支撑。
        [&](std::size_t point_index, const Eigen::Vector3d& target_point,
            const std::size_t* neighbor_indices, const float* neighbor_squared_distances,
            std::size_t found) {
            if (found < static_cast<std::size_t>(std::max(3, config_.plane_knn / 2)))
                return;
            if (max_plane_neighbor_distance > 0.0 &&
                neighbor_squared_distances[found - 1] >
                    max_plane_neighbor_distance * max_plane_neighbor_distance)
                return;
            Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
            for (std::size_t neighbor = 0; neighbor < found; ++neighbor) {
                const PointType& point = level->target->points[static_cast<std::size_t>(
                    neighbor_indices[static_cast<std::size_t>(neighbor)])];
                centroid += Eigen::Vector3d(point.x, point.y, point.z);
            }
            centroid /= static_cast<double>(found);

            Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
            for (std::size_t neighbor = 0; neighbor < found; ++neighbor) {
                const PointType& point = level->target->points[static_cast<std::size_t>(
                    neighbor_indices[static_cast<std::size_t>(neighbor)])];
                const Eigen::Vector3d delta = Eigen::Vector3d(point.x, point.y, point.z) - centroid;
                covariance.noalias() += delta * delta.transpose();
            }
            covariance /= static_cast<double>(found);

            const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
            if (solver.info() != Eigen::Success) {
                return;
            }
            const Eigen::Vector3d& values = solver.eigenvalues();
            if (values.z() <= 1.0e-12 || values.x() < -1.0e-12 ||
                values.x() / values.z() > config_.max_plane_eigenvalue_ratio ||
                values.y() / values.z() < config_.min_plane_middle_eigenvalue_ratio) {
                return;
            }

            Eigen::Vector3d normal = solver.eigenvectors().col(0).normalized();
            if (normal.dot(target_point - centroid) > 0.0) {
                normal = -normal;
            }
            PreparedPlane& plane = level->planes[point_index];
            plane.point = centroid;
            plane.normal = normal;
            plane.valid = true;
            ++valid_planes;
        });
    return valid_planes >= 3;
}

/// 准备给定目标并更新目标身份；无效输入清空准备状态。
bool Registration::setTarget(const Cloud& target, const int target_id) {
    return setTarget(Cloud::ConstPtr(new Cloud(target)), target_id);
}

/// 准备给定目标并更新目标身份；无效输入清空准备状态。
bool Registration::setTarget(const Cloud::ConstPtr& target, const int target_id) {
    if (target_id >= 0 && target_id == cached_prior_index_ && fine_level_ &&
        fine_level_->source.get() == target.get() && fine_level_->target &&
        !fine_level_->target->empty()) {
        return true;
    }
    if (!target || target->empty()) {
        BACKEND_WARN_STREAM("Registration::setTarget: empty target cloud");
        invalidateCache();
        return false;
    }

    const auto prepared = prepareCachedLevel(target, 1.0);
    if (!prepared) {
        invalidateCache();
        return false;
    }
    cached_prior_index_ = target_id;
    fine_level_ = prepared;
    coarse_level_.reset();
    return true;
}

/// 在需要时从细层持有的原始目标惰性准备粗层。
bool Registration::ensureCoarseLevel() {
    if (coarse_level_) {
        return coarse_level_->target && !coarse_level_->target->empty();
    }
    if (!fine_level_ || !fine_level_->source || fine_level_->source->empty() ||
        config_.voxel_resolution <= 0.0) {
        return false;
    }
    coarse_level_ = prepareCachedLevel(fine_level_->source, 2.0);
    return bool(coarse_level_);
}

/// 按点云身份及完整准备配置取得只读目标层。
std::shared_ptr<const Registration::PreparedLevel> Registration::prepareCachedLevel(
    const Cloud::ConstPtr& target, double scale) const {
    /// 清理非有限点并准备独立缓存层，同时保留原始输入身份。
    const auto load = [&]() -> std::shared_ptr<const PreparedLevel> {
        const Cloud::ConstPtr clean =
            target->is_dense ? target : Cloud::ConstPtr(finiteCloudCopy(*target));
        auto level = std::make_shared<PreparedLevel>();
        if (!prepareLevel(clean, scale, level.get()))
            return {};
        level->source = target;  // 缓存身份使用原始输入点云对象。
        return level;
    };
    if (!target_cache_)
        return load();
    const TargetCache::Impl::Key key{reinterpret_cast<std::uintptr_t>(target.get()),
                                     scale,
                                     config_.voxel_resolution,
                                     config_.plane_knn,
                                     config_.max_plane_eigenvalue_ratio,
                                     config_.min_plane_middle_eigenvalue_ratio,
                                     config_.max_plane_neighbor_distance,
                                     config_.max_correspondence_distance,
                                     config_.huber_delta,
                                     config_.min_correspondences};
    return target_cache_->impl_->cache.get(key, load, [](const PreparedLevel& level) {
        return sizeof(level) + level.planes.capacity() * sizeof(PreparedPlane) +
               level.tree.bytes() +
               (level.source->points.capacity() +
                (level.source == level.target ? 0 : level.target->points.capacity())) *
                   sizeof(PointType);
    });
}

/// 计算对齐源点落入当前细层目标邻域的比例。
double Registration::overlap(const Cloud& aligned, double distance) const {
    if (!fine_level_ || aligned.empty() || distance <= 0)
        return 0;
    std::size_t hits = 0;
    fine_level_->tree.query(
        aligned.size(), 1,
        [&](std::size_t i) {
            const auto& p = aligned[i];
            return Eigen::Vector3d(p.x, p.y, p.z);
        },
        [&](std::size_t, const Eigen::Vector3d&, const std::size_t*, const float* d, std::size_t) {
            if (static_cast<double>(d[0]) <= distance * distance)
                ++hits;
        });
    return static_cast<double>(hits) / aligned.size();
}

/// 清除当前目标身份及细、粗层准备结果。
void Registration::invalidateCache() {
    cached_prior_index_ = -1;
    fine_level_.reset();
    coarse_level_.reset();
}

}  // namespace backend
