/**
 * @file uwb_alignment.cpp
 * @brief 管理 UWB/里程计对齐样本，并实现确定性的 SE(2) 共识几何。
 */

#include "backend/uwb_alignment.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "alignment.hpp"

namespace backend {

using uwb_alignment_detail::kFiniteEpsilon;
using uwb_alignment_detail::median;
using uwb_alignment_detail::yawRotation;

/// 构造 UWB 平面对齐估计器并保存鲁棒估计配置。
UwbAlignmentBootstrapper::UwbAlignmentBootstrapper(const UwbAlignmentConfig& config) {
    reset(config);
}

/// 规范化 UWB 对齐配置并清空样本、初值与连续稳定性状态。
void UwbAlignmentBootstrapper::reset(const UwbAlignmentConfig& config,
                                     const std::optional<Eigen::Matrix4d>& initial_guess) {
    config_ = config;
    const UwbAlignmentConfig defaults;
    config_.max_pairs = std::max(2, config_.max_pairs);
    config_.min_pairs = std::clamp(config_.min_pairs, 2, config_.max_pairs);
    /// 对正值配置保留有限输入，其余情况使用默认值。
    const auto positive_or = [](const double value, const double fallback) {
        return std::isfinite(value) && value > 0.0 ? value : fallback;
    };
    config_.min_baseline = positive_or(config_.min_baseline, defaults.min_baseline);
    config_.inlier_threshold = positive_or(config_.inlier_threshold, defaults.inlier_threshold);
    config_.max_rmse = positive_or(config_.max_rmse, defaults.max_rmse);
    config_.position_std_xy = positive_or(config_.position_std_xy, defaults.position_std_xy);
    config_.position_std_z = positive_or(config_.position_std_z, defaults.position_std_z);
    config_.max_yaw_std_rad = positive_or(config_.max_yaw_std_rad, defaults.max_yaw_std_rad);
    config_.stable_solutions = std::max(1, config_.stable_solutions);
    config_.stable_translation_delta =
        positive_or(config_.stable_translation_delta, defaults.stable_translation_delta);
    config_.stable_yaw_delta_rad =
        positive_or(config_.stable_yaw_delta_rad, defaults.stable_yaw_delta_rad);
    config_.gravity_alignment_std_rad =
        positive_or(config_.gravity_alignment_std_rad, defaults.gravity_alignment_std_rad);

    pairs_.clear();
    initial_guess_.reset();
    if (initial_guess.has_value() && initial_guess->allFinite()) {
        initial_guess_ = *initial_guess;
    }
    previous_observable_estimate_.reset();
    consecutive_stable_solutions_ = 0;
    last_timestamp_ = -std::numeric_limits<double>::infinity();
}

/// 保存有限的里程计到地图初始变换，供后续共识假设使用。
void UwbAlignmentBootstrapper::setInitialGuess(const Eigen::Matrix4d& T_map_odom) {
    if (T_map_odom.allFinite()) {
        initial_guess_ = T_map_odom;
    }
}

/// 添加时间配对的位置样本并维护有界估计窗口。
std::optional<UwbAlignmentEstimate> UwbAlignmentBootstrapper::addPair(
    const double timestamp, const Eigen::Vector3d& position_odom_antenna,
    const Eigen::Vector3d& position_map_antenna) {
    const bool odom_finite = config_.use_z ? position_odom_antenna.allFinite()
                                           : position_odom_antenna.head<2>().allFinite();
    const bool map_finite = config_.use_z ? position_map_antenna.allFinite()
                                          : position_map_antenna.head<2>().allFinite();
    if (!std::isfinite(timestamp) || !odom_finite || !map_finite ||
        timestamp <= last_timestamp_ + 1.0e-9) {
        return std::nullopt;
    }
    last_timestamp_ = timestamp;
    Eigen::Vector3d odom = position_odom_antenna;
    Eigen::Vector3d map = position_map_antenna;
    if (!config_.use_z) {
        // 水平模型不保留不可信的 Z，避免其参与后续有限值判断。
        odom.z() = 0.0;
        map.z() = 0.0;
    }
    pairs_.push_back({timestamp, odom, map});
    if (static_cast<int>(pairs_.size()) > config_.max_pairs) {
        pairs_.erase(
            pairs_.begin(),
            pairs_.begin() + (pairs_.size() - static_cast<std::size_t>(config_.max_pairs)));
    }
    return estimate();
}

/// 按原始时间戳的精确身份保留活动配对，不将邻近时刻合并为同一样本。
void UwbAlignmentBootstrapper::retainPairs(const std::vector<double>& timestamps) {
    pairs_.erase(std::remove_if(pairs_.begin(), pairs_.end(),
                                [&](const Pair& pair) {
                                    return std::find(timestamps.begin(), timestamps.end(),
                                                     pair.timestamp) == timestamps.end();
                                }),
                 pairs_.end());
}

/// 使用指定样本拟合平面刚体对齐及可选高度偏移。
bool UwbAlignmentBootstrapper::fitIndices(const std::vector<std::size_t>& indices,
                                          Candidate* candidate) const {
    if (candidate == nullptr || indices.size() < 2U) {
        return false;
    }
    Eigen::Vector2d odom_mean = Eigen::Vector2d::Zero();
    Eigen::Vector2d map_mean = Eigen::Vector2d::Zero();
    for (const std::size_t index : indices) {
        odom_mean += pairs_[index].odom.head<2>();
        map_mean += pairs_[index].map.head<2>();
    }
    odom_mean /= static_cast<double>(indices.size());
    map_mean /= static_cast<double>(indices.size());

    double c = 0.0;
    double s = 0.0;
    double spread = 0.0;
    for (const std::size_t index : indices) {
        const Eigen::Vector2d odom = pairs_[index].odom.head<2>() - odom_mean;
        const Eigen::Vector2d map = pairs_[index].map.head<2>() - map_mean;
        c += odom.dot(map);
        s += odom.x() * map.y() - odom.y() * map.x();
        spread += odom.squaredNorm();
    }
    if (!std::isfinite(c) || !std::isfinite(s) || spread <= kFiniteEpsilon ||
        std::hypot(c, s) <= kFiniteEpsilon) {
        return false;
    }
    candidate->yaw = std::atan2(s, c);
    candidate->translation = map_mean - yawRotation(candidate->yaw) * odom_mean;
    if (config_.use_z) {
        std::vector<double> z_offsets;
        z_offsets.reserve(indices.size());
        for (const std::size_t index : indices) {
            z_offsets.push_back(pairs_[index].map.z() - pairs_[index].odom.z());
        }
        candidate->translation_z = median(std::move(z_offsets));
    }
    return candidate->translation.allFinite();
}

/// 按当前对齐假设和残差阈值选择内点。
void UwbAlignmentBootstrapper::classifyInliers(Candidate* candidate) const {
    if (candidate == nullptr) {
        return;
    }
    candidate->inliers.clear();
    double squared_error = 0.0;
    const Eigen::Matrix2d rotation = yawRotation(candidate->yaw);
    for (std::size_t index = 0; index < pairs_.size(); ++index) {
        const Eigen::Vector2d residual = rotation * pairs_[index].odom.head<2>() +
                                         candidate->translation - pairs_[index].map.head<2>();
        double squared_residual = residual.squaredNorm();
        if (config_.use_z) {
            const double residual_z =
                pairs_[index].odom.z() + candidate->translation_z - pairs_[index].map.z();
            squared_residual += residual_z * residual_z;
        }
        const double error = std::sqrt(squared_residual);
        if (std::isfinite(error) && error <= config_.inlier_threshold) {
            candidate->inliers.push_back(index);
            squared_error += squared_residual;
        }
    }
    candidate->rmse =
        candidate->inliers.empty()
            ? std::numeric_limits<double>::infinity()
            : std::sqrt(squared_error / static_cast<double>(candidate->inliers.size()));
}

/// 计算样本位置跨度以检验航向估计的空间支撑。
double UwbAlignmentBootstrapper::baseline(const std::vector<Pair>& pairs,
                                          const std::vector<std::size_t>& indices,
                                          const bool use_map) {
    double maximum = 0.0;
    for (std::size_t lhs = 0; lhs < indices.size(); ++lhs) {
        for (std::size_t rhs = lhs + 1U; rhs < indices.size(); ++rhs) {
            const Eigen::Vector2d a =
                use_map ? pairs[indices[lhs]].map.head<2>() : pairs[indices[lhs]].odom.head<2>();
            const Eigen::Vector2d b =
                use_map ? pairs[indices[rhs]].map.head<2>() : pairs[indices[rhs]].odom.head<2>();
            maximum = std::max(maximum, (a - b).norm());
        }
    }
    return maximum;
}

}  // namespace backend
