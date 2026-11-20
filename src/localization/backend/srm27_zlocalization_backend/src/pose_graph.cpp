/**
 * @file pose_graph.cpp
 * @brief 管理增量求解器生命周期、状态提交和图键转换。
 */

#include "backend/pose_graph.hpp"

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/linearExceptions.h>

#include <cmath>
#include <limits>
#include <set>

namespace backend {

/// 使用归一化配置创建增量位姿图。
PoseGraph::PoseGraph(const PoseGraphConfig& config) {
    reset(config);
}

/// 重建求解器并清空所有待提交状态和共享标定变量。
void PoseGraph::reset(const PoseGraphConfig& config) {
    config_ = config;
    const PoseGraphConfig fallback;
    for (Eigen::Index axis = 0; axis < config_.prior_rotation_variances.size(); ++axis) {
        if (!std::isfinite(config_.prior_rotation_variances(axis)) ||
            config_.prior_rotation_variances(axis) <= 0.0) {
            config_.prior_rotation_variances(axis) = fallback.prior_rotation_variances(axis);
        }
        if (!std::isfinite(config_.prior_translation_variances(axis)) ||
            config_.prior_translation_variances(axis) <= 0.0) {
            config_.prior_translation_variances(axis) = fallback.prior_translation_variances(axis);
        }
    }
    /// 返回有效正数，非法输入使用给定回退值。
    const auto positive_or = [](const double value, const double replacement) {
        return std::isfinite(value) && value > 0.0 ? value : replacement;
    };
    /// 返回有效非负数，非法输入使用给定回退值。
    const auto nonnegative_or = [](const double value, const double replacement) {
        return std::isfinite(value) && value >= 0.0 ? value : replacement;
    };
    config_.odom_noise.rotation_variance_per_second =
        positive_or(config_.odom_noise.rotation_variance_per_second,
                    fallback.odom_noise.rotation_variance_per_second);
    config_.odom_noise.translation_variance_per_second =
        positive_or(config_.odom_noise.translation_variance_per_second,
                    fallback.odom_noise.translation_variance_per_second);
    config_.odom_noise.rotation_variance_per_radian =
        nonnegative_or(config_.odom_noise.rotation_variance_per_radian,
                       fallback.odom_noise.rotation_variance_per_radian);
    config_.odom_noise.translation_variance_per_meter =
        nonnegative_or(config_.odom_noise.translation_variance_per_meter,
                       fallback.odom_noise.translation_variance_per_meter);
    if (!std::isfinite(config_.robust_kernel_size) || config_.robust_kernel_size <= 0.0) {
        config_.robust_kernel_size = fallback.robust_kernel_size;
    }
    if (!std::isfinite(config_.fixed_lag_seconds) || config_.fixed_lag_seconds < 0.0) {
        config_.fixed_lag_seconds = 0.0;
    }
    gtsam::ISAM2Params params;
    params.relinearizeThreshold = 0.01;
    params.relinearizeSkip = 1;
    params.findUnusedFactorSlots = true;
    isam_ = std::make_unique<gtsam::ISAM2>(params);
    pending_graph_ = gtsam::NonlinearFactorGraph();
    pending_values_ = gtsam::Values();
    pose_timestamps_.clear();
    pending_new_indices_.clear();
    uwb_alignment_initialized_ = false;
    uwb_alignment_use_z_ = false;
    uwb_alignment_fixed_z_ = 0.0;
    uwb_time_offset_initialized_ = false;
    timestamp_offset_ = 0.0;
    last_input_timestamp_ = std::numeric_limits<double>::quiet_NaN();
    latest_graph_timestamp_ = std::numeric_limits<double>::quiet_NaN();
}

/// 登记新轨迹节点，并将可能回退的传感器时间映射为单调图时间。
void PoseGraph::recordNewState(const int index, const Eigen::Matrix4d& pose,
                               const double timestamp) {
    pending_values_.insert(index, toGtsam(pose));
    pending_new_indices_.push_back(index);
    if (config_.fixed_lag_seconds > 0.0) {
        double graph_timestamp = static_cast<double>(index);
        if (std::isfinite(timestamp)) {
            // 输入时间回退时重设图时间偏移，后续样本沿用该偏移。
            if (std::isfinite(last_input_timestamp_) &&
                timestamp + 1.0e-3 < last_input_timestamp_) {
                timestamp_offset_ = latest_graph_timestamp_ + 1.0e-3 - timestamp;
            }
            graph_timestamp = timestamp + timestamp_offset_;
            last_input_timestamp_ = timestamp;
        }
        if (std::isfinite(latest_graph_timestamp_) && graph_timestamp <= latest_graph_timestamp_) {
            graph_timestamp = latest_graph_timestamp_ + 1.0e-6;
        }
        latest_graph_timestamp_ = graph_timestamp;
        pose_timestamps_.emplace_back(index, graph_timestamp);
    }
}

/// GTSAM 4.2 的 smoother 存在同团 frontal 重消元漏标缺陷，见上游 borglab/gtsam issue #1638。
/// 共享标定量参与时可触发该缺陷，使树仍引用已删除状态；上游 PR #1890 修复此问题，4.3a1 已包含修复。
/// 为兼容系统 4.2，此处直接控制 iSAM2：仅轨迹位姿过期，共享标定量常驻，并检测团内顺序冲突及子团依赖。
/// 必要时重新消元相关团及其子树，将过期位姿排到叶端后再边缘化，保留存活变量的联合信息。
void PoseGraph::updateWindow() {
    gtsam::KeyList expired;
    for (const auto& [key, timestamp] : pose_timestamps_) {
        if (timestamp >= latest_graph_timestamp_ - config_.fixed_lag_seconds)
            break;
        expired.push_back(key);
    }
    if (expired.empty()) {
        isam_->update(pending_graph_, pending_values_);
        return;
    }

    gtsam::FastMap<gtsam::Key, int> groups;
    for (const auto key : isam_->getLinearizationPoint().keys())
        groups[key] = 1;
    for (const auto key : pending_values_.keys())
        groups[key] = 1;
    for (const auto key : expired)
        groups[key] = 0;

    // 仅在存活变量排在过期变量之前、或子团仍依赖过期变量时重消元。
    // 已排在叶端的过期前缀可直接边缘化，避免每帧重排整个窗口。
    std::vector<gtsam::ISAM2::sharedClique> cliques;
    const std::set<gtsam::Key> expired_set(expired.begin(), expired.end());
    for (const auto key : expired) {
        if (!isam_->valueExists(key))
            continue;
        const auto clique = (*isam_)[key];
        const auto& frontals = clique->conditional()->frontals();
        const auto position = std::find(frontals.begin(), frontals.end(), key);
        bool reorder =
            std::any_of(frontals.begin(), position, [&](auto k) { return !expired_set.count(k); });
        for (const auto& child : clique->children) {
            const auto& c = child->conditional();
            reorder |= std::find(c->beginParents(), c->endParents(), key) != c->endParents();
        }
        if (reorder)
            cliques.push_back(clique);
    }
    std::set<const gtsam::ISAM2Clique*> visited;
    gtsam::KeyList affected;
    while (!cliques.empty()) {
        const auto clique = cliques.back();
        cliques.pop_back();
        if (!visited.insert(clique.get()).second)
            continue;
        const auto& frontals = clique->conditional()->frontals();
        affected.insert(affected.end(), frontals.begin(), frontals.end());
        cliques.insert(cliques.end(), clique->children.begin(), clique->children.end());
    }
    isam_->update(pending_graph_, pending_values_, {}, groups, boost::none, affected);
    isam_->marginalizeLeaves(expired);
    pose_timestamps_.erase(pose_timestamps_.begin(), pose_timestamps_.begin() + expired.size());
}

/// 提交待处理因子和值；全局校正时返回全部活动位姿，否则只返回新节点。
std::vector<std::pair<int, Eigen::Matrix4d>> PoseGraph::update(const bool global_factor_added) {
    try {
        const std::vector<int> new_indices = pending_new_indices_;
        // 全局校正比普通里程计提交多执行一次增量更新。
        updateWindow();
        isam_->update();
        if (global_factor_added)
            isam_->update();
        pending_graph_.resize(0);
        pending_values_.clear();
        pending_new_indices_.clear();

        if (global_factor_added) {
            return activeEstimates();
        }
        std::vector<std::pair<int, Eigen::Matrix4d>> poses;
        poses.reserve(new_indices.size());
        for (const int index : new_indices) {
            if (isam_->valueExists(index)) {
                poses.emplace_back(index, fromGtsam(isam_->calculateEstimate<gtsam::Pose3>(index)));
            }
        }
        return poses;
    } catch (const gtsam::IndeterminantLinearSystemException& error) {
        reset(config_);
        throw NumericalError(error.what());
    } catch (const gtsam::CholeskyFailed&) {
        reset(config_);
        throw NumericalError("incremental graph Cholesky factorization failed");
    }
}

/// 将后端齐次位姿转换为 GTSAM Pose3。
gtsam::Pose3 PoseGraph::toGtsam(const Eigen::Matrix4d& pose) {
    return gtsam::Pose3(gtsam::Rot3(pose.block<3, 3>(0, 0)),
                        gtsam::Point3(pose(0, 3), pose(1, 3), pose(2, 3)));
}

/// 将 GTSAM Pose3 转换为后端齐次位姿。
Eigen::Matrix4d PoseGraph::fromGtsam(const gtsam::Pose3& pose) {
    Eigen::Matrix4d out = Eigen::Matrix4d::Identity();
    out.block<3, 3>(0, 0) = pose.rotation().matrix();
    out(0, 3) = pose.translation().x();
    out(1, 3) = pose.translation().y();
    out(2, 3) = pose.translation().z();
    return out;
}

/// 返回全图唯一的 UWB 平面对齐变量键。
gtsam::Key PoseGraph::uwbAlignmentKey() {
    return gtsam::Symbol('u', 0);
}

/// 返回全图唯一的 UWB 高度对齐变量键。
gtsam::Key PoseGraph::uwbAlignmentZKey() {
    return gtsam::Symbol('h', 0);
}

/// 返回全图唯一的 UWB 时间偏置变量键。
gtsam::Key PoseGraph::uwbTimeOffsetKey() {
    return gtsam::Symbol('d', 0);
}

}  // namespace backend
