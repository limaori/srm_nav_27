/**
 * @file localization_trajectory.cpp
 * @brief 维护轨迹关键帧、图更新与跨代锚约束。
 */

#include <cmath>
#include <utility>

#include "backend/covariance.hpp"
#include "backend/localization.hpp"
#include "backend/math.hpp"

namespace backend {
/// 根据关键帧间时间、平移和转角计算里程计过程协方差。
PoseCovariance6d Localization::odometryFactorCovariance(const Keyframe& previous,
                                                        const FrontendFrame& current) const {
    const Eigen::Matrix4d measured_between = previous.odom_pose.inverse() * current.odom_pose;
    return config_.graph.odom_noise.covariance(current.timestamp - previous.timestamp,
                                               measured_between.block<3, 1>(0, 3).norm(),
                                               rotationAngleFromMatrix(measured_between));
}

/// 为启用回环的有效关键帧建立地点描述子。
void Localization::addDescriptorLocked(const Keyframe& keyframe) {
    if (!config_.loop.enabled)
        return;
    if (keyframe.index < 0 || !keyframe.cloud ||
        trajectory_descriptor_.size() != static_cast<std::size_t>(keyframe.index))
        throw std::logic_error("runtime descriptor index does not match the trajectory");
    addKeyframeDescriptor(trajectory_descriptor_, keyframe);
    if (trajectory_descriptor_.size() != static_cast<std::size_t>(keyframe.index) + 1U)
        throw std::logic_error("runtime descriptor did not append one keyframe");
}

/// 将图优化后的关键帧位置同步到描述子检索索引。
void Localization::updateDescriptorPositionsLocked(
    const std::vector<std::pair<int, Eigen::Matrix4d>>& poses) {
    if (!config_.loop.enabled) {
        return;
    }
    for (const auto& [index, pose] : poses) {
        trajectory_descriptor_.updateKeyframePosition(index, pose.block<3, 1>(0, 3));
    }
}

/// 提交图因子并刷新轨迹、共享标定及可信输出协方差锚。
PoseCorrectionDelta Localization::updateGraphLocked(const bool global_factor_added) {
    PoseCorrectionDelta applied_correction;
    Eigen::Matrix4d output_pose_before = Eigen::Matrix4d::Identity();
    bool output_pose_before_valid = false;
    if (frames_.hasLatest() && frames_.latest().epoch == active_epoch_) {
        output_pose_before = poses_.transform() * frames_.latest().odom_pose;
        output_pose_before_valid = output_pose_before.allFinite();
    } else if (!trajectory_.empty()) {
        const Keyframe latest_before = trajectory_.latest();
        output_pose_before = poses_.transform() * latest_before.odom_pose;
        output_pose_before_valid = output_pose_before.allFinite();
    }

    const std::vector<std::pair<int, Eigen::Matrix4d>> optimized =
        graph_.update(global_factor_added);
    uwb_.refreshAlignment();
    uwb_.refreshTimeOffset(global_factor_added);
    trajectory_.updateOptimizedPoses(optimized);
    if (trajectory_.empty()) {
        return applied_correction;
    }
    const Keyframe latest = trajectory_.latest();
    poses_.setTransform(latest.optimized_pose * latest.odom_pose.inverse());
    if (output_pose_before_valid) {
        Eigen::Matrix4d output_pose_after = latest.optimized_pose;
        if (frames_.hasLatest() && frames_.latest().epoch == active_epoch_) {
            output_pose_after = poses_.transform() * frames_.latest().odom_pose;
        }
        const Eigen::Matrix4d delta = output_pose_before.inverse() * output_pose_after;
        if (delta.allFinite()) {
            applied_correction.translation = delta.block<3, 1>(0, 3).norm();
            applied_correction.rotation_rad = rotationAngleFromMatrix(delta);
        }
    }
    poses_.refreshAnchor(latest, graph_);
    if (global_factor_added) {
        updateDescriptorPositionsLocked(optimized);
    }
    // 输出版本同时覆盖协方差锚，纯里程计提交也会改变同一时刻的预测协方差。
    poses_.advanceRevision();
    commitCurrentTrustedPoseLocked();
    return applied_correction;
}

/// 选择跨代边、同代里程计边或独立先验，并保留各自的协方差坐标。
void Localization::addTrajectoryConstraintLocked(const FrontendFrame& frame,
                                                 const Keyframe& keyframe,
                                                 const std::optional<Keyframe>& previous,
                                                 const Eigen::Matrix4d& initial_map_pose) {
    const EpochAnchor anchor = std::exchange(pending_epoch_anchor_, std::monostate{});
    const auto* odometry = std::get_if<PendingResetOdometry>(&anchor);
    if (odometry && odometry->target_epoch == keyframe.epoch &&
        graph_.hasEstimate(odometry->from_index)) {
        const Eigen::Matrix4d motion = odometry->reference_odom.inverse() * keyframe.odom_pose;
        const Eigen::Matrix4d measured_between = odometry->from_reference * motion;
        PoseCovariance6d covariance;
        if (odometry->prefix_covariance && keyframe.timestamp == odometry->reference_timestamp) {
            // 停车首帧直接入图时，完整保留原停车边，不重复增加零时长过程噪声。
            covariance = *odometry->prefix_covariance;
        } else {
            covariance = config_.graph.odom_noise.covariance(
                keyframe.timestamp - odometry->reference_timestamp, motion.block<3, 1>(0, 3).norm(),
                rotationAngleFromMatrix(motion));
            if (odometry->prefix_covariance) {
                // 将前缀右扰动搬到当前帧，再与独立运动噪声相加。
                const PoseCovariance6d adjoint = poseAdjoint(motion.inverse());
                covariance += adjoint * *odometry->prefix_covariance * adjoint.transpose();
            }
        }
        graph_.addResetContinuityMeasurement(odometry->from_index, keyframe.index, measured_between,
                                             initial_map_pose, covariance, keyframe.timestamp);
        return;
    }
    if (previous && previous->epoch == keyframe.epoch && graph_.hasEstimate(previous->index)) {
        const Eigen::Matrix4d measured_between = previous->odom_pose.inverse() * keyframe.odom_pose;
        graph_.addOdometryMeasurement(previous->index, keyframe.index, measured_between,
                                      initial_map_pose, odometryFactorCovariance(*previous, frame),
                                      keyframe.timestamp);
        return;
    }
    std::optional<PoseCovariance6d> prior_covariance;
    if (const auto* covariance = std::get_if<PoseCovariance6d>(&anchor))
        prior_covariance = *covariance;
    else if (poses_.hasTrustedCovariance())
        prior_covariance = poses_.covarianceAt(frame);
    if (!previous && !prior_covariance && !config_.prior_map_enabled) {
        graph_.addPrior(keyframe.index, initial_map_pose, keyframe.timestamp);
    } else {
        const PoseCovariance6d covariance_map =
            prior_covariance ? *prior_covariance
                             : (frame.pose_covariance_valid ? frame.pose_covariance
                                                            : fallbackFrontendPoseCovariance());
        const Eigen::Matrix4d& covariance_pose =
            prior_covariance ? initial_map_pose : frame.odom_pose;
        graph_.addPrior(keyframe.index, initial_map_pose,
                        worldAdditiveToLocalPoseCovariance(covariance_pose, covariance_map),
                        keyframe.timestamp);
    }
}

/// 创建轨迹关键帧，并依次提交图约束、优化结果和描述子。
void Localization::addTrajectoryKeyframeLocked(const FrontendFrame& frame) {
    std::optional<Keyframe> previous;
    if (!trajectory_.empty())
        previous = trajectory_.latest();

    Keyframe keyframe = trajectory_.add(frame);
    frames_.capture(keyframe.index, frame,
                    std::vector<FrontendFrame>(frames_.history().begin(), frames_.history().end()));
    const Eigen::Matrix4d initial_map_pose = poses_.transform() * frame.odom_pose;
    const bool graph_required =
        config_.prior_map_enabled || config_.loop.enabled || config_.uwb.enabled;
    if (!graph_required) {
        // 纯里程计模式保存轨迹和点云，直接按全局规范投影关键帧。
        const std::vector<std::pair<int, Eigen::Matrix4d>> optimized_pose{
            {keyframe.index, initial_map_pose}};
        trajectory_.updateOptimizedPoses(optimized_pose);
        pending_epoch_anchor_ = std::monostate{};
        return;
    }
    addTrajectoryConstraintLocked(frame, keyframe, previous, initial_map_pose);
    if (!config_.prior_map_enabled && config_.uwb.enabled) {
        // 无先验 UWB 模式使用前端重力方向，约束 map 与 odom 的横滚、俯仰对齐。
        graph_.addGravityAlignment(keyframe.index, frame.odom_pose,
                                   config_.uwb.bootstrap.gravity_alignment_std_rad);
    }
    uwb_.ensureTimeOffsetState();
    // 此处只提交里程计预测；该关键帧的连续外部观测都相对同一已提交预测检验，
    // 再由全局观测提交阶段一次性插入。
    updateGraphLocked(/*global_factor_added=*/false);
    trajectory_.get(keyframe.index, &keyframe);
    addDescriptorLocked(keyframe);
    if (config_.prior_map_enabled && !config_.loop.enabled) {
        // 未启用运行回环时，先验跟踪只需保留最新运行锚。
        trajectory_.retainLatest(1);
    }
}

/// 以恢复观测建立独立全位姿锚，并按需验收附加 UWB 因子。
void Localization::anchorRecoveredFrameLocked(const FrontendFrame& frame,
                                              const Eigen::Matrix4d& T_map_sensor,
                                              const PoseCovariance6d& covariance,
                                              std::vector<FrontendFrame> tracking_source_history,
                                              const bool add_uwb_factor) {
    // 恢复观测会在不可信区间后建立新的绝对分量。重置连续性边只服务于无全局
    // 观测的快速恢复路径，不能跨过该锚点连接到恢复前轨迹。
    pending_epoch_anchor_ = std::monostate{};
    if (!trajectory_.empty()) {
        const Keyframe latest = trajectory_.latest();
        if (latest.epoch == frame.epoch && std::abs(latest.timestamp - frame.timestamp) <= 1.0e-3 &&
            graph_.hasEstimate(latest.index)) {
            frames_.capture(latest.index, frame, std::move(tracking_source_history));
            uwb_.ensureTimeOffsetState();
            graph_.addAbsolutePose(latest.index, T_map_sensor, covariance);
            updateGraphLocked(/*global_factor_added=*/true);
            Keyframe anchored = latest;
            trajectory_.get(latest.index, &anchored);
            if (add_uwb_factor && uwb_.addToRecovery(anchored)) {
                updateGraphLocked(/*global_factor_added=*/true);
            }
            return;
        }
    }

    Keyframe keyframe = trajectory_.add(frame);
    frames_.capture(keyframe.index, frame, std::move(tracking_source_history));
    // 定位不可信时不写普通里程计因子，也不事后跨越冻结区间补边；恢复观测建立新的
    // 绝对锚定图分量。
    graph_.addPrior(keyframe.index, T_map_sensor, covariance, keyframe.timestamp);
    if (!config_.prior_map_enabled && config_.uwb.enabled) {
        graph_.addGravityAlignment(keyframe.index, frame.odom_pose,
                                   config_.uwb.bootstrap.gravity_alignment_std_rad);
    }
    uwb_.ensureTimeOffsetState();
    keyframe.optimized_pose = T_map_sensor;
    addDescriptorLocked(keyframe);
    updateGraphLocked(/*global_factor_added=*/true);
    trajectory_.get(keyframe.index, &keyframe);
    if (add_uwb_factor && uwb_.addToRecovery(keyframe)) {
        updateGraphLocked(/*global_factor_added=*/true);
    }
    if (config_.prior_map_enabled && !config_.loop.enabled) {
        trajectory_.retainLatest(1);
    }
}

/// 将当前代次的最新同步帧提交为可信输出位姿。
void Localization::commitCurrentTrustedPoseLocked() {
    if (frames_.hasLatest()) {
        if (LocalizationStateMachine::poseTrusted(state_.status()))
            odometry_connected_ = true;
        poses_.commitTrusted(frames_.latest(), active_epoch_,
                             LocalizationStateMachine::poseTrusted(state_.status()));
    }
}

}  // namespace backend
