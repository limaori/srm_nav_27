/**
 * @file localization_recovery.cpp
 * @brief 校验恢复任务身份并协调前端重置及可信锚提交。
 */

#include <cmath>

#include "backend/covariance.hpp"
#include "backend/localization.hpp"
#include "backend/logger.hpp"
#include "backend/math.hpp"

namespace backend {
/// 校验恢复任务的代次、状态及请求身份；不要求捕获帧仍是最新帧。
bool Localization::recoveryIsCurrentLocked(const RecoveryIdentity& identity) const {
    return active_epoch_ == identity.epoch && state_.status() == identity.status &&
           LocalizationStateMachine::recoveryPath(state_.status()) &&
           state_.generation() == identity.state_generation && !reset_coordinator_.pending() &&
           reset_coordinator_.lastSentTransactionId() == identity.reset_transaction_id &&
           (manual_request_ ? manual_request_->id : 0) == identity.manual_request_id;
}

/// 仅在认领恢复任务时要求点云和时间身份仍对应最新同步帧。
bool Localization::sourceIsLatestLocked(const RecoveryWork& work) const {
    return frames_.hasLatest() && frames_.latest().epoch == work.frame.epoch &&
           std::abs(frames_.latest().timestamp - work.frame.timestamp) < 1e-9 &&
           frames_.latest().cloud_local == work.frame.cloud_local;
}

/// 确认同步帧严格晚于手动请求捕获的输入边界。
bool Localization::manualRequestAllowsFrameLocked(const FrontendFrame& frame) const {
    if (!manual_request_) {
        return true;
    }
    if (frame.epoch != manual_request_->epoch) {
        return frame.epoch > manual_request_->epoch;
    }
    return !std::isfinite(manual_request_->after_timestamp) ||
           frame.timestamp > manual_request_->after_timestamp;
}

/// 将时间对齐的 UWB 天线位置及杆臂构造成恢复位置先验。
std::optional<RelocalizationPositionPrior> Localization::makeUwbRecoveryPriorLocked(
    const FrontendFrame& frame) const {
    const std::optional<UwbObservation>& latest_uwb = uwb_.observations().latest();
    if (!config_.uwb.enabled || !latest_uwb.has_value() || frames_.odometry().empty()) {
        return std::nullopt;
    }

    const double motion_timestamp = uwb_.motionTimestamp(*latest_uwb);
    const double age = frame.timestamp - motion_timestamp;
    if (age < -config_.uwb.odom_sync_tolerance || age > config_.uwb.max_age) {
        return std::nullopt;
    }
    const std::optional<Eigen::Matrix4d> timed_odom_pose =
        frames_.odometry().poseAt(motion_timestamp, config_.uwb.odom_sync_tolerance);
    if (!timed_odom_pose.has_value()) {
        return std::nullopt;
    }

    const Eigen::Matrix3d R_odom_sensor = frame.odom_pose.block<3, 3>(0, 0);
    const Eigen::Matrix3d R_odom_body = R_odom_sensor * config_.uwb.T_sensor_body.block<3, 3>(0, 0);
    const double body_yaw = yawFromRotation(R_odom_body);

    RelocalizationPositionPrior prior;
    prior.source = "uwb";
    prior.use_z = config_.uwb.use_z;
    prior.map_point = uwb_.positionMap(*latest_uwb);
    prior.R_yawless_sensor =
        yawGuess(-body_yaw, Eigen::Vector3d::Zero()).block<3, 3>(0, 0) * R_odom_sensor;
    const Eigen::Vector3d antenna_odom =
        timed_odom_pose->block<3, 3>(0, 0) * config_.uwb.antenna_position_sensor +
        timed_odom_pose->block<3, 1>(0, 3);
    prior.runtime_sensor_point = frame.odom_pose.block<3, 3>(0, 0).transpose() *
                                 (antenna_odom - frame.odom_pose.block<3, 1>(0, 3));
    // 初始代次的启动提示提供 UWB 分支航向初值，仍需通过恢复验收；后续代次使用当前观测。
    if (state_.status() == LocalizationStatus::Initializing && frame.epoch == 0 &&
        config_.startup_hint.T_map_sensor.has_value()) {
        const Eigen::Matrix3d R_map_body = config_.startup_hint.T_map_sensor->block<3, 3>(0, 0) *
                                           config_.uwb.T_sensor_body.block<3, 3>(0, 0);
        prior.yaw_seed_rad = yawFromRotation(R_map_body);
    } else {
        prior.yaw_seed_rad.reset();
    }
    prior.search_radius = config_.uwb.recovery_search_radius;
    prior.max_candidates = config_.uwb.recovery_candidates;
    if (!prior.map_point.allFinite() || !prior.runtime_sensor_point.allFinite()) {
        return std::nullopt;
    }
    return prior;
}

/// 在第零代初始化期间构造待几何验证的启动提示。
std::optional<RelocalizationPositionPrior> Localization::makeStartupHintPriorLocked(
    const FrontendFrame& frame) const {
    if (!config_.prior_map_enabled || state_.status() != LocalizationStatus::Initializing ||
        frame.epoch != 0 || !config_.startup_hint.T_map_sensor.has_value()) {
        return std::nullopt;
    }

    const Eigen::Matrix4d& T_map_sensor = *config_.startup_hint.T_map_sensor;
    const Eigen::Matrix3d R_map_body =
        T_map_sensor.block<3, 3>(0, 0) * config_.uwb.T_sensor_body.block<3, 3>(0, 0);
    const double body_yaw = yawFromRotation(R_map_body);

    RelocalizationPositionPrior prior;
    prior.source = "startup_hint";
    prior.map_point = T_map_sensor.block<3, 1>(0, 3);
    prior.runtime_sensor_point = Eigen::Vector3d::Zero();
    prior.R_yawless_sensor = yawGuess(-body_yaw, Eigen::Vector3d::Zero()).block<3, 3>(0, 0) *
                             T_map_sensor.block<3, 3>(0, 0);
    prior.yaw_seed_rad = body_yaw;
    prior.allow_yaw_sweep_fallback = false;
    prior.search_radius = config_.recovery.suspect_search_radius;
    prior.max_candidates = config_.relocalization.relocalization_max_candidates;
    return prior;
}

/// 冻结当前恢复任务的身份、源历史及位置先验。
std::optional<RecoveryWork> Localization::captureRecovery() {
    std::lock_guard<std::mutex> lock(mutex_);
    RecoveryWork work;

    const LocalizationStatus current = state_.status();
    if (!LocalizationStateMachine::recoveryPath(current) || reset_coordinator_.pending() ||
        !frames_.hasLatest()) {
        return std::nullopt;
    }
    if (current == LocalizationStatus::Lost && !config_.recovery.automatic_when_lost &&
        !manual_request_) {
        return std::nullopt;
    }
    if (manual_request_.has_value() && !manualRequestAllowsFrameLocked(frames_.latest())) {
        return std::nullopt;
    }
    if (std::abs(frames_.latest().timestamp - last_recovery_timestamp_) < 1.0e-9) {
        return std::nullopt;
    }

    work.frame = frames_.latest();
    work.runtime.index =
        !trajectory_.empty() && trajectory_.latest().epoch == frames_.latest().epoch &&
                std::abs(trajectory_.latest().timestamp - frames_.latest().timestamp) <= 1.0e-3
            ? trajectory_.latest().index
            : trajectory_.nextIndex();
    work.runtime.epoch = frames_.latest().epoch;
    work.runtime.timestamp = frames_.latest().timestamp;
    work.runtime.odom_pose = frames_.latest().odom_pose;
    work.identity.status = current;
    work.identity.state_generation = state_.generation();
    work.identity.reset_transaction_id = reset_coordinator_.lastSentTransactionId();
    work.identity.manual_request_id = manual_request_ ? manual_request_->id : 0;
    work.identity.epoch = active_epoch_;
    work.predicted_map_pose = poses_.transform() * frames_.latest().odom_pose;
    if (current == LocalizationStatus::Suspect && poses_.trusted().hasPose()) {
        // SUSPECT 只在最后可信位置附近召回；LOST 保持宽域或全局搜索。
        work.predicted_map_pose = poses_.trusted().mapSensorPose();
    }
    work.startup_hint_prior = makeStartupHintPriorLocked(frames_.latest());
    work.force_startup_hint =
        work.startup_hint_prior.has_value() && config_.startup_hint.force_until_matched;
    work.uwb_prior = makeUwbRecoveryPriorLocked(frames_.latest());
    work.source_history.assign(frames_.history().begin(), frames_.history().end());
    return work;
}

/// 安装恢复位姿锚，并把匹配期间的新里程计运动传播到当前可信输出。
PoseCorrectionDelta Localization::applyRecoveryLocked(const RecoveryWork& work,
                                                      const RecoveryResult& result,
                                                      LocalizationTransition& transition) {
    PoseCorrectionDelta applied_correction;
    Eigen::Matrix4d output_pose_before = poses_.transform() * frames_.latest().odom_pose;
    if (poses_.trusted().hasPose()) {
        output_pose_before = poses_.trusted().mapSensorPose();
    }
    // 恢复观测属于捕获帧，同状态代内的新帧继续使用校正后的全局规范。
    poses_.setTransform(result.pose * work.frame.odom_pose.inverse());
    anchorRecoveredFrameLocked(work.frame, result.pose, result.covariance, work.source_history);
    if (!trajectory_.empty()) {
        const Keyframe anchored = trajectory_.latest();
        if (anchored.epoch == work.frame.epoch &&
            std::abs(anchored.timestamp - work.frame.timestamp) <= 1.0e-3 &&
            uwb_.observeAnchor(anchored, result.pose)) {
            // 恢复锚已先提交；第二个事务共同引入 UWB 坐标对齐状态与历史原始观测因子。
            updateGraphLocked(/*global_factor_added=*/true);
        }
    }
    last_tracking_reference_index_ = -1;
    last_lidar_observation_runtime_index_ = trajectory_.latest().index;
    last_global_observation_cycle_time_ = Timestamp();
    clearManualRelocalizationRequestLocked();
    transition = state_.onTrustedCorrection("recovered via " + result.source,
                                            TrustedCorrectionSource::Relocalization);
    // 将匹配期间到达的里程计运动传播到最新同步帧，并提交可信输出。
    commitCurrentTrustedPoseLocked();
    const Eigen::Matrix4d output_pose_after = poses_.transform() * frames_.latest().odom_pose;
    const Eigen::Matrix4d correction = output_pose_before.inverse() * output_pose_after;
    if (correction.allFinite()) {
        applied_correction.translation = correction.block<3, 1>(0, 3).norm();
        applied_correction.rotation_rad = rotationAngleFromMatrix(correction);
    }
    return applied_correction;
}

/// 校验恢复任务身份并提交完整位姿锚或累计恢复失败。
void Localization::commit(const RecoveryWork& work, const RecoveryResult& result) {
    if (result.outcome == RecoveryOutcome::Deferred)
        return;
    const bool matched = result.outcome == RecoveryOutcome::Matched;
    PoseCorrectionDelta applied_correction;
    LocalizationTransition transition;
    if (!withOutputCommit([&] {
            if (!recoveryIsCurrentLocked(work.identity)) {
                BACKEND_INFO_STREAM("discarding stale recovery result: runtime="
                                    << work.runtime.index << ", epoch=" << work.frame.epoch
                                    << ", generation=" << work.identity.state_generation
                                    << ", reset_transaction=" << work.identity.reset_transaction_id
                                    << "->" << reset_coordinator_.lastSentTransactionId()
                                    << ", manual_request=" << work.identity.manual_request_id);
                return false;
            }
            if (matched) {
                applied_correction = applyRecoveryLocked(work, result, transition);
                estimator_failed_ = false;
            } else {
                if (work.identity.manual_request_id != 0)
                    clearManualRelocalizationRequestLocked();
                transition = state_.onRecoveryRelocalizationFailure(
                    config_.recovery.suspect_failures_before_lost, result.source);
            }
            return true;
        }))
        return;
    publishState();
    if (matched) {
        visualization_sink_.publishVisualization(visualization());
        BACKEND_INFO_STREAM("localization state -> "
                            << LocalizationStateMachine::statusName(transition.current) << " ("
                            << transition.reason << ", source_frames=" << work.accumulated_frames
                            << "/" << config_.recovery.source.max_frames
                            << ", source_span=" << work.source_span_seconds
                            << "s, delta_t=" << applied_correction.translation << "m, delta_r_deg="
                            << applied_correction.rotation_rad * 180.0 / M_PI << ")");
    } else {
        BACKEND_WARN_STREAM("localization recovery pending: " << transition.reason);
        if (transition.current == LocalizationStatus::Lost)
            triggerFrontendReset();
    }
}
/// 复核恢复身份与最新源帧，并按节流规则认领任务。
bool Localization::claim(RecoveryWork& work) {
    const bool waiting = !work.startup_hint_prior &&
                         work.source_evidence.disposition == ObservationDisposition::Unavailable;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto current = state_.status();
        const bool same = recoveryIsCurrentLocked(work.identity) && sourceIsLatestLocked(work);
        if (!waiting) {
            if (!same)
                return false;
            if (current == LocalizationStatus::Lost && !config_.recovery.automatic_when_lost &&
                work.identity.manual_request_id == 0)
                return false;
            if (!work.force_startup_hint &&
                !tryStartCycle(clock_.nowSteadyTimestamp(), config_.recovery.period,
                               last_recovery_cycle_time_))
                return false;
        }
        // 等待新源帧不消费周期或手动请求；旧帧在这一边界后不会反复检查。
        if (same)
            last_recovery_timestamp_ = work.frame.timestamp;
    }
    if (waiting && work.identity.manual_request_id != 0) {
        BACKEND_INFO_STREAM("manual relocalization request "
                            << work.identity.manual_request_id << " waiting for recovery source ("
                            << work.accumulated_frames << "/" << config_.recovery.source.min_frames
                            << " frames, span=" << work.source_span_seconds << "s)");
    }
    return !waiting;
}
/// 在 LOST 状态记录等待下一有效帧的手动恢复请求。
bool Localization::requestManualRelocalization() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_.status() != LocalizationStatus::Lost) {
        return false;
    }
    std::uint64_t request_id = ++next_manual_relocalization_request_id_;
    if (request_id == 0) {
        request_id = ++next_manual_relocalization_request_id_;
    }
    manual_request_ = ManualRequest{request_id, active_epoch_,
                                    frames_.hasLatest() ? frames_.latest().timestamp
                                                        : std::numeric_limits<double>::quiet_NaN()};
    return true;
}

/// 清除已经完成或失效的手动恢复请求。
void Localization::clearManualRelocalizationRequestLocked() {
    manual_request_.reset();
}

/// 验收可观的 UWB 启动估计并将其协方差传播到恢复位姿锚。
bool Localization::tryCommitUwbBootstrapLocked(const FrontendFrame& frame) {
    if (state_.status() != LocalizationStatus::Initializing &&
        state_.status() != LocalizationStatus::Suspect)
        return false;
    const auto estimate = uwb_.bootstrap(frame);
    if (!estimate)
        return false;
    const Eigen::Matrix4d output_pose_before = poses_.transform() * frame.odom_pose;
    const Eigen::Matrix4d candidate_T_map_odom = estimate->T_map_odom;
    const Eigen::Matrix4d T_map_sensor = candidate_T_map_odom * frame.odom_pose;
    PoseCorrectionDelta applied_correction;
    const Eigen::Matrix4d correction = output_pose_before.inverse() * T_map_sensor;
    if (correction.allFinite()) {
        applied_correction.translation = correction.block<3, 1>(0, 3).norm();
        applied_correction.rotation_rad = rotationAngleFromMatrix(correction);
    }
    // 拟合协方差属于 [对齐平移, 对齐 yaw]；先传播到当前传感器位姿，再把得到的
    // 世界加性协方差转换为 GTSAM 位姿局部切空间。
    const Eigen::Matrix3d R_map_odom = candidate_T_map_odom.block<3, 3>(0, 0);
    const Eigen::Vector3d position_odom = frame.odom_pose.block<3, 1>(0, 3);
    PoseCovariance6d covariance_map =
        alignmentToPoseCovariance(estimate->covariance, R_map_odom, position_odom);
    if (frame.pose_covariance_valid) {
        covariance_map +=
            rotatePoseCovariance(frame.pose_covariance, candidate_T_map_odom.block<3, 3>(0, 0));
    }
    PoseCovariance6d normalized_covariance_map;
    std::string covariance_error;
    if (!validateAndRegularizePoseCovariance(covariance_map, &normalized_covariance_map,
                                             &covariance_error)) {
        BACKEND_WARN_STREAM("UWB bootstrap covariance rejected ("
                            << covariance_error << "); remaining in current recovery state");
        return false;
    }
    const PoseCovariance6d covariance =
        worldAdditiveToLocalPoseCovariance(T_map_sensor, normalized_covariance_map);
    poses_.setTransform(candidate_T_map_odom);

    // 启动协方差与首个全位姿先验已概括建立规范所用的全部 UWB 配对。
    uwb_.finishBootstrap(*estimate);
    anchorRecoveredFrameLocked(
        frame, T_map_sensor, covariance,
        std::vector<FrontendFrame>(frames_.history().begin(), frames_.history().end()),
        /*add_uwb_factor=*/false);
    const LocalizationTransition transition = state_.onTrustedCorrection(
        "UWB/odometry alignment became yaw-observable", TrustedCorrectionSource::Relocalization);
    last_lidar_observation_runtime_index_ = -1;
    last_global_observation_cycle_time_ = Timestamp();
    commitCurrentTrustedPoseLocked();
    BACKEND_INFO_STREAM("UWB bootstrap accepted: pairs="
                        << estimate->pairs << ", inliers=" << estimate->inliers
                        << ", baseline=" << estimate->odom_baseline << " m, rmse=" << estimate->rmse
                        << " m, yaw_std=" << estimate->yaw_std_rad * 180.0 / M_PI
                        << " deg, delta_t=" << applied_correction.translation
                        << "m, delta_r_deg=" << applied_correction.rotation_rad * 180.0 / M_PI
                        << ", state=" << LocalizationStateMachine::statusName(transition.current));
    return true;
}

}  // namespace backend
