/**
 * @file localization_output.cpp
 * @brief 发布不可变输出视图并提供当前定位与传感器诊断。
 */

#include <cmath>
#include <utility>

#include "backend/localization.hpp"

namespace backend {
/// 在输出提交边界投影同步帧并刷新可信缓存。
LocalizationProjection Localization::project(const FrontendFrame& frame, const Timestamp& stamp) {
    return withOutputCommit(
        [&] { return poses_.project(frame, stamp, state_.snapshot(), active_epoch_); });
}
/// 从不可变输出视图投影高频前端位姿及协方差。
LocalizationProjection Localization::projectForOutput(const Eigen::Matrix4d& pose,
                                                      const PoseCovariance6d& covariance,
                                                      bool valid, double timestamp,
                                                      std::uint64_t epoch) const {
    const auto view = std::atomic_load_explicit(&output_view_, std::memory_order_acquire);
    return view->poses.projectForOutput(pose, covariance, valid, timestamp, epoch, view->state,
                                        view->epoch);
}

/// 在锁内安装状态、位姿和诊断原因一致的不可变输出视图。
void Localization::publishOutputViewLocked() {
    const auto state = state_.snapshot();
    const auto previous = std::atomic_load_explicit(&output_view_, std::memory_order_relaxed);
    // generation 只覆盖状态转移；同一状态下的诊断变化也不能被旧输出覆盖。
    if (previous && previous->epoch == active_epoch_ &&
        previous->state.generation == state.generation &&
        previous->poses.revision() == poses_.revision() &&
        (previous->state.reason != state.reason ||
         previous->state.estimator_failures.count != state.estimator_failures.count ||
         previous->state.estimator_failures.last_reason != state.estimator_failures.last_reason))
        poses_.advanceRevision();
    auto view = std::make_shared<const OutputView>(
        OutputView{poses_, state, active_epoch_, odometry_connected_});
    std::atomic_store_explicit(&output_view_, std::move(view), std::memory_order_release);
}

/// 核验输出快照的前端代次、状态代次及位姿版本。
bool Localization::projectionIsCurrent(const LocalizationProjection& projection) const {
    const auto view = std::atomic_load_explicit(&output_view_, std::memory_order_acquire);
    return projection.valid && projection.epoch == view->epoch &&
           projection.state_generation == view->state.generation &&
           projection.pose_revision == view->poses.revision();
}

/// 返回同一提交版本下的定位状态、前端代次和位姿版本。
LocalizationSnapshot Localization::snapshot() const {
    const auto view = std::atomic_load_explicit(&output_view_, std::memory_order_acquire);
    return {view->state, view->epoch, view->poses.revision()};
}

/// 从不可变输出视图读取当前定位状态。
LocalizationStatus Localization::status() const {
    return std::atomic_load_explicit(&output_view_, std::memory_order_acquire)->state.status;
}

/// 从不可变输出视图读取当前前端代次。
std::uint64_t Localization::activeEpoch() const {
    return std::atomic_load_explicit(&output_view_, std::memory_order_acquire)->epoch;
}

/// 捕获回环边供轨迹可视化发布。
VisualizationSnapshot Localization::visualization() const {
    std::lock_guard<std::mutex> lock(mutex_);
    VisualizationSnapshot snapshot;
    snapshot.loop_edges = loop_edges_;
    return snapshot;
}

/// 捕获回环边并发布本次配准源与目标的诊断快照。
void Localization::publishRegistrationDebug(const Cloud::Ptr& aligned_source,
                                            const Cloud::Ptr& target) {
    if (!config_.registration_debug_enabled || !aligned_source || !target) {
        return;
    }
    VisualizationSnapshot snapshot;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        snapshot.include_debug_clouds = true;
        snapshot.loop_edges = loop_edges_;
        snapshot.debug_aligned_source = aligned_source;
        snapshot.debug_target = target;
    }
    visualization_sink_.publishVisualization(snapshot);
}

/// 经外部端口发布当前已提交的定位状态快照。
void Localization::publishState() {
    state_sink_.publishState(snapshot());
}

/// 返回最新 UWB 物理解经当前时间偏置修正后的时刻。
std::optional<double> Localization::latestUwbMotionTimestamp() const {
    std::lock_guard<std::mutex> lock(mutex_);
    const std::optional<UwbObservation>& latest_uwb = uwb_.observations().latest();
    if (!latest_uwb.has_value()) {
        return std::nullopt;
    }
    const double timestamp = uwb_.motionTimestamp(*latest_uwb);
    return std::isfinite(timestamp) ? std::optional<double>(timestamp) : std::nullopt;
}

/// 利用对齐的前端姿态消除杆臂并返回机体原点诊断位置。
std::optional<Eigen::Vector3d> Localization::latestUwbBodyPositionMap() const {
    std::lock_guard<std::mutex> lock(mutex_);
    const std::optional<UwbObservation>& latest_uwb = uwb_.observations().latest();
    if (!latest_uwb.has_value() || frames_.odometry().empty()) {
        return std::nullopt;
    }
    const std::optional<Eigen::Matrix4d> timed_odom_pose = frames_.odometry().poseAt(
        uwb_.motionTimestamp(*latest_uwb), config_.uwb.odom_sync_tolerance);
    if (!timed_odom_pose.has_value()) {
        return std::nullopt;
    }

    const Eigen::Matrix4d T_map_sensor = poses_.transform() * *timed_odom_pose;
    const Eigen::Matrix3d R_map_sensor = T_map_sensor.block<3, 3>(0, 0);
    const Eigen::Vector3d body_origin_sensor = config_.uwb.T_sensor_body.block<3, 1>(0, 3);
    const Eigen::Vector3d body_to_antenna_sensor =
        config_.uwb.antenna_position_sensor - body_origin_sensor;
    Eigen::Vector3d position_map_body =
        uwb_.positionMap(*latest_uwb) - R_map_sensor * body_to_antenna_sensor;

    if (!config_.uwb.use_z) {
        // 平面 UWB 不提供高度；调试点沿用同一时刻前后端位姿的 body 高度。
        const Eigen::Matrix4d T_map_body = T_map_sensor * config_.uwb.T_sensor_body;
        position_map_body.z() = T_map_body(2, 3);
    }
    return position_map_body;
}

}  // namespace backend
