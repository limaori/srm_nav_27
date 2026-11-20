/**
 * @file pose_output.cpp
 * @brief 维护可信位姿、协方差锚与输出投影。
 */

#include "backend/pose_output.hpp"

#include "backend/covariance.hpp"
#include "backend/logger.hpp"
#include "backend/math.hpp"
#include "backend/pose_graph.hpp"

namespace backend {

/// 清除可信位姿及其前端代次和时间身份。
void TrustedPoseProjector::clear() {
    map_from_sensor_ = Eigen::Matrix4d::Identity();
    epoch_ = 0;
    stamp_ = Timestamp();
    has_pose_ = false;
}

/// 仅在有效且代次及时间不回退时更新可信位姿。
bool TrustedPoseProjector::updateIfNewer(const Eigen::Matrix4d& T_map_sensor,
                                         const Timestamp& stamp, const std::uint64_t epoch) {
    if (!T_map_sensor.allFinite() || stamp.isZero() ||
        (has_pose_ && (epoch < epoch_ || (epoch == epoch_ && stamp < stamp_)))) {
        return false;
    }
    return commitCorrection(T_map_sensor, stamp, epoch);
}

/// 原子安装已验收的绝对校正，允许新代次首帧的零时间戳。
bool TrustedPoseProjector::commitCorrection(const Eigen::Matrix4d& T_map_sensor,
                                            const Timestamp& stamp, const std::uint64_t epoch) {
    if (!T_map_sensor.allFinite()) {
        return false;
    }
    map_from_sensor_ = T_map_sensor;
    epoch_ = epoch;
    stamp_ = stamp;
    has_pose_ = true;
    return true;
}

/// 在可信状态使用当前全局规范，否则冻结最后可信传感器位姿。
TrustedPoseProjection TrustedPoseProjector::project(const bool pose_trusted,
                                                    const Eigen::Matrix4d& T_map_odom_nominal,
                                                    const Eigen::Matrix4d& T_odom_sensor) const {
    TrustedPoseProjection out;
    if (!pose_trusted && has_pose_) {
        out.T_map_sensor = map_from_sensor_;
        out.T_map_odom = map_from_sensor_ * T_odom_sensor.inverse();
        out.frozen_from_cache = true;
        return out;
    }

    out.T_map_odom = T_map_odom_nominal;
    out.T_map_sensor = T_map_odom_nominal * T_odom_sensor;
    return out;
}

/// 清空可信位姿、协方差锚和 TF 发布水位并重置全局规范。
void PoseOutput::initialize() {
    map_from_odom_ = Eigen::Matrix4d::Identity();
    pose_revision_ = 0;
    graph_anchor_.reset();
    trusted_covariance_.reset();
    trusted_pose_.clear();
    last_tf_epoch_ = 0;
    last_tf_stamp_ = Timestamp();
}
/// 保持地图中的物理锚不变，将其里程计坐标转换到新代次继续传播不确定度。
void PoseOutput::reframeOdometry(const Eigen::Matrix4d& old_from_new,
                                 const std::uint64_t source_epoch,
                                 const std::uint64_t target_epoch) {
    map_from_odom_ = (map_from_odom_ * old_from_new).eval();
    if (graph_anchor_ && graph_anchor_->epoch == source_epoch) {
        graph_anchor_->odom_pose = old_from_new.inverse() * graph_anchor_->odom_pose;
        graph_anchor_->epoch = target_epoch;
    }
}
/// 将当前帧及其地图系协方差安装为新的传播参考，不改变可信输出状态。
void PoseOutput::anchorCovariance(const FrontendFrame& frame, const PoseCovariance6d& covariance) {
    graph_anchor_ = GraphAnchor{covariance, frame.epoch, frame.timestamp,
                                (map_from_odom_ * frame.odom_pose).eval(), frame.odom_pose};
}
/// 使用当前全局规范传播帧协方差，不改变可信缓存或输出版本。
std::optional<PoseCovariance6d> PoseOutput::covarianceAt(const FrontendFrame& frame) const {
    LocalizationProjection projection;
    projection.T_map_odom = map_from_odom_;
    projection.T_map_sensor = map_from_odom_ * frame.odom_pose;
    populateCovariance(frame.odom_pose, frame.pose_covariance, frame.pose_covariance_valid,
                       frame.epoch, frame.timestamp, projection);
    if (!projection.pose_covariance_valid)
        return std::nullopt;
    return projection.pose_covariance;
}
/// 投影同步帧并推进可信缓存及严格单调的 TF 发布水位。
LocalizationProjection PoseOutput::project(const FrontendFrame& frame, const Timestamp& stamp,
                                           const LocalizationStateSnapshot& snap,
                                           std::uint64_t active_epoch) {
    LocalizationProjection out;
    out.epoch = frame.epoch;
    out.state_generation = snap.generation;
    out.pose_revision = pose_revision_;
    out.valid = frame.epoch == active_epoch;
    out.status = snap.status;
    out.reason = snap.reason;
    out.estimator_failures = snap.estimator_failures;
    out.pose_trusted = LocalizationStateMachine::poseTrusted(out.status);
    out.global_pose_available = out.pose_trusted || trusted_pose_.hasPose();
    const TrustedPoseProjection projection =
        trusted_pose_.project(out.pose_trusted, map_from_odom_, frame.odom_pose);
    out.T_map_odom = projection.T_map_odom;
    out.T_map_sensor = projection.T_map_sensor;

    const bool active_frame = frame.epoch == active_epoch;
    if (out.pose_trusted) {
        populateCovariance(frame.odom_pose, frame.pose_covariance, frame.pose_covariance_valid,
                           frame.epoch, frame.timestamp, out);
        if (active_frame && trusted_pose_.updateIfNewer(out.T_map_sensor, stamp, frame.epoch)) {
            trusted_covariance_ = out.pose_covariance_valid
                                      ? std::optional<PoseCovariance6d>(out.pose_covariance)
                                      : std::nullopt;
        }
    } else {
        out.pose_covariance = trusted_covariance_.value_or(PoseCovariance6d::Zero());
        out.pose_covariance_valid = trusted_covariance_.has_value();
    }

    const bool tf_stamp_is_newer =
        active_frame &&
        (frame.epoch > last_tf_epoch_ ||
         (frame.epoch == last_tf_epoch_ && (last_tf_stamp_.isZero() || last_tf_stamp_ < stamp)));
    if (out.global_pose_available && out.status != LocalizationStatus::Initializing &&
        !stamp.isZero() && tf_stamp_is_newer) {
        last_tf_epoch_ = frame.epoch;
        last_tf_stamp_ = stamp;
        out.should_publish_tf = true;
    }
    return out;
}
/// 根据不可变状态投影高频位姿及协方差，不改变缓存。
LocalizationProjection PoseOutput::projectForOutput(
    const Eigen::Matrix4d& T_odom_sensor, const PoseCovariance6d& frontend_covariance,
    const bool frontend_covariance_valid, const double timestamp, const std::uint64_t epoch,
    const LocalizationStateSnapshot& snap, std::uint64_t active_epoch) const {
    LocalizationProjection out;
    out.epoch = epoch;
    out.state_generation = snap.generation;
    out.pose_revision = pose_revision_;
    out.status = snap.status;
    out.reason = snap.reason;
    out.estimator_failures = snap.estimator_failures;
    if (epoch != active_epoch || !T_odom_sensor.allFinite() || !std::isfinite(timestamp)) {
        return out;
    }
    out.valid = true;
    out.pose_trusted = LocalizationStateMachine::poseTrusted(out.status);
    out.global_pose_available = out.pose_trusted || trusted_pose_.hasPose();
    const TrustedPoseProjection projection =
        trusted_pose_.project(out.pose_trusted, map_from_odom_, T_odom_sensor);
    out.T_map_odom = projection.T_map_odom;
    out.T_map_sensor = projection.T_map_sensor;
    if (!out.pose_trusted) {
        out.pose_covariance = trusted_covariance_.value_or(PoseCovariance6d::Zero());
        out.pose_covariance_valid = trusted_covariance_.has_value();
        return out;
    }

    populateCovariance(T_odom_sensor, frontend_covariance, frontend_covariance_valid, epoch,
                       timestamp, out);
    return out;
}
/// 由同代次图锚或前端协方差填充当前输出不确定度。
void PoseOutput::populateCovariance(const Eigen::Matrix4d& T_odom_sensor,
                                    const PoseCovariance6d& frontend_covariance,
                                    const bool frontend_covariance_valid, const std::uint64_t epoch,
                                    const double timestamp,
                                    LocalizationProjection& projection) const {
    if (!graph_anchor_) {
        if (frontend_covariance_valid) {
            projection.pose_covariance =
                rotatePoseCovariance(frontend_covariance, projection.T_map_odom.block<3, 3>(0, 0));
            projection.pose_covariance_valid = true;
        }
        return;
    }

    projection.pose_covariance = graph_anchor_->covariance;
    projection.pose_covariance_valid = true;
    const bool same_epoch = epoch == graph_anchor_->epoch;
    const bool is_anchor = same_epoch && std::abs(timestamp - graph_anchor_->timestamp) <= 1.0e-3;
    if (is_anchor) {
        return;
    }

    if (same_epoch) {
        const Eigen::Matrix4d measured_between = graph_anchor_->odom_pose.inverse() * T_odom_sensor;
        const PoseCovariance6d relative_covariance = noise_.covariance(
            timestamp - graph_anchor_->timestamp, measured_between.block<3, 1>(0, 3).norm(),
            rotationAngleFromMatrix(measured_between));
        projection.pose_covariance =
            propagatePoseWithRelativeCovariance(graph_anchor_->map_pose, graph_anchor_->covariance,
                                                projection.T_map_sensor, relative_covariance);
        return;
    }

    // 仅在新图锚提交前的 epoch 边界出现；保持保守有限输出，但不假设两个 epoch
    // 之间存在一段相对运动。
    if (frontend_covariance_valid) {
        projection.pose_covariance +=
            rotatePoseCovariance(frontend_covariance, projection.T_map_odom.block<3, 3>(0, 0));
    }
}
/// 验收图边缘协方差并安装对应关键帧的传播锚。
void PoseOutput::refreshAnchor(const Keyframe& latest, const PoseGraph& graph) {
    const bool new_epoch = !graph_anchor_ || latest.epoch != graph_anchor_->epoch;
    const auto marginal = graph.marginalCovariance(latest.index);
    if (marginal.has_value()) {
        const PoseCovariance6d covariance_map =
            rotatePoseCovariance(*marginal, latest.optimized_pose.block<3, 3>(0, 0));
        PoseCovariance6d normalized_covariance_map;
        std::string covariance_error;
        if (validateAndRegularizePoseCovariance(covariance_map, &normalized_covariance_map,
                                                &covariance_error)) {
            graph_anchor_ = GraphAnchor{normalized_covariance_map, latest.epoch, latest.timestamp,
                                        latest.optimized_pose, latest.odom_pose};
        } else {
            BACKEND_WARN_STREAM("graph marginal covariance rejected (" << covariance_error << ")");
            if (new_epoch) {
                graph_anchor_.reset();
            }
        }
    } else if (new_epoch) {
        // 协方差锚不得跨越 reset epoch 传播。
        graph_anchor_.reset();
    }
}
/// 提交当前可信同步帧的全局位姿和传播协方差。
void PoseOutput::commitTrusted(const FrontendFrame& latest, std::uint64_t active_epoch,
                               bool trusted) {
    if (!trusted || latest.epoch != active_epoch)
        return;
    LocalizationProjection projection;
    projection.T_map_odom = map_from_odom_;
    projection.T_map_sensor = map_from_odom_ * latest.odom_pose;
    populateCovariance(latest.odom_pose, latest.pose_covariance, latest.pose_covariance_valid,
                       latest.epoch, latest.timestamp, projection);
    trusted_pose_.commitCorrection(projection.T_map_sensor,
                                   Timestamp::fromSeconds(latest.timestamp), active_epoch);
    trusted_covariance_ = projection.pose_covariance_valid
                              ? std::optional<PoseCovariance6d>(projection.pose_covariance)
                              : std::nullopt;
}
}  // namespace backend
