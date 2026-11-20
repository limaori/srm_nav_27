/**
 * @file reset_coordinator.cpp
 * @brief 维护幂等重置事务和前后端代次衔接。
 */

#include "backend/reset_coordinator.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <random>
#include <utility>

#include "backend/covariance.hpp"
#include "backend/localization.hpp"
#include "backend/logger.hpp"
#include "backend/math.hpp"

namespace backend {
namespace {
/// 检查重置边界的非负时刻和有限刚体变换。
bool validResetBoundary(const FrontendResetBoundary& boundary) {
    const auto& transform = boundary.old_from_new;
    const Eigen::Matrix3d rotation = transform.block<3, 3>(0, 0);
    return boundary.stamp.nanoseconds() >= 0 && transform.allFinite() &&
           transform.row(3).isApprox(Eigen::RowVector4d(0, 0, 0, 1), 1e-9) &&
           (rotation.transpose() * rotation).isApprox(Eigen::Matrix3d::Identity(), 1e-9) &&
           std::abs(rotation.determinant() - 1.0) <= 1e-9;
}
}  // namespace

/// 生成本进程的非零重置请求者身份。
ResetCoordinator::ResetCoordinator() : requester_instance_id_(makeRequesterInstanceId()) {
}

/// 清除活动事务与前端响应水位并保留本进程事务序列。
void ResetCoordinator::clearRuntimeState() {
    clearActive();
    last_request_at_ = Timestamp();
    frontend_instance_id_ = 0;
    frontend_generation_ = 0;
}

/// 检查当前是否无在途事务且满足最小请求间隔。
bool ResetCoordinator::canBegin(const Timestamp& now, const double minimum_gap_seconds) const {
    const bool cooling_down =
        !last_request_at_.isZero() && (now - last_request_at_) < minimum_gap_seconds;
    return !pending() && !cooling_down;
}

/// 占用新的非零业务事务编号及下一前端代次。
std::optional<FrontendResetTransaction> ResetCoordinator::begin(const Timestamp& now,
                                                                const double minimum_gap_seconds,
                                                                const std::uint64_t active_epoch) {
    if (!canBegin(now, minimum_gap_seconds) ||
        active_epoch == std::numeric_limits<std::uint64_t>::max()) {
        return std::nullopt;
    }

    std::uint64_t transaction_id = ++next_transaction_id_;
    if (transaction_id == 0) {
        transaction_id = ++next_transaction_id_;
    }
    active_ = FrontendResetTransaction{requester_instance_id_, transaction_id, active_epoch + 1};
    timeout_reported_ = false;
    last_attempt_at_ = now;
    return active_;
}

/// 记录事务已经发送，并推进请求间隔水位。
void ResetCoordinator::markSent(const Timestamp& now) {
    if (pending()) {
        last_request_at_ = now;
        last_sent_transaction_id_ = active_->transaction_id;
    }
}

/// 释放尚未实际发出的事务身份占用。
void ResetCoordinator::cancelUnsent() {
    clearActive();
}

/// 在响应超时后返回同一幂等事务并推进重试时间。
ResetRetry ResetCoordinator::retryIfExpired(const Timestamp& now,
                                            const double response_timeout_seconds) {
    ResetRetry retry;
    if (!active_.has_value() || (now - last_attempt_at_) < response_timeout_seconds) {
        return retry;
    }

    retry.report_timeout = !timeout_reported_;
    retry.transaction = active_;
    timeout_reported_ = true;
    last_attempt_at_ = now;
    return retry;
}

/// 校验回复身份和前端代次水位，并返回确定的事务动作。
ResetReplyAction ResetCoordinator::acceptReply(const FrontendResetReply& reply,
                                               const Timestamp& now) {
    if (!active_.has_value() ||
        reply.transaction.requester_instance_id != active_->requester_instance_id ||
        reply.transaction.transaction_id != active_->transaction_id ||
        reply.transaction.target_epoch != active_->target_epoch) {
        return ResetReplyAction::IgnoreMismatch;
    }
    if (reply.frontend_instance_id == 0) {
        return ResetReplyAction::IgnoreMissingInstance;
    }
    if (reply.frontend_instance_id == frontend_instance_id_ &&
        reply.reset_generation < frontend_generation_) {
        return ResetReplyAction::IgnoreRegressedGeneration;
    }

    if (reply.outcome == FrontendResetOutcome::InProgress) {
        if (reply.frontend_instance_id != frontend_instance_id_) {
            frontend_instance_id_ = reply.frontend_instance_id;
            frontend_generation_ = reply.reset_generation;
        } else {
            frontend_generation_ = std::max(frontend_generation_, reply.reset_generation);
        }
        last_attempt_at_ = now;
        return ResetReplyAction::InProgress;
    }

    if ((reply.outcome == FrontendResetOutcome::Completed ||
         reply.outcome == FrontendResetOutcome::Indeterminate) &&
        reply.frontend_instance_id == frontend_instance_id_ &&
        reply.reset_generation <= frontend_generation_) {
        return ResetReplyAction::IgnoreReplayedTerminal;
    }

    frontend_instance_id_ = reply.frontend_instance_id;
    frontend_generation_ = reply.reset_generation;
    const FrontendResetOutcome outcome = reply.outcome;
    clearActive();
    switch (outcome) {
        case FrontendResetOutcome::Rejected:
            return ResetReplyAction::Rejected;
        case FrontendResetOutcome::Completed:
            return ResetReplyAction::Completed;
        case FrontendResetOutcome::Indeterminate:
            return ResetReplyAction::Indeterminate;
        case FrontendResetOutcome::InProgress:
            break;
    }
    return ResetReplyAction::Indeterminate;
}

/// 结合系统熵和单调时钟生成非零请求者身份。
std::uint64_t ResetCoordinator::makeRequesterInstanceId() noexcept {
    std::uint64_t entropy = 0;
    try {
        std::random_device random;
        entropy =
            (static_cast<std::uint64_t>(random()) << 32U) ^ static_cast<std::uint64_t>(random());
    } catch (...) {
        entropy = 0;
    }
    std::uint64_t value =
        entropy ^
        static_cast<std::uint64_t>(std::chrono::steady_clock::now().time_since_epoch().count());
    value ^= value >> 30U;
    value *= 0xbf58476d1ce4e5b9ULL;
    value ^= value >> 27U;
    value *= 0x94d049bb133111ebULL;
    value ^= value >> 31U;
    return value != 0 ? value : 1;
}

/// 释放当前事务及其超时状态。
void ResetCoordinator::clearActive() {
    active_.reset();
    timeout_reported_ = false;
    last_attempt_at_ = Timestamp();
}

/// 发起将定位冻结到 LOST 的前端协调重置。
bool Localization::requestCoordinatedReset(const bool assume_stationary) {
    return triggerFrontendReset(/*force_lost=*/true, std::nullopt, assume_stationary);
}

/// 保存可信输出并隔离失效前端，撤销旧输入及在途停车衔接资格。
void Localization::isolateFrontendLocked(FrontendStatus status, const char* reason) {
    commitCurrentTrustedPoseLocked();
    frontend_status_ = status;
    odometry_connected_ = false;
    stationary_reset_ = false;
    state_before_manual_reset_.reset();
    state_.forceLost(reason);
    frames_.invalidate();
}

/// 输入中断立即隔离旧观测，服务不可用或重复通知均不恢复旧输出可信度。
void Localization::notifyFrontendDiscontinuity(const std::uint64_t expected_epoch) {
    const bool changed = withOutputCommit([&] {
        if (expected_epoch != active_epoch_ || frontend_status_ != FrontendStatus::Active)
            return false;
        isolateFrontendLocked(FrontendStatus::Interrupted, "front-end input discontinuity");
        return true;
    });
    if (changed)
        publishState();
    triggerFrontendReset(/*force_lost=*/false, expected_epoch);
}

/// 普通失败只建立不可自动恢复的会话屏障，不丢弃已提交轨迹或请求重置。
void Localization::notifyFrontendFailure(const std::uint64_t expected_epoch) {
    const bool changed = withOutputCommit([&] {
        if (expected_epoch != active_epoch_ || frontend_status_ == FrontendStatus::Failed)
            return false;
        isolateFrontendLocked(FrontendStatus::Failed,
                              "front-end processing failure; explicit reset required");
        return true;
    });
    if (changed)
        publishState();
}

/// 完成服务预检并在事务锁内发送幂等前端重置请求。
bool Localization::triggerFrontendReset(const bool force_lost,
                                        const std::optional<std::uint64_t> expected_epoch,
                                        const bool assume_stationary) {
    const Timestamp now = clock_.nowSteadyTimestamp();
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (expected_epoch && *expected_epoch != active_epoch_)
            return false;
        if (frontend_status_ == FrontendStatus::Failed && !force_lost)
            return false;
        if (reset_coordinator_.pending()) {
            return force_lost;
        }
        if (!reset_coordinator_.canBegin(now, config_.reset.min_gap_sec)) {
            return false;
        }
    }
    // 先完成服务预检，避免手动 reset 在无法发送事务时把健康状态留在 LOST。
    if (!reset_port_.frontendResetReady()) {
        return false;
    }

    bool state_changed = false;
    if (!withOutputCommit([&] {
            if (expected_epoch && *expected_epoch != active_epoch_)
                return false;
            if (frontend_status_ == FrontendStatus::Failed && !force_lost)
                return false;
            const std::optional<FrontendResetTransaction> transaction =
                reset_coordinator_.begin(now, config_.reset.min_gap_sec, active_epoch_);
            if (!transaction.has_value()) {
                return false;
            }
            // 发送发生在 controller 锁内，异步响应不能越过随后的 LOST 状态提交。
            if (!reset_port_.sendFrontendReset(*transaction)) {
                reset_coordinator_.cancelUnsent();
                return false;
            }
            // 发送成功即作废旧恢复快照，即使保持 LOST，或终结响应随后为 REJECTED。
            reset_coordinator_.markSent(now);
            stationary_reset_ = assume_stationary;
            if (force_lost && state_.status() != LocalizationStatus::Lost) {
                commitCurrentTrustedPoseLocked();
                state_before_manual_reset_ = state_;
                state_changed = state_.forceLost("manual coordinated front-end reset").changed;
            } else {
                state_before_manual_reset_.reset();
            }
            return true;
        }))
        return false;
    if (state_changed) {
        publishState();
    }
    return true;
}

/// 在超时后以同一业务身份重发仍在途的重置事务。
void Localization::expireFrontendResetIfTimedOut() {
    const Timestamp now = clock_.nowSteadyTimestamp();
    // 与终结响应共用发送边界，不能在取出重试后解锁再发送。
    std::lock_guard<std::mutex> lock(mutex_);
    const ResetRetry retry =
        reset_coordinator_.retryIfExpired(now, config_.reset.response_timeout_sec);
    if (retry.report_timeout) {
        BACKEND_WARN_STREAM(
            "front-end reset response exceeded timeout; transaction remains "
            "pending and will be retried with the same idempotency key");
    }
    if (retry.transaction.has_value() && !reset_port_.sendFrontendReset(*retry.transaction)) {
        BACKEND_WARN_STREAM(
            "front-end reset retry could not be sent; the transaction remains "
            "pending and will retry when the service is available");
    }
}

/// 恢复手动重置前状态，并开放尚未提交观测的重新调度。
void Localization::rollbackRejectedResetLocked() {
    stationary_reset_ = false;
    if (state_before_manual_reset_.has_value()) {
        state_.restoreAfterResetRejected(*state_before_manual_reset_);
    }
    state_before_manual_reset_.reset();
    // 允许新观测立即调度；已融合的 LiDAR 证据不因 reset 被拒绝而重新消费。
    last_global_observation_cycle_time_ = Timestamp();
    last_recovery_cycle_time_ = Timestamp();
    last_recovery_timestamp_ = -std::numeric_limits<double>::infinity();
}

/// 从旧关键帧或尚未消费的连续边继承里程计参考，并组合新坐标边界。
Localization::EpochAnchor Localization::resetOdometryAnchorLocked(
    const FrontendResetBoundary& boundary, const std::uint64_t target_epoch) const {
    if (trajectory_.empty())
        return {};
    const Keyframe previous = trajectory_.latest();
    if (previous.epoch == active_epoch_ && graph_.hasEstimate(previous.index)) {
        PendingResetOdometry anchor;
        anchor.from_index = previous.index;
        anchor.target_epoch = target_epoch;
        anchor.reference_timestamp = previous.timestamp;
        anchor.reference_odom = (boundary.old_from_new.inverse() * previous.odom_pose).eval();
        return anchor;
    }

    // 尚无新关键帧时，继续组合此前已确认的坐标边界。
    const auto& pending = next_epoch_seed_ ? next_epoch_seed_->anchor : pending_epoch_anchor_;
    const auto* carried = std::get_if<PendingResetOdometry>(&pending);
    if (!carried || carried->target_epoch != active_epoch_ ||
        !graph_.hasEstimate(carried->from_index))
        return {};
    auto continued = *carried;
    continued.target_epoch = target_epoch;
    continued.reference_odom = (boundary.old_from_new.inverse() * continued.reference_odom).eval();
    return continued;
}

/// 根据停车位姿构造旧图连续边，缺少前驱时使用可信协方差先验。
Localization::EpochAnchor Localization::resetStationaryAnchorLocked(
    const Eigen::Matrix4d& pose, const std::uint64_t target_epoch) const {
    if (!trajectory_.empty() && graph_.hasEstimate(trajectory_.latest().index)) {
        const Keyframe previous = trajectory_.latest();
        PendingResetContinuity continuity;
        continuity.from_index = previous.index;
        continuity.target_epoch = target_epoch;
        continuity.measured_between = previous.optimized_pose.inverse() * pose;
        const auto& trusted = poses_.trusted();
        const double duration = trusted.epoch() == previous.epoch
                                    ? trusted.timestamp().seconds() - previous.timestamp
                                    : 0.0;
        continuity.covariance = config_.graph.odom_noise.covariance(
            duration, continuity.measured_between.block<3, 1>(0, 3).norm(),
            rotationAngleFromMatrix(continuity.measured_between));
        continuity.covariance.diagonal().head<3>().array() +=
            config_.reset.stationary_translation_std * config_.reset.stationary_translation_std;
        continuity.covariance.diagonal().tail<3>().array() +=
            config_.reset.stationary_rotation_std_rad * config_.reset.stationary_rotation_std_rad;
        return continuity;
    }
    if (poses_.hasTrustedCovariance())
        return poses_.trustedCovariance();
    return {};
}

/// 在停车首帧确定后保存真实参考，后续暖重置只改变其里程计坐标。
void Localization::bindResetAnchorLocked(const FrontendFrame& frame) {
    std::optional<PoseCovariance6d> reference_covariance_map;
    if (const auto* continuity = std::get_if<PendingResetContinuity>(&pending_epoch_anchor_)) {
        const auto previous_pose = graph_.estimate(continuity->from_index);
        const auto previous_covariance_local = graph_.marginalCovariance(continuity->from_index);
        if (previous_pose && previous_covariance_local) {
            reference_covariance_map = propagatePoseWithRelativeCovariance(
                *previous_pose,
                localToWorldAdditivePoseCovariance(*previous_pose, *previous_covariance_local),
                poses_.transform() * frame.odom_pose, continuity->covariance);
        } else if (poses_.hasTrustedCovariance()) {
            reference_covariance_map = poses_.trustedCovariance();
        }
        PendingResetOdometry bound;
        bound.from_index = continuity->from_index;
        bound.target_epoch = continuity->target_epoch;
        bound.reference_timestamp = frame.timestamp;
        bound.reference_odom = frame.odom_pose;
        bound.from_reference = continuity->measured_between;
        bound.prefix_covariance = continuity->covariance;
        pending_epoch_anchor_ = std::move(bound);
    } else if (const auto* prior = std::get_if<PoseCovariance6d>(&pending_epoch_anchor_)) {
        reference_covariance_map = *prior;
    }
    if (reference_covariance_map)
        poses_.anchorCovariance(frame, *reference_covariance_map);
}

/// 按有效坐标边界或明确停车保证接续，其他情况下仅保留恢复初值。
void Localization::prepareNextEpochSeedLocked(const FrontendResetReply& reply) {
    const auto target_epoch = reply.transaction.target_epoch;
    const auto* previous_origin =
        next_epoch_seed_ ? std::get_if<EpochTransform>(&next_epoch_seed_->origin) : nullptr;
    if (reply.outcome == FrontendResetOutcome::Completed && reply.boundary) {
        const auto& boundary = *reply.boundary;
        const auto& transform = boundary.old_from_new;
        const bool valid =
            boundary.source_epoch == active_epoch_ && validResetBoundary(boundary) &&
            (!frames_.hasLatest() || frames_.latest().timestamp <= boundary.stamp.seconds()) &&
            (!next_epoch_seed_ || previous_origin) &&
            (!previous_origin || boundary.stamp >= previous_origin->boundary_stamp);
        if (valid) {
            EpochAnchor anchor = odometry_connected_
                                     ? resetOdometryAnchorLocked(boundary, target_epoch)
                                     : EpochAnchor{};
            poses_.reframeOdometry(transform, active_epoch_, target_epoch);
            next_epoch_seed_ =
                EpochSeed{EpochTransform{poses_.transform(), boundary.stamp},
                          odometry_connected_ && poses_.trusted().hasPose(), std::move(anchor)};
            return;
        }
        BACKEND_WARN_STREAM(
            "reset coordinate boundary is inconsistent; stationary reset contract is required");
    }

    next_epoch_seed_.reset();
    const bool fresh_map = !config_.prior_map_enabled && !config_.uwb.enabled &&
                           !poses_.trusted().hasPose() && trajectory_.empty();
    odometry_connected_ = fresh_map || (reply.outcome == FrontendResetOutcome::Completed &&
                                        stationary_reset_ && poses_.trusted().hasPose());
    if (poses_.trusted().hasPose()) {
        const Eigen::Matrix4d pose = poses_.trusted().mapSensorPose();
        EpochSeed seed{pose, odometry_connected_, {}};
        if (odometry_connected_)
            seed.anchor = resetStationaryAnchorLocked(pose, target_epoch);
        next_epoch_seed_ = std::move(seed);
    } else if (frames_.hasLatest()) {
        next_epoch_seed_ =
            EpochSeed{(poses_.transform() * frames_.latest().odom_pose).eval(), false, {}};
    }
}

/// 推进前端代次并清理依赖旧里程计规范的运行窗口。
void Localization::commitFrontendResetBoundaryLocked(const FrontendResetReply& reply) {
    const auto target_epoch = reply.transaction.target_epoch;
    pending_epoch_anchor_ = std::monostate{};
    active_epoch_ = target_epoch;
    poses_.resetEpoch(target_epoch);
    const bool completed = reply.outcome == FrontendResetOutcome::Completed;
    if (config_.prior_map_enabled || (frontend_status_ == FrontendStatus::Failed && !completed)) {
        state_.onFrontendResetCompleted();
    } else if (odometry_connected_) {
        state_.initialize("waiting for first front-end frame after reset");
    } else if (config_.uwb.enabled) {
        state_.initialize("waiting for UWB/odometry alignment after input discontinuity");
    } else {
        state_.forceLost("front-end continuity unavailable; explicit global anchor required");
    }

    frames_.clear();
    uwb_.resetEpoch();
    last_lidar_observation_runtime_index_ = -1;
    last_global_observation_cycle_time_ = Timestamp();
    last_recovery_cycle_time_ = Timestamp();
    last_recovery_timestamp_ = -std::numeric_limits<double>::infinity();
    target_cache_.clear();
    last_tracking_reference_index_ = -1;
    state_before_manual_reset_.reset();
    stationary_reset_ = false;
    if (completed || frontend_status_ == FrontendStatus::Interrupted)
        frontend_status_ = FrontendStatus::Active;
    estimator_failed_ = false;
    // reset 只切换前端里程计规范，不消费 LOST 中已接受的手动重定位请求。
}

/// 校验重置响应并原子提交拒绝回退或新代次边界。
void Localization::onFrontendResetResponse(
    FrontendResetReply reply, const std::function<bool(FrontendResetReply&)>& translate) {
    bool boundary = false;
    bool publish = false;
    withOutputCommit([&] {
        if (!reset_coordinator_.pending() || (translate && !translate(reply)))
            return;
        const ResetReplyAction action =
            reset_coordinator_.acceptReply(reply, clock_.nowSteadyTimestamp());
        switch (action) {
            case ResetReplyAction::IgnoreMismatch:
                return;
            case ResetReplyAction::IgnoreMissingInstance:
                BACKEND_WARN_STREAM("ignoring front-end reset response with no instance id");
                return;
            case ResetReplyAction::IgnoreRegressedGeneration:
                BACKEND_WARN_STREAM("ignoring front-end reset response with regressed generation");
                return;
            case ResetReplyAction::IgnoreReplayedTerminal:
                BACKEND_WARN_STREAM(
                    "ignoring terminal front-end reset response without a new "
                    "reset generation");
                return;
            case ResetReplyAction::InProgress:
                return;
            case ResetReplyAction::Rejected:
                rollbackRejectedResetLocked();
                publish = true;
                break;
            case ResetReplyAction::Indeterminate:
                BACKEND_WARN_STREAM(
                    "front-end reset outcome is indeterminate; advancing the "
                    "backend epoch to avoid a false odometry edge");
                [[fallthrough]];
            case ResetReplyAction::Completed:
                prepareNextEpochSeedLocked(reply);
                commitFrontendResetBoundaryLocked(reply);
                boundary = true;
                publish = true;
                break;
        }
    });
    if (publish) {
        publishState();
    }
    if (boundary) {
        reset_port_.onFrontendResetBoundary();
    }
}
}  // namespace backend
