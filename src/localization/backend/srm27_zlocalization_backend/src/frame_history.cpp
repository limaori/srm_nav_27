/**
 * @file frame_history.cpp
 * @brief 维护有界观测历史和时间支撑，并构建因果配准源。
 */

#include "backend/frame_history.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <iterator>
#include <type_traits>

#include "backend/geometry.hpp"
#include "backend/sampling.hpp"

namespace backend {
/// 校验前端代次、有限值和当前时间轴的单调性。
bool FrameHistory::accepts(const FrontendFrame& frame, std::uint64_t epoch) const {
    if (frame.epoch != epoch || !std::isfinite(frame.timestamp) || !frame.odom_pose.allFinite() ||
        !frame.cloud_local || frame.cloud_local->empty() ||
        (restart_after_ && frame.timestamp <= *restart_after_)) {
        return false;
    }
    // Reset 完成会清空最新帧并递增 epoch，因此时间重启只允许跨越显式事务。
    return !latest_frame_.has_value() || frame.epoch != latest_frame_->epoch ||
           frame.timestamp >= latest_frame_->timestamp;
}
/// 保存最新同步帧并按配置裁剪点云历史及里程计时间支撑。
void FrameHistory::append(const FrontendFrame& frame) {
    latest_frame_ = frame;

    const double history_duration = std::max(
        2.0, timing_.max_age + timing_.odom_sync_tolerance + std::abs(timing_.time_offset_seconds) +
                 (timing_.estimate_time_offset_online ? 4.0 * timing_.time_offset_prior_std_seconds
                                                      : 0.0) +
                 1.0);
    odom_history_.append(frame.timestamp, frame.odom_pose, history_duration);

    if (!frame_history_.empty() && (frame.epoch != frame_history_.back().epoch ||
                                    frame.timestamp + 1.0e-3 < frame_history_.back().timestamp)) {
        frame_history_.clear();
    }
    frame_history_.push_back(frame);
    const std::size_t max_frames =
        static_cast<std::size_t>(std::max(tracking_.max_frames, recovery_.max_frames));
    while (frame_history_.size() > std::max<std::size_t>(1, max_frames)) {
        frame_history_.pop_front();
    }
}
/// 冻结给定关键帧的参考帧及有界因果源历史。
void FrameHistory::capture(const int runtime_index, const FrontendFrame& reference,
                           std::vector<FrontendFrame> causal_history) {
    const std::size_t frame_limit = static_cast<std::size_t>(std::max(1, tracking_.max_frames));
    if (causal_history.size() > frame_limit) {
        causal_history.erase(causal_history.begin(),
                             causal_history.end() - static_cast<std::ptrdiff_t>(frame_limit));
    }
    TrackingSourceSnapshot snapshot;
    snapshot.runtime_index = runtime_index;
    snapshot.reference = reference;
    snapshot.causal_history = std::move(causal_history);
    tracking_source_snapshot_ = std::move(snapshot);
}
/// 清空同步帧、里程计历史、跟踪源快照和输入时间边界。
void FrameHistory::clear() {
    invalidate();
    restart_after_.reset();
}
/// 清空观测支撑，并要求后续输入严格晚于失效边界。
void FrameHistory::invalidate() {
    if (latest_frame_)
        restart_after_ = latest_frame_->timestamp;
    latest_frame_.reset();
    odom_history_.clear();
    frame_history_.clear();
    tracking_source_snapshot_.reset();
}
/// 以最新同步帧重建恢复历史并作废旧跟踪源。
void FrameHistory::beginRecovery() {
    frame_history_.clear();
    if (latest_frame_.has_value() && latest_frame_->cloud_local &&
        !latest_frame_->cloud_local->empty())
        frame_history_.push_back(*latest_frame_);
    tracking_source_snapshot_.reset();
}

/// 追加里程计位姿并按保留时长裁剪；明显回退时重新建立时间轴。
void TimedPoseBuffer::append(const double timestamp, const Eigen::Matrix4d& pose,
                             const double duration_seconds) {
    if (!samples_.empty() && timestamp + 1.0e-3 < samples_.back().first) {
        samples_.clear();
    }
    samples_.emplace_back(timestamp, pose);
    while (samples_.size() > 1U && timestamp - samples_.front().first > duration_seconds) {
        samples_.pop_front();
    }
}

/// 在局部时间支撑内返回插值或最近的有效里程计位姿。
std::optional<Eigen::Matrix4d> TimedPoseBuffer::poseAt(const double timestamp,
                                                       const double tolerance_seconds) const {
    if (samples_.empty() || !std::isfinite(timestamp) || !std::isfinite(tolerance_seconds) ||
        tolerance_seconds < 0.0) {
        return std::nullopt;
    }
    const auto right = std::lower_bound(
        samples_.begin(), samples_.end(), timestamp,
        [](const Sample& sample, const double query) { return sample.first < query; });
    /// 在同步容差内验收最近的有限里程计样本。
    const auto accept_nearest = [&](const auto& sample) -> std::optional<Eigen::Matrix4d> {
        if (sample == samples_.end() || !std::isfinite(sample->first) ||
            !sample->second.allFinite() ||
            std::abs(sample->first - timestamp) > tolerance_seconds) {
            return std::nullopt;
        }
        return sample->second;
    };

    if (right == samples_.begin()) {
        return accept_nearest(right);
    }
    if (right == samples_.end()) {
        return accept_nearest(std::prev(right));
    }
    const auto left = std::prev(right);
    const double left_gap = timestamp - left->first;
    const double right_gap = right->first - timestamp;
    const double span = right->first - left->first;
    if (!std::isfinite(left_gap) || !std::isfinite(right_gap) || !std::isfinite(span) ||
        span <= 1.0e-12 || !left->second.allFinite() || !right->second.allFinite()) {
        return left_gap <= right_gap ? accept_nearest(left) : accept_nearest(right);
    }
    // 仅在查询点两侧均有局部时间支撑时执行插值。
    if (left_gap > tolerance_seconds || right_gap > tolerance_seconds) {
        return left_gap <= right_gap ? accept_nearest(left) : accept_nearest(right);
    }

    const double alpha = std::clamp(left_gap / span, 0.0, 1.0);
    Eigen::Quaterniond left_rotation(left->second.block<3, 3>(0, 0));
    Eigen::Quaterniond right_rotation(right->second.block<3, 3>(0, 0));
    if (!left_rotation.coeffs().allFinite() || !right_rotation.coeffs().allFinite() ||
        left_rotation.norm() <= 1.0e-12 || right_rotation.norm() <= 1.0e-12) {
        return std::nullopt;
    }
    left_rotation.normalize();
    right_rotation.normalize();
    if (left_rotation.dot(right_rotation) < 0.0) {
        right_rotation.coeffs() *= -1.0;
    }

    Eigen::Matrix4d interpolated = Eigen::Matrix4d::Identity();
    interpolated.block<3, 3>(0, 0) = left_rotation.slerp(alpha, right_rotation).toRotationMatrix();
    interpolated.block<3, 1>(0, 3) =
        (1.0 - alpha) * left->second.block<3, 1>(0, 3) + alpha * right->second.block<3, 1>(0, 3);
    return interpolated.allFinite() ? std::optional<Eigen::Matrix4d>(interpolated) : std::nullopt;
}

namespace {
/// 取得前端帧或关键帧持有的只读点云租约。
Cloud::ConstPtr cloudOf(const FrontendFrame& frame) {
    return frame.cloud_local;
}
/// 取得前端帧或关键帧持有的只读点云租约。
Cloud::ConstPtr cloudOf(const Keyframe& frame) {
    return frame.cloud.load();
}

/// 按代次、时间和运动门限累计源点云并记录实际时间跨度。
/// 同步帧按时间、关键帧按编号选择历史前缀。
template <class Frame>
SourceAccumulationResult accumulate(const Frame& reference, const std::vector<Frame>& history,
                                    const SourceAccumulationConfig& config) {
    Cloud::Ptr source(new Cloud);
    int used = 0;
    double span = 0.0;
    const auto reference_cloud = cloudOf(reference);
    if (reference_cloud && !reference_cloud->empty()) {
        *source += *reference_cloud;
        used = 1;
        const Eigen::Matrix4d inverse = reference.odom_pose.inverse();
        for (auto it = history.rbegin();
             it != history.rend() && used < std::max(1, config.max_frames); ++it) {
            const auto& candidate = *it;
            if constexpr (std::is_same_v<Frame, Keyframe>) {
                if (candidate.index >= reference.index)
                    continue;
            }
            const double age = reference.timestamp - candidate.timestamp;
            if (!std::isfinite(age) || age < -1e-3)
                continue;
            if constexpr (std::is_same_v<Frame, FrontendFrame>) {
                if (std::abs(age) <= 1e-3)
                    continue;
            }
            if (candidate.epoch != reference.epoch ||
                (config.max_time > 0 && age > config.max_time))
                break;
            const auto cloud = cloudOf(candidate);
            if (!cloud || cloud->empty())
                continue;
            const Eigen::Matrix4d relative = inverse * candidate.odom_pose;
            if ((config.max_translation > 0 &&
                 relative.block<3, 1>(0, 3).norm() > config.max_translation) ||
                (config.max_rotation_rad > 0 &&
                 rotationAngleFromMatrix(relative) > config.max_rotation_rad))
                break;
            *source += *transformCloud(*cloud, relative);
            ++used;
            span = std::max(span, age);
        }
    }
    return {std::move(source), used, span};
}
}  // namespace
/// 按各自帧身份和运动连续性累计参考坐标系下的因果点云。
SourceAccumulationResult accumulateSource(const FrontendFrame& reference,
                                          const std::vector<FrontendFrame>& history,
                                          const SourceAccumulationConfig& config) {
    return accumulate(reference, history, config);
}
/// 按各自帧身份和运动连续性累计参考坐标系下的因果点云。
SourceAccumulationResult accumulateSource(const Keyframe& reference,
                                          const std::vector<Keyframe>& history,
                                          const SourceAccumulationConfig& config) {
    return accumulate(reference, history, config);
}
}  // namespace backend
