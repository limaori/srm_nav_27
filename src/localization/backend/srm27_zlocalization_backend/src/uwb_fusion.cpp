/**
 * @file uwb_fusion.cpp
 * @brief 管理 UWB 物理解、消费记录和空间标定。
 */

#include "backend/uwb_fusion.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "backend/logger.hpp"
#include "backend/math.hpp"
#include "backend/pose_graph.hpp"

namespace backend {
namespace {

// 部署链路以厘米量化；微米阈值只吸收浮点转换噪声，不充当运动死区。
constexpr double kHeldPositionToleranceMeters = 1.0e-6;
constexpr double kIntervalToleranceSeconds = 1.0e-6;

}  // namespace

/// 验收新的 UWB 物理解，保留原始时间戳用于唯一性判断。
bool UwbObservationStream::ingest(const double timestamp,
                                  const Eigen::Vector3d& position_uwb_antenna) {
    const bool position_finite = config_.use_z ? position_uwb_antenna.allFinite()
                                               : position_uwb_antenna.head<2>().allFinite();
    if (!config_.enabled || !std::isfinite(timestamp) || !position_finite) {
        return false;
    }

    Eigen::Vector3d observation = position_uwb_antenna;
    if (!config_.use_z) {
        observation.z() = 0.0;
    }
    if (latest_.has_value() && timestamp + 1.0e-3 < latest_->timestamp) {
        return false;
    }
    if (latest_.has_value() && config_.effective_rate_hz > 0.0) {
        const double minimum_interval = 1.0 / config_.effective_rate_hz;
        const double elapsed = timestamp - latest_->timestamp;
        const Eigen::Vector3d delta = observation - latest_->position_uwb;
        const double change = config_.use_z ? delta.norm() : delta.head<2>().norm();
        if (change <= kHeldPositionToleranceMeters &&
            elapsed + kIntervalToleranceSeconds < minimum_interval) {
            // 不刷新时间戳；新鲜度和图权重必须跟随物理解频率，而非重发频率。
            return false;
        }
    }

    latest_ = UwbObservation{timestamp, observation};
    return true;
}

/// 绑定 UWB 输入流、图估计及里程计时间支撑并初始化生命周期。
UwbFusion::UwbFusion(UwbFusionConfig config, PoseGraph& graph, const TimedPoseBuffer& odometry)
    : config_(std::move(config)),
      graph_(graph),
      odom_history_(odometry),
      uwb_observations_({config_.uwb.enabled, config_.uwb.use_z, config_.uwb.effective_rate_hz}),
      uwb_factor_builder_({config_.uwb.odom_sync_tolerance, config_.uwb.time_jitter_std_seconds,
                           config_.uwb.antenna_position_sensor, config_.uwb.position_covariance,
                           config_.odometry_noise}),
      uwb_bootstrap_(config_.uwb.bootstrap),
      uwb_zero_pose_estimator_(config_.uwb.bootstrap) {
    resetAll();
}
/// 完整重置 UWB 物理解、因子消费水位及所有标定和启动窗口。
void UwbFusion::resetAll() {
    uwb_observations_.clear();
    last_uwb_factor_timestamp_ = -std::numeric_limits<double>::infinity();
    resetGraph();
}
/// 清除图标定与配对窗口，保留原始观测身份和因子消费水位。
void UwbFusion::resetGraph() {
    clearBootstrapWindow();
    clearCalibrationWindow();
    map_from_uwb_ = config_.uwb.T_map_uwb_fixed;
    uwb_zero_pose_ready_ = !alignmentEstimationEnabled();
    uwb_zero_pose_estimator_.reset(config_.uwb.bootstrap, map_from_uwb_);
    current_uwb_time_offset_seconds_ = config_.uwb.time_offset_seconds;
    last_reported_uwb_time_offset_seconds_ = current_uwb_time_offset_seconds_;
}
/// 清理依赖旧里程计的配对窗口，保留最新物理解及因子消费水位。
void UwbFusion::resetEpoch() {
    // 前端仅更换里程计坐标系；UWB 最近物理解及消费记录都独立于前端代次。
    clearBootstrapWindow();
    clearCalibrationWindow();
    if (alignmentEstimationEnabled() && !uwb_zero_pose_ready_) {
        map_from_uwb_ = config_.uwb.T_map_uwb_fixed;
        uwb_zero_pose_estimator_.reset(config_.uwb.bootstrap, map_from_uwb_);
    }
}
/// 清空运动配对，并以给定的 odom 到 map 变换重新启动对齐。
void UwbFusion::restartBootstrap(const Eigen::Matrix4d& seed) {
    clearBootstrapWindow();
    uwb_bootstrap_.setInitialGuess(seed);
}
/// 验收新的 UWB 物理解，保留原始时间戳用于唯一性判断。
bool UwbFusion::ingest(double stamp, const Eigen::Vector3d& position) {
    return std::isfinite(stamp + timeOffset()) && uwb_observations_.ingest(stamp, position);
}
/// 重置仅依赖本代里程计的启动配对。
void UwbFusion::clearBootstrapWindow() {
    uwb_bootstrap_.reset(config_.uwb.bootstrap);
    last_uwb_bootstrap_timestamp_ = -std::numeric_limits<double>::infinity();
}

/// 释放在线标定的图引用及对应原始解水位。
void UwbFusion::clearCalibrationWindow() {
    uwb_alignment_calibration_samples_.clear();
    last_uwb_calibration_timestamp_ = -std::numeric_limits<double>::infinity();
}

/// 保证因子消费只向有限的新时刻推进。
void UwbFusion::consumeThrough(double timestamp) {
    if (std::isfinite(timestamp))
        last_uwb_factor_timestamp_ = std::max(last_uwb_factor_timestamp_, timestamp);
}

/// 有效统计结论对接受和拒绝的物理解执行相同消费。
void UwbFusion::finishAssessment(const UwbFactorInput& input,
                                 const InnovationAssessment& assessment) {
    if (assessment.valid)
        consumeThrough(input.timestamp);
}

/// 稳定启动估计概括的观测不再作为独立因子使用。
void UwbFusion::finishBootstrap(const UwbAlignmentEstimate& estimate) {
    if (estimate.stable)
        consumeThrough(estimate.latest_timestamp);
}

/// 仅为新鲜且坐标标定就绪的物理解构造普通图因子。
std::optional<UwbFactorInput> UwbFusion::captureFresh(const Keyframe& keyframe) const {
    const auto& latest = uwb_observations_.latest();
    if (!ready() || !latest || latest->timestamp <= last_uwb_factor_timestamp_)
        return std::nullopt;
    return buildLatestFactor(keyframe);
}

/// 由最新物理解及里程计时间支撑构造原始因子输入。
std::optional<UwbFactorInput> UwbFusion::buildLatestFactor(const Keyframe& keyframe) const {
    const auto& latest_uwb = uwb_observations_.latest();
    if (!config_.uwb.enabled || !latest_uwb || odom_history_.empty())
        return std::nullopt;
    UwbFactorBuildMode mode;
    mode.time_offset_seconds = timeOffset();
    mode.estimate_time_offset = timeOffsetEstimationEnabled();
    mode.time_offset_state_available = graph_.hasUwbTimeOffset();
    mode.with_alignment = alignmentEstimationEnabled();
    const UwbFactorBuildContext context{*latest_uwb, keyframe, odom_history_, map_from_uwb_, mode};
    return uwb_factor_builder_.build(context);
}

/// 在当前图预测处线性化给定 UWB 因子以供创新检验。
InnovationTerm UwbFusion::linearize(const int runtime_index, const UwbFactorInput& input) const {
    return graph_.linearizeUwbPosition(runtime_index, input, config_.uwb.use_z);
}

/// 对单个 UWB 因子执行图预测下的联合创新检验。
InnovationAssessment UwbFusion::assess(const int runtime_index, const UwbFactorInput& input) const {
    return graph_.assessInnovationBatch({linearize(runtime_index, input)}).joint;
}

/// 将已验收的 UWB 原始位置因子插入当前图事务。
void UwbFusion::insert(const int runtime_index, const UwbFactorInput& input) {
    graph_.addUwbPosition(runtime_index, input, config_.uwb.use_z);
}

/// 对恢复锚附近的新 UWB 观测完成创新检验、消费和可选因子插入。
bool UwbFusion::addToRecovery(const Keyframe& keyframe) {
    const std::optional<UwbFactorInput> input = captureFresh(keyframe);
    if (!input.has_value()) {
        return false;
    }
    const InnovationAssessment assessment = assess(keyframe.index, *input);
    if (!assessment.valid) {
        BACKEND_WARN_STREAM("recovery UWB innovation unavailable: runtime="
                            << keyframe.index << ", reason=" << assessment.invalid_reason);
        return false;
    }
    // 每个物理解无论统计结论如何都只消费一次；此处按验收结果追加 UWB 因子，
    // 恢复状态由调用方依据已验证的完整位姿锚提交。
    finishAssessment(*input, assessment);
    if (assessment.upper_tail_probability < config_.rejection_probability) {
        BACKEND_WARN_STREAM("recovery UWB factor rejected without changing state: runtime="
                            << keyframe.index << ", NIS=" << assessment.nis << "/"
                            << assessment.degrees_of_freedom
                            << ", p=" << assessment.upper_tail_probability);
        return false;
    }

    insert(keyframe.index, *input);
    return true;
}

/// 返回当前有效配置是否启用共享坐标标定。
bool UwbFusion::alignmentEstimationEnabled() const {
    return config_.uwb.estimate_zero_pose_online;
}

/// 返回当前有效配置是否启用共享静态时间偏置估计。
bool UwbFusion::timeOffsetEstimationEnabled() const {
    return config_.uwb.estimate_time_offset_online;
}

/// 返回当前有效时间偏置后验，尚无后验时使用配置值。
double UwbFusion::timeOffset() const {
    return timeOffsetEstimationEnabled() && std::isfinite(current_uwb_time_offset_seconds_)
               ? current_uwb_time_offset_seconds_
               : config_.uwb.time_offset_seconds;
}

/// 将原始解时间修正为其代表的物理运动时刻。
double UwbFusion::motionTimestamp(const UwbObservation& observation) const {
    return observation.timestamp + timeOffset();
}

/// 按需为图建立共享静态时间偏置及其先验。
void UwbFusion::ensureTimeOffsetState() {
    if (!timeOffsetEstimationEnabled() || graph_.hasUwbTimeOffset()) {
        return;
    }
    graph_.addUwbTimeOffsetPrior(config_.uwb.time_offset_seconds,
                                 config_.uwb.time_offset_prior_std_seconds);
}

/// 读取时间偏置后验，并按需计算和报告其不确定度。
void UwbFusion::refreshTimeOffset(const bool refresh_covariance) {
    if (!timeOffsetEstimationEnabled() || !graph_.hasUwbTimeOffset()) {
        return;
    }
    bool report_update = false;
    const std::optional<double> estimate = graph_.uwbTimeOffsetEstimate();
    if (estimate.has_value() && std::isfinite(*estimate)) {
        current_uwb_time_offset_seconds_ = *estimate;
        report_update =
            refresh_covariance && std::abs(current_uwb_time_offset_seconds_ -
                                           last_reported_uwb_time_offset_seconds_) >= 1.0e-3;
    }
    if (!refresh_covariance) {
        return;
    }
    double posterior_standard_deviation = std::numeric_limits<double>::quiet_NaN();
    const std::optional<double> variance = graph_.uwbTimeOffsetMarginalVariance();
    if (variance.has_value() && std::isfinite(*variance) && *variance > 0.0) {
        posterior_standard_deviation = std::sqrt(*variance);
        BACKEND_DEBUG_STREAM("UWB time-offset posterior: dt="
                             << current_uwb_time_offset_seconds_
                             << "s, std=" << posterior_standard_deviation << "s");
    }
    if (report_update) {
        const double update_delta =
            current_uwb_time_offset_seconds_ - last_reported_uwb_time_offset_seconds_;
        BACKEND_INFO_STREAM("UWB time-offset posterior updated: dt="
                            << current_uwb_time_offset_seconds_ << "s, std="
                            << posterior_standard_deviation << "s, delta=" << update_delta << "s");
        last_reported_uwb_time_offset_seconds_ = current_uwb_time_offset_seconds_;
    }
}

/// 返回当前 UWB 坐标对齐是否允许普通因子进入图。
bool UwbFusion::ready() const {
    return !alignmentEstimationEnabled() || uwb_zero_pose_ready_;
}

/// 读取联合图中共享 UWB 坐标标定的当前后验。
void UwbFusion::refreshAlignment() {
    if (!alignmentEstimationEnabled() || !uwb_zero_pose_ready_ || !graph_.hasUwbAlignment()) {
        return;
    }
    const std::optional<Eigen::Matrix4d> estimate = graph_.uwbAlignmentEstimate();
    if (estimate.has_value() && estimate->allFinite()) {
        // 更新共享静态标定变量的后验均值。
        map_from_uwb_ = *estimate;
    }
}

/// 剔除已离图的关键帧配对，使标定可观性仅由活动变量支撑。
void UwbFusion::retainActiveCalibrationSamples() {
    // 先裁剪，再拟合与验收；已经离图的运动不能替活动因子提供 yaw 可观性。
    auto& samples = uwb_alignment_calibration_samples_;
    samples.erase(std::remove_if(samples.begin(), samples.end(),
                                 [&](const auto& sample) {
                                     return !graph_.hasEstimate(sample.runtime_index);
                                 }),
                  samples.end());
    std::vector<double> active_timestamps;
    for (const auto& sample : samples)
        active_timestamps.push_back(sample.factor.timestamp);
    uwb_zero_pose_estimator_.retainPairs(active_timestamps);
}

/// 将稳定标定的原始内点因子共同引入图，并消费该启动窗口覆盖的物理解。
void UwbFusion::initializeAlignment(const UwbAlignmentEstimate& estimate) {
    std::vector<const UwbAlignmentCalibrationSample*> inlier_samples;
    inlier_samples.reserve(estimate.inlier_timestamps.size());
    for (const UwbAlignmentCalibrationSample& candidate : uwb_alignment_calibration_samples_) {
        // 两侧均为同一输入时间戳的原值拷贝；精确相等关联身份，不做邻近时间匹配。
        const bool selected =
            std::find(estimate.inlier_timestamps.begin(), estimate.inlier_timestamps.end(),
                      candidate.factor.timestamp) != estimate.inlier_timestamps.end();
        if (selected) {
            inlier_samples.push_back(&candidate);
        }
    }
    const Eigen::Matrix4d delta = map_from_uwb_.inverse() * estimate.T_map_odom;
    graph_.addUwbAlignmentInitialValue(estimate.T_map_odom, config_.uwb.use_z);
    for (const UwbAlignmentCalibrationSample* inlier : inlier_samples) {
        insert(inlier->runtime_index, inlier->factor);
    }
    map_from_uwb_ = estimate.T_map_odom;
    uwb_zero_pose_ready_ = true;
    // 鲁棒启动器已分类截至该时间的全部解；内点已成为原始联合因子，外点被消费但
    // 不增加图信息，只有后续解进入普通 NIS 路径。
    finishBootstrap(estimate);
    uwb_alignment_calibration_samples_.clear();
    BACKEND_INFO_STREAM(
        "online UWB zero-pose joint calibration initialized: pairs="
        << estimate.pairs << ", inliers=" << estimate.inliers
        << ", baseline=" << estimate.map_baseline << "m, rmse=" << estimate.rmse
        << "m, yaw_std_deg=" << estimate.yaw_std_rad * 180.0 / M_PI << ", T_map_uwb={x="
        << map_from_uwb_(0, 3) << ", y=" << map_from_uwb_(1, 3) << ", z=" << map_from_uwb_(2, 3)
        << ", yaw_deg=" << yawFromMatrix(map_from_uwb_) * 180.0 / M_PI << "}, delta_t="
        << (config_.uwb.use_z ? delta.block<3, 1>(0, 3).norm() : delta.block<2, 1>(0, 3).norm())
        << "m, delta_r_deg=" << rotationAngleFromMatrix(delta) * 180.0 / M_PI);
}

/// 用独立地图位姿积累在线标定配对，并提交稳定内点的联合原始因子。
bool UwbFusion::observeAnchor(const Keyframe& reference, const Eigen::Matrix4d& T_map_reference) {
    const std::optional<UwbObservation>& latest_uwb = uwb_observations_.latest();
    if (!alignmentEstimationEnabled() || uwb_zero_pose_ready_ || !latest_uwb.has_value() ||
        odom_history_.empty() || !std::isfinite(reference.timestamp) ||
        !reference.odom_pose.allFinite() || !T_map_reference.allFinite() ||
        !graph_.hasEstimate(reference.index) ||
        latest_uwb->timestamp <= last_uwb_calibration_timestamp_) {
        return false;
    }

    const double motion_timestamp = motionTimestamp(*latest_uwb);
    const double age = reference.timestamp - motion_timestamp;
    if (age < -config_.uwb.odom_sync_tolerance || age > config_.uwb.max_age) {
        return false;
    }
    const std::optional<UwbFactorInput> factor = buildLatestFactor(reference);
    if (!factor.has_value() || !factor->with_alignment) {
        return false;
    }

    retainActiveCalibrationSamples();

    const Eigen::Vector3d position_map_antenna =
        T_map_reference.block<3, 3>(0, 0) * factor->antenna_in_keyframe +
        T_map_reference.block<3, 1>(0, 3);
    UwbAlignmentCalibrationSample sample;
    sample.runtime_index = reference.index;
    sample.factor = *factor;
    uwb_alignment_calibration_samples_.push_back(std::move(sample));
    const std::size_t max_samples =
        static_cast<std::size_t>(std::max(1, config_.uwb.bootstrap.max_pairs));
    if (uwb_alignment_calibration_samples_.size() > max_samples) {
        uwb_alignment_calibration_samples_.erase(
            uwb_alignment_calibration_samples_.begin(),
            uwb_alignment_calibration_samples_.begin() +
                static_cast<std::ptrdiff_t>(uwb_alignment_calibration_samples_.size() -
                                            max_samples));
    }
    last_uwb_calibration_timestamp_ = factor->timestamp;
    const std::optional<UwbAlignmentEstimate> estimate =
        uwb_zero_pose_estimator_.addPair(factor->timestamp, factor->position, position_map_antenna);
    if (!estimate.has_value() || !estimate->stable) {
        return false;
    }

    initializeAlignment(*estimate);
    return true;
}

/// 配对未消费的 UWB 与里程计运动，并返回稳定可观的启动估计。
std::optional<UwbAlignmentEstimate> UwbFusion::bootstrap(const FrontendFrame& frame) {
    const std::optional<UwbObservation>& latest_uwb = uwb_observations_.latest();
    if (config_.prior_map_enabled || !config_.uwb.enabled || !latest_uwb.has_value() ||
        odom_history_.empty() || latest_uwb->timestamp <= last_uwb_factor_timestamp_ ||
        latest_uwb->timestamp <= last_uwb_bootstrap_timestamp_) {
        return std::nullopt;
    }

    const double motion_timestamp = motionTimestamp(*latest_uwb);
    const double age = frame.timestamp - motion_timestamp;
    if (age < -config_.uwb.odom_sync_tolerance || age > config_.uwb.max_age) {
        return std::nullopt;
    }
    const std::optional<Eigen::Matrix4d> timed_odom_pose =
        odom_history_.poseAt(motion_timestamp, config_.uwb.odom_sync_tolerance);
    if (!timed_odom_pose.has_value()) {
        return std::nullopt;
    }

    const Eigen::Vector3d antenna_odom =
        timed_odom_pose->block<3, 3>(0, 0) * config_.uwb.antenna_position_sensor +
        timed_odom_pose->block<3, 1>(0, 3);
    last_uwb_bootstrap_timestamp_ = latest_uwb->timestamp;
    const std::optional<UwbAlignmentEstimate> estimate =
        uwb_bootstrap_.addPair(latest_uwb->timestamp, antenna_odom,
                               uwb_factor_builder_.positionMap(*latest_uwb, map_from_uwb_));
    if (!estimate.has_value() || !estimate->stable) {
        return std::nullopt;
    }

    return estimate;
}
}  // namespace backend
