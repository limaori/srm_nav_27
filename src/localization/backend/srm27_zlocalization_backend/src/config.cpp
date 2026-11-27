/**
 * @file config.cpp
 * @brief 校验并归一化定位参数及能力组合。
 */

#include "backend/config.hpp"

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <utility>

#include "backend/covariance.hpp"

namespace backend {
namespace {

/// 记录一个会改变有效能力或策略的显式覆盖。
void recordOverride(EffectiveConfig* effective, const std::string& field,
                    const std::string& requested, const std::string& value,
                    const std::string& reason) {
    if (effective == nullptr) {
        return;
    }
    effective->overrides.push_back({field, requested, value, reason});
}

/// 保证关键帧触发条件可用并协调最大与最小插入间隔。
void resolveKeyframePolicy(EffectiveConfig& effective) {
    Config& config = effective.values;
    // 至少保留一个关键帧触发条件，并保证最大间隔不短于最小插入间隔。
    if (config.keyframes.distance <= 0.0 && config.keyframes.angle_rad <= 0.0 &&
        config.keyframes.max_interval <= 0.0) {
        const double effective_interval =
            std::max(Config{}.keyframes.max_interval, config.keyframes.min_interval);
        config.keyframes.max_interval = effective_interval;
        recordOverride(&effective, "trajectory.keyframe.max_interval", "disabled",
                       std::to_string(effective_interval),
                       "at least one keyframe creation trigger must remain active");
    } else if (config.keyframes.max_interval > 0.0 &&
               config.keyframes.max_interval < config.keyframes.min_interval) {
        const double requested_interval = config.keyframes.max_interval;
        config.keyframes.max_interval = config.keyframes.min_interval;
        recordOverride(&effective, "trajectory.keyframe.max_interval",
                       std::to_string(requested_interval),
                       std::to_string(config.keyframes.max_interval),
                       "maximum keyframe interval cannot be shorter than min_interval");
    }
}

/// 校验启动提示位姿并约束强制匹配所需的先验地图能力。
void resolveStartupHint(EffectiveConfig& effective) {
    Config& config = effective.values;
    if (config.startup_hint.T_map_sensor.has_value() &&
        !config.startup_hint.T_map_sensor->allFinite()) {
        config.startup_hint.T_map_sensor.reset();
        recordOverride(&effective, "startup_hint.enabled", "true", "false",
                       "the configured pose contains a non-finite value");
    }
    if (config.startup_hint.force_until_matched && !config.startup_hint.T_map_sensor.has_value()) {
        config.startup_hint.force_until_matched = false;
        recordOverride(&effective, "startup_hint.force_until_matched", "true", "false",
                       "no startup hint pose is available");
    } else if (config.startup_hint.force_until_matched && !config.prior_map_enabled) {
        config.startup_hint.force_until_matched = false;
        recordOverride(&effective, "startup_hint.force_until_matched", "true", "false",
                       "force requires an immutable prior map for geometric verification");
    }
}

/// 一次性先验必须以先验地图为唯一先验来源；没有先验地图时该开关无意义。
void resolvePriorInitializationOnly(EffectiveConfig& effective) {
    Config& config = effective.values;
    if (config.prior_map_initialization_only && !config.prior_map_enabled) {
        config.prior_map_initialization_only = false;
        recordOverride(&effective, "prior_map.initialization_only", "true", "false",
                       "initialization-only matching requires an enabled prior map");
    }
}

/// 根据运行轨迹描述子的可用性解析回环能力，先验不排斥回环。
void resolveLoopCapability(EffectiveConfig& effective, const bool descriptor_available) {
    Config& config = effective.values;
    if (config.loop.enabled && !descriptor_available) {
        config.loop.enabled = false;
        recordOverride(&effective, "loop.enabled", "true", "false",
                       "loop closure requires an enabled place descriptor");
    }
}

/// 仅在没有全局观测或启动提示时保留纯里程计原点。
void resolveOdometryOrigin(EffectiveConfig& effective) {
    Config& config = effective.values;
    const bool pure_odometry =
        !config.prior_map_enabled && !config.uwb.enabled && !config.loop.enabled;
    const bool odometry_origin_requested = !config.pure_odometry.origin_xy.isZero(1.0e-12);
    if (odometry_origin_requested && !pure_odometry) {
        config.pure_odometry.origin_xy.setZero();
        recordOverride(&effective, "pure_odometry.origin", "nonzero", "(0, 0)",
                       "the odometry origin is only defined without prior map, UWB, or "
                       "loop closure");
    } else if (odometry_origin_requested && config.startup_hint.T_map_sensor.has_value()) {
        config.pure_odometry.origin_xy.setZero();
        recordOverride(&effective, "pure_odometry.origin", "nonzero", "(0, 0)",
                       "startup_hint already defines the epoch-0 gauge");
    }
}

/// 根据 UWB 输入和先验地图能力解析在线空间与时间标定。
void resolveUwbCalibration(EffectiveConfig& effective) {
    Config& config = effective.values;
    if (config.uwb.estimate_zero_pose_online && !config.uwb.enabled) {
        config.uwb.estimate_zero_pose_online = false;
        recordOverride(&effective, "uwb.estimate_zero_pose_online", "true", "false",
                       "UWB input is disabled");
    } else if (config.uwb.estimate_zero_pose_online && !config.prior_map_enabled) {
        config.uwb.estimate_zero_pose_online = false;
        recordOverride(&effective, "uwb.estimate_zero_pose_online", "true", "false",
                       "online UWB-frame alignment requires an immutable prior map");
    }

    if (config.uwb.estimate_time_offset_online && !config.uwb.enabled) {
        config.uwb.estimate_time_offset_online = false;
        recordOverride(&effective, "uwb.estimate_time_offset_online", "true", "false",
                       "UWB input is disabled");
    } else if (config.uwb.estimate_time_offset_online && !config.prior_map_enabled) {
        config.uwb.estimate_time_offset_online = false;
        recordOverride(&effective, "uwb.estimate_time_offset_online", "true", "false",
                       "online time calibration requires independent prior-map poses");
    }
}

/// 为启用回环的运行模式保留完整历史图变量。
void resolveGraphHistory(EffectiveConfig& effective) {
    Config& config = effective.values;
    if (config.loop.enabled && config.graph.fixed_lag_seconds > 0.0) {
        const double requested_lag = config.graph.fixed_lag_seconds;
        config.graph.fixed_lag_seconds = 0.0;
        recordOverride(&effective, "graph.fixed_lag_seconds", std::to_string(requested_lag), "0",
                       "loop closure requires historical pose variables");
    }
}

}  // namespace

/// 按可用地图、描述子和 UWB 能力解析有效配置并记录覆盖原因。
EffectiveConfig resolveConfig(Config requested, const bool descriptor_available) {
    EffectiveConfig effective;
    effective.values = std::move(requested);
    normalizeBackendConfigNumerics(&effective.values);
    resolveKeyframePolicy(effective);
    resolveStartupHint(effective);
    resolvePriorInitializationOnly(effective);
    resolveLoopCapability(effective, descriptor_available);
    resolveOdometryOrigin(effective);
    resolveUwbCalibration(effective);
    resolveGraphHistory(effective);
    return effective;
}

namespace {
/// 保留有限数值，否则使用对应默认值。
double finiteOr(double value, double fallback) {
    return std::isfinite(value) ? value : fallback;
}
/// 将数值限制到非负域并替换非有限输入。
double nonnegativeOr(double value, double fallback) {
    return std::max(0.0, finiteOr(value, fallback));
}
/// 保留有限正值，否则使用对应默认值。
double positiveOr(double value, double fallback) {
    return std::isfinite(value) && value > 0 ? value : fallback;
}

/// 对一组同类型配置字段应用统一数值域规则。
template <class Config, class Rule>
void fields(Config& config, const Config& defaults, std::initializer_list<double Config::*> members,
            Rule rule) {
    for (auto member : members)
        config.*member = rule(config.*member, defaults.*member);
}
/// 规范化因果源最小帧数、容量及连续性门限。
void source(SourceAccumulationConfig& config, const SourceAccumulationConfig& defaults) {
    config.max_frames = std::max(1, config.max_frames);
    config.min_frames = std::clamp(config.min_frames, 1, config.max_frames);
    fields(config, defaults,
           {&SourceAccumulationConfig::max_time, &SourceAccumulationConfig::max_translation,
            &SourceAccumulationConfig::max_rotation_rad},
           nonnegativeOr);
}

/// 规范化关键帧运动触发门限及最小插入间隔。
void normalizeKeyframeNumerics(KeyframePolicy& config, const KeyframePolicy& defaults) {
    fields(config, defaults,
           {&KeyframePolicy::distance, &KeyframePolicy::angle_rad, &KeyframePolicy::max_interval},
           finiteOr);
    config.min_interval = nonnegativeOr(config.min_interval, defaults.min_interval);
}

/// 规范化初始重定位的距离门限、候选数量及并行预算。
void normalizeRelocalizationNumerics(RelocalizationConfig& config,
                                     const RelocalizationConfig& defaults) {
    fields(config, defaults,
           {&RelocalizationConfig::initial_relocalization_max_descriptor_distance,
            &RelocalizationConfig::relocalization_descriptor_distance_limit,
            &RelocalizationConfig::relocalization_submap_radius},
           nonnegativeOr);
    config.relocalization_yaw_sweep_count = std::max(1, config.relocalization_yaw_sweep_count);
    config.relocalization_parallel_workers = std::max(0, config.relocalization_parallel_workers);
    config.relocalization_max_candidates = std::max(1, config.relocalization_max_candidates);
    config.relocalization_submap_keyframes = std::max(1, config.relocalization_submap_keyframes);
}

/// 按既定顺序规范化跟踪、恢复、一致性门限及各观测源容量。
void normalizeObservationNumerics(Config& c, const Config& d) {
    fields(
        c.tracking, d.tracking,
        {&TrackingConfig::correction_period, &TrackingConfig::target_radius,
         &TrackingConfig::nearest_reference_radius, &TrackingConfig::target_hysteresis_distance,
         &TrackingConfig::max_translation_correction, &TrackingConfig::max_rotation_correction_rad,
         &TrackingConfig::max_body_angular_rate_rad,
         &TrackingConfig::suspicious_linear_speed, &TrackingConfig::suspicious_angular_speed},
        nonnegativeOr);
    c.tracking.target_keyframes = std::max<std::size_t>(1, c.tracking.target_keyframes);
    c.loop.target_keyframes = std::max<std::size_t>(1, c.loop.target_keyframes);
    c.innovation.rejection_probability =
        std::clamp(finiteOr(c.innovation.rejection_probability, d.innovation.rejection_probability),
                   1e-12, 0.5);
    c.innovation.conflicts_before_suspect = std::max(1, c.innovation.conflicts_before_suspect);
    c.recovery.suspect_failures_before_lost = std::max(1, c.recovery.suspect_failures_before_lost);
    fields(c.recovery, d.recovery,
           {&RecoveryConfig::period, &RecoveryConfig::suspect_search_radius,
            &RecoveryConfig::lost_search_radius, &RecoveryConfig::max_translation_correction,
            &RecoveryConfig::max_rotation_correction_rad},
           nonnegativeOr);
    source(c.tracking.source, d.tracking.source);
    source(c.recovery.source, d.recovery.source);
    source(c.loop.source, d.loop.source);
    c.recovery.global_fallback_candidates = std::max(0, c.recovery.global_fallback_candidates);

    // 累计修正预算：两个上限 0 表示"该轴不检查"，两个 reseat 阈值 0 表示"永不跟随"，
    // 都是非负语义，因此统一走 nonnegativeOr（非有限值退回默认）。
    fields(c.correction_anchor, d.correction_anchor,
           {&CorrectionAnchorConfig::max_rotation_rad, &CorrectionAnchorConfig::max_translation,
            &CorrectionAnchorConfig::reseat_rotation_rad,
            &CorrectionAnchorConfig::reseat_translation,
            &CorrectionAnchorConfig::release_agree_translation,
            &CorrectionAnchorConfig::release_agree_rotation_rad,
            &CorrectionAnchorConfig::release_max_translation,
            &CorrectionAnchorConfig::release_max_rotation_rad},
           nonnegativeOr);
    c.correction_anchor.reseat_quiet_frames =
        std::max(1, c.correction_anchor.reseat_quiet_frames);
    c.correction_anchor.release_frames = std::max(1, c.correction_anchor.release_frames);
    // 放行半径不得超过对应单次闸：否则"一致证据放行"就等于绕过单次幅度检查，
    // 一次提交能把车搬过单次预算（那是 2026-11-26 传送事故的拦口）。
    if (c.correction_anchor.release_max_translation > 0.0 &&
        c.recovery.max_translation_correction > 0.0) {
        c.correction_anchor.release_max_translation =
            std::min(c.correction_anchor.release_max_translation,
                     c.recovery.max_translation_correction);
    }
    if (c.correction_anchor.release_max_rotation_rad > 0.0 &&
        c.recovery.max_rotation_correction_rad > 0.0) {
        c.correction_anchor.release_max_rotation_rad =
            std::min(c.correction_anchor.release_max_rotation_rad,
                     c.recovery.max_rotation_correction_rad);
    }
}

/// 规范化 UWB 时空参数、位置协方差及依赖其数值的启动标定。
void normalizeUwbNumerics(UwbConfig& uwb, const UwbConfig& ud) {
    uwb.recovery_candidates = std::max(1, uwb.recovery_candidates);
    fields(uwb, ud,
           {&UwbConfig::effective_rate_hz, &UwbConfig::time_jitter_std_seconds, &UwbConfig::max_age,
            &UwbConfig::odom_sync_tolerance, &UwbConfig::recovery_search_radius},
           nonnegativeOr);
    uwb.time_offset_seconds = finiteOr(uwb.time_offset_seconds, ud.time_offset_seconds);
    uwb.time_offset_prior_std_seconds =
        positiveOr(uwb.time_offset_prior_std_seconds, ud.time_offset_prior_std_seconds);
    if (!uwb.T_sensor_body.allFinite())
        uwb.T_sensor_body = ud.T_sensor_body;
    if (!uwb.T_map_uwb_fixed.allFinite())
        uwb.T_map_uwb_fixed = ud.T_map_uwb_fixed;
    if (!uwb.antenna_position_sensor.allFinite())
        uwb.antenna_position_sensor = ud.antenna_position_sensor;
    Eigen::MatrixXd covariance;
    uwb.position_covariance = validateAndRegularizeCovariance(uwb.position_covariance, &covariance)
                                  ? Eigen::Matrix3d(covariance)
                                  : ud.position_covariance;
    auto& bootstrap = uwb.bootstrap;
    bootstrap.use_z = uwb.use_z;
    bootstrap.max_pairs = std::max(2, bootstrap.max_pairs);
    bootstrap.min_pairs = std::clamp(bootstrap.min_pairs, 2, bootstrap.max_pairs);
    fields(
        bootstrap, ud.bootstrap,
        {&UwbAlignmentConfig::min_baseline, &UwbAlignmentConfig::inlier_threshold,
         &UwbAlignmentConfig::max_rmse, &UwbAlignmentConfig::max_yaw_std_rad,
         &UwbAlignmentConfig::stable_translation_delta, &UwbAlignmentConfig::stable_yaw_delta_rad,
         &UwbAlignmentConfig::gravity_alignment_std_rad},
        positiveOr);
    bootstrap.position_std_xy = std::sqrt(std::max(uwb.position_covariance(0, 0), 1e-12));
    bootstrap.position_std_z = std::sqrt(std::max(uwb.position_covariance(2, 2), 1e-12));
    bootstrap.stable_solutions = std::max(1, bootstrap.stable_solutions);
}

/// 规范化回环检索、验证门限及一致性历史预算。
void normalizeLoopNumerics(LoopConfig& loop, const LoopConfig& defaults) {
    loop.cooldown_keyframes = std::max(0, loop.cooldown_keyframes);
    fields(loop, defaults, {&LoopConfig::candidate_max_distance, &LoopConfig::target_radius},
           nonnegativeOr);
    loop.descriptor_ratio_threshold = std::clamp(
        finiteOr(loop.descriptor_ratio_threshold, defaults.descriptor_ratio_threshold), 0.0, 1.0);
    loop.max_parallel_verifications = std::max(1, loop.max_parallel_verifications);
    loop.history_size = std::max(0, loop.history_size);
    fields(loop.verification, defaults.verification,
           {&LoopClosureConfig::translation_max, &LoopClosureConfig::z_max,
            &LoopClosureConfig::roll_pitch_max_rad, &LoopClosureConfig::yaw_residual_max_rad,
            &LoopClosureConfig::overlap_distance, &LoopClosureConfig::pcm_translation_threshold,
            &LoopClosureConfig::pcm_rotation_threshold_rad},
           nonnegativeOr);
    loop.verification.min_overlap_ratio = std::clamp(
        finiteOr(loop.verification.min_overlap_ratio, defaults.verification.min_overlap_ratio), 0.0,
        1.0);
    loop.verification.min_consistent_loops = std::max(1, loop.verification.min_consistent_loops);
    loop.verification.min_candidate_clique_size =
        std::max(1, loop.verification.min_candidate_clique_size);
}

/// 规范化前端重置间隔、响应超时及静止运动噪声。
void normalizeResetNumerics(ResetConfig& config, const ResetConfig& defaults) {
    config.min_gap_sec = nonnegativeOr(config.min_gap_sec, defaults.min_gap_sec);
    config.response_timeout_sec =
        std::max(0.05, finiteOr(config.response_timeout_sec, defaults.response_timeout_sec));
    fields(config, defaults,
           {&ResetConfig::stationary_translation_std, &ResetConfig::stationary_rotation_std_rad},
           positiveOr);
}

/// 规范化图先验、里程计过程噪声、历史窗口及稳健核参数。
void normalizeGraphNumerics(PoseGraphConfig& graph, const PoseGraphConfig& defaults) {
    for (int axis = 0; axis < 3; ++axis) {
        graph.prior_rotation_variances(axis) = positiveOr(graph.prior_rotation_variances(axis),
                                                          defaults.prior_rotation_variances(axis));
        graph.prior_translation_variances(axis) = positiveOr(
            graph.prior_translation_variances(axis), defaults.prior_translation_variances(axis));
    }
    fields(graph.odom_noise, defaults.odom_noise,
           {&OdometryProcessNoiseConfig::rotation_variance_per_second,
            &OdometryProcessNoiseConfig::translation_variance_per_second},
           positiveOr);
    fields(graph.odom_noise, defaults.odom_noise,
           {&OdometryProcessNoiseConfig::rotation_variance_per_radian,
            &OdometryProcessNoiseConfig::translation_variance_per_meter},
           nonnegativeOr);
    graph.fixed_lag_seconds = std::isfinite(graph.fixed_lag_seconds) && graph.fixed_lag_seconds >= 0
                                  ? graph.fixed_lag_seconds
                                  : 0.0;
    graph.robust_kernel_size = positiveOr(graph.robust_kernel_size, defaults.robust_kernel_size);
}

/// 将纯里程计原点的非有限分量恢复为默认值。
void normalizeOdometryOriginNumerics(PureOdometryConfig& config,
                                     const PureOdometryConfig& defaults) {
    for (int axis = 0; axis < 2; ++axis)
        config.origin_xy(axis) = finiteOr(config.origin_xy(axis), defaults.origin_xy(axis));
}

}  // namespace

/// 按各参数的数值域修正非有限值和越界配置。
void normalizeBackendConfigNumerics(Config* config) {
    if (!config)
        return;
    auto& c = *config;
    const Config d;
    normalizeKeyframeNumerics(c.keyframes, d.keyframes);
    normalizeRelocalizationNumerics(c.relocalization, d.relocalization);
    normalizeObservationNumerics(c, d);
    normalizeUwbNumerics(c.uwb, d.uwb);
    normalizeLoopNumerics(c.loop, d.loop);
    normalizeResetNumerics(c.reset, d.reset);
    normalizeGraphNumerics(c.graph, d.graph);
    normalizeOdometryOriginNumerics(c.pure_odometry, d.pure_odometry);
}
}  // namespace backend
