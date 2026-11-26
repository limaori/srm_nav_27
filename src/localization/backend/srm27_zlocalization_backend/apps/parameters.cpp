/**
 * @file parameters.cpp
 * @brief 加载输入、匹配、恢复和输出参数并验证配置集合。
 */

#include "apps/parameters.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

#include "backend/math.hpp"

namespace backend::ros {

/// 读取全部后端参数并在返回前校验能力关系及未知字段。
NodeConfig ParameterReader::load() {
    loadCommonInput();
    loadMapTrajectoryAndRegistration();
    loadTracking();
    loadRecovery();
    loadUwb();
    loadLoopAndGraph();
    loadResetAndStartup();
    loadOutputs();
    validateRelationships();
    loadDescriptor();
    validateParameterSchema();
    return std::move(parameters_);
}

/// 读取坐标系、话题、同步器和 body 外参。
void ParameterReader::loadCommonInput() {
    getParam("common/frontend", parameters_.input.frontend);
    getParam("common/frame_topic", parameters_.input.frame_topic);
    getParam("common/prediction_topic", parameters_.input.prediction_topic);
    getParam("common/session_topic", parameters_.input.session_topic);
    getParam("common/sensor_frame", parameters_.input.sensor_frame);
    if (parameters_.input.frontend != "legacy" && parameters_.input.frontend != "zlio")
        throw std::invalid_argument("common.frontend must be legacy or zlio");
    getParam("common/map_frame", parameters_.input.map_frame);
    getParam("common/odom_frame", parameters_.input.odom_frame);
    getParam("common/odom_topic", parameters_.input.odom_topic);
    getParam("common/sync_odom_topic", parameters_.input.sync_odom_topic);
    getParam("common/cloud_topic", parameters_.input.cloud_topic);
    getParam("common/input_cloud_in_local_frame", parameters_.input.cloud_in_local_frame);
    if (parameters_.input.frontend == "zlio" && !parameters_.input.cloud_in_local_frame)
        throw std::invalid_argument(
            "ZLIO observations require common.input_cloud_in_local_frame=true");
    getParam("common/odom_cloud_sync/subscription_queue_depth",
             parameters_.input.subscription_queue);
    getParam("common/odom_cloud_sync/synchronizer_queue_depth",
             parameters_.input.synchronizer_queue);
    getParam("common/odom_cloud_sync/max_interval", parameters_.input.synchronizer_max_interval);
    getParam("common/maintenance_period", parameters_.input.maintenance_period);
    parameters_.input.subscription_queue = std::max(1, parameters_.input.subscription_queue);
    parameters_.input.synchronizer_queue = std::max(1, parameters_.input.synchronizer_queue);
    parameters_.input.synchronizer_max_interval =
        std::max(1.0e-3, parameters_.input.synchronizer_max_interval);
    parameters_.input.maintenance_period = std::max(0.01, parameters_.input.maintenance_period);

    getParam("common/body/frame", parameters_.input.body_frame);
    Eigen::Vector3d body_offset = Eigen::Vector3d::Zero();
    Eigen::Vector3d body_rpy = Eigen::Vector3d::Zero();
    getParam<double>("common/body/offset/x", body_offset.x(), 0.0);
    getParam<double>("common/body/offset/y", body_offset.y(), 0.0);
    getParam<double>("common/body/offset/z", body_offset.z(), 0.0);
    getParam<double>("common/body/rotation/roll_deg", body_rpy.x(), 0.0);
    getParam<double>("common/body/rotation/pitch_deg", body_rpy.y(), 0.0);
    getParam<double>("common/body/rotation/yaw_deg", body_rpy.z(), 0.0);
    if (!body_offset.allFinite() || !body_rpy.allFinite()) {
        throw std::invalid_argument("common.body calibration contains a non-finite value");
    }
    parameters_.input.body_transform.configure(body_offset, body_rpy);
}

/// 读取地图持久化、关键帧和唯一点到面配准参数。
void ParameterReader::loadMapTrajectoryAndRegistration() {
    int cloud_cache_mb = 64, submap_cache_mb = 64;
    getParam<int>("storage/cloud_cache_mb", cloud_cache_mb, 64);
    getParam<int>("storage/submap_cache_mb", submap_cache_mb, 64);
    parameters_.runtime.storage.cloud_cache_bytes =
        static_cast<std::size_t>(std::max(0, cloud_cache_mb)) * 1024U * 1024U;
    parameters_.runtime.storage.submap_cache_bytes =
        static_cast<std::size_t>(std::max(0, submap_cache_mb)) * 1024U * 1024U;
    getParam("prior_map/enabled", parameters_.runtime.prior_map_enabled);
    getParam("prior_map/initialization_only",
             parameters_.runtime.prior_map_initialization_only);
    getParam<std::string>("prior_map/directory", parameters_.maps.prior_directory, "pcd/prior_map");
    getParam("prior_map/voxel_resolution", parameters_.maps.prior_voxel);

    getParam("trajectory/save_on_shutdown", parameters_.maps.save_on_shutdown);
    getParam<std::string>("trajectory/save_directory", parameters_.maps.save_directory,
                          "pcd/runtime_map");
    getParam("trajectory/save_global_pcd", parameters_.maps.save_global_pcd);
    getParam("trajectory/map_voxel_resolution", parameters_.maps.runtime_voxel);
    getParam("trajectory/keyframe/distance", parameters_.runtime.keyframes.distance);
    double keyframe_angle_deg = 12.0;
    getParam<double>("trajectory/keyframe/angle_deg", keyframe_angle_deg, 12.0);
    parameters_.runtime.keyframes.angle_rad = degreesToRadians(keyframe_angle_deg);
    getParam("trajectory/keyframe/min_interval", parameters_.runtime.keyframes.min_interval);
    getParam("trajectory/keyframe/max_interval", parameters_.runtime.keyframes.max_interval);
    RegistrationConfig& registration = parameters_.runtime.relocalization.point_to_plane_config;
    getParam("registration/voxel_resolution", registration.voxel_resolution);
    getParam<int>("registration/max_iterations", registration.max_iterations, 10);
    getParam("registration/max_correspondence_distance", registration.max_correspondence_distance);
    getParam<int>("registration/plane_knn", registration.plane_knn, 10);
    getParam("registration/max_plane_eigenvalue_ratio", registration.max_plane_eigenvalue_ratio);
    getParam("registration/min_plane_middle_eigenvalue_ratio",
             registration.min_plane_middle_eigenvalue_ratio);
    getParam("registration/max_plane_neighbor_distance", registration.max_plane_neighbor_distance);
    getParam("registration/min_correspondences", registration.min_correspondences);
    getParam("registration/min_correspondence_ratio", registration.min_correspondence_ratio);
    getParam("registration/coarse_trigger_min_correspondence_ratio",
             registration.coarse_trigger_min_correspondence_ratio);
    getParam("registration/max_rmse", registration.max_rmse);
    getParam("registration/huber_delta", registration.huber_delta);
    getParam("registration/residual_std_floor", registration.residual_std_floor);
    getParam("registration/measurement_translation_std_floor",
             registration.measurement_translation_std_floor);
    double measurement_rotation_std_floor_deg = 0.0;
    getParam<double>("registration/measurement_rotation_std_floor_deg",
                     measurement_rotation_std_floor_deg, 0.0);
    registration.measurement_rotation_std_floor_rad =
        degreesToRadians(measurement_rotation_std_floor_deg);
    getParam("registration/min_coverage_sectors", registration.min_coverage_sectors);
    getParam("registration/degeneracy_condition_threshold",
             registration.degeneracy_condition_threshold);
}

/// 读取 NORMAL 跟踪和系统联合创新参数。
void ParameterReader::loadTracking() {
    getParam("tracking/correction_period", parameters_.runtime.tracking.correction_period);
    int tracking_target_keyframes = 16;
    getParam<int>("tracking/target_keyframes", tracking_target_keyframes, 16);
    parameters_.runtime.tracking.target_keyframes =
        static_cast<std::size_t>(std::max(1, tracking_target_keyframes));
    getParam("tracking/target_radius", parameters_.runtime.tracking.target_radius);
    getParam("tracking/nearest_reference_radius",
             parameters_.runtime.tracking.nearest_reference_radius);
    getParam("tracking/target_hysteresis_distance",
             parameters_.runtime.tracking.target_hysteresis_distance);
    getParam("tracking/source/min_frames", parameters_.runtime.tracking.source.min_frames);
    getParam("tracking/source/max_frames", parameters_.runtime.tracking.source.max_frames);
    getParam("tracking/source/max_time", parameters_.runtime.tracking.source.max_time);
    getParam("tracking/source/max_translation",
             parameters_.runtime.tracking.source.max_translation);
    double tracking_source_max_rotation_deg = 0.0;
    getParam<double>("tracking/source/max_rotation_deg", tracking_source_max_rotation_deg, 0.0);
    parameters_.runtime.tracking.source.max_rotation_rad =
        degreesToRadians(tracking_source_max_rotation_deg);
    getParam("tracking/max_translation_correction",
             parameters_.runtime.tracking.max_translation_correction);
    double max_rotation_correction_deg = 15.0;
    getParam<double>("tracking/max_rotation_correction_deg", max_rotation_correction_deg, 15.0);
    parameters_.runtime.tracking.max_rotation_correction_rad =
        degreesToRadians(max_rotation_correction_deg);
    getParam("tracking/suspicious_linear_speed",
             parameters_.runtime.tracking.suspicious_linear_speed);
    getParam("tracking/suspicious_angular_speed",
             parameters_.runtime.tracking.suspicious_angular_speed);

    getParam("innovation/rejection_probability",
             parameters_.runtime.innovation.rejection_probability);
    getParam("innovation/conflicts_before_suspect",
             parameters_.runtime.innovation.conflicts_before_suspect);
}

/// 读取 INITIALIZING、SUSPECT 和 LOST 共用的恢复参数。
void ParameterReader::loadRecovery() {
    getParam("recovery/period", parameters_.runtime.recovery.period);
    getParam("recovery/suspect_search_radius", parameters_.runtime.recovery.suspect_search_radius);
    getParam("recovery/lost_search_radius", parameters_.runtime.recovery.lost_search_radius);
    getParam("recovery/global_fallback_candidates",
             parameters_.runtime.recovery.global_fallback_candidates);
    getParam("recovery/suspect_failures_before_lost",
             parameters_.runtime.recovery.suspect_failures_before_lost);
    getParam("recovery/automatic_when_lost", parameters_.runtime.recovery.automatic_when_lost);
    getParam("recovery/max_translation_correction",
             parameters_.runtime.recovery.max_translation_correction);
    double recovery_max_rotation_correction_deg = 20.0;
    getParam<double>("recovery/max_rotation_correction_deg", recovery_max_rotation_correction_deg,
                     20.0);
    parameters_.runtime.recovery.max_rotation_correction_rad =
        degreesToRadians(recovery_max_rotation_correction_deg);
    getParam("recovery/manual_bypasses_correction_budget",
             parameters_.runtime.recovery.manual_bypasses_correction_budget);
    getParam("recovery/source/min_frames", parameters_.runtime.recovery.source.min_frames);
    getParam("recovery/source/max_frames", parameters_.runtime.recovery.source.max_frames);
    getParam("recovery/source/max_time", parameters_.runtime.recovery.source.max_time);
    getParam("recovery/source/max_translation",
             parameters_.runtime.recovery.source.max_translation);
    double recovery_source_max_rotation_deg = 0.0;
    getParam<double>("recovery/source/max_rotation_deg", recovery_source_max_rotation_deg, 0.0);
    parameters_.runtime.recovery.source.max_rotation_rad =
        degreesToRadians(recovery_source_max_rotation_deg);
    getParam("recovery/initial_max_descriptor_distance",
             parameters_.runtime.relocalization.initial_relocalization_max_descriptor_distance);
    getParam("recovery/descriptor_distance_limit",
             parameters_.runtime.relocalization.relocalization_descriptor_distance_limit);
    getParam("recovery/yaw_sweep_count",
             parameters_.runtime.relocalization.relocalization_yaw_sweep_count);
    getParam("recovery/parallel_candidates",
             parameters_.runtime.relocalization.relocalization_parallel_workers);
    getParam("recovery/max_candidates",
             parameters_.runtime.relocalization.relocalization_max_candidates);
    getParam("recovery/target_keyframes",
             parameters_.runtime.relocalization.relocalization_submap_keyframes);
    getParam("recovery/target_radius",
             parameters_.runtime.relocalization.relocalization_submap_radius);
}

namespace {

/// 识别归属于 rclcpp 与 QoS 覆盖机制的框架参数。
bool isFrameworkParameter(const std::string& name) {
    // 这些参数归 rclcpp 所有；QoS 覆盖叶子由 ROS 工具按命名空间注入。
    return name == "use_sim_time" || name == "start_type_description_service" ||
           name.rfind("qos_overrides.", 0) == 0;
}

}  // namespace

/// 读取可视化、服务、兼容接口和世界点云输出参数。
void ParameterReader::loadOutputs() {
    getParam("visualization/enabled", parameters_.visualization.enabled);
    getParam<bool>("visualization/publish_prior_map", parameters_.visualization.publish_prior_map,
                   parameters_.visualization.enabled);
    getParam<bool>("visualization/publish_runtime_map",
                   parameters_.visualization.publish_runtime_map,
                   parameters_.visualization.enabled);
    getParam<bool>("visualization/publish_map_axes", parameters_.visualization.publish_map_axes,
                   parameters_.visualization.enabled);
    getParam<bool>("visualization/publish_paths", parameters_.visualization.publish_paths,
                   parameters_.visualization.enabled);
    getParam("visualization/publish_debug_registration",
             parameters_.visualization.publish_registration);
    parameters_.runtime.registration_debug_enabled = parameters_.visualization.publish_registration;
    getParam("visualization/map_publish_period", parameters_.maps.publish_period);

    getParam("services/save_map", parameters_.services.save_map);
    getParam("services/relocalize", parameters_.services.relocalize);
    getParam("services/reset", parameters_.services.reset);
    getParam("output/planar_map_to_odom", parameters_.planar_map_to_odom);
    getParam("output/legacy/enabled", parameters_.legacy.enabled);
    getParam("output/legacy/publish_pose", parameters_.legacy.publish_pose);
    getParam("output/legacy/publish_twist", parameters_.legacy.publish_twist);
    getParam("output/legacy/publish_path", parameters_.legacy.publish_path);
    getParam("output/legacy/publish_tf", parameters_.legacy.publish_tf);
    getParam("output/legacy/publish_initializing", parameters_.legacy.publish_initializing);
    getParam("output/legacy/publish_untrusted", parameters_.legacy.publish_untrusted);
    getParam("output/legacy/pose_topic", parameters_.legacy.pose_topic);
    getParam("output/legacy/twist_topic", parameters_.legacy.twist_topic);
    getParam("output/legacy/path_topic", parameters_.legacy.path_topic);
    getParam("output/legacy/danger_topic", parameters_.legacy.danger_topic);
    getParam("output/legacy/path_period", parameters_.legacy.path_period);
    getParam("output/legacy/path_max_size", parameters_.legacy.path_max_size);
    getParam("output/world_cloud/enabled", parameters_.world_cloud.enabled);
    getParam("output/world_cloud/publish_period", parameters_.world_cloud.publish_period);
    getParam("output/world_cloud/publish_initializing",
             parameters_.world_cloud.publish_initializing);
    getParam("output/world_cloud/publish_untrusted", parameters_.world_cloud.publish_untrusted);
    getParam("output/world_cloud/topic", parameters_.world_cloud.topic);
    parameters_.world_cloud.publish_period = std::max(0.0, parameters_.world_cloud.publish_period);
}

/// 校验跨参数能力关系，并执行明确记录的安全覆盖。
void ParameterReader::validateRelationships() {
    if (parameters_.runtime.prior_map_enabled && parameters_.maps.prior_directory.empty()) {
        throw std::invalid_argument("prior_map.enabled requires prior_map.directory");
    }
    if (parameters_.runtime.prior_map_enabled && parameters_.maps.save_on_shutdown) {
        RCLCPP_WARN(node_.get_logger(),
                    "prior-map mode does not export a runtime map bundle; "
                    "disabling trajectory.save_on_shutdown");
        parameters_.maps.save_on_shutdown = false;
    }
    if (parameters_.runtime.prior_map_enabled && parameters_.visualization.publish_runtime_map) {
        RCLCPP_WARN(node_.get_logger(),
                    "prior-map mode displays the immutable prior map; "
                    "disabling visualization.publish_runtime_map");
        parameters_.visualization.publish_runtime_map = false;
    }
}

/// 拒绝 schema 未消费的拼写错误或未知参数。
void ParameterReader::validateParameterSchema() const {
    const auto listed = node_.list_parameters({}, 100);
    std::vector<std::string> unknown;
    unknown.reserve(listed.names.size());
    for (const std::string& name : listed.names) {
        if (consumed_parameter_names_.count(name) == 0U && !isFrameworkParameter(name)) {
            unknown.push_back(name);
        }
    }
    if (unknown.empty()) {
        return;
    }

    std::ostringstream message;
    message << "unknown backend parameters detected; refusing to silently use "
               "defaults. Check spelling and the capability schema. First "
               "offending parameters: ";
    const std::size_t count = std::min<std::size_t>(unknown.size(), 12);
    for (std::size_t index = 0; index < count; ++index) {
        if (index != 0) {
            message << ", ";
        }
        message << unknown[index];
    }
    throw std::invalid_argument(message.str());
}

/// 读取所选地点描述子的完整参数集。
void ParameterReader::loadDescriptor() {
    getParam("common/descriptor_type", parameters_.descriptor.type);
    if (parameters_.descriptor.type == "scan_context") {
        ScanContextDescriptorConfig& descriptor_config = parameters_.descriptor.scan_context;
        getParam("common/scan_context/enabled", descriptor_config.enabled);
        getParam("common/scan_context/num_rings", descriptor_config.num_rings);
        getParam("common/scan_context/num_sectors", descriptor_config.num_sectors);
        getParam("common/scan_context/max_radius", descriptor_config.max_radius);
        getParam("common/scan_context/downsample", descriptor_config.downsample);
        getParam("common/scan_context/lidar_height", descriptor_config.lidar_height);
        getParam("common/scan_context/min_bin_points", descriptor_config.min_bin_points);
        getParam("common/scan_context/augment_count", descriptor_config.augment_count);
        getParam("common/scan_context/lateral_shift", descriptor_config.lateral_shift);
        getParam("common/scan_context/search_ratio", descriptor_config.search_ratio);
        getParam("common/scan_context/exclude_recent", descriptor_config.exclude_recent);
        getParam("common/scan_context/candidates", descriptor_config.candidates);
        getParam("common/scan_context/distance_threshold", descriptor_config.distance_threshold);
        getParam("common/scan_context/level_with_odom_roll_pitch", parameters_.descriptor.leveling);
        descriptor_config.level_with_odom_roll_pitch = parameters_.descriptor.leveling;
        return;
    }

    if (parameters_.descriptor.type == "gprobe") {
        GProbeDescriptorConfig& descriptor_config = parameters_.descriptor.gprobe;
        getParam("common/gprobe/enabled", descriptor_config.enabled);
        getParam("common/gprobe/level_with_odom_roll_pitch", parameters_.descriptor.leveling);
        descriptor_config.level_with_odom_roll_pitch = parameters_.descriptor.leveling;
        getParam("common/gprobe/num_rings", descriptor_config.num_rings);
        getParam("common/gprobe/num_sectors", descriptor_config.num_sectors);
        getParam("common/gprobe/max_range", descriptor_config.max_range);
        getParam("common/gprobe/voxel_size", descriptor_config.voxel_size);
        getParam("common/gprobe/sigma_t", descriptor_config.sigma_t);
        getParam<double>("common/gprobe/query_fov_deg", descriptor_config.query_fov_deg, 360.0);
        getParam<double>("common/gprobe/query_fov_center_deg",
                         descriptor_config.query_fov_center_deg, 0.0);
        getParam("common/gprobe/branch_top_k", descriptor_config.branch_top_k);
        getParam("common/gprobe/verification_top_k", descriptor_config.verification_top_k);
        getParam("common/gprobe/spatial_nms_radius", descriptor_config.spatial_nms_radius);
        getParam("common/gprobe/heading_window_deg", descriptor_config.heading_window_deg);
        getParam("common/gprobe/column_trim_ratio", descriptor_config.column_trim_ratio);
        getParam("common/gprobe/sgrt_kappa", descriptor_config.sgrt_kappa);
        getParam("common/gprobe/exclude_recent", descriptor_config.exclude_recent);
        getParam("common/gprobe/candidates", descriptor_config.candidates);
        return;
    }
    throw std::invalid_argument("common.descriptor_type must be scan_context or gprobe");
}

}  // namespace backend::ros
