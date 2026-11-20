/**
 * @file config.hpp
 * @brief 定义后端配置类型与配置解析接口。
 */
#pragma once

#include <Eigen/Core>
#include <cmath>
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "backend/covariance.hpp"
#include "backend/math.hpp"

namespace backend {

/// UWB 与里程计的坐标对齐及结果验收参数。
struct UwbAlignmentConfig {
    /// 是否估计高度对齐；false 时仅估计水平平移与航向，高度沿用初值。
    bool use_z = true;
    int max_pairs = 120;
    int min_pairs = 5;
    double min_baseline = 3.0;
    double inlier_threshold = 0.75;
    double max_rmse = 0.45;
    double position_std_xy = 0.2;
    double position_std_z = 0.3;
    double max_yaw_std_rad = 10.0 * M_PI / 180.0;
    int stable_solutions = 2;
    double stable_translation_delta = 0.25;
    double stable_yaw_delta_rad = 3.0 * M_PI / 180.0;
    /// 启动及无先验 UWB 重力对齐的横滚、俯仰标准差，rad。
    double gravity_alignment_std_rad = 1.0 * M_PI / 180.0;
};

/// 关键帧的平移、旋转和时间触发参数。
struct KeyframePolicy {
    double distance = 0.8;                   ///< 平移触发阈值，单位 m。
    double angle_rad = 12.0 * M_PI / 180.0;  ///< 旋转触发阈值，单位 rad。
    double max_interval = 1.0;               ///< 最大时间间隔，单位 s。
    double min_interval = 0.25;  ///< 相邻关键帧的最小时间间隔，单位 s。
};

/// 点到面配准、几何验收和观测不确定度参数。
struct RegistrationConfig {
    double voxel_resolution = 0.2;
    int max_iterations = 8;
    double max_correspondence_distance = 0.6;
    int plane_knn = 8;
    /// 局部平面邻域的最小 / 最大特征值比例上限。
    double max_plane_eigenvalue_ratio = 0.2;
    /// 局部平面邻域的中间 / 最大特征值比例下限。
    double min_plane_middle_eigenvalue_ratio = 0.08;
    /// 平面锚点到最远近邻点的距离上限，m；非正值关闭该门限。
    double max_plane_neighbor_distance = 1.2;
    int min_correspondences = 80;
    double min_correspondence_ratio = 0.25;
    /// 细层初始对应点比例低于该值时，尝试粗层配准。
    double coarse_trigger_min_correspondence_ratio = 0.25;
    double max_rmse = 0.18;
    double huber_delta = 0.15;
    /// 缩放观测信息时使用的点到面残差标准差下限，m。
    double residual_std_floor = 0.03;
    /// 配准观测在传感器局部切空间中的共模平移标准差下限，m；0 关闭。
    double measurement_translation_std_floor = 0.0;
    /// 配准观测在传感器局部切空间中的共模旋转标准差下限，rad；0 关闭。
    double measurement_rotation_std_floor_rad = 0.0;
    int min_coverage_sectors = 6;
    /// Schur 子空间退化判定与预条件器使用的条件数阈值。
    double degeneracy_condition_threshold = 10.0;
};

/// 重定位候选检索、航向搜索和目标子图参数。
struct RelocalizationConfig {
    double initial_relocalization_max_descriptor_distance = 0.45;
    double relocalization_descriptor_distance_limit = 0.45;
    int relocalization_yaw_sweep_count = 12;
    int relocalization_parallel_workers = 0;
    int relocalization_max_candidates = 10;
    int relocalization_submap_keyframes = 12;
    double relocalization_submap_radius = 12.0;
    RegistrationConfig point_to_plane_config;
};

/// 回环几何验收和成对一致性检查参数。
struct LoopClosureConfig {
    double translation_max = 7.0;
    double z_max = 1.0;
    double roll_pitch_max_rad = 15.0 * kPi / 180.0;
    double yaw_residual_max_rad = 45.0 * kPi / 180.0;
    /// 配准后统计点云重叠率的近邻距离阈值，m。
    double overlap_distance = 0.5;
    double min_overlap_ratio = 0.25;
    bool enable_pcm = true;
    double pcm_translation_threshold = 1.5;
    double pcm_rotation_threshold_rad = 20.0 * kPi / 180.0;
    int min_consistent_loops = 1;
    int min_candidate_clique_size = 1;
};

/// 按时间、位移和转角累计的里程计增量噪声参数。
struct OdometryProcessNoiseConfig {
    /// 每秒累计的旋转方差，rad²/s。
    double rotation_variance_per_second = 2.0e-4;
    /// 每秒累计的平移方差，m²/s。
    double translation_variance_per_second = 2.0e-2;
    /// 每弧度转角附加的旋转方差，rad²/rad。
    double rotation_variance_per_radian = 0.0;
    /// 每米位移附加的平移方差，m²/m。
    double translation_variance_per_meter = 0.0;

    /// 计算给定时间、位移和转角的增量协方差，采用位姿局部系的 [平移, 旋转] 顺序。
    PoseCovariance6d covariance(double delta_time, double translation_distance,
                                double rotation_angle) const;
};

/// 位姿图的先验、里程计噪声、鲁棒核和固定滞后窗口参数。
struct PoseGraphConfig {
    /// 位姿先验的三轴旋转方差，rad²。
    Eigen::Vector3d prior_rotation_variances = Eigen::Vector3d::Constant(1.0e-4);
    /// 位姿先验的三轴平移方差，m²。
    Eigen::Vector3d prior_translation_variances = Eigen::Vector3d::Constant(1.0e-2);
    OdometryProcessNoiseConfig odom_noise;
    bool use_robust_kernel = true;
    /// 鲁棒核类型，可选 huber 或 cauchy。
    std::string robust_kernel = "huber";
    /// 鲁棒核对白化残差分量 RMS 的转折尺度。
    double robust_kernel_size = 1.0;
    /// 固定滞后窗口时长，s；0 保留全图，启用回环时使用全图。
    double fixed_lag_seconds = 30.0;
};

/// 源点云累积的帧数、时间跨度和运动范围参数。
struct SourceAccumulationConfig {
    /// 累积点云可用于观测的最少帧数。
    int min_frames = 1;
    /// 累积窗口的最大帧数；1 使用单帧。
    int max_frames = 1;
    /// 窗口时间跨度上限，s；0 不限。
    double max_time = 0.0;
    /// 窗口相对平移上限，m；0 不限。
    double max_translation = 0.0;
    /// 窗口相对旋转上限，rad；0 不限。
    double max_rotation_rad = 0.0;
};

/// 初始代次的全局位姿提示及匹配策略。
struct StartupHintConfig {
    /// 初始传感器在 map 系中的位姿；空值表示未设置提示。
    std::optional<Eigen::Matrix4d> T_map_sensor;
    /// 有先验图时，初始代次持续使用提示位姿执行地图匹配，直至成功。
    bool force_until_matched = false;
};

/// 纯里程计模式的地图原点配置。
struct PureOdometryConfig {
    /// odom 原点在 map 系中的水平坐标，m。
    Eigen::Vector2d origin_xy = Eigen::Vector2d::Zero();
};

/// 常规跟踪的观测周期、目标子图和运动检查参数。
struct TrackingConfig {
    double correction_period = 0.8;
    std::size_t target_keyframes = 16;
    double target_radius = 12.0;
    double nearest_reference_radius = 20.0;
    double target_hysteresis_distance = 0.5;
    SourceAccumulationConfig source;
    double max_translation_correction = 1.5;
    double max_rotation_correction_rad = 15.0 * M_PI / 180.0;
    double suspicious_linear_speed = 4.0;
    double suspicious_angular_speed = 3.0;
};

/// 全局观测的一致性检验和连续冲突判定参数。
struct InnovationConfig {
    /// 创新检验的卡方上尾概率下限，低于该值判定为冲突。
    double rejection_probability = 1.0e-3;
    /// 连续校正冲突进入 SUSPECT 的次数阈值。
    int conflicts_before_suspect = 7;
};

/// 恢复调度、候选搜索和失败升级参数。
struct RecoveryConfig {
    double period = 0.2;
    /// SUSPECT 围绕最后可信位置的搜索半径，m。
    double suspect_search_radius = 8.0;
    /// LOST 及初始化阶段的描述子搜索半径，m；0 不限。
    double lost_search_radius = 30.0;
    int global_fallback_candidates = 8;
    int suspect_failures_before_lost = 3;
    bool automatic_when_lost = true;
    SourceAccumulationConfig source;
};

/// UWB 位置观测、时空对齐和启动参数。
struct UwbConfig {
    bool enabled = false;
    /// 是否使用 UWB 高度观测；false 时使用水平二维位置。
    bool use_z = true;
    /// UWB 坐标系到 map 系的固定变换，也用于在线对齐就绪前的初值。
    Eigen::Matrix4d T_map_uwb_fixed = Eigen::Matrix4d::Identity();
    /// 是否借助先验地图在线估计 UWB 坐标系到 map 系的变换。
    bool estimate_zero_pose_online = false;
    /// 独立位置解的更新频率，Hz；用于过滤周期内的保持重发，0 关闭该过滤。
    double effective_rate_hz = 1.0;
    /// UWB 固定时间偏置，s；运动时刻 = 消息时间戳 + 此值。
    double time_offset_seconds = 0.0;
    /// 是否借助先验地图在线估计共享时间偏置，以 time_offset_seconds 为先验均值。
    bool estimate_time_offset_online = false;
    /// 在线时间偏置的先验标准差，s。
    double time_offset_prior_std_seconds = 0.1;
    /// 单次位置解的时间抖动标准差，s；经天线速度传播到位置协方差。
    double time_jitter_std_seconds = 0.05;
    double max_age = 0.25;
    /// UWB 运动时刻与里程计插值端点的最大时间间隔，s。
    double odom_sync_tolerance = 0.25;
    double recovery_search_radius = 2.0;
    int recovery_candidates = 8;
    /// 车体坐标系到前端传感器坐标系的固定变换。
    Eigen::Matrix4d T_sensor_body = Eigen::Matrix4d::Identity();
    /// UWB 天线相位中心在前端传感器坐标系中的位置，m。
    Eigen::Vector3d antenna_position_sensor = Eigen::Vector3d::Zero();
    /// UWB 原始位置观测的协方差，m²。
    Eigen::Matrix3d position_covariance =
        Eigen::Matrix3d::Identity() * 0.04;  // 默认标准差为 20 cm。
    UwbAlignmentConfig bootstrap;
};

/// 回环检索、源点云累积、目标子图和验证参数。
struct LoopConfig {
    bool enabled = false;
    int cooldown_keyframes = 5;
    double candidate_max_distance = 0.0;
    double descriptor_ratio_threshold = 1.0;
    /// 回环源点云的历史累积配置，统一表达在最新关键帧坐标系。
    SourceAccumulationConfig source{1, 4, 0.0, 0.0, 0.0};
    std::size_t target_keyframes = 16;
    double target_radius = 12.0;
    int max_parallel_verifications = 4;
    /// PCM 与可视化保留的回环数量上限；0 保留完整历史。
    int history_size = 200;
    LoopClosureConfig verification;
};

/// 协调重置的时序和停车衔接噪声参数。
struct ResetConfig {
    double min_gap_sec = 10.0;
    double response_timeout_sec = 5.0;
    /// 停车重置衔接的平移标准差，m。
    double stationary_translation_std = 0.05;
    /// 停车重置衔接的旋转标准差，rad。
    double stationary_rotation_std_rad = 1.0 * M_PI / 180.0;
};

/// 原始点云和派生子图的缓存预算。
struct StorageConfig {
    /// 原始点云的缓存保留容量，字节；0 关闭缓存保留。
    std::size_t cloud_cache_bytes = 64U * 1024U * 1024U;
    /// 子图及派生几何的缓存保留容量，字节；0 关闭缓存保留。
    std::size_t submap_cache_bytes = 64U * 1024U * 1024U;
};

/// 汇集后端各模块的运行配置。
struct Config {
    StorageConfig storage;
    /// 是否使用固定先验地图进行全局定位。
    bool prior_map_enabled = false;
    StartupHintConfig startup_hint;
    PureOdometryConfig pure_odometry;
    KeyframePolicy keyframes;
    PoseGraphConfig graph;
    RelocalizationConfig relocalization;
    TrackingConfig tracking;
    InnovationConfig innovation;
    RecoveryConfig recovery;
    UwbConfig uwb;
    LoopConfig loop;
    ResetConfig reset;
    /// 是否生成配准调试点云和可视化快照。
    bool registration_debug_enabled = false;
};

/// 记录功能开关或运行策略的配置覆盖结果。
struct ConfigOverride {
    /// 被覆盖的配置项名称。
    std::string field;
    /// 请求值的文本表示。
    std::string requested;
    /// 生效值的文本表示。
    std::string effective;
    /// 配置覆盖的原因。
    std::string reason;
};

/// 生效的后端参数及其配置覆盖记录。
struct EffectiveConfig {
    Config values;
    std::vector<ConfigOverride> overrides;
};

/// 按各参数的数值范围归一化后端配置。
void normalizeBackendConfigNumerics(Config* config);

/// 归一化数值并解析各功能的启用关系，返回生效配置及覆盖记录。
EffectiveConfig resolveConfig(Config requested, bool descriptor_available);

}  // namespace backend
