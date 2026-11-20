/**
 * @file search.hpp
 * @brief 定义重定位请求、位置先验、结果及检索接口。
 */
#pragma once

#include <Eigen/Dense>
#include <optional>
#include <string>
#include <vector>

#include "backend/config.hpp"
#include "backend/descriptor.hpp"
#include "backend/keyframe_map.hpp"
#include "backend/localization_state.hpp"
#include "backend/registration.hpp"
#include "backend/target_cache.hpp"
#include "backend/types.hpp"

namespace backend {

/// 传感器局部刚性点与地图位置之间的对应先验。
struct RelocalizationPositionPrior {
    /// 位置先验的来源名称。
    std::string source;
    /// 是否使用高度；false 时按 XY 位置检索和验收。
    bool use_z = true;
    /// 先验点在 map 坐标系中的位置，m。
    Eigen::Vector3d map_point = Eigen::Vector3d::Zero();
    /// 同一刚性点在当前传感器坐标系中的位置，m。
    Eigen::Vector3d runtime_sensor_point = Eigen::Vector3d::Zero();
    /// 去除当前车体航向后的传感器朝向，用于叠加候选航向。
    Eigen::Matrix3d R_yawless_sensor = Eigen::Matrix3d::Identity();
    /// 可选的 map 系车体航向初值，rad。
    std::optional<double> yaw_seed_rad;
    /// 航向初值验证失败后，是否继续尝试其余采样航向。
    bool allow_yaw_sweep_fallback = true;
    /// 配准后先验点的位置误差上限，m。
    double search_radius = 0.0;
    /// 位置检索保留的先验关键帧数量上限。
    int max_candidates = 0;
};

/// 单次重定位的输入快照；引用的数据须在检索期间有效。
struct RelocalizationRequest {
    /// 当前运行关键帧。
    const Keyframe& runtime_kf;
    /// 重定位与配准配置。
    const RelocalizationConfig& config;
    /// 先验关键帧的 map 系位置，按索引排列。
    const std::vector<Eigen::Vector3d>& prior_kf_positions;
    /// 可选的细层配准源点云。
    Cloud::ConstPtr fine_source{};
    /// 可选的粗层配准源点云。
    Cloud::ConstPtr coarse_source{};
    /// 捕获请求时的定位状态。
    LocalizationStatus status = LocalizationStatus::Initializing;
    /// 当前运行传感器在 map 系中的预测位姿。
    Eigen::Matrix4d T_map_runtime_pred = Eigen::Matrix4d::Identity();
    /// 可选位置先验；空指针时使用描述子检索。
    const RelocalizationPositionPrior* position_prior = nullptr;
    /// 描述子候选在预测位置附近的搜索半径，m；0 不限。
    double search_radius = 0.0;
    /// 局部检索失败后补充的全局描述子候选数量。
    int global_fallback_candidates = 0;
    /// 源点云配准观测的信息缩放系数。
    double source_information_scale = 1.0;
};

/// 重定位的匹配结果、全局变换与协方差。
struct RelocalizationResult {
    /// 是否找到有效重定位结果。
    bool success = false;
    /// 里程计坐标系到 map 坐标系的恢复变换。
    Eigen::Matrix4d T_map_odom = Eigen::Matrix4d::Identity();
    /// 匹配的先验关键帧索引。
    int prior_index = -1;
    /// 成功匹配的候选来源。
    std::string source;
    /// 恢复时传感器位姿的局部右扰动协方差，顺序为 [平移, 旋转]。
    PoseCovariance6d covariance = PoseCovariance6d::Zero();
    /// 恢复位姿协方差是否有效。
    bool covariance_valid = false;
};

/// 分批执行的对称航向采样序列，偏移值为采样角间隔的整数倍。
struct YawSweepSchedule {
    /// 首批验证的航向偏移序列。
    std::vector<int> primary_offsets;
    /// 首批全部失败后验证的航向偏移序列。
    std::vector<int> fallback_offsets;
};

/// 生成航向采样批次；有可信初值时首批仅含零偏移，否则首批包含完整序列。
YawSweepSchedule makeYawSweepSchedule(int samples, bool has_trusted_seed);

/// 按二维或三维距离选择最近的先验关键帧，等距离时按索引排序。
std::vector<int> nearestPriorKeyframeIndices(const std::vector<Eigen::Vector3d>& prior_positions,
                                             const Eigen::Vector3d& map_point, bool use_z,
                                             int max_candidates);

/// 执行候选检索、航向验证与几何配准，返回重定位结果。
RelocalizationResult runRelocalization(const KeyframeMap& keyframes,
                                       const PlaceDescriptor& descriptor,
                                       RelocalizationTargetCache& target_cache,
                                       const RelocalizationRequest& request);

}  // namespace backend
