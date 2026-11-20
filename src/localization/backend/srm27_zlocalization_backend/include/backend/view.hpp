/**
 * @file view.hpp
 * @brief 定义后端输出位姿、状态与可视化快照。
 */
#pragma once

#include <Eigen/Dense>
#include <string>
#include <utility>
#include <vector>

#include "backend/covariance.hpp"
#include "backend/localization_state.hpp"
#include "backend/types.hpp"

namespace backend {

/// 单次输出的位姿、协方差、定位状态和发布标记。
struct LocalizationProjection {
    /// 输出所属的前端里程计代次。
    std::uint64_t epoch = 0;
    /// 捕获输出时的定位状态代次。
    std::uint64_t state_generation = 0;
    /// 地图系位姿与协方差的提交版本。
    std::uint64_t pose_revision = 0;
    /// 投影是否通过输入与代次有效性检查。
    bool valid = false;
    /// 当前定位状态。
    LocalizationStatus status = LocalizationStatus::Initializing;
    /// 定位状态的诊断原因。
    std::string reason;
    /// 累计估计器故障记录。
    EstimatorFailureHistory estimator_failures{};
    /// 当前输出位姿是否可信。
    bool pose_trusted = false;
    /// 是否具备全局位姿输出：当前位姿可信，或已有可冻结的可信缓存。
    bool global_pose_available = false;
    /// 本次投影是否允许推进全局 TF 发布。
    bool should_publish_tf = false;
    /// 里程计坐标系到地图坐标系的变换。
    Eigen::Matrix4d T_map_odom = Eigen::Matrix4d::Identity();
    /// 传感器在地图坐标系中的位姿。
    Eigen::Matrix4d T_map_sensor = Eigen::Matrix4d::Identity();
    /// 地图系位置加性与固定轴旋转协方差，顺序为 [平移, 旋转]。
    PoseCovariance6d pose_covariance = PoseCovariance6d::Zero();
    /// 输出位姿协方差是否有效。
    bool pose_covariance_valid = false;
};

/// 同时捕获的定位状态、前端代次和位姿版本。
struct LocalizationSnapshot {
    /// 定位状态及其诊断信息。
    LocalizationStateSnapshot localization;
    /// 前端里程计代次。
    std::uint64_t epoch = 0;
    /// 位姿与协方差的提交版本。
    std::uint64_t pose_revision = 0;
};

/// 回环连线与配准调试点云的可视化快照。
struct VisualizationSnapshot {
    /// 是否包含配准调试点云。
    bool include_debug_clouds = false;
    /// 回环连接的关键帧索引对。
    std::vector<std::pair<int, int>> loop_edges;
    /// 完成对齐的源点云。
    Cloud::Ptr debug_aligned_source;
    /// 配准使用的目标点云。
    Cloud::Ptr debug_target;
};

}  // namespace backend
