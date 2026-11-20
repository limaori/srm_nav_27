/**
 * @file config.hpp
 * @brief 定义后端 ROS 输入、输出和地图管理参数。
 */
#pragma once

#include <string>

#include "backend/body.hpp"
#include "backend/config.hpp"
#include "backend/gprobe.hpp"
#include "backend/scan_context.hpp"

namespace backend::ros {

/// 前端消息、坐标系、同步器和维护周期参数。
struct RosInputParameters {
    std::string frontend = "legacy";
    std::string frame_topic = "/frontend/frame";
    std::string prediction_topic = "/frontend/odometry";
    std::string session_topic = "/frontend/session";
    std::string sensor_frame = "imu";
    std::string uwb_topic = "/uwb_pose";
    std::string uwb_frame = "uwb";
    std::string map_frame = "map";
    std::string odom_frame = "camera_init";
    std::string body_frame = "body";
    std::string odom_topic = "/Odometry";
    std::string sync_odom_topic = "/Odometry_scan_end";
    std::string cloud_topic = "/cloud_registered";
    bool cloud_in_local_frame = false;
    int subscription_queue = 50;
    int synchronizer_queue = 20;
    double synchronizer_max_interval = 0.05;
    double maintenance_period = 0.2;
    BodyTransform body_transform;
};

/// 描述子类型、姿态整平及各实现的参数。
struct DescriptorParameters {
    std::string type = "scan_context";
    bool leveling = true;
    ScanContextDescriptorConfig scan_context;
    GProbeDescriptorConfig gprobe;
};

/// 先验图加载、运行轨迹保存和地图可视化频率参数。
struct MapStorageParameters {
    std::string prior_directory;
    double prior_voxel = 0.3;
    bool save_on_shutdown = false;
    bool save_global_pcd = true;
    std::string save_directory;
    double runtime_voxel = 0.3;
    double publish_period = 2.0;
};

/// 地图、轨迹、状态和配准点云的可视化开关。
struct VisualizationParameters {
    bool enabled = false;
    bool publish_prior_map = false;
    bool publish_runtime_map = false;
    bool publish_map_axes = false;
    bool publish_paths = false;
    bool publish_registration = false;
};

/// 前端重置客户端及后端公开服务的名称。
struct ServiceParameters {
    std::string frontend_reset;
    std::string save_map = "~/save_map";
    std::string relocalize = "~/relocalize";
    std::string reset = "/localization_reset";
};

/// 兼容位姿、速度、路径、TF 和危险码的发布参数。
struct LegacyOutputParameters {
    bool enabled = false;
    bool publish_pose = true;
    bool publish_twist = true;
    bool publish_path = false;
    bool publish_tf = true;
    bool publish_initializing = false;
    bool publish_untrusted = true;
    std::string pose_topic = "/tdt_lio_pose";
    std::string twist_topic = "/tdt_lio_twist";
    std::string path_topic = "/tdt_lio_path";
    std::string danger_topic = "/LAM_danger_situation";
    double path_period = 0.1;
    int path_max_size = 2000;
};

/// 世界坐标系实时点云的发布参数。
struct WorldCloudOutputParameters {
    bool enabled = false;
    double publish_period = 0.0;
    bool publish_initializing = false;
    bool publish_untrusted = false;
    std::string topic = "/localization/lio_world_cloud";
};

/// 汇集后端核心参数与 ROS 接口配置。
struct NodeConfig {
    Config runtime;
    RosInputParameters input;
    DescriptorParameters descriptor;
    MapStorageParameters maps;
    VisualizationParameters visualization;
    ServiceParameters services;
    LegacyOutputParameters legacy;
    WorldCloudOutputParameters world_cloud;
};

/// 校验 UWB 输入坐标系名称与固定变换、在线空间标定开关是否一致。
inline bool uwbFrameContractValid(const UwbConfig& uwb, const std::string& source_frame,
                                  const std::string& map_frame) {
    if (!uwb.enabled) {
        return true;
    }
    if (source_frame.empty()) {
        return false;
    }
    if (source_frame != map_frame) {
        return true;
    }
    return !uwb.estimate_zero_pose_online &&
           uwb.T_map_uwb_fixed.isApprox(Eigen::Matrix4d::Identity(), 1.0e-12);
}

}  // namespace backend::ros
