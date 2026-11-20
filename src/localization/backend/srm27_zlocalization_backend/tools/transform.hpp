/**
 * @file transform.hpp
 * @brief 声明地图包坐标变换的参数、对齐及文件输出接口。
 */
#pragma once

#include <Eigen/Dense>
#include <filesystem>
#include <string>
#include <vector>

#include "backend/keyframe_map.hpp"

namespace backend::tools::transform {

struct TransformOptions {
    std::string input_dir;
    std::string output_dir;
    Eigen::Vector3d origin = Eigen::Vector3d::Zero();
    Eigen::Vector3d x_axis_point = Eigen::Vector3d::Zero();
    Eigen::Vector3d y_axis_point = Eigen::Vector3d::Zero();
    Eigen::Vector3d z_axis_point = Eigen::Vector3d::Zero();
    bool has_origin = false;
    bool has_x_axis = false;
    bool has_y_axis = false;
    bool has_z_axis = false;

    Eigen::Vector3d manual_translation = Eigen::Vector3d::Zero();
    double manual_qx = 0.0;
    double manual_qy = 0.0;
    double manual_qz = 0.0;
    double manual_qw = 1.0;
    bool has_manual_translation = false;
    bool has_manual_quaternion = false;
    bool has_tx = false;
    bool has_ty = false;
    bool has_tz = false;
    bool has_qx = false;
    bool has_qy = false;
    bool has_qz = false;
    bool has_qw = false;

    std::string target_map;
    std::string source_map;
    double alignment_voxel_resolution = 0.2;
    double alignment_max_correspondence_distance = 3.0;
    double alignment_overlap_distance = 0.3;
    // 地图配准最终验收所需的最低重叠率和最大均方根误差。
    double alignment_min_overlap = 0.4;
    double alignment_rmse_threshold = 0.3;
    int alignment_max_iterations = 40;
    double alignment_yaw_search_step_deg = 15.0;
    bool has_initial_yaw = false;
    double initial_yaw_deg = 0.0;
    bool has_alignment_options = false;
    bool verbose_alignment = false;
    bool help = false;
};

/// 输出命令行使用说明。
void printUsage();

/// 解析并校验三种互斥的坐标变换模式。
bool parseArgs(int argc, char** argv, TransformOptions& args);

/// 由四个基准点构造旧地图到新地图的刚体变换。
Eigen::Matrix4d buildNewFromOldTransform(const TransformOptions& args);

/// 由显式平移和四元数构造刚体变换。
Eigen::Matrix4d buildManualTransform(const TransformOptions& args);

/// 通过源地图到目标地图的分级配准估计刚体变换。
Eigen::Matrix4d buildTargetMapTransform(const TransformOptions& args,
                                        const std::vector<Keyframe>& keyframes);

/// 输出变换矩阵及便于人工核对的姿态摘要。
void printTransformSummary(const Eigen::Matrix4d& transform, const std::string& label);

/// 复制不需要重建的地图包文件，跳过点云和描述子产物。
bool copyPassthroughFiles(const std::filesystem::path& input_dir,
                          const std::filesystem::path& output_dir);

/// 若地图包带全局点云，则应用同一刚体变换并写入输出目录。
bool transformGlobalMapIfPresent(const std::string& input_dir, const std::string& output_dir,
                                 const Eigen::Matrix4d& t_new_old);

}  // namespace backend::tools::transform
