/**
 * @file alignment.cpp
 * @brief 由基准点、显式位姿或多分辨率点面配准构造地图间刚体变换。
 */

#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

#include "backend/geometry.hpp"
#include "backend/registration.hpp"
#include "backend/sampling.hpp"
#include "transform.hpp"

namespace backend::tools::transform {
namespace fs = std::filesystem;

constexpr double kDegToRad = M_PI / 180.0;

/// 由原点和轴向基准点构造右手正交坐标系及旧图到新图变换。
Eigen::Matrix4d buildNewFromOldTransform(const TransformOptions& args) {
    const Eigen::Vector3d ex_raw = args.x_axis_point - args.origin;
    const Eigen::Vector3d ey_raw = args.y_axis_point - args.origin;
    const Eigen::Vector3d ez_hint = args.z_axis_point - args.origin;

    if (ex_raw.norm() < 1e-9 || ey_raw.norm() < 1e-9 || ez_hint.norm() < 1e-9) {
        throw std::runtime_error("Axis points must be different from origin");
    }

    Eigen::Vector3d ex = ex_raw.normalized();
    Eigen::Vector3d ey = ey_raw - ex * ex.dot(ey_raw);
    if (ey.norm() < 1e-9) {
        throw std::runtime_error("x-axis and y-axis points are nearly collinear");
    }
    ey.normalize();

    Eigen::Vector3d ez = ex.cross(ey);
    if (ez.norm() < 1e-9) {
        throw std::runtime_error("x-axis and y-axis points define an invalid frame");
    }
    ez.normalize();
    if (ez.dot(ez_hint) < 0.0) {
        ey = -ey;
        ez = -ez;
    }

    Eigen::Matrix4d t_new_old = Eigen::Matrix4d::Identity();
    Eigen::Matrix3d r_old_new;
    r_old_new.col(0) = ex;
    r_old_new.col(1) = ey;
    r_old_new.col(2) = ez;
    t_new_old.block<3, 3>(0, 0) = r_old_new.transpose();
    t_new_old.block<3, 1>(0, 3) = -r_old_new.transpose() * args.origin;
    return t_new_old;
}

/// 归一化指定四元数并结合平移构造源到目标的刚体变换。
Eigen::Matrix4d buildManualTransform(const TransformOptions& args) {
    Eigen::Quaterniond q(args.manual_qw, args.manual_qx, args.manual_qy, args.manual_qz);
    if (q.norm() < 1e-12) {
        throw std::runtime_error("Manual quaternion norm is too small");
    }
    q.normalize();

    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block<3, 3>(0, 0) = q.toRotationMatrix();
    transform.block<3, 1>(0, 3) = args.manual_translation;
    return transform;
}

/// 使用 PCL 有限点过滤生成用于配准的点云副本。
Cloud::Ptr cleanCloud(const Cloud& input) {
    Cloud::Ptr output(new Cloud);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(input, *output, indices);
    return output;
}

/// 读取 PCD 头并检查指定点字段是否存在。
bool pcdHasField(const std::string& path, const std::string& field_name) {
    pcl::PCDReader reader;
    pcl::PCLPointCloud2 header;
    if (reader.readHeader(path, header) != 0) {
        return false;
    }
    return std::any_of(header.fields.begin(), header.fields.end(),
                       [&field_name](const auto& field) { return field.name == field_name; });
}

/// 兼容带反射强度或仅 XYZ 的 PCD，补齐字段并过滤无效点。
Cloud::Ptr loadCloudCompat(const std::string& path) {
    Cloud::Ptr cloud(new Cloud);
    if (pcdHasField(path, "intensity")) {
        if (pcl::io::loadPCDFile<PointType>(path, *cloud) != 0 || cloud->empty()) {
            throw std::runtime_error("Failed to load point cloud: " + path);
        }
        return cleanCloud(*cloud);
    }

    pcl::PointCloud<pcl::PointXYZ> xyz_cloud;
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, xyz_cloud) != 0 || xyz_cloud.empty()) {
        throw std::runtime_error("Failed to load point cloud: " + path);
    }

    cloud->reserve(xyz_cloud.size());
    for (const auto& p : xyz_cloud.points) {
        PointType out;
        out.x = p.x;
        out.y = p.y;
        out.z = p.z;
        out.intensity = 0.0f;
        cloud->push_back(out);
    }
    cloud->width = static_cast<std::uint32_t>(cloud->size());
    cloud->height = 1;
    cloud->is_dense = xyz_cloud.is_dense;
    return cleanCloud(*cloud);
}

/// 计算非空点云的三维质心，空点云抛出异常。
Eigen::Vector3d centroidOf(const Cloud& cloud) {
    if (cloud.empty()) {
        throw std::runtime_error("Cannot compute centroid of an empty cloud");
    }

    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    for (const auto& point : cloud.points) {
        sum += Eigen::Vector3d(point.x, point.y, point.z);
    }
    return sum / static_cast<double>(cloud.size());
}

/// 构造指定偏航旋转，并使变换后的源质心与目标质心重合。
Eigen::Matrix4d yawTransformAboutCentroids(const double yaw_rad,
                                           const Eigen::Vector3d& source_centroid,
                                           const Eigen::Vector3d& target_centroid) {
    const double c = std::cos(yaw_rad);
    const double s = std::sin(yaw_rad);
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    R << c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0;

    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block<3, 3>(0, 0) = R;
    transform.block<3, 1>(0, 3) = target_centroid - R * source_centroid;
    return transform;
}

/// 通过归一化四元数规整变换中的旋转部分。
Eigen::Matrix4d orthonormalizedTransform(const Eigen::Matrix4d& input) {
    Eigen::Matrix4d output = input;
    Eigen::Quaterniond q(input.block<3, 3>(0, 0));
    q.normalize();
    output.block<3, 3>(0, 0) = q.toRotationMatrix();
    return output;
}

/// 按指定体素尺度准备配准点云，禁用采样时返回完整副本。
Cloud::Ptr downsampleForAlignment(const Cloud& cloud, const double voxel_resolution) {
    if (voxel_resolution <= 0.0) {
        Cloud::Ptr copy(new Cloud(cloud));
        return copy;
    }
    return voxelize(cloud, voxel_resolution);
}

/// 构造去重的粗到细配准尺度，按配置保留全分辨率终轮。
std::vector<double> alignmentVoxelStages(const TransformOptions& args) {
    std::vector<double> stages;
    /// 规整禁用采样的零尺度，并保留各尺度第一次出现的顺序。
    const auto add_stage = [&stages](const double value) {
        const double normalized = value < 1e-9 ? 0.0 : value;
        for (const double existing : stages) {
            if (std::abs(existing - normalized) < 1e-6) {
                return;
            }
        }
        stages.push_back(normalized);
    };

    if (args.alignment_voxel_resolution <= 0.0) {
        add_stage(0.5);
        add_stage(0.2);
        add_stage(0.05);
        add_stage(0.0);
    } else {
        add_stage(std::max(0.5, args.alignment_voxel_resolution * 8.0));
        add_stage(std::max(0.2, args.alignment_voxel_resolution * 4.0));
        add_stage(args.alignment_voxel_resolution);
    }
    return stages;
}

/// 从关键帧快照重建全局地图并过滤无效点。
Cloud::Ptr buildGlobalMap(const std::vector<Keyframe>& keyframes) {
    return cleanCloud(*KeyframeMap::buildOptimizedMapFromSnapshot(keyframes));
}

/// 依次选择显式源 PCD、地图包全局 PCD 或关键帧重建几何。
Cloud::Ptr loadSourceMapForAlignment(const TransformOptions& args,
                                     const std::vector<Keyframe>& keyframes) {
    if (!args.source_map.empty()) {
        std::cout << "Alignment source map: " << args.source_map << "\n";
        return loadCloudCompat(args.source_map);
    }

    const fs::path map_path = fs::path(args.input_dir) / "map.pcd";
    if (fs::exists(map_path)) {
        std::cout << "Alignment source map: " << map_path.string() << "\n";
        return loadCloudCompat(map_path.string());
    }

    std::cout << "Alignment source map: rebuilt from keyframes\n";
    Cloud::Ptr map = buildGlobalMap(keyframes);
    if (map->empty()) {
        throw std::runtime_error("Cannot build a non-empty source map");
    }
    return map;
}

struct AlignmentResult {
    bool converged = false;
    double rmse = std::numeric_limits<double>::infinity();
    double overlap_ratio = 0.0;
    Eigen::Matrix4d source_to_target = Eigen::Matrix4d::Identity();
    ObservabilityQuality quality;
};

/// 结合当前点数和体素尺度构造点面配准的支持、鲁棒及可观性参数。
RegistrationConfig alignmentConfig(const Cloud& source, const Cloud& target,
                                   const TransformOptions& args, const double voxel_resolution) {
    const std::size_t min_points = std::min(source.size(), target.size());
    if (min_points < 30) {
        throw std::runtime_error("Alignment cloud has too few points");
    }

    RegistrationConfig config;
    config.voxel_resolution = std::max(0.0, voxel_resolution);
    config.max_iterations = args.alignment_max_iterations;
    config.max_correspondence_distance = args.alignment_max_correspondence_distance;
    config.plane_knn = 8;
    config.max_plane_eigenvalue_ratio = 0.2;
    config.min_correspondences = std::clamp(static_cast<int>(min_points / 100U), 30, 200);
    config.min_correspondence_ratio = 0.01;
    config.max_rmse = args.alignment_rmse_threshold;
    config.huber_delta =
        std::min(args.alignment_max_correspondence_distance,
                 voxel_resolution > 0.0 ? std::max(0.1, 1.5 * voxel_resolution) : 0.15);
    config.min_coverage_sectors = 4;
    config.degeneracy_condition_threshold = 10.0;
    return config;
}

/// 执行已准备目标的一轮配准，并计算有限全维似然与重叠率诊断。
AlignmentResult runPreparedAlignmentStage(Registration& matcher, const Cloud& source,
                                          const Cloud& target, const Eigen::Matrix4d& initial_guess,
                                          const TransformOptions& args) {
    const RegistrationResult registration = matcher.align(source, initial_guess);
    AlignmentResult result;
    // 粗偏航和逐级精化可在最终 RMSE/重叠率验收前改善几何，但每一级至少必须
    // 构造有限似然并完整观测刚体变换。
    result.converged = registration.likelihoodValid() && registration.quality.observable;
    result.rmse = registration.quality.rmse;
    result.source_to_target = orthonormalizedTransform(registration.source_to_target);
    result.overlap_ratio =
        overlapRatio(source, target, result.source_to_target, args.alignment_overlap_distance);
    result.quality = registration.quality;
    return result;
}

/// 返回显式偏航初值或覆盖整周的偏航搜索种子，单位为弧度。
std::vector<double> yawSeedsRad(const TransformOptions& args) {
    std::vector<double> seeds;
    if (args.has_initial_yaw) {
        seeds.push_back(args.initial_yaw_deg * kDegToRad);
        return seeds;
    }

    if (args.alignment_yaw_search_step_deg <= 0.0) {
        seeds.push_back(0.0);
        return seeds;
    }

    for (double yaw_deg = 0.0; yaw_deg < 360.0; yaw_deg += args.alignment_yaw_search_step_deg) {
        seeds.push_back(yaw_deg * kDegToRad);
    }
    return seeds;
}

/// 从变换旋转部分提取以度表示的偏航角。
double yawFromTransformDeg(const Eigen::Matrix4d& transform) {
    return std::atan2(transform(1, 0), transform(0, 0)) / kDegToRad;
}

/// 打印平移、偏航角及完整刚体变换矩阵。
void printTransformSummary(const Eigen::Matrix4d& transform, const std::string& label) {
    const Eigen::Vector3d t = transform.block<3, 1>(0, 3);
    std::cout << label << "\n";
    std::cout << "  translation: " << t.x() << " " << t.y() << " " << t.z() << "\n";
    std::cout << "  yaw_deg: " << yawFromTransformDeg(transform) << "\n";
    std::cout << "  matrix:\n" << transform << "\n";
}

/// 先按粗尺度选择偏航种子，再逐级精化并验收最终误差和重叠率。
Eigen::Matrix4d estimatePointToPlaneTransform(const TransformOptions& args,
                                              const Cloud& source_full, const Cloud& target_full) {
    const std::vector<double> stages = alignmentVoxelStages(args);
    if (stages.empty()) {
        throw std::runtime_error("No alignment stages configured");
    }

    const Eigen::Vector3d source_centroid = centroidOf(source_full);
    const Eigen::Vector3d target_centroid = centroidOf(target_full);
    const double coarse_voxel = stages.front();
    Cloud::Ptr source_coarse = downsampleForAlignment(source_full, coarse_voxel);
    Cloud::Ptr target_coarse = downsampleForAlignment(target_full, coarse_voxel);

    Registration coarse_matcher(
        alignmentConfig(*source_coarse, *target_coarse, args, coarse_voxel));
    if (!coarse_matcher.setTarget(target_coarse, 0)) {
        throw std::runtime_error("Target map has insufficient planar support");
    }

    // 在最粗尺度复用同一目标几何，比较全部允许的偏航种子。
    AlignmentResult best;
    double best_key = std::numeric_limits<double>::infinity();
    double best_yaw = 0.0;
    for (const double yaw_rad : yawSeedsRad(args)) {
        const Eigen::Matrix4d initial =
            yawTransformAboutCentroids(yaw_rad, source_centroid, target_centroid);
        const AlignmentResult result = runPreparedAlignmentStage(coarse_matcher, *source_coarse,
                                                                 *target_coarse, initial, args);

        const double key =
            (result.converged ? 0.0 : 1e9) + result.rmse - result.overlap_ratio * 0.01;
        if (args.verbose_alignment) {
            std::cout << "yaw seed " << yaw_rad / kDegToRad
                      << " deg: converged=" << result.converged << " rmse=" << result.rmse
                      << " overlap=" << result.overlap_ratio << "\n";
        }
        if (result.converged && key < best_key) {
            best_key = key;
            best = result;
            best_yaw = yaw_rad;
        }
    }

    if (!std::isfinite(best_key)) {
        throw std::runtime_error("Point-to-plane alignment failed for all yaw seeds");
    }

    Eigen::Matrix4d estimate = best.source_to_target;
    if (args.verbose_alignment) {
        std::cout << "Selected yaw seed: " << best_yaw / kDegToRad << " deg\n";
    }

    // 从已选的完整可观位姿出发，按分辨率顺序逐级精化。
    AlignmentResult accepted = best;
    for (std::size_t i = 1; i < stages.size(); ++i) {
        const double voxel = stages[i];
        Cloud::Ptr source_stage = downsampleForAlignment(source_full, voxel);
        Cloud::Ptr target_stage = downsampleForAlignment(target_full, voxel);
        Registration matcher(alignmentConfig(*source_stage, *target_stage, args, voxel));
        if (!matcher.setTarget(target_stage, static_cast<int>(i))) {
            throw std::runtime_error(
                "Point-to-plane refinement target has insufficient support "
                "at voxel " +
                std::to_string(voxel));
        }
        const AlignmentResult stage =
            runPreparedAlignmentStage(matcher, *source_stage, *target_stage, estimate, args);
        if (args.verbose_alignment) {
            std::cout << "refine voxel " << voxel << ": converged=" << stage.converged
                      << " rmse=" << stage.rmse << " overlap=" << stage.overlap_ratio << "\n";
        }
        if (!stage.converged) {
            throw std::runtime_error("Point-to-plane refinement did not converge at voxel " +
                                     std::to_string(voxel));
        }
        accepted = stage;
        estimate = stage.source_to_target;
    }

    // 在请求的最终采样尺度验收重叠率，并检查最后一轮配准残差。
    Cloud::Ptr source_eval = downsampleForAlignment(source_full, args.alignment_voxel_resolution);
    Cloud::Ptr target_eval = downsampleForAlignment(target_full, args.alignment_voxel_resolution);
    const double final_overlap =
        overlapRatio(*source_eval, *target_eval, estimate, args.alignment_overlap_distance);
    if (accepted.rmse > args.alignment_rmse_threshold) {
        throw std::runtime_error("Alignment RMSE exceeds threshold");
    }
    if (final_overlap < args.alignment_min_overlap) {
        throw std::runtime_error("Alignment overlap is below threshold");
    }

    std::cout << "Alignment RMSE: " << accepted.rmse << ", overlap: " << final_overlap << "\n";
    return estimate;
}

/// 加载源和目标几何，并通过分级点面配准估计地图间变换。
Eigen::Matrix4d buildTargetMapTransform(const TransformOptions& args,
                                        const std::vector<Keyframe>& keyframes) {
    Cloud::Ptr source = loadSourceMapForAlignment(args, keyframes);
    Cloud::Ptr target = loadCloudCompat(args.target_map);
    if (source->empty() || target->empty()) {
        throw std::runtime_error("Source and target maps must be non-empty");
    }

    std::cout << "Target map: " << args.target_map << "\n";
    std::cout << "Alignment points: source=" << source->size() << ", target=" << target->size()
              << "\n";

    return estimatePointToPlaneTransform(args, *source, *target);
}

}  // namespace backend::tools::transform
