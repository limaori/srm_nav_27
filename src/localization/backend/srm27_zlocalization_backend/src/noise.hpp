/**
 * @file noise.hpp
 * @brief 声明因子图测量边界共用的噪声、白化和坐标顺序工具。
 */
#pragma once

#include <string>
#include <vector>

#include "backend/pose_graph.hpp"

namespace backend::detail {

/// 将具名先验方差转换为 GTSAM 的旋转、平移顺序。
Eigen::Matrix<double, 6, 1> priorVariancesGtsamOrder(const PoseGraphConfig& config);

/// 在后端平移、旋转顺序与 GTSAM 旋转、平移顺序之间转换。
gtsam::Matrix rosToGtsamOrder(const PoseCovariance6d& covariance);

/// 从合法的 SE(2)+可选 Z 齐次变换提取平面对齐状态。
gtsam::Pose2 planarAlignmentFromMatrix(const Eigen::Matrix4d& T_map_uwb);

/// 将世界加性对齐协方差约化到共享平面状态的局部坐标。
gtsam::Matrix alignmentPriorCovarianceLocal(const PoseCovariance6d& world_covariance,
                                            const gtsam::Pose2& planar_alignment, bool use_z);

/// 构造协方差噪声；无效输入使用显式保守回退。
gtsam::SharedNoiseModel covarianceNoiseOrFallback(const gtsam::Matrix& covariance,
                                                  const gtsam::Matrix& fallback,
                                                  const char* context,
                                                  bool zero_means_missing = false);

/// 构造位姿协方差噪声，并处理后端与 GTSAM 的坐标顺序。
gtsam::SharedNoiseModel poseNoise(const PoseCovariance6d& covariance,
                                  const Eigen::Matrix<double, 6, 1>& fallback_gtsam_variances,
                                  const char* context);

/// 严格验收协方差；明显损坏时拒绝因子。
gtsam::SharedNoiseModel strictCovarianceNoise(const gtsam::Matrix& covariance, const char* context);

/// 按残差维数构造统计意义一致的块鲁棒核。
gtsam::SharedNoiseModel robustNoise(const gtsam::SharedNoiseModel& base,
                                    const PoseGraphConfig& config);

/// 构造带原因的无效创新项。
InnovationTerm invalidInnovationTerm(const std::string& reason);

/// 用测量协方差白化一个多变量创新项。
InnovationTerm makeWhitenedInnovationTerm(std::vector<gtsam::Key> keys,
                                          const Eigen::VectorXd& residual,
                                          std::vector<Eigen::MatrixXd> jacobians,
                                          const Eigen::MatrixXd& measurement_covariance,
                                          const std::string& covariance_context);

}  // namespace backend::detail
