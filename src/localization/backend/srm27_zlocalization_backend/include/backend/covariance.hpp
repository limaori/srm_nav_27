/**
 * @file covariance.hpp
 * @brief 6DoF 位姿协方差转换和传播工具。
 */
#pragma once

#include <Eigen/Core>
#include <array>
#include <string>

namespace backend {

using PoseCovariance6d = Eigen::Matrix<double, 6, 6>;

struct CovarianceValidationOptions {
    /// 只修复舍入量级的非对称误差。
    double absolute_symmetry_tolerance = 1.0e-12;
    double relative_symmetry_tolerance = 1.0e-6;
    /// 超过该尺度的负特征值证明输入不是协方差；舍入量级负值可保守抬升。
    double absolute_negative_eigenvalue_tolerance = 1.0e-12;
    double relative_negative_eigenvalue_tolerance = 1.0e-9;
    /// 数值与物理共同约束的最小特征值，不允许接受更高置信度。
    double minimum_eigenvalue = 1.0e-8;
    /// 允许强各向异性；近似奇异矩阵只能通过增大小特征值进行正则化。
    double maximum_condition_number = 1.0e12;
    double maximum_diagonal = 1.0e6;
};

/// 校验协方差且绝不增加其信息量。明显非对称、非有限或不定的输入会被拒绝；只允许
/// 对称化舍入误差、抬升舍入量级负特征值以及增大过小的正特征值。
bool validateAndRegularizeCovariance(const Eigen::MatrixXd& input, Eigen::MatrixXd* output,
                                     std::string* invalid_reason = nullptr,
                                     bool* regularized = nullptr,
                                     const CovarianceValidationOptions& options = {});

/// 验收六维位姿协方差并保守正则化，保持平移、旋转及交叉块。
bool validateAndRegularizePoseCovariance(const PoseCovariance6d& input, PoseCovariance6d* output,
                                         std::string* invalid_reason = nullptr,
                                         bool* regularized = nullptr);

/// 将 ROS 行主序数组恢复为平移、旋转顺序的六维协方差。
PoseCovariance6d poseCovarianceFromRos(const std::array<double, 36>& covariance);

/// 返回缺少有效前端协方差时的保守位姿不确定性。
PoseCovariance6d fallbackFrontendPoseCovariance();

/// 旋转位姿协方差的平移、旋转及交叉块。
PoseCovariance6d rotatePoseCovariance(const PoseCovariance6d& covariance,
                                      const Eigen::Matrix3d& rotation);

/// 把 ROS 绝对位姿协方差转换为 GTSAM Pose3 使用的局部右扰动切空间。ROS 在位姿
/// header 坐标系表达位置加性误差和固定轴姿态误差，并非完整 SE(3) 左扰动，
/// 因此绝对平移不得引入杆臂项。
PoseCovariance6d worldAdditiveToLocalPoseCovariance(
    const Eigen::Matrix4d& world_pose, const PoseCovariance6d& world_additive_covariance);

/// 把 GTSAM Pose3 局部右扰动边缘协方差转换为位姿 world/header 坐标系中的 ROS
/// 位置加性误差与固定轴姿态协方差。
PoseCovariance6d localToWorldAdditivePoseCovariance(const Eigen::Matrix4d& world_pose,
                                                    const PoseCovariance6d& local_covariance);

/// 将姿态与位置的不确定性传播到刚性连杆臂端点。
PoseCovariance6d propagateLeverArmCovariance(const PoseCovariance6d& covariance,
                                             const Eigen::Vector3d& lever_arm);

/// 把重力对齐坐标变换的 [平移, 旋转] 协方差传播到 `position_alignment` 处位姿的
/// 世界加性协方差。旋转不确定度通过完整 roll/pitch/yaw 杆臂 Jacobian 影响位置；
/// 位姿偏离对齐原点时只保留 yaw 会低估位置不确定度。
PoseCovariance6d alignmentToPoseCovariance(const PoseCovariance6d& alignment_covariance,
                                           const Eigen::Matrix3d& R_world_alignment,
                                           const Eigen::Vector3d& position_alignment);

/// 后端切空间顺序 [平移, 旋转] 下的 SE(3) Adjoint。
PoseCovariance6d poseAdjoint(const Eigen::Matrix4d& transform);

/// 把 `Exp(delta_left) * pose` 的协方差转换为 GTSAM 使用的局部右扰动
/// `pose * Exp(delta_right)`。
PoseCovariance6d leftToRightPoseCovariance(const Eigen::Matrix4d& pose,
                                           const PoseCovariance6d& covariance);

/// 将世界系位姿边缘协方差通过局部右扰动协方差描述的相对运动传播。结果是
/// `world_from * from_to` 在 ROS 位姿约定下的世界系位置加性与固定轴旋转协方差。
PoseCovariance6d propagatePoseWithRelativeCovariance(
    const Eigen::Matrix4d& world_from, const PoseCovariance6d& from_world_covariance,
    const Eigen::Matrix4d& world_to, const PoseCovariance6d& relative_local_covariance);

/// 将六维协方差写为 ROS 行主序数组。
void writePoseCovariance(const PoseCovariance6d& covariance, std::array<double, 36>* out);

/// 施加配置的平移与旋转方差下限。
void applyPoseCovarianceFloor(PoseCovariance6d* covariance, const double translation_std,
                              const double rotation_std);
}  // namespace backend
