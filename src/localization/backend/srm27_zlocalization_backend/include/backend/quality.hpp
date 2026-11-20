/**
 * @file quality.hpp
 * @brief 定义配准质量、Hessian 分析与位姿线性求解接口。
 */
#pragma once

#include <Eigen/Dense>
#include <array>
#include <limits>
#include <string>

namespace backend {

/// 配准残差、对应支撑和可观性评估结果。
struct ObservabilityQuality {
    /// 残差是否满足验收阈值。
    bool residual_ok = false;
    /// 对应点数量、比例和方位覆盖是否通过验收。
    bool correspondences_ok = false;
    /// 六维位姿是否满秩可观。
    bool observable = false;
    /// 参与配准的源点数。
    int source_points = 0;
    /// 有效点面对应数量。
    int correspondences = 0;
    /// 有效对应覆盖的方位扇区数量。
    int coverage_sectors = 0;
    /// 有效对应点数占源点数的比例。
    double correspondence_ratio = 0.0;
    /// 点到面残差均方根，m。
    double rmse = std::numeric_limits<double>::max();
    /// 轴对齐归一化谱强度的最小值。
    double min_eigenvalue = 0.0;
    /// 轴对齐归一化谱强度的最大值。
    double max_eigenvalue = 0.0;
    /// 最大与最小归一化谱强度的比值。
    double condition_number = std::numeric_limits<double>::infinity();
    /// 位姿观测的数值秩。
    int observable_rank = 0;
    /// 平移 Schur 子空间的特征值。
    Eigen::Vector3d translation_spectrum = Eigen::Vector3d::Zero();
    /// 旋转 Schur 子空间的特征值。
    Eigen::Vector3d rotation_spectrum = Eigen::Vector3d::Zero();
    /// 平移 Schur 子空间的条件数。
    double translation_condition = std::numeric_limits<double>::infinity();
    /// 旋转 Schur 子空间的条件数。
    double rotation_condition = std::numeric_limits<double>::infinity();
    /// 各坐标轴最匹配谱方向的弱观测标记，顺序为 [平移, 旋转]。
    std::array<bool, 6> weak_directions{};

    /// 判断残差与几何支撑是否同时通过候选验收。
    bool geometrySupported() const { return residual_ok && correspondences_ok; }
};

/// 判断左侧配准结果是否更优：依次比较较小残差、较大最小谱强度和较小条件数。
bool betterScanToMapRegistration(double lhs_residual_score, const ObservabilityQuality& lhs_quality,
                                 double rhs_residual_score,
                                 const ObservabilityQuality& rhs_quality);

/// 配准残差、对应数量与方位覆盖的验收阈值。
struct ObservabilityThresholds {
    int min_correspondences = 1;
    double min_correspondence_ratio = 0.0;
    double max_residual_score = std::numeric_limits<double>::max();
    int min_coverage_sectors = 0;
};

/// 位姿 Hessian 的数值秩、Schur 谱与预条件矩阵。
/// 六维量采用调用方的切空间，分量顺序为 [平移, 旋转]。
struct PoseHessianAnalysis {
    /// 谱分析结果是否有效。
    bool valid = false;
    /// 完整 Hessian 的数值秩。
    int rank = 0;
    /// 对称化后的原始位姿 Hessian。
    Eigen::Matrix<double, 6, 6> hessian = Eigen::Matrix<double, 6, 6>::Zero();
    /// 平移与旋转各自使用一个尺度的对角平方根缩放矩阵。
    Eigen::Matrix<double, 6, 6> diagonal_sqrt = Eigen::Matrix<double, 6, 6>::Zero();
    /// 对角平方根缩放矩阵的逆。
    Eigen::Matrix<double, 6, 6> inverse_diagonal_sqrt = Eigen::Matrix<double, 6, 6>::Zero();
    /// 缩放后的完整 Hessian 特征值。
    Eigen::Matrix<double, 6, 1> eigenvalues = Eigen::Matrix<double, 6, 1>::Zero();
    /// 与 eigenvalues 对应的特征向量，按列排列。
    Eigen::Matrix<double, 6, 6> eigenvectors = Eigen::Matrix<double, 6, 6>::Identity();
    /// 平移 Schur 子空间的特征值。
    Eigen::Vector3d translation_spectrum = Eigen::Vector3d::Zero();
    /// 旋转 Schur 子空间的特征值。
    Eigen::Vector3d rotation_spectrum = Eigen::Vector3d::Zero();
    /// 按坐标轴匹配排列的平移谱基。
    Eigen::Matrix3d translation_basis = Eigen::Matrix3d::Identity();
    /// 按坐标轴匹配排列的旋转谱基。
    Eigen::Matrix3d rotation_basis = Eigen::Matrix3d::Identity();
    /// 按各子空间最大特征值归一化的轴对齐谱强度。
    Eigen::Matrix<double, 6, 1> aligned_strength = Eigen::Matrix<double, 6, 1>::Zero();
    /// 轴对齐谱方向的弱观测标记，顺序为 [平移, 旋转]。
    std::array<bool, 6> weak_directions{};
    /// 平移 Schur 子空间的条件数。
    double translation_condition = std::numeric_limits<double>::infinity();
    /// 旋转 Schur 子空间的条件数。
    double rotation_condition = std::numeric_limits<double>::infinity();
    /// 位姿线性系统使用的分块预条件矩阵。
    Eigen::Matrix<double, 6, 6> preconditioner = Eigen::Matrix<double, 6, 6>::Identity();
    /// 谱分析无效时的原因。
    std::string invalid_reason;
};

/// 分析平移与旋转 Schur 子空间及完整正规方程的数值秩。
PoseHessianAnalysis analyzePoseHessian(const Eigen::Matrix<double, 6, 6>& hessian,
                                       double degeneracy_condition_threshold = 10.0);

/// 位姿线性系统的求解统计。
struct PoseLinearSolveReport {
    /// 预条件共轭梯度迭代次数。
    int iterations = 0;
    /// 按分块尺度归一化的相对线性残差。
    double relative_residual = std::numeric_limits<double>::infinity();
    /// 是否使用数值秩伪逆求解。
    bool used_rank_fallback = false;
};

/// 求解 H * increment = gradient，并按需返回迭代统计。
/// 使用预条件共轭梯度或数值秩伪逆；输入无效或求解失败时返回 false。
bool solveObservablePoseSystem(const PoseHessianAnalysis& analysis,
                               const Eigen::Matrix<double, 6, 1>& gradient,
                               Eigen::Matrix<double, 6, 1>* increment,
                               PoseLinearSolveReport* report = nullptr);

/// 按支撑数量、残差、覆盖和 Hessian 生成质量判定。
ObservabilityQuality evaluateObservabilityQuality(int source_points, int correspondences,
                                                  int coverage_sectors, double residual_score,
                                                  const PoseHessianAnalysis& analysis,
                                                  const ObservabilityThresholds& thresholds);

}  // namespace backend
