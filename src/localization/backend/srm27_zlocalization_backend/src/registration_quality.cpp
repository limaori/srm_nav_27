/**
 * @file registration_quality.cpp
 * @brief 实现扫描匹配共享的质量分析与信息矩阵构造。
 */

#include <algorithm>
#include <cmath>

#include "backend/quality.hpp"

namespace backend {
/// 按残差及几何质量执行确定性的配准结果比较。
bool betterScanToMapRegistration(const double lhs_residual_score,
                                 const ObservabilityQuality& lhs_quality,
                                 const double rhs_residual_score,
                                 const ObservabilityQuality& rhs_quality) {
    if (lhs_residual_score < rhs_residual_score - 1.0e-9) {
        return true;
    }
    if (lhs_residual_score > rhs_residual_score + 1.0e-9) {
        return false;
    }
    if (lhs_quality.min_eigenvalue > rhs_quality.min_eigenvalue + 1.0e-9) {
        return true;
    }
    if (lhs_quality.min_eigenvalue < rhs_quality.min_eigenvalue - 1.0e-9) {
        return false;
    }
    return lhs_quality.condition_number < rhs_quality.condition_number;
}

/// 按支撑数量、残差、覆盖和 Hessian 生成质量判定。
ObservabilityQuality evaluateObservabilityQuality(const int source_points,
                                                  const int correspondences,
                                                  const int coverage_sectors,
                                                  const double residual_score,
                                                  const PoseHessianAnalysis& analysis,
                                                  const ObservabilityThresholds& thresholds) {
    ObservabilityQuality quality;
    quality.source_points = source_points;
    quality.correspondences = correspondences;
    quality.coverage_sectors = coverage_sectors;
    quality.correspondence_ratio = source_points > 0 ? static_cast<double>(correspondences) /
                                                           static_cast<double>(source_points)
                                                     : 0.0;
    quality.rmse = residual_score;

    quality.residual_ok =
        thresholds.max_residual_score <= 0.0 ||
        (std::isfinite(quality.rmse) && quality.rmse <= thresholds.max_residual_score);
    quality.correspondences_ok =
        correspondences >= std::max(0, thresholds.min_correspondences) &&
        quality.correspondence_ratio >= std::max(0.0, thresholds.min_correspondence_ratio) &&
        coverage_sectors >= std::max(0, thresholds.min_coverage_sectors);

    if (analysis.valid) {
        quality.min_eigenvalue = analysis.aligned_strength.minCoeff();
        quality.max_eigenvalue = analysis.aligned_strength.maxCoeff();
        quality.observable_rank = analysis.rank;
        quality.translation_spectrum = analysis.translation_spectrum;
        quality.rotation_spectrum = analysis.rotation_spectrum;
        quality.translation_condition = analysis.translation_condition;
        quality.rotation_condition = analysis.rotation_condition;
        quality.weak_directions = analysis.weak_directions;
        if (quality.min_eigenvalue > 0.0) {
            quality.condition_number = quality.max_eigenvalue / quality.min_eigenvalue;
        }
    }

    // observable 保持初始化/恢复所需的全位姿可观语义；NORMAL 跟踪读取
    // observable_rank，可插入部分秩观测而不把整帧判为不可用。
    quality.observable = analysis.valid && quality.observable_rank == 6;
    return quality;
}

}  // namespace backend
