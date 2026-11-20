/**
 * @file alignment.hpp
 * @brief 提供 UWB 对齐实现内部共享的确定性二维数学工具。
 */
#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace backend::uwb_alignment_detail {

inline constexpr double kFiniteEpsilon = 1.0e-12;
inline constexpr std::size_t kMaxPairHypotheses = 1024U;
inline constexpr double kGaussianMedianVarianceFactor = 1.57079632679489661923;  // pi / 2

/// 由弧度偏航角构造二维旋转矩阵。
inline Eigen::Matrix2d yawRotation(const double yaw) {
    Eigen::Matrix2d rotation;
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    rotation << c, -s, s, c;
    return rotation;
}

/// 计算标量样本中位数，偶数样本取中间两项均值。
inline double median(std::vector<double> values) {
    if (values.empty()) {
        return 0.0;
    }
    const std::size_t middle = values.size() / 2U;
    std::nth_element(values.begin(), values.begin() + middle, values.end());
    const double upper = values[middle];
    if (values.size() % 2U != 0U) {
        return upper;
    }
    std::nth_element(values.begin(), values.begin() + middle - 1U, values.begin() + middle);
    return 0.5 * (values[middle - 1U] + upper);
}

}  // namespace backend::uwb_alignment_detail
