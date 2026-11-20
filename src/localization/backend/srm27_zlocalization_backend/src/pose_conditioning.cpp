/**
 * @file pose_conditioning.cpp
 * @brief Schur 诊断、谱方向表征和原始正规方程的预条件求解。
 */

#include <algorithm>
#include <cmath>
#include <limits>

#include "backend/quality.hpp"

namespace backend {
namespace {

using Matrix6 = Eigen::Matrix<double, 6, 6>;
using Vector6 = Eigen::Matrix<double, 6, 1>;
constexpr double kRoundoff = 64.0 * std::numeric_limits<double>::epsilon();

/// 对对称三维矩阵计算仅截断机器精度零空间的伪逆。
Eigen::Matrix3d pseudoInverse(const Eigen::Matrix3d& matrix) {
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(matrix);
    const double tolerance = kRoundoff * solver.eigenvalues().maxCoeff();
    Eigen::Vector3d inverse = Eigen::Vector3d::Zero();
    for (int i = 0; i < 3; ++i) {
        if (solver.eigenvalues()(i) > tolerance) {
            inverse(i) = 1.0 / solver.eigenvalues()(i);
        }
    }
    return solver.eigenvectors() * inverse.asDiagonal() * solver.eigenvectors().transpose();
}

/// 计算 Schur 子空间谱、轴对齐强度和谱封顶预条件器。
bool analyzeBlock(const Eigen::Matrix3d& schur, const Eigen::Matrix3d& original, const double limit,
                  Eigen::Vector3d* spectrum, Eigen::Matrix3d* aligned_basis,
                  Eigen::Vector3d* strength, std::array<bool, 6>* weak, const int offset,
                  double* condition, Eigen::Matrix3d* preconditioner) {
    const Eigen::Matrix3d symmetric = 0.5 * (schur + schur.transpose());
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(symmetric);
    if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite()) {
        return false;
    }
    // 消元抵消可能产生舍入量级的小负值；不得修复实质不定的观测。
    const double roundoff = kRoundoff * original.norm();
    if (solver.eigenvalues().minCoeff() < -8.0 * roundoff) {
        return false;
    }
    *spectrum = solver.eigenvalues().cwiseMax(0.0);
    for (int i = 0; i < 3; ++i) {
        if ((*spectrum)(i) <= roundoff) {
            (*spectrum)(i) = 0.0;
        }
    }
    const double largest = spectrum->maxCoeff();
    *condition = spectrum->minCoeff() > 0.0 ? largest / spectrum->minCoeff()
                                            : std::numeric_limits<double>::infinity();

    // 六种排列中选择总轴对齐最强者；只置换/翻转符号，保持原谱子空间。
    std::array<int, 3> order{0, 1, 2}, best = order;
    double best_score = -1.0;
    do {
        double score = 0.0;
        for (int axis = 0; axis < 3; ++axis) {
            score += std::abs(solver.eigenvectors()(axis, order[axis]));
        }
        if (score > best_score) {
            best = order;
            best_score = score;
        }
    } while (std::next_permutation(order.begin(), order.end()));

    Eigen::Vector3d inverse = Eigen::Vector3d::Zero();
    const double floor = std::max(largest / limit, std::max(roundoff, 1.0e-300));
    for (int axis = 0; axis < 3; ++axis) {
        const int mode = best[axis];
        aligned_basis->col(axis) = solver.eigenvectors().col(mode);
        if ((*aligned_basis)(axis, axis) < 0.0) {
            aligned_basis->col(axis) *= -1.0;
        }
        (*strength)(axis) = largest > 0.0 ? (*spectrum)(mode) / largest : 0.0;
        (*weak)[offset + axis] = (*strength)(axis) < 1.0 / limit;
        inverse(axis) = 1.0 / std::max((*spectrum)(mode), floor);
    }
    *preconditioner = *aligned_basis * inverse.asDiagonal() * aligned_basis->transpose();
    return preconditioner->allFinite();
}

}  // namespace

/// 分析平移与旋转 Schur 子空间及完整正规方程的数值秩。
PoseHessianAnalysis analyzePoseHessian(const Matrix6& hessian, const double requested_limit) {
    PoseHessianAnalysis out;
    if (!hessian.allFinite()) {
        out.invalid_reason = "pose Hessian contains a non-finite value";
        return out;
    }
    const double limit =
        std::isfinite(requested_limit) && requested_limit > 1.0 ? requested_limit : 10.0;
    out.hessian = 0.5 * (hessian + hessian.transpose());
    const double magnitude = out.hessian.cwiseAbs().maxCoeff();
    if (magnitude == 0.0) {
        out.valid = true;
        out.weak_directions.fill(true);
        return out;
    }

    // 两个各向同性块尺度仅用于数值运算。逐轴归一化会把噪声主导轴放大为强轴。
    for (int offset : {0, 3}) {
        const double scale =
            std::max(out.hessian.block<3, 3>(offset, offset).norm(), magnitude * kRoundoff);
        const double root = std::sqrt(scale);
        out.diagonal_sqrt.block<3, 3>(offset, offset) = root * Eigen::Matrix3d::Identity();
        out.inverse_diagonal_sqrt.block<3, 3>(offset, offset) =
            (1.0 / root) * Eigen::Matrix3d::Identity();
    }
    const Matrix6 balanced = out.inverse_diagonal_sqrt * out.hessian * out.inverse_diagonal_sqrt;
    Eigen::SelfAdjointEigenSolver<Matrix6> full(balanced);
    if (full.info() != Eigen::Success || !full.eigenvalues().allFinite()) {
        out.invalid_reason = "pose Hessian eigendecomposition failed";
        return out;
    }
    const double tolerance = kRoundoff * full.eigenvalues().cwiseAbs().maxCoeff();
    if (full.eigenvalues().minCoeff() < -8.0 * tolerance) {
        out.invalid_reason = "pose Hessian is materially indefinite";
        return out;
    }
    out.eigenvalues = full.eigenvalues();
    out.eigenvectors = full.eigenvectors();
    for (int i = 0; i < 6; ++i) {
        if (out.eigenvalues(i) > tolerance) {
            ++out.rank;
        } else {
            out.eigenvalues(i) = 0.0;
        }
    }

    const Eigen::Matrix3d tt = out.hessian.topLeftCorner<3, 3>();
    const Eigen::Matrix3d rr = out.hessian.bottomRightCorner<3, 3>();
    const Eigen::Matrix3d tr = out.hessian.topRightCorner<3, 3>();
    const Eigen::Matrix3d st = tt - tr * pseudoInverse(rr) * tr.transpose();
    const Eigen::Matrix3d sr = rr - tr.transpose() * pseudoInverse(tt) * tr;
    Eigen::Matrix3d pt, pr;
    Eigen::Vector3d strength_t, strength_r;
    if (!analyzeBlock(st, tt, limit, &out.translation_spectrum, &out.translation_basis, &strength_t,
                      &out.weak_directions, 0, &out.translation_condition, &pt) ||
        !analyzeBlock(sr, rr, limit, &out.rotation_spectrum, &out.rotation_basis, &strength_r,
                      &out.weak_directions, 3, &out.rotation_condition, &pr)) {
        out.invalid_reason = "Schur complement is invalid or materially indefinite";
        return out;
    }
    out.aligned_strength << strength_t, strength_r;
    out.preconditioner.setZero();
    out.preconditioner.topLeftCorner<3, 3>() = pt;
    out.preconditioner.bottomRightCorner<3, 3>() = pr;
    out.valid = true;
    return out;
}

/// 使用预条件迭代或数值秩伪逆求解可观正规方程。
bool solveObservablePoseSystem(const PoseHessianAnalysis& analysis, const Vector6& gradient,
                               Vector6* increment, PoseLinearSolveReport* diagnostics) {
    PoseLinearSolveReport report;
    if (diagnostics != nullptr) {
        *diagnostics = report;
    }
    if (increment == nullptr || !analysis.valid || analysis.rank == 0 || !gradient.allFinite()) {
        return false;
    }
    increment->setZero();
    const double rhs_norm = (analysis.inverse_diagonal_sqrt * gradient).norm();
    if (rhs_norm == 0.0) {
        report.relative_residual = 0.0;
        if (diagnostics != nullptr)
            *diagnostics = report;
        return true;
    }
    /// 在原正规方程中按平衡尺度计算相对残差。
    const auto relativeResidual = [&](const Vector6& value) {
        return (analysis.inverse_diagonal_sqrt * (gradient - analysis.hessian * value)).norm() /
               rhs_norm;
    };
    constexpr double kTolerance = 1.0e-10;
    if (analysis.rank == 6) {
        Vector6 residual = gradient;
        Vector6 z = analysis.preconditioner * residual;
        Vector6 direction = z;
        double rz = residual.dot(z);
        for (int i = 0; i < 24; ++i) {
            const Vector6 hd = analysis.hessian * direction;
            const double denominator = direction.dot(hd);
            if (!std::isfinite(denominator) || denominator <= 0.0 || !std::isfinite(rz) ||
                rz <= 0.0)
                break;
            const double alpha = rz / denominator;
            *increment += alpha * direction;
            residual -= alpha * hd;
            report.iterations = i + 1;
            report.relative_residual = relativeResidual(*increment);
            if (increment->allFinite() && report.relative_residual <= kTolerance) {
                if (diagnostics != nullptr)
                    *diagnostics = report;
                return true;
            }
            z = analysis.preconditioner * residual;
            const double next = residual.dot(z);
            if (!std::isfinite(next))
                break;
            direction = z + (next / rz) * direction;
            rz = next;
        }
    }

    // 数值秩伪逆保留可解部分；阈值只消除机器精度零空间，不按弱观测标签裁剪。
    Vector6 inverse = Vector6::Zero();
    for (int i = 0; i < 6; ++i) {
        if (analysis.eigenvalues(i) > 0.0)
            inverse(i) = 1.0 / analysis.eigenvalues(i);
    }
    *increment = analysis.inverse_diagonal_sqrt * analysis.eigenvectors * inverse.asDiagonal() *
                 analysis.eigenvectors.transpose() * analysis.inverse_diagonal_sqrt * gradient;
    report.used_rank_fallback = true;
    report.relative_residual = relativeResidual(*increment);
    if (diagnostics != nullptr)
        *diagnostics = report;
    return increment->allFinite() && report.relative_residual <= 1.0e-7;
}

}  // namespace backend
