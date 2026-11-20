/**
 * @file uwb_alignment_solver.cpp
 * @brief 从 UWB 对齐共识集计算协方差、可观性和连续稳定性。
 */

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <numeric>
#include <utility>

#include "alignment.hpp"
#include "backend/math.hpp"
#include "backend/uwb_alignment.hpp"

namespace backend {

using uwb_alignment_detail::kFiniteEpsilon;
using uwb_alignment_detail::kGaussianMedianVarianceFactor;
using uwb_alignment_detail::kMaxPairHypotheses;
using uwb_alignment_detail::median;
using uwb_alignment_detail::yawRotation;

/// 从样本共识计算对齐及协方差，更新可观性和连续稳定性。
std::optional<UwbAlignmentEstimate> UwbAlignmentBootstrapper::estimate() {
    const auto best = static_cast<int>(pairs_.size()) >= config_.min_pairs
                          ? selectConsensus(buildHypotheses())
                          : std::nullopt;
    if (!best) {
        consecutive_stable_solutions_ = 0;
        previous_observable_estimate_.reset();
        return std::nullopt;
    }

    UwbAlignmentEstimate estimate = makeEstimateSummary(*best);
    if (!populateEstimateCovariance(*best, &estimate)) {
        consecutive_stable_solutions_ = 0;
        previous_observable_estimate_.reset();
        return estimate;
    }
    if (!estimate.observable) {
        consecutive_stable_solutions_ = 0;
        previous_observable_estimate_.reset();
        return estimate;
    }
    updateEstimateStability(&estimate);
    return estimate;
}

/// 按初值、全样本和有限点对生成确定性的对齐假设。
std::vector<UwbAlignmentBootstrapper::Candidate> UwbAlignmentBootstrapper::buildHypotheses() const {
    std::vector<Candidate> hypotheses;
    hypotheses.reserve(2U + kMaxPairHypotheses);
    if (initial_guess_.has_value()) {
        Candidate seeded;
        seeded.yaw = yawFromMatrix(*initial_guess_);
        seeded.translation = initial_guess_->block<2, 1>(0, 3);
        seeded.translation_z = (*initial_guess_)(2, 3);
        hypotheses.push_back(seeded);
    }

    std::vector<std::size_t> all_indices(pairs_.size());
    std::iota(all_indices.begin(), all_indices.end(), 0U);
    Candidate all_fit;
    if (fitIndices(all_indices, &all_fit)) {
        hypotheses.push_back(all_fit);
    }
    appendPairHypotheses(&hypotheses);
    return hypotheses;
}

/// 追加满足基线门限的确定性点对对齐假设。
void UwbAlignmentBootstrapper::appendPairHypotheses(std::vector<Candidate>* hypotheses) const {
    if (hypotheses == nullptr) {
        return;
    }
    const double hypothesis_baseline = std::max(0.25, 0.25 * config_.min_baseline);
    std::vector<std::pair<std::size_t, std::size_t>> eligible;
    eligible.reserve(pairs_.size() * (pairs_.size() - 1U) / 2U);
    for (std::size_t lhs = 0; lhs < pairs_.size(); ++lhs) {
        for (std::size_t rhs = lhs + 1; rhs < pairs_.size(); ++rhs) {
            const double odom_distance =
                (pairs_[rhs].odom.head<2>() - pairs_[lhs].odom.head<2>()).norm();
            const double map_distance =
                (pairs_[rhs].map.head<2>() - pairs_[lhs].map.head<2>()).norm();
            if (odom_distance < hypothesis_baseline || map_distance < hypothesis_baseline ||
                std::abs(odom_distance - map_distance) >
                    std::max(2.0 * config_.inlier_threshold,
                             0.35 * std::max(odom_distance, map_distance)))
                continue;
            eligible.emplace_back(lhs, rhs);
        }
    }
    // 先保留原合格序列的同一等距索引，再计算昂贵的方向和完整假设。
    const auto count = std::min(eligible.size(), kMaxPairHypotheses);
    for (std::size_t sample = 0; sample < count; ++sample) {
        const auto index = eligible.size() <= kMaxPairHypotheses
                               ? sample
                               : sample * (eligible.size() - 1) / (kMaxPairHypotheses - 1);
        const auto [lhs, rhs] = eligible[index];
        const Eigen::Vector2d odom_delta = pairs_[rhs].odom.head<2>() - pairs_[lhs].odom.head<2>();
        const Eigen::Vector2d map_delta = pairs_[rhs].map.head<2>() - pairs_[lhs].map.head<2>();
        Candidate pair_fit;
        pair_fit.yaw = normalizeAngle(std::atan2(map_delta.y(), map_delta.x()) -
                                      std::atan2(odom_delta.y(), odom_delta.x()));
        const Eigen::Matrix2d rotation = yawRotation(pair_fit.yaw);
        pair_fit.translation =
            0.5 * ((pairs_[lhs].map.head<2>() - rotation * pairs_[lhs].odom.head<2>()) +
                   (pairs_[rhs].map.head<2>() - rotation * pairs_[rhs].odom.head<2>()));
        if (config_.use_z) {
            pair_fit.translation_z = 0.5 * ((pairs_[lhs].map.z() - pairs_[lhs].odom.z()) +
                                            (pairs_[rhs].map.z() - pairs_[rhs].odom.z()));
        }
        hypotheses->push_back(std::move(pair_fit));
    }
}

/// 比较假设内点支持并选择最优鲁棒对齐结果。
std::optional<UwbAlignmentBootstrapper::Candidate> UwbAlignmentBootstrapper::selectConsensus(
    const std::vector<Candidate>& hypotheses) const {
    Candidate best;
    bool found = false;
    for (Candidate candidate : hypotheses) {
        classifyInliers(&candidate);
        if (!found || candidate.inliers.size() > best.inliers.size() ||
            (candidate.inliers.size() == best.inliers.size() && candidate.rmse < best.rmse)) {
            best = std::move(candidate);
            found = true;
        }
    }
    if (!found || static_cast<int>(best.inliers.size()) < config_.min_pairs) {
        return std::nullopt;
    }

    // 在共识集上做有限次精确拟合，结果与线程调度无关。
    for (int iteration = 0; iteration < 3; ++iteration) {
        Candidate refined = best;
        if (!fitIndices(best.inliers, &refined)) {
            break;
        }
        classifyInliers(&refined);
        if (refined.inliers == best.inliers) {
            best = std::move(refined);
            break;
        }
        best = std::move(refined);
    }
    return best;
}

/// 汇总对齐变换、内点数量、残差和空间基线。
UwbAlignmentEstimate UwbAlignmentBootstrapper::makeEstimateSummary(const Candidate& best) const {
    UwbAlignmentEstimate estimate;
    estimate.pairs = static_cast<int>(pairs_.size());
    estimate.inliers = static_cast<int>(best.inliers.size());
    estimate.rmse = best.rmse;
    estimate.latest_timestamp = pairs_.back().timestamp;
    estimate.odom_baseline = baseline(pairs_, best.inliers, false);
    estimate.map_baseline = baseline(pairs_, best.inliers, true);
    estimate.inlier_timestamps.reserve(best.inliers.size());
    for (const std::size_t index : best.inliers) {
        estimate.inlier_timestamps.push_back(pairs_[index].timestamp);
    }
    return estimate;
}

/// 计算平面对齐及可选高度的联合协方差和可观性。
bool UwbAlignmentBootstrapper::populateEstimateCovariance(const Candidate& best,
                                                          UwbAlignmentEstimate* estimate) const {
    if (estimate == nullptr) {
        return false;
    }

    Eigen::Matrix3d information = Eigen::Matrix3d::Zero();
    double squared_error = 0.0;
    const Eigen::Matrix2d rotation = yawRotation(best.yaw);
    std::vector<double> z_offsets;
    z_offsets.reserve(best.inliers.size());
    for (const std::size_t index : best.inliers) {
        const Eigen::Vector2d odom = pairs_[index].odom.head<2>();
        const Eigen::Vector2d predicted = rotation * odom + best.translation;
        const Eigen::Vector2d residual = predicted - pairs_[index].map.head<2>();
        squared_error += residual.squaredNorm();

        Eigen::Matrix<double, 2, 3> jacobian;
        const double c = std::cos(best.yaw);
        const double s = std::sin(best.yaw);
        jacobian << 1.0, 0.0, -s * odom.x() - c * odom.y(), 0.0, 1.0, c * odom.x() - s * odom.y();
        information += jacobian.transpose() * jacobian;
        if (config_.use_z) {
            z_offsets.push_back(pairs_[index].map.z() - pairs_[index].odom.z());
        }
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(information);
    if (solver.info() != Eigen::Success || solver.eigenvalues().minCoeff() <= kFiniteEpsilon) {
        return false;
    }
    const double residual_variance = std::max(
        config_.position_std_xy * config_.position_std_xy,
        squared_error /
            static_cast<double>(std::max(1, 2 * static_cast<int>(best.inliers.size()) - 3)));
    const Eigen::Matrix3d se2_covariance = information.inverse() * residual_variance;
    estimate->yaw_std_rad = std::sqrt(std::max(0.0, se2_covariance(2, 2)));

    const double z_translation = config_.use_z
                                     ? median(z_offsets)
                                     : (initial_guess_.has_value() ? (*initial_guess_)(2, 3) : 0.0);
    estimate->T_map_odom = yawGuess(
        best.yaw, Eigen::Vector3d(best.translation.x(), best.translation.y(), z_translation));
    estimate->covariance.setZero();
    estimate->covariance(0, 0) = se2_covariance(0, 0);
    estimate->covariance(0, 1) = se2_covariance(0, 1);
    estimate->covariance(1, 0) = se2_covariance(1, 0);
    estimate->covariance(1, 1) = se2_covariance(1, 1);
    estimate->covariance(0, 5) = se2_covariance(0, 2);
    estimate->covariance(5, 0) = se2_covariance(2, 0);
    estimate->covariance(1, 5) = se2_covariance(1, 2);
    estimate->covariance(5, 1) = se2_covariance(2, 1);
    estimate->covariance(5, 5) = se2_covariance(2, 2);
    estimate->covariance(2, 2) = config_.position_std_z * config_.position_std_z;
    if (config_.use_z) {
        std::vector<double> z_absolute_deviations;
        z_absolute_deviations.reserve(z_offsets.size());
        for (const double offset : z_offsets) {
            z_absolute_deviations.push_back(std::abs(offset - z_translation));
        }
        constexpr double kMedianAbsoluteDeviationToGaussianSigma = 1.482602218505602;
        const double robust_z_std =
            std::max(config_.position_std_z, kMedianAbsoluteDeviationToGaussianSigma *
                                                 median(std::move(z_absolute_deviations)));
        estimate->covariance(2, 2) =
            robust_z_std * robust_z_std * kGaussianMedianVarianceFactor /
            static_cast<double>(std::max<std::size_t>(1U, best.inliers.size()));
    }
    const double gravity_variance =
        config_.gravity_alignment_std_rad * config_.gravity_alignment_std_rad;
    estimate->covariance(3, 3) = gravity_variance;
    estimate->covariance(4, 4) = gravity_variance;

    estimate->observable = static_cast<int>(best.inliers.size()) >= config_.min_pairs &&
                           estimate->odom_baseline >= config_.min_baseline &&
                           estimate->map_baseline >= config_.min_baseline &&
                           std::isfinite(estimate->rmse) && estimate->rmse <= config_.max_rmse &&
                           std::isfinite(estimate->yaw_std_rad) &&
                           estimate->yaw_std_rad <= config_.max_yaw_std_rad;
    return true;
}

/// 按相邻估计的位移与偏航变化更新稳定性判定。
void UwbAlignmentBootstrapper::updateEstimateStability(UwbAlignmentEstimate* estimate) {
    if (estimate == nullptr) {
        return;
    }
    if (previous_observable_estimate_.has_value()) {
        const Eigen::Matrix4d delta =
            previous_observable_estimate_->inverse() * estimate->T_map_odom;
        const double translation_delta =
            config_.use_z ? delta.block<3, 1>(0, 3).norm() : delta.block<2, 1>(0, 3).norm();
        const double yaw_delta = std::abs(yawFromMatrix(delta));
        if (translation_delta <= config_.stable_translation_delta &&
            yaw_delta <= config_.stable_yaw_delta_rad) {
            ++consecutive_stable_solutions_;
        } else {
            consecutive_stable_solutions_ = 1;
        }
    } else {
        consecutive_stable_solutions_ = 1;
    }
    previous_observable_estimate_ = estimate->T_map_odom;
    estimate->consecutive_stable_solutions = consecutive_stable_solutions_;
    estimate->stable = consecutive_stable_solutions_ >= config_.stable_solutions;
}

}  // namespace backend
