/**
 * @file registration_solver.cpp
 * @brief 构造点面对应、求解可观增量并生成测量信息。
 */

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <vector>

#include "backend/geometry.hpp"
#include "backend/math.hpp"
#include "backend/registration.hpp"
#include "backend/sampling.hpp"
#include "registration.hpp"

namespace backend {
namespace {

/// 对 [rho, phi] 顺序的切向量计算 SE(3) 指数映射；平移采用 SO(3) 左 Jacobian，
/// 以正确处理粗恢复步骤中的非小量旋转。
Eigen::Matrix4d se3Exp(const Eigen::Matrix<double, 6, 1>& tangent) {
    const Eigen::Vector3d rho = tangent.head<3>();
    const Eigen::Vector3d phi = tangent.tail<3>();
    const double theta = phi.norm();
    const Eigen::Matrix3d omega = skewMatrix(phi);
    const Eigen::Matrix3d omega_squared = omega * omega;

    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d left_jacobian = Eigen::Matrix3d::Identity();
    if (theta < 1.0e-8) {
        rotation += omega + 0.5 * omega_squared;
        left_jacobian += 0.5 * omega + (1.0 / 6.0) * omega_squared;
    } else {
        const double theta_squared = theta * theta;
        rotation += (std::sin(theta) / theta) * omega +
                    ((1.0 - std::cos(theta)) / theta_squared) * omega_squared;
        left_jacobian += ((1.0 - std::cos(theta)) / theta_squared) * omega +
                         ((theta - std::sin(theta)) / (theta_squared * theta)) * omega_squared;
    }

    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block<3, 3>(0, 0) = rotation;
    transform.block<3, 1>(0, 3) = left_jacobian * rho;
    return transform;
}

/// 计算 Huber 点残差权重，非正阈值返回单位权重。
double robustWeight(const double residual, const double huber_delta) {
    if (huber_delta <= 0.0) {
        return 1.0;
    }
    const double absolute = std::abs(residual);
    return absolute <= huber_delta ? 1.0 : huber_delta / absolute;
}

/// 将水平点方向映射到有界的等角扇区索引。
int sectorIndex(const Eigen::Vector3d& point, const int sectors) {
    const double angle = std::atan2(point.y(), point.x());
    const int index =
        static_cast<int>(std::floor((normalizeAngle(angle) + kPi) / (2.0 * kPi) * sectors));
    return std::clamp(index, 0, sectors - 1);
}

}  // namespace

/// 按目标平面索引和距离门限建立当前位姿的点面对应。
std::vector<Registration::PlaneCorrespondence> Registration::buildCorrespondences(
    const PreparedLevel& level, const Cloud& source,
    const Eigen::Matrix4d& source_to_target) const {
    std::vector<PlaneCorrespondence> correspondences;
    correspondences.reserve(source.size());
    const Eigen::Matrix3d rotation = source_to_target.block<3, 3>(0, 0);
    const Eigen::Vector3d translation = source_to_target.block<3, 1>(0, 3);
    level.tree.query(
        source.size(), 1,
        /// 将源点转换到目标坐标系后查询近邻。
        [&](std::size_t i) -> Eigen::Vector3d {
            const auto& p = source[i];
            return rotation * Eigen::Vector3d(p.x, p.y, p.z) + translation;
        },
        /// 验收对应平面与残差并保存原始源点和目标系 Jacobian 支撑。
        [&](std::size_t i, const Eigen::Vector3d& target_point, const std::size_t* indices,
            const float* distances, std::size_t) {
            if (!pointIsFinite(source[i]))
                return;
            if (distances[0] >
                level.max_correspondence_distance * level.max_correspondence_distance)
                return;
            const auto& plane = level.planes[indices[0]];
            if (!plane.valid)
                return;
            const double residual = plane.normal.dot(target_point - plane.point);
            if (!std::isfinite(residual))
                return;
            const auto& point = source[i];
            correspondences.push_back(
                {Eigen::Vector3d(point.x, point.y, point.z), target_point, plane.normal, residual});
        });
    return correspondences;
}

/// 累加鲁棒正规方程并求解可观位姿增量。
Eigen::Matrix4d Registration::solveOnce(const std::vector<PlaneCorrespondence>& correspondences,
                                        const double huber_delta,
                                        const double max_correspondence_distance,
                                        PoseHessianAnalysis* analysis, bool* solved,
                                        PoseLinearSolveReport* report) const {
    Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 1> gradient = Eigen::Matrix<double, 6, 1>::Zero();
    for (const PlaneCorrespondence& correspondence : correspondences) {
        Eigen::Matrix<double, 1, 6> jacobian;
        jacobian.block<1, 3>(0, 0) = correspondence.normal.transpose();
        // 左扰动作用于目标坐标系：d(Rp+t)/d(phi) = -[Rp+t]x。
        jacobian.block<1, 3>(0, 3) =
            -correspondence.normal.transpose() * skewMatrix(correspondence.target);
        const double weight = robustWeight(correspondence.residual, huber_delta);
        information.noalias() += weight * jacobian.transpose() * jacobian;
        gradient.noalias() += -weight * jacobian.transpose() * correspondence.residual;
    }
    information = 0.5 * (information + information.transpose());

    const PoseHessianAnalysis spectral =
        analyzePoseHessian(information, config_.degeneracy_condition_threshold);
    if (analysis != nullptr) {
        *analysis = spectral;
    }
    Eigen::Matrix<double, 6, 1> increment = Eigen::Matrix<double, 6, 1>::Zero();
    if (!solveObservablePoseSystem(spectral, gradient, &increment, report)) {
        if (solved != nullptr) {
            *solved = false;
        }
        return Eigen::Matrix4d::Identity();
    }

    const double translation_limit = std::max(0.05, max_correspondence_distance);
    const bool coarse = max_correspondence_distance > config_.max_correspondence_distance * 1.5;
    const double rotation_limit = degreesToRadians(coarse ? 16.0 : 8.0);
    const double translation_norm = increment.head<3>().norm();
    const double rotation_norm = increment.tail<3>().norm();
    if (translation_norm > translation_limit) {
        increment.head<3>() *= translation_limit / translation_norm;
    }
    if (rotation_norm > rotation_limit) {
        increment.tail<3>() *= rotation_limit / rotation_norm;
    }

    if (solved != nullptr) {
        *solved = true;
    }
    return se3Exp(increment);
}

/// 迭代单层点面配准，保留最终对应和正规方程诊断。
Registration::LevelRun Registration::runLevel(
    const PreparedLevel& level, const Cloud& source, const Eigen::Matrix4d& initial_guess,
    const int max_iterations, std::vector<PlaneCorrespondence> correspondences) const {
    LevelRun run;
    run.estimate = initial_guess;
    for (int iteration = 0; iteration < std::max(1, max_iterations); ++iteration) {
        if (correspondences.empty()) {
            correspondences = buildCorrespondences(level, source, run.estimate);
        }
        if (static_cast<int>(correspondences.size()) < level.min_correspondences) {
            break;
        }

        bool solved = false;
        const Eigen::Matrix4d delta =
            solveOnce(correspondences, level.huber_delta, level.max_correspondence_distance,
                      nullptr, &solved);
        if (!solved) {
            break;
        }
        run.estimate = delta * run.estimate;
        ++run.iterations;
        const double translation = delta.block<3, 1>(0, 3).norm();
        const double rotation = rotationAngleFromMatrix(delta);
        correspondences.clear();
        if (translation < 1.0e-4 && rotation < degreesToRadians(0.02)) {
            break;
        }
    }

    run.correspondences = buildCorrespondences(level, source, run.estimate);
    bool final_step_solved = false;
    const Eigen::Matrix4d final_step =
        solveOnce(run.correspondences, level.huber_delta, level.max_correspondence_distance,
                  &run.hessian_analysis, &final_step_solved, &run.linear_solve);
    run.converged = false;
    if (final_step_solved) {
        // 达到最大迭代数不代表收敛；在重建的最终对应集上用略宽阈值确认数值稳定。
        // 早期小步长不作依据，因为重新关联对应点后最终线性化仍可能明显移动。
        constexpr double kAcceptedFinalTranslationStep = 5.0e-3;
        const double kAcceptedFinalRotationStep = degreesToRadians(0.1);
        run.converged = final_step.block<3, 1>(0, 3).norm() <= kAcceptedFinalTranslationStep &&
                        rotationAngleFromMatrix(final_step) <= kAcceptedFinalRotationStep;
    }
    return run;
}

/// 结合残差、覆盖和谱分析计算几何及可观性质量。
ObservabilityQuality Registration::evaluateQuality(
    const std::vector<PlaneCorrespondence>& correspondences, const int source_points,
    const PoseHessianAnalysis& analysis) const {
    constexpr int kSectors = 36;
    std::array<int, kSectors> sectors{};
    double sum_squared = 0.0;
    for (const PlaneCorrespondence& correspondence : correspondences) {
        sum_squared += correspondence.residual * correspondence.residual;
        sectors[sectorIndex(correspondence.source, kSectors)] = 1;
    }
    const int coverage = std::accumulate(sectors.begin(), sectors.end(), 0);
    const double rmse = correspondences.empty()
                            ? std::numeric_limits<double>::max()
                            : std::sqrt(sum_squared / static_cast<double>(correspondences.size()));

    ObservabilityThresholds thresholds;
    thresholds.min_correspondences = config_.min_correspondences;
    thresholds.min_correspondence_ratio = config_.min_correspondence_ratio;
    thresholds.max_residual_score = config_.max_rmse;
    thresholds.min_coverage_sectors = config_.min_coverage_sectors;
    return evaluateObservabilityQuality(source_points, static_cast<int>(correspondences.size()),
                                        coverage, rmse, analysis, thresholds);
}

/// 按残差尺度和独立信息预算构造原始位姿测量信息。
PoseMeasurementInformation Registration::buildMeasurementInformation(
    const PoseHessianAnalysis& analysis, const double rmse, const int correspondences,
    const double information_scale) const {
    PoseMeasurementInformation result;
    /// 标记位姿信息不可用并保存失败原因。
    const auto reject = [&](const std::string& reason) {
        result.invalid_reason = reason;
        return result;
    };
    if (!analysis.valid || !std::isfinite(rmse) || rmse < 0.0 ||
        !std::isfinite(information_scale) || information_scale <= 0.0 ||
        correspondences < config_.min_correspondences || analysis.rank <= 0) {
        if (!analysis.valid && !analysis.invalid_reason.empty()) {
            return reject(analysis.invalid_reason);
        }
        return reject(
            "point-to-plane information has invalid scaling, "
            "non-finite values, or insufficient residual support");
    }

    const double count = static_cast<double>(correspondences);
    const double degrees_of_freedom = std::max(1.0, count - static_cast<double>(analysis.rank));
    result.residual_variance = std::max(config_.residual_std_floor * config_.residual_std_floor,
                                        rmse * rmse * count / degrees_of_freedom);
    if (!std::isfinite(result.residual_variance) || result.residual_variance <= 0.0) {
        return reject("invalid residual variance");
    }

    result.sqrt_information.resize(static_cast<Eigen::Index>(analysis.rank), 6);
    const double inverse_residual_std = std::sqrt(information_scale / result.residual_variance);
    const int first_retained = 6 - analysis.rank;
    for (int row = 0; row < analysis.rank; ++row) {
        const int eigen_index = first_retained + row;
        result.sqrt_information.row(row) =
            inverse_residual_std * std::sqrt(analysis.eigenvalues(eigen_index)) *
            analysis.eigenvectors.col(eigen_index).transpose() * analysis.diagonal_sqrt;
    }
    result.rank = analysis.rank;
    result.valid = validatePoseSqrtInformation(result.sqrt_information, &result.invalid_reason);
    return result;
}

/// 按对应原序累加残差平方和，空集合返回无穷大以表示没有初值支撑。
double Registration::residualRmse(const std::vector<PlaneCorrespondence>& correspondences) {
    double sum_squared = 0.0;
    for (const PlaneCorrespondence& correspondence : correspondences) {
        sum_squared += correspondence.residual * correspondence.residual;
    }
    return correspondences.empty()
               ? std::numeric_limits<double>::infinity()
               : std::sqrt(sum_squared / static_cast<double>(correspondences.size()));
}

/// 在局部位姿切空间施加共模协方差，并仅为满秩测量生成完整协方差。
void Registration::boundMeasurementInformation(RegistrationResult& result) const {
    if (result.measurement_information.valid &&
        (config_.measurement_translation_std_floor > 0.0 ||
         config_.measurement_rotation_std_floor_rad > 0.0)) {
        PoseCovariance6d common_mode_covariance = PoseCovariance6d::Zero();
        common_mode_covariance.diagonal().head<3>().setConstant(
            config_.measurement_translation_std_floor * config_.measurement_translation_std_floor);
        common_mode_covariance.diagonal().tail<3>().setConstant(
            config_.measurement_rotation_std_floor_rad *
            config_.measurement_rotation_std_floor_rad);

        const PoseSqrtInformation local_information = leftToRightPoseSqrtInformation(
            result.source_to_target, result.measurement_information.sqrt_information);
        const auto bounded_local_information =
            addPoseMeasurementCovariance(local_information, common_mode_covariance);
        if (!bounded_local_information.has_value()) {
            result.measurement_information.valid = false;
            result.measurement_information.invalid_reason =
                "failed to apply scan-to-map common-mode covariance";
        } else {
            result.measurement_information.sqrt_information =
                *bounded_local_information * poseAdjoint(result.source_to_target.inverse());
            result.measurement_information.valid =
                validatePoseSqrtInformation(result.measurement_information.sqrt_information,
                                            &result.measurement_information.invalid_reason);
        }
    }
    if (result.measurement_information.valid) {
        const auto covariance =
            poseCovarianceFromSqrtInformation(result.measurement_information.sqrt_information);
        if (covariance.has_value()) {
            result.covariance = *covariance;
            result.covariance_valid = true;
        }
    }
}

/// 执行细层与按需粗层配准，生成位姿似然和质量诊断。
RegistrationResult Registration::align(const Cloud& source, const Eigen::Matrix4d& initial_guess,
                                       const double information_scale,
                                       const Cloud::ConstPtr& shared_coarse_source) {
    RegistrationResult result;
    result.model_validity_rmse_limit =
        config_.max_rmse > 0.0
            ? std::max(2.0 * config_.max_rmse, 0.5 * config_.max_correspondence_distance)
            : config_.max_correspondence_distance;
    if (source.empty() || !initial_guess.allFinite() || !fine_level_ || !fine_level_->target ||
        fine_level_->target->empty()) {
        return result;
    }
    const Cloud::ConstPtr finite_source =
        source.is_dense ? nonOwningCloudPtr(source) : Cloud::ConstPtr(finiteCloudCopy(source));
    if (!finite_source || finite_source->empty()) {
        return result;
    }
    const Cloud& clean_source = *finite_source;

    std::vector<PlaneCorrespondence> fine_initial =
        buildCorrespondences(*fine_level_, clean_source, initial_guess);
    const double initial_rmse = residualRmse(fine_initial);
    const int required_support =
        std::max(config_.min_correspondences,
                 static_cast<int>(std::ceil(config_.coarse_trigger_min_correspondence_ratio *
                                            clean_source.size())));
    const double residual_trigger = result.model_validity_rmse_limit;
    const int initial_support = static_cast<int>(fine_initial.size());
    const bool needs_coarse = initial_support < required_support || initial_rmse > residual_trigger;

    Eigen::Matrix4d fine_seed = initial_guess;
    int total_iterations = 0;
    if (needs_coarse && ensureCoarseLevel()) {
        const Cloud::ConstPtr coarse_source =
            shared_coarse_source ? shared_coarse_source
                                 : voxelize(clean_source, 2.0 * config_.voxel_resolution);
        if (coarse_source && !coarse_source->empty()) {
            result.used_coarse_level = true;
            const LevelRun coarse = runLevel(*coarse_level_, *coarse_source, initial_guess,
                                             std::max(2, (config_.max_iterations + 1) / 2), {});
            total_iterations += coarse.iterations;
            if (coarse.iterations > 0 && coarse.estimate.allFinite()) {
                std::vector<PlaneCorrespondence> candidate_fine =
                    buildCorrespondences(*fine_level_, clean_source, coarse.estimate);
                const double candidate_rmse = residualRmse(candidate_fine);
                const bool enough_candidate_support =
                    static_cast<int>(candidate_fine.size()) >= config_.min_correspondences;
                const bool improves_support =
                    static_cast<int>(candidate_fine.size()) > initial_support;
                const bool improves_residual =
                    std::isfinite(candidate_rmse) && candidate_rmse + 1.0e-9 < initial_rmse;
                if (enough_candidate_support && (improves_support || improves_residual)) {
                    fine_seed = coarse.estimate;
                    fine_initial = std::move(candidate_fine);
                }
            }
        }
    }

    LevelRun fine = runLevel(*fine_level_, clean_source, fine_seed, config_.max_iterations,
                             std::move(fine_initial));
    total_iterations += fine.iterations;
    result.iterations = total_iterations;
    result.source_to_target = fine.estimate;
    result.quality = evaluateQuality(fine.correspondences, static_cast<int>(clean_source.size()),
                                     fine.hessian_analysis);
    result.measurement_information = buildMeasurementInformation(
        fine.hessian_analysis, result.quality.rmse, static_cast<int>(fine.correspondences.size()),
        information_scale);
    boundMeasurementInformation(result);
    result.hessian_analysis = fine.hessian_analysis;
    result.linear_solve = fine.linear_solve;
    result.converged = fine.converged;
    return result;
}

}  // namespace backend
