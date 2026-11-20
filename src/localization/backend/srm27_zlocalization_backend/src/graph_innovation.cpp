/**
 * @file graph_innovation.cpp
 * @brief 计算联合创新统计和增量图联合边缘协方差。
 */

#include <gtsam/linear/GaussianFactorGraph.h>

#include <algorithm>
#include <boost/math/distributions/chi_squared.hpp>
#include <cmath>
#include <map>
#include <set>

#include "backend/pose_graph.hpp"

namespace backend {
namespace {

/// 按预测协方差与测量协方差计算一项创新的 NIS 和卡方尾概率。
InnovationAssessment assessInnovation(const Eigen::VectorXd& innovation,
                                      Eigen::MatrixXd covariance) {
    InnovationAssessment assessment;
    /// 构造保留失败原因的无效创新评估。
    const auto reject = [&](const std::string& reason) {
        assessment.invalid_reason = reason;
        return assessment;
    };
    const Eigen::Index dimension = innovation.size();
    if (dimension <= 0 || covariance.rows() != dimension || covariance.cols() != dimension ||
        !innovation.allFinite() || !covariance.allFinite())
        return reject("innovation dimensions or values are invalid");

    covariance = 0.5 * (covariance + covariance.transpose());
    Eigen::LDLT<Eigen::MatrixXd> solver(covariance);
    if (solver.info() != Eigen::Success || (solver.vectorD().array() <= 1.0e-12).any()) {
        return reject("innovation covariance is not positive definite");
    }
    const Eigen::VectorXd solved = solver.solve(innovation);
    if (!solved.allFinite()) {
        return reject("innovation covariance solve failed");
    }

    assessment.degrees_of_freedom = static_cast<int>(dimension);
    assessment.nis = std::max(0.0, innovation.dot(solved));
    try {
        const boost::math::chi_squared distribution(
            static_cast<double>(assessment.degrees_of_freedom));
        assessment.upper_tail_probability = std::clamp(
            boost::math::cdf(boost::math::complement(distribution, assessment.nis)), 0.0, 1.0);
    } catch (const std::exception& error) {
        return reject(std::string("chi-square probability failed: ") + error.what());
    }
    assessment.valid =
        std::isfinite(assessment.nis) && std::isfinite(assessment.upper_tail_probability);
    if (!assessment.valid) {
        assessment.invalid_reason = "innovation statistics are non-finite";
    }
    return assessment;
}

/// 一批创新项共用的键顺序、切空间维数和残差行数。
struct InnovationLayout {
    std::vector<gtsam::Key> keys;
    std::vector<Eigen::Index> dimensions;
    Eigen::Index rows = 0;
    std::string invalid_reason;
};

/// 验收逐项 Jacobian，并按首次出现顺序合并共享图变量。
InnovationLayout innovationLayout(const std::vector<InnovationTerm>& terms,
                                  const gtsam::ISAM2& graph) {
    InnovationLayout layout;
    auto& keys = layout.keys;
    auto& key_dimensions = layout.dimensions;
    auto& total_rows = layout.rows;
    /// 保留输入验收失败原因，不构造可用于统计的矩阵。
    const auto reject = [&](const std::string& reason) {
        layout.invalid_reason = reason;
        return layout;
    };
    for (const InnovationTerm& term : terms) {
        if (!term.valid) {
            return reject(term.invalid_reason.empty() ? "innovation batch contains an invalid term"
                                                      : term.invalid_reason);
        }
        if (term.keys.empty() || term.keys.size() != term.whitened_jacobians.size() ||
            term.whitened_residual.size() <= 0 || !term.whitened_residual.allFinite()) {
            return reject("innovation batch term dimensions are invalid");
        }
        total_rows += term.whitened_residual.size();
        for (std::size_t block = 0; block < term.keys.size(); ++block) {
            const Eigen::MatrixXd& jacobian = term.whitened_jacobians[block];
            if (jacobian.rows() != term.whitened_residual.size() || jacobian.cols() <= 0 ||
                !jacobian.allFinite() || !graph.valueExists(term.keys[block])) {
                return reject("innovation batch term Jacobian/key is invalid");
            }
            if (std::find(term.keys.begin(), term.keys.begin() + static_cast<std::ptrdiff_t>(block),
                          term.keys[block]) !=
                term.keys.begin() + static_cast<std::ptrdiff_t>(block)) {
                return reject("innovation batch term contains a duplicate key");
            }
            const auto existing = std::find(keys.begin(), keys.end(), term.keys[block]);
            if (existing == keys.end()) {
                keys.push_back(term.keys[block]);
                key_dimensions.push_back(jacobian.cols());
            } else {
                const std::size_t existing_index =
                    static_cast<std::size_t>(std::distance(keys.begin(), existing));
                if (key_dimensions[existing_index] != jacobian.cols()) {
                    return reject("innovation batch key has inconsistent dimensions");
                }
            }
        }
    }

    return layout;
}

/// 按已验收布局堆叠残差与 Jacobian，完整保留共享变量的列对应关系。
bool assembleInnovations(const std::vector<InnovationTerm>& terms, const InnovationLayout& layout,
                         Eigen::VectorXd& residual, Eigen::MatrixXd& jacobian) {
    const auto& keys = layout.keys;
    const auto& key_dimensions = layout.dimensions;
    std::vector<Eigen::Index> key_offsets(keys.size(), 0);
    Eigen::Index total_columns = 0;
    for (std::size_t index = 0; index < keys.size(); ++index) {
        key_offsets[index] = total_columns;
        total_columns += key_dimensions[index];
    }
    jacobian = Eigen::MatrixXd::Zero(layout.rows, total_columns);
    Eigen::Index row = 0;
    for (const InnovationTerm& term : terms) {
        const Eigen::Index rows = term.whitened_residual.size();
        residual.segment(row, rows) = term.whitened_residual;
        for (std::size_t block = 0; block < term.keys.size(); ++block) {
            const auto found = std::find(keys.begin(), keys.end(), term.keys[block]);
            if (found == keys.end()) {
                return false;
            }
            const std::size_t found_index =
                static_cast<std::size_t>(std::distance(keys.begin(), found));
            const Eigen::Index columns = key_dimensions[found_index];
            jacobian.block(row, key_offsets[found_index], rows, columns) =
                term.whitened_jacobians[block];
        }
        row += rows;
    }
    return true;
}

}  // namespace

/// 在统一预测快照上联合评估一批已白化观测，并保留逐项诊断。
InnovationBatchAssessment PoseGraph::assessInnovationBatch(
    const std::vector<InnovationTerm>& terms) const {
    InnovationBatchAssessment batch;
    /// 构造保留失败原因的无效创新评估。
    const auto reject = [&](const std::string& reason) {
        batch.joint.invalid_reason = reason;
        return batch;
    };
    if (terms.empty()) {
        return reject("innovation batch is empty");
    }
    if (pending_graph_.size() != 0U || !pending_values_.empty()) {
        return reject("innovation assessment requires a committed prediction graph");
    }

    const auto layout = innovationLayout(terms, *isam_);
    if (!layout.invalid_reason.empty())
        return reject(layout.invalid_reason);

    const std::optional<Eigen::MatrixXd> prediction_covariance =
        jointMarginalCovariance(layout.keys, layout.dimensions);
    if (!prediction_covariance.has_value()) {
        return reject("incremental joint prediction covariance is unavailable");
    }

    Eigen::VectorXd residual(layout.rows);
    Eigen::MatrixXd jacobian;
    if (!assembleInnovations(terms, layout, residual, jacobian))
        return reject("innovation batch key assembly failed");
    batch.individual.reserve(terms.size());
    const Eigen::MatrixXd covariance = Eigen::MatrixXd::Identity(layout.rows, layout.rows) +
                                       jacobian * *prediction_covariance * jacobian.transpose();
    batch.joint = assessInnovation(residual, covariance);
    if (terms.size() == 1) {
        batch.individual.push_back(batch.joint);
    } else {
        Eigen::Index row = 0;
        for (const auto& term : terms) {
            const auto rows = term.whitened_residual.size();
            batch.individual.push_back(
                assessInnovation(term.whitened_residual, covariance.block(row, row, rows, rows)));
            row += rows;
        }
    }
    return batch;
}

/// 按 keys 顺序计算联合边缘协方差，包含查询变量间的交叉协方差。
/// 复用 iSAM2 Bayes tree 的条件分布与共同祖先边缘量，通过局部消元求解。
std::optional<Eigen::MatrixXd> PoseGraph::jointMarginalCovariance(
    const std::vector<gtsam::Key>& keys, const std::vector<Eigen::Index>& dimensions) const {
    if (keys.empty() || keys.size() != dimensions.size())
        return std::nullopt;
    gtsam::Ordering ordering;
    Eigen::Index dimension = 0;
    for (std::size_t i = 0; i < keys.size(); ++i) {
        if (!isam_->valueExists(keys[i]) || dimensions[i] <= 0 ||
            isam_->getLinearizationPoint().at(keys[i]).dim() !=
                static_cast<std::size_t>(dimensions[i]) ||
            std::find(keys.begin(), keys.begin() + i, keys[i]) != keys.begin() + i)
            return std::nullopt;
        ordering.push_back(keys[i]);
        dimension += dimensions[i];
    }
    try {
        Eigen::MatrixXd covariance;
        if (keys.size() == 1) {
            covariance = isam_->marginalCovariance(keys.front());
        } else {
            using Path = std::vector<const gtsam::ISAM2Clique*>;
            std::vector<Path> paths;
            std::map<const gtsam::ISAM2Clique*, Path> common_paths;
            for (const auto key : keys) {
                Path path;
                for (auto clique = (*isam_)[key]; clique; clique = clique->parent())
                    path.push_back(clique.get());
                auto [found, inserted] = common_paths.emplace(path.back(), path);
                if (!inserted) {
                    auto& common = found->second;
                    const auto mismatch =
                        std::mismatch(common.rbegin(), common.rend(), path.rbegin(), path.rend());
                    common.erase(common.begin(), mismatch.first.base());
                }
                paths.push_back(std::move(path));
            }
            gtsam::GaussianFactorGraph joint_factors;
            std::set<const gtsam::ISAM2Clique*> visited;
            for (const auto& path : paths) {
                const auto ancestor = common_paths.at(path.back()).front();
                for (const auto clique : path) {
                    if (!visited.insert(clique).second)
                        break;
                    if (clique == ancestor) {
                        // 每棵独立树只加入一次祖先边缘量，保留全部分隔变量的相关性。
                        joint_factors += clique->marginal2();
                        break;
                    }
                    joint_factors.push_back(clique->conditional());
                }
            }
            const auto joint = joint_factors.marginalMultifrontalBayesNet(ordering);
            const Eigen::MatrixXd R = joint->matrix(ordering).first;
            if (R.rows() != dimension || R.cols() != dimension || !R.allFinite())
                return std::nullopt;
            const Eigen::MatrixXd information = R.transpose() * R;
            Eigen::LDLT<Eigen::MatrixXd> solver(0.5 * (information + information.transpose()));
            if (solver.info() != Eigen::Success || (solver.vectorD().array() <= 1e-12).any())
                return std::nullopt;
            covariance = solver.solve(Eigen::MatrixXd::Identity(dimension, dimension));
        }
        covariance = 0.5 * (covariance + covariance.transpose());
        if (!covariance.allFinite())
            return std::nullopt;
        Eigen::LDLT<Eigen::MatrixXd> check(covariance);
        if (check.info() != Eigen::Success || (check.vectorD().array() <= 1e-12).any())
            return std::nullopt;
        return covariance;
    } catch (const std::exception&) {
        return std::nullopt;
    }
}
}  // namespace backend
