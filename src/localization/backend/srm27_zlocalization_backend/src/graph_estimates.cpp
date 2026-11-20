/**
 * @file graph_estimates.cpp
 * @brief 查询轨迹、UWB 共享状态及其边缘协方差。
 */

#include <array>
#include <cmath>
#include <limits>

#include "backend/pose_graph.hpp"

namespace backend {
namespace {

/// 将 GTSAM 的旋转优先切空间协方差转换为后端的平移优先顺序。
PoseCovariance6d gtsamToRosOrder(const gtsam::Matrix& covariance) {
    PoseCovariance6d out = PoseCovariance6d::Zero();
    if (covariance.rows() != 6 || covariance.cols() != 6) {
        return out;
    }
    // GTSAM 的排列顺序为 [旋转, 平移]，后端及 ROS 为 [平移, 旋转]。
    out.block<3, 3>(0, 0) = covariance.block<3, 3>(3, 3);
    out.block<3, 3>(0, 3) = covariance.block<3, 3>(3, 0);
    out.block<3, 3>(3, 0) = covariance.block<3, 3>(0, 3);
    out.block<3, 3>(3, 3) = covariance.block<3, 3>(0, 0);
    return 0.5 * (out + out.transpose());
}

}  // namespace

/// 返回当前求解器内全部活动轨迹位姿，不包含共享标定变量。
std::vector<std::pair<int, Eigen::Matrix4d>> PoseGraph::activeEstimates() const {
    const gtsam::Values values = isam_->calculateEstimate();
    std::vector<std::pair<int, Eigen::Matrix4d>> poses;
    poses.reserve(values.size());
    for (const gtsam::Key key : values.keys()) {
        // 轨迹编号使用非负 int，带字符前缀的共享变量属于独立键空间。
        if (key > static_cast<gtsam::Key>(std::numeric_limits<int>::max())) {
            continue;
        }
        poses.emplace_back(static_cast<int>(key), fromGtsam(values.at<gtsam::Pose3>(key)));
    }
    return poses;
}

/// 判断指定轨迹节点是否仍有活动估计。
bool PoseGraph::hasEstimate(const int index) const {
    return isam_->valueExists(index);
}

/// 查询指定轨迹节点位姿；节点不存在时返回空值。
std::optional<Eigen::Matrix4d> PoseGraph::estimate(const int index) const {
    if (!isam_->valueExists(index)) {
        return std::nullopt;
    }
    return fromGtsam(isam_->calculateEstimate<gtsam::Pose3>(index));
}

/// 判断 UWB 平面对齐及可选高度变量是否均已初始化。
bool PoseGraph::hasUwbAlignment() const {
    if (!uwb_alignment_initialized_) {
        return false;
    }
    const gtsam::Key planar_key = uwbAlignmentKey();
    const bool has_planar = pending_values_.exists(planar_key) || isam_->valueExists(planar_key);
    if (!has_planar || !uwb_alignment_use_z_) {
        return has_planar;
    }
    const gtsam::Key z_key = uwbAlignmentZKey();
    return pending_values_.exists(z_key) || isam_->valueExists(z_key);
}

/// 将 UWB 的 Pose2 加可选高度状态还原为齐次变换。
std::optional<Eigen::Matrix4d> PoseGraph::uwbAlignmentEstimate() const {
    if (!hasUwbAlignment()) {
        return std::nullopt;
    }
    try {
        const gtsam::Key planar_key = uwbAlignmentKey();
        const gtsam::Pose2 planar = pending_values_.exists(planar_key)
                                        ? pending_values_.at<gtsam::Pose2>(planar_key)
                                        : isam_->calculateEstimate<gtsam::Pose2>(planar_key);
        double z = uwb_alignment_fixed_z_;
        if (uwb_alignment_use_z_) {
            const gtsam::Key z_key = uwbAlignmentZKey();
            z = pending_values_.exists(z_key) ? pending_values_.at<double>(z_key)
                                              : isam_->calculateEstimate<double>(z_key);
        }
        if (!std::isfinite(z)) {
            return std::nullopt;
        }
        Eigen::Matrix4d estimate = Eigen::Matrix4d::Identity();
        const double c = std::cos(planar.theta());
        const double s = std::sin(planar.theta());
        estimate(0, 0) = c;
        estimate(0, 1) = -s;
        estimate(1, 0) = s;
        estimate(1, 1) = c;
        estimate(0, 3) = planar.x();
        estimate(1, 3) = planar.y();
        estimate(2, 3) = z;
        return estimate;
    } catch (const std::exception&) {
        return std::nullopt;
    }
}

/// 查询 UWB 对齐状态并转换为世界加性六维协方差。
std::optional<PoseCovariance6d> PoseGraph::uwbAlignmentMarginalCovariance() const {
    const gtsam::Key planar_key = uwbAlignmentKey();
    if (!uwb_alignment_initialized_ || !isam_->valueExists(planar_key)) {
        return std::nullopt;
    }
    try {
        const gtsam::Pose2 planar = isam_->calculateEstimate<gtsam::Pose2>(planar_key);
        const Eigen::Matrix2d R_world_planar = planar.rotation().matrix();

        PoseCovariance6d result = PoseCovariance6d::Zero();
        if (!uwb_alignment_use_z_) {
            Eigen::Matrix3d local = isam_->marginalCovariance(planar_key);
            Eigen::Matrix3d J = Eigen::Matrix3d::Identity();
            J.topLeftCorner<2, 2>() = R_world_planar;
            const Eigen::Matrix3d world = J * local * J.transpose();
            const std::array<int, 3> indices = {0, 1, 5};
            for (Eigen::Index row = 0; row < 3; ++row) {
                for (Eigen::Index col = 0; col < 3; ++col) {
                    result(indices[static_cast<std::size_t>(row)],
                           indices[static_cast<std::size_t>(col)]) = world(row, col);
                }
            }
            return 0.5 * (result + result.transpose());
        }

        const gtsam::Key z_key = uwbAlignmentZKey();
        const std::optional<Eigen::MatrixXd> local =
            jointMarginalCovariance({planar_key, z_key}, {3, 1});
        if (!local.has_value()) {
            return std::nullopt;
        }
        Eigen::Matrix4d J = Eigen::Matrix4d::Identity();
        J.topLeftCorner<2, 2>() = R_world_planar;
        const Eigen::Matrix4d world = J * *local * J.transpose();
        const std::array<int, 4> indices = {0, 1, 5, 2};
        for (Eigen::Index row = 0; row < 4; ++row) {
            for (Eigen::Index col = 0; col < 4; ++col) {
                result(indices[static_cast<std::size_t>(row)],
                       indices[static_cast<std::size_t>(col)]) = world(row, col);
            }
        }
        return 0.5 * (result + result.transpose());
    } catch (const std::exception&) {
        return std::nullopt;
    }
}

/// 判断共享 UWB 时间偏置变量是否已初始化且仍可查询。
bool PoseGraph::hasUwbTimeOffset() const {
    if (!uwb_time_offset_initialized_) {
        return false;
    }
    const gtsam::Key key = uwbTimeOffsetKey();
    return pending_values_.exists(key) || isam_->valueExists(key);
}

/// 查询共享 UWB 时间偏置估计。
std::optional<double> PoseGraph::uwbTimeOffsetEstimate() const {
    if (!uwb_time_offset_initialized_) {
        return std::nullopt;
    }
    const gtsam::Key key = uwbTimeOffsetKey();
    if (pending_values_.exists(key)) {
        const double value = pending_values_.at<double>(key);
        return std::isfinite(value) ? std::optional<double>(value) : std::nullopt;
    }
    if (!isam_->valueExists(key)) {
        return std::nullopt;
    }
    try {
        const double value = isam_->calculateEstimate<double>(key);
        return std::isfinite(value) ? std::optional<double>(value) : std::nullopt;
    } catch (const std::exception&) {
        return std::nullopt;
    }
}

/// 查询共享 UWB 时间偏置的边缘方差。
std::optional<double> PoseGraph::uwbTimeOffsetMarginalVariance() const {
    const gtsam::Key key = uwbTimeOffsetKey();
    if (!uwb_time_offset_initialized_ || !isam_->valueExists(key)) {
        return std::nullopt;
    }
    try {
        const Eigen::MatrixXd covariance = isam_->marginalCovariance(key);
        if (covariance.rows() != 1 || covariance.cols() != 1 || !std::isfinite(covariance(0, 0)) ||
            covariance(0, 0) <= 0.0) {
            return std::nullopt;
        }
        return covariance(0, 0);
    } catch (const std::exception&) {
        return std::nullopt;
    }
}

/// 查询轨迹位姿的局部右扰动边缘协方差，并转换为后端 [平移, 旋转] 顺序。
std::optional<PoseCovariance6d> PoseGraph::marginalCovariance(const int index) const {
    if (!isam_->valueExists(index)) {
        return std::nullopt;
    }
    try {
        return gtsamToRosOrder(isam_->marginalCovariance(index));
    } catch (const std::exception&) {
        return std::nullopt;
    }
}

}  // namespace backend
