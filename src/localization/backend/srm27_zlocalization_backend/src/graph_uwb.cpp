/**
 * @file graph_uwb.cpp
 * @brief 构造并线性化固定或在线时空对齐的 UWB 位置测量。
 */

#include <stdexcept>

#include "backend/pose_graph.hpp"
#include "noise.hpp"

namespace backend {
namespace {

class PositionWithLeverArmFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
public:
    using gtsam::NoiseModelFactor1<gtsam::Pose3>::evaluateError;

    /// 构造包含固定天线连杆臂的三维位置因子。
    PositionWithLeverArmFactor(const gtsam::Key key, const gtsam::Point3& measurement,
                               const gtsam::Point3& lever_arm,
                               const gtsam::SharedNoiseModel& noise_model)
        : gtsam::NoiseModelFactor1<gtsam::Pose3>(noise_model, key),
          measurement_(measurement),
          lever_arm_(lever_arm) {}

    /// 计算测量残差，并在请求时写出各状态的解析 Jacobian。
    gtsam::Vector evaluateError(const gtsam::Pose3& pose, boost::optional<gtsam::Matrix&> jacobian =
                                                              boost::none) const override {
        gtsam::Matrix36 H;
        const gtsam::Point3 predicted = pose.transformFrom(lever_arm_, H);
        if (jacobian) {
            *jacobian = H;
        }
        return predicted - measurement_;
    }

private:
    gtsam::Point3 measurement_;
    gtsam::Point3 lever_arm_;
};

class PositionXYWithLeverArmFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
public:
    using gtsam::NoiseModelFactor1<gtsam::Pose3>::evaluateError;

    /// 构造包含固定天线连杆臂的水平位置因子。
    PositionXYWithLeverArmFactor(const gtsam::Key key, const gtsam::Vector2& measurement,
                                 const gtsam::Point3& lever_arm,
                                 const gtsam::SharedNoiseModel& noise_model)
        : gtsam::NoiseModelFactor1<gtsam::Pose3>(noise_model, key),
          measurement_(measurement),
          lever_arm_(lever_arm) {}

    /// 计算测量残差，并在请求时写出各状态的解析 Jacobian。
    gtsam::Vector evaluateError(const gtsam::Pose3& pose, boost::optional<gtsam::Matrix&> jacobian =
                                                              boost::none) const override {
        gtsam::Matrix36 H;
        const gtsam::Point3 predicted = pose.transformFrom(lever_arm_, H);
        if (jacobian) {
            *jacobian = H.topRows<2>();
        }
        gtsam::Vector2 error;
        error << predicted.x() - measurement_.x(), predicted.y() - measurement_.y();
        return error;
    }

private:
    gtsam::Vector2 measurement_;
    gtsam::Point3 lever_arm_;
};

struct AlignedUwbPrediction {
    Eigen::Vector3d position_uwb = Eigen::Vector3d::Zero();
    gtsam::Matrix36 pose_jacobian = gtsam::Matrix36::Zero();
    gtsam::Matrix33 planar_jacobian = gtsam::Matrix33::Zero();
    gtsam::Vector3 z_jacobian = gtsam::Vector3::Zero();
    gtsam::Matrix22 map_xy_jacobian = gtsam::Matrix22::Zero();
};

/// 将地图天线位置转换到 UWB 坐标并计算共享状态 Jacobian。
inline AlignedUwbPrediction predictAlignedUwb(const gtsam::Pose3& pose,
                                              const gtsam::Pose2& T_map_uwb_planar,
                                              const double z_map_uwb,
                                              const Eigen::Vector3d& antenna_in_pose) {
    gtsam::Matrix36 H_map_antenna_pose;
    const gtsam::Point3 position_map_antenna =
        pose.transformFrom(gtsam::Point3(antenna_in_pose), H_map_antenna_pose);
    gtsam::Matrix23 H_planar;
    gtsam::Matrix22 H_map_xy;
    const gtsam::Point2 position_uwb_xy = T_map_uwb_planar.transformTo(
        gtsam::Point2(position_map_antenna.x(), position_map_antenna.y()), H_planar, H_map_xy);

    AlignedUwbPrediction prediction;
    prediction.position_uwb << position_uwb_xy.x(), position_uwb_xy.y(),
        position_map_antenna.z() - z_map_uwb;
    prediction.pose_jacobian.topRows<2>() = H_map_xy * H_map_antenna_pose.topRows<2>();
    prediction.pose_jacobian.row(2) = H_map_antenna_pose.row(2);
    prediction.planar_jacobian.topRows<2>() = H_planar;
    prediction.z_jacobian(2) = -1.0;
    prediction.map_xy_jacobian = H_map_xy;
    return prediction;
}

class UwbPositionXYWithAlignmentFactor
    : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose2> {
public:
    using gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose2>::evaluateError;

    /// 构造联合估计平面对齐与传感器位姿的水平位置因子。
    UwbPositionXYWithAlignmentFactor(const gtsam::Key pose_key, const gtsam::Key alignment_key,
                                     const gtsam::Vector2& measurement_uwb,
                                     const gtsam::Point3& lever_arm,
                                     const gtsam::SharedNoiseModel& noise_model)
        : gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose2>(noise_model, pose_key,
                                                               alignment_key),
          measurement_uwb_(measurement_uwb),
          lever_arm_(lever_arm) {}

    /// 计算测量残差，并在请求时写出各状态的解析 Jacobian。
    gtsam::Vector evaluateError(
        const gtsam::Pose3& pose, const gtsam::Pose2& T_map_uwb_planar,
        boost::optional<gtsam::Matrix&> pose_jacobian = boost::none,
        boost::optional<gtsam::Matrix&> alignment_jacobian = boost::none) const override {
        const AlignedUwbPrediction prediction =
            predictAlignedUwb(pose, T_map_uwb_planar, /*z_map_uwb=*/0.0,
                              Eigen::Vector3d(lever_arm_.x(), lever_arm_.y(), lever_arm_.z()));
        if (pose_jacobian) {
            *pose_jacobian = prediction.pose_jacobian.topRows<2>();
        }
        if (alignment_jacobian) {
            *alignment_jacobian = prediction.planar_jacobian.topRows<2>();
        }
        return prediction.position_uwb.head<2>() - measurement_uwb_;
    }

private:
    gtsam::Vector2 measurement_uwb_;
    gtsam::Point3 lever_arm_;
};

class UwbPositionWithAlignmentFactor
    : public gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Pose2, double> {
public:
    using gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Pose2, double>::evaluateError;

    /// 构造联合估计空间对齐与传感器位姿的三维位置因子。
    UwbPositionWithAlignmentFactor(const gtsam::Key pose_key, const gtsam::Key alignment_key,
                                   const gtsam::Key alignment_z_key,
                                   const gtsam::Point3& measurement_uwb,
                                   const gtsam::Point3& lever_arm,
                                   const gtsam::SharedNoiseModel& noise_model)
        : gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Pose2, double>(
              noise_model, pose_key, alignment_key, alignment_z_key),
          measurement_uwb_(measurement_uwb),
          lever_arm_(lever_arm) {}

    /// 计算测量残差，并在请求时写出各状态的解析 Jacobian。
    gtsam::Vector evaluateError(
        const gtsam::Pose3& pose, const gtsam::Pose2& T_map_uwb_planar, const double& z_map_uwb,
        boost::optional<gtsam::Matrix&> pose_jacobian = boost::none,
        boost::optional<gtsam::Matrix&> alignment_jacobian = boost::none,
        boost::optional<gtsam::Matrix&> z_jacobian = boost::none) const override {
        const AlignedUwbPrediction prediction =
            predictAlignedUwb(pose, T_map_uwb_planar, z_map_uwb,
                              Eigen::Vector3d(lever_arm_.x(), lever_arm_.y(), lever_arm_.z()));
        if (pose_jacobian) {
            *pose_jacobian = prediction.pose_jacobian;
        }
        if (alignment_jacobian) {
            *alignment_jacobian = prediction.planar_jacobian;
        }
        if (z_jacobian) {
            *z_jacobian = prediction.z_jacobian;
        }
        return prediction.position_uwb - measurement_uwb_;
    }

private:
    gtsam::Point3 measurement_uwb_;
    gtsam::Point3 lever_arm_;
};

class TimedUwbPositionFactor : public gtsam::NoiseModelFactor2<gtsam::Pose3, double> {
public:
    using gtsam::NoiseModelFactor2<gtsam::Pose3, double>::evaluateError;

    /// 构造使用共享时间偏置查询局部轨迹的固定对齐位置因子。
    TimedUwbPositionFactor(const gtsam::Key pose_key, const gtsam::Key time_offset_key,
                           const Eigen::Vector3d& measurement_map, UwbTimeOffsetSupport support,
                           const bool use_z, const gtsam::SharedNoiseModel& noise_model)
        : gtsam::NoiseModelFactor2<gtsam::Pose3, double>(noise_model, pose_key, time_offset_key),
          measurement_map_(measurement_map),
          support_(std::move(support)),
          use_z_(use_z) {}

    /// 计算测量残差，并在请求时写出各状态的解析 Jacobian。
    gtsam::Vector evaluateError(
        const gtsam::Pose3& pose, const double& time_offset_seconds,
        boost::optional<gtsam::Matrix&> pose_jacobian = boost::none,
        boost::optional<gtsam::Matrix&> time_jacobian = boost::none) const override {
        Eigen::Vector3d antenna_in_keyframe;
        Eigen::Vector3d velocity_in_keyframe;
        if (!support_.evaluate(time_offset_seconds, &antenna_in_keyframe, &velocity_in_keyframe)) {
            throw std::runtime_error("timed UWB odometry support cannot be evaluated");
        }
        gtsam::Matrix36 H_map_antenna_pose;
        const gtsam::Point3 position_map_antenna =
            pose.transformFrom(gtsam::Point3(antenna_in_keyframe), H_map_antenna_pose);
        const Eigen::Vector3d velocity_map = pose.rotation().matrix() * velocity_in_keyframe;
        if (use_z_) {
            if (pose_jacobian) {
                *pose_jacobian = H_map_antenna_pose;
            }
            if (time_jacobian) {
                time_jacobian->resize(3, 1);
                time_jacobian->col(0) = velocity_map;
            }
            return position_map_antenna - measurement_map_;
        }

        if (pose_jacobian) {
            *pose_jacobian = H_map_antenna_pose.topRows<2>();
        }
        if (time_jacobian) {
            time_jacobian->resize(2, 1);
            time_jacobian->col(0) = velocity_map.head<2>();
        }
        gtsam::Vector2 error;
        error << position_map_antenna.x() - measurement_map_.x(),
            position_map_antenna.y() - measurement_map_.y();
        return error;
    }

private:
    Eigen::Vector3d measurement_map_ = Eigen::Vector3d::Zero();
    UwbTimeOffsetSupport support_;
    bool use_z_ = true;
};

class TimedUwbPositionXYWithAlignmentFactor
    : public gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Pose2, double> {
public:
    using gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Pose2, double>::evaluateError;

    /// 构造包含共享平面对齐和时间偏置的水平位置因子。
    TimedUwbPositionXYWithAlignmentFactor(const gtsam::Key pose_key, const gtsam::Key alignment_key,
                                          const gtsam::Key time_offset_key,
                                          const Eigen::Vector3d& measurement_uwb,
                                          UwbTimeOffsetSupport support,
                                          const gtsam::SharedNoiseModel& noise_model)
        : gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Pose2, double>(
              noise_model, pose_key, alignment_key, time_offset_key),
          measurement_uwb_(measurement_uwb),
          support_(std::move(support)) {}

    /// 计算测量残差，并在请求时写出各状态的解析 Jacobian。
    gtsam::Vector evaluateError(
        const gtsam::Pose3& pose, const gtsam::Pose2& T_map_uwb_planar,
        const double& time_offset_seconds,
        boost::optional<gtsam::Matrix&> pose_jacobian = boost::none,
        boost::optional<gtsam::Matrix&> alignment_jacobian = boost::none,
        boost::optional<gtsam::Matrix&> time_jacobian = boost::none) const override {
        Eigen::Vector3d antenna_in_keyframe;
        Eigen::Vector3d velocity_in_keyframe;
        if (!support_.evaluate(time_offset_seconds, &antenna_in_keyframe, &velocity_in_keyframe)) {
            throw std::runtime_error("timed aligned UWB odometry support cannot be evaluated");
        }
        const AlignedUwbPrediction prediction =
            predictAlignedUwb(pose, T_map_uwb_planar, /*z_map_uwb=*/0.0, antenna_in_keyframe);
        const Eigen::Vector3d velocity_map = pose.rotation().matrix() * velocity_in_keyframe;
        if (pose_jacobian) {
            *pose_jacobian = prediction.pose_jacobian.topRows<2>();
        }
        if (alignment_jacobian) {
            *alignment_jacobian = prediction.planar_jacobian.topRows<2>();
        }
        if (time_jacobian) {
            time_jacobian->resize(2, 1);
            time_jacobian->col(0) = prediction.map_xy_jacobian * velocity_map.head<2>();
        }
        return prediction.position_uwb.head<2>() - measurement_uwb_.head<2>();
    }

private:
    Eigen::Vector3d measurement_uwb_ = Eigen::Vector3d::Zero();
    UwbTimeOffsetSupport support_;
};

class TimedUwbPositionWithAlignmentFactor
    : public gtsam::NoiseModelFactor4<gtsam::Pose3, gtsam::Pose2, double, double> {
public:
    using gtsam::NoiseModelFactor4<gtsam::Pose3, gtsam::Pose2, double, double>::evaluateError;

    /// 构造包含共享空间对齐和时间偏置的三维位置因子。
    TimedUwbPositionWithAlignmentFactor(const gtsam::Key pose_key, const gtsam::Key alignment_key,
                                        const gtsam::Key alignment_z_key,
                                        const gtsam::Key time_offset_key,
                                        const Eigen::Vector3d& measurement_uwb,
                                        UwbTimeOffsetSupport support,
                                        const gtsam::SharedNoiseModel& noise_model)
        : gtsam::NoiseModelFactor4<gtsam::Pose3, gtsam::Pose2, double, double>(
              noise_model, pose_key, alignment_key, alignment_z_key, time_offset_key),
          measurement_uwb_(measurement_uwb),
          support_(std::move(support)) {}

    /// 计算测量残差，并在请求时写出各状态的解析 Jacobian。
    gtsam::Vector evaluateError(
        const gtsam::Pose3& pose, const gtsam::Pose2& T_map_uwb_planar, const double& z_map_uwb,
        const double& time_offset_seconds,
        boost::optional<gtsam::Matrix&> pose_jacobian = boost::none,
        boost::optional<gtsam::Matrix&> alignment_jacobian = boost::none,
        boost::optional<gtsam::Matrix&> z_jacobian = boost::none,
        boost::optional<gtsam::Matrix&> time_jacobian = boost::none) const override {
        Eigen::Vector3d antenna_in_keyframe;
        Eigen::Vector3d velocity_in_keyframe;
        if (!support_.evaluate(time_offset_seconds, &antenna_in_keyframe, &velocity_in_keyframe)) {
            throw std::runtime_error("timed aligned UWB odometry support cannot be evaluated");
        }
        const AlignedUwbPrediction prediction =
            predictAlignedUwb(pose, T_map_uwb_planar, z_map_uwb, antenna_in_keyframe);
        const Eigen::Vector3d velocity_map = pose.rotation().matrix() * velocity_in_keyframe;
        if (pose_jacobian) {
            *pose_jacobian = prediction.pose_jacobian;
        }
        if (alignment_jacobian) {
            *alignment_jacobian = prediction.planar_jacobian;
        }
        if (z_jacobian) {
            *z_jacobian = prediction.z_jacobian;
        }
        if (time_jacobian) {
            time_jacobian->resize(3, 1);
            time_jacobian->topRows<2>() = prediction.map_xy_jacobian * velocity_map.head<2>();
            (*time_jacobian)(2, 0) = velocity_map.z();
        }
        return prediction.position_uwb - measurement_uwb_;
    }

private:
    Eigen::Vector3d measurement_uwb_ = Eigen::Vector3d::Zero();
    UwbTimeOffsetSupport support_;
};

/// 按高度使用策略选择二维或三维位置测量协方差。
Eigen::MatrixXd measurementCovariance(const UwbMeasurement& measurement, bool use_z) {
    return use_z ? Eigen::MatrixXd(measurement.covariance)
                 : Eigen::MatrixXd(measurement.covariance.topLeftCorner<2, 2>());
}
}  // namespace

/// 检查位置、连杆臂及共享时空状态，返回不可用原因。
std::string PoseGraph::uwbInputError(const UwbMeasurement& measurement, bool use_z) const {
    if (!(use_z ? measurement.position.allFinite() : measurement.position.head<2>().allFinite()) ||
        (!measurement.time_support && !measurement.antenna_in_keyframe.allFinite()))
        return "UWB position or lever arm is non-finite";
    if (measurement.with_alignment && (!hasUwbAlignment() || use_z != uwb_alignment_use_z_))
        return "UWB alignment state is unavailable or has a different dimension";
    if (measurement.time_support) {
        const auto time = uwbTimeOffsetEstimate();
        Eigen::Vector3d antenna;
        if (!time || !measurement.time_support->evaluate(*time, &antenna))
            return "timed UWB odometry support or time state is unavailable";
    }
    return {};
}

/// 按高度、空间对齐和时间支撑组合构造对应的 UWB 因子。
gtsam::NoiseModelFactor::shared_ptr PoseGraph::makeUwbFactor(
    int index, const UwbMeasurement& m, bool use_z, const gtsam::SharedNoiseModel& noise) const {
    using namespace detail;
    if (m.time_support) {
        if (!m.with_alignment)
            return gtsam::make_shared<TimedUwbPositionFactor>(index, uwbTimeOffsetKey(), m.position,
                                                              *m.time_support, use_z, noise);
        if (use_z)
            return gtsam::make_shared<TimedUwbPositionWithAlignmentFactor>(
                index, uwbAlignmentKey(), uwbAlignmentZKey(), uwbTimeOffsetKey(), m.position,
                *m.time_support, noise);
        return gtsam::make_shared<TimedUwbPositionXYWithAlignmentFactor>(
            index, uwbAlignmentKey(), uwbTimeOffsetKey(), m.position, *m.time_support, noise);
    }
    const gtsam::Point3 lever = m.antenna_in_keyframe;
    if (m.with_alignment) {
        if (use_z)
            return gtsam::make_shared<UwbPositionWithAlignmentFactor>(
                index, uwbAlignmentKey(), uwbAlignmentZKey(), m.position, lever, noise);
        return gtsam::make_shared<UwbPositionXYWithAlignmentFactor>(
            index, uwbAlignmentKey(), m.position.head<2>(), lever, noise);
    }
    if (use_z)
        return gtsam::make_shared<PositionWithLeverArmFactor>(index, m.position, lever, noise);
    return gtsam::make_shared<PositionXYWithLeverArmFactor>(index, m.position.head<2>(), lever,
                                                            noise);
}

/// 校验 UWB 测量并将对应的位置因子加入待提交图。
void PoseGraph::addUwbPosition(int index, const UwbMeasurement& measurement, bool use_z) {
    const auto error = uwbInputError(measurement, use_z);
    if (!error.empty())
        throw std::invalid_argument(error);
    const auto factor = makeUwbFactor(
        index, measurement, use_z,
        detail::strictCovarianceNoise(measurementCovariance(measurement, use_z), "UWB position"));
    pending_graph_.add(factor);
}

/// 在已提交图上计算 UWB 残差及共享时空状态的白化 Jacobian。
InnovationTerm PoseGraph::linearizeUwbPosition(int index, const UwbMeasurement& measurement,
                                               bool use_z) const {
    using detail::invalidInnovationTerm;
    if (!pending_graph_.empty() || !pending_values_.empty())
        return invalidInnovationTerm("innovation assessment requires a committed prediction graph");
    if (!isam_->valueExists(index))
        return invalidInnovationTerm("UWB pose index is unavailable");
    const auto error = uwbInputError(measurement, use_z);
    if (!error.empty())
        return invalidInnovationTerm(error);

    gtsam::Values values;
    values.insert(index, isam_->calculateEstimate<gtsam::Pose3>(index));
    if (measurement.with_alignment) {
        values.insert(uwbAlignmentKey(), isam_->calculateEstimate<gtsam::Pose2>(uwbAlignmentKey()));
        if (use_z)
            values.insert(uwbAlignmentZKey(), isam_->calculateEstimate<double>(uwbAlignmentZKey()));
    }
    if (measurement.time_support)
        values.insert(uwbTimeOffsetKey(), *uwbTimeOffsetEstimate());
    const auto factor =
        makeUwbFactor(index, measurement, use_z, gtsam::noiseModel::Unit::Create(use_z ? 3 : 2));
    std::vector<gtsam::Matrix> jacobians(factor->keys().size());
    const auto residual = factor->unwhitenedError(values, jacobians);
    return detail::makeWhitenedInnovationTerm(
        std::vector<gtsam::Key>(factor->keys().begin(), factor->keys().end()), residual, jacobians,
        measurementCovariance(measurement, use_z), "UWB");
}
}  // namespace backend
