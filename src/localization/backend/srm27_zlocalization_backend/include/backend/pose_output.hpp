/**
 * @file pose_output.hpp
 * @brief 管理可信位姿缓存、输出投影与协方差传播锚。
 */
#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <cstdint>
#include <optional>

#include "backend/correction_anchor.hpp"
#include "backend/factors.hpp"
#include "backend/time.hpp"
#include "backend/view.hpp"

namespace backend {
class PoseGraph;

/// 将 map→odom 投影为 4-DoF：保留 x/y/yaw，z/roll/pitch 置零。
/// 仅供 2D Nav2 栈消费；内部估计仍保留完整 6-DoF。
inline Eigen::Matrix4d planarMapFromOdom(const Eigen::Matrix4d& map_from_odom) {
    Eigen::Matrix4d planar = Eigen::Matrix4d::Identity();
    planar(0, 3) = map_from_odom(0, 3);
    planar(1, 3) = map_from_odom(1, 3);
    const double yaw = std::atan2(map_from_odom(1, 0), map_from_odom(0, 0));
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    planar(0, 0) = c;
    planar(0, 1) = -s;
    planar(1, 0) = s;
    planar(1, 1) = c;
    return planar;
}

struct TrustedPoseProjection {
    Eigen::Matrix4d T_map_odom = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d T_map_sensor = Eigen::Matrix4d::Identity();
    /// 不受信时保持 map→odom 不变（不再施加新校正），位姿随前端里程计递推。
    bool map_from_odom_frozen = false;
};

/// 缓存最后可信的地图系传感器位姿，并按定位状态计算全局输出变换。
class TrustedPoseProjector {
public:
    /// 清除可信位姿及其前端代次和时间身份。
    void clear();
    /// 返回是否已经保存可信全局传感器位姿。
    bool hasPose() const { return has_pose_; }
    /// 返回最后可信的地图系传感器位姿。
    const Eigen::Matrix4d& mapSensorPose() const { return map_from_sensor_; }
    /// 返回可信位姿对应的传感器时刻，调用方须先确认缓存有效。
    Timestamp timestamp() const { return stamp_; }
    /// 返回可信位姿所属的前端代次，避免跨代次比较传感器时间。
    std::uint64_t epoch() const { return epoch_; }

    /// 仅当 (epoch, stamp) 不回退时更新；先比较代次，再比较该代次内的时间戳。
    bool updateIfNewer(const Eigen::Matrix4d& T_map_sensor, const Timestamp& stamp,
                       std::uint64_t epoch);

    /// 提交已接受的绝对校正，允许新代次首帧使用零时间戳。
    bool commitCorrection(const Eigen::Matrix4d& T_map_sensor, const Timestamp& stamp,
                          std::uint64_t epoch);

    /// 按定位状态计算地图系输出位姿。
    ///
    /// 可信时沿用 `T_map_odom_nominal`；不可信时**保持 `T_map_odom` 不变**
    /// （不施加新校正，只用前端里程计递推），而不是冻结 `T_map_sensor`。
    /// 后者在传感器/云台自转时会让 `map→odom` 1:1 反向旋转，map 系里的
    /// 代价地图、全局路径和点云会跟着传感器转（实测 bag real_zloc_1127_174433）。
    /// 换代时调用方会先 `reframeOdometry`，nominal 已换算到新 odom 原点，
    /// 因此这里直接沿用 nominal 仍能保持物理位姿连续。
    TrustedPoseProjection project(bool pose_trusted, const Eigen::Matrix4d& T_map_odom_nominal,
                                  const Eigen::Matrix4d& T_odom_sensor) const;

private:
    Eigen::Matrix4d map_from_sensor_ = Eigen::Matrix4d::Identity();
    std::uint64_t epoch_ = 0;
    Timestamp stamp_;
    bool has_pose_ = false;
};

class PoseOutput {
public:
    /// 保存相对位姿传播使用的里程计过程噪声参数。
    explicit PoseOutput(const OdometryProcessNoiseConfig& noise) : noise_(noise) {}
    /// 清空可信位姿、协方差锚和 TF 发布水位并重置全局规范。
    void initialize();
    /// 推进输出版本并重置新代次的 TF 发布水位。
    void resetEpoch(std::uint64_t epoch) {
        ++pose_revision_;
        last_tf_epoch_ = epoch;
        last_tf_stamp_ = Timestamp();
    }
    /// 返回当前 odom 坐标系到 map 坐标系的变换。
    const Eigen::Matrix4d& transform() const { return map_from_odom_; }
    /// 替换当前 odom 坐标系到 map 坐标系的变换。
    void setTransform(const Eigen::Matrix4d& pose) { map_from_odom_ = pose; }
    /// 按同一物理状态的新旧坐标关系转换全局规范及图协方差传播锚。
    void reframeOdometry(const Eigen::Matrix4d& old_from_new, std::uint64_t source_epoch,
                         std::uint64_t target_epoch);
    /// 将已验收的地图系协方差绑定到当前帧，作为后续相对运动的传播参考。
    void anchorCovariance(const FrontendFrame& frame, const PoseCovariance6d& covariance);
    /// 根据图锚或有效前端边缘查询当前帧的地图系协方差，缺少支撑时返回空值。
    std::optional<PoseCovariance6d> covarianceAt(const FrontendFrame& frame) const;
    /// 设置地图坐标中里程计原点的平面位置。
    void setOrigin(const Eigen::Vector2d& origin) { map_from_odom_.block<2, 1>(0, 3) = origin; }
    /// 返回输出位姿及协方差的提交版本。
    std::uint64_t revision() const { return pose_revision_; }
    /// 设置 map→odom 累计修正预算的阈值（启动与复位时由 Localization 注入）。
    void setCorrectionAnchorConfig(const CorrectionAnchorConfig& config) {
        correction_anchor_.configure(config);
    }
    /// 返回累计修正预算状态，供诊断与状态打印。
    const CorrectionAnchor& correctionAnchor() const { return correction_anchor_; }
    /// 候选 map→odom 是否仍在"相对锚点"的累计预算内。
    bool acceptsCorrection(const Eigen::Matrix4d& T_map_odom) const {
        return correction_anchor_.accepts(T_map_odom);
    }
    /// 裁决候选（预算内 / 一致证据放行 / 拒绝）；放行时提交端必须随后重落锚。
    AnchorDecision judgeCorrection(const Eigen::Matrix4d& T_map_odom) const {
        return correction_anchor_.judge(T_map_odom);
    }
    /// 记录一次已提交的修正；只有"安静"的修正才让锚点前移，`released` 时直接跟随。
    void noteCorrection(const Eigen::Matrix4d& T_map_odom, const double applied_rotation_rad,
                        const double applied_translation, const bool released = false) {
        correction_anchor_.observe(T_map_odom, applied_rotation_rad, applied_translation,
                                   released);
    }
    /// 人工重定位提交成功后强制重落锚（显式声明"我知道车在哪"）。
    void reseatCorrectionAnchor(const Eigen::Matrix4d& T_map_odom) {
        correction_anchor_.reseat(T_map_odom);
    }
    /// 推进输出版本，使旧预测快照失效。
    void advanceRevision() { ++pose_revision_; }
    /// 返回最后可信传感器位姿的只读投影状态。
    const TrustedPoseProjector& trusted() const { return trusted_pose_; }
    /// 返回最后可信输出是否具有有效协方差。
    bool hasTrustedCovariance() const { return trusted_covariance_.has_value(); }
    /// 返回最后可信协方差；调用方须先确认其有效。
    const PoseCovariance6d& trustedCovariance() const { return *trusted_covariance_; }
    /// 投影同步帧并推进可信缓存及严格单调的 TF 发布水位。
    LocalizationProjection project(const FrontendFrame& frame, const Timestamp& stamp,
                                   const LocalizationStateSnapshot& state,
                                   std::uint64_t active_epoch);
    /// 根据不可变状态投影高频位姿及协方差，不改变缓存。
    LocalizationProjection projectForOutput(const Eigen::Matrix4d& T_odom_sensor,
                                            const PoseCovariance6d& frontend_covariance,
                                            bool frontend_covariance_valid, double timestamp,
                                            std::uint64_t epoch,
                                            const LocalizationStateSnapshot& state,
                                            std::uint64_t active_epoch) const;
    /// 验收图边缘协方差并安装对应关键帧的传播锚。
    void refreshAnchor(const Keyframe& latest, const PoseGraph& graph);
    /// 提交当前可信同步帧的全局位姿和传播协方差。
    void commitTrusted(const FrontendFrame& latest, std::uint64_t active_epoch, bool trusted);

private:
    /// 由同代次图锚或前端协方差填充当前输出不确定度。
    void populateCovariance(const Eigen::Matrix4d& T_odom_sensor,
                            const PoseCovariance6d& frontend_covariance,
                            bool frontend_covariance_valid, std::uint64_t epoch, double timestamp,
                            LocalizationProjection& projection) const;
    OdometryProcessNoiseConfig noise_;
    Eigen::Matrix4d map_from_odom_ = Eigen::Matrix4d::Identity();
    /// map→odom 的相对锚点：拦住"每帧都合规、累计转过去"的积分饱和。
    CorrectionAnchor correction_anchor_;
    /// 图关键帧或停车参考帧的协方差传播锚，与最新可信输出拥有独立时间身份。
    struct GraphAnchor {
        PoseCovariance6d covariance = PoseCovariance6d::Zero();
        std::uint64_t epoch = 0;
        double timestamp = 0.0;
        Eigen::Matrix4d map_pose = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d odom_pose = Eigen::Matrix4d::Identity();
    };
    std::optional<GraphAnchor> graph_anchor_;
    TrustedPoseProjector trusted_pose_;
    std::optional<PoseCovariance6d> trusted_covariance_;
    std::uint64_t pose_revision_ = 0;
    std::uint64_t last_tf_epoch_ = 0;
    Timestamp last_tf_stamp_;
};
}  // namespace backend
