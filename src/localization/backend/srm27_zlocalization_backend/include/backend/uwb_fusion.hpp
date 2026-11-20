/**
 * @file uwb_fusion.hpp
 * @brief 声明 UWB 观测、时间关联和融合生命周期接口。
 */
#pragma once

#include <Eigen/Core>
#include <optional>

#include "backend/config.hpp"
#include "backend/factors.hpp"
#include "backend/frame_history.hpp"
#include "backend/types.hpp"
#include "backend/uwb_alignment.hpp"

namespace backend {
class PoseGraph;

/// UWB 输入验收所需的固定能力和传输策略。
struct UwbObservationStreamConfig {
    bool enabled = false;
    bool use_z = false;
    double effective_rate_hz = 0.0;
};

/// 已通过有限值、顺序和重复检查的原始 UWB 天线位置。
struct UwbObservation {
    /// 原始消息时间戳；用于顺序、保持样本去重和因子唯一性。
    double timestamp = 0.0;
    Eigen::Vector3d position_uwb = Eigen::Vector3d::Zero();
};

/// 将高频传输重发收敛为按物理解更新的最新值流。
class UwbObservationStream {
public:
    /// 保存 UWB 有效维度及保持重发验收策略。
    explicit UwbObservationStream(const UwbObservationStreamConfig& config) : config_(config) {}

    /// 接受一个新的物理解；无效、明显乱序或过快重复的保持样本返回 false。
    bool ingest(double timestamp, const Eigen::Vector3d& position_uwb_antenna);

    /// 清除最新原始物理解及其去重身份。
    void clear() { latest_.reset(); }
    /// 返回最新已验收值；调用方须先确认该值存在。
    const std::optional<UwbObservation>& latest() const { return latest_; }

private:
    UwbObservationStreamConfig config_;
    std::optional<UwbObservation> latest_;
};

using UwbFactorInput = UwbMeasurement;

/// UWB 因子构造所需的固定物理参数和噪声模型。
struct UwbFactorBuilderConfig {
    double odom_sync_tolerance = 0.25;
    double time_jitter_std_seconds = 0.0;
    Eigen::Vector3d antenna_position_sensor = Eigen::Vector3d::Zero();
    Eigen::Matrix3d position_covariance = Eigen::Matrix3d::Identity();
    OdometryProcessNoiseConfig odometry_noise;
};

/// 单次构造的有效能力状态；这些值由控制器在锁内冻结。
struct UwbFactorBuildMode {
    double time_offset_seconds = 0.0;
    bool estimate_time_offset = false;
    bool time_offset_state_available = false;
    bool with_alignment = false;
};

/// 单次构造使用的只读输入快照。
struct UwbFactorBuildContext {
    const UwbObservation& observation;
    const Keyframe& keyframe;
    const TimedPoseBuffer& odometry;
    const Eigen::Matrix4d& T_map_uwb;
    UwbFactorBuildMode mode;
};

/// 统一实现 UWB 时间关联、杆臂投影和测量协方差传播。
class UwbFactorBuilder {
public:
    /// 绑定时间同步、杆臂和不确定度传播的固定参数。
    explicit UwbFactorBuilder(const UwbFactorBuilderConfig& config) : config_(config) {}

    /// 构造因子输入；时间支撑不足或任一中间量无效时返回空。
    std::optional<UwbFactorInput> build(const UwbFactorBuildContext& context) const;

    /// 将原始 UWB 天线位置变换到 map 坐标系。
    Eigen::Vector3d positionMap(const UwbObservation& observation,
                                const Eigen::Matrix4d& T_map_uwb) const;
    /// 将 UWB 解算坐标系中的位置协方差旋转到地图坐标系。
    Eigen::Matrix3d positionCovarianceMap(const Eigen::Matrix4d& T_map_uwb) const;

private:
    /// 因果查询 UWB 物理时刻，并把天线均值和速度投影到关键帧局部系。
    struct TimingProjection {
        Eigen::Matrix4d T_keyframe_measurement = Eigen::Matrix4d::Identity();
        Eigen::Vector3d antenna_in_keyframe = Eigen::Vector3d::Zero();
        Eigen::Vector3d residual_velocity = Eigen::Vector3d::Zero();
        std::optional<UwbTimeOffsetSupport> time_support;
        double keyframe_time_offset = 0.0;
    };

    /// 在物理测量时刻查询里程计并构造天线位置与速度支撑。
    std::optional<TimingProjection> projectMeasurementTime(
        const UwbFactorBuildContext& context) const;
    /// 用关键帧与测量时刻之间的前端运动传播附加位置方差。
    std::optional<double> propagatedPositionVariance(const TimingProjection& projection) const;

    UwbFactorBuilderConfig config_;
};

/// UWB 生命周期所需的物理参数、过程噪声和观测接纳策略。
struct UwbFusionConfig {
    UwbConfig uwb;
    OdometryProcessNoiseConfig odometry_noise;
    double rejection_probability = 1.0e-3;
    bool prior_map_enabled = false;
};

/// UWB 流、共享标定和因子生产的唯一所有者；由会话锁串行访问。
class UwbFusion {
public:
    /// 绑定 UWB 输入流、图估计及里程计时间支撑并初始化生命周期。
    UwbFusion(UwbFusionConfig config, PoseGraph& graph, const TimedPoseBuffer& odometry);
    /// 完整重置 UWB 物理解、因子消费水位及所有标定和启动窗口。
    void resetAll();
    /// 清除图标定与配对窗口，保留原始观测身份和因子消费水位。
    void resetGraph();
    /// 清理依赖旧里程计的配对窗口，保留最新物理解及因子消费水位。
    void resetEpoch();
    /// 清空运动配对，并以给定的 odom 到 map 变换重新启动对齐。
    void restartBootstrap(const Eigen::Matrix4d& initial_guess);
    /// 设置 UWB 启动对齐的数值初值，不改变观测消费水位。
    void seedBootstrap(const Eigen::Matrix4d& seed) { uwb_bootstrap_.setInitialGuess(seed); }
    /// 验收新的 UWB 物理解，保留原始时间戳用于唯一性判断。
    bool ingest(double stamp, const Eigen::Vector3d& position);
    /// 返回最新原始物理解的只读输入流。
    const UwbObservationStream& observations() const { return uwb_observations_; }
    /// 返回已完成统计评估或被启动估计概括的物理解时间水位。
    double lastConsumed() const { return last_uwb_factor_timestamp_; }
    /// 统计评估有效后消费该物理解，接受与拒绝使用相同单调水位。
    void finishAssessment(const UwbFactorInput& input, const InnovationAssessment& assessment);
    /// 消费已被稳定启动估计概括的全部物理解。
    void finishBootstrap(const UwbAlignmentEstimate& estimate);
    /// 将原始 UWB 天线位置变换到地图坐标系。
    Eigen::Vector3d positionMap(const UwbObservation& observation) const {
        return uwb_factor_builder_.positionMap(observation, map_from_uwb_);
    }
    /// 配对未消费的 UWB 与里程计运动，并返回稳定可观的启动估计。
    std::optional<UwbAlignmentEstimate> bootstrap(const FrontendFrame& frame);
    /// 仅为新鲜且坐标标定就绪的物理解构造普通图因子。
    std::optional<UwbFactorInput> captureFresh(const Keyframe& keyframe) const;
    /// 在当前图预测处线性化给定 UWB 因子以供创新检验。
    InnovationTerm linearize(const int runtime_index, const UwbFactorInput& input) const;
    /// 对单个 UWB 因子执行图预测下的联合创新检验。
    InnovationAssessment assess(const int runtime_index, const UwbFactorInput& input) const;
    /// 将已验收的 UWB 原始位置因子插入当前图事务。
    void insert(const int runtime_index, const UwbFactorInput& input);
    /// 对恢复锚附近的新 UWB 观测完成创新检验、消费和可选因子插入。
    bool addToRecovery(const Keyframe& keyframe);
    /// 返回当前有效配置是否启用共享坐标标定。
    bool alignmentEstimationEnabled() const;
    /// 返回当前有效配置是否启用共享静态时间偏置估计。
    bool timeOffsetEstimationEnabled() const;
    /// 返回当前有效时间偏置后验，尚无后验时使用配置值。
    double timeOffset() const;
    /// 将原始解时间修正为其代表的物理运动时刻。
    double motionTimestamp(const UwbObservation& observation) const;
    /// 按需为图建立共享静态时间偏置及其先验。
    void ensureTimeOffsetState();
    /// 读取时间偏置后验，并按需计算和报告其不确定度。
    void refreshTimeOffset(const bool refresh_covariance);
    /// 返回当前 UWB 坐标对齐是否允许普通因子进入图。
    bool ready() const;
    /// 读取联合图中共享 UWB 坐标标定的当前后验。
    void refreshAlignment();
    /// 用独立地图位姿积累在线标定配对，并提交稳定内点的联合原始因子。
    bool observeAnchor(const Keyframe& reference, const Eigen::Matrix4d& T_map_reference);

private:
    /// 推进物理解消费水位，旧时刻和非有限值不改变状态。
    void consumeThrough(double timestamp);
    /// 构造最新物理解的原始因子；仅由普通准入和在线标定内部路径调用。
    std::optional<UwbFactorInput> buildLatestFactor(const Keyframe& keyframe) const;
    /// 清空依赖当前前端里程计规范的启动配对窗口。
    void clearBootstrapWindow();
    /// 清空在线坐标标定的配对与图引用窗口。
    void clearCalibrationWindow();
    /// 剔除已离图的标定配对并同步鲁棒估计器的活动窗口。
    void retainActiveCalibrationSamples();
    /// 将稳定标定及原始内点因子共同提交，并完成启动观测消费。
    void initializeAlignment(const UwbAlignmentEstimate& estimate);
    /// 在线 UWB 坐标对齐期间保留的图关键帧与原始因子对应关系。
    struct UwbAlignmentCalibrationSample {
        int runtime_index = -1;
        UwbFactorInput factor;
    };

    const UwbFusionConfig config_;
    PoseGraph& graph_;
    const TimedPoseBuffer& odom_history_;
    UwbObservationStream uwb_observations_;
    UwbFactorBuilder uwb_factor_builder_;
    UwbAlignmentBootstrapper uwb_bootstrap_;
    UwbAlignmentBootstrapper uwb_zero_pose_estimator_;
    Eigen::Matrix4d map_from_uwb_ = Eigen::Matrix4d::Identity();
    bool uwb_zero_pose_ready_ = true;
    double current_uwb_time_offset_seconds_ = 0.0;
    double last_reported_uwb_time_offset_seconds_ = 0.0;
    double last_uwb_factor_timestamp_ = -std::numeric_limits<double>::infinity();
    double last_uwb_bootstrap_timestamp_ = -std::numeric_limits<double>::infinity();
    double last_uwb_calibration_timestamp_ = -std::numeric_limits<double>::infinity();
    std::vector<UwbAlignmentCalibrationSample> uwb_alignment_calibration_samples_;
};
}  // namespace backend
