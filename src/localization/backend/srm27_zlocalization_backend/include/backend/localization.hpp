/**
 * @file localization.hpp
 * @brief 协调同步输入、维护任务和唯一状态提交边界。
 */
#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <variant>
#include <vector>

#include "backend/config.hpp"
#include "backend/descriptor.hpp"
#include "backend/frame_history.hpp"
#include "backend/loop.hpp"
#include "backend/matching.hpp"
#include "backend/ports.hpp"
#include "backend/pose_graph.hpp"
#include "backend/pose_output.hpp"
#include "backend/reset_coordinator.hpp"
#include "backend/uwb_fusion.hpp"

namespace backend {
struct GlobalObservationTransaction;
/// 表示估计器已隔离失败事务并发布冻结的 LOST 输出。
class EstimationFailure final : public std::runtime_error {
public:
    /// 保存已隔离的估计故障原因。
    explicit EstimationFailure(const std::string& reason) : std::runtime_error(reason) {}
};
/// 统一输入、任务调度与状态提交；几何计算由只读匹配器完成。
class Localization {
public:
    /// 按描述子能力解析请求配置，并绑定地图及外部端口。
    Localization(KeyframeMap& trajectory, KeyframeMap& prior_map, PlaceDescriptor& prior_descriptor,
                 PlaceDescriptor& trajectory_descriptor,
                 Config requested_config, SteadyClockPort& clock, FrontendResetPort& reset_port,
                 StateSink& state_sink, VisualizationSink& visualization_sink);

    /// 使用已解析的有效配置绑定地图、图估计及外部端口。
    Localization(KeyframeMap& trajectory, KeyframeMap& prior_map, PlaceDescriptor& prior_descriptor,
                 PlaceDescriptor& trajectory_descriptor,
                 EffectiveConfig effective_config, SteadyClockPort& clock,
                 FrontendResetPort& reset_port, StateSink& state_sink,
                 VisualizationSink& visualization_sink);

    /// 初始化 epoch 0。启动提示只提供初值：先验图负责几何验证，UWB-only 等待
    /// 可观运动，二者都没有时直接选择里程计 gauge。
    void initialize();

    /// 接入同步前端帧；旧 epoch、无效数据或当前 epoch 内时间回退时返回 false，
    /// 调用方不得发布或投影被拒绝的帧。
    bool ingestFrame(const FrontendFrame& frame, const Eigen::Vector3d& linear_velocity_sensor,
                     const Eigen::Vector3d& angular_velocity_sensor);
    /// 接入 UWB 天线位置；只有新的物理解返回 true。无效、乱序和保持重发返回
    /// false，避免诊断输出用新时间戳重标旧解。
    bool ingestUwbPosition(double timestamp, const Eigen::Vector3d& position_uwb_antenna);
    /// 返回最新原始 UWB 解所代表的物理时刻；在线估计开启时使用当前共享 dt
    /// 后验。
    std::optional<double> latestUwbMotionTimestamp() const;
    /// 用时间对齐的前端姿态消除天线杆臂并返回机体原点；仅供诊断，不进入图。
    std::optional<Eigen::Vector3d> latestUwbBodyPositionMap() const;

    /// 执行一次有界维护周期；事件数据由帧回调捕获，本函数按状态运行跟踪、恢复或
    /// 一次回环验证。
    void runMaintenanceCycle();

    /// 在输出提交边界投影同步帧并刷新可信缓存。
    LocalizationProjection project(const FrontendFrame& frame, const Timestamp& stamp);
    /// 从不可变输出视图投影高频前端位姿及协方差。
    LocalizationProjection projectForOutput(const Eigen::Matrix4d& T_odom_sensor,
                                            const PoseCovariance6d& frontend_covariance,
                                            bool frontend_covariance_valid, double timestamp,
                                            std::uint64_t epoch) const;
    /// 在 ROS 发布边界重新校验锁外输出快照是否仍属于当前状态和位姿版本。
    bool projectionIsCurrent(const LocalizationProjection& projection) const;

    /// 在 LOST 状态记录等待下一有效帧的手动恢复请求。
    bool requestManualRelocalization();
    /// 冻结定位并请求协调重置；仅调用方明确保证整个跨代空档停车时允许静止衔接。
    bool requestCoordinatedReset(bool assume_stationary = false);
    /// 为匹配代次的前端输入中断冻结输出并作废旧观测，幂等尝试重置且保留图和轨迹。
    void notifyFrontendDiscontinuity(std::uint64_t expected_epoch);
    /// 锁存匹配代次的前端普通失败并冻结输出，等待显式重置或新会话协调。
    void notifyFrontendFailure(std::uint64_t expected_epoch);
    /// 每次请求捕获的身份校验在事务锁内执行且不得重入，不以最近一次请求代替它。
    void onFrontendResetResponse(FrontendResetReply reply,
                                 const std::function<bool(FrontendResetReply&)>& translate = {});

    /// 返回同一提交版本下的定位状态、前端代次和位姿版本。
    LocalizationSnapshot snapshot() const;
    /// 从不可变输出视图读取当前定位状态。
    LocalizationStatus status() const;
    /// 从不可变输出视图读取当前前端代次。
    std::uint64_t activeEpoch() const;

    /// 捕获回环边供轨迹可视化发布。
    VisualizationSnapshot visualization() const;

private:
    /// 区分正常输入、可协调中断及只能显式恢复的普通失败。
    enum class FrontendStatus { Active, Interrupted, Failed };
    /// 在停车重置合同下连接旧图节点与新代首个关键帧。
    struct PendingResetContinuity {
        int from_index = -1;
        std::uint64_t target_epoch = 0;
        Eigen::Matrix4d measured_between = Eigen::Matrix4d::Identity();
        PoseCovariance6d covariance = PoseCovariance6d::Zero();
    };
    /// 保存旧关键帧到参考帧的测量，以及参考帧在当前里程计坐标中的位置和时间。
    struct PendingResetOdometry {
        int from_index = -1;
        std::uint64_t target_epoch = 0;
        /// 参考帧的传感器时刻，单位为秒。
        double reference_timestamp = 0;
        /// T_odom_reference，参考帧在当前代次里程计系中的位姿。
        Eigen::Matrix4d reference_odom = Eigen::Matrix4d::Identity();
        /// T_previous_reference，将参考帧坐标变换到旧关键帧坐标。
        Eigen::Matrix4d from_reference = Eigen::Matrix4d::Identity();
        /// 前缀测量的局部右扰动协方差；空值表示参考帧就是旧关键帧。
        std::optional<PoseCovariance6d> prefix_covariance;
    };
    /// 首个图关键帧只接受独立规范先验或跨代连续性边之一。
    using EpochAnchor = std::variant<std::monostate, PoseCovariance6d, PendingResetContinuity,
                                     PendingResetOdometry>;
    /// 固定新里程计到地图的坐标关系，并限定重置后的首帧时间。
    struct EpochTransform {
        Eigen::Matrix4d map_from_odom;
        Timestamp boundary_stamp;
    };
    /// 首同步帧建立新里程计规范，锚约束随后交给首个图关键帧。
    struct EpochSeed {
        std::variant<Eigen::Matrix4d, EpochTransform> origin = Eigen::Matrix4d::Identity().eval();
        bool trusted = false;
        EpochAnchor anchor;
    };

    /// 保存一次完整提交的位姿、状态和代次。
    struct OutputView {
        PoseOutput poses;
        LocalizationStateSnapshot state;
        std::uint64_t epoch;
        bool odometry_connected;
    };
    /// 在锁内安装状态、位姿和诊断原因一致的不可变输出视图。
    void publishOutputViewLocked();
    /// 丢弃失败估计事务并以最后已提交的可信输出建立恢复边界。
    void isolateEstimatorFailureLocked(const std::string& reason);
    /// 在锁内提交完整输出；数值求解失败时隔离事务并建立 LOST 恢复边界。
    template <class Operation>
    auto withOutputCommit(Operation&& operation) {
        std::unique_lock<std::mutex> lock(mutex_);
        try {
            if constexpr (std::is_void_v<std::invoke_result_t<Operation>>) {
                operation();
                publishOutputViewLocked();
            } else {
                auto result = operation();
                publishOutputViewLocked();
                return result;
            }
        } catch (const NumericalError& error) {
            const std::string reason =
                std::string("estimator numerical failure; runtime map discarded: ") + error.what();
            isolateEstimatorFailureLocked(reason);
            throw EstimationFailure(reason);
        }
    }
    /// 顺序完成维护任务的捕获、准备、身份认领、锁外计算和提交。
    template <class Work, class Matcher>
    void execute(std::optional<Work> (Localization::*capture)(), Matcher& matcher);
    /// 冻结当前可信关键帧及其全局观测所需的因果输入。
    std::optional<GlobalObservationWork> captureTracking();
    /// 以里程计路径长度估计车体角速度，并维护带迟滞的自转闸门。
    /// 调用方须持有 mutex_。
    bool bodyRateGateLocked();
    /// 冻结当前恢复任务的身份、源历史及位置先验。
    std::optional<RecoveryWork> captureRecovery();
    /// 按冷却与缓存身份捕获回环候选及完整轨迹快照。
    std::optional<LoopWork> captureLoop();
    /// 复核跟踪任务的前端与状态代次，并记录所选参考关键帧。
    bool claim(GlobalObservationWork& work);
    /// 复核恢复身份与最新源帧，并按节流规则认领任务。
    bool claim(RecoveryWork& work);
    /// 原子提交全局观测的联合创新结论、图因子和定位状态。
    GlobalObservationCommit commit(const GlobalObservationWork& work,
                                   const PriorMapObservation& result);
    /// 校验恢复任务身份并提交完整位姿锚或累计恢复失败。
    void commit(const RecoveryWork& work, const struct RecoveryResult& result);
    /// 在已校验身份的事务内安装恢复锚、定位状态及当前可信输出。
    /// @param released 本次恢复是累计锚点"一致证据放行"换来的：提交后强制重落锚。
    PoseCorrectionDelta applyRecoveryLocked(const RecoveryWork& work, const RecoveryResult& result,
                                            LocalizationTransition& transition, bool released);
    /// 将通过联合创新检验的回环写入图及可视化历史。
    LoopCommitSummary commit(const LoopWork& work, const LoopVerificationBatch& result);

    /// 在线性化和联合统计有效时验收回环，并返回完整创新诊断。
    bool assessLoopLocked(const LoopWork& work, const VerifiedLoop& verified,
                          LoopCommitSummary& summary) const;
    /// 按最小周期推进调度水位，时钟回退时重建节流基准。
    static bool tryStartCycle(const Timestamp& now, const double target_period_sec,
                              Timestamp& last_cycle_time);

    /// 建立 SUSPECT 恢复源边界并重启必要的 UWB 启动配对。
    void beginSuspectRecoveryLocked();

    /// 收集尚未消费且可线性化的全局观测及不可用原因。
    void collectGlobalInnovationsLocked(const GlobalObservationWork& work,
                                        const PriorMapObservation& prior_observation,
                                        GlobalObservationTransaction& transaction) const;
    /// 执行共同预测下的联合创新检验并完成有效 UWB 观测消费。
    void assessGlobalInnovationsLocked(const GlobalObservationWork& work,
                                       GlobalObservationTransaction& transaction,
                                       GlobalObservationCommit& commit);
    /// 插入已通过创新检验的因子并更新图及在线标定。
    void applyGlobalObservationFactorsLocked(const GlobalObservationWork& work,
                                             const PriorMapObservation& prior_observation,
                                             GlobalObservationTransaction& transaction,
                                             GlobalObservationCommit& commit);
    /// 根据同周期接受、冲突及不可用证据推进定位状态。
    void updateGlobalObservationStateLocked(GlobalObservationTransaction& transaction,
                                            GlobalObservationCommit& commit);

    /// 捕获回环边并发布本次配准源与目标的诊断快照。
    void publishRegistrationDebug(const Cloud::Ptr& aligned_source, const Cloud::Ptr& target);
    /// 经外部端口发布当前已提交的定位状态快照。
    void publishState();
    /// 校验恢复任务的代次、状态及请求身份，不要求捕获源仍为最新帧。
    bool recoveryIsCurrentLocked(const RecoveryIdentity& identity) const;
    /// 确认认领任务时的源仍对应最新同步帧。
    bool sourceIsLatestLocked(const RecoveryWork& work) const;
    /// 确认同步帧严格晚于手动请求捕获的输入边界。
    bool manualRequestAllowsFrameLocked(const FrontendFrame& frame) const;
    /// 将时间对齐的 UWB 天线位置及杆臂构造成恢复位置先验。
    std::optional<RelocalizationPositionPrior> makeUwbRecoveryPriorLocked(
        const FrontendFrame& frame) const;
    /// 在第零代初始化期间构造待几何验证的启动提示。
    std::optional<RelocalizationPositionPrior> makeStartupHintPriorLocked(
        const FrontendFrame& frame) const;
    /// 预估一次恢复提交相对当前冻结输出的位姿变化；不改变任何状态。
    std::optional<PoseCorrectionDelta> prospectiveRecoveryCorrectionLocked(
        const RecoveryWork& work, const RecoveryResult& result) const;
    /// 判断恢复提交幅度是否落在预算内；判据见 backend/recovery_budget.hpp。
    bool recoveryCorrectionWithinBudgetLocked(const PoseCorrectionDelta& delta) const;

    /// 清除已经完成或失效的手动恢复请求。
    void clearManualRelocalizationRequestLocked();
    /// 保存可信输出并隔离失效前端，撤销旧输入及在途停车衔接资格。
    void isolateFrontendLocked(FrontendStatus status, const char* reason);
    /// 完成服务预检并在事务锁内发送幂等前端重置请求。
    bool triggerFrontendReset(bool force_lost = false,
                              std::optional<std::uint64_t> expected_epoch = std::nullopt,
                              bool assume_stationary = false);
    /// 在超时后以同一业务身份重发仍在途的重置事务。
    void expireFrontendResetIfTimedOut();
    /// 恢复手动重置前状态，并开放尚未提交观测的重新调度。
    void rollbackRejectedResetLocked();
    /// 根据重置坐标边界或停车合同建立新代规范及互斥图锚。
    void prepareNextEpochSeedLocked(const FrontendResetReply& reply);
    /// 从旧图关键帧或待提交的连续边构造新代里程计锚。
    EpochAnchor resetOdometryAnchorLocked(const FrontendResetBoundary& boundary,
                                          std::uint64_t target_epoch) const;
    /// 根据停车位姿构造旧图连续边，缺少前驱时使用可信协方差先验。
    EpochAnchor resetStationaryAnchorLocked(const Eigen::Matrix4d& pose,
                                            std::uint64_t target_epoch) const;
    /// 将停车种子绑定到实际首帧，保存后续运动所需的相对测量和全局协方差参考。
    void bindResetAnchorLocked(const FrontendFrame& frame);
    /// 推进前端代次并清理依赖旧里程计规范的运行窗口。
    void commitFrontendResetBoundaryLocked(const FrontendResetReply& reply);

    /// 根据关键帧间时间、平移和转角计算里程计过程协方差。
    PoseCovariance6d odometryFactorCovariance(const Keyframe& previous,
                                              const FrontendFrame& current) const;
    /// 为启用回环的有效关键帧建立地点描述子。
    void addDescriptorLocked(const Keyframe& keyframe);
    /// 将图优化后的关键帧位置同步到描述子检索索引。
    void updateDescriptorPositionsLocked(const std::vector<std::pair<int, Eigen::Matrix4d>>& poses);
    /// 提交图因子并刷新轨迹、共享标定及可信输出协方差锚。
    PoseCorrectionDelta updateGraphLocked(const bool global_factor_added);
    /// 将当前代次的最新同步帧提交为可信输出位姿。
    void commitCurrentTrustedPoseLocked();
    /// 为无先验地图的首个可信同步帧提交全局规范及状态说明。
    bool startTrajectoryLocked(const FrontendFrame& frame, bool applied_trusted_seed);
    /// 创建轨迹关键帧并消费本代首个图锚或追加里程计约束。
    void addTrajectoryKeyframeLocked(const FrontendFrame& frame);
    /// 为当前关键帧选择并加入唯一的里程计边、跨代边或独立先验。
    void addTrajectoryConstraintLocked(const FrontendFrame& frame, const Keyframe& keyframe,
                                       const std::optional<Keyframe>& previous,
                                       const Eigen::Matrix4d& initial_map_pose);
    /// 以恢复观测建立独立全位姿锚，并按需验收附加 UWB 因子。
    void anchorRecoveredFrameLocked(const FrontendFrame& frame, const Eigen::Matrix4d& T_map_sensor,
                                    const PoseCovariance6d& covariance,
                                    std::vector<FrontendFrame> tracking_source_history,
                                    const bool add_uwb_factor = true);
    /// 验收可观的 UWB 启动估计并将其协方差传播到恢复位姿锚。
    bool tryCommitUwbBootstrapLocked(const FrontendFrame& frame);
    KeyframeMap& trajectory_;
    KeyframeMap& prior_map_;
    // 先验检索库只读；运行回环库随轨迹增长、优化和故障隔离单独维护。
    PlaceDescriptor& prior_descriptor_;
    PlaceDescriptor& trajectory_descriptor_;
    Config config_;
    SteadyClockPort& clock_;
    FrontendResetPort& reset_port_;
    StateSink& state_sink_;
    VisualizationSink& visualization_sink_;
    ResetCoordinator reset_coordinator_;

    PoseGraph graph_;
    RelocalizationTargetCache target_cache_;
    LocalizationStateMachine state_;
    FrameHistory frames_;
    PoseOutput poses_;
    UwbFusion uwb_;

    mutable std::mutex mutex_;
    std::shared_ptr<const OutputView> output_view_;
    struct ManualRequest {
        std::uint64_t id = 0;
        std::uint64_t epoch = 0;
        double after_timestamp = std::numeric_limits<double>::quiet_NaN();
    };
    struct EmptyLoopIdentity {
        int index = -1;
        std::uint64_t trajectory_revision = 0;
        std::uint64_t descriptor_revision = 0;
    };
    std::optional<EpochSeed> next_epoch_seed_;
    EpochAnchor pending_epoch_anchor_;

    double last_recovery_timestamp_ = -std::numeric_limits<double>::infinity();
    /// 自转闸门的锁定期截止时间（秒，前端时间轴）；0 = 未锁定。
    double body_rate_gate_until_ = 0.0;
    /// 闸门锁定所对应的前端代次；换代时时间轴可能重启，必须清零截止时间。
    std::uint64_t body_rate_gate_epoch_ = 0;
    int last_lidar_observation_runtime_index_ = -1;
    Timestamp last_global_observation_cycle_time_;
    Timestamp last_recovery_cycle_time_;
    int last_tracking_reference_index_ = -1;
    /// prior_map.initialization_only：首次先验匹配已验收，此后不再产生先验观测。
    bool prior_initialization_only_done_ = false;
    /// 一次性先验锁存后进入恢复状态只告警一次，避免 5 Hz 刷屏。
    bool prior_initialization_only_recovery_warned_ = false;

    std::optional<EmptyLoopIdentity> empty_loop_;
    int last_loop_checked_index_ = -1;
    int last_accepted_loop_index_ = -1;
    std::vector<AcceptedLoop> accepted_loops_;
    std::vector<std::pair<int, int>> loop_edges_;

    std::uint64_t next_manual_relocalization_request_id_ = 0;
    std::optional<ManualRequest> manual_request_;
    std::optional<LocalizationStateMachine> state_before_manual_reset_;
    bool estimator_failed_ = false;
    /// 当前里程计规范是否仍与已建立的全局轨迹相连。
    bool odometry_connected_ = false;
    /// 当前已发送事务是否获得调用方对完整空档停车的明确保证。
    bool stationary_reset_ = false;
    /// 记录本代次已确认的输入健康状态，重复通知不撤销后续显式授权。
    FrontendStatus frontend_status_ = FrontendStatus::Active;
    std::uint64_t active_epoch_ = 0;

    TrackingMatcher tracking_;
    RecoveryMatcher recovery_;
    LoopMatcher loop_cycle_;
    std::atomic<bool> maintenance_running_{false};
};
}  // namespace backend
