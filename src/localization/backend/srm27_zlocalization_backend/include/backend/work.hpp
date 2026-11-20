/**
 * @file work.hpp
 * @brief 定义维护任务跨捕获、计算与提交边界的值类型。
 */
#pragma once

#include <Eigen/Core>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "backend/factors.hpp"
#include "backend/frame_history.hpp"
#include "backend/localization_state.hpp"
#include "backend/loop_consistency.hpp"
#include "backend/registration.hpp"
#include "backend/search.hpp"
#include "backend/types.hpp"
#include "backend/uwb_fusion.hpp"

namespace backend {

/// 一次图优化对当前全局位姿产生的 SE(3) 修正幅度。
struct PoseCorrectionDelta {
    double translation = 0.0;
    double rotation_rad = 0.0;
};

/// 观测生产阶段的互斥结果，不包含后续联合创新检验结论。
enum class ObservationDisposition {
    Unavailable,
    ModelInvalid,
    Conflict,
    Measurement,
};

/// 与传感器类型无关的观测生产结论。
///
/// UNAVAILABLE 表示本周期没有健康证据；MODEL_INVALID 表示输入超出局部模型；
/// CONFLICT 表示合法测量在联合创新检验前已冲突；MEASUREMENT 携带待提交因子。
struct ObservationEvidence {
    bool configured = false;
    ObservationDisposition disposition = ObservationDisposition::Unavailable;
    std::string reason;
};

/// 普通全局观测周期的先验图匹配输入与已捕获 UWB 因子。
struct GlobalObservationWork {
    Keyframe runtime;
    std::uint64_t state_generation = 0;
    TrackingSourceSnapshot source_snapshot;
    bool source_snapshot_missing = false;
    CloudBlock fallback_source;
    int previous_reference_index = -1;
    Keyframe reference;
    Cloud::Ptr source;
    int accumulated_frames = 0;
    double source_span_seconds = 0.0;
    Eigen::Matrix4d predicted_map_pose = Eigen::Matrix4d::Identity();
    Cloud::ConstPtr target;
    /// 捕获时可用的新鲜 UWB 因子；提交前再次核对消费水位。
    std::optional<UwbFactorInput> uwb;
    ObservationEvidence prior_input;
};

/// 锁外 scan-to-map 计算产生的先验图观测候选。
struct PriorMapObservation {
    ObservationEvidence evidence;
    RegistrationResult registration;
    Eigen::Matrix4d measured_map_pose = Eigen::Matrix4d::Identity();
    double predicted_translation_delta = 0.0;
    double predicted_rotation_delta_rad = 0.0;
    double elapsed_ms = 0.0;
    Cloud::Ptr debug_aligned_source;
    Cloud::Ptr debug_target;
};

/// NORMAL 维护周期完成原子提交后的系统结论。
struct GlobalObservationCommit {
    bool stale = false;
    bool graph_node_unavailable = false;
    bool prior_accepted = false;
    bool uwb_accepted = false;
    bool had_conflict = false;
    bool model_invalid = false;
    PoseCorrectionDelta applied_correction;
    InnovationAssessment joint_innovation;
    InnovationAssessment prior_innovation;
    InnovationAssessment uwb_innovation;
    LocalizationTransition transition;
};

/// 跨捕获、认领及提交阶段保持不变的恢复任务身份。
struct RecoveryIdentity {
    std::uint64_t epoch = 0;
    LocalizationStatus status = LocalizationStatus::Initializing;
    std::uint64_t state_generation = 0;
    std::uint64_t reset_transaction_id = 0;
    std::uint64_t manual_request_id = 0;
};

/// INITIALIZING、SUSPECT 或 LOST 恢复周期的不可变输入快照。
struct RecoveryWork {
    std::vector<Eigen::Vector3d> prior_positions;
    Cloud::ConstPtr fine_source;
    Cloud::ConstPtr coarse_source;
    FrontendFrame frame;
    Keyframe runtime;
    RecoveryIdentity identity;
    Eigen::Matrix4d predicted_map_pose = Eigen::Matrix4d::Identity();
    std::optional<RelocalizationPositionPrior> startup_hint_prior;
    std::optional<RelocalizationPositionPrior> uwb_prior;
    std::vector<FrontendFrame> source_history;
    int accumulated_frames = 0;
    double source_span_seconds = 0.0;
    /// 恢复源与普通传感器观测使用同一证据语义：历史尚未形成时为
    /// UNAVAILABLE，连续性门拒绝已有历史时为 MODEL_INVALID，可执行匹配时为
    /// MEASUREMENT。
    ObservationEvidence source_evidence;
    bool force_startup_hint = false;
};

/// 单次回环候选验证所使用的轨迹和候选快照。
struct LoopWork {
    Keyframe latest;
    std::uint64_t state_generation = 0;
    std::vector<Keyframe> trajectory;
    std::vector<LoopCandidate> candidates;
    std::vector<AcceptedLoop> accepted_loops;
    bool descriptor_level_with_odom_roll_pitch = false;
};

/// 回环观测完成图提交后供发布与日志使用的摘要。
struct LoopCommitSummary {
    bool accepted = false;
    int target_index = -1;
    PoseCorrectionDelta applied_correction;
    InnovationAssessment innovation;
    InnovationAssessment joint_innovation;
};

}  // namespace backend
