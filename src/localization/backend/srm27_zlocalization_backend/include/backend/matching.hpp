/**
 * @file matching.hpp
 * @brief 定义只读地图跟踪与恢复计算接口。
 */
#pragma once

#include <utility>

#include "backend/config.hpp"
#include "backend/work.hpp"

namespace backend {
struct LoopVerificationBatch;

/// 先验跟踪只读取源累计、目标选择、配准及诊断配置。
struct TrackingMatcherConfig {
    TrackingConfig tracking;
    RegistrationConfig registration;
    bool debug_enabled = false;
};

/// 恢复计算只读取恢复准入、候选检索、配准及诊断配置。
struct RecoveryMatcherConfig {
    RecoveryConfig recovery;
    RelocalizationConfig relocalization;
    bool debug_enabled = false;
};

/// 计算端只有配置、地图和几何缓存；没有图/状态机/会话锁访问权。
class TrackingMatcher {
public:
    /// 绑定只读先验地图、目标缓存及点面配准器。
    TrackingMatcher(TrackingMatcherConfig config, const KeyframeMap& prior,
                    RelocalizationTargetCache& targets);
    /// 准备锁外匹配所需的因果源及目标快照。
    void prepare(GlobalObservationWork& work) const;
    /// 计算并返回当前工作流的几何观测结果。
    PriorMapObservation compute(GlobalObservationWork& work);

private:
    /// 按当前工作流的因果规则准备配准源点云。
    bool prepareSource(GlobalObservationWork& work) const;
    /// 按距离与切换滞回选择先验地图参考关键帧。
    void selectReference(GlobalObservationWork& work) const;
    /// 按似然有效性及物理幅度分类点面观测，并构造地图系测量。
    void classifyObservation(const GlobalObservationWork& work,
                             PriorMapObservation& observation) const;
    const TrackingMatcherConfig config_;
    const KeyframeMap& prior_map_;
    RelocalizationTargetCache& target_cache_;
    Registration corrector_;
};

enum class RecoveryOutcome { Deferred, Matched, Failed };
struct RecoveryResult {
    RecoveryOutcome outcome = RecoveryOutcome::Deferred;
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    PoseCovariance6d covariance = PoseCovariance6d::Zero();
    std::string source;
    Cloud::Ptr debug_aligned_source;
    Cloud::Ptr debug_target;
};

class RecoveryMatcher {
public:
    /// 绑定恢复计算所需的只读地图、描述子及目标缓存。
    RecoveryMatcher(RecoveryMatcherConfig config, const KeyframeMap& prior,
                    const PlaceDescriptor& descriptor, RelocalizationTargetCache& targets)
        : config_(std::move(config)),
          prior_map_(prior),
          descriptor_(descriptor),
          target_cache_(targets) {}
    /// 准备锁外匹配所需的因果源及目标快照。
    void prepare(RecoveryWork& work) const;
    /// 计算并返回当前工作流的几何观测结果。
    RecoveryResult compute(const RecoveryWork& work);

private:
    /// 执行单个位置先验或描述子恢复分支并保留完整可观位姿结果。
    bool match(const RecoveryWork& work, const RelocalizationPositionPrior* prior,
               RecoveryResult& result);
    const RecoveryMatcherConfig config_;
    const KeyframeMap& prior_map_;
    const PlaceDescriptor& descriptor_;
    RelocalizationTargetCache& target_cache_;
};
/// 报告跟踪观测、创新检验和最终图提交的诊断结果。
void reportTracking(const GlobalObservationWork&, const PriorMapObservation&,
                    const GlobalObservationCommit&, const Config&, double measurement_ms);
/// 报告已接受回环的创新统计和全局校正幅度。
void reportLoop(const LoopWork&, const LoopVerificationBatch&, const LoopCommitSummary&);
}  // namespace backend
