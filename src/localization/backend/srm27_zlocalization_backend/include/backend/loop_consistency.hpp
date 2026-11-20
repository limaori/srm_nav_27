/**
 * @file loop_consistency.hpp
 * @brief 回环闭合验证、PCM 一致性和候选选择。
 */
#pragma once

#include <Eigen/Dense>
#include <functional>
#include <limits>
#include <string>
#include <vector>

#include "backend/config.hpp"
#include "backend/math.hpp"
#include "backend/measurement.hpp"
#include "backend/types.hpp"

namespace backend {

/// 已接受的回环约束（PCM 一致性校验用）。
struct AcceptedLoop {
    int from = -1;
    int to = -1;
    Eigen::Matrix4d measured_between = Eigen::Matrix4d::Identity();
};

struct LoopRegistrationQuality {
    bool valid = false;
    bool converged = false;
    double rmse = std::numeric_limits<double>::max();
    double overlap_ratio = 0.0;
};

/// 经过共享 point-to-plane + 几何验证的回环候选。
struct VerifiedLoop {
    LoopCandidate candidate;
    Keyframe target_keyframe;
    LoopRegistrationQuality registration;
    Cloud::Ptr aligned_source{new Cloud};
    Cloud::Ptr target_raw{new Cloud};
    Eigen::Matrix4d measured_between = Eigen::Matrix4d::Identity();
    PoseSqrtInformation sqrt_information;
};

class LoopClosureVerifier {
public:
    using KeyframeLookup = std::function<bool(int, Keyframe*)>;

    /// 构造回环几何和成对一致性验证器。
    explicit LoopClosureVerifier(const LoopClosureConfig& config = LoopClosureConfig());

    /// 更新回环几何门限和成对一致性配置。
    void reconfigure(const LoopClosureConfig& config);
    /// 返回当前生效的算法配置。
    const LoopClosureConfig& config() const { return config_; }

    /// 验证回环配准质量、位移、姿态和描述子航向一致性。
    bool validateMeasurement(const Keyframe& latest, const Keyframe& candidate,
                             const LoopCandidate& loop_candidate,
                             const Eigen::Matrix4d& measured_between,
                             const LoopRegistrationQuality& registration,
                             std::string* reject_reason = nullptr) const;

    /// 历史一致边达到最低要求即通过；没有同代可比较的历史时允许初始回环。
    bool passesAcceptedConsistency(const Keyframe& latest, const Keyframe& candidate,
                                   const Eigen::Matrix4d& measured_between,
                                   const std::vector<AcceptedLoop>& accepted_loops,
                                   const KeyframeLookup& keyframe_lookup) const;

    /// 检查同一源帧的两个候选是否满足成对位姿一致性。
    bool areSameSourceLoopsConsistent(const VerifiedLoop& lhs, const VerifiedLoop& rhs) const;

    /// 在候选一致性图上确定性选择最大一致团。
    std::vector<int> selectConsistentCandidateClique(
        const std::vector<VerifiedLoop>& candidates) const;

private:
    /// 在同 epoch 原始里程计或跨 epoch 优化地图坐标下验证 Z、横滚/俯仰和偏航。
    bool validateWorldGeometry(const Keyframe& latest, const Keyframe& candidate,
                               const LoopCandidate& loop_candidate,
                               const Eigen::Matrix4d& measured_between,
                               std::string* reject_reason) const;

    /// 验证可信描述子偏航与几何推导的源到目标偏航是否一致。
    bool validateDescriptorYaw(const Eigen::Matrix4d& latest_world_pose,
                               const Eigen::Matrix4d& predicted,
                               const LoopCandidate& loop_candidate,
                               std::string* reject_reason) const;

    LoopClosureConfig config_;
};

}  // namespace backend
