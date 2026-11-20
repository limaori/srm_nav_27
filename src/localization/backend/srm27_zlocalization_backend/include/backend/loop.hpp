/**
 * @file loop.hpp
 * @brief 构造历史回环几何并并行验证及确定性选择候选。
 */
#pragma once

#include <cstddef>
#include <optional>
#include <vector>

#include "backend/config.hpp"
#include "backend/loop_consistency.hpp"
#include "backend/registration.hpp"
#include "backend/work.hpp"

namespace backend {

/// 单次回环验证周期的锁外结果。
struct LoopVerificationBatch {
    int source_keyframes = 0;
    /// 同一源帧的候选完成一致性筛选后，最多保留一个独立约束。
    std::optional<VerifiedLoop> selected;
};

struct LoopMatcherConfig {
    LoopConfig loop;
    RegistrationConfig registration;
};

/// 纯锁外回环处理组件；不持有图、状态机、ROS 或控制器锁。
class LoopMatcher {
public:
    /// 绑定回环几何、配准及一致性验收参数。
    explicit LoopMatcher(const LoopMatcherConfig& config);

    /// 验证一次候选批次；同一 source 的相关候选最终只返回最佳一条。
    LoopVerificationBatch compute(const LoopWork& work) const;

    /// 从同 epoch 历史前缀选取空间邻近关键帧，并变换到中心关键帧坐标系。
    static Cloud::Ptr buildSpatialSubmapFromSnapshot(const std::vector<Keyframe>& keyframes,
                                                     int center_index, std::size_t max_keyframes,
                                                     double max_radius, int max_inclusive_index);

private:
    /// 在按索引排列的冻结轨迹中查询指定关键帧。
    static bool getSnapshotKeyframe(const std::vector<Keyframe>& keyframes, int index,
                                    Keyframe* out);
    /// 累计历史因果源并按回环分辨率降采样。
    Cloud::Ptr prepareSource(const LoopWork& work, int& source_keyframes) const;
    /// 对单个回环候选执行配准、几何和历史一致性验证。
    std::optional<VerifiedLoop> verifyCandidate(const LoopWork& work,
                                                const LoopCandidate& candidate,
                                                const Cloud::Ptr& source,
                                                int source_keyframes) const;
    /// 对候选一致性团按配准质量、描述子距离和索引确定性择优。
    std::optional<VerifiedLoop> selectBest(std::vector<VerifiedLoop> valid) const;

    LoopMatcherConfig config_;
    LoopClosureVerifier verifier_;
};

}  // namespace backend
