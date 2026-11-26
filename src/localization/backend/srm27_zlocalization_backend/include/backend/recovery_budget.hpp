/**
 * @file recovery_budget.hpp
 * @brief 恢复提交的幅度预算：判断一次重定位结果是否偏离最后可信位姿过远。
 *
 * 为什么需要
 * ==========
 * 恢复路径（`applyRecoveryLocked`）直接把匹配结果装成 map->odom，**没有幅度上限**，
 * 这一点与上游逐字相同。上游靠 UWB 把恢复的候选搜索半径压到 2 m
 * （`uwb.recovery_search_radius`），所以恢复结果不可能离位太远；没有 UWB 时恢复退化成
 * 30/35 m 的全局描述子检索，对称走廊下 Scan Context 的 180° 混叠可以凭"残差好看"
 * 通过验收——2026-11-26 实车连续提交了 +45.2 / +11.4 / +30.4 m 的修正，
 * 其中两次还在数秒后被反向弹回。
 *
 * 判据
 * ====
 * 与 `tracking.max_translation_correction` / `max_rotation_correction_deg` 对称，
 * 但**只在已有可信位姿时生效**：冷启动（INITIALIZING）本来就没有"上一时刻"可比，
 * 与上游 UWB/启动提示分支的行为一致，不设限。`0` 表示该轴不检查。
 */
#pragma once

#include "backend/config.hpp"
#include "backend/work.hpp"

namespace backend {

/// 恢复提交幅度是否落在预算内。
///
/// @param delta 本次恢复提交相对当前冻结输出（有可信位姿时即最后可信位姿）的位姿变化。
/// @param config 恢复配置，读取两项上限；`0` 表示关闭对应轴。
/// @param has_trusted_pose 是否已有可信位姿；`false` 时恒为 `true`（冷启动不设限）。
/// @return 平移与旋转都不越限时为 `true`。
inline bool recoveryCorrectionWithinBudget(const PoseCorrectionDelta& delta,
                                           const RecoveryConfig& config,
                                           const bool has_trusted_pose) {
    if (!has_trusted_pose) {
        return true;
    }
    if (config.max_translation_correction > 0.0 &&
        delta.translation > config.max_translation_correction) {
        return false;
    }
    if (config.max_rotation_correction_rad > 0.0 &&
        delta.rotation_rad > config.max_rotation_correction_rad) {
        return false;
    }
    return true;
}

}  // namespace backend
