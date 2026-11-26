// 验证恢复提交的幅度预算（backend/recovery_budget.hpp）。
//
// 背景：恢复路径原先没有幅度上限（与上游逐字相同）。上游靠 UWB 把恢复的候选搜索
// 半径压到 2 m（uwb.recovery_search_radius），本车没有 UWB，恢复退化成 30/35 m 的
// 全局描述子检索；2026-11-26 实车连续提交了 +45.2 / +11.4 / +30.4 m 的修正
// （180° 混叠 + 十余米位移，残差还很好看），其中两次数秒后被反向弹回。
// 这里只测判据本身；提交端接线由 backend_localization_test 覆盖（正常恢复仍能提交）。
#include "backend/recovery_budget.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace backend {
namespace {

constexpr double kDeg = M_PI / 180.0;

/// 与 backend.yaml 默认值一致的配置：2.5 m / 20°。
RecoveryConfig defaultBudget() {
    RecoveryConfig config;
    config.max_translation_correction = 2.5;
    config.max_rotation_correction_rad = 20.0 * kDeg;
    return config;
}

/// 构造一次位姿变化。
PoseCorrectionDelta delta(const double translation_m, const double rotation_deg) {
    PoseCorrectionDelta value;
    value.translation = translation_m;
    value.rotation_rad = rotation_deg * kDeg;
    return value;
}

/// 冷启动没有"上一时刻"可比，任何幅度都必须放行（否则首帧无法建立 map->odom）。
TEST(RecoveryBudget, ColdStartIgnoresBudget) {
    const auto config = defaultBudget();
    EXPECT_TRUE(recoveryCorrectionWithinBudget(delta(1000.0, 179.0), config, false));
    EXPECT_TRUE(recoveryCorrectionWithinBudget(delta(0.0, 0.0), config, false));
}

/// 平移越限即拒绝；边界值（等于上限）仍算通过。
TEST(RecoveryBudget, TranslationOverBudgetRejected) {
    const auto config = defaultBudget();
    EXPECT_TRUE(recoveryCorrectionWithinBudget(delta(2.5, 0.0), config, true));
    EXPECT_FALSE(recoveryCorrectionWithinBudget(delta(2.51, 0.0), config, true));
    EXPECT_FALSE(recoveryCorrectionWithinBudget(delta(45.2, 0.0), config, true));
}

/// 旋转越限即拒绝，即使平移很小（180° 混叠常见的是旋转越限、平移也越限）。
TEST(RecoveryBudget, RotationOverBudgetRejected) {
    const auto config = defaultBudget();
    EXPECT_TRUE(recoveryCorrectionWithinBudget(delta(0.1, 20.0), config, true));
    EXPECT_FALSE(recoveryCorrectionWithinBudget(delta(0.1, 20.1), config, true));
    EXPECT_FALSE(recoveryCorrectionWithinBudget(delta(0.1, 176.6), config, true));
}

/// 平移在预算内但旋转越限，同样拒绝——两轴都要过。
TEST(RecoveryBudget, BothAxesMustPass) {
    const auto config = defaultBudget();
    EXPECT_FALSE(recoveryCorrectionWithinBudget(delta(0.4, 30.0), config, true));
    EXPECT_TRUE(recoveryCorrectionWithinBudget(delta(0.4, 5.0), config, true));
}

/// 0 表示关闭该轴检查（给"确实需要放开"的现场留出口）。
TEST(RecoveryBudget, ZeroDisablesTheAxis) {
    auto config = defaultBudget();
    config.max_translation_correction = 0.0;
    EXPECT_TRUE(recoveryCorrectionWithinBudget(delta(1.0e6, 0.0), config, true));
    EXPECT_FALSE(recoveryCorrectionWithinBudget(delta(1.0e6, 25.0), config, true));

    config = defaultBudget();
    config.max_rotation_correction_rad = 0.0;
    EXPECT_TRUE(recoveryCorrectionWithinBudget(delta(0.1, 179.0), config, true));
    EXPECT_FALSE(recoveryCorrectionWithinBudget(delta(9.0, 179.0), config, true));
}

/// 两轴都置 0 = 完全关闭闸门，与改动前的行为一致。
TEST(RecoveryBudget, ZeroBothAxesDisablesTheGate) {
    auto config = defaultBudget();
    config.max_translation_correction = 0.0;
    config.max_rotation_correction_rad = 0.0;
    EXPECT_TRUE(recoveryCorrectionWithinBudget(delta(45.2, 176.6), config, true));
}

}  // namespace
}  // namespace backend
