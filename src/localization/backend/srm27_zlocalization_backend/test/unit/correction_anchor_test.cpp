// 验证 map→odom 的累计修正预算（backend/correction_anchor.hpp）。
//
// 背景：换 227to219_1125 之后 map→odom 会**跳**，状态机上对应
// SUSPECT/LOST → "recovered via Scan Context + scan-to-map"。
// 原因是那张图的描述子已不可分（实测任意两处距离≈0.000），检索等于随机挑候选。
//
// 逐次上限（tracking 15°/0.5 s、recovery 20°/次）拦不住"跳一次、再跳一次"：
// 闸门比较的是"本次测量 vs 当前预测"，而预测拿当前 map→odom 算，
// 上一次吃进去的错误变成下一次的基准。这里测的是**累计**那条判据本身。
#include "backend/correction_anchor.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace backend {
namespace {

constexpr double kDeg = M_PI / 180.0;

/// 与实车 profile 一致的配置：8° / 0.8 m，安静阈值 1° / 0.15 m，连续 5 帧。
CorrectionAnchorConfig realProfileConfig() {
    CorrectionAnchorConfig config;
    config.enabled = true;
    config.max_rotation_rad = 8.0 * kDeg;
    config.max_translation = 0.8;
    config.reseat_rotation_rad = 1.0 * kDeg;
    config.reseat_translation = 0.15;
    config.reseat_quiet_frames = 5;
    return config;
}

/// 构造一个"相对 base 转 yaw_deg、平移 (x, y)"的 map→odom。
Eigen::Matrix4d pose(const double base_deg, const double yaw_deg, const double x, const double y) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    const double yaw = (base_deg + yaw_deg) * kDeg;
    T(0, 0) = std::cos(yaw);
    T(0, 1) = -std::sin(yaw);
    T(1, 0) = std::sin(yaw);
    T(1, 1) = std::cos(yaw);
    T(0, 3) = x;
    T(1, 3) = y;
    return T;
}

/// 冷启动（未落锚）不能设限，否则首帧建不起 map->odom。
TEST(CorrectionAnchor, ColdStartAcceptsEverything) {
    CorrectionAnchor anchor;
    anchor.configure(realProfileConfig());
    EXPECT_FALSE(anchor.engaged());
    EXPECT_TRUE(anchor.accepts(pose(0.0, 179.0, 45.0, 0.0)));
}

/// 第一次提交直接落锚，之后的候选相对它判。
TEST(CorrectionAnchor, FirstObserveSeatsAnchor) {
    CorrectionAnchor anchor;
    anchor.configure(realProfileConfig());
    anchor.observe(pose(0.0, 20.0, 1.0, 2.0), 0.0, 0.0);  // 冷启动：无论多大都落锚
    EXPECT_TRUE(anchor.engaged());
    EXPECT_NEAR(anchor.rotationFromAnchor(pose(0.0, 20.0, 1.0, 2.0)), 0.0, 1e-9);
    EXPECT_TRUE(anchor.accepts(pose(0.0, 27.9, 1.0, 2.0)));
    EXPECT_FALSE(anchor.accepts(pose(0.0, 28.1, 1.0, 2.0)));
}

/// 连续同向的大步修正：第一步不追认锚点，累计到上限就拒 —— 这正是"跳"的拦口。
TEST(CorrectionAnchor, ConsecutiveJumpCorrectionsTripTheBudget) {
    CorrectionAnchor anchor;
    anchor.configure(realProfileConfig());
    const Eigen::Matrix4d start = pose(0.0, 0.0, 0.0, 0.0);
    anchor.observe(start, 0.0, 0.0);

    // 第一次跳 5°：在 8° 预算内，提交（但因为是"大步"，锚点不挪）。
    const Eigen::Matrix4d after_jump1 = pose(0.0, 5.0, 0.0, 0.0);
    EXPECT_TRUE(anchor.accepts(after_jump1));
    anchor.observe(after_jump1, 5.0 * kDeg, 0.0);
    EXPECT_NEAR(anchor.rotationFromAnchor(start), 0.0, 1e-9);  // 锚点没被追认

    // 第二次同向 5°：累计 10° > 8°，拒。
    EXPECT_FALSE(anchor.accepts(pose(0.0, 10.0, 0.0, 0.0)));
    // 反向 3°：累计 2°，仍可接受。
    EXPECT_TRUE(anchor.accepts(pose(0.0, 2.0, 0.0, 0.0)));
}

/// 平移轴独立判：旋转很小但累计平移越限也要拒。
TEST(CorrectionAnchor, TranslationBudgetIsIndependent) {
    CorrectionAnchor anchor;
    anchor.configure(realProfileConfig());
    anchor.observe(pose(0.0, 0.0, 0.0, 0.0), 0.0, 0.0);
    EXPECT_TRUE(anchor.accepts(pose(0.0, 0.0, 0.79, 0.0)));
    EXPECT_FALSE(anchor.accepts(pose(0.0, 0.0, 0.81, 0.0)));
    EXPECT_FALSE(anchor.accepts(pose(0.0, 0.0, 0.0, 45.2)));
}

/// 缓慢的真实漂移（每帧都很小）必须一路跟随，不能被累计预算误杀。
TEST(CorrectionAnchor, QuietCorrectionsReseatAndNeverTrip) {
    CorrectionAnchor anchor;
    auto config = realProfileConfig();
    config.reseat_quiet_frames = 5;
    anchor.configure(config);
    anchor.observe(pose(0.0, 0.0, 0.0, 0.0), 0.0, 0.0);

    Eigen::Matrix4d current = pose(0.0, 0.0, 0.0, 0.0);
    double total = 0.0;
    // 每帧进步 0.8°，连续 30 帧（累计 24°，远超 8° 的单次上限口径）。
    for (int i = 0; i < 30; ++i) {
        total += 0.8;
        const Eigen::Matrix4d next = pose(0.0, total, 0.0, 0.0);
        ASSERT_TRUE(anchor.accepts(next)) << "第 " << i << " 帧被误杀";
        anchor.observe(next, 0.8 * kDeg, 0.0);
        current = next;
    }
    EXPECT_TRUE(anchor.accepts(current));
    EXPECT_NEAR(anchor.rotationFromAnchor(current), 0.0, 1e-9);
}

/// 锯齿："跳一次 5° → 干净一帧 → 再跳 5°"不能被追认。
TEST(CorrectionAnchor, SawtoothDoesNotReseat) {
    CorrectionAnchor anchor;
    anchor.configure(realProfileConfig());
    anchor.observe(pose(0.0, 0.0, 0.0, 0.0), 0.0, 0.0);

    const Eigen::Matrix4d jump1 = pose(0.0, 5.0, 0.0, 0.0);
    anchor.observe(jump1, 5.0 * kDeg, 0.0);    // 大步，不追认
    anchor.observe(jump1, 0.05 * kDeg, 0.01);  // 一帧安静（不足 5 帧）
    anchor.observe(jump1, 0.05 * kDeg, 0.01);
    anchor.observe(jump1, 0.05 * kDeg, 0.01);
    anchor.observe(jump1, 0.05 * kDeg, 0.01);
    EXPECT_NEAR(anchor.rotationFromAnchor(pose(0.0, 0.0, 0.0, 0.0)), 0.0, 1e-9);
    // 还差一帧，锚点仍在原处，第二次同向跳直接被拦。
    EXPECT_FALSE(anchor.accepts(pose(0.0, 10.0, 0.0, 0.0)));
}

/// 连续够多帧安静后，锚点才前移 —— 这是"跳完停稳"之后恢复正常的通道。
TEST(CorrectionAnchor, ReseatAfterEnoughQuietFrames) {
    CorrectionAnchor anchor;
    anchor.configure(realProfileConfig());
    const Eigen::Matrix4d start = pose(0.0, 0.0, 0.0, 0.0);
    anchor.observe(start, 0.0, 0.0);

    const Eigen::Matrix4d jumped = pose(0.0, 5.0, 0.0, 0.0);
    anchor.observe(jumped, 5.0 * kDeg, 0.0);  // 大步
    for (int i = 0; i < 5; ++i) {
        anchor.observe(jumped, 0.0, 0.0);     // 连续 5 帧安静
    }
    EXPECT_NEAR(anchor.rotationFromAnchor(jumped), 0.0, 1e-9);
    // 锚点已经跟到 jumped，后续 6° 又回到预算内。
    EXPECT_TRUE(anchor.accepts(pose(0.0, 11.0, 0.0, 0.0)));
}

/// 人工重定位提交成功后强制重落锚：显式声明"我知道车在哪"。
TEST(CorrectionAnchor, ReseatOverridesAnchor) {
    CorrectionAnchor anchor;
    anchor.configure(realProfileConfig());
    anchor.observe(pose(0.0, 0.0, 0.0, 0.0), 0.0, 0.0);
    EXPECT_FALSE(anchor.accepts(pose(0.0, 170.0, 40.0, 0.0)));
    anchor.reseat(pose(0.0, 170.0, 40.0, 0.0));
    EXPECT_NEAR(anchor.rotationFromAnchor(pose(0.0, 170.0, 40.0, 0.0)), 0.0, 1e-9);
}

/// 关闭开关后退化成旧行为（只看 commit 处的单次上限）。
TEST(CorrectionAnchor, DisabledAcceptsEverything) {
    CorrectionAnchor anchor;
    auto config = realProfileConfig();
    config.enabled = false;
    anchor.configure(config);
    anchor.observe(pose(0.0, 0.0, 0.0, 0.0), 0.0, 0.0);
    EXPECT_TRUE(anchor.accepts(pose(0.0, 179.0, 45.0, 0.0)));
}

/// 复位（initialize）之后必须回到未落锚，否则冷启动会被上一局的锚点误杀。
TEST(CorrectionAnchor, ClearReturnsToColdStart) {
    CorrectionAnchor anchor;
    anchor.configure(realProfileConfig());
    anchor.observe(pose(0.0, 0.0, 0.0, 0.0), 0.0, 0.0);
    EXPECT_TRUE(anchor.engaged());
    anchor.clear();
    EXPECT_FALSE(anchor.engaged());
    EXPECT_TRUE(anchor.accepts(pose(0.0, 179.0, 45.0, 0.0)));
}

/// 非有限输入不能进图，也不能静默接受。
TEST(CorrectionAnchor, NonFiniteCandidateRejected) {
    CorrectionAnchor anchor;
    anchor.configure(realProfileConfig());
    anchor.observe(pose(0.0, 0.0, 0.0, 0.0), 0.0, 0.0);
    Eigen::Matrix4d nan = pose(0.0, 0.0, 0.0, 0.0);
    nan(0, 3) = std::nan("");
    EXPECT_FALSE(anchor.accepts(nan));
}

// ---------------------------------------------------------------------------
// 一致证据放行（2026-11-27 19:39 包：绝对预算把恢复路径锁死 70 s）
// ---------------------------------------------------------------------------

/// 真实漂移稳定在 1.2 m：前 4 帧拒（只累计证据），第 5 帧放行。
/// 数字直接取 real_zloc_1127_193946 包里 47.3~50.9 s 那 7 次被拒的观测
/// （delta_t=1.1673~1.2799 m / delta_r=3.4006~3.6974°）。
TEST(CorrectionAnchor, ConsistentDriftReleasesAfterEnoughAgreeingFrames) {
    CorrectionAnchor anchor;
    anchor.configure(realProfileConfig());
    anchor.observe(pose(0.0, 0.0, 0.0, 0.0), 0.0, 0.0);
    const double recorded_t[7] = {1.21864, 1.27991, 1.24547, 1.16730,
                                  1.18117, 1.23473, 1.20933};
    const double recorded_r_deg[7] = {3.44537, 3.69741, 3.57608, 3.42868,
                                      3.40063, 3.51418, 3.41720};
    for (int i = 0; i < 7; ++i) {
        const auto decision = anchor.judge(pose(0.0, recorded_r_deg[i], recorded_t[i], 0.0));
        EXPECT_EQ(decision.pending_frames, i + 1) << "frame " << i;
        if (i < 4) {
            EXPECT_EQ(decision.verdict, AnchorVerdict::Reject) << "frame " << i;
        } else {
            EXPECT_EQ(decision.verdict, AnchorVerdict::Release) << "frame " << i;
            EXPECT_TRUE(decision.within_release_cap);
        }
    }
}

/// 幅度超过放行半径时，一致得再多也不放 —— 传送（+45 m）不会被这条出口捞进来。
TEST(CorrectionAnchor, ReleaseIsCappedByMagnitude) {
    CorrectionAnchor anchor;
    anchor.configure(realProfileConfig());
    anchor.observe(pose(0.0, 0.0, 0.0, 0.0), 0.0, 0.0);
    for (int i = 0; i < 20; ++i) {
        const auto decision = anchor.judge(pose(0.0, 3.5, 3.0, 0.0));  // 3 m > 1.6 m 放行半径
        EXPECT_EQ(decision.verdict, AnchorVerdict::Reject) << "frame " << i;
        EXPECT_FALSE(decision.within_release_cap);
    }
    // 45 m 的对称走廊传送同理。
    EXPECT_EQ(anchor.judge(pose(0.0, 0.0, 45.0, 0.0)).verdict, AnchorVerdict::Reject);
}

/// 逐帧长大的偏差（积分饱和）不算"一致"：每帧长 1.5° > 1° 容差，计数一直重开。
TEST(CorrectionAnchor, GrowingDriftNeverReleases) {
    CorrectionAnchor anchor;
    anchor.configure(realProfileConfig());
    anchor.observe(pose(0.0, 0.0, 0.0, 0.0), 0.0, 0.0);
    int releases = 0;
    // 从 9° 起（已经越过 8° 预算），每帧再长 1.5°：这是 2026-11-26 "每帧 15°、2 Hz" 的缩影。
    for (int step = 1; step <= 20; ++step) {
        const auto decision = anchor.judge(pose(0.0, 8.0 + 1.5 * step, 0.0, 0.0));
        if (decision.verdict == AnchorVerdict::Release)
            ++releases;
        EXPECT_EQ(decision.pending_frames, 1) << "step " << step;
    }
    EXPECT_EQ(releases, 0);
}

/// 预算内的候选会清空待定队列：中间插一帧正常观测，就不能算"连续一致"。
TEST(CorrectionAnchor, AcceptResetsPendingEvidence) {
    CorrectionAnchor anchor;
    anchor.configure(realProfileConfig());
    anchor.observe(pose(0.0, 0.0, 0.0, 0.0), 0.0, 0.0);
    for (int i = 0; i < 4; ++i)
        EXPECT_EQ(anchor.judge(pose(0.0, 0.0, 1.2, 0.0)).verdict, AnchorVerdict::Reject);
    EXPECT_EQ(anchor.judge(pose(0.0, 0.0, 0.2, 0.0)).verdict, AnchorVerdict::Accept);
    const auto decision = anchor.judge(pose(0.0, 0.0, 1.2, 0.0));
    EXPECT_EQ(decision.verdict, AnchorVerdict::Reject);
    EXPECT_EQ(decision.pending_frames, 1);
}

/// 放行后的提交必须重落锚：否则下一帧又被同一道闸拦下（19:39 包就是这么冻结的）。
TEST(CorrectionAnchor, ReleaseCommitReseatsAnchor) {
    CorrectionAnchor anchor;
    anchor.configure(realProfileConfig());
    anchor.observe(pose(0.0, 0.0, 0.0, 0.0), 0.0, 0.0);
    for (int i = 0; i < 4; ++i)
        anchor.judge(pose(0.0, 0.0, 1.2, 0.0));
    ASSERT_EQ(anchor.judge(pose(0.0, 0.0, 1.2, 0.0)).verdict, AnchorVerdict::Release);

    const Eigen::Matrix4d committed = pose(0.0, 0.0, 1.2, 0.0);
    anchor.observe(committed, 1.2, 0.0, /*released=*/true);
    EXPECT_EQ(anchor.releaseCount(), 1);
    EXPECT_NEAR(anchor.translationFromAnchor(committed), 0.0, 1e-9);
    // 放行之后同一位置就是"预算内"，不会再被拒（不会每帧都放行一次）。
    EXPECT_EQ(anchor.judge(committed).verdict, AnchorVerdict::Accept);
    EXPECT_EQ(anchor.judge(pose(0.0, 0.0, 1.4, 0.0)).verdict, AnchorVerdict::Accept);
}

/// 关掉放行开关就退回"只有绝对预算"的旧行为（现场 A/B 用）。
TEST(CorrectionAnchor, DisabledReleaseKeepsHardBudget) {
    CorrectionAnchor anchor;
    auto config = realProfileConfig();
    config.release_enabled = false;
    anchor.configure(config);
    anchor.observe(pose(0.0, 0.0, 0.0, 0.0), 0.0, 0.0);
    for (int i = 0; i < 20; ++i)
        EXPECT_EQ(anchor.judge(pose(0.0, 0.0, 1.2, 0.0)).verdict, AnchorVerdict::Reject);
    EXPECT_EQ(anchor.releaseCount(), 0);
}

/// 冷启动/复位后没有锚点，任何候选都直接接受（不能因为放行机制反而变严）。
TEST(CorrectionAnchor, ReleaseNeverBlocksColdStart) {
    CorrectionAnchor anchor;
    anchor.configure(realProfileConfig());
    EXPECT_EQ(anchor.judge(pose(0.0, 170.0, 40.0, 0.0)).verdict, AnchorVerdict::Accept);
}

}  // namespace
}  // namespace backend
