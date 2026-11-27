/**
 * @file correction_anchor.hpp
 * @brief map→odom 相对"锚点"的累计修正预算。
 *
 * 为什么需要
 * ==========
 * 逐次的幅度上限（`tracking.max_rotation_correction_deg` / `recovery.max_*_correction`）
 * 只约束**这一步**走了多远，约束不了**一共走了多远**：
 *
 * - 闸门比较的是"本次测量 vs 当前预测"，而预测是拿**当前** map→odom 算的
 *   （`observation.cpp`: `predicted_map_pose = poses_.transform() * runtime.odom_pose`），
 *   所以上一次已经吃进去的错误，下一次变成了基准 —— 典型的积分饱和；
 * - `correction_period: 0.5 s` 配 `15°` 的单步上限，等价于允许 **30°/s** 的稳态旋转，
 *   180° 只要 6 秒；恢复路径 `20°` 一次、一秒内可以连提多次，实测 2.8 s 内发生过 3 次。
 *
 * 判据
 * ====
 * 记一个锚点 A（map→odom），每次提交前检查：
 *
 *     |yaw(A⁻¹ · candidate)| <= max_rotation_deg   且   |xy(A⁻¹ · candidate)| <= max_translation
 *
 * 锚点 A **不随错误移动**：只有"安静"的提交（本次实际修正量 <= reseat_*）才把 A 前移到
 * 当前 map→odom。于是：
 *
 * - 正常小幅抖动/缓慢真实漂移：每一步都算安静，A 一路跟随，永远不触发；
 * - 连续同向的错误修正：第一步就把 A 落住，之后累计到 max_* 立刻拒 —— 误差被卡住，
 *   状态机走 Conflict → SUSPECT，宁可冻结也不"慢慢转过去"。
 *
 * 冷启动（没有可信位姿、`initialize()` 之后）不设限：那时还没有"上一时刻"可比，
 * 首次提交直接落锚。人工重定位（`/backend_node/relocalize`）提交成功后强制重落锚：
 * 那是"我知道车在哪"的显式声明。
 *
 * 为什么还要"一致证据放行"（2026-11-27 19:39 包）
 * ================================================
 * 只有上面那条判据时，一旦**真实漂移**超过 max_*，系统就再也出不来：锚点不随错误移动
 * （这是它存在的理由），而重落锚要求连续 5 帧"安静"，可偏差稳定在 1.2 m 时永远没有安静帧。
 * 实测 `real_zloc_1127_193946`：47.3 s 起连续 7 次先验观测都报
 * `delta_t=1.17~1.28 m / delta_r=3.4~3.7°`（彼此只差 5 cm / 0.3°），全部被锚点拒；
 * 7 连冲突 → SUSPECT → LOST 之后**跟踪和恢复两条路走的是同一道闸**，70 s 内一次都提交
 * 不进去，位姿纯里程计递推，包尾点云相对先验图歪到 1.8 m / ≥9.5°。
 *
 * 所以补一条**有界**的出口：越界候选如果连续 `release_frames` 帧彼此一致
 * （平移差 ≤ release_agree_translation、旋转差 ≤ release_agree_rotation_rad），
 * 且相对锚点不超过 `release_max_*`，判 `Release` —— 放行一次，提交后必须重落锚。
 *
 * 为什么这样不会把 2026-11-26 那种"传送"放进来：
 * - `release_max_translation / release_max_rotation` 默认 1.6 m / 10°，**低于**恢复路径的
 *   单次闸（2.5 m / 20°）和跟踪路径的单次闸（1.5 m / 15°）；当时那几次传送是
 *   +45.2 / +11.4 / +30.4 m，连单次闸都过不去，更进不到放行半径里；
 * - 一致性与"锚点不追认"是并联的：随机错解不会连续 5 帧落在同一个 12 cm 的盒子里。
 * 代价：真遇到"同构走廊稳定错解"时会接受一次 ≤1.6 m 的错误修正；相比"永久冻结、
 * 纯递推漂到几米"，这是可恢复的坏结果（下一次观测会把残差拉回来）。
 */
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>

#include "backend/math.hpp"

namespace backend {

/// 累计修正预算的阈值；旋转类字段单位为弧度，长度类字段单位为米。
struct CorrectionAnchorConfig {
    /// 是否启用累计预算；false 时退化为"只看单次上限"的旧行为。
    bool enabled = true;
    /// 相对锚点允许的累计旋转，rad；0 表示不检查该轴。
    double max_rotation_rad = 10.0 * M_PI / 180.0;
    /// 相对锚点允许的累计平移，m；0 表示不检查该轴。
    double max_translation = 1.0;
    /// 单次实际修正量不超过它才算"安静"，此时锚点前移到当前 map→odom，rad。
    double reseat_rotation_rad = 1.0 * M_PI / 180.0;
    /// 单次实际平移修正不超过它才算"安静"，m。
    double reseat_translation = 0.15;
    /// 连续多少帧"安静"才允许锚点前移；1 表示每帧都跟。
    /// 取大于 1 是为了掐掉"跳一次 → 安静一帧 → 再跳一次"的锯齿：同一方向的连续错解
    /// 之间插不进足够的安静帧，锚点就不会被追认。
    int reseat_quiet_frames = 5;

    /// 一致证据放行：越界候选连续多帧彼此一致时，允许"按一次提交"（提交后重落锚）。
    /// false 时退化为只有绝对预算的旧行为（会死锁，见文件头）。
    bool release_enabled = true;
    /// 连续多少帧彼此一致的越界候选才放行；恢复路径约 1 Hz、跟踪路径 2 Hz。
    int release_frames = 5;
    /// 相邻两次越界候选之间允许的平移差，m；超过就重新计数。
    double release_agree_translation = 0.12;
    /// 相邻两次越界候选之间允许的旋转差，rad；超过就重新计数。
    double release_agree_rotation_rad = 1.0 * M_PI / 180.0;
    /// 放行时相对锚点允许的最大平移，m；必须 <= 单次闸，否则等于绕过单次幅度检查。
    double release_max_translation = 1.6;
    /// 放行时相对锚点允许的最大旋转，rad；同上。
    double release_max_rotation_rad = 10.0 * M_PI / 180.0;
};

/// 候选 map→odom 相对锚点的裁决结果。
enum class AnchorVerdict {
    /// 仍在累计预算内，正常提交。
    Accept,
    /// 越界，但连续多帧一致且幅度有界：放行一次，提交端**必须**随后重落锚。
    Release,
    /// 越界且证据不足：按冲突处理（跟踪路径计入 conflict，恢复路径计入一次失败）。
    Reject,
};

/// 一次裁决的完整证据，供提交端打日志/决定是否重落锚。
struct AnchorDecision {
    AnchorVerdict verdict = AnchorVerdict::Accept;
    /// 候选相对锚点的旋转偏差，rad。
    double rotation_rad = 0.0;
    /// 候选相对锚点的平移偏差，m。
    double translation = 0.0;
    /// 连续一致的越界候选帧数（含本次）。
    int pending_frames = 0;
    /// 本次是否落在放行半径内（越界但超半径时说明"一致得再多也不放"）。
    bool within_release_cap = false;
};

/**
 * @brief 记住一个不随错误移动的 map→odom 锚点，并裁决候选是否仍在累计预算内。
 *
 * 纯逻辑、无 ROS/PCL 依赖，提交端（跟踪与恢复）共用同一份实例，
 * 由 `PoseOutput` 持有以保证与 TF 输出同生命周期。
 */
class CorrectionAnchor {
public:
    /// 替换阈值配置；不改变已落定的锚点。
    void configure(const CorrectionAnchorConfig& config) { config_ = config; }

    /// 返回当前阈值配置。
    const CorrectionAnchorConfig& config() const { return config_; }

    /// 清除锚点，回到"未落锚"（冷启动）状态。
    void clear() {
        anchor_.setIdentity();
        engaged_ = false;
        quiet_frames_ = 0;
        clearPending();
    }

    /// 是否已经落锚。
    bool engaged() const { return engaged_; }

    /// 返回当前锚点（未落锚时为上一次/reset 后的单位阵，调用方应先查 engaged()）。
    const Eigen::Matrix4d& anchor() const { return anchor_; }

    /// 候选 map→odom 相对锚点的旋转偏差，rad；未落锚或矩阵非法时返回 0。
    double rotationFromAnchor(const Eigen::Matrix4d& T_map_odom) const {
        if (!engaged_ || !T_map_odom.allFinite())
            return 0.0;
        const Eigen::Matrix4d delta = anchor_.inverse() * T_map_odom;
        return delta.allFinite() ? rotationAngleFromMatrix(delta) : 0.0;
    }

    /// 候选 map→odom 相对锚点的平移偏差，m；未落锚或矩阵非法时返回 0。
    double translationFromAnchor(const Eigen::Matrix4d& T_map_odom) const {
        if (!engaged_ || !T_map_odom.allFinite())
            return 0.0;
        const Eigen::Matrix4d delta = anchor_.inverse() * T_map_odom;
        return delta.allFinite() ? delta.block<3, 1>(0, 3).norm() : 0.0;
    }

    /**
     * @brief 判据：候选 map→odom 是否仍在"相对锚点"的累计预算内。
     *
     * 未启用、未落锚（冷启动）或矩阵非法时行为：
     * - 未启用 / 未落锚 → 恒为 true（不设限）；
     * - 矩阵非法 → false（宁可不提交）。
     */
    bool accepts(const Eigen::Matrix4d& T_map_odom) const {
        if (!config_.enabled || !engaged_)
            return true;
        if (!T_map_odom.allFinite())
            return false;
        const Eigen::Matrix4d delta = anchor_.inverse() * T_map_odom;
        if (!delta.allFinite())
            return false;
        if (config_.max_rotation_rad > 0.0 &&
            rotationAngleFromMatrix(delta) > config_.max_rotation_rad) {
            return false;
        }
        if (config_.max_translation > 0.0 &&
            delta.block<3, 1>(0, 3).norm() > config_.max_translation) {
            return false;
        }
        return true;
    }

    /**
     * @brief 裁决候选：预算内 → Accept；越界但连续一致且有界 → Release；否则 Reject。
     *
     * 与 `accepts()` 的区别只有一点：**越界候选会被记进"待定队列"**，连续
     * `release_frames` 帧彼此一致时给出 `Release`。提交端拿到 `Release` 后照常提交，
     * 并在 `observe()` 里把 `released` 置真（锚点直接跟过去）；否则下一次观测又会被
     * 同一道闸拦下 —— 这正是 2026-11-27 19:39 包冻结 70 s 的机制。
     *
     * 计数状态是 `mutable` 的"观察记录"，不改变锚点本身，所以本函数声明为 const：
     * 跟踪路径的 `collectGlobalInnovationsLocked()` 是 const 方法。
     */
    AnchorDecision judge(const Eigen::Matrix4d& T_map_odom) const {
        AnchorDecision decision;
        decision.rotation_rad = rotationFromAnchor(T_map_odom);
        decision.translation = translationFromAnchor(T_map_odom);
        decision.pending_frames = pending_frames_;
        if (!config_.enabled || !engaged_) {
            decision.verdict = AnchorVerdict::Accept;
            return decision;
        }
        if (!T_map_odom.allFinite()) {
            decision.verdict = AnchorVerdict::Reject;
            return decision;
        }
        const bool rotation_ok = config_.max_rotation_rad <= 0.0 ||
                                 decision.rotation_rad <= config_.max_rotation_rad;
        const bool translation_ok =
            config_.max_translation <= 0.0 || decision.translation <= config_.max_translation;
        if (rotation_ok && translation_ok) {
            decision.verdict = AnchorVerdict::Accept;
            clearPending();
            return decision;
        }
        if (!config_.release_enabled) {
            decision.verdict = AnchorVerdict::Reject;
            clearPending();
            return decision;
        }
        // 越界：和上一次越界候选比"是否同一个偏差"。一致就累加，不一致就重新开一队。
        if (pending_engaged_ && agree(pending_, T_map_odom)) {
            ++pending_frames_;
        } else {
            pending_ = T_map_odom;
            pending_engaged_ = true;
            pending_frames_ = 1;
        }
        decision.pending_frames = pending_frames_;
        const bool cap_rotation_ok = config_.release_max_rotation_rad <= 0.0 ||
                                     decision.rotation_rad <= config_.release_max_rotation_rad;
        const bool cap_translation_ok = config_.release_max_translation <= 0.0 ||
                                        decision.translation <= config_.release_max_translation;
        decision.within_release_cap = cap_rotation_ok && cap_translation_ok;
        decision.verdict = (pending_frames_ >= std::max(1, config_.release_frames) &&
                            decision.within_release_cap)
                               ? AnchorVerdict::Release
                               : AnchorVerdict::Reject;
        return decision;
    }

    /// 待定队列里连续一致的越界候选帧数（诊断用）。
    int pendingFrames() const { return pending_frames_; }

    /// 放行累计次数（诊断用：现场能看到"这套配置到底放行过几次"）。
    int releaseCount() const { return release_count_; }

    /**
     * @brief 记录一次**已被接受并已提交**的修正，按"安静才跟随"的规则决定是否挪锚点。
     *
     * @param T_map_odom 提交之后的 map→odom。
     * @param applied_rotation_rad 本次提交实际发生的旋转量（不是候选相对锚点的量）。
     * @param applied_translation 本次提交实际发生的平移量。
     * @param released 本次提交是 `judge()` 给出 `Release` 换来的：锚点直接跟过去
     *        （否则同一道闸会把后续观测继续拦下）。
     */
    void observe(const Eigen::Matrix4d& T_map_odom, const double applied_rotation_rad,
                 const double applied_translation, const bool released = false) {
        if (!config_.enabled || !T_map_odom.allFinite())
            return;
        clearPending();
        if (!engaged_) {
            // 冷启动/复位后的第一次提交：直接落锚。
            anchor_ = T_map_odom;
            engaged_ = true;
            quiet_frames_ = 0;
            return;
        }
        if (released) {
            // 一致证据放行的提交：锚点必须跟过去，否则下一次观测又被同一道闸拦下。
            anchor_ = T_map_odom;
            quiet_frames_ = 0;
            ++release_count_;
            return;
        }
        const bool quiet = applied_rotation_rad <= config_.reseat_rotation_rad &&
                           applied_translation <= config_.reseat_translation;
        if (!quiet) {
            // 大步修正不追认：锚点留在原地，后续同向修正会撞上累计上限。
            quiet_frames_ = 0;
            return;
        }
        if (++quiet_frames_ >= std::max(1, config_.reseat_quiet_frames)) {
            anchor_ = T_map_odom;
            quiet_frames_ = 0;
        }
    }

    /**
     * @brief 按里程计代次切换的坐标关系整体换算锚点（`map→odom_new = map→odom_old · old_from_new`）。
     *
     * 换代（前端重置）只是换了个 odom 原点，物理位姿没变，所以锚点必须跟着换算；
     * 不换算的话切换后第一个候选就会被判"相对锚点偏了几十度"而全部误杀。
     */
    void reframe(const Eigen::Matrix4d& old_from_new) {
        if (!engaged_ || !old_from_new.allFinite())
            return;
        const Eigen::Matrix4d reframed = anchor_ * old_from_new;
        if (reframed.allFinite())
            anchor_ = reframed;
        quiet_frames_ = 0;
        clearPending();
    }

    /// 无条件把锚点挪到给定 map→odom（人工重定位提交成功后使用）。
    void reseat(const Eigen::Matrix4d& T_map_odom) {
        if (!T_map_odom.allFinite())
            return;
        anchor_ = T_map_odom;
        engaged_ = true;
        quiet_frames_ = 0;
        clearPending();
    }

private:
    /// 两个越界候选是否"说的是同一个偏差"（用于一致证据计数）。
    bool agree(const Eigen::Matrix4d& reference, const Eigen::Matrix4d& candidate) const {
        const Eigen::Matrix4d delta = reference.inverse() * candidate;
        if (!delta.allFinite())
            return false;
        if (rotationAngleFromMatrix(delta) > config_.release_agree_rotation_rad)
            return false;
        return delta.block<3, 1>(0, 3).norm() <= config_.release_agree_translation;
    }

    /// 清空"待放行"队列（任何一次落锚/换代/复位都会让旧证据失效）。
    void clearPending() const {
        pending_ = Eigen::Matrix4d::Identity();
        pending_engaged_ = false;
        pending_frames_ = 0;
    }

    CorrectionAnchorConfig config_;
    Eigen::Matrix4d anchor_ = Eigen::Matrix4d::Identity();
    bool engaged_ = false;
    int quiet_frames_ = 0;
    // 一致证据放行的观察记录：`judge()` 是 const（跟踪路径在 const 方法里调用），
    // 这几项只记录"最近几帧说了什么"，不影响锚点，所以用 mutable。
    mutable Eigen::Matrix4d pending_ = Eigen::Matrix4d::Identity();
    mutable bool pending_engaged_ = false;
    mutable int pending_frames_ = 0;
    int release_count_ = 0;
};

}  // namespace backend
