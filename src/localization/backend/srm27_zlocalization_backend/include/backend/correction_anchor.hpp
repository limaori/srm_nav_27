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
     * @brief 记录一次**已被接受并已提交**的修正，按"安静才跟随"的规则决定是否挪锚点。
     *
     * @param T_map_odom 提交之后的 map→odom。
     * @param applied_rotation_rad 本次提交实际发生的旋转量（不是候选相对锚点的量）。
     * @param applied_translation 本次提交实际发生的平移量。
     */
    void observe(const Eigen::Matrix4d& T_map_odom, const double applied_rotation_rad,
                 const double applied_translation) {
        if (!config_.enabled || !T_map_odom.allFinite())
            return;
        if (!engaged_) {
            // 冷启动/复位后的第一次提交：直接落锚。
            anchor_ = T_map_odom;
            engaged_ = true;
            quiet_frames_ = 0;
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
    }

    /// 无条件把锚点挪到给定 map→odom（人工重定位提交成功后使用）。
    void reseat(const Eigen::Matrix4d& T_map_odom) {
        if (!T_map_odom.allFinite())
            return;
        anchor_ = T_map_odom;
        engaged_ = true;
        quiet_frames_ = 0;
    }

private:
    CorrectionAnchorConfig config_;
    Eigen::Matrix4d anchor_ = Eigen::Matrix4d::Identity();
    bool engaged_ = false;
    int quiet_frames_ = 0;
};

}  // namespace backend
