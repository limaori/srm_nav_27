/**
 * @file uwb_alignment.hpp
 * @brief 定义 UWB 与里程计坐标对齐的估计结果和求解接口。
 */
#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <vector>

#include "backend/config.hpp"
#include "backend/covariance.hpp"

namespace backend {

/// UWB 与里程计的坐标对齐、协方差及验收结果。
struct UwbAlignmentEstimate {
    /// 是否通过对齐可观性验收。
    bool observable = false;
    /// 连续可观解是否达到稳定次数要求。
    bool stable = false;
    /// 里程计坐标系到地图坐标系的对齐变换。
    Eigen::Matrix4d T_map_odom = Eigen::Matrix4d::Identity();
    /// 对齐参数 [tx, ty, tz, 重力横滚, 重力俯仰, yaw] 的协方差。
    /// use_z=false 时，tz 表示固定高度。
    PoseCovariance6d covariance = PoseCovariance6d::Zero();
    /// 当前保留的位置配对数量。
    int pairs = 0;
    /// 本次对齐的内点数量。
    int inliers = 0;
    /// 连续稳定对齐解的数量。
    int consecutive_stable_solutions = 0;
    /// 里程计系内点的最大水平位置跨度，m。
    double odom_baseline = 0.0;
    /// 地图系内点的最大水平位置跨度，m。
    double map_baseline = 0.0;
    /// 内点位置残差均方根，m。
    double rmse = std::numeric_limits<double>::infinity();
    /// 对齐航向的标准差，rad。
    double yaw_std_rad = std::numeric_limits<double>::infinity();
    /// 最近一次配对样本的时间戳，s。
    double latest_timestamp = -std::numeric_limits<double>::infinity();
    /// 内点对应的原始测量时间戳，s；与输入样本逐值一致。
    std::vector<double> inlier_timestamps;
};

/// 根据 UWB 与里程计位置配对估计平面刚体变换及可选高度偏移。
class UwbAlignmentBootstrapper {
public:
    /// 构造 UWB 平面对齐估计器并保存鲁棒估计配置。
    explicit UwbAlignmentBootstrapper(const UwbAlignmentConfig& config = UwbAlignmentConfig());

    /// 应用对齐配置和可选初值，清空样本及稳定性记录。
    void reset(const UwbAlignmentConfig& config,
               const std::optional<Eigen::Matrix4d>& initial_guess = std::nullopt);
    /// 保存有限的里程计到地图初始变换，供后续共识假设使用。
    void setInitialGuess(const Eigen::Matrix4d& T_map_odom);

    /// 添加已完成时间对齐的天线位置对，返回对齐估计及其可观性、稳定性。
    /// 样本无效、时间未递增或数据不足时返回空值。
    std::optional<UwbAlignmentEstimate> addPair(double timestamp,
                                                const Eigen::Vector3d& position_odom_antenna,
                                                const Eigen::Vector3d& position_map_antenna);

    /// 按给定原始时间戳筛选保留的配对样本。
    void retainPairs(const std::vector<double>& timestamps);

private:
    /// 同一时刻在里程计系和地图系中的天线位置。
    struct Pair {
        /// 原始测量时间戳，s。
        double timestamp = 0.0;
        /// 里程计系天线位置，m。
        Eigen::Vector3d odom = Eigen::Vector3d::Zero();
        /// 地图系天线位置，m。
        Eigen::Vector3d map = Eigen::Vector3d::Zero();
    };

    /// 对齐候选及其内点、残差统计。
    struct Candidate {
        /// 里程计系到地图系的航向旋转，rad。
        double yaw = 0.0;
        /// 水平平移，m。
        Eigen::Vector2d translation = Eigen::Vector2d::Zero();
        /// 高度偏移，m。
        double translation_z = 0.0;
        /// 内点在配对样本中的索引。
        std::vector<std::size_t> inliers;
        /// 内点位置残差均方根，m。
        double rmse = std::numeric_limits<double>::infinity();
    };

    /// 使用指定样本拟合平面刚体对齐及可选高度偏移。
    bool fitIndices(const std::vector<std::size_t>& indices, Candidate* candidate) const;
    /// 按当前对齐假设和残差阈值选择内点。
    void classifyInliers(Candidate* candidate) const;
    /// 构造初值、全量拟合和有界的两点假设集合。
    std::vector<Candidate> buildHypotheses() const;
    /// 按确定性采样规则追加两点几何假设。
    void appendPairHypotheses(std::vector<Candidate>* hypotheses) const;
    /// 选择最大共识集，并在该集合上做有限次精化。
    std::optional<Candidate> selectConsensus(const std::vector<Candidate>& hypotheses) const;
    /// 从共识集填写样本数、基线、误差和时间身份。
    UwbAlignmentEstimate makeEstimateSummary(const Candidate& best) const;
    /// 计算 SE(2)+可选 Z 协方差，并判定本次解是否可观。
    bool populateEstimateCovariance(const Candidate& best, UwbAlignmentEstimate* estimate) const;
    /// 用相邻可观解的变化量更新连续稳定计数。
    void updateEstimateStability(UwbAlignmentEstimate* estimate);
    /// 从样本共识计算对齐及协方差，更新可观性和连续稳定性。
    std::optional<UwbAlignmentEstimate> estimate();
    /// 计算指定内点在里程计系或地图系中的最大水平两点距离，m。
    static double baseline(const std::vector<Pair>& pairs, const std::vector<std::size_t>& indices,
                           bool use_map);

    /// 当前对齐与验收配置。
    UwbAlignmentConfig config_;
    /// 当前保留的位置配对样本。
    std::vector<Pair> pairs_;
    /// 可选的里程计系到地图系初始变换。
    std::optional<Eigen::Matrix4d> initial_guess_;
    /// 上一次通过可观性验收的对齐变换。
    std::optional<Eigen::Matrix4d> previous_observable_estimate_;
    /// 当前连续稳定解的数量。
    int consecutive_stable_solutions_ = 0;
    /// 最近接纳的原始测量时间戳，s。
    double last_timestamp_ = -std::numeric_limits<double>::infinity();
};

}  // namespace backend
