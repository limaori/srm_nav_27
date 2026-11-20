/**
 * @file frame_history.hpp
 * @brief 声明观测历史、时间支撑与因果源累计接口。
 */
#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <utility>
#include <vector>

#include "backend/config.hpp"
#include "backend/types.hpp"

namespace backend {

/// 单调时间轴上的有界 SE(3) 位姿缓冲。
class TimedPoseBuffer {
public:
    using Sample = std::pair<double, Eigen::Matrix4d>;
    using Samples = std::deque<Sample>;

    /// 追加位姿并按 duration_seconds 裁剪旧样本；明显时间回退会先清空历史。
    void append(double timestamp, const Eigen::Matrix4d& pose, double duration_seconds);

    /// 在 tolerance_seconds 支撑范围内返回插值或最近位姿，否则返回空。
    std::optional<Eigen::Matrix4d> poseAt(double timestamp, double tolerance_seconds) const;

    /// 清空全部里程计时间支撑样本。
    void clear() { samples_.clear(); }
    /// 返回当前里程计历史是否为空。
    bool empty() const { return samples_.empty(); }
    /// 返回当前保留的里程计样本数量。
    std::size_t size() const { return samples_.size(); }
    /// 返回按时间排列的只读里程计样本。
    const Samples& samples() const { return samples_; }

private:
    Samples samples_;
};

/// 创建运行时关键帧时冻结的有界、因果点云输入视图。
///
/// FrontendFrame 共享 Cloud::Ptr 所有权，冻结快照不会复制点数据。
struct TrackingSourceSnapshot {
    int runtime_index = -1;
    FrontendFrame reference;
    std::vector<FrontendFrame> causal_history;
};

class FrameHistory {
public:
    /// 绑定帧历史和配准源窗口所需的只读配置。
    FrameHistory(const UwbConfig& timing, const SourceAccumulationConfig& tracking,
                 const SourceAccumulationConfig& recovery)
        : timing_(timing), tracking_(tracking), recovery_(recovery) {}
    /// 校验前端代次、有限值和当前时间轴的单调性。
    bool accepts(const FrontendFrame& frame, std::uint64_t epoch) const;
    /// 保存最新同步帧并按配置裁剪点云历史及里程计时间支撑。
    void append(const FrontendFrame& frame);
    /// 清空同步帧、里程计历史、跟踪源快照和输入时间边界。
    void clear();
    /// 清空观测支撑，并要求后续输入严格晚于失效边界。
    void invalidate();
    /// 以最新同步帧重建恢复历史并作废旧跟踪源。
    void beginRecovery();
    /// 返回是否已有当前代次的同步帧。
    bool hasLatest() const { return latest_frame_.has_value(); }
    /// 返回最新已验收值；调用方须先确认该值存在。
    const FrontendFrame& latest() const { return *latest_frame_; }
    /// 返回用于因果源组装的有界同步帧历史。
    const std::deque<FrontendFrame>& history() const { return frame_history_; }
    /// 返回 UWB 时间对齐使用的只读里程计历史。
    const TimedPoseBuffer& odometry() const { return odom_history_; }
    /// 返回关键帧创建时冻结的因果源快照。
    const std::optional<TrackingSourceSnapshot>& trackingSource() const {
        return tracking_source_snapshot_;
    }
    /// 冻结给定关键帧的参考帧及有界因果源历史。
    void capture(int runtime_index, const FrontendFrame& reference,
                 std::vector<FrontendFrame> causal_history);

private:
    const UwbConfig& timing_;
    const SourceAccumulationConfig& tracking_;
    const SourceAccumulationConfig& recovery_;
    std::optional<FrontendFrame> latest_frame_;
    std::optional<double> restart_after_;
    TimedPoseBuffer odom_history_;
    std::deque<FrontendFrame> frame_history_;
    std::optional<TrackingSourceSnapshot> tracking_source_snapshot_;
};
/// 一次因果源累计产生的点云及其实际观测支撑。
struct SourceAccumulationResult {
    Cloud::Ptr cloud;
    int frame_count = 0;
    double span_seconds = 0.0;
};

/// 从图关键帧历史前缀累计参考传感器坐标系下的点云。
SourceAccumulationResult accumulateSource(const Keyframe& reference,
                                          const std::vector<Keyframe>& history,
                                          const SourceAccumulationConfig& config);
/// 从同步帧历史按时间去重累计点云，并返回实际帧数及时间跨度。
SourceAccumulationResult accumulateSource(const FrontendFrame& reference,
                                          const std::vector<FrontendFrame>& history,
                                          const SourceAccumulationConfig& config);
}  // namespace backend
