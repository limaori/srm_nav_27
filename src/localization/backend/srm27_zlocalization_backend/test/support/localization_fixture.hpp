/**
 * @file localization_fixture.hpp
 * @brief 提供定位集成测试的共享场景、可控时钟和记录端口。
 */
#pragma once

#include "backend/localization.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "backend/covariance.hpp"
#include "backend/math.hpp"
#include "backend/scan_context.hpp"
#include "scene.hpp"

namespace backend::testing {
/// 由测试显式推进的稳态时钟。
class ManualClock final : public SteadyClockPort {
public:
    /// 返回当前模拟调度时刻。
    Timestamp nowSteadyTimestamp() const override { return Timestamp::fromSeconds(seconds_); }
    /// 按指定秒数推进调度时钟。
    void advance(double duration = 1) { seconds_ += duration; }

private:
    double seconds_ = 10;
};

/// 可控制服务可用性与发送结果，并记录每次重置请求。
struct ResetEndpoint final : FrontendResetPort {
    bool available = false, accepts = true;
    int boundaries = 0;
    std::vector<FrontendResetTransaction> requests;
    /// 返回本场景配置的服务可用状态。
    bool frontendResetReady() const override { return available; }
    /// 按配置拒绝发送或保存请求，等待测试交付响应。
    bool sendFrontendReset(const FrontendResetTransaction& request) override {
        if (!accepts)
            return false;
        requests.push_back(request);
        return true;
    }
    /// 记录已提交的前端代次边界数量。
    void onFrontendResetBoundary() override { ++boundaries; }
};
/// 保存控制器发布的定位状态快照。
struct StateRecorder final : StateSink {
    std::vector<LocalizationSnapshot> snapshots;
    /// 追加本次状态快照，供场景断言检查。
    void publishState(const LocalizationSnapshot& snapshot) override {
        snapshots.push_back(snapshot);
    }
};
/// 在调试几何发布处注入计算与提交之间的输入事件。
struct VisualizationRecorder final : VisualizationSink {
    std::function<void()> before_commit;
    int debug_frames = 0;
    /// 统计调试帧，并执行一次由测试设置的提交前事件。
    void publishVisualization(const VisualizationSnapshot& snapshot) override {
        if (snapshot.include_debug_clouds) {
            ++debug_frames;
            // 此端口在提交锁外调用，用同线程事件精确构造计算与提交之间的输入变化。
            auto event = std::move(before_commit);
            if (event)
                event();
        }
    }
};

/// 使用真实地图、描述子和图估计器的定位场景夹具。
class LocalizationFlow : public ::testing::Test {
protected:
    Cloud::Ptr world = testing::roomCloud();
    KeyframeMap trajectory, prior;
    ScanContextDescriptor descriptor, prior_descriptor;
    Config config;
    ManualClock clock;
    ResetEndpoint reset;
    StateRecorder states;
    VisualizationRecorder visualization;
    std::unique_ptr<Localization> controller;

    /// 设置可重复的小型场景配置，时钟和前端响应由测试控制。
    void SetUp() override {
        ScanContextDescriptorConfig options;
        options.augment_count = 1;
        options.downsample = .2;
        options.exclude_recent = 2;
        descriptor.configure(options);
        prior_descriptor.configure(options);
        config.loop.enabled = false;
        config.keyframes.min_interval = 0;
        config.keyframes.max_interval = .5;
        config.relocalization.relocalization_parallel_workers = 1;
        config.relocalization.relocalization_yaw_sweep_count = 4;
        config.relocalization.point_to_plane_config.max_iterations = 16;
        config.reset.min_gap_sec = 0;
    }
    /// 按场景选择创建先验地图，然后构造并初始化控制器。
    void start(bool with_prior) {
        config.prior_map_enabled = with_prior;
        if (with_prior) {
            prior.add(testing::sceneFrame(*world, 1, Eigen::Matrix4d::Identity(),
                                          Eigen::Matrix4d::Identity()));
            rebuildMapDescriptors(prior_descriptor, prior.snapshot());
        }
        controller = std::make_unique<Localization>(trajectory, prior, prior_descriptor, descriptor,
                                                    config, clock, reset, states, visualization);
        controller->initialize();
    }
    /// 根据独立真值位姿生成局部点云，并附带指定里程计位姿和代次。
    FrontendFrame frame(double time, const Eigen::Matrix4d& truth, const Eigen::Matrix4d& odom,
                        std::uint64_t epoch = 0) const {
        return testing::sceneFrame(*world, time, truth, odom, epoch);
    }
    /// 向控制器提交同步帧及可选线速度。
    bool ingest(const FrontendFrame& input,
                const Eigen::Vector3d& speed = Eigen::Vector3d::Zero()) {
        return controller->ingestFrame(input, speed, Eigen::Vector3d::Zero());
    }
    /// 推进模拟时间并执行一次维护周期。
    void maintain() {
        clock.advance();
        controller->runMaintenanceCycle();
    }
    /// 查询指定帧在当前已提交输出状态下的全局投影。
    LocalizationProjection output(const FrontendFrame& input) const {
        return controller->projectForOutput(input.odom_pose, input.pose_covariance, true,
                                            input.timestamp, input.epoch);
    }
    /// 将当前请求的终态与可选坐标边界交付控制器。
    void reply(FrontendResetOutcome outcome, std::uint64_t generation = 1,
               std::optional<FrontendResetBoundary> boundary = std::nullopt) {
        ASSERT_FALSE(reset.requests.empty());
        controller->onFrontendResetResponse(
            {reset.requests.back(), outcome, 71, generation, std::move(boundary)});
    }
    /// 以带横向激励的解析 UWB 观测与里程计序列建立可信对齐和运行图。
    void feedUwbTrajectory(const Eigen::Matrix4d& alignment, FrontendFrame& current,
                           std::uint64_t epoch = 0, double begin = 10) {
        for (int index = 0; index < 12; ++index) {
            SCOPED_TRACE(index);
            const auto odom = testing::scenePose(.5 * index, .15 * (index % 3), 0, 0);
            const Eigen::Matrix4d truth = alignment * odom;
            const Eigen::Vector3d antenna =
                truth.block<3, 3>(0, 0) * config.uwb.antenna_position_sensor +
                truth.block<3, 1>(0, 3);
            ASSERT_TRUE(controller->ingestUwbPosition(begin + index, antenna));
            current = frame(begin + index, truth, odom, epoch);
            ASSERT_TRUE(ingest(current));
            maintain();
        }
    }
};

}  // namespace backend::testing
