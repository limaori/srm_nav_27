/**
 * @file ports.hpp
 * @brief 定义时钟、前端重置、状态及可视化的外部端口。
 */
#pragma once

#include <Eigen/Core>
#include <cstdint>
#include <optional>

#include "backend/localization_state.hpp"
#include "backend/time.hpp"
#include "backend/view.hpp"

namespace backend {

/// Reset 包含三个终态和一个非终态 IN_PROGRESS。REJECTED 证明前端未被触碰；
/// INDETERMINATE 表示操作可能已跨越 epoch 边界，必须按成功同样保守处理。
enum class FrontendResetOutcome : std::uint8_t {
    Rejected = 0,
    Completed = 1,
    Indeterminate = 2,
    InProgress = 3,
};

/// 用请求者、事务编号和目标代次标识一次幂等重置。
struct FrontendResetTransaction {
    std::uint64_t requester_instance_id = 0;
    std::uint64_t transaction_id = 0;
    std::uint64_t target_epoch = 0;
};

/// 保存同一前端会话在真实重置时刻的新旧里程计坐标关系。
struct FrontendResetBoundary {
    std::uint64_t source_epoch = 0;
    Timestamp stamp;
    Eigen::Matrix4d old_from_new = Eigen::Matrix4d::Identity();
};

/// 返回重置终态、前端身份及可用的同刻坐标边界。
struct FrontendResetReply {
    FrontendResetTransaction transaction;
    FrontendResetOutcome outcome = FrontendResetOutcome::Indeterminate;
    std::uint64_t frontend_instance_id = 0;
    std::uint64_t reset_generation = 0;
    std::optional<FrontendResetBoundary> boundary = std::nullopt;
};

/// 单调调度时钟；传感器或 ROS 时间不得驱动事务超时。
class SteadyClockPort {
public:
    /// 释放单调时钟端口资源。
    virtual ~SteadyClockPort() = default;
    /// 返回驱动维护调度和事务超时的单调时间。
    virtual Timestamp nowSteadyTimestamp() const = 0;
};

/// 前端重置的服务预检、请求发送与代次边界通知接口。
class FrontendResetPort {
public:
    /// 释放前端重置传输端口资源。
    virtual ~FrontendResetPort() = default;
    /// 在手动 reset 改变定位状态前执行的轻量服务可用性预检。
    virtual bool frontendResetReady() const = 0;
    /// 发送前端 reset 请求；返回服务是否可用并已发出。
    /// 实现必须异步交付响应，且不得在本方法返回前重入 Localization。
    virtual bool sendFrontendReset(const FrontendResetTransaction& transaction) = 0;
    /// 已完成或结果不确定的远端操作构成输入 epoch 边界；适配器接受新 epoch
    /// 样本前必须清空同步输入队列。
    virtual void onFrontendResetBoundary() = 0;
};

/// 定位状态快照的发布接口。
class StateSink {
public:
    /// 释放状态发布端口资源。
    virtual ~StateSink() = default;
    /// 经外部端口发布当前已提交的定位状态快照。
    virtual void publishState(const LocalizationSnapshot& snapshot) = 0;
};

/// 可选轨迹、回环边和配准调试信息发布端口。
class VisualizationSink {
public:
    /// 释放可视化发布端口资源。
    virtual ~VisualizationSink() = default;
    /// 发布已冻结的轨迹、回环边及调试几何快照。
    virtual void publishVisualization(const VisualizationSnapshot& visualization) = 0;
};

}  // namespace backend
