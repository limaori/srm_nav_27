/**
 * @file node.hpp
 * @brief 适配 ROS 输入、发布边界和后端服务生命周期。
 */
#pragma once

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <exception>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <localization_interfaces/msg/frontend_frame.hpp>
#include <localization_interfaces/msg/frontend_odometry.hpp>
#include <localization_interfaces/msg/frontend_session.hpp>
#include <localization_interfaces/srv/reset_frontend.hpp>
#include <memory>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <new>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <utility>
#include <vector>

#include "apps/config.hpp"
#include "apps/frontend_connection.hpp"
#include "apps/publisher.hpp"
#include "backend/descriptor.hpp"
#include "backend/keyframe_map.hpp"
#include "backend/localization.hpp"

namespace backend::ros {

/// 使用系统稳态时钟驱动调度周期、冷却和超时。
class SystemSteadyClock final : public SteadyClockPort {
public:
    /// 将系统稳态时钟转换为后端调度时间戳。
    Timestamp nowSteadyTimestamp() const override {
        const auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                     std::chrono::steady_clock::now().time_since_epoch())
                                     .count();
        return Timestamp::fromNanoseconds(nanoseconds);
    }
};

class LocalizationNode final : public rclcpp::Node {
public:
    /// 构造后端节点并依次安装运行时、ROS 端口及周期任务。
    explicit LocalizationNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    /// 释放节点持有的模型、订阅及发布资源。
    ~LocalizationNode() override;
    /// 执行器退出后，按关闭保存策略完成运行地图写出。
    void finish();
    /// 返回运行期是否因不可安全恢复的回调异常请求停止。
    bool failed() const noexcept { return failed_.load(); }

private:
    enum class FailurePolicy { Stop, Skip };
    /// 隔离回调异常；估计器故障由核心恢复，可选输出的普通异常只跳过本次发布。
    template <class Action>
    bool invoke(const char* name, Action&& action, FailurePolicy policy = FailurePolicy::Stop) {
        if (failed())
            return false;
        try {
            std::forward<Action>(action)();
            return true;
        } catch (const EstimationFailure& error) {
            reportEstimationFailure(name, error.what());
        } catch (const std::bad_alloc& error) {
            stopAfterFailure(name, error.what());
        } catch (const std::exception& error) {
            if (policy == FailurePolicy::Skip)
                RCLCPP_ERROR(get_logger(), "%s skipped: %s", name, error.what());
            else
                stopAfterFailure(name, error.what());
        } catch (...) {
            stopAfterFailure(name, "unknown exception");
        }
        return false;
    }
    /// 标记失败并停止 ROS 上下文，让执行器正常汇合全部工作线程。
    void stopAfterFailure(const char* name, const char* reason) noexcept;
    /// 报告已隔离的估计故障并发布 LOST 快照，避免再次进入业务操作。
    void reportEstimationFailure(const char* name, const char* reason);
    using SyncPolicy =
        message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry,
                                                        sensor_msgs::msg::PointCloud2>;

    /// 加载并归一化参数，构造描述子、先验地图和运行时控制器。
    void initializeRuntime();
    /// 创建互斥回调组并接入当前前端 epoch 的输入订阅。
    void configureExecution();
    /// 创建前端 reset 客户端和后端控制服务。
    void configureServices();
    /// 创建维护周期和可选的运行时地图发布周期。
    void configureTimers();
    /// 发布静态可视化、初始状态并记录生效能力摘要。
    void completeStartup();

    /// 验收同步前端帧并在最终身份复核后发布位姿与世界点云。
    void frameCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& odom,
                       const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud,
                       std::uint64_t subscription_epoch);
    /// 将高频里程计投影到当前不可变输出视图并复核发布身份。
    void highRateOdomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& odom,
                              std::uint64_t subscription_epoch);
    /// 验收 UWB 消息坐标系及物理解，并按需发布天线杆臂修正后的诊断位置。
    void uwbCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& pose);
    /// 校验前端协方差并把同步里程计和点云转换为传感器局部帧。
    FrontendFrame makeFrontendFrame(const nav_msgs::msg::Odometry& odom,
                                    const sensor_msgs::msg::PointCloud2& cloud,
                                    std::uint64_t subscription_epoch);
    /// 在前端代次边界重建订阅和同步队列，并保持高频输出回调独立。
    void configureInputs();
    /// 创建带代次捕获的 ZLIO 会话、原子观测及高频预测订阅。
    void configureZlioInputs(std::uint64_t subscription_epoch);
    /// 回收超时传输登记并推进身份确认和后端维护周期。
    void maintenanceCallback();
    /// 按配置周期发布当前运行轨迹地图。
    void mapPublishCallback();

    /// 提交前端新代次身份并重建输入队列和输出时间水位。
    void onFrontendResetBoundary();

    /// 加载不可变先验关键帧及描述子，并校验数量和位置索引。
    bool loadPriorMap(const std::string& directory);
    /// 校验运行轨迹保存目的地并调用地图包的统一提交接口。
    bool saveTrajectoryMap(const std::string& directory);

    /// 处理地图保存请求并返回当前能力和提交操作的结果。
    void saveMapService(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    /// 将手动重定位请求交给控制器并报告是否已经登记。
    void relocalizeService(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                           std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    /// 发起协调前端重置事务并报告服务可用性及冷却结果。
    void resetService(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void handleFailure(const char* name, std::exception_ptr error);

    NodeConfig parameters_;

    KeyframeMap trajectory_;
    KeyframeMap prior_map_;
    std::unique_ptr<PlaceDescriptor> descriptor_;
    std::unique_ptr<PlaceDescriptor> trajectory_descriptor_;
    SystemSteadyClock steady_clock_;
    std::atomic<bool> failed_{false};
    FrontendConnection frontend_;
    Publisher publisher_;
    std::unique_ptr<Localization> controller_;
    rclcpp::Subscription<localization_interfaces::msg::FrontendFrame>::SharedPtr zlio_frames_;
    rclcpp::Subscription<localization_interfaces::msg::FrontendOdometry>::SharedPtr
        zlio_predictions_;
    rclcpp::Subscription<localization_interfaces::msg::FrontendSession>::SharedPtr zlio_identity_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    rclcpp::CallbackGroup::SharedPtr high_rate_callback_group_;
    rclcpp::CallbackGroup::SharedPtr maintenance_callback_group_;
    rclcpp::CallbackGroup::SharedPtr map_callback_group_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr high_rate_odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr uwb_sub_;
    rclcpp::TimerBase::SharedPtr maintenance_timer_;
    rclcpp::TimerBase::SharedPtr map_timer_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr relocalize_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
    bool logged_first_frame_ = false;
};

}  // namespace backend::ros
