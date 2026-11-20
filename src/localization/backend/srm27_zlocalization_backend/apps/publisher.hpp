/**
 * @file publisher.hpp
 * @brief 拥有 ROS 输出资源、缓存和发布身份边界。
 */
#pragma once
#include <atomic>
#include <exception>
#include <functional>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <memory>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <tf2_ros/transform_broadcaster.hpp>
#include <utility>
#include <visualization_msgs/msg/marker.hpp>

#include "apps/config.hpp"
#include "backend/ports.hpp"

namespace backend {
class Localization;
class KeyframeMap;
}  // namespace backend

namespace backend::ros {
/// 将已提交核心结果发布为 ROS 消息；拥有输出缓存和最终身份复核锁。
class Publisher final : public StateSink, public VisualizationSink {
public:
    using FailureHandler = std::function<void(const char*, std::exception_ptr)>;
    /// 借用节点、配置和地图，并保存节点级失败处理回调。
    Publisher(rclcpp::Node& node, const NodeConfig& parameters, const KeyframeMap& trajectory,
              const KeyframeMap& prior, const std::atomic<bool>& failed, FailureHandler failure);
    /// 装配后绑定只读输出查询；控制器寿命覆盖所有发布调用。
    void bind(const Localization& controller) { controller_ = &controller; }
    /// 根据有效能力一次性创建发布资源。
    void configure();
    /// 查询运行地图发布器是否已创建，供节点决定是否启用地图定时器。
    bool runtimeMapEnabled() const { return static_cast<bool>(runtime_map_pub_); }
    /// 清空旧前端代次的路径缓存及路径、点云发布水位。
    void resetInput();
    /// 复核同步帧投影身份，发布 TF、定位状态及可选世界点云。
    void publishFrame(const FrontendFrame& frame, const LocalizationProjection& projection,
                      const builtin_interfaces::msg::Time& stamp,
                      const builtin_interfaces::msg::Time& cloud_stamp);
    /// 复核高频投影身份，发布传感器里程计、车体位姿及兼容输出。
    void publishPrediction(const nav_msgs::msg::Odometry& odom,
                           const LocalizationProjection& projection);
    /// 显示已接纳 UWB 观测经时空对齐与杆臂修正后的 map 系车体位置。
    void publishUwb(const geometry_msgs::msg::PoseStamped& pose);
    /// 复核状态快照的代次与输出版本后发布定位诊断。
    void publishState(const LocalizationSnapshot& snapshot) override;
    /// 在可选输出的失败边界内发布轨迹、回环和配准调试几何。
    void publishVisualization(const VisualizationSnapshot& visualization) override;
    /// 按显示分辨率发布先验关键帧合成的 map 系点云。
    void publishPriorMap();
    /// 按显示分辨率发布当前运行轨迹合成的 map 系点云。
    void publishRuntimeMap();
    /// 发布地图原点处的三条彩色坐标轴。
    void publishMapAxes();

private:
    /// 可选输出失败只跳过当前输出；资源、估计或未知失败交回节点统一处理。
    template <class Action>
    void publishOptional(const char* name, Action&& action) {
        if (failed_.load())
            return;
        try {
            std::forward<Action>(action)();
        } catch (...) {
            handleOptionalFailure(name, std::current_exception());
        }
    }
    /// 按异常类型记录本次输出失败，或交由节点统一处理。
    void handleOptionalFailure(const char* name, std::exception_ptr failure);
    /// 更新 map 系传感器位姿与协方差，不可信时清零速度并放大不确定度。
    void publishOdometry(const nav_msgs::msg::Odometry& input,
                         const LocalizationProjection& projection);
    /// 发布允许推进的 map → odom TF 及同版本定位状态。
    void publishProjection(const LocalizationProjection& projection,
                           const builtin_interfaces::msg::Time& stamp);
    /// 在最近一次车体位置上方显示状态、原因和累计故障信息。
    void publishStatusMarker(LocalizationStatus status, const std::string& reason,
                             const EstimatorFailureHistory& estimator_failures,
                             const rclcpp::Time& stamp, std::uint64_t epoch);
    /// 将状态及其身份字段编码为诊断文本，并同步更新危险码和状态标记。
    void publishLocalizationState(const LocalizationStatus status, const std::string& reason,
                                  const EstimatorFailureHistory& estimator_failures,
                                  const rclcpp::Time& stamp, const bool tf_published,
                                  const std::uint64_t epoch, const std::uint64_t state_generation,
                                  const std::uint64_t pose_revision);
    /// 将传感器位姿及协方差按车体杆臂传播到车体原点后发布。
    void publishPoseWithCovariance(const LocalizationProjection& projection,
                                   const builtin_interfaces::msg::Time& stamp);
    /// 按观测时间限频、裁剪路径缓存并发布兼容路径快照。
    void publishLegacyPath(const geometry_msgs::msg::PoseStamped& pose,
                           const builtin_interfaces::msg::Time& stamp_message);
    /// 按兼容策略发布车体位姿、杆臂补偿速度、路径及 TF。
    void publishLegacyOutput(const nav_msgs::msg::Odometry& odom,
                             const LocalizationProjection& projection);
    /// 在发布锁外变换点云，发送前再次复核投影身份与限频水位。
    void publishWorldCloud(const FrontendFrame& frame, const LocalizationProjection& projection,
                           const builtin_interfaces::msg::Time& stamp);
    /// 将定位可信性转换为兼容危险码：可信为 0，其余为 9。
    void publishLegacyDanger(const LocalizationStatus status);
    /// 按开关构造地图系轨迹、回环连线及配准源和目标点云。
    void publishVisualizationImpl(const VisualizationSnapshot& visualization);

    /// 借用的节点、配置和地图由装配方持有，其寿命覆盖全部发布调用。
    rclcpp::Node& node_;
    const NodeConfig& parameters_;
    const KeyframeMap& trajectory_;
    const KeyframeMap& prior_map_;
    const std::atomic<bool>& failed_;
    FailureHandler failure_;
    const Localization* controller_ = nullptr;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
        pose_covariance_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr prior_map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr runtime_map_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr odom_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr optimized_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr loop_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr map_axes_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr uwb_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr status_marker_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_source_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr target_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr legacy_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr legacy_twist_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr legacy_path_pub_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr legacy_danger_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr world_cloud_pub_;

    /// 保护路径、发布水位及状态标记的位置缓存。
    std::mutex output_mutex_;
    /// 串行化状态发布与投影最终复核，使消费者看到全序关系：旧投影要么在状态
    /// 转移前发布，要么在转移后被丢弃。
    std::mutex publication_mutex_;
    nav_msgs::msg::Path legacy_path_;
    Eigen::Vector3d last_visualized_body_position_ = Eigen::Vector3d::Zero();
    rclcpp::Time last_legacy_path_stamp_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_world_cloud_stamp_{0, 0, RCL_ROS_TIME};
};
}  // namespace backend::ros
