/**
 * @file publisher.cpp
 * @brief 管理 ROS 发布资源及完整输出的身份复核边界。
 */
#include "apps/publisher.hpp"

#include "apps/ros.hpp"
#include "backend/covariance.hpp"
#include "backend/localization.hpp"

namespace backend::ros {
/// 绑定发布配置、只读地图及节点的失败边界。
Publisher::Publisher(rclcpp::Node& node, const NodeConfig& parameters,
                     const KeyframeMap& trajectory, const KeyframeMap& prior,
                     const std::atomic<bool>& failed, FailureHandler failure)
    : node_(node),
      parameters_(parameters),
      trajectory_(trajectory),
      prior_map_(prior),
      failed_(failed),
      failure_(std::move(failure)) {
}

/// 保留可选输出的失败分类，资源和估计失败由节点统一处理。
void Publisher::handleOptionalFailure(const char* name, std::exception_ptr failure) {
    try {
        std::rethrow_exception(failure);
    } catch (const EstimationFailure&) {
        failure_(name, failure);
    } catch (const std::bad_alloc&) {
        failure_(name, failure);
    } catch (const std::exception& error) {
        RCLCPP_ERROR(node_.get_logger(), "%s skipped: %s", name, error.what());
    } catch (...) {
        failure_(name, failure);
    }
}

/// 按已启用能力创建状态、位姿、点云及可视化发布器。
void Publisher::configure() {
    if (parameters_.input.frontend == "zlio")
        odometry_pub_ =
            node_.create_publisher<nav_msgs::msg::Odometry>("/localization/odometry", 200);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
    // 状态保留最后一条，便于晚加入的消费者立即获取当前定位可信性。
    state_pub_ = node_.create_publisher<std_msgs::msg::String>("~/localization_state",
                                                               rclcpp::QoS(1).transient_local());
    pose_covariance_pub_ = node_.create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "~/localization_pose", rclcpp::QoS(20).reliable());

    // 地图和调试视图按开关创建，并为晚加入的显示端保留最近一次快照。
    if (parameters_.visualization.publish_prior_map && parameters_.runtime.prior_map_enabled) {
        prior_map_pub_ = node_.create_publisher<sensor_msgs::msg::PointCloud2>(
            "~/prior_map", rclcpp::QoS(1).transient_local());
    }
    if (parameters_.visualization.publish_runtime_map) {
        runtime_map_pub_ = node_.create_publisher<sensor_msgs::msg::PointCloud2>(
            "~/runtime_map", rclcpp::QoS(1).transient_local());
    }
    if (parameters_.visualization.publish_paths) {
        odom_path_pub_ = node_.create_publisher<nav_msgs::msg::Path>(
            "~/odom_path", rclcpp::QoS(1).transient_local());
        optimized_path_pub_ = node_.create_publisher<nav_msgs::msg::Path>(
            "~/optimized_path", rclcpp::QoS(1).transient_local());
        loop_marker_pub_ = node_.create_publisher<visualization_msgs::msg::Marker>(
            "~/loop_edges", rclcpp::QoS(1).transient_local());
    }
    if (parameters_.visualization.publish_map_axes) {
        map_axes_pub_ = node_.create_publisher<visualization_msgs::msg::Marker>(
            "~/map_axes", rclcpp::QoS(1).transient_local());
    }
    if (parameters_.visualization.enabled) {
        uwb_marker_pub_ = node_.create_publisher<visualization_msgs::msg::Marker>(
            "~/uwb_position", rclcpp::QoS(1).transient_local());
        status_marker_pub_ = node_.create_publisher<visualization_msgs::msg::Marker>(
            "~/localization_status", rclcpp::QoS(1).transient_local());
    }
    if (parameters_.visualization.publish_registration) {
        aligned_source_pub_ = node_.create_publisher<sensor_msgs::msg::PointCloud2>(
            "~/debug/aligned_source", rclcpp::QoS(1).transient_local());
        target_pub_ = node_.create_publisher<sensor_msgs::msg::PointCloud2>(
            "~/debug/target", rclcpp::QoS(1).transient_local());
    }

    if (parameters_.legacy.enabled) {
        if (parameters_.legacy.publish_pose) {
            legacy_pose_pub_ = node_.create_publisher<geometry_msgs::msg::PoseStamped>(
                parameters_.legacy.pose_topic, rclcpp::SensorDataQoS());
        }
        if (parameters_.legacy.publish_twist) {
            legacy_twist_pub_ = node_.create_publisher<geometry_msgs::msg::TwistStamped>(
                parameters_.legacy.twist_topic, rclcpp::SensorDataQoS());
        }
        if (parameters_.legacy.publish_path) {
            legacy_path_pub_ = node_.create_publisher<nav_msgs::msg::Path>(
                parameters_.legacy.path_topic, rclcpp::QoS(1));
            legacy_path_.header.frame_id = parameters_.input.map_frame;
        }
        legacy_danger_pub_ = node_.create_publisher<std_msgs::msg::Int16>(
            parameters_.legacy.danger_topic, rclcpp::QoS(2));
    }
    if (parameters_.world_cloud.enabled) {
        world_cloud_pub_ = node_.create_publisher<sensor_msgs::msg::PointCloud2>(
            parameters_.world_cloud.topic, rclcpp::QoS(1).reliable());
    }
}

/// 发布地图系传感器里程计及协方差，不可信时清零速度并提高不确定度。
void Publisher::publishOdometry(const nav_msgs::msg::Odometry& input,
                                const LocalizationProjection& projection) {
    if (!odometry_pub_)
        return;
    // 位姿由后端投影替换；输入时间、传感器子坐标系和可信时的局部速度沿用前端消息。
    auto output = input;
    output.header.frame_id = parameters_.input.map_frame;
    output.pose.pose =
        matrixToPoseStamped(projection.T_map_sensor, output.header.frame_id, output.header.stamp)
            .pose;
    PoseCovariance6d covariance = projection.pose_covariance_valid
                                      ? projection.pose_covariance
                                      : fallbackFrontendPoseCovariance();
    if (!projection.pose_trusted) {
        // 冻结位姿只作保守输出，速度清零同时赋予较大方差，避免被解释为高置信静止。
        applyPoseCovarianceFloor(&covariance, 10.0, M_PI);
        output.twist = geometry_msgs::msg::TwistWithCovariance{};
        for (int i = 0; i < 6; ++i)
            output.twist.covariance[i * 6 + i] = 1e6;
    }
    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 6; ++j)
            output.pose.covariance[6 * i + j] = covariance(i, j);
    odometry_pub_->publish(output);
}

/// 将已接纳的 UWB 位置转换为诊断标记，普通显示异常跳过本次输出。
void Publisher::publishUwb(const geometry_msgs::msg::PoseStamped& pose) {
    publishOptional("UWB visualization", [&] {
        const rclcpp::Time stamp(pose.header.stamp);
        const std::optional<Eigen::Vector3d> position_map = controller_->latestUwbBodyPositionMap();
        if (!uwb_marker_pub_ || !position_map.has_value()) {
            return;
        }
        visualization_msgs::msg::Marker marker;
        marker.header = pose.header;
        marker.header.frame_id = parameters_.input.map_frame;
        // 标记使用时间偏置补偿后的运动时刻，与核心接纳的 UWB 位置保持一致。
        const std::optional<double> motion_timestamp = controller_->latestUwbMotionTimestamp();
        if (motion_timestamp.has_value() && *motion_timestamp > 0.0) {
            marker.header.stamp =
                rclcpp::Time(static_cast<std::int64_t>(std::llround(*motion_timestamp * 1.0e9)),
                             stamp.get_clock_type());
        }
        marker.ns = "uwb_position";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = position_map->x();
        marker.pose.position.y = position_map->y();
        marker.pose.position.z = position_map->z();
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.4;
        marker.scale.y = 0.4;
        marker.scale.z = 0.4;
        marker.color.r = 0.1F;
        marker.color.g = 0.65F;
        marker.color.b = 1.0F;
        marker.color.a = 0.95F;
        uwb_marker_pub_->publish(marker);
    });
}

/// 在发布锁内发送同步投影，再在锁外准备可选世界点云。
void Publisher::publishFrame(const FrontendFrame& frame, const LocalizationProjection& projection,
                             const builtin_interfaces::msg::Time& stamp,
                             const builtin_interfaces::msg::Time& cloud_stamp) {
    {
        std::lock_guard<std::mutex> lock(publication_mutex_);
        if (!controller_->projectionIsCurrent(projection))
            return;
        publishProjection(projection, stamp);
    }
    // 点云转换耗时较长，释放发布锁后准备；publishWorldCloud 在真正发送前再次复核。
    if (controller_->projectionIsCurrent(projection))
        publishOptional("world cloud publication",
                        [&] { publishWorldCloud(frame, projection, cloud_stamp); });
}

/// 在同一发布边界内发送高频位姿、里程计和兼容输出。
void Publisher::publishPrediction(const nav_msgs::msg::Odometry& odom,
                                  const LocalizationProjection& projection) {
    std::lock_guard<std::mutex> lock(publication_mutex_);
    if (!controller_->projectionIsCurrent(projection))
        return;
    if (projection.status != LocalizationStatus::Initializing) {
        publishPoseWithCovariance(projection, odom.header.stamp);
        publishOdometry(odom, projection);
    }
    publishLegacyOutput(odom, projection);
}

/// 在前端代次边界清空依赖旧输入时间轴的输出缓存。
void Publisher::resetInput() {
    std::lock_guard<std::mutex> lock(output_mutex_);
    legacy_path_.poses.clear();
    last_legacy_path_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    last_world_cloud_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
}

/// 将可视化输出隔离在可选发布的失败边界内。
void Publisher::publishVisualization(const VisualizationSnapshot& visualization) {
    publishOptional("trajectory visualization", [&] { publishVisualizationImpl(visualization); });
}
}  // namespace backend::ros
