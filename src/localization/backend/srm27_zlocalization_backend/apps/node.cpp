/**
 * @file node.cpp
 * @brief 接入观测、执行服务和维护任务并启动 ROS 节点。
 */

#include "apps/node.hpp"

#include <pcl/common/io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <exception>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <utility>

#include "apps/ros.hpp"
#include "backend/covariance.hpp"
#include "backend/geometry.hpp"
#include "backend/logger.hpp"

namespace backend::ros {
/// 报告已隔离的估计故障并发布 LOST 快照，避免再次进入业务操作。
void LocalizationNode::reportEstimationFailure(const char* name, const char* reason) {
    RCLCPP_ERROR(get_logger(), "%s: %s", name, reason);
    try {
        publisher_.publishState(controller_->snapshot());
    } catch (const std::bad_alloc& error) {
        stopAfterFailure("estimator failure state", error.what());
    } catch (const std::exception& error) {
        RCLCPP_ERROR(get_logger(), "estimator failure state publication skipped: %s", error.what());
    } catch (...) {
        stopAfterFailure("estimator failure state", "unknown exception");
    }
}

/// 标记失败并停止 ROS 上下文，让执行器正常汇合全部工作线程。
void LocalizationNode::stopAfterFailure(const char* name, const char* reason) noexcept {
    if (failed_.exchange(true))
        return;
    try {
        RCLCPP_FATAL(get_logger(), "%s failed; stopping backend: %s", name, reason);
        get_node_base_interface()->get_context()->shutdown("backend callback failure");
    } catch (...) {
        // 无法完成受控停机时终止进程，禁止继续发布未知状态。
        std::terminate();
    }
}

/// 校验前端协方差并把同步里程计和点云转换为传感器局部帧。
FrontendFrame LocalizationNode::makeFrontendFrame(
    const nav_msgs::msg::Odometry& odom, const sensor_msgs::msg::PointCloud2& cloud_message,
    const std::uint64_t subscription_epoch) {
    FrontendFrame frame;
    frame.epoch = subscription_epoch;
    frame.timestamp = rclcpp::Time(odom.header.stamp).seconds();
    frame.odom_pose = odomToMatrix(odom);
    const PoseCovariance6d reported_covariance = poseCovarianceFromRos(odom.pose.covariance);
    std::string covariance_error;
    frame.pose_covariance_valid = validateAndRegularizePoseCovariance(
        reported_covariance, &frame.pose_covariance, &covariance_error);
    if (!frame.pose_covariance_valid) {
        frame.pose_covariance.setZero();
        const bool missing_covariance =
            reported_covariance.allFinite() && reported_covariance.cwiseAbs().maxCoeff() <= 1.0e-15;
        if (!missing_covariance) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "front-end pose covariance rejected (%s); using configured "
                                 "conservative graph fallback",
                                 covariance_error.c_str());
        }
    }

    Cloud cloud;
    const bool has_intensity =
        std::any_of(cloud_message.fields.begin(), cloud_message.fields.end(),
                    [](const auto& field) { return field.name == "intensity"; });
    if (has_intensity) {
        pcl::fromROSMsg(cloud_message, cloud);
    } else {
        pcl::PCLPointCloud2 serialized;
        pcl_conversions::toPCL(cloud_message, serialized);
        pcl::MsgFieldMap fields;
        pcl::createMapping<pcl::PointXYZ>(serialized.fields, fields);
        pcl::fromPCLPointCloud2(serialized, cloud, fields);  // XYZ 同偏移，intensity 保持默认零。
    }
    frame.cloud_local = parameters_.input.cloud_in_local_frame
                            ? Cloud::Ptr(new Cloud(std::move(cloud)))
                            : transformCloud(cloud, frame.odom_pose.inverse());
    return frame;
}

/// 在前端代次边界重建订阅和同步队列，并保持高频输出回调独立。
void LocalizationNode::configureInputs() {
    // 重建订阅是前端 epoch 间的队列清空边界；高频输出使用独立互斥回调组，
    // 高频投影读取已提交输出视图，不等待图更新锁。
    sync_.reset();
    odom_sub_.reset();
    cloud_sub_.reset();
    high_rate_odom_sub_.reset();
    zlio_frames_.reset();
    zlio_predictions_.reset();
    zlio_identity_.reset();
    uwb_sub_.reset();

    if (parameters_.input.frontend == "zlio") {
        configureZlioInputs(frontend_.epoch());
    } else {
        auto sensor_qos = rclcpp::SensorDataQoS();
        sensor_qos.keep_last(static_cast<std::size_t>(parameters_.input.subscription_queue));
        SyncPolicy sync_policy(parameters_.input.synchronizer_queue);
        sync_policy.setMaxIntervalDuration(
            rclcpp::Duration::from_seconds(parameters_.input.synchronizer_max_interval));
        sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
            static_cast<const SyncPolicy&>(sync_policy));
        const std::uint64_t subscription_epoch = frontend_.epoch();
        sync_->registerCallback(std::bind(&LocalizationNode::frameCallback, this,
                                          std::placeholders::_1, std::placeholders::_2,
                                          subscription_epoch));
        // 在真实订阅入口保护同步器内部的时间处理及队列操作。
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            parameters_.input.sync_odom_topic, sensor_qos,
            [this, subscription_epoch](nav_msgs::msg::Odometry::ConstSharedPtr message) {
                invoke("odometry synchronization", [&] {
                    if (!message || !validStamp(message->header.stamp) ||
                        subscription_epoch != frontend_.epoch())
                        return;
                    sync_->add<0>(message);
                });
            });
        cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            parameters_.input.cloud_topic, sensor_qos,
            [this, subscription_epoch](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
                invoke("cloud synchronization", [&] {
                    if (!message || !validStamp(message->header.stamp) ||
                        subscription_epoch != frontend_.epoch())
                        return;
                    sync_->add<1>(message);
                });
            });

        rclcpp::SubscriptionOptions high_rate_options;
        high_rate_options.callback_group = high_rate_callback_group_;
        high_rate_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            parameters_.input.odom_topic, rclcpp::SensorDataQoS(),
            [this, subscription_epoch](const nav_msgs::msg::Odometry::ConstSharedPtr& odom) {
                highRateOdomCallback(odom, subscription_epoch);
            },
            high_rate_options);
    }
    if (parameters_.runtime.uwb.enabled) {
        uwb_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            parameters_.input.uwb_topic, rclcpp::QoS(10),
            std::bind(&LocalizationNode::uwbCallback, this, std::placeholders::_1));
    }
}

/// 验收同步前端帧并在最终身份复核后发布位姿与世界点云。
void LocalizationNode::frameCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& odom,
                                     const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud,
                                     const std::uint64_t subscription_epoch) {
    invoke("synchronized frame", [&] {
        if (!odom || !cloud || !controller_ || subscription_epoch != frontend_.epoch() ||
            !hasUsableOdometryPose(*odom)) {
            return;
        }
        const FrontendFrame frame = makeFrontendFrame(*odom, *cloud, subscription_epoch);
        if (!logged_first_frame_) {
            logged_first_frame_ = true;
            RCLCPP_INFO_STREAM(
                get_logger(),
                "first synchronized front-end frame: stamp="
                    << frame.timestamp
                    << ", points=" << (frame.cloud_local ? frame.cloud_local->size() : 0U)
                    << ", covariance=" << (frame.pose_covariance_valid ? "valid" : "fallback"));
        }

        Eigen::Vector3d linear = vectorFromRos(odom->twist.twist.linear);
        Eigen::Vector3d angular = vectorFromRos(odom->twist.twist.angular);
        if (!linear.allFinite()) {
            linear.setZero();
        }
        if (!angular.allFinite()) {
            angular.setZero();
        }
        if (!controller_->ingestFrame(frame, linear, angular)) {
            return;
        }
        const LocalizationProjection projection =
            controller_->project(frame, toTimestamp(rclcpp::Time(odom->header.stamp)));
        publisher_.publishFrame(frame, projection, odom->header.stamp, cloud->header.stamp);
    });
}

/// 将高频里程计投影到当前不可变输出视图并复核发布身份。
void LocalizationNode::highRateOdomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& odom,
                                            const std::uint64_t subscription_epoch) {
    invoke("odometry output", [&] {
        if (!odom || !controller_ || !validStamp(odom->header.stamp) ||
            subscription_epoch != frontend_.epoch() || !hasUsableOdometryPose(*odom)) {
            return;
        }
        const PoseCovariance6d reported_covariance = poseCovarianceFromRos(odom->pose.covariance);
        PoseCovariance6d covariance = PoseCovariance6d::Zero();
        std::string covariance_error;
        const bool covariance_valid = validateAndRegularizePoseCovariance(
            reported_covariance, &covariance, &covariance_error);
        if (!covariance_valid) {
            const bool missing_covariance = reported_covariance.allFinite() &&
                                            reported_covariance.cwiseAbs().maxCoeff() <= 1.0e-15;
            if (!missing_covariance) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                     "high-rate front-end pose covariance rejected (%s); using "
                                     "the latest conservative graph covariance",
                                     covariance_error.c_str());
            }
        }
        const LocalizationProjection projection = controller_->projectForOutput(
            odomToMatrix(*odom), covariance, covariance_valid,
            rclcpp::Time(odom->header.stamp).seconds(), subscription_epoch);
        publisher_.publishPrediction(*odom, projection);
    });
}

/// 验收 UWB 消息坐标系及物理解，并按需发布天线杆臂修正后的诊断位置。
void LocalizationNode::uwbCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& pose) {
    invoke("UWB input", [&] {
        if (!pose || !controller_ || !validStamp(pose->header.stamp)) {
            return;
        }
        if (pose->header.frame_id != parameters_.input.uwb_frame) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "ignoring UWB observation in frame '%s'; expected '%s'",
                                 pose->header.frame_id.c_str(),
                                 parameters_.input.uwb_frame.c_str());
            return;
        }
        const rclcpp::Time stamp(pose->header.stamp);
        if (stamp.nanoseconds() == 0) {
            return;
        }
        Eigen::Vector3d position(pose->pose.position.x, pose->pose.position.y,
                                 pose->pose.position.z);
        const bool position_finite =
            parameters_.runtime.uwb.use_z ? position.allFinite() : position.head<2>().allFinite();
        if (!position_finite) {
            return;
        }
        if (!parameters_.runtime.uwb.use_z) {
            position.z() = 0.0;
        }
        if (!controller_->ingestUwbPosition(stamp.seconds(), position)) {
            return;
        }
        publisher_.publishUwb(*pose);
    });
}

/// 回收超时传输登记并推进身份确认和后端维护周期。
void LocalizationNode::maintenanceCallback() {
    invoke("maintenance", [&] {
        frontend_.maintain(parameters_.runtime.reset.response_timeout_sec);
        if (controller_)
            controller_->runMaintenanceCycle();
    });
}

/// 按配置周期发布当前运行轨迹地图。
void LocalizationNode::mapPublishCallback() {
    invoke(
        "runtime map publication", [&] { publisher_.publishRuntimeMap(); }, FailurePolicy::Skip);
}

/// 处理地图保存请求并返回当前能力和提交操作的结果。
void LocalizationNode::saveMapService(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    if (!invoke(
            "map save",
            [&] {
                if (parameters_.runtime.prior_map_enabled) {
                    response->success = false;
                    response->message =
                        "immutable-prior localization does not accumulate a runtime map";
                    return;
                }
                if (parameters_.maps.save_directory.empty()) {
                    response->success = false;
                    response->message = "trajectory.save_directory is empty";
                    return;
                }
                const bool success = saveTrajectoryMap(parameters_.maps.save_directory);
                response->success = success;
                response->message = success
                                        ? "saved runtime map to " + parameters_.maps.save_directory
                                        : "runtime map save failed";
            },
            FailurePolicy::Skip)) {
        response->success = false;
        response->message.clear();
    }
}

/// 将手动重定位请求交给控制器并报告是否已经登记。
void LocalizationNode::relocalizeService(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    if (!invoke("relocalization request", [&] {
            const bool accepted = controller_ && controller_->requestManualRelocalization();
            response->success = accepted;
            response->message = accepted ? "relocalization scheduled for the next new frame"
                                         : "manual relocalization is accepted only while LOST";
        })) {
        response->success = false;
        response->message.clear();
    }
}

/// 发起人工协调重置；缺少有效暖边界时，调用方须保证跨代期间停车。
void LocalizationNode::resetService(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    if (!invoke("reset request", [&] {
            const bool accepted =
                controller_ && controller_->requestCoordinatedReset(/*assume_stationary=*/true);
            response->success = accepted;
            response->message = accepted ? "coordinated front-end reset transaction started"
                                         : "front-end reset service unavailable or cooling down";
        })) {
        response->success = false;
        response->message.clear();
    }
}

/// 重建新前端代次的订阅队列和输出缓存。
void LocalizationNode::onFrontendResetBoundary() {
    configureInputs();
    publisher_.resetInput();
    logged_first_frame_ = false;
    RCLCPP_WARN_STREAM(get_logger(), "front-end reset transaction completed; accepting epoch "
                                         << frontend_.epoch());
}

}  // namespace backend::ros
