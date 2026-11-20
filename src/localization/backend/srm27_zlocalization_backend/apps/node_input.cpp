/**
 * @file node_input.cpp
 * @brief 验收 ZLIO 原子观测和会话身份，并投影高频里程计输出。
 */

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "apps/node.hpp"
#include "apps/ros.hpp"
#include "backend/covariance.hpp"

namespace backend::ros {
namespace {
/// 校验点云存储范围、字节序和单精度 XYZ 字段布局。
bool usableCloud(const sensor_msgs::msg::PointCloud2& cloud) {
    if (cloud.is_bigendian || !cloud.width || !cloud.height || !cloud.point_step ||
        cloud.row_step < static_cast<std::uint64_t>(cloud.width) * cloud.point_step ||
        cloud.data.size() < static_cast<std::uint64_t>(cloud.row_step) * cloud.height)
        return false;
    for (const auto* name : {"x", "y", "z"}) {
        const auto f = std::find_if(cloud.fields.begin(), cloud.fields.end(),
                                    [&](const auto& field) { return field.name == name; });
        if (f == cloud.fields.end() || f->datatype != sensor_msgs::msg::PointField::FLOAT32 ||
            f->count != 1 || static_cast<std::uint64_t>(f->offset) + 4 > cloud.point_step)
            return false;
    }
    return true;
}
}  // namespace

/// 创建带代次捕获的 ZLIO 会话、原子观测及高频预测订阅。
void LocalizationNode::configureZlioInputs(std::uint64_t subscription_epoch) {
    using Frame = localization_interfaces::msg::FrontendFrame;
    using Prediction = localization_interfaces::msg::FrontendOdometry;
    using Session = localization_interfaces::msg::FrontendSession;
    zlio_identity_ = create_subscription<Session>(
        parameters_.input.session_topic, rclcpp::QoS(1).transient_local(),
        [this, subscription_epoch](Session::ConstSharedPtr m) {
            invoke("front-end session", [&] {
                if (subscription_epoch == frontend_.epoch())
                    frontend_.acceptIdentity(m->instance_id, m->epoch, std::nullopt,
                                             m->reset_required, m->failed);
            });
        });
    zlio_frames_ = create_subscription<Frame>(
        parameters_.input.frame_topic, rclcpp::QoS(parameters_.input.subscription_queue),
        // 同时验收帧布局、坐标系、时间和会话身份后交给同步帧入口。
        [this, subscription_epoch](Frame::ConstSharedPtr m) {
            invoke("front-end frame", [&] {
                if (subscription_epoch != frontend_.epoch())
                    return;
                if (m->odometry.header.frame_id != parameters_.input.odom_frame ||
                    !validStamp(m->begin) || !validStamp(m->odometry.header.stamp) ||
                    m->odometry.child_frame_id != parameters_.input.sensor_frame ||
                    m->cloud.header.frame_id != m->odometry.child_frame_id ||
                    m->cloud.header.stamp != m->odometry.header.stamp ||
                    rclcpp::Time(m->begin) > rclcpp::Time(m->odometry.header.stamp) ||
                    !hasUsableOdometryPose(m->odometry) || !usableCloud(m->cloud)) {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                         "invalid ZLIO observation frame/time/point layout");
                    return;
                }
                if (!frontend_.acceptIdentity(m->instance_id, m->epoch, m->sequence))
                    return;
                // 别名 shared_ptr 共享原消息所有权，不重新同步或复制两路观测。
                frameCallback(nav_msgs::msg::Odometry::ConstSharedPtr(m, &m->odometry),
                              sensor_msgs::msg::PointCloud2::ConstSharedPtr(m, &m->cloud),
                              subscription_epoch);
            });
        });
    rclcpp::SubscriptionOptions options;
    options.callback_group = high_rate_callback_group_;
    zlio_predictions_ = create_subscription<Prediction>(
        parameters_.input.prediction_topic, rclcpp::QoS(200),
        // 高频预测经过相同会话屏障，保持其独立发布回调组。
        [this, subscription_epoch](Prediction::ConstSharedPtr m) {
            invoke("front-end prediction", [&] {
                if (subscription_epoch != frontend_.epoch() ||
                    m->odometry.header.frame_id != parameters_.input.odom_frame ||
                    !validStamp(m->odometry.header.stamp) ||
                    m->odometry.child_frame_id != parameters_.input.sensor_frame ||
                    !hasUsableOdometryPose(m->odometry))
                    return;
                if (frontend_.acceptIdentity(m->instance_id, m->epoch))
                    highRateOdomCallback(nav_msgs::msg::Odometry::ConstSharedPtr(m, &m->odometry),
                                         subscription_epoch);
            });
        },
        options);
}

}  // namespace backend::ros
