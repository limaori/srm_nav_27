/**
 * @file publisher_state.cpp
 * @brief 将已提交状态、动态坐标变换和位姿协方差发布为 ROS 消息。
 */

#include <sstream>

#include "apps/publisher.hpp"
#include "apps/ros.hpp"
#include "backend/covariance.hpp"
#include "backend/localization.hpp"

namespace backend::ros {
namespace {
/// 将状态字段中的分隔符及换行按百分号编码，保持每条机器字段完整。
void writeStateField(std::ostream& output, const std::string& value) {
    for (const char character : value) {
        switch (character) {
            case '%':
                output << "%25";
                break;
            case ';':
                output << "%3B";
                break;
            case '\r':
                output << "%0D";
                break;
            case '\n':
                output << "%0A";
                break;
            default:
                output << character;
                break;
        }
    }
}
}  // namespace

/// 发布定位状态文本、兼容危险标志及按需启用的状态标记。
void Publisher::publishLocalizationState(const LocalizationStatus status, const std::string& reason,
                                         const EstimatorFailureHistory& estimator_failures,
                                         const rclcpp::Time& stamp, const bool tf_published,
                                         const std::uint64_t epoch,
                                         const std::uint64_t state_generation,
                                         const std::uint64_t pose_revision) {
    publishLegacyDanger(status);
    if (!state_pub_) {
        return;
    }
    std_msgs::msg::String message;
    std::ostringstream text;
    text << "status=" << LocalizationStateMachine::statusName(status)
         << ";pose_trusted=" << (LocalizationStateMachine::poseTrusted(status) ? "true" : "false")
         << ";tf_frozen=" << (LocalizationStateMachine::tfFrozen(status) ? "true" : "false")
         << ";tf_published=" << (tf_published ? "true" : "false") << ";epoch=" << epoch
         << ";generation=" << state_generation << ";pose_revision=" << pose_revision
         << ";stamp_ns=" << stamp.nanoseconds()
         << ";estimator_failure_count=" << estimator_failures.count << ";last_estimator_failure=";
    writeStateField(text, estimator_failures.last_reason);
    text << ";reason=";
    writeStateField(text, reason);
    message.data = text.str();
    state_pub_->publish(message);

    publishOptional("status visualization",
                    [&] { publishStatusMarker(status, reason, estimator_failures, stamp, epoch); });
}

/// 捕获最近机体位置并发布当前定位状态、颜色及原因标记。
void Publisher::publishStatusMarker(LocalizationStatus status, const std::string& reason,
                                    const EstimatorFailureHistory& estimator_failures,
                                    const rclcpp::Time& stamp, std::uint64_t epoch) {
    if (!status_marker_pub_) {
        return;
    }
    Eigen::Vector3d marker_position;
    {
        std::lock_guard<std::mutex> lock(output_mutex_);
        marker_position = last_visualized_body_position_;
    }
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = parameters_.input.map_frame;
    marker.header.stamp = timeToMsg(stamp);
    marker.ns = "localization_status";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = marker_position.x();
    marker.pose.position.y = marker_position.y();
    marker.pose.position.z = marker_position.z() + 1.5;
    marker.pose.orientation.w = 1.0;
    marker.scale.z = 0.55;
    marker.color.a = 0.95F;
    switch (status) {
        case LocalizationStatus::Normal:
            marker.color.r = 0.15F;
            marker.color.g = 1.0F;
            marker.color.b = 0.3F;
            break;
        case LocalizationStatus::ObsDegraded:
            marker.color.r = 1.0F;
            marker.color.g = 0.85F;
            marker.color.b = 0.15F;
            break;
        case LocalizationStatus::Suspect:
            marker.color.r = 1.0F;
            marker.color.g = 0.45F;
            marker.color.b = 0.05F;
            break;
        case LocalizationStatus::Lost:
            marker.color.r = 1.0F;
            marker.color.g = 0.1F;
            marker.color.b = 0.1F;
            break;
        case LocalizationStatus::Initializing:
            marker.color.r = 0.2F;
            marker.color.g = 0.65F;
            marker.color.b = 1.0F;
            break;
    }
    marker.text = std::string(LocalizationStateMachine::statusName(status)) + " | epoch " +
                  std::to_string(epoch) + "\n" + reason;
    if (estimator_failures.count != 0)
        marker.text += "\nestimator failures: " + std::to_string(estimator_failures.count) +
                       "\nlast: " + estimator_failures.last_reason;
    status_marker_pub_->publish(marker);
}

/// 在发布锁内复核状态快照身份并发送定位诊断。
void Publisher::publishState(const LocalizationSnapshot& snapshot) {
    std::lock_guard<std::mutex> publication_lock(publication_mutex_);
    if (!controller_)
        return;
    const auto current = controller_->snapshot();
    if (snapshot.epoch != current.epoch || snapshot.pose_revision != current.pose_revision ||
        snapshot.localization.generation != current.localization.generation)
        return;
    publishLocalizationState(snapshot.localization.status, snapshot.localization.reason,
                             snapshot.localization.estimator_failures,
                             rclcpp::Time(0, 0, RCL_ROS_TIME),
                             /*tf_published=*/false, snapshot.epoch,
                             snapshot.localization.generation, snapshot.pose_revision);
}

/// 发布允许推进的 map → odom TF 及同版本定位状态。
void Publisher::publishProjection(const LocalizationProjection& projection,
                                  const builtin_interfaces::msg::Time& stamp) {
    if (!projection.should_publish_tf || !tf_broadcaster_) {
        return;
    }
    tf_broadcaster_->sendTransform(toRosTransform(
        projection.T_map_odom, parameters_.input.map_frame, parameters_.input.odom_frame, stamp));
    publishLocalizationState(projection.status, projection.reason, projection.estimator_failures,
                             rclcpp::Time(stamp),
                             /*tf_published=*/true, projection.epoch, projection.state_generation,
                             projection.pose_revision);
}

/// 将传感器位姿及协方差传播到机体原点并发布。
void Publisher::publishPoseWithCovariance(const LocalizationProjection& projection,
                                          const builtin_interfaces::msg::Time& stamp) {
    if (!pose_covariance_pub_) {
        return;
    }
    const Eigen::Matrix4d T_map_body =
        parameters_.input.body_transform.bodyPoseFromSensorPose(projection.T_map_sensor);
    PoseCovariance6d covariance = projection.pose_covariance_valid
                                      ? projection.pose_covariance
                                      : fallbackFrontendPoseCovariance();
    const Eigen::Vector3d lever_map =
        T_map_body.block<3, 1>(0, 3) - projection.T_map_sensor.block<3, 1>(0, 3);
    covariance = propagateLeverArmCovariance(covariance, lever_map);
    if (!projection.pose_trusted) {
        applyPoseCovarianceFloor(&covariance, 10.0, M_PI);
    }
    covariance = 0.5 * (covariance + covariance.transpose());

    geometry_msgs::msg::PoseWithCovarianceStamped message;
    message.header.frame_id = parameters_.input.map_frame;
    message.header.stamp = stamp;
    const Eigen::Vector3d translation = T_map_body.block<3, 1>(0, 3);
    {
        std::lock_guard<std::mutex> lock(output_mutex_);
        last_visualized_body_position_ = translation;
    }
    Eigen::Quaterniond orientation(T_map_body.block<3, 3>(0, 0));
    orientation.normalize();
    message.pose.pose.position.x = translation.x();
    message.pose.pose.position.y = translation.y();
    message.pose.pose.position.z = translation.z();
    message.pose.pose.orientation.x = orientation.x();
    message.pose.pose.orientation.y = orientation.y();
    message.pose.pose.orientation.z = orientation.z();
    message.pose.pose.orientation.w = orientation.w();
    writePoseCovariance(covariance, &message.pose.covariance);
    pose_covariance_pub_->publish(message);
}

}  // namespace backend::ros
