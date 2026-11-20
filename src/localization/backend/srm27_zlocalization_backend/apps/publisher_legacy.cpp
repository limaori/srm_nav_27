/**
 * @file publisher_legacy.cpp
 * @brief 发布可选的 robot_navigation 位姿、速度、路径与危险状态接口。
 */

#include "apps/publisher.hpp"
#include "apps/ros.hpp"

namespace backend::ros {

/// 将定位可信性映射为兼容接口的危险状态码。
void Publisher::publishLegacyDanger(const LocalizationStatus status) {
    if (!parameters_.legacy.enabled || !legacy_danger_pub_) {
        return;
    }
    std_msgs::msg::Int16 message;
    message.data = LocalizationStateMachine::poseTrusted(status) ? 0 : 9;
    legacy_danger_pub_->publish(message);
}

/// 在输出锁内限频并裁剪兼容路径，随后发布本轮冻结的路径快照。
void Publisher::publishLegacyPath(const geometry_msgs::msg::PoseStamped& pose,
                                  const builtin_interfaces::msg::Time& stamp_message) {
    nav_msgs::msg::Path path;
    bool due = false;
    {
        std::lock_guard<std::mutex> lock(output_mutex_);
        const rclcpp::Time stamp(stamp_message);
        due = last_legacy_path_stamp_.nanoseconds() == 0 || parameters_.legacy.path_period <= 0.0 ||
              (stamp - last_legacy_path_stamp_).seconds() < 0.0 ||
              (stamp - last_legacy_path_stamp_).seconds() >= parameters_.legacy.path_period;
        if (due) {
            legacy_path_.header.frame_id = parameters_.input.map_frame;
            legacy_path_.header.stamp = stamp_message;
            legacy_path_.poses.push_back(pose);
            last_legacy_path_stamp_ = stamp;
            if (parameters_.legacy.path_max_size > 0 &&
                legacy_path_.poses.size() >
                    static_cast<std::size_t>(parameters_.legacy.path_max_size)) {
                const auto remove_count =
                    legacy_path_.poses.size() -
                    static_cast<std::size_t>(parameters_.legacy.path_max_size);
                legacy_path_.poses.erase(
                    legacy_path_.poses.begin(),
                    legacy_path_.poses.begin() + static_cast<std::ptrdiff_t>(remove_count));
            }
            path = legacy_path_;
        }
    }
    if (due) {
        legacy_path_pub_->publish(path);
    }
}

/// 按兼容策略发布机体位姿、路径、坐标变换及杆臂补偿速度。
void Publisher::publishLegacyOutput(const nav_msgs::msg::Odometry& odom,
                                    const LocalizationProjection& projection) {
    if (!parameters_.legacy.enabled ||
        (projection.status == LocalizationStatus::Initializing &&
         !parameters_.legacy.publish_initializing) ||
        (projection.status != LocalizationStatus::Initializing && !projection.pose_trusted &&
         !parameters_.legacy.publish_untrusted)) {
        return;
    }

    const Eigen::Matrix4d T_map_body =
        parameters_.input.body_transform.bodyPoseFromSensorPose(projection.T_map_sensor);
    const geometry_msgs::msg::PoseStamped pose =
        matrixToPoseStamped(T_map_body, parameters_.input.map_frame, odom.header.stamp);
    if (legacy_pose_pub_) {
        legacy_pose_pub_->publish(pose);
    }
    if (projection.global_pose_available && parameters_.legacy.publish_tf && tf_broadcaster_ &&
        !parameters_.input.body_frame.empty()) {
        tf_broadcaster_->sendTransform(toRosTransform(T_map_body, parameters_.input.map_frame,
                                                      parameters_.input.body_frame,
                                                      odom.header.stamp));
    }

    if (legacy_path_pub_)
        publishLegacyPath(pose, odom.header.stamp);

    if (!legacy_twist_pub_) {
        return;
    }
    Eigen::Vector3d linear_sensor = vectorFromRos(odom.twist.twist.linear);
    Eigen::Vector3d angular_sensor = vectorFromRos(odom.twist.twist.angular);
    if (!linear_sensor.allFinite() || !angular_sensor.allFinite()) {
        return;
    }
    if (!projection.pose_trusted) {
        linear_sensor.setZero();
        angular_sensor.setZero();
    }

    const Eigen::Matrix3d R_map_sensor = projection.T_map_sensor.block<3, 3>(0, 0);
    const Eigen::Matrix3d R_map_body = T_map_body.block<3, 3>(0, 0);
    const Eigen::Vector3d angular_map = R_map_sensor * angular_sensor;
    const Eigen::Vector3d lever_map = R_map_body * parameters_.input.body_transform.offset();
    const Eigen::Vector3d linear_body_map =
        R_map_sensor * linear_sensor + angular_map.cross(lever_map);

    geometry_msgs::msg::TwistStamped message;
    message.header.frame_id = parameters_.input.map_frame;
    message.header.stamp = odom.header.stamp;
    message.twist.linear.x = linear_body_map.x();
    message.twist.linear.y = linear_body_map.y();
    message.twist.linear.z = linear_body_map.z();
    message.twist.angular.x = angular_map.x();
    message.twist.angular.y = angular_map.y();
    message.twist.angular.z = angular_map.z();
    legacy_twist_pub_->publish(message);
}

}  // namespace backend::ros
