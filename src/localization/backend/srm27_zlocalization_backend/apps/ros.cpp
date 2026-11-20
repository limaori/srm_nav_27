/**
 * @file ros.cpp
 * @brief 实现核心值与 ROS 时间、位姿、点云消息的边界转换。
 */
#include "apps/ros.hpp"

#include <pcl_conversions/pcl_conversions.h>

#include <cmath>
namespace backend::ros {
/// 验收输入消息的非负秒数及规范纳秒字段。
bool validStamp(const builtin_interfaces::msg::Time& time) {
    return time.sec >= 0 && time.nanosec < 1000000000;
}

/// 检查位置和四元数分量有限，并拒绝无法安全归一化的近零四元数。
bool hasUsableOdometryPose(const nav_msgs::msg::Odometry& odom) {
    const auto& position = odom.pose.pose.position;
    const auto& orientation = odom.pose.pose.orientation;
    const bool finite = std::isfinite(position.x) && std::isfinite(position.y) &&
                        std::isfinite(position.z) && std::isfinite(orientation.x) &&
                        std::isfinite(orientation.y) && std::isfinite(orientation.z) &&
                        std::isfinite(orientation.w);
    if (!finite) {
        return false;
    }
    const double norm_squared = orientation.x * orientation.x + orientation.y * orientation.y +
                                orientation.z * orientation.z + orientation.w * orientation.w;
    return norm_squared > 1.0e-12;
}

/// 保持整数纳秒精度，将 ROS 时间值传入领域层。
Timestamp toTimestamp(const rclcpp::Time& time) {
    return Timestamp::fromNanoseconds(time.nanoseconds());
}

/// 归一化已验收的四元数，构造传感器子坐标系到里程计参考系的位姿。
Eigen::Matrix4d odomToMatrix(const nav_msgs::msg::Odometry& odom) {
    Eigen::Quaterniond q(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,
                         odom.pose.pose.orientation.y, odom.pose.pose.orientation.z);
    q.normalize();

    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    pose.block<3, 3>(0, 0) = q.toRotationMatrix();
    pose(0, 3) = odom.pose.pose.position.x;
    pose(1, 3) = odom.pose.pose.position.y;
    pose(2, 3) = odom.pose.pose.position.z;
    return pose;
}

/// 由参考系中的齐次位姿生成消息，保留指定坐标系和观测时间。
geometry_msgs::msg::PoseStamped matrixToPoseStamped(const Eigen::Matrix4d& pose,
                                                    const std::string& frame_id,
                                                    const builtin_interfaces::msg::Time& stamp) {
    Eigen::Quaterniond q(pose.block<3, 3>(0, 0));
    q.normalize();

    geometry_msgs::msg::PoseStamped out;
    out.header.frame_id = frame_id;
    out.header.stamp = stamp;
    out.pose.position.x = pose(0, 3);
    out.pose.position.y = pose(1, 3);
    out.pose.position.z = pose(2, 3);
    out.pose.orientation.x = q.x();
    out.pose.orientation.y = q.y();
    out.pose.orientation.z = q.z();
    out.pose.orientation.w = q.w();
    return out;
}

/// 将秒值舍入为整数纳秒，再拆分为 ROS 时间消息的秒和纳秒字段。
builtin_interfaces::msg::Time stampFromSec(double sec) {
    builtin_interfaces::msg::Time stamp;
    if (!std::isfinite(sec)) {
        return stamp;
    }
    constexpr std::int64_t kNsPerSec = 1000000000LL;
    const std::int64_t total_ns = static_cast<std::int64_t>(std::llround(sec * kNsPerSec));
    std::int64_t whole = total_ns / kNsPerSec;
    std::int64_t nsec = total_ns % kNsPerSec;
    // C++ 负数取余仍可能为负；向秒字段借一秒，使 nanosec 落在 [0, 1e9)。
    if (nsec < 0) {
        --whole;
        nsec += kNsPerSec;
    }
    stamp.sec = static_cast<int32_t>(whole);
    stamp.nanosec = static_cast<uint32_t>(nsec);
    return stamp;
}

/// 使用 rclcpp 的消息转换保留已有时间值。
builtin_interfaces::msg::Time timeToMsg(const rclcpp::Time& time) {
    return time;
}

/// 编码点云字段，并填入其已所属的坐标系及采样时刻。
sensor_msgs::msg::PointCloud2 toRosCloud(const Cloud& cloud, const std::string& frame_id,
                                         const builtin_interfaces::msg::Time& stamp) {
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    msg.header.frame_id = frame_id;
    msg.header.stamp = stamp;
    return msg;
}

/// 将子系在父系中的位置与旋转写入 TF 消息，并归一化输出四元数。
geometry_msgs::msg::TransformStamped toRosTransform(const Eigen::Matrix4d& transform,
                                                    const std::string& parent_frame,
                                                    const std::string& child_frame,
                                                    const builtin_interfaces::msg::Time& stamp) {
    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = parent_frame;
    msg.child_frame_id = child_frame;
    const Eigen::Vector3d t = transform.block<3, 1>(0, 3);
    Eigen::Quaterniond q(transform.block<3, 3>(0, 0));
    q.normalize();
    msg.transform.translation.x = t.x();
    msg.transform.translation.y = t.y();
    msg.transform.translation.z = t.z();
    msg.transform.rotation.x = q.x();
    msg.transform.rotation.y = q.y();
    msg.transform.rotation.z = q.z();
    msg.transform.rotation.w = q.w();
    return msg;
}

/// 按 x、y、z 顺序复制向量分量，保持输入坐标约定。
Eigen::Vector3d vectorFromRos(const geometry_msgs::msg::Vector3& vector) {
    return Eigen::Vector3d(vector.x, vector.y, vector.z);
}
}  // namespace backend::ros
