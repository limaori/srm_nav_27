/**
 * @file visualization.cpp
 * @brief 将前端原子观测转换为独立 RViz 使用的配准点云和有界轨迹。
 */
#include <Eigen/Geometry>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <iostream>
#include <iterator>
#include <limits>
#include <list>
#include <localization_interfaces/msg/frontend_frame.hpp>
#include <localization_interfaces/msg/frontend_session.hpp>
#include <memory>
#include <nav_msgs/msg/path.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/header.hpp>
#include <stdexcept>
#include <unordered_map>

#include "zlio/voxel.hpp"

namespace zlio::ros {
namespace {

using Frame = localization_interfaces::msg::FrontendFrame;
using Session = localization_interfaces::msg::FrontendSession;

/// 保存固定体素的显示代表点；重复观测只刷新保留顺序，不移动既有显示点。
class DisplayMap {
public:
    /// 以空间分辨率和点数上限约束显示缓存。
    DisplayMap(double resolution, std::size_t capacity)
        : resolution_(resolution), capacity_(capacity) {}

    /// 清理当前会话的全部点，下一张快照为空。
    void clear() {
        cells_.clear();
        recent_.clear();
    }

    /// 合并新增的 odom 系回波，超限时逐体素淘汰最久未观测的点。
    void insert(const sensor_msgs::msg::PointCloud2& cloud) {
        sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
        const double key_limit = static_cast<double>(std::numeric_limits<std::int64_t>::max());
        for (; x != x.end(); ++x, ++y, ++z) {
            const Vec3 point(*x, *y, *z);
            const Vec3 scaled = point / resolution_;
            if (!scaled.allFinite() || scaled.cwiseAbs().maxCoeff() >= key_limit) {
                continue;
            }
            const VoxelKey key = voxelKey(point, resolution_);
            const auto existing = cells_.find(key);
            if (existing != cells_.end()) {
                recent_.splice(recent_.end(), recent_, existing->second.recent);
                continue;
            }
            recent_.push_back(key);
            try {
                cells_.emplace(key, Cell{{*x, *y, *z}, std::prev(recent_.end())});
            } catch (...) {
                recent_.pop_back();
                throw;
            }
            if (cells_.size() > capacity_) {
                cells_.erase(recent_.front());
                recent_.pop_front();
            }
        }
    }

    /// 按最近观测顺序编码完整显示快照，消息中不包含重复体素。
    sensor_msgs::msg::PointCloud2 snapshot(const std_msgs::msg::Header& header) const {
        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header = header;
        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(cells_.size());
        cloud.is_dense = true;
        sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
        for (const auto& key : recent_) {
            const auto& point = cells_.at(key).point;
            *x = point[0];
            *y = point[1];
            *z = point[2];
            ++x;
            ++y;
            ++z;
        }
        return cloud;
    }

private:
    struct Cell {
        std::array<float, 3> point;
        std::list<VoxelKey>::iterator recent;
    };
    double resolution_;
    std::size_t capacity_;
    std::list<VoxelKey> recent_;
    std::unordered_map<VoxelKey, Cell, VoxelHash> cells_;
};

/// 使用同一观测的后验变换去畸变点云，输出坐标和时间均来自前端观测。
sensor_msgs::msg::PointCloud2 registeredCloud(const Frame& frame) {
    const auto& pose = frame.odometry.pose.pose;
    const Eigen::Vector3d position(pose.position.x, pose.position.y, pose.position.z);
    Eigen::Quaterniond rotation(pose.orientation.w, pose.orientation.x, pose.orientation.y,
                                pose.orientation.z);
    if (!position.allFinite() || !rotation.coeffs().allFinite() || rotation.squaredNorm() < 1e-12) {
        throw std::invalid_argument("visualization received an invalid front-end pose");
    }
    rotation.normalize();
    auto cloud = frame.cloud;
    cloud.header = frame.odometry.header;
    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
    for (; x != x.end(); ++x, ++y, ++z) {
        const Eigen::Vector3d point = rotation * Eigen::Vector3d(*x, *y, *z) + position;
        *x = static_cast<float>(point.x());
        *y = static_cast<float>(point.y());
        *z = static_cast<float>(point.z());
    }
    return cloud;
}

/// 维护前端轨迹与地图显示历史，并发布可视化消息。
class FrontendVisualizer final : public rclcpp::Node {
public:
    /// 建立前端观测订阅和标准 ROS 可视化输出。
    FrontendVisualizer() : Node("frontend_visualization") {
        const auto limit = declare_parameter<int>("path_max_poses", 5000);
        if (limit <= 0) {
            throw std::invalid_argument("path_max_poses must be positive");
        }
        path_limit_ = static_cast<std::size_t>(limit);
        const double map_resolution = declare_parameter<double>("map_voxel_size", 0.15);
        const int map_capacity = declare_parameter<int>("map_max_voxels", 200000);
        const double map_period = declare_parameter<double>("map_publish_period", 0.2);
        if (!std::isfinite(map_resolution) || map_resolution <= 0 || map_capacity <= 0 ||
            !std::isfinite(map_period) || map_period < 0 || map_period > 60) {
            throw std::invalid_argument("invalid visualization map resolution, capacity or period");
        }
        map_ = std::make_unique<DisplayMap>(map_resolution, static_cast<std::size_t>(map_capacity));
        map_period_ns_ = static_cast<std::int64_t>(map_period * 1e9);
        path_.header.frame_id = "odom";
        cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("frontend/cloud_registered",
                                                                     rclcpp::QoS(1));
        path_pub_ = create_publisher<nav_msgs::msg::Path>("frontend/path",
                                                          rclcpp::QoS(1).transient_local());
        map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            "frontend/cloud_map", rclcpp::QoS(1).transient_local());
        session_sub_ = create_subscription<Session>(
            "frontend/session", rclcpp::QoS(1).transient_local(),
            [this](Session::ConstSharedPtr message) { onSession(*message); });
        frame_sub_ = create_subscription<Frame>(
            "frontend/frame", rclcpp::QoS(5).best_effort(),
            [this](Frame::ConstSharedPtr message) { onFrame(*message); });
    }

private:
    /// 同时清理点云与轨迹，让 RViz 和迟加入的订阅者都收到空的显示快照。
    void clearDisplay() {
        path_.poses.clear();
        path_.header.stamp = builtin_interfaces::msg::Time{};
        map_->clear();
        last_map_stamp_.reset();
        const auto empty = map_->snapshot(path_.header);
        cloud_pub_->publish(empty);
        map_pub_->publish(empty);
        path_pub_->publish(path_);
    }

    /// 会话边界统一清理显示，拒绝同一前端进程中迟到的旧代次通知。
    void onSession(const Session& session) {
        if (has_session_ && session.instance_id == instance_ && session.epoch <= epoch_) {
            return;
        }
        has_session_ = true;
        instance_ = session.instance_id;
        epoch_ = session.epoch;
        sequence_ = 0;
        clearDisplay();
    }

    /// 发布当前会话的配准点云与轨迹，显示故障只影响本次可视化输出。
    void onFrame(const Frame& frame) {
        if (!has_session_ || frame.instance_id != instance_ || frame.epoch != epoch_ ||
            frame.sequence <= sequence_) {
            return;
        }
        if (frame.odometry.header.frame_id.empty() ||
            frame.cloud.header.frame_id != frame.odometry.child_frame_id ||
            frame.cloud.header.stamp != frame.odometry.header.stamp) {
            RCLCPP_WARN(get_logger(), "Skipping visualization frame with inconsistent headers");
            return;
        }
        try {
            auto cloud = registeredCloud(frame);
            if (path_.header.frame_id != frame.odometry.header.frame_id) {
                path_.header.frame_id = frame.odometry.header.frame_id;
                clearDisplay();
            }
            map_->insert(cloud);
            path_.header = frame.odometry.header;
            geometry_msgs::msg::PoseStamped pose;
            pose.header = frame.odometry.header;
            pose.pose = frame.odometry.pose.pose;
            if (path_.poses.size() == path_limit_) {
                path_.poses.erase(path_.poses.begin());
            }
            path_.poses.push_back(pose);
            sequence_ = frame.sequence;
            cloud_pub_->publish(cloud);
            path_pub_->publish(path_);
            const std::int64_t stamp = rclcpp::Time(cloud.header.stamp).nanoseconds();
            if (!last_map_stamp_ || stamp - *last_map_stamp_ >= map_period_ns_) {
                map_pub_->publish(map_->snapshot(cloud.header));
                last_map_stamp_ = stamp;
            }
        } catch (const std::exception& error) {
            RCLCPP_WARN(get_logger(), "Skipping visualization frame: %s", error.what());
        }
    }

    bool has_session_ = false;
    std::uint64_t instance_ = 0, epoch_ = 0, sequence_ = 0;
    std::size_t path_limit_ = 5000;
    nav_msgs::msg::Path path_;
    std::unique_ptr<DisplayMap> map_;
    std::optional<std::int64_t> last_map_stamp_;
    std::int64_t map_period_ns_ = 200000000;
    rclcpp::Subscription<Session>::SharedPtr session_sub_;
    rclcpp::Subscription<Frame>::SharedPtr frame_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_, map_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
};

}  // namespace
}  // namespace zlio::ros

/// 启动独立的前端可视化适配器。
int main(int argc, char** argv) {
    try {
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<zlio::ros::FrontendVisualizer>());
        rclcpp::shutdown();
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Front-end visualization failed: " << error.what() << '\n';
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
        return 1;
    }
}
