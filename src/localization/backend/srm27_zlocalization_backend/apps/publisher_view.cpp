/**
 * @file publisher_view.cpp
 * @brief 发布可选轨迹、回环、地图及配准诊断几何。
 */

#include <std_msgs/msg/color_rgba.hpp>

#include "apps/publisher.hpp"
#include "apps/ros.hpp"
#include "backend/geometry.hpp"
#include "backend/localization.hpp"
#include "backend/sampling.hpp"

namespace backend::ros {
namespace {

/// 将有效回环边的关键帧端点构造为线段标记。
visualization_msgs::msg::Marker loopMarker(const std::vector<Keyframe>& keyframes,
                                           const std::vector<std::pair<int, int>>& edges,
                                           const std::string& frame,
                                           const builtin_interfaces::msg::Time& stamp) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame;
    marker.header.stamp = stamp;
    marker.ns = "loop_edges";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.08;
    marker.color.r = 1.0F;
    marker.color.g = 0.8F;
    marker.color.b = 0.1F;
    marker.color.a = 1.0F;
    for (const auto& edge : edges) {
        if (edge.first < 0 || edge.second < 0 || edge.first >= static_cast<int>(keyframes.size()) ||
            edge.second >= static_cast<int>(keyframes.size())) {
            continue;
        }
        geometry_msgs::msg::Point from;
        geometry_msgs::msg::Point to;
        from.x = keyframes[static_cast<std::size_t>(edge.first)].optimized_pose(0, 3);
        from.y = keyframes[static_cast<std::size_t>(edge.first)].optimized_pose(1, 3);
        from.z = keyframes[static_cast<std::size_t>(edge.first)].optimized_pose(2, 3);
        to.x = keyframes[static_cast<std::size_t>(edge.second)].optimized_pose(0, 3);
        to.y = keyframes[static_cast<std::size_t>(edge.second)].optimized_pose(1, 3);
        to.z = keyframes[static_cast<std::size_t>(edge.second)].optimized_pose(2, 3);
        marker.points.push_back(from);
        marker.points.push_back(to);
    }
    return marker;
}

}  // namespace

/// 将当前轨迹及已提交回环边转换为按需启用的可视化消息。
void Publisher::publishVisualizationImpl(const VisualizationSnapshot& visualization) {
    const bool path_output =
        parameters_.visualization.publish_paths && (odom_path_pub_ || optimized_path_pub_);
    const bool loop_output = static_cast<bool>(loop_marker_pub_);
    const bool debug_output = visualization.include_debug_clouds &&
                              ((aligned_source_pub_ && visualization.debug_aligned_source) ||
                               (target_pub_ && visualization.debug_target));
    if (!path_output && !loop_output && !debug_output) {
        return;
    }
    const std::vector<Keyframe> keyframes = trajectory_.snapshot();
    // 轨迹和 TF 必须使用同一观测时间轴；回放时节点时钟可能仍是系统时间。
    // 空轨迹以零时刻清空显示，不引入与观测无关的时间戳。
    const builtin_interfaces::msg::Time stamp = keyframes.empty()
                                                    ? builtin_interfaces::msg::Time{}
                                                    : stampFromSec(keyframes.back().timestamp);
    if (parameters_.visualization.publish_paths && (odom_path_pub_ || optimized_path_pub_)) {
        nav_msgs::msg::Path odom_path;
        nav_msgs::msg::Path optimized_path;
        // 将原始 odom 轨迹整体对齐到同一快照的 map 系，保留其相对几何形状。
        Eigen::Matrix4d map_from_odom = Eigen::Matrix4d::Identity();
        if (!keyframes.empty()) {
            const Keyframe& latest = keyframes.back();
            map_from_odom = latest.optimized_pose * latest.odom_pose.inverse();
        }
        odom_path.header.frame_id = parameters_.input.map_frame;
        optimized_path.header.frame_id = parameters_.input.map_frame;
        odom_path.header.stamp = stamp;
        optimized_path.header.stamp = stamp;
        for (const Keyframe& keyframe : keyframes) {
            const auto keyframe_stamp = stampFromSec(keyframe.timestamp);
            // 原始 odom 轨迹只连接当前代次，避免将重置前后的不同原点混在一起。
            if (keyframe.epoch == keyframes.back().epoch) {
                odom_path.poses.push_back(matrixToPoseStamped(map_from_odom * keyframe.odom_pose,
                                                              parameters_.input.map_frame,
                                                              keyframe_stamp));
            }
            optimized_path.poses.push_back(matrixToPoseStamped(
                keyframe.optimized_pose, parameters_.input.map_frame, keyframe_stamp));
        }
        if (odom_path_pub_) {
            odom_path_pub_->publish(odom_path);
        }
        if (optimized_path_pub_) {
            optimized_path_pub_->publish(optimized_path);
        }
    }
    if (loop_marker_pub_) {
        loop_marker_pub_->publish(
            loopMarker(keyframes, visualization.loop_edges, parameters_.input.map_frame, stamp));
    }
    if (visualization.include_debug_clouds && aligned_source_pub_ &&
        visualization.debug_aligned_source) {
        aligned_source_pub_->publish(
            toRosCloud(*visualization.debug_aligned_source, parameters_.input.map_frame, stamp));
    }
    if (visualization.include_debug_clouds && target_pub_ && visualization.debug_target) {
        target_pub_->publish(
            toRosCloud(*visualization.debug_target, parameters_.input.map_frame, stamp));
    }
}

/// 按配置分辨率构建并发布不可变先验地图。
void Publisher::publishPriorMap() {
    if (!prior_map_pub_ || prior_map_.empty()) {
        return;
    }
    const Cloud::Ptr map = prior_map_.buildOptimizedMap(parameters_.maps.prior_voxel);
    prior_map_pub_->publish(toRosCloud(*map, parameters_.input.map_frame, timeToMsg(node_.now())));
}

/// 按配置分辨率构建并发布运行轨迹地图。
void Publisher::publishRuntimeMap() {
    if (!runtime_map_pub_ || trajectory_.empty()) {
        return;
    }
    const Cloud::Ptr map = trajectory_.buildOptimizedMap(parameters_.maps.runtime_voxel);
    runtime_map_pub_->publish(
        toRosCloud(*map, parameters_.input.map_frame, timeToMsg(node_.now())));
}

/// 发布地图坐标系的三条彩色坐标轴。
void Publisher::publishMapAxes() {
    if (!map_axes_pub_) {
        return;
    }
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = parameters_.input.map_frame;
    marker.header.stamp = timeToMsg(node_.now());
    marker.ns = "map_axes";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.08;
    marker.color.a = 1.0F;
    geometry_msgs::msg::Point origin;
    /// 为指定轴端点追加从原点出发的线段及对应颜色。
    const auto add_axis = [&](const double x, const double y, const double z, const float r,
                              const float g, const float b) {
        geometry_msgs::msg::Point end;
        end.x = x;
        end.y = y;
        end.z = z;
        marker.points.push_back(origin);
        marker.points.push_back(end);
        std_msgs::msg::ColorRGBA color;
        color.r = r;
        color.g = g;
        color.b = b;
        color.a = 1.0F;
        marker.colors.push_back(color);
        marker.colors.push_back(color);
    };
    add_axis(2.0, 0.0, 0.0, 1.0F, 0.0F, 0.0F);
    add_axis(0.0, 2.0, 0.0, 0.0F, 1.0F, 0.0F);
    add_axis(0.0, 0.0, 2.0, 0.0F, 0.0F, 1.0F);
    map_axes_pub_->publish(marker);
}

/// 在锁外变换点云，并在复核输出身份及发布周期后发送世界点云。
void Publisher::publishWorldCloud(const FrontendFrame& frame,
                                  const LocalizationProjection& projection,
                                  const builtin_interfaces::msg::Time& stamp) {
    if (!world_cloud_pub_ || !frame.cloud_local) {
        return;
    }
    if ((projection.status == LocalizationStatus::Initializing &&
         !parameters_.world_cloud.publish_initializing) ||
        (projection.status != LocalizationStatus::Initializing && !projection.pose_trusted &&
         !parameters_.world_cloud.publish_untrusted)) {
        return;
    }
    const rclcpp::Time cloud_stamp(stamp);
    {
        std::lock_guard<std::mutex> lock(output_mutex_);
        const double elapsed = (cloud_stamp - last_world_cloud_stamp_).seconds();
        if (last_world_cloud_stamp_.nanoseconds() != 0 && elapsed >= 0.0 &&
            elapsed < parameters_.world_cloud.publish_period)
            return;
    }
    // 在发布屏障外变换点云；随后复核身份，避免 NORMAL 中捕获的昂贵转换结果在
    // 并发状态转移或 reset 后泄漏出去。
    const Cloud::Ptr map_cloud = transformCloud(*frame.cloud_local, projection.T_map_sensor);
    const auto message = toRosCloud(*map_cloud, parameters_.input.map_frame, stamp);
    std::lock_guard<std::mutex> publication_lock(publication_mutex_);
    if (!controller_ || !controller_->projectionIsCurrent(projection)) {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(output_mutex_);
        if (last_world_cloud_stamp_.nanoseconds() != 0 &&
            parameters_.world_cloud.publish_period > 0.0) {
            const double elapsed = (cloud_stamp - last_world_cloud_stamp_).seconds();
            if (elapsed >= 0.0 && elapsed < parameters_.world_cloud.publish_period) {
                return;
            }
        }
        last_world_cloud_stamp_ = cloud_stamp;
    }
    world_cloud_pub_->publish(message);
}

}  // namespace backend::ros
