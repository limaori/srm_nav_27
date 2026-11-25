// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "srm27_costmap_plugins/layers/intensity_voxel_layer.hpp"

#include <cmath>
#include <limits>
#include <vector>

#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/time.h"
#include "tf2/utils.h"
#include "tf2/exceptions.h"

#define VOXEL_BITS 16

using nav2_costmap_2d::FREE_SPACE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

using nav2_costmap_2d::Observation;
using nav2_costmap_2d::ObservationBuffer;

namespace srm27_nav_costmap
{

void IntensityVoxelLayer::onInitialize()
{
  auto node = node_.lock();
  clock_ = node->get_clock();
  ObstacleLayer::onInitialize();
  footprint_clearing_enabled_ =
    node->get_parameter(name_ + ".footprint_clearing_enabled").as_bool();
  enabled_ = node->get_parameter(name_ + ".enabled").as_bool();
  max_obstacle_height_ = node->get_parameter(name_ + ".max_obstacle_height").as_double();
  combination_method_ = node->get_parameter(name_ + ".combination_method").as_int();

  size_z_ = node->declare_parameter(name_ + ".z_voxels", 16);
  origin_z_ = node->declare_parameter(name_ + ".origin_z", 16.0);
  min_obstacle_intensity_ = node->declare_parameter(name_ + ".min_obstacle_intensity", 0.1);
  max_obstacle_intensity_ = node->declare_parameter(name_ + ".max_obstacle_intensity", 2.0);
  z_resolution_ = node->declare_parameter(name_ + ".z_resolution", 0.05);
  unknown_threshold_ =
    node->declare_parameter(name_ + ".unknown_threshold", 15) + (VOXEL_BITS - size_z_);
  mark_threshold_ = node->declare_parameter(name_ + ".mark_threshold", 0);
  publish_voxel_ = node->declare_parameter(name_ + ".publish_voxel_map", false);

  // —— 静态差集过滤参数（2026-11-24 评审 §5/§8.3）——
  static_diff_only_ = node->declare_parameter(name_ + ".static_diff_only", false);
  static_map_topic_ = node->declare_parameter(name_ + ".static_map_topic", "map");
  static_occupied_threshold_ = static_cast<int8_t>(
    node->declare_parameter(name_ + ".static_occupied_threshold", 65));
  if (static_diff_only_) {
    // 与 StaticLayer 同款 transient local 订阅，拿一次全图常驻缓存。
    static_map_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
      static_map_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      std::bind(&IntensityVoxelLayer::staticMapCallback, this, std::placeholders::_1));
  }

  if (publish_voxel_) {
    voxel_pub_ = node->create_publisher<nav2_msgs::msg::VoxelGrid>("voxel_grid", 1);
  }

  matchSize();
}

void IntensityVoxelLayer::staticMapCallback(
  const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  // 回调跑在代价地图节点的 executor 上，任何未捕获异常都会终止容器
  // （2026-11-23 动态掩码 SIGABRT 教训），这里整体兜底。
  try {
    std::lock_guard<std::mutex> lock(static_map_mutex_);
    static_map_ = msg;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("IntensityVoxelLayer"), "静态图缓存异常: %s", e.what());
  }
}

IntensityVoxelLayer::~IntensityVoxelLayer() {}

void IntensityVoxelLayer::updateFootprint(
  double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y, double * max_x,
  double * max_y)
{
  if (!footprint_clearing_enabled_) {
    return;
  }

  nav2_costmap_2d::transformFootprint(
    robot_x, robot_y, robot_yaw, getFootprint(), transformed_footprint_);

  for (auto & i : transformed_footprint_) {
    touch(i.x, i.y, min_x, min_y, max_x, max_y);
  }

  setConvexPolygonCost(transformed_footprint_, nav2_costmap_2d::FREE_SPACE);
}

void IntensityVoxelLayer::matchSize()
{
  ObstacleLayer::matchSize();
  voxel_grid_.resize(size_x_, size_y_, size_z_);
}

void IntensityVoxelLayer::reset()
{
  ObstacleLayer::reset();
  resetMaps();
}

void IntensityVoxelLayer::resetMaps()
{
  ObstacleLayer::resetMaps();
  // This layer publishes a binary obstacle map. The base Costmap2D reset uses
  // its default value (often NO_INFORMATION), which would leave stale or
  // unknown costs after an observation disappears.
  if (costmap_ != nullptr && size_x_ > 0 && size_y_ > 0) {
    resetMapToValue(0, 0, size_x_, size_y_, FREE_SPACE);
  }
  voxel_grid_.reset();
}

void IntensityVoxelLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y, double * max_x,
  double * max_y)
{
  // update origin information for rolling costmap publication
  if (rolling_window_) {
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  }

  // reset maps each iteration
  resetMaps();

  // Include the old observation bounds so LayeredCostmap clears and recombines every
  // layer in that region, even when the latest cloud is empty or smaller.
  if (has_previous_bounds_) {
    touch(previous_min_x_, previous_min_y_, min_x, min_y, max_x, max_y);
    touch(previous_max_x_, previous_max_y_, min_x, min_y, max_x, max_y);
    has_previous_bounds_ = false;
  }

  // if not enabled, stop here
  if (!enabled_) {
    return;
  }

  // get the maximum sized window required to operate
  useExtraBounds(min_x, min_y, max_x, max_y);

  double current_min_x = std::numeric_limits<double>::infinity();
  double current_min_y = std::numeric_limits<double>::infinity();
  double current_max_x = -std::numeric_limits<double>::infinity();
  double current_max_y = -std::numeric_limits<double>::infinity();

  // get the marking observations
  bool current = true;
  std::vector<Observation> observations;
  current = getMarkingObservations(observations) && current;

  // update the global current status
  current_ = current;

  // —— 静态差集过滤上下文（每周期一次，2026-11-24 评审 §5/§8.3）——
  // global_frame_ 可能是 odom（局部代价地图），先换到静态图所在系（map）再查格。
  // TF 不可用时退化为不过滤（旧行为），保证真实障碍不因过滤器失效而漏标。
  bool diff_active = false;
  double diff_tx = 0.0, diff_ty = 0.0, diff_cos = 1.0, diff_sin = 0.0;
  nav_msgs::msg::OccupancyGrid::SharedPtr diff_map;
  int diff_w = 0, diff_h = 0;
  double diff_ox = 0.0, diff_oy = 0.0, diff_res = 0.05;
  if (static_diff_only_) {
    {
      std::lock_guard<std::mutex> lock(static_map_mutex_);
      diff_map = static_map_;
    }
    if (diff_map && tf_ != nullptr) {
      try {
        const auto tf_msg = tf_->lookupTransform(
          diff_map->header.frame_id, global_frame_, tf2::TimePointZero);
        diff_tx = tf_msg.transform.translation.x;
        diff_ty = tf_msg.transform.translation.y;
        const double yaw = tf2::getYaw(tf_msg.transform.rotation);
        diff_cos = std::cos(yaw);
        diff_sin = std::sin(yaw);
        diff_w = static_cast<int>(diff_map->info.width);
        diff_h = static_cast<int>(diff_map->info.height);
        diff_ox = diff_map->info.origin.position.x;
        diff_oy = diff_map->info.origin.position.y;
        diff_res = diff_map->info.resolution;
        diff_active = diff_w > 0 && diff_h > 0;
      } catch (const tf2::TransformException & e) {
        const double now_sec = clock_->now().seconds();
        if (now_sec - last_tf_warn_sec_ > 5.0) {
          last_tf_warn_sec_ = now_sec;
          RCLCPP_WARN(
            rclcpp::get_logger("IntensityVoxelLayer"),
            "静态差集过滤 TF 失败 (%s → %s)，本周期不过滤: %s",
            global_frame_.c_str(), diff_map->header.frame_id.c_str(), e.what());
        }
      }
    }
  }

  // place the new obstacles into a priority queue... each with a priority of zero to begin with
  for (const auto & obs : observations) {
    double sq_obstacle_max_range = obs.obstacle_max_range_ * obs.obstacle_max_range_;
    double sq_obstacle_min_range = obs.obstacle_min_range_ * obs.obstacle_min_range_;

    sensor_msgs::PointCloud2ConstIterator<float> it_x(*obs.cloud_, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(*obs.cloud_, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(*obs.cloud_, "z");
    sensor_msgs::PointCloud2ConstIterator<float> it_i(*obs.cloud_, "intensity");
    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z, ++it_i) {
      double px = *it_x, py = *it_y, pz = *it_z;

      // if the obstacle is too low/high, we won't add it
      if (pz < min_obstacle_height_ || pz > max_obstacle_height_) {
        continue;
      }

      // if the intensity is not in the range we want, we won't add it
      if (*it_i < min_obstacle_intensity_ || *it_i > max_obstacle_intensity_) {
        continue;
      }

      // compute the squared distance from the hitpoint to the pointcloud's origin
      double sq_dist = (px - obs.origin_.x) * (px - obs.origin_.x) +
                       (py - obs.origin_.y) * (py - obs.origin_.y) +
                       (pz - obs.origin_.z) * (pz - obs.origin_.z);

      // if the point is far/close enough away... we won't consider it
      if (sq_dist <= sq_obstacle_min_range || sq_dist >= sq_obstacle_max_range) {
        continue;
      }

      // now we need to compute the map coordinates for the observation
      unsigned int mx, my, mz;
      if (pz < origin_z_) {
        if (!worldToMap3D(px, py, origin_z_, mx, my, mz)) {
          continue;
        }
      } else if (!worldToMap3D(px, py, pz, mx, my, mz)) {
        continue;
      }

      // 静态差集过滤：该点落在静态占据/未知格 → 已有障碍，跳过（只标"新增"）。
      // 墙点(80% 抖动)几乎全部落在静态墙上，在此处被丢弃；未建图桌子/对手车
      // 落在自由格，正常标记。TF 未就绪时 diff_active=false，不过滤（旧行为）。
      if (diff_active) {
        const double mwx = diff_tx + diff_cos * px - diff_sin * py;
        const double mwy = diff_ty + diff_sin * px + diff_cos * py;
        const int si = static_cast<int>((mwx - diff_ox) / diff_res);
        const int sj = static_cast<int>((mwy - diff_oy) / diff_res);
        if (si >= 0 && si < diff_w && sj >= 0 && sj < diff_h) {
          const int8_t v = diff_map->data[sj * diff_w + si];
          if (v == -1 || v >= static_occupied_threshold_) {
            continue;
          }
        }
      }

      // mark the cell in the voxel grid and check if we should also mark it in the costmap
      if (voxel_grid_.markVoxelInMap(mx, my, mz, mark_threshold_)) {
        costmap_[getIndex(mx, my)] = LETHAL_OBSTACLE;
        touch(px, py, &current_min_x, &current_min_y, &current_max_x, &current_max_y);
      }
    }
  }

  if (publish_voxel_) {
    nav2_msgs::msg::VoxelGrid grid_msg;
    unsigned int size = voxel_grid_.sizeX() * voxel_grid_.sizeY();
    grid_msg.size_x = voxel_grid_.sizeX();
    grid_msg.size_y = voxel_grid_.sizeY();
    grid_msg.size_z = voxel_grid_.sizeZ();
    grid_msg.data.resize(size);
    memcpy(&grid_msg.data[0], voxel_grid_.getData(), size * sizeof(unsigned int));

    grid_msg.origin.x = origin_x_;
    grid_msg.origin.y = origin_y_;
    grid_msg.origin.z = origin_z_;

    grid_msg.resolutions.x = resolution_;
    grid_msg.resolutions.y = resolution_;
    grid_msg.resolutions.z = z_resolution_;
    grid_msg.header.frame_id = global_frame_;
    grid_msg.header.stamp = clock_->now();
    voxel_pub_->publish(grid_msg);
  }

  updateFootprint(
    robot_x, robot_y, robot_yaw, &current_min_x, &current_min_y, &current_max_x, &current_max_y);
  if (std::isfinite(current_min_x) && std::isfinite(current_min_y)) {
    previous_min_x_ = current_min_x;
    previous_min_y_ = current_min_y;
    previous_max_x_ = current_max_x;
    previous_max_y_ = current_max_y;
    has_previous_bounds_ = true;
    touch(current_min_x, current_min_y, min_x, min_y, max_x, max_y);
    touch(current_max_x, current_max_y, min_x, min_y, max_x, max_y);
  }
}

void IntensityVoxelLayer::updateOrigin(double new_origin_x, double new_origin_y)
{
  // project the new origin into the grid
  int cell_ox, cell_oy;
  cell_ox = static_cast<int>((new_origin_x - origin_x_) / resolution_);
  cell_oy = static_cast<int>((new_origin_y - origin_y_) / resolution_);

  // update the origin with the appropriate world coordinates
  origin_x_ = origin_x_ + cell_ox * resolution_;
  origin_y_ = origin_y_ + cell_oy * resolution_;
}

}  // namespace srm27_nav_costmap

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(srm27_nav_costmap::IntensityVoxelLayer, nav2_costmap_2d::Layer)
