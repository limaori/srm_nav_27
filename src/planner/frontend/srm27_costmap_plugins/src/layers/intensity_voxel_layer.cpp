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

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include "sensor_msgs/point_cloud2_iterator.hpp"

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

  // 残影保持：0（默认）＝旧语义（每周期重建、无记忆）；> 0 ＝超过该时长未再被观测才擦除。
  hold_time_ = node->declare_parameter(name_ + ".hold_time", 0.0);
  const auto hold_max_cells = node->declare_parameter(name_ + ".hold_max_cells", 200000);
  if (!std::isfinite(hold_time_) || hold_time_ < 0.0) {
    RCLCPP_WARN(
      node->get_logger(), "IntensityVoxelLayer[%s]: hold_time %.3f 非法，按 0 处理",
      name_.c_str(), hold_time_);
    hold_time_ = 0.0;
  }
  if (hold_max_cells > 0) {
    hold_max_cells_ = static_cast<std::size_t>(hold_max_cells);
  }
  hold_resolution_ = resolution_;
  hold_size_x_ = size_x_;
  hold_size_y_ = size_y_;

  if (publish_voxel_) {
    voxel_pub_ = node->create_publisher<nav2_msgs::msg::VoxelGrid>("voxel_grid", 1);
  }

  matchSize();
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

  // 尺寸或分辨率变化后旧的世界格键不再对应同一块空间，整表作废。
  if (
    hold_size_x_ != size_x_ || hold_size_y_ != size_y_ ||
    std::abs(hold_resolution_ - resolution_) > 1.0e-12)
  {
    last_seen_.clear();
    cell_offset_x_ = 0;
    cell_offset_y_ = 0;
    hold_size_x_ = size_x_;
    hold_size_y_ = size_y_;
    hold_resolution_ = resolution_;
  }
}

void IntensityVoxelLayer::reset()
{
  ObstacleLayer::reset();
  last_seen_.clear();
  resetMaps();
}

void IntensityVoxelLayer::enforceHoldCap()
{
  if (hold_max_cells_ == 0 || last_seen_.size() <= hold_max_cells_) {
    hold_cap_warned_ = false;
    return;
  }
  if (!hold_cap_warned_) {
    auto node = node_.lock();
    if (node) {
      RCLCPP_WARN(
        node->get_logger(),
        "IntensityVoxelLayer[%s]: 残影表 %zu 格超过 hold_max_cells=%zu，整表清空",
        name_.c_str(), last_seen_.size(), hold_max_cells_);
    }
    hold_cap_warned_ = true;
  }
  last_seen_.clear();
}

std::uint64_t IntensityVoxelLayer::cellKey(long _ix, long _iy)
{
  // 两个分量各平移 2^31 变成非负，再打包进 64 位；0.05 m 分辨率下覆盖 ±1e8 m。
  const std::uint64_t x = static_cast<std::uint64_t>(_ix + (1L << 31));
  const std::uint64_t y = static_cast<std::uint64_t>(_iy + (1L << 31));
  return (x << 32) | (y & 0xffffffffULL);
}

void IntensityVoxelLayer::keyToCell(std::uint64_t _key, long & _ix, long & _iy)
{
  _ix = static_cast<long>((_key >> 32) & 0xffffffffULL) - (1L << 31);
  _iy = static_cast<long>(_key & 0xffffffffULL) - (1L << 31);
}

bool IntensityVoxelLayer::pointInFootprint(
  double _x, double _y, const std::vector<geometry_msgs::msg::Point> & _polygon)
{
  const std::size_t n = _polygon.size();
  if (n < 3) {
    return false;
  }
  bool inside = false;
  for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
    const double xi = _polygon[i].x, yi = _polygon[i].y;
    const double xj = _polygon[j].x, yj = _polygon[j].y;
    if (
      ((yi > _y) != (yj > _y)) &&
      (_x < (xj - xi) * (_y - yi) / (yj - yi) + xi))
    {
      inside = !inside;
    }
  }
  return inside;
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

  // 残影表：关掉 hold 立即释放；分辨率变化则整表作废（旧键不再对应同一块空间）。
  // 图层原点每周期可能滚动，残影键用"世界格 = 格偏移 + 图层格"，与原点解耦。
  const bool keep_history = hold_time_ > 0.0 && costmap_ != nullptr;
  if (!keep_history) {
    last_seen_.clear();
  } else if (std::abs(hold_resolution_ - resolution_) > 1.0e-12) {
    last_seen_.clear();
    hold_resolution_ = resolution_;
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

  // 残影表用的时间基准：与图层同一时钟（受 use_sim_time 影响）。
  const double now = keep_history ? clock_->now().seconds() : 0.0;

  // 车体 footprint 内"只清不保"：不记入残影表、重刷时也跳过。
  // 否则车体自身回波会在残影表里留一份，机器人走开后就变成跟车的幽灵障碍。
  const bool skip_footprint =
    keep_history && footprint_clearing_enabled_ && !getFootprint().empty();
  if (skip_footprint) {
    nav2_costmap_2d::transformFootprint(
      robot_x, robot_y, robot_yaw, getFootprint(), transformed_footprint_);
    fp_min_x_ = fp_min_y_ = std::numeric_limits<double>::infinity();
    fp_max_x_ = fp_max_y_ = -std::numeric_limits<double>::infinity();
    for (const auto & vertex : transformed_footprint_) {
      fp_min_x_ = std::min(fp_min_x_, static_cast<double>(vertex.x));
      fp_min_y_ = std::min(fp_min_y_, static_cast<double>(vertex.y));
      fp_max_x_ = std::max(fp_max_x_, static_cast<double>(vertex.x));
      fp_max_y_ = std::max(fp_max_y_, static_cast<double>(vertex.y));
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

      // mark the cell in the voxel grid and check if we should also mark it in the costmap
      if (voxel_grid_.markVoxelInMap(mx, my, mz, mark_threshold_)) {
        costmap_[getIndex(mx, my)] = LETHAL_OBSTACLE;
        touch(px, py, &current_min_x, &current_min_y, &current_max_x, &current_max_y);
        if (keep_history) {
          // 车身 footprint 内不进残影表（先用 bbox 快速排除，再判多边形）。
          bool in_footprint = false;
          if (
            skip_footprint && px >= fp_min_x_ && px <= fp_max_x_ && py >= fp_min_y_ &&
            py <= fp_max_y_)
          {
            in_footprint = pointInFootprint(px, py, transformed_footprint_);
          }
          if (!in_footprint) {
            // 记"世界格"，而不是图层格：滚动窗口移动后图层格会整体平移。
            last_seen_[cellKey(cell_offset_x_ + static_cast<long>(mx),
                               cell_offset_y_ + static_cast<long>(my))] = now;
          }
        }
      }
    }
  }

  if (keep_history) {
    // 1) 超过 hold_time 没再被观测到的格作废（这就是"残影"的寿命）。
    for (auto it = last_seen_.begin(); it != last_seen_.end();) {
      if (now - it->second > hold_time_) {
        it = last_seen_.erase(it);
      } else {
        ++it;
      }
    }

    // 2) 其余格重刷回本周期图层：本周期没看到、但还在寿命内的障碍靠这一步留在图上。
    //    滚动窗口外的历史先留在表里，等窗口再覆盖回来时自动恢复。
    for (const auto & entry : last_seen_) {
      long ix = 0, iy = 0;
      keyToCell(entry.first, ix, iy);
      const long mx = ix - cell_offset_x_;
      const long my = iy - cell_offset_y_;
      if (
        mx < 0 || my < 0 || mx >= static_cast<long>(size_x_) ||
        my >= static_cast<long>(size_y_))
      {
        continue;
      }
      const double wx = origin_x_ + (static_cast<double>(mx) + 0.5) * resolution_;
      const double wy = origin_y_ + (static_cast<double>(my) + 0.5) * resolution_;
      if (skip_footprint && pointInFootprint(wx, wy, transformed_footprint_)) {
        continue;
      }
      costmap_[getIndex(static_cast<unsigned int>(mx), static_cast<unsigned int>(my))] =
        LETHAL_OBSTACLE;
      touch(wx, wy, &current_min_x, &current_min_y, &current_max_x, &current_max_y);
    }

    enforceHoldCap();
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

  // 残影键用世界格，随窗口平移同步累加偏移；否则滚动后旧残影会错位一格。
  cell_offset_x_ += static_cast<long>(cell_ox);
  cell_offset_y_ += static_cast<long>(cell_oy);
}

}  // namespace srm27_nav_costmap

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(srm27_nav_costmap::IntensityVoxelLayer, nav2_costmap_2d::Layer)
