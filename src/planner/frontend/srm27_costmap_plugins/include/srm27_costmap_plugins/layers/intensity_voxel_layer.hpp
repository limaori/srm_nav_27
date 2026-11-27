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

#ifndef SRM27_COSTMAP_PLUGINS__LAYERS__INTENSITY_VOXEL_LAYER_HPP_
#define SRM27_COSTMAP_PLUGINS__LAYERS__INTENSITY_VOXEL_LAYER_HPP_

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

#include "laser_geometry/laser_geometry.hpp"
#include "message_filters/subscriber.h"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/observation_buffer.hpp"
#include "nav2_costmap_2d/obstacle_layer.hpp"
#include "nav2_msgs/msg/voxel_grid.hpp"
#include "nav2_voxel_grid/voxel_grid.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/point_cloud2.h"
#include "tf2_ros/message_filter.h"

namespace srm27_nav_costmap
{

class IntensityVoxelLayer : public nav2_costmap_2d::ObstacleLayer
{
public:
  IntensityVoxelLayer() : voxel_grid_(0, 0, 0)
  {
    // this is the unsigned char* member of parent class's parent class Costmap2D.
    costmap_ = nullptr;
  }

  ~IntensityVoxelLayer() override;

  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y,
    double * max_x, double * max_y) override;
  void updateOrigin(double new_origin_x, double new_origin_y) override;
  void matchSize() override;
  void reset() override;
  bool isClearable() override { return false; }

protected:
  void resetMaps() override;
  void updateFootprint(
    double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y,
    double * max_x, double * max_y);

private:
  /// \brief 把一个世界格坐标压成 64 位键（两个分量各平移 2^31 后打包）。
  static std::uint64_t cellKey(long _ix, long _iy);
  /// \brief 键拆回 (ix, iy)，与 `cellKey` 严格互逆。
  static void keyToCell(std::uint64_t _key, long & _ix, long & _iy);
  /// \brief 残影表内存上限保护：超限时整表作废并告警，避免无界增长。
  void enforceHoldCap();
  /// \brief 点是否落在（已变换到世界系的）车体多边形内；多边形为空视为"不在"。
  static bool pointInFootprint(
    double _x, double _y, const std::vector<geometry_msgs::msg::Point> & _polygon);

  bool publish_voxel_{false};
  rclcpp::Publisher<nav2_msgs::msg::VoxelGrid>::SharedPtr voxel_pub_;
  nav2_voxel_grid::VoxelGrid voxel_grid_;
  // ObstacleLayer::onInitialize() can call our virtual matchSize() before parameters load.
  double z_resolution_{0.05}, origin_z_{0.0};
  double min_obstacle_intensity_{0.1}, max_obstacle_intensity_{2.0};
  unsigned int unknown_threshold_{15}, mark_threshold_{0}, size_z_{16};
  rclcpp::Clock::SharedPtr clock_;
  bool has_previous_bounds_{false};
  double previous_min_x_{0.0}, previous_min_y_{0.0};
  double previous_max_x_{0.0}, previous_max_y_{0.0};

  // ---- 残影保持（hold_time > 0 时启用；= 0 时行为与旧版完全一致）----
  //
  // 旧版每个更新周期 `resetMaps()` 后按"最近 decayTime 秒的点云"重建，图层本身零记忆；
  // 云台半视野自转时同一障碍会被"看见 0.5 s、丢失 1~2 s"地反复刷新/擦掉。
  // 残影表把"最近一次被观测到的世界格"留在内存里，超过 hold_time 没再被观测才擦掉，
  // 使有效记忆 = 点云衰减窗 + hold_time，与云台重访周期解耦。
  //
  // 注意：这里只按时间保活，不含"扫过且为空"的证据（见文档 L1/L2 分级）。因此
  // hold_time 越长，移动目标留下的拖影越久，窄道调参时要一起看。
  double hold_time_{0.0};
  std::size_t hold_max_cells_{200000};
  /// \brief 世界格键 → 最近一次被观测到的时刻（ROS 秒，受 use_sim_time 影响）。
  std::unordered_map<std::uint64_t, double> last_seen_;
  /// \brief 残影表建立时的分辨率；分辨率变化时整表作废。
  double hold_resolution_{0.0};
  /// \brief 当前图层原点对应的格偏移：`世界格 = 格偏移 + 图层格`。
  /// 滚动窗口移动时按整数格累加，保证残影键不随窗口漂移。
  long cell_offset_x_{0};
  long cell_offset_y_{0};
  /// \brief 残影表建立时的图层尺寸；尺寸变化（换图/重建窗口）时整表作废。
  unsigned int hold_size_x_{0}, hold_size_y_{0};
  /// \brief 本周期车体 footprint 的世界系包围盒，用来给点数很多的观测做快速排除。
  double fp_min_x_{0.0}, fp_min_y_{0.0}, fp_max_x_{0.0}, fp_max_y_{0.0};
  bool hold_cap_warned_{false};
  // ---- 2026-11-27：判定收紧（默认值与旧行为逐位一致）----
  /// \brief 同一 0.05 m 格本周期的点计数；只有 >= min_obstacle_points_ 才标致命格。
  std::vector<unsigned int> cell_hits_;
  /// \brief 每格最少点数；1（默认）= 单点即障碍（旧行为）。
  unsigned int min_obstacle_points_{1};
  /// \brief 车体原点 R 内的点直接跳过（自车回波）；0（默认）= 不启用。
  double self_filter_radius_{0.0};
  inline bool worldToMap3DFloat(
    double wx, double wy, double wz, double & mx, double & my, double & mz)
  {
    if (wx < origin_x_ || wy < origin_y_ || wz < origin_z_) {
      return false;
    }
    mx = ((wx - origin_x_) / resolution_);
    my = ((wy - origin_y_) / resolution_);
    mz = ((wz - origin_z_) / z_resolution_);
    return mx < size_x_ && my < size_y_ && mz < size_z_;
  }

  inline bool worldToMap3D(
    double wx, double wy, double wz, unsigned int & mx, unsigned int & my, unsigned int & mz)
  {
    if (wx < origin_x_ || wy < origin_y_ || wz < origin_z_) {
      return false;
    }

    mx = static_cast<int>((wx - origin_x_) / resolution_);
    my = static_cast<int>((wy - origin_y_) / resolution_);
    mz = static_cast<int>((wz - origin_z_) / z_resolution_);

    return mx < size_x_ && my < size_y_ && mz < size_z_;
  }

  inline void mapToWorld3D(
    unsigned int mx, unsigned int my, unsigned int mz, double & wx, double & wy, double & wz)
  {
    // returns the center point of the cell
    wx = origin_x_ + (mx + 0.5) * resolution_;
    wy = origin_y_ + (my + 0.5) * resolution_;
    wz = origin_z_ + (mz + 0.5) * z_resolution_;
  }

  inline double dist(double x0, double y0, double z0, double x1, double y1, double z1)
  {
    return sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0) + (z1 - z0) * (z1 - z0));
  }
};

}  // namespace srm27_nav_costmap

#endif  // SRM27_COSTMAP_PLUGINS__LAYERS__INTENSITY_VOXEL_LAYER_HPP_
