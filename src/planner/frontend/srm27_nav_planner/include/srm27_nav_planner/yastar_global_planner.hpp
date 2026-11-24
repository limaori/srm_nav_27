// Copyright 2026 SRM27 Navigation Team
// Licensed under the Apache License, Version 2.0.
//
// 算法来自东北大学 T-DT 的 tdt-nav-kit（MIT），vendored 源码在本包
// vendor/src/YAstar/ 内，见本包 THIRD_PARTY_NOTICES.md 与 PATCHES.md。

#ifndef SRM27_NAV_PLANNER__YASTAR_GLOBAL_PLANNER_HPP_
#define SRM27_NAV_PLANNER__YASTAR_GLOBAL_PLANNER_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "srm27_map_adapter/costmap_adapter.hpp"
#include "srm27_nav_common/path_conversion.hpp"
#include "srm27_nav_planner/goal_resolver.hpp"
#include "tf2_ros/buffer.h"

namespace srm27_nav_planner
{

/// Nav2 全局规划器插件：YAstar（分层 Mask 的 A*）的包装层。
///
/// 只输出 nav_msgs/msg/Path（map 帧），不输出速度；Costmap / 控制器 /
/// 平滑器 / 底盘链路保持原样。
///
/// 签名已按 ros-navigation/navigation2 的 humble 分支头文件核对（2026-11-04）：
///   nav2_core/include/nav2_core/global_planner.hpp
/// 注意 Humble 的 createPlan 是**两参**（start, goal），没有较新版本的
/// cancel_checker 第三参数；configure 的 name 为按值 std::string。
class YAstarGlobalPlanner : public nav2_core::GlobalPlanner
{
public:
  YAstarGlobalPlanner() = default;
  ~YAstarGlobalPlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::string plugin_name_;
  rclcpp::Clock::SharedPtr clock_;
  bool active_{false};
  StableGoal stable_goal_;

  /// Nav2 代价图 → 地图快照。阈值与未知策略在 configure 时确定。
  srm27_map_adapter::CostmapAdapter costmap_adapter_;
  int occupancy_threshold_{252};
  bool treat_unknown_as_obstacle_{true};
  /// 上游 main.cpp: `setMaskNumLayers(4)`；本仓库不接动态 Mask，多层只是空层。
  int mask_num_layers_{4};
  float cost_inflate_radius_{1.0f};
  float simplify_epsilon_{0.1f};
  float save_point_max_radius_{0.4f};
  float goal_snap_max_radius_{0.4f};
  /// "identity"（默认 w=1）或 "tangent"（由相邻路径点求 yaw）。
  srm27_nav_common::OrientationMode orientation_mode_{
    srm27_nav_common::OrientationMode::kIdentity};

  /// 打开后在每次 createPlan 末尾打印分段耗时，用于定位瓶颈（默认关闭）。
  bool debug_timing_{false};
};

}  // namespace srm27_nav_planner

#endif  // SRM27_NAV_PLANNER__YASTAR_GLOBAL_PLANNER_HPP_
