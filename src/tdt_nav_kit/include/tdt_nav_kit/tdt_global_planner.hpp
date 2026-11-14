// Copyright 2026 SRM27 Navigation Team
// Licensed under the Apache License, Version 2.0.
//
// 上游 vendor/ 内的文件适用 MIT，见 vendor/LICENCE 与 THIRD_PARTY_NOTICES.md。

#ifndef TDT_NAV_KIT__TDT_GLOBAL_PLANNER_HPP_
#define TDT_NAV_KIT__TDT_GLOBAL_PLANNER_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tdt_nav_kit/costmap_converter.hpp"
#include "tf2_ros/buffer.h"

namespace tdt_nav_kit
{

/// 把 vendored YAstar 包装成 Nav2 全局规划器。
///
/// 阶段一只输出 nav_msgs/msg/Path（map 帧），不输出速度；
/// Costmap / 控制器 / 平滑器 / 底盘链路保持原样。
///
/// 签名已按 ros-navigation/navigation2 的 humble 分支头文件核对（2026-11-04）：
///   nav2_core/include/nav2_core/global_planner.hpp
/// 注意 Humble 的 createPlan 是**两参**（start, goal），没有较新版本的
/// cancel_checker 第三参数；configure 的 name 为按值 std::string。
class TdtGlobalPlanner : public nav2_core::GlobalPlanner
{
public:
  TdtGlobalPlanner() = default;
  ~TdtGlobalPlanner() override = default;

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

  ConverterConfig converter_cfg_;
  int mask_num_layers_{1};
  float cost_inflate_radius_{1.0f};
  float simplify_epsilon_{0.1f};
  float save_point_max_radius_{0.4f};
  float path_resample_step_{0.0f};
  /// "identity"（默认 w=1）或 "tangent"（由相邻路径点求 yaw）。
  std::string orientation_mode_{"identity"};

  /// 打开后在每次 createPlan 末尾打印分段耗时，用于定位瓶颈（默认关闭）。
  bool debug_timing_{false};
};

}  // namespace tdt_nav_kit

#endif  // TDT_NAV_KIT__TDT_GLOBAL_PLANNER_HPP_
