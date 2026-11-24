// Copyright 2026 SRM27 Navigation Team
// Licensed under the Apache License, Version 2.0.
//
// 算法来自东北大学 T-DT 的 tdt-nav-kit（MIT），vendored 源码在本包
// vendor/src/YAstar/ 内，见本包 THIRD_PARTY_NOTICES.md 与 PATCHES.md。

#include "srm27_nav_planner/yastar_global_planner.hpp"

#include <cmath>
#include <string>
#include <vector>

#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "srm27_map_core/occupancy_geometry.hpp"
#include "srm27_nav_common/path_utils.hpp"
#include "srm27_nav_planner/yastar_path.hpp"
#include "yastar.hpp"

using nav2_util::declare_parameter_if_not_declared;
using srm27_map_core::nearestFreePoint;
using srm27_map_core::pathIsFree;
using srm27_nav_common::resamplePath;
using srm27_nav_common::toPath;

namespace srm27_nav_planner
{

void YAstarGlobalPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  if (!node) {
    throw nav2_core::PlannerException("YAstarGlobalPlanner: unable to lock lifecycle node");
  }
  if (!costmap_ros) {
    throw nav2_core::PlannerException("YAstarGlobalPlanner: costmap_ros is null");
  }

  node_ = parent;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  plugin_name_ = name;
  clock_ = node->get_clock();
  stable_goal_.reset();

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".occupancy_threshold",
    rclcpp::ParameterValue(occupancy_threshold_));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".treat_unknown_as_obstacle",
    rclcpp::ParameterValue(treat_unknown_as_obstacle_));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".mask_num_layers", rclcpp::ParameterValue(mask_num_layers_));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".cost_inflate_radius",
    rclcpp::ParameterValue(static_cast<double>(cost_inflate_radius_)));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".simplify_epsilon",
    rclcpp::ParameterValue(static_cast<double>(simplify_epsilon_)));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".save_point_max_radius",
    rclcpp::ParameterValue(static_cast<double>(save_point_max_radius_)));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".path_resample_step",
    rclcpp::ParameterValue(static_cast<double>(path_resample_step_)));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".orientation_mode",
    rclcpp::ParameterValue(
      std::string(srm27_nav_common::orientationModeName(orientation_mode_))));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".debug_timing", rclcpp::ParameterValue(debug_timing_));

  occupancy_threshold_ =
    static_cast<int>(node->get_parameter(plugin_name_ + ".occupancy_threshold").as_int());
  treat_unknown_as_obstacle_ =
    node->get_parameter(plugin_name_ + ".treat_unknown_as_obstacle").as_bool();
  mask_num_layers_ =
    node->get_parameter(plugin_name_ + ".mask_num_layers").as_int();
  cost_inflate_radius_ = static_cast<float>(
    node->get_parameter(plugin_name_ + ".cost_inflate_radius").as_double());
  simplify_epsilon_ = static_cast<float>(
    node->get_parameter(plugin_name_ + ".simplify_epsilon").as_double());
  save_point_max_radius_ = static_cast<float>(
    node->get_parameter(plugin_name_ + ".save_point_max_radius").as_double());
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".goal_snap_max_radius",
    rclcpp::ParameterValue(static_cast<double>(save_point_max_radius_)));
  goal_snap_max_radius_ = static_cast<float>(
    node->get_parameter(plugin_name_ + ".goal_snap_max_radius").as_double());
  path_resample_step_ = static_cast<float>(
    node->get_parameter(plugin_name_ + ".path_resample_step").as_double());
  orientation_mode_ = srm27_nav_common::parseOrientationMode(
    node->get_parameter(plugin_name_ + ".orientation_mode").as_string());
  debug_timing_ = node->get_parameter(plugin_name_ + ".debug_timing").as_bool();

  srm27_map_adapter::CostmapConfig costmap_config;
  costmap_config.occupied_cost_threshold = occupancy_threshold_;
  std::string costmap_reason;
  if (!costmap_adapter_.configure(costmap_ros_, costmap_config, &costmap_reason)) {
    throw nav2_core::PlannerException(
      "YAstarGlobalPlanner: costmap adapter: " + costmap_reason);
  }

  if (mask_num_layers_ < 1) {
    throw nav2_core::PlannerException("YAstarGlobalPlanner: mask_num_layers must be >= 1");
  }
  if (cost_inflate_radius_ <= 0.0f || simplify_epsilon_ <= 0.0f) {
    throw nav2_core::PlannerException(
      "YAstarGlobalPlanner: cost_inflate_radius / simplify_epsilon must be positive");
  }
  if (!std::isfinite(save_point_max_radius_) || save_point_max_radius_ < 0.0f ||
    !std::isfinite(goal_snap_max_radius_) || goal_snap_max_radius_ < 0.0f ||
    !std::isfinite(path_resample_step_) || path_resample_step_ < 0.0f)
  {
    throw nav2_core::PlannerException(
      "YAstarGlobalPlanner: save_point_max_radius, goal_snap_max_radius and path_resample_step "
      "must be finite and nonnegative");
  }
}

void YAstarGlobalPlanner::cleanup()
{
  stable_goal_.reset();
  costmap_adapter_ = srm27_map_adapter::CostmapAdapter();
  costmap_ros_.reset();
  tf_.reset();
  node_.reset();
}

void YAstarGlobalPlanner::activate()
{
  active_ = true;
}

void YAstarGlobalPlanner::deactivate()
{
  active_ = false;
  stable_goal_.reset();
}

nav_msgs::msg::Path YAstarGlobalPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  if (!active_ || !costmap_ros_) {
    throw nav2_core::PlannerException("YAstarGlobalPlanner: plugin is not active");
  }
  auto node = node_.lock();
  if (!node) {
    throw nav2_core::PlannerException("YAstarGlobalPlanner: lifecycle node expired");
  }

  const auto t_begin = clock_->now();
  double setup_ms = 0.0, search_ms = 0.0;

  srm27_map_core::GridSnapshot snap;
  std::uint64_t map_version = 0;
  std::string snapshot_reason;
  if (!costmap_adapter_.snapshot(
      node->now().seconds(), snap, map_version, &snapshot_reason))
  {
    throw nav2_core::PlannerException(
      "YAstarGlobalPlanner: costmap snapshot failed: " + snapshot_reason);
  }

  // YAstar 6 参 setMap 要求 0 = 可行 / 1 = 阻挡；未知格是否算障碍由参数决定。
  std::vector<std::int8_t> mask = snap.buildBinaryMask(treat_unknown_as_obstacle_);

  YAstar planner;
  // 只能用 6 参重载：0 = 可行、非 0 = 占用，且同时设置 mapping / originPos。
  // 3 参 u_char* 重载语义相反（0 = 障碍），且不更新 resolution / 原点。
  planner.setMap(
    snap.sizeX(), snap.sizeY(), static_cast<float>(snap.resolution()),
    static_cast<float>(snap.originX()), static_cast<float>(snap.originY()), mask);
  // setMaskNumLayers 会按 occMap 尺寸重建掩码，必须在 setMap 之后。
  planner.setMaskNumLayers(mask_num_layers_);
  // 【2026-11-09 v2】代价场改「0.60 m 内重罚 + 远处零代价」。
  // v1（0.30 m）实测不足：inscribed 半径 0.33 m、膨胀 0.55 m，距墙 0.5 m 处
  // costmap 已是 253（INSCRIBED）。0.30 m 重罚放行了 0.3~0.55 m 的贴墙段，
  // smoother 的 SFC（threshold 252）判撞 → OSQP 六轮迭代必败 → BT 死循环。
  // 0.60 m 覆盖 inscribed+膨胀带，A* 路径不再进入 253 区。
  // 【2026-11-22 v3 实车】二值罚价在窄走廊（<1.2 m）里退化成"处处等价"：
  // 带内每格距两侧墙都 <0.60 m，penalty 全部 = 100，A* 失去居中倾向，
  // 实车 219_1121 北走廊路径贴墙（车身边缘蹭墙，人工目视确认）。
  // 改线性衰减（对齐上游 bench_yastar 的梯度思路）：
  //   0.40 m 内 100·(1−d/0.40)，走廊中轴（d≈缝宽/2）代价最低，向墙单调上升。
  //   0.56 m 缝：中轴 d=0.28 → 罚 30，贴边 d=0.16 → 罚 60，梯度 30 拉回中轴。
  //   宽阔区（d>0.40）罚 0，行为与 v2 一致。253/254 仍被 occupancy_threshold
  //   252 挡住，不受本改动影响。
  planner.setCostField(
    cost_inflate_radius_,
    [](float x) { return x < 0.40f ? 100.0f * (1.0f - x / 0.40f) : 0.0f; });
  planner.initCostMap();

  const auto resolved_start = nearestFreePoint(
    snap, Eigen::Vector2f(start.pose.position.x, start.pose.position.y), save_point_max_radius_,
    treat_unknown_as_obstacle_);
  if (!resolved_start) {
    throw nav2_core::PlannerException(
      "YAstarGlobalPlanner: no free start within configured tolerance");
  }
  const Eigen::Vector2f requested_goal(goal.pose.position.x, goal.pose.position.y);
  const auto resolved_goal = stable_goal_.resolve(
    snap, requested_goal, goal_snap_max_radius_, treat_unknown_as_obstacle_);
  if (!resolved_goal) {
    throw nav2_core::PlannerException(
      "YAstarGlobalPlanner: goal unavailable at (" + std::to_string(requested_goal.x()) + ", " +
      std::to_string(requested_goal.y()) + "), goal_snap_max_radius=" +
      std::to_string(goal_snap_max_radius_) + " m");
  }
  const Eigen::Vector2f s = *resolved_start;
  const Eigen::Vector2f g = *resolved_goal;
  setup_ms = (clock_->now() - t_begin).seconds() * 1000.0;
  const auto t_search = clock_->now();

  std::vector<Eigen::Vector2f> path = planner.search(s, g);
  search_ms = (clock_->now() - t_search).seconds() * 1000.0;

  if (debug_timing_) {
    RCLCPP_INFO(
      node->get_logger(),
      "YAstarGlobalPlanner 计时: 转换+初始化=%.2f 吸附+搜索=%.2f 总=%.2f ms",
      setup_ms, search_ms, (clock_->now() - t_begin).seconds() * 1000.0);
  }

  // Humble 的 createPlan 没有 cancel_checker 参数，取消由 planner_server 在
  // 动作层处理；这里不做主动取消检查。
  // 上游搜索失败与起终点同格都返回单点；先区分两者，再转换到 Nav2 栅格中心。
  path = nav2PathFromYastar(snap, path, s, g, treat_unknown_as_obstacle_);
  if (!pathIsFree(snap, path, treat_unknown_as_obstacle_)) {
    throw nav2_core::PlannerException(
      "YAstarGlobalPlanner: YAstar returned no collision-free route");
  }

  const auto unsimplified = path;
  path = planner.simplifyPath(path, simplify_epsilon_);
  path = planner.simplifyPathHypot(path);
  // Vendor simplifiers use rasterized line checks, which can miss a cell touched
  // at a corner. Keep the validated search route if a shortcut crosses a cell.
  if (!pathIsFree(snap, path, treat_unknown_as_obstacle_)) {
    path = unsimplified;
  }

  const auto dense = resamplePath(path, path_resample_step_, 10000);
  if (!pathIsFree(snap, dense, treat_unknown_as_obstacle_)) {
    throw nav2_core::PlannerException(
      "YAstarGlobalPlanner: resampled path crosses an occupied cell");
  }
  return toPath(
    dense, costmap_ros_->getGlobalFrameID(), node->now(), orientation_mode_);
}

}  // namespace srm27_nav_planner

// 必须在 namespace 之外（与 srm27_minco_controller.cpp:2015 的写法一致）。
PLUGINLIB_EXPORT_CLASS(srm27_nav_planner::YAstarGlobalPlanner, nav2_core::GlobalPlanner)
