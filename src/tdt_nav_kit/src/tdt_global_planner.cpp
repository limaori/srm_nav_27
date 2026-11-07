// Copyright 2026 SRM27 Navigation Team
// Licensed under the Apache License, Version 2.0.

#include "tdt_nav_kit/tdt_global_planner.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "yastar.hpp"

using nav2_util::declare_parameter_if_not_declared;

namespace tdt_nav_kit
{

namespace
{

/// 起/终点落在障碍或地图外时才吸附到安全栅格；已在自由格则原样返回，
/// 避免无谓移动合法的导航目标。map_img: 0 = 障碍，255 = 可行（YAstar::getMap()）。
Eigen::Vector2f snapToSafe(
  YAstar & planner,
  const YAstar::TMatrix<u_char> & map_img,
  const geometry_msgs::msg::Point & point,
  float max_radius)
{
  const Eigen::Vector2f world(
    static_cast<float>(point.x), static_cast<float>(point.y));
  const Eigen::Vector2i cell = planner.transformPos(world);

  const bool safe =
    cell.x() >= 0 && cell.y() >= 0 &&
    cell.x() < static_cast<int>(map_img.cols()) &&
    cell.y() < static_cast<int>(map_img.rows()) &&
    map_img(cell.y(), cell.x()) != 0;

  if (safe) {
    return world;
  }
  return planner.findSavePoint(world.x(), world.y(), max_radius);
}

/// 上游路径点只有 xy，无时间戳 / 朝向 / frame_id，这里补齐 Path 所需字段。
nav_msgs::msg::Path toPath(
  const std::vector<Eigen::Vector2f> & points,
  const std::string & frame_id,
  const rclcpp::Time & stamp,
  const std::string & orientation_mode)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = frame_id;
  path.header.stamp = stamp;
  path.poses.reserve(points.size());

  for (std::size_t i = 0; i < points.size(); ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = points[i].x();
    pose.pose.position.y = points[i].y();
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;

    if (orientation_mode == "tangent") {
      const std::size_t prev = (i > 0) ? i - 1 : i;
      const std::size_t next = (i + 1 < points.size()) ? i + 1 : i;
      const double dx = static_cast<double>(points[next].x()) - points[prev].x();
      const double dy = static_cast<double>(points[next].y()) - points[prev].y();
      if (std::hypot(dx, dy) > 1e-9) {
        const double half_yaw = 0.5 * std::atan2(dy, dx);
        pose.pose.orientation.w = std::cos(half_yaw);
        pose.pose.orientation.z = std::sin(half_yaw);
      }
    }
    path.poses.push_back(pose);
  }
  return path;
}

}  // namespace

void TdtGlobalPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  if (!node) {
    throw nav2_core::PlannerException("TdtGlobalPlanner: unable to lock lifecycle node");
  }
  if (!costmap_ros) {
    throw nav2_core::PlannerException("TdtGlobalPlanner: costmap_ros is null");
  }

  node_ = parent;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  plugin_name_ = name;
  clock_ = node->get_clock();

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".occupancy_threshold",
    rclcpp::ParameterValue(converter_cfg_.occupancy_threshold));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".treat_unknown_as_obstacle",
    rclcpp::ParameterValue(converter_cfg_.treat_unknown_as_obstacle));
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
    node, plugin_name_ + ".orientation_mode", rclcpp::ParameterValue(orientation_mode_));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".debug_timing", rclcpp::ParameterValue(debug_timing_));

  converter_cfg_.occupancy_threshold =
    node->get_parameter(plugin_name_ + ".occupancy_threshold").as_int();
  converter_cfg_.treat_unknown_as_obstacle =
    node->get_parameter(plugin_name_ + ".treat_unknown_as_obstacle").as_bool();
  mask_num_layers_ =
    node->get_parameter(plugin_name_ + ".mask_num_layers").as_int();
  cost_inflate_radius_ = static_cast<float>(
    node->get_parameter(plugin_name_ + ".cost_inflate_radius").as_double());
  simplify_epsilon_ = static_cast<float>(
    node->get_parameter(plugin_name_ + ".simplify_epsilon").as_double());
  save_point_max_radius_ = static_cast<float>(
    node->get_parameter(plugin_name_ + ".save_point_max_radius").as_double());
  orientation_mode_ =
    node->get_parameter(plugin_name_ + ".orientation_mode").as_string();
  debug_timing_ = node->get_parameter(plugin_name_ + ".debug_timing").as_bool();

  if (mask_num_layers_ < 1) {
    throw nav2_core::PlannerException("TdtGlobalPlanner: mask_num_layers must be >= 1");
  }
  if (cost_inflate_radius_ <= 0.0f || simplify_epsilon_ <= 0.0f) {
    throw nav2_core::PlannerException(
      "TdtGlobalPlanner: cost_inflate_radius / simplify_epsilon must be positive");
  }
}

void TdtGlobalPlanner::cleanup()
{
  costmap_ros_.reset();
  tf_.reset();
  node_.reset();
}

void TdtGlobalPlanner::activate()
{
  active_ = true;
}

void TdtGlobalPlanner::deactivate()
{
  active_ = false;
}

nav_msgs::msg::Path TdtGlobalPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  if (!active_ || !costmap_ros_) {
    throw nav2_core::PlannerException("TdtGlobalPlanner: plugin is not active");
  }
  auto node = node_.lock();
  if (!node) {
    throw nav2_core::PlannerException("TdtGlobalPlanner: lifecycle node expired");
  }

  const auto t_begin = clock_->now();
  double setup_ms = 0.0, search_ms = 0.0;

  // 6 参 setMap 的签名要求非常量 std::vector<int8_t>&，故 snap 不能是 const。
  CostmapSnapshot snap = makeSnapshot(costmap_ros_.get(), converter_cfg_);

  YAstar planner;
  // 只能用 6 参重载：0 = 可行、非 0 = 占用，且同时设置 mapping / originPos。
  // 3 参 u_char* 重载语义相反（0 = 障碍），且不更新 resolution / 原点。
  planner.setMap(
    snap.size_x, snap.size_y, snap.resolution, snap.origin_x, snap.origin_y,
    snap.data);
  // setMaskNumLayers 会按 occMap 尺寸重建掩码，必须在 setMap 之后。
  planner.setMaskNumLayers(mask_num_layers_);
  planner.setCostField(
    cost_inflate_radius_, [](float x) { return 0.5f / (0.1f + x); });
  planner.initCostMap();

  const YAstar::TMatrix<u_char> map_img = planner.getMap();
  const Eigen::Vector2f s =
    snapToSafe(planner, map_img, start.pose.position, save_point_max_radius_);
  const Eigen::Vector2f g =
    snapToSafe(planner, map_img, goal.pose.position, save_point_max_radius_);
  setup_ms = (clock_->now() - t_begin).seconds() * 1000.0;
  const auto t_search = clock_->now();

  std::vector<Eigen::Vector2f> path = planner.search(s, g);
  search_ms = (clock_->now() - t_search).seconds() * 1000.0;

  if (debug_timing_) {
    RCLCPP_INFO(
      node->get_logger(),
      "TdtGlobalPlanner 计时: 转换+初始化=%.2f 吸附+搜索=%.2f 总=%.2f ms",
      setup_ms, search_ms, (clock_->now() - t_begin).seconds() * 1000.0);
  }

  // Humble 的 createPlan 没有 cancel_checker 参数，取消由 planner_server 在
  // 动作层处理；这里不做主动取消检查。
  // 上游 search() 失败时返回「仅含 start 的单点路径」，而不是空数组。
  if (path.size() < 2) {
    throw nav2_core::PlannerException(
      "TdtGlobalPlanner: YAstar returned a start-only path (no feasible route)");
  }

  path = planner.simplifyPath(path, simplify_epsilon_);
  path = planner.simplifyPathHypot(path);
  if (path.size() < 2) {
    throw nav2_core::PlannerException("TdtGlobalPlanner: path collapsed during simplification");
  }

  return toPath(
    path, costmap_ros_->getGlobalFrameID(), node->now(), orientation_mode_);
}

}  // namespace tdt_nav_kit

// 必须在 namespace 之外（与 srm27_minco_controller.cpp:2015 的写法一致）。
PLUGINLIB_EXPORT_CLASS(tdt_nav_kit::TdtGlobalPlanner, nav2_core::GlobalPlanner)
