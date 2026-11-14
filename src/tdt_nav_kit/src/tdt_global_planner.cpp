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
#include "tdt_nav_kit/path_utils.hpp"
#include "yastar.hpp"

using nav2_util::declare_parameter_if_not_declared;

namespace tdt_nav_kit
{

namespace
{

/// 只在配置的欧氏距离内吸附到最近的自由栅格中心。
Eigen::Vector2f snapToSafe(
  const CostmapSnapshot & map,
  const geometry_msgs::msg::Point & point,
  float max_radius,
  const char * endpoint)
{
  const Eigen::Vector2f world(
    static_cast<float>(point.x), static_cast<float>(point.y));
  const Eigen::Vector2f origin(map.origin_x, map.origin_y);
  const Eigen::Vector2f cell = (world - origin) / map.resolution;
  const auto is_free = [&map](int x, int y) {return map.data[y * map.size_x + x] == 0;};
  if (cell.x() >= 0.0f && cell.x() < map.size_x &&
    cell.y() >= 0.0f && cell.y() < map.size_y &&
    is_free(static_cast<int>(cell.x()), static_cast<int>(cell.y())))
  {
    return world;
  }

  const float radius_cells = max_radius / map.resolution;
  const auto first_cell = [radius_cells](float coordinate, int size) {
      return static_cast<int>(std::clamp(coordinate - radius_cells, 0.0f, float(size)));
    };
  const auto last_cell = [radius_cells](float coordinate, int size) {
      return static_cast<int>(std::clamp(coordinate + radius_cells, -1.0f, float(size - 1)));
    };
  Eigen::Vector2f snapped = world;
  float best_distance = max_radius;
  bool found = false;
  if (cell.allFinite()) {
    const int x0 = first_cell(cell.x(), map.size_x);
    const int x1 = last_cell(cell.x(), map.size_x);
    const int y0 = first_cell(cell.y(), map.size_y);
    const int y1 = last_cell(cell.y(), map.size_y);
    for (int y = y0; y <= y1; ++y) {
      for (int x = x0; x <= x1; ++x) {
        if (!is_free(x, y)) {
          continue;
        }
        const Eigen::Vector2f center = origin + map.resolution * Eigen::Vector2f(x + 0.5f, y + 0.5f);
        const float distance = (center - world).norm();
        if (distance <= best_distance) {
          best_distance = distance;
          snapped = center;
          found = true;
        }
      }
    }
  }
  if (found) {
    return snapped;
  }
  throw nav2_core::PlannerException(
    std::string("TdtGlobalPlanner: no free cell within tolerance of ") + endpoint);
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
    node, plugin_name_ + ".path_resample_step",
    rclcpp::ParameterValue(static_cast<double>(path_resample_step_)));
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
  path_resample_step_ = static_cast<float>(
    node->get_parameter(plugin_name_ + ".path_resample_step").as_double());
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
  if (!std::isfinite(save_point_max_radius_) || save_point_max_radius_ < 0.0f ||
    !std::isfinite(path_resample_step_) || path_resample_step_ < 0.0f)
  {
    throw nav2_core::PlannerException(
      "TdtGlobalPlanner: save_point_max_radius / path_resample_step must be finite and nonnegative");
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
  // 【2026-11-09 v2】代价场改「0.60 m 内重罚 + 远处零代价」。
  // v1（0.30 m）实测不足：inscribed 半径 0.33 m、膨胀 0.55 m，距墙 0.5 m 处
  // costmap 已是 253（INSCRIBED）。0.30 m 重罚放行了 0.3~0.55 m 的贴墙段，
  // smoother 的 SFC（threshold 252）判撞 → OSQP 六轮迭代必败 → BT 死循环。
  // 0.60 m 覆盖 inscribed+膨胀带，A* 路径不再进入 253 区。
  planner.setCostField(
    cost_inflate_radius_, [](float x) { return x < 0.60f ? 100.0f : 0.0f; });
  planner.initCostMap();

  const Eigen::Vector2f s =
    snapToSafe(snap, start.pose.position, save_point_max_radius_, "start");
  const Eigen::Vector2f g =
    snapToSafe(snap, goal.pose.position, save_point_max_radius_, "goal");
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
    resamplePath(path, path_resample_step_, 10000),
    costmap_ros_->getGlobalFrameID(), node->now(), orientation_mode_);
}

}  // namespace tdt_nav_kit

// 必须在 namespace 之外（与 srm27_minco_controller.cpp:2015 的写法一致）。
PLUGINLIB_EXPORT_CLASS(tdt_nav_kit::TdtGlobalPlanner, nav2_core::GlobalPlanner)
