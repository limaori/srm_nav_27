// Copyright 2026 SRM27 Navigation Team
// Licensed under the Apache License, Version 2.0.

#include "tdt_nav_kit/tdt_global_planner.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_set>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tdt_nav_kit/path_utils.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2/utils.h"
#include "yastar.hpp"

using nav2_util::declare_parameter_if_not_declared;

namespace tdt_nav_kit
{

namespace
{

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
  stable_goal_.reset();

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
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".goal_snap_max_radius",
    rclcpp::ParameterValue(static_cast<double>(save_point_max_radius_)));
  goal_snap_max_radius_ = static_cast<float>(
    node->get_parameter(plugin_name_ + ".goal_snap_max_radius").as_double());
  path_resample_step_ = static_cast<float>(
    node->get_parameter(plugin_name_ + ".path_resample_step").as_double());
  orientation_mode_ =
    node->get_parameter(plugin_name_ + ".orientation_mode").as_string();
  debug_timing_ = node->get_parameter(plugin_name_ + ".debug_timing").as_bool();

  // —— 限时动态掩码参数（2026-11-23，复现上游 setMaskMapTimed 语义）——
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".dynamic_mask_topic",
    rclcpp::ParameterValue(std::string("")));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".dynamic_mask_duration",
    rclcpp::ParameterValue(static_cast<double>(dynamic_mask_duration_)));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".dynamic_mask_dilation",
    rclcpp::ParameterValue(static_cast<double>(dynamic_mask_dilation_)));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".dynamic_mask_min_z",
    rclcpp::ParameterValue(static_cast<double>(dynamic_mask_min_z_)));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".dynamic_mask_max_z",
    rclcpp::ParameterValue(static_cast<double>(dynamic_mask_max_z_)));
  dynamic_mask_topic_ =
    node->get_parameter(plugin_name_ + ".dynamic_mask_topic").as_string();
  dynamic_mask_duration_ =
    node->get_parameter(plugin_name_ + ".dynamic_mask_duration").as_double();
  dynamic_mask_dilation_ =
    node->get_parameter(plugin_name_ + ".dynamic_mask_dilation").as_double();
  dynamic_mask_min_z_ =
    node->get_parameter(plugin_name_ + ".dynamic_mask_min_z").as_double();
  dynamic_mask_max_z_ =
    node->get_parameter(plugin_name_ + ".dynamic_mask_max_z").as_double();

  if (mask_num_layers_ < 1) {
    throw nav2_core::PlannerException("TdtGlobalPlanner: mask_num_layers must be >= 1");
  }
  if (cost_inflate_radius_ <= 0.0f || simplify_epsilon_ <= 0.0f) {
    throw nav2_core::PlannerException(
      "TdtGlobalPlanner: cost_inflate_radius / simplify_epsilon must be positive");
  }
  if (!std::isfinite(save_point_max_radius_) || save_point_max_radius_ < 0.0f ||
    !std::isfinite(goal_snap_max_radius_) || goal_snap_max_radius_ < 0.0f ||
    !std::isfinite(path_resample_step_) || path_resample_step_ < 0.0f)
  {
    throw nav2_core::PlannerException(
      "TdtGlobalPlanner: save_point_max_radius, goal_snap_max_radius and path_resample_step "
      "must be finite and nonnegative");
  }
  if (dynamic_mask_duration_ <= 0.0 || dynamic_mask_dilation_ < 0.0 ||
    dynamic_mask_min_z_ >= dynamic_mask_max_z_)
  {
    throw nav2_core::PlannerException(
      "TdtGlobalPlanner: dynamic_mask_duration must be positive, dilation nonnegative, "
      "and min_z < max_z");
  }
  if (!dynamic_mask_topic_.empty()) {
    // 相对话题名跟随 planner_server 的命名空间（与 costmap 的
    // <robot_namespace>/terrain_map_ext 同一套约定）。
    dynamic_cloud_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
      dynamic_mask_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
      std::bind(&TdtGlobalPlanner::dynamicCloudCallback, this, std::placeholders::_1));
    RCLCPP_INFO(
      node->get_logger(),
      "TdtGlobalPlanner: 限时动态掩码已启用 (topic=%s, duration=%.2fs, dilation=%.2fm, "
      "z=[%.2f,%.2f])",
      dynamic_mask_topic_.c_str(), dynamic_mask_duration_, dynamic_mask_dilation_,
      dynamic_mask_min_z_, dynamic_mask_max_z_);
  }
}

void TdtGlobalPlanner::cleanup()
{
  stable_goal_.reset();
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
  stable_goal_.reset();
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
  // 限时动态掩码必须在 initCostMap 之前推送：SDF / 代价场由 getMap() 计算，
  // getMap 包含 mask（isInObstacle = mask || occMap），晚于 initCostMap 推送
  // 只影响搜索、不影响代价梯度。
  applyDynamicMask(
    planner, snap.size_x, snap.size_y, snap.resolution, snap.origin_x, snap.origin_y);
  planner.initCostMap();

  const auto resolved_start = nearestFreePoint(
    snap, Eigen::Vector2f(start.pose.position.x, start.pose.position.y), save_point_max_radius_);
  if (!resolved_start) {
    throw nav2_core::PlannerException("TdtGlobalPlanner: no free start within configured tolerance");
  }
  const Eigen::Vector2f requested_goal(goal.pose.position.x, goal.pose.position.y);
  const auto resolved_goal = stable_goal_.resolve(snap, requested_goal, goal_snap_max_radius_);
  if (!resolved_goal) {
    throw nav2_core::PlannerException(
      "TdtGlobalPlanner: goal unavailable at (" + std::to_string(requested_goal.x()) + ", " +
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
      "TdtGlobalPlanner 计时: 转换+初始化=%.2f 吸附+搜索=%.2f 总=%.2f ms",
      setup_ms, search_ms, (clock_->now() - t_begin).seconds() * 1000.0);
  }

  // Humble 的 createPlan 没有 cancel_checker 参数，取消由 planner_server 在
  // 动作层处理；这里不做主动取消检查。
  // 上游搜索失败与起终点同格都返回单点；先区分两者，再转换到 Nav2 栅格中心。
  path = nav2PathFromYastar(snap, path, s, g);
  if (!pathIsFree(snap, path)) {
    throw nav2_core::PlannerException(
      "TdtGlobalPlanner: YAstar returned no collision-free route");
  }

  const auto unsimplified = path;
  path = planner.simplifyPath(path, simplify_epsilon_);
  path = planner.simplifyPathHypot(path);
  // Vendor simplifiers use rasterized line checks, which can miss a cell touched
  // at a corner. Keep the validated search route if a shortcut crosses a cell.
  if (!pathIsFree(snap, path)) {
    path = unsimplified;
  }

  const auto dense = resamplePath(path, path_resample_step_, 10000);
  if (!pathIsFree(snap, dense)) {
    throw nav2_core::PlannerException("TdtGlobalPlanner: resampled path crosses an occupied cell");
  }
  return toPath(
    dense,
    costmap_ros_->getGlobalFrameID(), node->now(), orientation_mode_);
}

void TdtGlobalPlanner::dynamicCloudCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // 回调跑在 planner_server 的 executor 线程上，任何未捕获异常都会
  // std::terminate 整个组件容器（实车 SIGABRT 教训），这里整体兜底。
  try {
    dynamicCloudCallbackImpl(msg);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("TdtGlobalPlanner.dynamic_mask"),
      "动态掩码回调异常: %s", e.what());
  } catch (...) {
    RCLCPP_ERROR(
      rclcpp::get_logger("TdtGlobalPlanner.dynamic_mask"),
      "动态掩码回调未知异常");
  }
}

void TdtGlobalPlanner::dynamicCloudCallbackImpl(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  auto node = node_.lock();
  if (!node || !tf_ || !costmap_ros_) {
    return;
  }

  // 点云 → 全局系（terrain_map_ext 发布在 odom 系）。用最新可用变换，
  // 查不到就节流告警后丢弃本帧（窗口内后续帧会补上）。
  const std::string global_frame = costmap_ros_->getGlobalFrameID();
  geometry_msgs::msg::TransformStamped tf_msg;
  try {
    tf_msg = tf_->lookupTransform(global_frame, msg->header.frame_id, tf2::TimePointZero);
  } catch (const tf2::TransformException & e) {
    const double now_sec = node->now().seconds();
    if (now_sec - last_tf_warn_sec_ > 5.0) {
      last_tf_warn_sec_ = now_sec;
      RCLCPP_WARN(
        node->get_logger(), "TdtGlobalPlanner: 动态掩码 TF 失败 (%s → %s): %s",
        msg->header.frame_id.c_str(), global_frame.c_str(), e.what());
    }
    return;
  }
  const double tx = tf_msg.transform.translation.x;
  const double ty = tf_msg.transform.translation.y;
  const double yaw = tf2::getYaw(tf_msg.transform.rotation);
  const double cos_yaw = std::cos(yaw), sin_yaw = std::sin(yaw);

  // 全局代价地图几何（global_costmap 尺寸固定，直接读当前值即可）。
  const auto * costmap = costmap_ros_->getCostmap();
  const int size_x = static_cast<int>(costmap->getSizeInCellsX());
  const int size_y = static_cast<int>(costmap->getSizeInCellsY());
  const double res = costmap->getResolution();
  const double ox = costmap->getOriginX();
  const double oy = costmap->getOriginY();
  if (size_x <= 0 || size_y <= 0) {
    return;
  }

  // 栅格化（含 dilation 膨胀）。z 过滤带与代价地图高度窗一致。
  const int radius = static_cast<int>(std::floor(dynamic_mask_dilation_ / res));
  std::unordered_set<int> stamped;
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    const float z = *iter_z;
    if (z < dynamic_mask_min_z_ || z > dynamic_mask_max_z_) {
      continue;
    }
    const double wx = tx + cos_yaw * static_cast<double>(*iter_x) -
      sin_yaw * static_cast<double>(*iter_y);
    const double wy = ty + sin_yaw * static_cast<double>(*iter_x) +
      cos_yaw * static_cast<double>(*iter_y);
    const int ci = static_cast<int>((wx - ox) / res);
    const int cj = static_cast<int>((wy - oy) / res);
    if (ci < -radius || ci >= size_x + radius || cj < -radius || cj >= size_y + radius) {
      continue;
    }
    for (int dj = -radius; dj <= radius; ++dj) {
      for (int di = -radius; di <= radius; ++di) {
        if (di * di + dj * dj > radius * radius) {
          continue;
        }
        const int i = ci + di, j = cj + dj;
        if (i < 0 || i >= size_x || j < 0 || j >= size_y) {
          continue;
        }
        stamped.insert(j * size_x + i);
      }
    }
  }

  const rclcpp::Time now = node->now();
  std::lock_guard<std::mutex> lock(dynamic_mask_mutex_);
  // 过期清理（时间窗语义的载体）与容量兜底。
  while (!dynamic_cells_.empty() && (now - dynamic_cells_.front().first).seconds() >
         dynamic_mask_duration_)
  {
    dynamic_cells_.pop_front();
  }
  if (stamped.empty()) {
    return;
  }
  std::vector<std::pair<int, int>> cell_vec;
  cell_vec.reserve(stamped.size());
  for (int idx : stamped) {
    cell_vec.emplace_back(idx / size_x, idx % size_x);
  }
  dynamic_cells_.emplace_back(now, std::move(cell_vec));
  // 防御：正常 10Hz 点云 + 0.8s 窗口 = 8 条，超过 32 条说明回调堆积，丢最老的。
  while (dynamic_cells_.size() > 32) {
    dynamic_cells_.pop_front();
  }
}

void TdtGlobalPlanner::applyDynamicMask(
  YAstar & planner, int size_x, int size_y, float resolution,
  float origin_x, float origin_y)
{
  auto node = node_.lock();
  if (!node) {
    return;
  }
  const rclcpp::Time now = node->now();
  std::vector<std::pair<int, int>> cells;
  {
    std::lock_guard<std::mutex> lock(dynamic_mask_mutex_);
    while (!dynamic_cells_.empty() && (now - dynamic_cells_.front().first).seconds() >
           dynamic_mask_duration_)
    {
      dynamic_cells_.pop_front();
    }
    for (const auto & entry : dynamic_cells_) {
      cells.insert(cells.end(), entry.second.begin(), entry.second.end());
    }
  }
  if (cells.empty()) {
    return;
  }

  // 与 occMap 同形状：rows = height(y)，cols = width(x)；maskValue=1 的格 = 障碍。
  YAstar::TMatrix<u_char> mask = YAstar::TMatrix<u_char>::Zero(size_y, size_x);
  int painted = 0;
  for (const auto & rc : cells) {
    const int j = rc.first, i = rc.second;
    if (i < 0 || i >= size_x || j < 0 || j >= size_y) {
      continue;
    }
    mask(j, i) = 1;
    ++painted;
  }
  if (painted == 0) {
    return;
  }
  // createPlan 每次 new 局部 YAstar，setMaskMap（非限时）的生命周期 = 本次搜索，
  // "duration 记忆"由 dynamic_cells_ 的时间窗承担（见头文件说明）。
  planner.setMaskMap(mask, "dynamic", /*maskValue=*/1);
  RCLCPP_INFO_THROTTLE(
    node->get_logger(), *clock_, 5000,
    "TdtGlobalPlanner: 动态掩码生效 %d 格 (窗口 %.1fs, %zu 帧缓存)",
    painted, dynamic_mask_duration_, dynamic_cells_.size());
}

}  // namespace tdt_nav_kit

// 必须在 namespace 之外（与 srm27_minco_controller.cpp:2015 的写法一致）。
PLUGINLIB_EXPORT_CLASS(tdt_nav_kit::TdtGlobalPlanner, nav2_core::GlobalPlanner)
