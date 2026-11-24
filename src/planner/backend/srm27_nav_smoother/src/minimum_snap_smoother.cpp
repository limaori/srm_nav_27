// Copyright 2026 SRM27 Navigation Team
// Licensed under the Apache License, Version 2.0.
//
// 算法来自东北大学 T-DT 的 tdt-nav-kit（MIT），vendored 源码在本包
// vendor/src/MinimumSnapOsqp/ 内，见本包 THIRD_PARTY_NOTICES.md 与 PATCHES.md。

#include "srm27_nav_smoother/minimum_snap_smoother.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <exception>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include "minimumSnap.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "srm27_map_adapter/costmap_adapter.hpp"
#include "srm27_map_core/grid_snapshot.hpp"
#include "srm27_map_core/sfc_grid.hpp"
#include "srm27_nav_common/path_utils.hpp"

using nav2_util::declare_parameter_if_not_declared;
using srm27_map_adapter::makeGridSnapshot;
using srm27_map_core::GridSnapshot;
using srm27_map_core::makeSfcGrid;
using srm27_map_core::SfcGrid;
using srm27_nav_common::pointsFromPath;
using srm27_nav_common::writePath;

namespace srm27_nav_smoother
{
namespace
{

/// max_corridor_range 的安全上限。上游 _getBound 把 maxRange/mapping 转 int，
/// 必须保证结果落在 int 范围内（1e6 / 0.05 = 2e7）。
constexpr float kMaxCorridorRangeLimit = 1.0e6f;

MinimumSnap::Backend toBackend(const std::string & name)
{
  if (name == "path") {
    return MinimumSnap::Backend::OSQPPath;
  }
  if (name == "close") {
    return MinimumSnap::Backend::Close;
  }
  return MinimumSnap::Backend::OSQPCorridor;
}

/// 把栅格拷进 SfcSquare 的 Eigen::Map（行主序，rows=y, cols=x）。
/// 上游 Map 是 `Eigen::Matrix<unsigned char, Dynamic, Dynamic, RowMajor>`
/// （sfcSquare.hpp:34），线性索引同为 y * cols + x，顺序拷贝即可，不做 Y 翻转。
MinimumSnap::Map toEigenMap(const SfcGrid & grid)
{
  MinimumSnap::Map map(grid.size_y, grid.size_x);
  std::memcpy(
    map.data(), grid.data.data(),
    grid.data.size() * sizeof(unsigned char));
  return map;
}

std::string frameIdOf(const nav_msgs::msg::Path & path)
{
  return path.header.frame_id.empty() ? "map" : path.header.frame_id;
}

/// 只统计 RViz 真的会画的方块（有限值且有面积），与 makeSfcMarkerArray 的过滤一致。
struct BoxSummary
{
  std::size_t count{0};
  float mean_w{0.0f};
  float mean_h{0.0f};
};

BoxSummary summarizeBoxes(const std::vector<SfcBox> & boxes)
{
  BoxSummary summary;
  double sum_w = 0.0;
  double sum_h = 0.0;
  for (const auto & box : boxes) {
    if (!std::all_of(
        box.begin(), box.end(), [](float value) {return std::isfinite(value);}) ||
      box[2] <= box[0] || box[3] <= box[1])
    {
      continue;
    }
    ++summary.count;
    sum_w += static_cast<double>(box[2] - box[0]);
    sum_h += static_cast<double>(box[3] - box[1]);
  }
  if (summary.count > 0) {
    summary.mean_w = static_cast<float>(sum_w / static_cast<double>(summary.count));
    summary.mean_h = static_cast<float>(sum_h / static_cast<double>(summary.count));
  }
  return summary;
}

std::string formatBoxSummary(const BoxSummary & summary)
{
  if (summary.count == 0) {
    return "none";
  }
  std::ostringstream text;
  text << std::fixed << std::setprecision(2);
  text << summary.count << (summary.count == 1 ? " box " : " boxes ") <<
    summary.mean_w << "x" << summary.mean_h << " m";
  return text.str();
}

/// 白字标签：放在路径中点上方，避免路径靠近画面边缘时被裁掉。
/// 路径为空时返回 DELETE，避免上一次的文字残留在 RViz 里。
visualization_msgs::msg::Marker makeLegendMarker(
  const nav_msgs::msg::Path & path,
  const std::string & frame_id,
  const rclcpp::Time & stamp,
  const std::string & text,
  int id,
  float z)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = stamp;
  marker.ns = "sfc_legend";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;

  if (path.poses.empty()) {
    marker.action = visualization_msgs::msg::Marker::DELETE;
    return marker;
  }

  const auto & mid = path.poses[path.poses.size() / 2].pose.position;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = mid.x;
  marker.pose.position.y = mid.y;
  marker.pose.position.z = z;
  marker.pose.orientation.w = 1.0;
  marker.scale.z = 0.09f;
  marker.color.r = 1.0f;
  marker.color.g = 1.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0f;
  marker.text = text;
  return marker;
}

}  // namespace

void MinimumSnapSmoother::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub,
  std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub)
{
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("MinimumSnapSmoother: unable to lock lifecycle node");
  }
  if (!costmap_sub) {
    throw std::runtime_error("MinimumSnapSmoother: costmap_sub is null");
  }

  node_ = parent;
  tf_ = tf;
  costmap_sub_ = costmap_sub;
  footprint_sub_ = footprint_sub;
  plugin_name_ = name;
  clock_ = node->get_clock();

  const std::string p = plugin_name_ + ".";
  declare_parameter_if_not_declared(
    node, p + "occupancy_threshold", rclcpp::ParameterValue(costmap_config_.occupied_cost_threshold));
  declare_parameter_if_not_declared(
    node, p + "treat_unknown_as_obstacle",
    rclcpp::ParameterValue(treat_unknown_as_obstacle_));
  declare_parameter_if_not_declared(node, p + "order", rclcpp::ParameterValue(order_));
  declare_parameter_if_not_declared(node, p + "max_dx", rclcpp::ParameterValue(max_dx_));
  declare_parameter_if_not_declared(
    node, p + "strict_collision", rclcpp::ParameterValue(strict_collision_));
  declare_parameter_if_not_declared(node, p + "backend", rclcpp::ParameterValue(backend_));
  declare_parameter_if_not_declared(
    node, p + "collision_iter", rclcpp::ParameterValue(collision_iter_));
  declare_parameter_if_not_declared(node, p + "norm_time", rclcpp::ParameterValue(norm_time_));
  declare_parameter_if_not_declared(
    node, p + "max_corridor_range", rclcpp::ParameterValue(max_corridor_range_));
  declare_parameter_if_not_declared(
    node, p + "corridor_shrink", rclcpp::ParameterValue(corridor_shrink_));
  declare_parameter_if_not_declared(node, p + "max_speed", rclcpp::ParameterValue(max_speed_));
  declare_parameter_if_not_declared(node, p + "max_acc", rclcpp::ParameterValue(max_acc_));
  declare_parameter_if_not_declared(
    node, p + "orientation_mode",
    rclcpp::ParameterValue(
      std::string(srm27_nav_common::orientationModeName(orientation_mode_))));
  declare_parameter_if_not_declared(node, p + "publish_sfc", rclcpp::ParameterValue(publish_sfc_));
  declare_parameter_if_not_declared(node, p + "sfc_topic", rclcpp::ParameterValue(sfc_topic_));
  declare_parameter_if_not_declared(
    node, p + "sfc_line_width", rclcpp::ParameterValue(static_cast<double>(sfc_line_width_)));

  costmap_config_.occupied_cost_threshold =
    static_cast<int>(node->get_parameter(p + "occupancy_threshold").as_int());
  treat_unknown_as_obstacle_ =
    node->get_parameter(p + "treat_unknown_as_obstacle").as_bool();
  order_ = static_cast<int>(node->get_parameter(p + "order").as_int());
  max_dx_ = static_cast<int>(node->get_parameter(p + "max_dx").as_int());
  strict_collision_ = node->get_parameter(p + "strict_collision").as_bool();
  backend_ = node->get_parameter(p + "backend").as_string();
  collision_iter_ = static_cast<int>(node->get_parameter(p + "collision_iter").as_int());
  norm_time_ = node->get_parameter(p + "norm_time").as_bool();
  max_corridor_range_ =
    static_cast<float>(node->get_parameter(p + "max_corridor_range").as_double());
  corridor_shrink_ = static_cast<float>(node->get_parameter(p + "corridor_shrink").as_double());
  max_speed_ = static_cast<float>(node->get_parameter(p + "max_speed").as_double());
  max_acc_ = static_cast<float>(node->get_parameter(p + "max_acc").as_double());

  orientation_mode_ = srm27_nav_common::parseOrientationMode(
    node->get_parameter(p + "orientation_mode").as_string());
  publish_sfc_ = node->get_parameter(p + "publish_sfc").as_bool();
  sfc_topic_ = node->get_parameter(p + "sfc_topic").as_string();
  sfc_line_width_ = static_cast<float>(node->get_parameter(p + "sfc_line_width").as_double());
  sfc_style_.line_width = sfc_line_width_;

  // SFC 走廊：transient_local + 固定 (ns, id)，RViz 晚打开也能看到最近一次的走廊，
  // 每次发布整体替换，方块数量变化不会留下残影。
  if (publish_sfc_) {
    sfc_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
      sfc_topic_, rclcpp::QoS(1).transient_local());
    RCLCPP_INFO(
      node->get_logger(),
      "MinimumSnapSmoother: SFC 方形走廊 -> %s（publish_sfc=false 可关闭）",
      sfc_pub_->get_topic_name());
  } else {
    sfc_pub_.reset();
  }

  RCLCPP_INFO(
    node->get_logger(),
    "MinimumSnapSmoother: 上游 main.cpp 参数 order=%d max_dx=%d backend=%s "
    "collision_iter=%d norm_time=%s corridor=[%.2f m, shrink %.2f m]",
    order_, max_dx_, backend_.c_str(), collision_iter_,
    norm_time_ ? "true" : "false",
    static_cast<double>(max_corridor_range_), static_cast<double>(corridor_shrink_));
}

void MinimumSnapSmoother::cleanup()
{
  sfc_pub_.reset();
  costmap_sub_.reset();
  footprint_sub_.reset();
  tf_.reset();
  active_ = false;
}

void MinimumSnapSmoother::activate()
{
  active_ = true;
}

void MinimumSnapSmoother::deactivate()
{
  active_ = false;
}

void MinimumSnapSmoother::publishSfcMarkers(
  const nav_msgs::msg::Path & path,
  const std::vector<SfcBox> & free_space_boxes,
  const std::vector<SfcBox> & final_boxes,
  bool solve_ok)
{
  if (!sfc_pub_) {
    return;
  }

  const std::string frame_id = frameIdOf(path);
  const auto stamp = clock_->now();

  // 黄色细线：getCorridor() 的初始自由方块（未碰撞收缩），代表"实际还剩多少空间"。
  SfcMarkerStyle free_style = sfc_style_;
  free_style.marker_namespace = "sfc_corridor";
  free_style.red = 1.0f;
  free_style.green = 0.90f;
  free_style.blue = 0.10f;
  free_style.alpha = 0.95f;

  // 橙色粗线：求解迭代后的最终约束，可能被 upstream shrink 成小方块/点。
  SfcMarkerStyle final_style = sfc_style_;
  final_style.marker_namespace = "sfc_corridor_final";
  final_style.red = 1.0f;
  final_style.green = 0.35f;
  final_style.blue = 0.0f;
  final_style.alpha = 0.95f;
  final_style.line_width = std::max(0.03f, sfc_line_width_ * 1.5f);

  auto array = makeSfcMarkerArray(free_space_boxes, frame_id, stamp, free_style);
  auto final_array = makeSfcMarkerArray(final_boxes, frame_id, stamp, final_style);
  array.markers.insert(
    array.markers.end(), final_array.markers.begin(), final_array.markers.end());
  const auto free_summary = summarizeBoxes(free_space_boxes);
  const auto final_summary = summarizeBoxes(final_boxes);
  const std::string free_text = "SFC free (yellow): " + formatBoxSummary(free_summary);
  const std::string final_text = std::string("SFC final (orange): ") +
    (solve_ok ? formatBoxSummary(final_summary) : "no result");
  array.markers.push_back(makeLegendMarker(path, frame_id, stamp, free_text, 0, 0.90f));
  array.markers.push_back(makeLegendMarker(path, frame_id, stamp, final_text, 1, 1.02f));

  sfc_pub_->publish(array);
}

bool MinimumSnapSmoother::smooth(
  nav_msgs::msg::Path & path,
  const rclcpp::Duration & max_time)
{
  // 上游 main.cpp:149 固定 setTL(10)；Nav2 的 max_smoothing_duration 不再截断求解。
  (void)max_time;
  if (!active_) {
    return false;
  }
  if (path.poses.size() < 2) {
    // 单点/空路径：无走廊可解，保持原路径并清掉上一次的方块。
    publishSfcMarkers(path, {}, {}, false);
    return true;
  }

  auto node = node_.lock();
  if (!node) {
    return false;
  }

  // 1) 取栅格快照。CostmapSubscriber 返回的是共享指针，
  //    但内容仍可能被 costmap 线程更新，故先深拷贝再算。
  auto costmap = costmap_sub_ ? costmap_sub_->getCostmap() : nullptr;
  if (!costmap) {
    RCLCPP_WARN_THROTTLE(
      node->get_logger(), *clock_, 2000,
      "MinimumSnapSmoother: costmap unavailable, keep original path");
    publishSfcMarkers(path, {}, {}, false);
    return false;
  }

  SfcGrid grid;
  try {
    GridSnapshot snapshot;
    std::string reason;
    if (!makeGridSnapshot(
        costmap.get(), costmap_config_, node->now().seconds(), snapshot, nullptr, &reason))
    {
      throw std::runtime_error(reason);
    }
    grid = makeSfcGrid(snapshot, treat_unknown_as_obstacle_);
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(
      node->get_logger(), *clock_, 2000,
      "MinimumSnapSmoother: costmap conversion failed (%s), keep original path", e.what());
    publishSfcMarkers(path, {}, {}, false);
    return false;
  }

  // 2) 上游对象。setMap 必须传 originx/originy：SfcSquare 用
  //    (x - originPos)/mapping 反算索引（P7 补丁已统一为减号）。
  MinimumSnap smoother;
  smoother.setMap(
    toEigenMap(grid), grid.resolution, grid.origin_x, grid.origin_y);
  smoother.setOrder(order_);
  smoother.setMaxDx(max_dx_);
  smoother.setDt(0.1f);
  smoother.setStrictCollision(strict_collision_);
  // 上游 main.cpp:147/149：采样间隔 0.1 s、迭代时间上限 10 s。
  smoother.setTL(10.0);

  // 3) 输入：直接使用规划器给出的稀疏路径，不切角、不加密（上游 main.cpp:152）。
  const std::vector<Eigen::Vector2f> points = pointsFromPath(path);

  // 诊断用：同一批路径点的**初始** getCorridor 方块（未经过碰撞迭代 shrink），
  // 这才是"实际还剩多少自由空间"。求解仍然只用 result.corridor。
  std::vector<SfcBox> free_space_boxes;
  if (smoother.getSfc().isUseable() && max_corridor_range_ > 0.0f) {
    const auto free_corridor = smoother.getSfc().getCorridor(
      points, std::min(max_corridor_range_, kMaxCorridorRangeLimit), corridor_shrink_);
    free_space_boxes.assign(free_corridor.corridor.begin(), free_corridor.corridor.end());
  }

  MinimumSnap::SolveInput input;
  input.setPath(points);
  // 上游 main.cpp:164：起点速度显式约束为 0；setPath 已把终点速度设为 0。
  input.setInitVel(Eigen::Vector2f::Zero());
  input.setBackend(toBackend(backend_));
  input.setMaxSpeed(max_speed_);
  input.setMaxAcc(max_acc_);
  input.setNormTime(norm_time_);
  input.setCollisionCheckIter(collision_iter_);

  // 上游 SfcSquare::_getBound() 把 maxRange 直接 static_cast<int>(maxRange/mapping)。
  // 若参数给 1e9（"相当于不限"），在 0.05 m/格 的地图上得到 2e10，超出 int 范围，
  // 溢出后循环边界异常，_getBound 会越界读并直接把进程打崩。
  // 这里做一次硬性夹紧：1e6 / 0.05 = 2e7，仍在 int 范围内。
  input.setMaxCorridorRange(std::min(max_corridor_range_, kMaxCorridorRangeLimit));
  input.setCorridorShrink(corridor_shrink_);

  // 4) 求解。上游不收敛时返回空路径且 success=false（无闭式兜底、不回退），
  //    此时保留原路径并返回 false，由 smoother_server 决定后续行为。
  const auto t0 = std::chrono::steady_clock::now();
  MinimumSnap::SolveOutput result = smoother.solve(input);
  const double elapsed =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();

  // 上游 main.cpp:184-190：橙色方块是求解迭代里真正当约束用的那组（可能被 shrink）；
  // 黄色方块是初始自由空间，即使 OSQP 失败也会发出来，便于判断是"空间小"还是"解失败"。
  const bool solved = result.success && result.path.size() >= 2;
  std::vector<SfcBox> final_boxes;
  if (solved) {
    final_boxes.assign(result.corridor.begin(), result.corridor.end());
  }
  publishSfcMarkers(path, free_space_boxes, final_boxes, solved);

  if (!solved) {
    RCLCPP_WARN_THROTTLE(
      node->get_logger(), *clock_, 2000,
      "MinimumSnapSmoother: OSQP solve failed (backend=%s, iter=%d, %.1f ms), keep original path",
      backend_.c_str(), result.iter, result.time * 1000.0);
    return false;
  }

  // 5) 写回。只有确认成功后才改 path，保证失败路径不被破坏。
  writePath(path, result.path, orientation_mode_);
  path.header.stamp = node->now();

  RCLCPP_DEBUG(
    node->get_logger(),
    "MinimumSnapSmoother: %zu -> %zu points, iter=%d, solve %.1f ms, wall %.1f ms",
    points.size(), result.path.size(), result.iter, result.time * 1000.0, elapsed * 1000.0);
  return true;
}

}  // namespace srm27_nav_smoother

PLUGINLIB_EXPORT_CLASS(srm27_nav_smoother::MinimumSnapSmoother, nav2_core::Smoother)
