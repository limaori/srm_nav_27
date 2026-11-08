// Copyright 2024 Polaris Xia
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

#include "srm27_nav_plugins/behaviors/back_up_free_space.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"

namespace srm27_nav_behaviors
{

void BackUpFreeSpace::onConfigure()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  nav2_util::declare_parameter_if_not_declared(node, "global_frame", rclcpp::ParameterValue("map"));
  nav2_util::declare_parameter_if_not_declared(node, "max_radius", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, "service_name", rclcpp::ParameterValue("local_costmap/get_costmap"));
  nav2_util::declare_parameter_if_not_declared(node, "visualize", rclcpp::ParameterValue(false));
  // 方向来源：默认用 ESDF 解析梯度（与控制器内部的脱困判定同源），
  // `costmap_gap` 是原实现（扫描"连续安全扇区"取中点）的回退与 A/B 对照。
  nav2_util::declare_parameter_if_not_declared(
    node, "escape_direction_mode", rclcpp::ParameterValue(std::string("esdf_gradient")));
  nav2_util::declare_parameter_if_not_declared(
    node, "esdf_search_radius", rclcpp::ParameterValue(0.4));
  nav2_util::declare_parameter_if_not_declared(
    node, "esdf_search_samples", rclcpp::ParameterValue(36));
  nav2_util::declare_parameter_if_not_declared(
    node, "esdf_unknown_is_seed", rclcpp::ParameterValue(true));
  // === 提前退出（P1）：语义对应链路 A 的 escaped_clearance ===
  // 背景见 shouldExitEarly() 的注释：原实现的三个出口里没有"我已经在空地了"。
  nav2_util::declare_parameter_if_not_declared(
    node, "early_exit_enable", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, "early_exit_clearance", rclcpp::ParameterValue(0.55));
  nav2_util::declare_parameter_if_not_declared(
    node, "early_exit_hold_cycles", rclcpp::ParameterValue(3));
  nav2_util::declare_parameter_if_not_declared(
    node, "early_exit_min_travel", rclcpp::ParameterValue(0.30));
  nav2_util::declare_parameter_if_not_declared(
    node, "early_exit_min_time", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, "early_exit_require_obstacles", rclcpp::ParameterValue(true));

  node->get_parameter("global_frame", global_frame_);
  node->get_parameter("max_radius", max_radius_);
  node->get_parameter("service_name", service_name_);
  node->get_parameter("visualize", visualize_);
  node->get_parameter("escape_direction_mode", escape_direction_mode_);
  node->get_parameter("esdf_search_radius", esdf_search_radius_);
  node->get_parameter("esdf_search_samples", esdf_search_samples_);
  node->get_parameter("esdf_unknown_is_seed", esdf_unknown_is_seed_);
  node->get_parameter("early_exit_enable", early_exit_enable_);
  node->get_parameter("early_exit_clearance", early_exit_clearance_);
  node->get_parameter("early_exit_hold_cycles", early_exit_hold_cycles_);
  node->get_parameter("early_exit_min_travel", early_exit_min_travel_);
  node->get_parameter("early_exit_min_time", early_exit_min_time_);
  node->get_parameter("early_exit_require_obstacles", early_exit_require_obstacles_);

  // 参数来自 yaml，畸形值一律夹到安全侧而不是抛异常：抛异常会让整个 behavior_server
  // configure 失败，代价远大于"用一个保守默认值继续跑"。
  if (!std::isfinite(early_exit_clearance_) || early_exit_clearance_ <= 0.0) {
    early_exit_clearance_ = 0.55;
  }
  early_exit_hold_cycles_ = std::max(1, early_exit_hold_cycles_);
  if (!std::isfinite(early_exit_min_travel_) || early_exit_min_travel_ < 0.0) {
    early_exit_min_travel_ = 0.0;
  }
  if (!std::isfinite(early_exit_min_time_) || early_exit_min_time_ < 0.0) {
    early_exit_min_time_ = 0.0;
  }

  if (escape_direction_mode_ != "esdf_gradient" && escape_direction_mode_ != "costmap_gap") {
    RCLCPP_WARN(
      logger_, "escape_direction_mode: '%s' 未识别，回退到 esdf_gradient",
      escape_direction_mode_.c_str());
    escape_direction_mode_ = "esdf_gradient";
  }

  costmap_client_ = node->create_client<nav2_msgs::srv::GetCostmap>(service_name_);

  if (visualize_) {
    marker_pub_ = node->template create_publisher<visualization_msgs::msg::MarkerArray>(
      "back_up_free_space_markers", 1);
    marker_pub_->on_activate();
  }
}

void BackUpFreeSpace::onCleanup()
{
  costmap_client_.reset();
  marker_pub_.reset();
  early_exit_hold_count_ = 0;
}

nav2_behaviors::Status BackUpFreeSpace::onRun(
  const std::shared_ptr<const BackUpAction::Goal> command)
{
  while (!costmap_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(logger_, "Interrupted while waiting for the service. Exiting.");
      return nav2_behaviors::Status::FAILED;
    }
    RCLCPP_WARN(logger_, "service not available, waiting again...");
  }

  auto request = std::make_shared<nav2_msgs::srv::GetCostmap::Request>();
  auto result = costmap_client_->async_send_request(request);
  if (result.wait_for(std::chrono::seconds(1)) == std::future_status::timeout) {
    RCLCPP_ERROR(logger_, "Interrupted while waiting for the service. Exiting.");
    return nav2_behaviors::Status::FAILED;
  }

  // get costmap
  auto costmap = result.get()->map;

  if (!nav2_util::getCurrentPose(
        initial_pose_, *tf_, global_frame_, robot_base_frame_, transform_tolerance_)) {
    RCLCPP_ERROR(logger_, "Initial robot pose is not available.");
    return nav2_behaviors::Status::FAILED;
  }

  // get current pose
  geometry_msgs::msg::Pose2D pose;
  pose.x = initial_pose_.pose.position.x;
  pose.y = initial_pose_.pose.position.y;
  pose.theta = tf2::getYaw(initial_pose_.pose.orientation);

  // Find the best direction to back up.
  //
  // 方向来源与 MINCO 控制器内部的脱困判定同源（ESDF 解析梯度 + 环形搜索），
  // 这样"控制器认为该往哪推"和"行为树实际往哪推"不会分叉。
  // ESDF 分支失败（代价图无效 / 机器人在地图外）时必须回退到代价图直方图法，
  // 绝不能因为"算不出来"就沿用默认角度盲目前冲。
  float best_angle = 0.0f;
  bool direction_ok = false;
  if (escape_direction_mode_ == "esdf_gradient") {
    direction_ok = planEscapeDirectionByEsdf(costmap, pose, &best_angle);
    if (!direction_ok) {
      RCLCPP_WARN(
        logger_, "ESDF 逃逸方向不可用（代价图无效或机器人在地图外），回退到代价图直方图法");
    }
  }
  if (!direction_ok) {
    best_angle = findBestDirection(costmap, pose, -M_PI, M_PI, max_radius_, M_PI / 32.0);
    last_used_circular_search_ = false;
  }

  // Calculate move command
  twist_x_ = std::cos(best_angle) * command->speed;
  twist_y_ = std::sin(best_angle) * command->speed;
  command_x_ = command->target.x;
  command_time_allowance_ = command->time_allowance;

  end_time_ = clock_->now() + command_time_allowance_;
  // 提前退出的判据以"本次 backup 启动"为基准计时/计数，每次动作都必须从零开始，
  // 否则上一轮留下的 hold 计数会让下一轮一上来就满足"连续 N 个周期"。
  early_exit_start_stamp_ = clock_->now();
  early_exit_hold_count_ = 0;

  if (!nav2_util::getCurrentPose(
        initial_pose_, *tf_, global_frame_, robot_base_frame_, transform_tolerance_)) {
    RCLCPP_ERROR(logger_, "Initial robot pose is not available.");
    return nav2_behaviors::Status::FAILED;
  }
  RCLCPP_WARN(
    logger_, "backing up %f meters towards free space at angle %f (mode=%s, 净空 %.3f m, %s)",
    command_x_, best_angle, escape_direction_mode_.c_str(), last_clearance_,
    last_used_circular_search_ ? "环形搜索" : "ESDF 梯度");

  return nav2_behaviors::Status::SUCCEEDED;
}

nav2_behaviors::Status BackUpFreeSpace::onCycleUpdate()
{
  rclcpp::Duration time_remaining = end_time_ - clock_->now();
  if (time_remaining.seconds() < 0.0 && command_time_allowance_.seconds() > 0.0) {
    stopRobot();
    RCLCPP_WARN(
      logger_,
      "Exceeded time allowance before reaching the "
      "DriveOnHeading goal - Exiting DriveOnHeading");
    return nav2_behaviors::Status::FAILED;
  }

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
        current_pose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_)) {
    RCLCPP_ERROR(logger_, "Current robot pose is not available.");
    return nav2_behaviors::Status::FAILED;
  }

  float diff_x = initial_pose_.pose.position.x - current_pose.pose.position.x;
  float diff_y = initial_pose_.pose.position.y - current_pose.pose.position.y;
  float distance = hypot(diff_x, diff_y);

  feedback_->distance_traveled = distance;
  action_server_->publish_feedback(feedback_);

  if (distance >= std::fabs(command_x_)) {
    stopRobot();
    return nav2_behaviors::Status::SUCCEEDED;
  }

  geometry_msgs::msg::Pose2D pose;
  pose.x = current_pose.pose.position.x;
  pose.y = current_pose.pose.position.y;
  pose.theta = tf2::getYaw(current_pose.pose.orientation);

  // === 提前退出（P1）：已经站在正常空地就停，不再盲推满 backup_dist ===
  //
  // 对应链路 A 的 `escaped_clearance` 判定。两道时间/位移护栏必须先满足：
  //  * `early_exit_min_travel_`：必须先真走出去一段，否则会退化成
  //    "清图 -> 净空虚高 -> 立刻退出 -> 重规划仍失败 -> 再清图"的原地空转；
  //  * `early_exit_min_time_`：给雷达留出重新标记代价图的时间（BT 的
  //    RecoveryFallback 是"先清图再 Backup"，进这个函数时图可能还是空的）。
  // 两个护栏都不满足的周期里把 hold 计数清零，保证"连续"是真的连续。
  if (early_exit_enable_) {
    const double elapsed = (clock_->now() - early_exit_start_stamp_).seconds();
    if (distance >= early_exit_min_travel_ && elapsed >= early_exit_min_time_) {
      double clearance = std::numeric_limits<double>::quiet_NaN();
      if (shouldExitEarly(pose, &clearance)) {
        stopRobot();
        RCLCPP_WARN(
          logger_,
          "提前退出脱困：自身净空 %.3f m > %.2f m，已无必要继续推 —— "
          "已走 %.2f m / %.2f s（本次目标 %.2f m），交回重规划。",
          clearance, early_exit_clearance_, static_cast<double>(distance), elapsed,
          std::fabs(command_x_));
        return nav2_behaviors::Status::SUCCEEDED;
      }
    } else {
      early_exit_hold_count_ = 0;
    }
  }

  auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
  cmd_vel->linear.y = twist_y_;
  cmd_vel->linear.x = twist_x_;

  if (!isCollisionFree(distance, cmd_vel.get(), pose)) {
    stopRobot();
    RCLCPP_WARN(logger_, "Collision Ahead - Exiting DriveOnHeading");
    return nav2_behaviors::Status::FAILED;
  }

  vel_pub_->publish(std::move(cmd_vel));

  return nav2_behaviors::Status::RUNNING;
}

double BackUpFreeSpace::queryCurrentClearance(const geometry_msgs::msg::Pose2D & pose)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  if (!costmap_client_ || !costmap_client_->service_is_ready()) {
    return nan;
  }

  auto request = std::make_shared<nav2_msgs::srv::GetCostmap::Request>();
  auto result = costmap_client_->async_send_request(request);
  // 只等 100 ms：正常情况 1~2 ms 就回。**不能**像 onRun 那样等 1 s ——
  // 那会把 10 Hz 的行为周期直接拖死，而"这一周期没有可信数据"本来就该被跳过。
  if (result.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
    return nan;
  }
  const auto costmap = result.get()->map;

  // 空图保护：整张代价图里一格致命障碍都没有 => 这份数据不可信。
  // 最典型的来源就是"刚清完图、雷达还没重新标记"，此时"到处都开阔"是假象
  // （2026-11-03 那次实测：清图后 0.4 s 报出 净空 0.978 m 并据此选了方向）。
  if (early_exit_require_obstacles_) {
    bool has_obstacle = false;
    for (const auto cost : costmap.data) {
      if (cost == nav2_costmap_2d::LETHAL_OBSTACLE) {
        has_obstacle = true;
        break;
      }
    }
    if (!has_obstacle) {
      return nan;
    }
  }

  // 与 planEscapeDirectionByEsdf 完全同一条链路：快照 -> ESDF -> 插值查询。
  // 净空的语义（只把原始占用与未知当种子、膨胀代价 1..252 视为自由）因此与
  // "选方向"和控制器内部脱困判定完全一致。
  //
  // 唯一区别是**只抠机器人周围一小块**：整张全局图建一次 ESDF 实测 15~24 ms，
  // 而这里是 10 Hz 的周期函数，付不起（详见 buildGridSnapshotAround 的说明）。
  const double crop_radius = std::max(1.0, 2.0 * early_exit_clearance_);
  srm27_minco_core::GridSnapshot snapshot;
  if (!buildGridSnapshotAround(costmap, pose, crop_radius, &snapshot)) {
    return nan;
  }
  srm27_minco_core::Esdf2D esdf;
  srm27_minco_core::Esdf2D::Config esdf_config;
  esdf_config.unknown_is_seed = esdf_unknown_is_seed_;
  if (!esdf.build(snapshot, esdf_config)) {
    return nan;
  }
  const srm27_minco_core::EsdfQueryResult here = esdf.query(pose.x, pose.y);
  if (!here.valid || !std::isfinite(here.distance)) {
    return nan;
  }
  // 窗口内一个障碍种子都没有时，`Esdf2D` 返回的是 `no_obstacle_distance`(1e3) 哨兵值：
  // 它只说明"最近障碍比窗口半径还远"，**不是**精确测量值。直接往日志里打
  // 1000.000 m 会被当成 bug（2026-11-03 的日志里就这么出现过一次），因此把上报
  // 值夹到窗口半径。判据不受影响：阈值(0.55) 永远小于窗口半径(1.10)。
  const double reported = std::min(here.distance, crop_radius);
  last_clearance_ = reported;
  return reported;
}

bool BackUpFreeSpace::shouldExitEarly(
  const geometry_msgs::msg::Pose2D & pose, double * clearance)
{
  if (clearance != nullptr) {
    *clearance = std::numeric_limits<double>::quiet_NaN();
  }

  const double current = queryCurrentClearance(pose);
  if (clearance != nullptr) {
    *clearance = current;
  }
  // 数据不可信（NaN）与"确实还很挤"一律清零计数：宁可多推一格，也不要误退。
  if (!std::isfinite(current) || current <= early_exit_clearance_) {
    early_exit_hold_count_ = 0;
    return false;
  }

  ++early_exit_hold_count_;
  if (early_exit_hold_count_ < early_exit_hold_cycles_) {
    if (early_exit_hold_count_ == 1) {
      RCLCPP_INFO(
        logger_, "脱困净空已恢复（%.3f m > %.2f m），连续确认中 (%d/%d)",
        current, early_exit_clearance_, early_exit_hold_count_, early_exit_hold_cycles_);
    }
    return false;
  }
  return true;
}

bool BackUpFreeSpace::planEscapeDirectionByEsdf(
  const nav2_msgs::msg::Costmap & costmap, geometry_msgs::msg::Pose2D pose, float * best_angle)
{
  if (best_angle == nullptr) {
    return false;
  }

  srm27_minco_core::GridSnapshot snapshot;
  if (!buildGridSnapshot(costmap, &snapshot)) {
    return false;
  }

  srm27_minco_core::Esdf2D esdf;
  srm27_minco_core::Esdf2D::Config esdf_config;
  esdf_config.unknown_is_seed = esdf_unknown_is_seed_;
  if (!esdf.build(snapshot, esdf_config)) {
    return false;
  }

  srm27_minco_core::RecoveryConfig recovery_config;
  recovery_config.search_radius = esdf_search_radius_;
  recovery_config.search_samples = esdf_search_samples_;
  // 这里只借用"算方向"这一件事，不涉及失败计数/冷却/窗口，那几个成员保持默认即可。

  Eigen::Vector2d direction = Eigen::Vector2d::Zero();
  double clearance = -1.0;
  const Eigen::Vector2d position(static_cast<double>(pose.x), static_cast<double>(pose.y));
  if (!srm27_minco_core::RecoveryDecider::planEscapeDirection(
        esdf, position, recovery_config, &direction, &clearance)) {
    return false;
  }

  last_clearance_ = clearance;
  const float angle = static_cast<float>(std::atan2(direction.y(), direction.x()));
  // 诊断标注：`planEscapeDirection` 有三个分支（梯度上升 / 环形搜索 / 失败）。
  // 这里复现它的分支条件，好让日志能说清"这次方向是梯度给的还是环形搜索给的"——
  // 两者在现场对应完全不同的处境：梯度 = 还贴着障碍、环形 = 已经陷在梯度平台的死角里。
  const srm27_minco_core::EsdfQueryResult here = esdf.query(position.x(), position.y());
  last_used_circular_search_ = !(
    here.gradient_valid && here.gradient.allFinite() &&
    here.gradient.norm() >= recovery_config.gradient_epsilon);
  *best_angle = angle;

  if (visualize_) {
    visualizeEscape(pose, static_cast<float>(clearance), angle);
  }
  return true;
}

bool BackUpFreeSpace::buildGridSnapshot(
  const nav2_msgs::msg::Costmap & costmap, srm27_minco_core::GridSnapshot * snapshot) const
{
  if (snapshot == nullptr) {
    return false;
  }
  const int size_x = static_cast<int>(costmap.metadata.size_x);
  const int size_y = static_cast<int>(costmap.metadata.size_y);
  const double resolution = costmap.metadata.resolution;
  if (size_x <= 0 || size_y <= 0 || !(resolution > 0.0)) {
    return false;
  }
  const std::size_t count = static_cast<std::size_t>(size_x) * static_cast<std::size_t>(size_y);
  if (costmap.data.size() != count) {
    return false;
  }

  // 与 `CostmapAdapter::snapshot` 同一套语义：只有原始占用与未知进距离场，
  // 膨胀代价 1..252 一律视为自由 —— 机器人半径已经在净空阈值里单独计入，
  // 重复计入会让有效通道凭空变窄（会把本来能过的窄通道判成"无处可逃"）。
  std::vector<srm27_minco_core::CellState> cells(count, srm27_minco_core::CellState::kFree);
  for (std::size_t i = 0; i < count; ++i) {
    const std::uint8_t cost = costmap.data[i];
    if (cost == nav2_costmap_2d::LETHAL_OBSTACLE) {
      cells[i] = srm27_minco_core::CellState::kOccupied;
    } else if (cost == nav2_costmap_2d::NO_INFORMATION) {
      cells[i] = srm27_minco_core::CellState::kUnknown;
    }
  }

  return snapshot->reset(
    costmap.metadata.origin.position.x, costmap.metadata.origin.position.y, resolution, size_x,
    size_y, std::move(cells), /*map_version=*/0u, clock_->now().seconds());
}

bool BackUpFreeSpace::buildGridSnapshotAround(
  const nav2_msgs::msg::Costmap & costmap, const geometry_msgs::msg::Pose2D & pose,
  double radius, srm27_minco_core::GridSnapshot * snapshot) const
{
  if (snapshot == nullptr) {
    return false;
  }
  const int size_x = static_cast<int>(costmap.metadata.size_x);
  const int size_y = static_cast<int>(costmap.metadata.size_y);
  const double resolution = costmap.metadata.resolution;
  const double origin_x = costmap.metadata.origin.position.x;
  const double origin_y = costmap.metadata.origin.position.y;
  if (size_x <= 0 || size_y <= 0 || !(resolution > 0.0)) {
    return false;
  }
  const std::size_t count = static_cast<std::size_t>(size_x) * static_cast<std::size_t>(size_y);
  if (costmap.data.size() != count) {
    return false;
  }
  if (!std::isfinite(pose.x) || !std::isfinite(pose.y)) {
    return false;
  }

  // 机器人所在格（可能落在图外 -> 给不出可信净空）。
  const int robot_mx = static_cast<int>(std::floor((pose.x - origin_x) / resolution));
  const int robot_my = static_cast<int>(std::floor((pose.y - origin_y) / resolution));
  if (robot_mx < 0 || robot_my < 0 || robot_mx >= size_x || robot_my >= size_y) {
    return false;
  }

  const double crop = (std::isfinite(radius) && radius > 0.0) ? radius : 1.0;
  const int half = std::max(1, static_cast<int>(std::ceil(crop / resolution)));
  const int x0 = std::max(0, robot_mx - half);
  const int y0 = std::max(0, robot_my - half);
  const int x1 = std::min(size_x, robot_mx + half + 1);
  const int y1 = std::min(size_y, robot_my + half + 1);
  const int nx = x1 - x0;
  const int ny = y1 - y0;
  if (nx <= 0 || ny <= 0) {
    return false;
  }

  // 逐行抠图。索引布局与 `buildGridSnapshot` 完全一致：行优先、自左下角起，
  // 即 `my * size_x + mx`（`GetCostmap` 返回的是 `Costmap2D::getCharMap()`，不是
  // nav_msgs/OccupancyGrid 那种自上而下的布局，不需要翻转 y）。
  std::vector<srm27_minco_core::CellState> cells(
    static_cast<std::size_t>(nx) * static_cast<std::size_t>(ny),
    srm27_minco_core::CellState::kFree);
  for (int my = y0; my < y1; ++my) {
    const std::size_t src_row = static_cast<std::size_t>(my) * static_cast<std::size_t>(size_x);
    const std::size_t dst_row = static_cast<std::size_t>(my - y0) * static_cast<std::size_t>(nx);
    for (int mx = x0; mx < x1; ++mx) {
      const std::uint8_t cost = costmap.data[src_row + static_cast<std::size_t>(mx)];
      srm27_minco_core::CellState & cell = cells[dst_row + static_cast<std::size_t>(mx - x0)];
      if (cost == nav2_costmap_2d::LETHAL_OBSTACLE) {
        cell = srm27_minco_core::CellState::kOccupied;
      } else if (cost == nav2_costmap_2d::NO_INFORMATION) {
        cell = srm27_minco_core::CellState::kUnknown;
      }
    }
  }

  return snapshot->reset(
    origin_x + static_cast<double>(x0) * resolution,
    origin_y + static_cast<double>(y0) * resolution, resolution, nx, ny, std::move(cells),
    /*map_version=*/0u, clock_->now().seconds());
}

void BackUpFreeSpace::visualizeEscape(
  geometry_msgs::msg::Pose2D pose, float clearance, float best_angle)
{
  if (!marker_pub_) {
    return;
  }
  visualization_msgs::msg::MarkerArray markers;

  visualization_msgs::msg::Marker clearance_marker;
  clearance_marker.header.frame_id = global_frame_;
  clearance_marker.header.stamp = clock_->now();
  clearance_marker.ns = "escape_clearance";
  clearance_marker.id = 0;
  clearance_marker.type = visualization_msgs::msg::Marker::SPHERE;
  clearance_marker.action = visualization_msgs::msg::Marker::ADD;
  clearance_marker.pose.position.x = pose.x;
  clearance_marker.pose.position.y = pose.y;
  clearance_marker.pose.position.z = 0.0;
  clearance_marker.pose.orientation.w = 1.0;
  const float drawn = (std::isfinite(clearance) && clearance > 0.0f)
                        ? std::min(clearance, 2.0f)
                        : 0.05f;
  clearance_marker.scale.x = 2.0f * drawn;
  clearance_marker.scale.y = 2.0f * drawn;
  clearance_marker.scale.z = 0.05;
  clearance_marker.color.r = 1.0f;
  clearance_marker.color.g = 1.0f;
  clearance_marker.color.b = 0.0f;
  clearance_marker.color.a = 0.35f;
  markers.markers.push_back(clearance_marker);

  visualization_msgs::msg::Marker arrow;
  arrow.header.frame_id = global_frame_;
  arrow.header.stamp = clearance_marker.header.stamp;
  arrow.ns = "escape_direction";
  arrow.id = 1;
  arrow.type = visualization_msgs::msg::Marker::ARROW;
  arrow.action = visualization_msgs::msg::Marker::ADD;
  arrow.scale.x = 0.06;
  arrow.scale.y = 0.12;
  arrow.scale.z = 0.12;
  arrow.color.r = 0.0f;
  arrow.color.g = 0.6f;
  arrow.color.b = 1.0f;
  arrow.color.a = 1.0f;
  geometry_msgs::msg::Point start;
  start.x = pose.x;
  start.y = pose.y;
  start.z = 0.0;
  geometry_msgs::msg::Point end = start;
  const float reach = static_cast<float>(max_radius_);
  end.x += reach * std::cos(best_angle);
  end.y += reach * std::sin(best_angle);
  arrow.points.push_back(start);
  arrow.points.push_back(end);
  markers.markers.push_back(arrow);

  marker_pub_->publish(markers);
}

float BackUpFreeSpace::findBestDirection(
  const nav2_msgs::msg::Costmap & costmap, geometry_msgs::msg::Pose2D pose, float start_angle,
  float end_angle, float radius, float angle_increment)
{
  float best_angle = start_angle;

  float first_safe_angle = -1.0f;
  float last_unsafe_angle = -1.0f;

  float final_safe_angle = 0.0f;
  float final_unsafe_angle = 0.0f;

  float resolution = costmap.metadata.resolution;
  float origin_x = costmap.metadata.origin.position.x;
  float origin_y = costmap.metadata.origin.position.y;
  int size_x = costmap.metadata.size_x;
  int size_y = costmap.metadata.size_y;

  float map_min_x = origin_x;
  float map_max_x = origin_x + (size_x * resolution);
  float map_min_y = origin_y;
  float map_max_y = origin_y + (size_y * resolution);

  for (float angle = start_angle; angle <= end_angle; angle += angle_increment) {
    bool is_safe = true;

    for (float r = 0; r <= radius; r += resolution) {
      float x = pose.x + r * std::cos(angle);
      float y = pose.y + r * std::sin(angle);

      if (x >= map_min_x && x <= map_max_x && y >= map_min_y && y <= map_max_y) {
        int i = static_cast<int>((x - origin_x) / resolution);
        int j = static_cast<int>((y - origin_y) / resolution);

        if (i >= 0 && i < size_x && j >= 0 && j < size_y) {
          if (costmap.data[i + j * size_x] >= 253) {
            is_safe = false;
            break;
          }
        } else {
          is_safe = false;
          break;
        }
      } else {
        is_safe = false;
        break;
      }
    }
    if (is_safe && first_safe_angle == -1.0f) {
      first_safe_angle = angle;
    }

    if (!is_safe && first_safe_angle != -1.0f && last_unsafe_angle == -1.0f) {
      last_unsafe_angle = angle;
    }

    if (
      last_unsafe_angle - first_safe_angle > final_unsafe_angle - final_safe_angle &&
      first_safe_angle != -1.0f && last_unsafe_angle != -1.0f) {
      final_safe_angle = first_safe_angle;
      final_unsafe_angle = last_unsafe_angle;
      first_safe_angle = -1.0f;
      last_unsafe_angle = -1.0f;
    }
  }
  best_angle = (final_safe_angle + final_unsafe_angle) / 2.0f;

  if (visualize_) {
    visualize(pose, radius, final_safe_angle, final_unsafe_angle);
  }

  return best_angle;
}

std::vector<geometry_msgs::msg::Point> BackUpFreeSpace::gatherFreePoints(
  const nav2_msgs::msg::Costmap & costmap, geometry_msgs::msg::Pose2D pose, float radius)
{
  std::vector<geometry_msgs::msg::Point> results;
  for (unsigned int i = 0; i < costmap.metadata.size_x; i++) {
    for (unsigned int j = 0; j < costmap.metadata.size_y; j++) {
      auto idx = i + j * costmap.metadata.size_x;
      auto x = i * costmap.metadata.resolution + costmap.metadata.origin.position.x;
      auto y = j * costmap.metadata.resolution + costmap.metadata.origin.position.y;
      if (std::hypot(x - pose.x, y - pose.y) <= radius && costmap.data[idx] == 0) {
        geometry_msgs::msg::Point p;
        p.x = x;
        p.y = y;
        results.push_back(p);
      }
    }
  }
  return results;
}

void BackUpFreeSpace::visualize(
  geometry_msgs::msg::Pose2D pose, float radius, float first_safe_angle, float last_unsafe_angle)
{
  visualization_msgs::msg::MarkerArray markers;

  visualization_msgs::msg::Marker sector_marker;
  sector_marker.header.frame_id = global_frame_;
  sector_marker.header.stamp = clock_->now();
  sector_marker.ns = "direction";
  sector_marker.id = 0;
  sector_marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
  sector_marker.action = visualization_msgs::msg::Marker::ADD;
  sector_marker.scale.x = 1.0;
  sector_marker.scale.y = 1.0;
  sector_marker.scale.z = 1.0;
  sector_marker.color.r = 0.0f;
  sector_marker.color.g = 1.0f;
  sector_marker.color.b = 0.0f;
  sector_marker.color.a = 0.2f;

  const float angle_step = 0.05f;
  for (float angle = first_safe_angle; angle <= last_unsafe_angle; angle += angle_step) {
    const float next_angle = std::min(angle + angle_step, last_unsafe_angle);

    geometry_msgs::msg::Point origin;
    origin.x = pose.x;
    origin.y = pose.y;
    origin.z = 0.0;

    geometry_msgs::msg::Point p1;
    p1.x = pose.x + radius * std::cos(angle);
    p1.y = pose.y + radius * std::sin(angle);
    p1.z = 0.0;

    geometry_msgs::msg::Point p2;
    p2.x = pose.x + radius * std::cos(next_angle);
    p2.y = pose.y + radius * std::sin(next_angle);
    p2.z = 0.0;

    sector_marker.points.push_back(origin);
    sector_marker.points.push_back(p1);
    sector_marker.points.push_back(p2);
  }
  markers.markers.push_back(sector_marker);

  auto create_arrow = [&](float angle, int id, float r, float g, float b) {
    visualization_msgs::msg::Marker arrow;
    arrow.header.frame_id = global_frame_;
    arrow.header.stamp = clock_->now();
    arrow.ns = "direction";
    arrow.id = id;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;
    arrow.scale.x = 0.05;
    arrow.scale.y = 0.1;
    arrow.scale.z = 0.1;
    arrow.color.r = r;
    arrow.color.g = g;
    arrow.color.b = b;
    arrow.color.a = 1.0;

    geometry_msgs::msg::Point start;
    start.x = pose.x;
    start.y = pose.y;
    start.z = 0.0;

    geometry_msgs::msg::Point end;
    end.x = start.x + radius * std::cos(angle);
    end.y = start.y + radius * std::sin(angle);
    end.z = 0.0;

    arrow.points.push_back(start);
    arrow.points.push_back(end);
    return arrow;
  };

  markers.markers.push_back(create_arrow(first_safe_angle, 1, 0.0f, 0.0f, 1.0f));
  markers.markers.push_back(create_arrow(last_unsafe_angle, 2, 0.0f, 0.0f, 1.0f));

  const float best_angle = (first_safe_angle + last_unsafe_angle) / 2.0f;
  markers.markers.push_back(create_arrow(best_angle, 3, 0.0f, 1.0f, 0.0f));

  marker_pub_->publish(markers);
}

}  // namespace srm27_nav_behaviors

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(srm27_nav_behaviors::BackUpFreeSpace, nav2_core::Behavior)
