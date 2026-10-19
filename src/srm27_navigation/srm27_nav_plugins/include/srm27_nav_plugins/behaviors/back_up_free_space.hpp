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

#ifndef SRM27_NAV_PLUGINS__BEHAVIORS__BACK_UP_FREE_SPACE_HPP_
#define SRM27_NAV_PLUGINS__BEHAVIORS__BACK_UP_FREE_SPACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "nav2_behaviors/plugins/drive_on_heading.hpp"
#include "nav2_msgs/action/back_up.hpp"
#include "nav2_msgs/srv/get_costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "srm27_minco_core/recovery_decider.hpp"

using BackUpAction = nav2_msgs::action::BackUp;

namespace srm27_nav_behaviors
{

/**
 * @class srm27_nav_behaviors::BackUpFreeSpace
 * @brief An enhanced back_up action that move toward free space
 */
class BackUpFreeSpace : public nav2_behaviors::DriveOnHeading<nav2_msgs::action::BackUp>
{
public:
  BackUpFreeSpace() = default;

  /**
   * @brief Configuration of behavior action
   */
  void onConfigure() override;

  /**
   * @brief Cleanup server on lifecycle transition
   */
  void onCleanup() override;

  /**
   * @brief Initialization to run behavior
   * @param command Goal to execute
   * @return Status of behavior
   */
  nav2_behaviors::Status onRun(const std::shared_ptr<const BackUpAction::Goal> command) override;

  /**
   * @brief Loop function to run behavior
   * @return Status of behavior
   */
  nav2_behaviors::Status onCycleUpdate() override;

protected:
  /**
   * @brief Gather free points within a specified radius from the center in the costmap.
   *
   * This function iterates through the costmap and collects points that are free (costmap value is 0)
   * and within the specified radius from the given center coordinates (center_x, center_y).
   *
   * @param costmap The costmap to search for free points.
   * @param center_x The x-coordinate of the center point.
   * @param center_y The y-coordinate of the center point.
   * @param radius The radius within which to gather free points.
   * @return A vector of points that are free and within the specified radius.
   */
  std::vector<geometry_msgs::msg::Point> gatherFreePoints(
    const nav2_msgs::msg::Costmap & costmap, geometry_msgs::msg::Pose2D pose, float radius);

  /**
   * @brief 代价图直方图法选方向（原实现）：按角度扫描每一条射线，取"连续安全扇区"的中点。
   *
   * 保留为 `escape_direction_mode: costmap_gap` 的回退路径与 A/B 对照。
   */
  float findBestDirection(
    const nav2_msgs::msg::Costmap & costmap, geometry_msgs::msg::Pose2D pose, float start_angle,
    float end_angle, float radius, float angle_increment);

  /**
   * @brief 把 `GetCostmap` 返回的代价图转成 `srm27_minco_core::GridSnapshot`。
   *
   * 语义必须与 MINCO 控制器里的 `CostmapAdapter::snapshot` 保持一致，否则脱困方向
   * 与规划器看到的障碍不是同一份：
   *  * `LETHAL_OBSTACLE`(254) -> 占用；
   *  * `NO_INFORMATION`(255) -> 未知（距离场按障碍种子处理）；
   *  * **膨胀代价 1..252 不写入原始占用** —— 机器人半径由调用方的净空阈值单独计入。
   *
   * 数据布局：`GetCostmap` 直接返回 `Costmap2D::getCharMap()`，索引是
   * `my * size_x + mx`（行优先、自左下角起），与 `GridSnapshot` 完全一致，**不需要翻转 y**。
   * （nav_msgs/OccupancyGrid 才是自上而下，本服务不是。）
   */
  bool buildGridSnapshot(
    const nav2_msgs::msg::Costmap & costmap, srm27_minco_core::GridSnapshot * snapshot) const;

  /**
   * @brief 用 ESDF 解析梯度 + 环形搜索规划脱困方向。
   *
   * 算法本体就是 `srm27_minco_core::RecoveryDecider::planEscapeDirection`——
   * 与控制器内部的脱困判定共用同一份实现，保证"控制器认为该往哪推"和
   * "行为树实际往哪推"不会分叉。
   *
   * @return 无法得到方向（代价图无效 / 机器人在地图外）时返回 false，
   *         此时调用方必须回退到 `findBestDirection`，绝不能盲目前冲。
   */
  bool planEscapeDirectionByEsdf(
    const nav2_msgs::msg::Costmap & costmap, geometry_msgs::msg::Pose2D pose, float * best_angle);

  void visualize(
    geometry_msgs::msg::Pose2D pose, float radius, float first_safe_angle, float last_unsafe_angle);

  /// @brief 发布 ESDF 分支的脱困可视化（净空球 + 逃逸方向箭头）。
  void visualizeEscape(geometry_msgs::msg::Pose2D pose, float clearance, float best_angle);

  rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedPtr costmap_client_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>>
    marker_pub_;
  double twist_x_, twist_y_;

  // parameters
  std::string service_name_;
  double max_radius_;
  bool visualize_;
  /// @brief 方向来源：`esdf_gradient`（默认，ESDF 解析梯度 + 环形搜索）或 `costmap_gap`。
  std::string escape_direction_mode_;
  /// @brief 环形搜索半径（m）。
  double esdf_search_radius_;
  /// @brief 环形搜索采样数量。
  int esdf_search_samples_;
  /// @brief ESDF 是否把未知区域当作障碍种子。
  bool esdf_unknown_is_seed_;
  /// @brief 最近一次算出的自身净空（m），诊断用。
  double last_clearance_{0.0};
  /// @brief 最近一次是走环形搜索（梯度退化）还是梯度分支，诊断用。
  bool last_used_circular_search_{false};
};

}  // namespace srm27_nav_behaviors

#endif  // SRM27_NAV_PLUGINS__BEHAVIORS__BACK_UP_FREE_SPACE_HPP_
