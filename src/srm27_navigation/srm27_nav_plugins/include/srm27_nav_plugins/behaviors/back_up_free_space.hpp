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
   * @brief 只把机器人周围 `radius` 米见方的一块抠成快照（用于每周期量净空）。
   *
   * 为什么要抠窗口：`service_name_` 指向的是**全局**代价地图（583×324 @0.05 m），
   * 在它上面整张建一次 ESDF 实测 **15~24 ms**；而 `onCycleUpdate` 是 10 Hz 周期，
   * 每周期付 20 ms 等于吃掉 20% 的时间预算（实车 CPU 更慢）。抠成
   * `2 × radius` 见方的窗口后只需 ~0.2 ms。
   *
   * 正确性：窗口以**机器人当前格**为中心、半边长 `radius`。距离场只会把窗口内的
   * 单元当种子，因此
   *  * 量到的净空 **≥** 真实净空 —— 判"还很挤"永远不会误判；
   *  * 只要 `radius` > `early_exit_clearance_` + 一个栅格，窗口外的种子必然比阈值远，
   *    所以量到"净空 > 阈值"时真实净空也一定 > 阈值 —— 判"已经空出来"也不会误判。
   * 调用方按 `max(1.0, 2 × early_exit_clearance_)` 取半径即可满足这两条。
   */
  bool buildGridSnapshotAround(
    const nav2_msgs::msg::Costmap & costmap, const geometry_msgs::msg::Pose2D & pose,
    double radius, srm27_minco_core::GridSnapshot * snapshot) const;

  /**
   * @brief 取一次当前代价地图上的自身净空（米）；数据不可信时返回 NaN。
   *
   * 数据来源是 `service_name_`（默认即 `global_costmap/get_costmap`），与
   * `planEscapeDirectionByEsdf` 完全同一份处理链路：`buildGridSnapshot` ->
   * `Esdf2D::build` -> 双线性插值查询。因此这里量到的净空和"选方向"用的净空是
   * 同一个数，也与 MINCO 控制器内部脱困判定的 `escaped_clearance` 同源。
   *
   * 返回 NaN 的三种情形都必须被调用方当作"不知道"，绝不当成"很空"：
   *  1. 服务不可用 / 100 ms 内没有回包（不能像 `onRun` 那样等 1 s，会拖死 10 Hz 周期）；
   *  2. 代价图无效（尺寸/分辨率非正、数据长度对不上、机器人在地图外）；
   *  3. `early_exit_require_obstacles_` 为真而整张图里一格致命障碍都没有 ——
   *     见 `shouldExitEarly()` 里的说明。
   */
  double queryCurrentClearance(const geometry_msgs::msg::Pose2D & pose);

  /**
   * @brief 提前退出判定：**已经站在正常空地里**就不再盲推满 `backup_dist`。
   *
   * 这是链路 A（控制器内部脱困 `RecoveryDecider::updateRecovery`）的
   * `escaped_clearance` 在链路 B 上的对应物。原实现只有三个出口：走满距离 / 碰撞
   * 预演失败 / `time_allowance` 超时，于是"方向早已无意义、车已经脱离危险"之后
   * 还会继续按冻结的方向推满 1 m —— 2026-11-03 那次实测里 4 次 backup 全部报
   * `completed successfully`，同期自身净空 0.978 -> 0.369 -> 0.321 -> 0.204 ->
   * 0.006 m 单调下降，最后把车顶死在障碍上。
   *
   * 判据刻意做成"连续 N 个周期都满足"而不是单帧：代价图在清图后、雷达重新标记前
   * 有短暂的空窗期，单帧判据会在空窗期里误判为"自由"。
   *
   * @return true 表示应当停止并返回 SUCCEEDED（调用方负责 `stopRobot()`）。
   */
  bool shouldExitEarly(const geometry_msgs::msg::Pose2D & pose, double * clearance);

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

  // === 提前退出（P1）：见 shouldExitEarly() 的说明 ===
  /// @brief 是否启用提前退出。
  bool early_exit_enable_{true};
  /// @brief 提前退出阈值（m）：自身净空超过它即认为已在正常空地。
  ///
  /// 必须 > 代价地图的 `robot_radius`（仿真/实车 MINCO 都是 0.40）——
  /// 否则全局规划器会把"车自己那一格"判成内接膨胀里的障碍，重规划依旧失败，
  /// 退出等于白退（2026-11-03 日志里那 10 条
  /// `Either of the start or goal pose are an obstacle!` 就是这么来的）。
  double early_exit_clearance_{0.55};
  /// @brief 需要连续多少个周期满足才退出（10 Hz 下 3 个周期 ≈ 0.3 s）。
  int early_exit_hold_cycles_{3};
  /// @brief 至少先真走出多少米才有资格判"脱困了"（m）。
  ///
  /// 防抖核心：挡住"清图 -> 净空虚高 -> 立刻退出 -> 重规划仍失败 -> 再清图"的
  /// 零位移空转，保证每一轮 backup 至少产生 `early_exit_min_travel_` 的位移。
  double early_exit_min_travel_{0.30};
  /// @brief 至少先跑多少秒才允许判退出（s）：同时给雷达留出重新标记代价图的时间。
  double early_exit_min_time_{1.0};
  /// @brief 空图保护：整张代价图里没有任何致命障碍时不认净空结论。
  bool early_exit_require_obstacles_{true};
  /// @brief 连续满足阈值的计数（低于阈值或数据不可信时清零）。
  int early_exit_hold_count_{0};
  /// @brief 本次 backup 的起始时刻，用于 `early_exit_min_time_`。
  rclcpp::Time early_exit_start_stamp_{0, 0, RCL_ROS_TIME};
};

}  // namespace srm27_nav_behaviors

#endif  // SRM27_NAV_PLUGINS__BEHAVIORS__BACK_UP_FREE_SPACE_HPP_
