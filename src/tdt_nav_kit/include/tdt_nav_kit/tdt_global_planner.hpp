// Copyright 2026 SRM27 Navigation Team
// Licensed under the Apache License, Version 2.0.
//
// 上游 vendor/ 内的文件适用 MIT，见 vendor/LICENCE 与 THIRD_PARTY_NOTICES.md。

#ifndef TDT_NAV_KIT__TDT_GLOBAL_PLANNER_HPP_
#define TDT_NAV_KIT__TDT_GLOBAL_PLANNER_HPP_

#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tdt_nav_kit/costmap_converter.hpp"
#include "tdt_nav_kit/planner_geometry.hpp"
#include "tf2_ros/buffer.h"

// vendor 前向声明：applyDynamicMask 只以引用传递，完整定义由 cpp 包含 yastar.hpp。
class YAstar;

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
  StableGoal stable_goal_;

  ConverterConfig converter_cfg_;
  int mask_num_layers_{1};
  float cost_inflate_radius_{1.0f};
  float simplify_epsilon_{0.1f};
  float save_point_max_radius_{0.4f};
  float goal_snap_max_radius_{0.4f};
  float path_resample_step_{0.0f};
  /// "identity"（默认 w=1）或 "tangent"（由相邻路径点求 yaw）。
  std::string orientation_mode_{"identity"};

  /// 打开后在每次 createPlan 末尾打印分段耗时，用于定位瓶颈（默认关闭）。
  bool debug_timing_{false};

  // ===================== 限时动态掩码（2026-11-23） =====================
  // 复现上游 setMaskMapTimed 语义：实时障碍点云（对手车 / 场地新增障碍 /
  // 未建图桌子）栅格化后作为命名掩码层推给 YAstar，duration 窗口内的点持续
  // 生效、超窗自动失效。单帧噪声（墙点抖动）在窗口内自愈，真实障碍只要
  // 仍被观测就每帧刷新、持续存在。
  //
  // wrapper 侧实现说明：createPlan 每次 new 局部 YAstar 实例，vendor 的
  // 限时层随实例销毁，因此"duration 记忆"由本类承担——订阅回调把点云
  // 栅格化成 (row,col) 格子集合并按时间戳缓存，createPlan 时合并 duration
  // 窗口内的格子推送（非限时 setMaskMap，实例生命周期即有效期）。

  /// 点云回调入口（整体异常兜底，见 cpp 内注释）。
  void dynamicCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  /// 回调实现：TF 变换到全局系 → z 带过滤 → 栅格化（含 dilation 膨胀）→ 缓存。
  void dynamicCloudCallbackImpl(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  /// createPlan 内调用：合并 duration 窗口内的缓存格子，推送 "dynamic" 掩码层。
  void applyDynamicMask(
    YAstar & planner, int size_x, int size_y, float resolution,
    float origin_x, float origin_y);

  /// 动态障碍点云话题；空串（默认）= 关闭。
  std::string dynamic_mask_topic_;
  /// 掩码有效期（秒）：窗口内的观测合并生效。
  double dynamic_mask_duration_{0.8};
  /// 每个障碍点的膨胀半径（米）：让规划路径与实时障碍保持车体级间距。
  double dynamic_mask_dilation_{0.15};
  /// 点云 z 过滤带（米）：与代价地图高度窗一致，滤地面噪点与高处结构。
  double dynamic_mask_min_z_{0.10};
  double dynamic_mask_max_z_{0.35};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr dynamic_cloud_sub_;
  std::mutex dynamic_mask_mutex_;
  /// (接收时刻, 格子集合)；格子为 (row, col) 对（map 系、跟随全局代价地图栅格）。
  std::deque<std::pair<rclcpp::Time, std::vector<std::pair<int, int>>>> dynamic_cells_;
  /// TF 超时告警节流（用 double 秒存储，避免 rclcpp::Time 跨时间源相减抛异常：
  /// node->now() 是 ROS_TIME，默认构造的 Time 是 SYSTEM_TIME）。
  double last_tf_warn_sec_{-1.0e9};
};

}  // namespace tdt_nav_kit

#endif  // TDT_NAV_KIT__TDT_GLOBAL_PLANNER_HPP_
