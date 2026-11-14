// Copyright 2026 SRM27 Navigation Team
// Licensed under the Apache License, Version 2.0.
//
// 上游 vendor/ 内的文件适用 MIT，见 vendor/LICENCE 与 THIRD_PARTY_NOTICES.md。

#ifndef TDT_NAV_KIT__TDT_SMOOTHER_HPP_
#define TDT_NAV_KIT__TDT_SMOOTHER_HPP_

#include <memory>
#include <string>

#include "nav2_core/smoother.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_costmap_2d/footprint_subscriber.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tdt_nav_kit/costmap_converter.hpp"
#include "tf2_ros/buffer.h"

namespace tdt_nav_kit
{

/// 把 vendored MinimumSnap(SfcSquare + OSQP) 包装成 Nav2 平滑器。
///
/// 签名按 ros-navigation/navigation2 的 humble 分支头文件核对（2026-11-04）：
///   nav2_core/include/nav2_core/smoother.hpp
/// configure 与 GlobalPlanner 不同：第 4/5 参数是 CostmapSubscriber 与
/// FootprintSubscriber（不是 Costmap2DROS）；smooth 的第二参数是 rclcpp::Duration。
///
/// **失败时不得改动传入的 path**：上游 OSQP 不收敛会返回空路径且没有闭式兜底
/// （PATCHES.md「未打补丁的已知上游问题」），此时直接返回 false，
/// 让 smoother_server 保留原始路径继续下发。
class TdtSmoother : public nav2_core::Smoother
{
public:
  TdtSmoother() = default;
  ~TdtSmoother() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub,
    std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  bool smooth(
    nav_msgs::msg::Path & path,
    const rclcpp::Duration & max_time) override;

private:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;

  /// 起点速度：上游默认「自由」（不约束），实车切换目标时会造成横向加速度突变。
  /// 打开后取底盘当前速度（车体系）旋转到地图系，作为 setInitVel 的约束。
  bool init_vel_from_odom_{false};
  std::string odom_topic_{"odom"};
  double init_vel_max_age_{0.3};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  Eigen::Vector2f init_vel_{Eigen::Vector2f::Zero()};
  rclcpp::Time init_vel_stamp_{0, 0, RCL_ROS_TIME};
  bool init_vel_valid_{false};

  void onOdom(const nav_msgs::msg::Odometry & msg);
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;
  std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub_;
  std::string plugin_name_;
  rclcpp::Clock::SharedPtr clock_;
  bool active_{false};

  ConverterConfig converter_cfg_;

  /// SFC / MinimumSnap 参数，默认值刻意与上游 README 示例一致而非库默认。
  int order_{5};
  int max_dx_{3};
  bool strict_collision_{false};
  /// "path"（OSQPPath，默认）| "close"（闭式）| "corridor"（OSQPCorridor）
  ///
  /// 2026-11-05 在 RMUC 2026 仿真地图上用**加密后**的路径重测（11 点 / 34.46 m
  /// 经 path_resample_step=0.30 加密到 1191 点）：
  ///   path     : 成功，2 / 4 / 10 / 17 ms（多次）
  ///   close    : 成功，83 ms
  ///   corridor : 失败（OSQP 不收敛），408 ms
  /// 三者输出几何一致（34.56 m、最大步长 0.04 m、无碰撞），故取最快的 path。
  /// 注意：加密前 path 与 corridor 都会失败，换后端前先确认加密是开的。
  std::string backend_{"path"};
  int collision_iter_{2};
  bool norm_time_{true};
  /// 走廊最大扩张范围（米）。不要给 1e9 之类的"不限"值：
  /// 上游 _getBound 会把它除以分辨率再转 int，0.05 m/格 时直接溢出。
  float max_corridor_range_{10.0f};
  float corridor_shrink_{0.0f};
  float max_speed_{2.0f};
  float max_acc_{2.0f};

  /// 少于该点数的路径不做平滑（直接返回 true）。
  int min_points_{2};
  /// 平滑前的路径加密步长（米）。0 表示不加密。
  ///
  /// 上游 MinimumSnap 的碰撞检查是按「相邻输入点连线/样条采样」判定的，
  /// 输入路径越稀，样条在拐角处越容易切进膨胀层：实测 11 点 / 段长 5.7 m 的
  /// YAstar 路径在 check_for_collisions=true 时**必定失败**并原样返回。
  /// 先按该步长线性插值，段长降到 0.3 m 量级，碰撞检查才能通过。
  /// 只影响送进求解器的采样，不影响输出语义（输出仍是平滑后的轨迹）。
  float path_resample_step_{0.3f};
  /// 加密后的点数上限，防止病态输入（极长路径 + 极小步长）撑爆内存。
  int path_resample_max_points_{2000};

  /// 求解失败时是否返回「已加密的折线」而不是原稀疏路径。
  ///
  /// Nav2 的控制器按 2.5 m 半径裁剪全局路径：上游规划器输出 10 m 级长段，
  /// 失败后原样返回会让裁剪切到只剩 1 个点，前视点落在车身上，
  /// 控制器于是只给 0.2 m/s 的爬行速度，行为树判「无进展」后中止。
  /// 打开后失败时回退到 0.30 m 间距的折线，控制器仍能正常跟踪。
  bool fallback_to_resampled_{true};

  /// 折角圆弧化的切角长度（米）。0 表示不切角。
  ///
  /// MinimumSnap 对输入点是插值的，折线本身的尖角会被原样保留：实测一条
  /// 545° 总转角的 YAstar 路径上出现 24 rad/m 的瞬时曲率（转弯半径 4 cm），
  /// 底盘无法跟随，控制器会切内弯。用二次贝塞尔在拐点切角后曲率显著下降。
  float corner_round_cut_{0.4f};
  /// 只有转角大于该值（度）才切角，避免把近似直行的路径切出人为波动。
  float corner_round_min_turn_deg_{25.0f};
  /// 切角贝塞尔的采样步长（米）。
  float corner_round_sample_step_{0.2f};
  /// "identity"（w=1）或 "tangent"（由相邻点求 yaw）。
  std::string orientation_mode_{"tangent"};
};

}  // namespace tdt_nav_kit

#endif  // TDT_NAV_KIT__TDT_SMOOTHER_HPP_
