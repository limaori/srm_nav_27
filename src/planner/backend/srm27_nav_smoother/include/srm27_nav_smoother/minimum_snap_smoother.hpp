// Copyright 2026 SRM27 Navigation Team
// Licensed under the Apache License, Version 2.0.
//
// 算法来自东北大学 T-DT 的 tdt-nav-kit（MIT），vendored 源码在本包
// vendor/src/MinimumSnapOsqp/ 内，见本包 THIRD_PARTY_NOTICES.md 与 PATCHES.md。

#ifndef SRM27_NAV_SMOOTHER__MINIMUM_SNAP_SMOOTHER_HPP_
#define SRM27_NAV_SMOOTHER__MINIMUM_SNAP_SMOOTHER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "nav2_core/smoother.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_costmap_2d/footprint_subscriber.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "srm27_map_adapter/costmap_adapter.hpp"
#include "srm27_nav_common/path_conversion.hpp"
#include "srm27_nav_smoother/sfc_visualization.hpp"
#include "tf2_ros/buffer.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace srm27_nav_smoother
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
class MinimumSnapSmoother : public nav2_core::Smoother
{
public:
  MinimumSnapSmoother() = default;
  ~MinimumSnapSmoother() override = default;

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

  /// 发布一次走廊 MarkerArray。
  /// 黄色细线 = `getCorridor()` 的初始自由方块（未做碰撞收缩）；
  /// 橙色粗线 = 求解迭代后实际使用的最终约束（可能被 shrink 成小方块）。
  /// 任一集合为空时对应 namespace 发 DELETE；白色文字给出数量/平均尺寸。
  void publishSfcMarkers(
    const nav_msgs::msg::Path & path,
    const std::vector<SfcBox> & free_space_boxes,
    const std::vector<SfcBox> & final_boxes,
    bool solve_ok);

  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;
  std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub_;
  std::string plugin_name_;
  rclcpp::Clock::SharedPtr clock_;
  bool active_{false};

  /// Nav2 代价图 → 地图快照（阈值固定为参数 occupancy_threshold）。
  srm27_map_adapter::CostmapConfig costmap_config_;
  bool treat_unknown_as_obstacle_{true};

  /// 以下默认值全部对齐上游 main.cpp:145-164，不再保留本仓库的调参。
  /// order 是**多项式系数个数**：6 = 五次多项式（0~5 阶），不是阶数。
  int order_{6};
  int max_dx_{3};
  bool strict_collision_{false};
  /// "path"（OSQPPath）| "close"（闭式）| "corridor"（OSQPCorridor）
  /// 上游 main.cpp:163 固定 OSQPCorridor，这里默认也用 corridor。
  std::string backend_{"corridor"};
  int collision_iter_{6};
  bool norm_time_{true};
  /// 走廊最大扩张范围（米）。上游值 2.5；不要给 1e9 之类的"不限"值，
  /// 上游 _getBound 会把它除以分辨率再转 int，0.05 m/格 时直接溢出。
  float max_corridor_range_{2.5f};
  float corridor_shrink_{0.0f};
  float max_speed_{3.0f};
  float max_acc_{1.0f};
  /// "identity"（w=1）或 "tangent"（由相邻点求 yaw）。
  srm27_nav_common::OrientationMode orientation_mode_{
    srm27_nav_common::OrientationMode::kTangent};

  // —— SFC（方形安全飞行走廊）可视化 ——
  /// 是否把走廊发成 MarkerArray 供 RViz 显示。
  ///
  /// 与上游 main.cpp:184-190 一致：默认 corridor 后端发布求解迭代里真正用到的那组
  /// 方块（可能被插入/收缩过）；path/close 后端只会有退化点。
  bool publish_sfc_{true};
  /// 发布话题。私有名 `~/sfc_corridor` 解析为 `/<smoother_server>/sfc_corridor`。
  std::string sfc_topic_{"~/sfc_corridor"};
  /// 走廊线宽（米）。
  float sfc_line_width_{0.02f};
  SfcMarkerStyle sfc_style_{};
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr sfc_pub_;
};

}  // namespace srm27_nav_smoother

#endif  // SRM27_NAV_SMOOTHER__MINIMUM_SNAP_SMOOTHER_HPP_
