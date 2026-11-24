// Copyright 2026 SRM27 Navigation Team
// Licensed under the Apache License, Version 2.0.
//
// 算法来自东北大学 T-DT 的 tdt-nav-kit（MIT），vendored 源码在本包
// vendor/src/MinimumSnapOsqp/ 内，见本包 THIRD_PARTY_NOTICES.md。

#ifndef SRM27_NAV_SMOOTHER__SFC_VISUALIZATION_HPP_
#define SRM27_NAV_SMOOTHER__SFC_VISUALIZATION_HPP_

#include <array>
#include <string>
#include <vector>

#include "rclcpp/time.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace srm27_nav_smoother
{

/// 一个方形走廊块：世界坐标下的轴对齐矩形 (x_min, y_min, x_max, y_max)，单位米。
/// 字段顺序与 SfcSquare::PointPair 完全一致，可以直接互转/assign。
using SfcBox = std::array<float, 4>;

/// SFC 走廊的显示样式。默认值按「叠在 costmap / 路径上仍能看清」取。
struct SfcMarkerStyle
{
  /// Marker 命名空间。同一 (ns, id) 会被下一次发布整体替换，不会残留旧方块。
  std::string marker_namespace{"sfc_corridor"};
  /// 线宽（米）。
  float line_width{0.02f};
  /// 抬高一点画，避免与 z=0 的 costmap 平面打架（z-fighting）。
  float z{0.02f};
  float red{0.10f};
  float green{0.85f};
  float blue{1.00f};
  float alpha{0.90f};
};

/**
 * @brief 把方形安全飞行走廊转成 RViz 可直接显示的 MarkerArray。
 *
 * 所有方块画在**同一个** LINE_LIST 里（每块 4 条边、8 个点）：
 * 上游 `getCorridor()` 沿加密路径逐点给块，一条 30 m 的路径就是上千块，
 * 一块一个 Marker 会让 RViz 明显卡顿。
 *
 * 退化方块（`x_min == x_max` 或 `y_min == y_max`，以及含非有限值的块）会被跳过：
 * 上游 `getCorridor()` 的首末块本来就是退化成点的方块，`backend=path` 时整组
 * 走廊也都是这种点（见 minimum_snap_smoother.cpp 里对实际约束走廊的选取）。
 *
 * @param boxes     走廊方块，通常来自 SfcSquare::getCorridor().corridor
 * @param frame_id  Marker 所在坐标系（与传入的 Path 的 frame_id 一致）
 * @param stamp     时间戳
 * @param style     线宽/颜色等样式
 * @return 单元素 MarkerArray；boxes 为空或全部退化时返回一个 DELETE Marker，
 *         用于把 RViz 里上一目标的走廊清掉
 */
visualization_msgs::msg::MarkerArray makeSfcMarkerArray(
  const std::vector<SfcBox> & boxes,
  const std::string & frame_id,
  const rclcpp::Time & stamp,
  const SfcMarkerStyle & style = SfcMarkerStyle());

}  // namespace srm27_nav_smoother

#endif  // SRM27_NAV_SMOOTHER__SFC_VISUALIZATION_HPP_
