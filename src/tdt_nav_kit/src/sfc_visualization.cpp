// Copyright 2026 SRM27 Navigation Team
// Licensed under the Apache License, Version 2.0.

#include "tdt_nav_kit/sfc_visualization.hpp"

#include <algorithm>
#include <cmath>

namespace tdt_nav_kit
{
namespace
{

/// Marker 的固定 id：同一 (ns, id) 每次发布都会整体替换，
/// 这样方块数量变化时 RViz 里也不会残留上一次的方块。
constexpr int kSfcMarkerId = 0;

/// 方块有面积、且四个角都是有限值才画。
bool isDrawableBox(const SfcBox & box)
{
  if (!std::all_of(box.begin(), box.end(), [](float value) {return std::isfinite(value);})) {
    return false;
  }
  return box[2] > box[0] && box[3] > box[1];
}

void appendEdge(
  visualization_msgs::msg::Marker & marker, float x0, float y0, float x1, float y1, float z)
{
  geometry_msgs::msg::Point point;
  point.x = x0;
  point.y = y0;
  point.z = z;
  marker.points.push_back(point);

  point.x = x1;
  point.y = y1;
  marker.points.push_back(point);
}

}  // namespace

visualization_msgs::msg::MarkerArray makeSfcMarkerArray(
  const std::vector<SfcBox> & boxes,
  const std::string & frame_id,
  const rclcpp::Time & stamp,
  const SfcMarkerStyle & style)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = stamp;
  marker.ns = style.marker_namespace;
  marker.id = kSfcMarkerId;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.frame_locked = false;
  marker.scale.x = style.line_width;
  marker.color.r = style.red;
  marker.color.g = style.green;
  marker.color.b = style.blue;
  marker.color.a = style.alpha;
  marker.points.reserve(boxes.size() * 8);

  for (const auto & box : boxes) {
    if (!isDrawableBox(box)) {
      continue;
    }
    // 逆时针四条边，闭合回起点。
    appendEdge(marker, box[0], box[1], box[2], box[1], style.z);
    appendEdge(marker, box[2], box[1], box[2], box[3], style.z);
    appendEdge(marker, box[2], box[3], box[0], box[3], style.z);
    appendEdge(marker, box[0], box[3], box[0], box[1], style.z);
  }

  if (marker.points.empty()) {
    // 没有可画的方块：显式删除上一帧，避免 RViz 里留着过期的走廊。
    marker.action = visualization_msgs::msg::Marker::DELETE;
  }

  visualization_msgs::msg::MarkerArray array;
  array.markers.push_back(marker);
  return array;
}

}  // namespace tdt_nav_kit
