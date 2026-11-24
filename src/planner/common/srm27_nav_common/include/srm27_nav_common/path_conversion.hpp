// Copyright 2026 SRM27 Navigation Team
// Licensed under the Apache License, Version 2.0.

#ifndef SRM27_NAV_COMMON__PATH_CONVERSION_HPP_
#define SRM27_NAV_COMMON__PATH_CONVERSION_HPP_

#include <string>

#include "nav_msgs/msg/path.hpp"
#include "rclcpp/time.hpp"
#include "srm27_nav_common/path_utils.hpp"

namespace srm27_nav_common
{

/// 输出的朝向语义。
///
/// "identity"（默认）：四元数恒为 w=1，朝向由下游控制器决定；
/// "tangent"：由相邻路径点求切线 yaw（规划器与平滑器共用同一实现）。
enum class OrientationMode
{
  kIdentity,
  kTangent,
};

OrientationMode parseOrientationMode(const std::string & name);

/// 与 parseOrientationMode 互逆的名称（用于参数默认值与日志）。
const char * orientationModeName(OrientationMode mode);

/// Path -> 平面点列。上游算法只处理 xy，z 与朝向在此丢弃。
PointVec pointsFromPath(const nav_msgs::msg::Path & path);

/// 平面点列 -> Path，补齐 frame_id / stamp / 朝向。
nav_msgs::msg::Path toPath(
  const PointVec & points, const std::string & frame_id,
  const rclcpp::Time & stamp, OrientationMode mode);

/// 用新的点列改写 path，保留原 header（frame_id / stamp 由调用方按需覆盖）。
/// 只有确认规划/平滑成功后才应调用，保证失败路径不被破坏。
void writePath(
  nav_msgs::msg::Path & path, const PointVec & points, OrientationMode mode);

}  // namespace srm27_nav_common

#endif  // SRM27_NAV_COMMON__PATH_CONVERSION_HPP_
