// Copyright 2026 SRM27 Navigation Team
// Licensed under the Apache License, Version 2.0.

#include "srm27_nav_common/path_conversion.hpp"

#include <cmath>
#include <cstddef>
#include <vector>

namespace srm27_nav_common
{

OrientationMode parseOrientationMode(const std::string & name)
{
  return name == "tangent" ? OrientationMode::kTangent : OrientationMode::kIdentity;
}

const char * orientationModeName(OrientationMode mode)
{
  return mode == OrientationMode::kTangent ? "tangent" : "identity";
}

PointVec pointsFromPath(const nav_msgs::msg::Path & path)
{
  PointVec points;
  points.reserve(path.poses.size());
  for (const auto & pose : path.poses) {
    points.emplace_back(
      static_cast<float>(pose.pose.position.x),
      static_cast<float>(pose.pose.position.y));
  }
  return points;
}

namespace
{

void setPosition(geometry_msgs::msg::PoseStamped & pose, const Eigen::Vector2f & point)
{
  pose.pose.position.x = point.x();
  pose.pose.position.y = point.y();
  pose.pose.position.z = 0.0;
  pose.pose.orientation.w = 1.0;
}

/// 由相邻点求切线 yaw；退化段保持 identity（w=1）。
void setTangent(geometry_msgs::msg::PoseStamped & pose, const Eigen::Vector2f & from,
  const Eigen::Vector2f & to)
{
  const double dx = static_cast<double>(to.x()) - from.x();
  const double dy = static_cast<double>(to.y()) - from.y();
  if (std::hypot(dx, dy) > 1e-9) {
    const double half_yaw = 0.5 * std::atan2(dy, dx);
    pose.pose.orientation.w = std::cos(half_yaw);
    pose.pose.orientation.z = std::sin(half_yaw);
  }
}

}  // namespace

nav_msgs::msg::Path toPath(
  const PointVec & points, const std::string & frame_id,
  const rclcpp::Time & stamp, OrientationMode mode)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = frame_id;
  path.header.stamp = stamp;
  path.poses.reserve(points.size());

  for (std::size_t i = 0; i < points.size(); ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    setPosition(pose, points[i]);
    if (mode == OrientationMode::kTangent) {
      const std::size_t prev = (i > 0) ? i - 1 : i;
      const std::size_t next = (i + 1 < points.size()) ? i + 1 : i;
      setTangent(pose, points[prev], points[next]);
    }
    path.poses.push_back(pose);
  }
  return path;
}

void writePath(
  nav_msgs::msg::Path & path, const PointVec & points, OrientationMode mode)
{
  const auto header = path.header;
  path.poses.clear();
  path.poses.reserve(points.size());

  for (std::size_t i = 0; i < points.size(); ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = header;
    setPosition(pose, points[i]);
    if (mode == OrientationMode::kTangent) {
      const std::size_t prev = (i > 0) ? i - 1 : i;
      const std::size_t next = (i + 1 < points.size()) ? i + 1 : i;
      setTangent(pose, points[prev], points[next]);
    }
    path.poses.push_back(pose);
  }
}

}  // namespace srm27_nav_common
