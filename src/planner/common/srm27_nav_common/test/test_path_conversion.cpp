// Copyright 2026 SRM27 Navigation Team
// Licensed under the Apache License, Version 2.0.
//
// Path 消息 <-> 平面点列的转换（规划器与平滑器共用同一实现）。
// 2026-11-24 合并 tdt_nav_kit 时从两个插件里各抽出一份重复代码，
// 这里把朝向语义与 header 保留规则钉死。

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "srm27_nav_common/path_conversion.hpp"
#include "srm27_nav_common/path_utils.hpp"

namespace
{

using srm27_nav_common::OrientationMode;
using srm27_nav_common::orientationModeName;
using srm27_nav_common::parseOrientationMode;
using srm27_nav_common::PointVec;
using srm27_nav_common::pointsFromPath;
using srm27_nav_common::toPath;
using srm27_nav_common::writePath;

const rclcpp::Time kStamp(1700000000, 123456789, RCL_ROS_TIME);

PointVec makeLine(float x0, float y0, float x1, float y1, int n)
{
  PointVec pts;
  for (int i = 0; i < n; ++i) {
    const float t = static_cast<float>(i) / static_cast<float>(n - 1);
    pts.emplace_back(x0 + t * (x1 - x0), y0 + t * (y1 - y0));
  }
  return pts;
}

double yawOf(const geometry_msgs::msg::PoseStamped & pose)
{
  return 2.0 * std::atan2(pose.pose.orientation.z, pose.pose.orientation.w);
}

}  // namespace

TEST(PathConversionTest, ParseModeRoundTrip)
{
  EXPECT_EQ(parseOrientationMode("tangent"), OrientationMode::kTangent);
  // 未知/空值一律回退 identity，与合并前的行为一致。
  EXPECT_EQ(parseOrientationMode("identity"), OrientationMode::kIdentity);
  EXPECT_EQ(parseOrientationMode(""), OrientationMode::kIdentity);
  EXPECT_STREQ(orientationModeName(OrientationMode::kTangent), "tangent");
  EXPECT_STREQ(orientationModeName(OrientationMode::kIdentity), "identity");
}

TEST(PathConversionTest, ToPathPreservesFrameStampAndXY)
{
  const PointVec points = makeLine(0.0f, 0.0f, 2.0f, 0.0f, 3);
  const auto path = toPath(points, "map", kStamp, OrientationMode::kIdentity);

  ASSERT_EQ(path.poses.size(), points.size());
  EXPECT_EQ(path.header.frame_id, "map");
  EXPECT_EQ(path.header.stamp, kStamp);
  for (std::size_t i = 0; i < points.size(); ++i) {
    EXPECT_FLOAT_EQ(path.poses[i].pose.position.x, points[i].x());
    EXPECT_FLOAT_EQ(path.poses[i].pose.position.y, points[i].y());
    // identity：四元数恒为 w=1，朝向交给下游控制器。
    EXPECT_DOUBLE_EQ(path.poses[i].pose.orientation.w, 1.0);
    EXPECT_DOUBLE_EQ(path.poses[i].pose.orientation.z, 0.0);
    EXPECT_EQ(path.poses[i].header.frame_id, "map");
  }
}

TEST(PathConversionTest, TangentFollowsSegmentDirection)
{
  // 先沿 +x 再沿 +y：中间拐点的切线由相邻两点求得。
  PointVec points;
  points.emplace_back(0.0f, 0.0f);
  points.emplace_back(1.0f, 0.0f);
  points.emplace_back(1.0f, 1.0f);
  const auto path = toPath(points, "map", kStamp, OrientationMode::kTangent);

  ASSERT_EQ(path.poses.size(), 3u);
  EXPECT_NEAR(yawOf(path.poses[0]), 0.0, 1e-6);
  EXPECT_NEAR(yawOf(path.poses[1]), M_PI / 4.0, 1e-6);
  EXPECT_NEAR(yawOf(path.poses[2]), M_PI / 2.0, 1e-6);

  const auto round_trip = pointsFromPath(path);
  ASSERT_EQ(round_trip.size(), points.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    EXPECT_FLOAT_EQ(round_trip[i].x(), points[i].x());
    EXPECT_FLOAT_EQ(round_trip[i].y(), points[i].y());
  }
}

TEST(PathConversionTest, WritePathKeepsHeaderButDropsStalePoses)
{
  const PointVec source = makeLine(0.0f, 0.0f, 1.0f, 0.0f, 2);
  auto path = toPath(source, "odom", kStamp, OrientationMode::kIdentity);

  const PointVec smoothed = makeLine(0.0f, 0.0f, 1.0f, 1.0f, 4);
  writePath(path, smoothed, OrientationMode::kTangent);

  ASSERT_EQ(path.poses.size(), smoothed.size());
  EXPECT_EQ(path.header.frame_id, "odom");
  EXPECT_EQ(path.header.stamp, kStamp);
  for (const auto & pose : path.poses) {
    EXPECT_EQ(pose.header.frame_id, "odom");
  }
  EXPECT_NEAR(yawOf(path.poses.front()), M_PI / 4.0, 1e-6);
}

TEST(PathConversionTest, EmptyPathIsHandled)
{
  const nav_msgs::msg::Path empty;
  EXPECT_TRUE(pointsFromPath(empty).empty());

  const auto path = toPath({}, "map", kStamp, OrientationMode::kTangent);
  EXPECT_TRUE(path.poses.empty());
  EXPECT_EQ(path.header.frame_id, "map");
}
