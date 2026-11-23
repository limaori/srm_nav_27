// Copyright 2026 SRM27 Navigation Team
// Licensed under the Apache License, Version 2.0.
//
// SFC 走廊 MarkerArray 的结构测试：RViz 只认 Marker 的字段，画不出来时
// 现场很难分辨是「没发」还是「发了但字段不对」，所以这里把关键字段钉死。

#include <gtest/gtest.h>

#include <limits>
#include <vector>

#include "tdt_nav_kit/sfc_visualization.hpp"

namespace
{
const rclcpp::Time kStamp(1700000000, 123456789, RCL_ROS_TIME);
}  // namespace

TEST(SfcMarkerTest, OneLineListWithFourEdgesPerSquare)
{
  const std::vector<tdt_nav_kit::SfcBox> boxes = {
    {0.0f, 0.0f, 2.0f, 1.0f},
    {1.0f, 1.0f, 1.5f, 2.0f}};

  const auto array = tdt_nav_kit::makeSfcMarkerArray(boxes, "map", kStamp);

  ASSERT_EQ(array.markers.size(), 1u);
  const auto & marker = array.markers[0];
  EXPECT_EQ(marker.header.frame_id, "map");
  EXPECT_EQ(marker.header.stamp.sec, 1700000000);
  EXPECT_EQ(marker.header.stamp.nanosec, 123456789u);
  EXPECT_EQ(marker.ns, "tdt_sfc");
  EXPECT_EQ(marker.id, 0);
  EXPECT_EQ(marker.type, visualization_msgs::msg::Marker::LINE_LIST);
  EXPECT_EQ(marker.action, visualization_msgs::msg::Marker::ADD);
  ASSERT_EQ(marker.points.size(), boxes.size() * 8);

  // 第一块的第一条边：(0,0) -> (2,0)
  EXPECT_DOUBLE_EQ(marker.points[0].x, 0.0);
  EXPECT_DOUBLE_EQ(marker.points[0].y, 0.0);
  EXPECT_DOUBLE_EQ(marker.points[1].x, 2.0);
  EXPECT_DOUBLE_EQ(marker.points[1].y, 0.0);
  // 第一块闭合回起点：最后一条边是 (0,1) -> (0,0)
  EXPECT_DOUBLE_EQ(marker.points[6].x, 0.0);
  EXPECT_DOUBLE_EQ(marker.points[6].y, 1.0);
  EXPECT_DOUBLE_EQ(marker.points[7].x, 0.0);
  EXPECT_DOUBLE_EQ(marker.points[7].y, 0.0);

  EXPECT_GT(marker.points[0].z, 0.0) << "必须抬到 costmap 平面之上，否则会 z-fighting";
  EXPECT_GT(marker.scale.x, 0.0f);
  EXPECT_GT(marker.color.a, 0.0f);
}

TEST(SfcMarkerTest, SkipsDegenerateAndNonFiniteBoxes)
{
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const std::vector<tdt_nav_kit::SfcBox> boxes = {
    {1.0f, 1.0f, 1.0f, 1.0f},   // getCorridor 的首末块就是这种退化成点的方块
    {0.0f, 0.0f, 2.0f, 2.0f},   // 唯一一块可画的
    {0.0f, 0.0f, 2.0f, 0.0f},   // y 方向没有高度
    {nan, 0.0f, 1.0f, 1.0f}};   // 非有限值

  const auto array = tdt_nav_kit::makeSfcMarkerArray(boxes, "map", kStamp);

  ASSERT_EQ(array.markers.size(), 1u);
  EXPECT_EQ(array.markers[0].action, visualization_msgs::msg::Marker::ADD);
  EXPECT_EQ(array.markers[0].points.size(), 8u);
}

TEST(SfcMarkerTest, EmptyCorridorDeletesPreviousMarker)
{
  const auto array = tdt_nav_kit::makeSfcMarkerArray({}, "map", kStamp);

  ASSERT_EQ(array.markers.size(), 1u);
  const auto & marker = array.markers[0];
  EXPECT_EQ(marker.action, visualization_msgs::msg::Marker::DELETE)
    << "空走廊要用 DELETE 清掉 RViz 里上一次的方块";
  EXPECT_TRUE(marker.points.empty());
  EXPECT_EQ(marker.ns, "tdt_sfc");
}

TEST(SfcMarkerTest, AppliesStyle)
{
  tdt_nav_kit::SfcMarkerStyle style;
  style.marker_namespace = "corridor";
  style.line_width = 0.05f;
  style.z = 0.10f;
  style.red = 1.0f;
  style.green = 0.0f;
  style.blue = 0.0f;
  style.alpha = 0.5f;

  const auto array = tdt_nav_kit::makeSfcMarkerArray(
    {{0.0f, 0.0f, 1.0f, 1.0f}}, "odom", kStamp, style);

  const auto & marker = array.markers[0];
  EXPECT_EQ(marker.ns, "corridor");
  EXPECT_EQ(marker.header.frame_id, "odom");
  EXPECT_FLOAT_EQ(marker.scale.x, 0.05f);
  EXPECT_NEAR(marker.points[0].z, 0.10, 1e-6);
  EXPECT_FLOAT_EQ(marker.color.r, 1.0f);
  EXPECT_FLOAT_EQ(marker.color.g, 0.0f);
  EXPECT_FLOAT_EQ(marker.color.b, 0.0f);
  EXPECT_FLOAT_EQ(marker.color.a, 0.5f);
}
