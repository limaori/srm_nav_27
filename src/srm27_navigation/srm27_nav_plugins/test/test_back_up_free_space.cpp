// Copyright 2026 SRM
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include "srm27_nav_plugins/behaviors/back_up_free_space.hpp"

namespace
{
class BackUp : public srm27_nav_behaviors::BackUpFreeSpace
{
public:
  using BackUpFreeSpace::findBestDirection;
  using BackUpFreeSpace::isCollisionFree;
  double ahead() const { return simulate_ahead_time_; }
};

class BackupTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("backup_test");
    node->declare_parameter("cycle_frequency", 10.0);
    node->declare_parameter("global_frame", "map");
    node->declare_parameter("robot_base_frame", "base_link");
    node->declare_parameter("transform_tolerance", 0.1);
    node->declare_parameter("simulate_ahead_time", 2.0);
    tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    backup.configure(node, "backup", tf, nullptr);
    map.header.frame_id = "map";
    map.metadata.size_x = map.metadata.size_y = 200;
    map.metadata.resolution = 0.05;
    map.metadata.origin.orientation.w = 1;
    map.data.assign(40000, 0);
    pose.x = pose.y = 5;
  }
  void TearDown() override { backup.cleanup(); }
  rclcpp_lifecycle::LifecycleNode::SharedPtr node;
  std::shared_ptr<tf2_ros::Buffer> tf;
  BackUp backup;
  nav2_msgs::msg::Costmap map;
  geometry_msgs::msg::Pose2D pose;
};

TEST_F(BackupTest, ConfiguredCollisionHorizonIsApplied)
{
  EXPECT_DOUBLE_EQ(backup.ahead(), 2.0);
  geometry_msgs::msg::Twist cmd;
  cmd.linear.y = 0.2;
  EXPECT_FALSE(backup.isCollisionFree(0.0, &cmd, pose));
}

TEST_F(BackupTest, FullyOccupiedMapHasNoDirection)
{
  map.data.assign(map.data.size(), 254);
  float angle = 123;
  EXPECT_FALSE(backup.findBestDirection(map, pose, -M_PI, M_PI, 1, M_PI / 32, &angle));
  EXPECT_FLOAT_EQ(angle, 123);
}

TEST_F(BackupTest, SectorAcrossMinusPiAndPiIsNotLost)
{
  // Only the negative-x side is free; this safe sector crosses the angular boundary.
  for (int y = 0; y < 200; ++y) {
    for (int x = 0; x < 200; ++x) {
      if (x > 103 || y < 95 || y > 105) {
        map.data[y * 200 + x] = 254;
      }
    }
  }
  float angle;
  ASSERT_TRUE(backup.findBestDirection(map, pose, -M_PI, M_PI, 1, M_PI / 32, &angle));
  EXPECT_LT(std::cos(angle), -0.95);
}

TEST_F(BackupTest, InvalidAndOutOfMapInputsHaveNoDirection)
{
  float angle;
  pose.x = -1;
  EXPECT_FALSE(backup.findBestDirection(map, pose, -M_PI, M_PI, 1, M_PI / 32, &angle));
  map.data.clear();
  EXPECT_FALSE(backup.findBestDirection(map, pose, -M_PI, M_PI, 1, M_PI / 32, &angle));
}
}  // namespace
