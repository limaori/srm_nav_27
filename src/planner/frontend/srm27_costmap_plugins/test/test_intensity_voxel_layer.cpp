// Copyright 2026 SRM
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <memory>
#include <thread>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_costmap_2d/observation.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "srm27_costmap_plugins/layers/intensity_voxel_layer.hpp"

namespace
{
class PersistentObstacleLayer : public nav2_costmap_2d::Layer
{
public:
  void reset() override {}
  bool isClearable() override { return false; }

  void onInitialize() override
  {
    enabled_ = true;
    current_ = true;
  }

  void updateBounds(
    double, double, double, double * min_x, double * min_y, double * max_x,
    double * max_y) override
  {
    *min_x = std::min(*min_x, obstacle_x_);
    *min_y = std::min(*min_y, obstacle_y_);
    *max_x = std::max(*max_x, obstacle_x_);
    *max_y = std::max(*max_y, obstacle_y_);
  }

  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid, int, int, int, int) override
  {
    unsigned int mx = 0, my = 0;
    if (master_grid.worldToMap(obstacle_x_, obstacle_y_, mx, my)) {
      master_grid.setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
    }
  }

private:
  const double obstacle_x_{2.05};
  const double obstacle_y_{2.05};
};

class RosEnvironment : public ::testing::Environment
{
public:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};
const bool registered = []() {
  ::testing::AddGlobalTestEnvironment(new RosEnvironment());
  return true;
}();

class IntensityVoxelLayerTest : public ::testing::Test
{
protected:
  void configure(
    bool _rolling = false, bool _persistent_obstacle = false, double _hold_time = 0.0,
    bool _footprint_clearing = false)
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides(
      {rclcpp::Parameter("terrain.footprint_clearing_enabled", _footprint_clearing),
       rclcpp::Parameter("terrain.origin_z", 0.0),
       rclcpp::Parameter("terrain.hold_time", _hold_time)});
    node_ = std::make_shared<nav2_util::LifecycleNode>("intensity_layer_test", "", options);
    tf_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    map_ = std::make_unique<nav2_costmap_2d::LayeredCostmap>("map", _rolling, false);
    map_->resizeMap(100, 100, 0.1, -5.0, -5.0);
    if (_footprint_clearing) {
      map_->setFootprint(nav2_costmap_2d::makeFootprintFromRadius(0.3));
    }
    if (_persistent_obstacle) {
      persistent_layer_ = std::make_shared<PersistentObstacleLayer>();
      map_->addPlugin(persistent_layer_);
      persistent_layer_->initialize(
        map_.get(), "persistent", tf_.get(), node_,
        node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive));
    }
    layer_ = std::make_shared<srm27_nav_costmap::IntensityVoxelLayer>();
    map_->addPlugin(layer_);
    layer_->initialize(
      map_.get(), "terrain", tf_.get(), node_,
      node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive));
  }

  void observe(float _x, float _y)
  {
    sensor_msgs::msg::PointCloud2 cloud;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2Fields(
      4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
      sensor_msgs::msg::PointField::FLOAT32, "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(1);
    *sensor_msgs::PointCloud2Iterator<float>(cloud, "x") = _x;
    *sensor_msgs::PointCloud2Iterator<float>(cloud, "y") = _y;
    *sensor_msgs::PointCloud2Iterator<float>(cloud, "z") = 0.2;
    *sensor_msgs::PointCloud2Iterator<float>(cloud, "intensity") = 0.2;
    nav2_costmap_2d::Observation observation(cloud, 10.0, 0.0);
    layer_->addStaticObservation(observation, true, false);
  }

  unsigned char cost(double _x, double _y)
  {
    unsigned int x = 0, y = 0;
    EXPECT_TRUE(map_->getCostmap()->worldToMap(_x, _y, x, y));
    return map_->getCostmap()->getCost(x, y);
  }

  std::shared_ptr<nav2_util::LifecycleNode> node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::unique_ptr<nav2_costmap_2d::LayeredCostmap> map_;
  std::shared_ptr<PersistentObstacleLayer> persistent_layer_;
  std::shared_ptr<srm27_nav_costmap::IntensityVoxelLayer> layer_;
};
}  // namespace

TEST_F(IntensityVoxelLayerTest, DisappearedObservationClearsMasterCell)
{
  configure();
  observe(2.05, 2.05);
  map_->updateMap(0, 0, 0);
  ASSERT_EQ(cost(2.05, 2.05), nav2_costmap_2d::LETHAL_OBSTACLE);
  layer_->clearStaticObservations(true, false);
  map_->updateMap(0, 0, 0);
  unsigned int layer_x = 0, layer_y = 0;
  ASSERT_TRUE(layer_->worldToMap(2.05, 2.05, layer_x, layer_y));
  EXPECT_EQ(layer_->getCost(layer_x, layer_y), nav2_costmap_2d::FREE_SPACE);
  EXPECT_EQ(cost(2.05, 2.05), nav2_costmap_2d::FREE_SPACE);
}

TEST_F(IntensityVoxelLayerTest, NewObservationElsewhereDoesNotLeaveOldObstacle)
{
  configure();
  observe(2.05, 2.05);
  map_->updateMap(0, 0, 0);
  layer_->clearStaticObservations(true, false);
  observe(-2.05, -2.05);
  map_->updateMap(0, 0, 0);
  EXPECT_EQ(cost(2.05, 2.05), nav2_costmap_2d::FREE_SPACE);
  EXPECT_EQ(cost(-2.05, -2.05), nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST_F(IntensityVoxelLayerTest, RollingOriginStillClearsPreviousWorldLocation)
{
  configure(true);
  observe(2.05, 2.05);
  map_->updateMap(0, 0, 0);
  ASSERT_EQ(cost(2.05, 2.05), nav2_costmap_2d::LETHAL_OBSTACLE);
  layer_->clearStaticObservations(true, false);
  map_->updateMap(1.0, 0, 0);
  EXPECT_EQ(cost(2.05, 2.05), nav2_costmap_2d::FREE_SPACE);
}

TEST_F(IntensityVoxelLayerTest, DisablingLayerClearsItsPreviousBounds)
{
  configure();
  observe(2.05, 2.05);
  map_->updateMap(0, 0, 0);
  ASSERT_EQ(cost(2.05, 2.05), nav2_costmap_2d::LETHAL_OBSTACLE);
  ASSERT_TRUE(node_->set_parameter(rclcpp::Parameter("terrain.enabled", false)).successful);
  map_->updateMap(0, 0, 0);
  EXPECT_EQ(cost(2.05, 2.05), nav2_costmap_2d::FREE_SPACE);
}

TEST_F(IntensityVoxelLayerTest, DisappearedObservationDoesNotEraseOtherLayerObstacle)
{
  configure(false, true);
  observe(2.05, 2.05);
  map_->updateMap(0, 0, 0);
  ASSERT_EQ(cost(2.05, 2.05), nav2_costmap_2d::LETHAL_OBSTACLE);
  layer_->clearStaticObservations(true, false);
  map_->updateMap(0, 0, 0);
  EXPECT_EQ(cost(2.05, 2.05), nav2_costmap_2d::LETHAL_OBSTACLE);
}

// ---- hold_time（残影保持）：云台半视野自转时"看见一下又丢几秒"的补偿 ----

TEST_F(IntensityVoxelLayerTest, HoldTimeKeepsMarkAfterObservationStops)
{
  configure(false, false, 10.0);
  observe(2.05, 2.05);
  map_->updateMap(0, 0, 0);
  ASSERT_EQ(cost(2.05, 2.05), nav2_costmap_2d::LETHAL_OBSTACLE);
  // 视野转走：本周期没有任何观测，残影表必须把它留在图上。
  layer_->clearStaticObservations(true, false);
  map_->updateMap(0, 0, 0);
  EXPECT_EQ(cost(2.05, 2.05), nav2_costmap_2d::LETHAL_OBSTACLE);
  map_->updateMap(0, 0, 0);
  EXPECT_EQ(cost(2.05, 2.05), nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST_F(IntensityVoxelLayerTest, HoldTimeExpiresMarkAfterDeadline)
{
  configure(false, false, 0.15);
  observe(2.05, 2.05);
  map_->updateMap(0, 0, 0);
  ASSERT_EQ(cost(2.05, 2.05), nav2_costmap_2d::LETHAL_OBSTACLE);
  layer_->clearStaticObservations(true, false);
  std::this_thread::sleep_for(std::chrono::milliseconds(250));
  map_->updateMap(0, 0, 0);
  EXPECT_EQ(cost(2.05, 2.05), nav2_costmap_2d::FREE_SPACE);
}

TEST_F(IntensityVoxelLayerTest, HoldTimeSurvivesRollingWindowShift)
{
  configure(true, false, 10.0);
  observe(2.05, 2.05);
  map_->updateMap(0, 0, 0);
  ASSERT_EQ(cost(2.05, 2.05), nav2_costmap_2d::LETHAL_OBSTACLE);
  // 窗口向右滚动 1 m（10 格），残影键是世界格，障碍物必须还在原世界位置。
  layer_->clearStaticObservations(true, false);
  map_->updateMap(1.0, 0, 0);
  EXPECT_EQ(cost(2.05, 2.05), nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST_F(IntensityVoxelLayerTest, HoldTimeDoesNotRecordObstacleInsideFootprint)
{
  configure(false, false, 10.0, true);
  observe(0.25, 0.0);   // 车体 footprint（半径 0.3 m）内
  map_->updateMap(0, 0, 0);
  EXPECT_EQ(cost(0.25, 0.0), nav2_costmap_2d::FREE_SPACE);  // 每周期被 footprint 清掉

  // 机器人让开这个格子：残影表里没有它，所以不会变成跟车的幽灵障碍。
  layer_->clearStaticObservations(true, false);
  map_->updateMap(1.0, 0, 0);
  EXPECT_EQ(cost(0.25, 0.0), nav2_costmap_2d::FREE_SPACE);

  // 车体外的障碍照常进残影表：丢观测后仍保持。
  observe(2.05, 2.05);
  map_->updateMap(1.0, 0, 0);
  ASSERT_EQ(cost(2.05, 2.05), nav2_costmap_2d::LETHAL_OBSTACLE);
  layer_->clearStaticObservations(true, false);
  map_->updateMap(1.0, 0, 0);
  EXPECT_EQ(cost(2.05, 2.05), nav2_costmap_2d::LETHAL_OBSTACLE);
}
