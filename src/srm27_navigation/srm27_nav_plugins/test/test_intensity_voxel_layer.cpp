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

#include <memory>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/observation.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "srm27_nav_plugins/layers/intensity_voxel_layer.hpp"

namespace
{
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
  void configure(bool _rolling = false)
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides(
      {rclcpp::Parameter("terrain.footprint_clearing_enabled", false),
       rclcpp::Parameter("terrain.origin_z", 0.0)});
    node_ = std::make_shared<nav2_util::LifecycleNode>("intensity_layer_test", "", options);
    tf_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    map_ = std::make_unique<nav2_costmap_2d::LayeredCostmap>("map", _rolling, false);
    map_->resizeMap(100, 100, 0.1, -5.0, -5.0);
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
