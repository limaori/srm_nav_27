// Copyright 2026 SRM
//
// 插件导出冒烟测试：只验证 pluginlib 能按名字把插件类构造出来。
// 真实行为由仿真端到端验证覆盖（需要 TF / costmap / 距离场）。

#include <gtest/gtest.h>

#include <memory>
#include <string>

#include "pluginlib/class_loader.hpp"
#include "nav2_core/global_planner.hpp"

TEST(MincoPlannerPluginlib, LoadsByClassName)
{
  pluginlib::ClassLoader<nav2_core::GlobalPlanner> loader(
    "nav2_core", "nav2_core::GlobalPlanner");
  std::shared_ptr<nav2_core::GlobalPlanner> planner;
  ASSERT_NO_THROW(planner = loader.createSharedInstance("srm27_minco_planner::MincoPlanner"));
  EXPECT_NE(planner, nullptr);
}
