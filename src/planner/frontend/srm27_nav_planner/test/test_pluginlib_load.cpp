// Copyright 2026 SRM27 Navigation Team
// Licensed under the Apache License, Version 2.0.
//
// 验证全局规划器插件确实通过 pluginlib 导出并可被 Nav2 加载。
// 只加载类、不调用 configure()（那需要真实的 TF / costmap）。
// 范式参照 srm27_minco_controller/test/test_pluginlib_loading.cpp。

#include <gtest/gtest.h>

#include <algorithm>
#include <memory>
#include <nav2_core/global_planner.hpp>
#include <pluginlib/class_loader.hpp>
#include <string>
#include <vector>

namespace
{
constexpr const char * kPackage = "nav2_core";
constexpr const char * kPlannerPlugin = "srm27_nav_planner::YAstarGlobalPlanner";

template<typename BaseT>
bool declared(pluginlib::ClassLoader<BaseT> & loader, const std::string & name)
{
  const std::vector<std::string> classes = loader.getDeclaredClasses();
  return std::find(classes.begin(), classes.end(), name) != classes.end();
}
}  // namespace

TEST(YAstarPlannerPluginTest, DeclaredAsNav2GlobalPlanner)
{
  pluginlib::ClassLoader<nav2_core::GlobalPlanner> loader(kPackage, "nav2_core::GlobalPlanner");
  EXPECT_TRUE(declared(loader, kPlannerPlugin))
    << "插件未在 nav2_core 描述文件中声明；已声明的类共 "
    << loader.getDeclaredClasses().size() << " 个";
}

TEST(YAstarPlannerPluginTest, CanBeInstantiated)
{
  pluginlib::ClassLoader<nav2_core::GlobalPlanner> loader(kPackage, "nav2_core::GlobalPlanner");
  std::shared_ptr<nav2_core::GlobalPlanner> instance;
  ASSERT_NO_THROW(instance = loader.createSharedInstance(kPlannerPlugin));
  ASSERT_TRUE(instance != nullptr);
  // 能创建即说明共享库与 PLUGINLIB_EXPORT_CLASS 导出符号正确；未 configure 前不可直接使用。
}

TEST(YAstarPlannerPluginTest, PluginXmlIsInstalled)
{
  pluginlib::ClassLoader<nav2_core::GlobalPlanner> loader(kPackage, "nav2_core::GlobalPlanner");
  const std::string manifest = loader.getPluginManifestPath(kPlannerPlugin);
  EXPECT_FALSE(manifest.empty());
  EXPECT_NE(manifest.find("srm27_nav_planner_plugins.xml"), std::string::npos)
    << "插件描述文件路径异常: " << manifest;
  EXPECT_NE(manifest.find("srm27_nav_planner"), std::string::npos)
    << "插件描述文件不属于 srm27_nav_planner 包: " << manifest;
}
