// Copyright 2026 SRM27 Navigation Team
// Licensed under the Apache License, Version 2.0.
//
// 验证平滑器插件确实通过 pluginlib 导出并可被 Nav2 加载。
// 只加载类、不调用 configure()（那需要真实的 TF / costmap）。

#include <gtest/gtest.h>

#include <algorithm>
#include <memory>
#include <nav2_core/smoother.hpp>
#include <pluginlib/class_loader.hpp>
#include <string>
#include <vector>

namespace
{
constexpr const char * kPackage = "nav2_core";
constexpr const char * kSmootherPlugin = "srm27_nav_smoother::MinimumSnapSmoother";

template<typename BaseT>
bool declared(pluginlib::ClassLoader<BaseT> & loader, const std::string & name)
{
  const std::vector<std::string> classes = loader.getDeclaredClasses();
  return std::find(classes.begin(), classes.end(), name) != classes.end();
}
}  // namespace

TEST(MinimumSnapSmootherPluginTest, DeclaredAsNav2Smoother)
{
  pluginlib::ClassLoader<nav2_core::Smoother> loader(kPackage, "nav2_core::Smoother");
  EXPECT_TRUE(declared(loader, kSmootherPlugin))
    << "平滑器未在 nav2_core 描述文件中声明；已声明的类共 "
    << loader.getDeclaredClasses().size() << " 个";
}

TEST(MinimumSnapSmootherPluginTest, CanBeInstantiated)
{
  pluginlib::ClassLoader<nav2_core::Smoother> loader(kPackage, "nav2_core::Smoother");
  std::shared_ptr<nav2_core::Smoother> instance;
  ASSERT_NO_THROW(instance = loader.createSharedInstance(kSmootherPlugin));
  ASSERT_TRUE(instance != nullptr);
}

TEST(MinimumSnapSmootherPluginTest, PluginXmlIsInstalled)
{
  pluginlib::ClassLoader<nav2_core::Smoother> loader(kPackage, "nav2_core::Smoother");
  const std::string manifest = loader.getPluginManifestPath(kSmootherPlugin);
  EXPECT_FALSE(manifest.empty());
  EXPECT_NE(manifest.find("srm27_nav_smoother_plugins.xml"), std::string::npos)
    << "插件描述文件路径异常: " << manifest;
  EXPECT_NE(manifest.find("srm27_nav_smoother"), std::string::npos)
    << "插件描述文件不属于 srm27_nav_smoother 包: " << manifest;
}
