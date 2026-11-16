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
#include <nav2_core/controller.hpp>
#include <pluginlib/class_loader.hpp>
#include <string>

/// \file
/// \brief 验证 MINCO + MPC 插件确实通过 pluginlib 导出并可被 Nav2 加载。
///
/// 只加载插件类并检查其类型，不调用 `configure()`（那需要真实的 TF/costmap）。
/// 这一步用来防止“只改了 YAML 而漏装插件 XML”的典型失败（方案 §4.3、§4.5.4）。

TEST(MincoMpcPluginTest, DeclaredAsNav2ControllerPlugin)
{
  pluginlib::ClassLoader<nav2_core::Controller> loader("nav2_core", "nav2_core::Controller");
  const std::vector<std::string> classes = loader.getDeclaredClasses();
  const std::string expected = "srm27_minco_controller::MincoMpcController";
  EXPECT_NE(std::find(classes.begin(), classes.end(), expected), classes.end())
    << "插件未在 nav2_core 插件描述文件中声明；已声明的类有 " << classes.size() << " 个";
}

TEST(MincoMpcPluginTest, CanBeInstantiated)
{
  pluginlib::ClassLoader<nav2_core::Controller> loader("nav2_core", "nav2_core::Controller");
  std::shared_ptr<nav2_core::Controller> instance;
  ASSERT_NO_THROW(
    instance = loader.createSharedInstance("srm27_minco_controller::MincoMpcController"));
  ASSERT_TRUE(instance != nullptr);
  // 能创建即说明共享库与导出符号正确；未 configure 前不允许直接使用。
}

TEST(MincoMpcPluginTest, PluginXmlIsInstalledNextToPackage)
{
  pluginlib::ClassLoader<nav2_core::Controller> loader("nav2_core", "nav2_core::Controller");
  const std::string manifest =
    loader.getPluginManifestPath("srm27_minco_controller::MincoMpcController");
  EXPECT_FALSE(manifest.empty());
  EXPECT_NE(manifest.find("srm27_minco_controller.xml"), std::string::npos)
    << "插件描述文件路径异常: " << manifest;
}
