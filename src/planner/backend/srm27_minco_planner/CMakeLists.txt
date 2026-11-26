cmake_minimum_required(VERSION 3.10)
project(srm27_minco_planner)

# MINCO 规划器：Nav2 GlobalPlanner 插件。
#
# 结构对齐 navi_minco_bit 的 minco_planner：createPlan() 只做全局搜索并记录目标，
# 真正的 MINCO 在内部线程按 opt_freq 连续重规划，结果通过 MincoTrajectory 话题
# 直接发给控制器（控制器退化为纯 MPC）。2026-11-26 从 srm27_minco_controller 迁出。

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/minco_planner.cpp
)

pluginlib_export_plugin_description_file(nav2_core srm27_minco_planner.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_pluginlib_load test/test_pluginlib_load.cpp)
  target_link_libraries(test_pluginlib_load ${PROJECT_NAME})
endif()

ament_auto_package()
