cmake_minimum_required(VERSION 3.10)
project(srm27_nav_planner)

# 轨迹规划前端：YAstar 全局规划器（Nav2 GlobalPlanner 插件）。
# self-contained：上游 YAstar 源码就在本包 vendor/ 下（MIT），包装层为本工作区
# Apache-2.0；与 srm27_nav_common（路径工具）和地图包共享代码。
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(Eigen3 REQUIRED)
find_package(pluginlib REQUIRED)
find_package(srm27_nav_common REQUIRED)
find_package(srm27_map_core REQUIRED)
find_package(srm27_map_adapter REQUIRED)

# kinodynamicAstar.cpp 不编译：上游自述全局搜索约 2 s，主链路不用（见 PATCHES.md）。
ament_auto_add_library(${PROJECT_NAME} SHARED
  vendor/src/YAstar/yastar.cpp
  src/goal_resolver.cpp
  src/yastar_global_planner.cpp
  src/yastar_path.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
# vendored 头只给本包（源码与测试）用；不导出给下游，避免别人依赖上游内部类型。
target_include_directories(${PROJECT_NAME} PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/vendor/src/YAstar"
)
# ament_auto_add_library 已按 package.xml 链接 srm27_nav_common 等依赖；
# Eigen 由 srm27_nav_common 的 export 传递，这里保持显式以匹配上游告警面。
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
ament_export_dependencies(Eigen3 srm27_nav_common srm27_map_core srm27_map_adapter)

# 库名 = XML 的 <library path>，由 CMake 注册到 ament resource index。
pluginlib_export_plugin_description_file(nav2_core srm27_nav_planner_plugins.xml)

# 上游头文件随包安装，便于溯源；不放进公共 include 搜索路径。
install(DIRECTORY vendor/src/YAstar DESTINATION include/${PROJECT_NAME}/vendor)
install(FILES vendor/LICENCE THIRD_PARTY_NOTICES.md PATCHES.md
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_pluginlib_load test/test_pluginlib_load.cpp)
  target_link_libraries(test_pluginlib_load ${PROJECT_NAME})
  set_tests_properties(test_pluginlib_load PROPERTIES ENVIRONMENT
    "ROS_HOME=${CMAKE_BINARY_DIR}/ros_home;ROS_LOG_DIR=${CMAKE_BINARY_DIR}/ros_home/log")

  ament_add_gtest(test_planner_helpers test/test_planner_helpers.cpp)
  # 用例里直接用 YAstar 跑一遍「搜索+简化」，需要 vendored 头。
  target_include_directories(test_planner_helpers PRIVATE vendor/src/YAstar)
  target_link_libraries(test_planner_helpers ${PROJECT_NAME})

  # vendored YAstar 的纯算法用例（SDF 一致性与路径可复现）。
  ament_add_gtest(test_yastar_chain test/test_yastar_chain.cpp)
  target_include_directories(test_yastar_chain PRIVATE vendor/src/YAstar)
  target_link_libraries(test_yastar_chain ${PROJECT_NAME})
endif()

# 不启用 ament_lint_auto：vendored 上游源码与出处注释会触发格式/版权 lint。
ament_auto_package()
