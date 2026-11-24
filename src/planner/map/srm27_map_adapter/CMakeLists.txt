cmake_minimum_required(VERSION 3.10)
project(srm27_map_adapter)

# 地图适配层：Nav2 Costmap2D/ROS → srm27_map_core::GridSnapshot。
# 原 nav_common 的 costmap 转换与 controller 的 CostmapAdapter 合并到这里，
# 规划器、平滑器、controller 共用同一份加锁复制 + 内容版本逻辑。
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(srm27_map_core REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/costmap_adapter.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
ament_export_dependencies(srm27_map_core)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_costmap_adapter test/test_costmap_adapter.cpp)
  target_link_libraries(test_costmap_adapter ${PROJECT_NAME})
endif()

ament_auto_package()
