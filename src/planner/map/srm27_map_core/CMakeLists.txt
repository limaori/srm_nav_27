cmake_minimum_required(VERSION 3.10)
project(srm27_map_core)

# 规划地图底座（纯 C++，不依赖 ROS / Nav2）：
#   GridSnapshot（栅格语义/几何/编码）、ESDF、占据几何、SFC 栅格视图。
# 规划前端（YAstar）、后端（MINCO/ESDF、MinimumSnap/SFC）与 controller 共用。
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(Eigen3 REQUIRED)

if(NOT TARGET Eigen3::Eigen)
  message(FATAL_ERROR "srm27_map_core 需要 Eigen3::Eigen，未找到可用的 Eigen3 目标。")
endif()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/grid_snapshot.cpp
  src/esdf_2d.cpp
  src/occupancy_geometry.cpp
  src/sfc_grid.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
ament_export_dependencies(Eigen3)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_grid_snapshot test/test_grid_snapshot.cpp)
  target_link_libraries(test_grid_snapshot ${PROJECT_NAME})

  ament_add_gtest(test_esdf_2d test/test_esdf_2d.cpp)
  target_link_libraries(test_esdf_2d ${PROJECT_NAME})

  ament_add_gtest(test_occupancy_geometry test/test_occupancy_geometry.cpp)
  target_link_libraries(test_occupancy_geometry ${PROJECT_NAME})

  ament_add_gtest(test_sfc_grid test/test_sfc_grid.cpp)
  target_link_libraries(test_sfc_grid ${PROJECT_NAME})
endif()

ament_auto_package()
