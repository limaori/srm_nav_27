cmake_minimum_required(VERSION 3.10)
project(srm27_nav_smoother)

# 轨迹规划后端：MinimumSnap（OSQP 求解）+ SfcSquare 方形走廊的 Nav2 平滑器插件。
# self-contained：上游 MinimumSnap/SfcSquare 源码就在本包 vendor/ 下（MIT），
# 包装层为本工作区 Apache-2.0。
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
# OSQP 由 osqp_vendor（ROS 包）提供；OsqpEigen 没有 rosdep key，
# 需源码安装（见 README「依赖」），所以这里直接 find_package。
find_package(osqp_vendor REQUIRED)
find_package(OsqpEigen REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
  vendor/src/MinimumSnapOsqp/minimumSnap.cpp
  vendor/src/MinimumSnapOsqp/sfcSquare.cpp
  src/minimum_snap_smoother.cpp
  src/sfc_visualization.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
# vendored 头只给本包（源码与测试）用；不导出给下游。
target_include_directories(${PROJECT_NAME} PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/vendor/src/MinimumSnapOsqp"
)
# ament_auto_add_library 已按 package.xml 链接 srm27_nav_common 等依赖；
# Eigen 由 srm27_nav_common 的 export 传递，这里保持显式。
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen OsqpEigen::OsqpEigen)
ament_export_dependencies(Eigen3 srm27_nav_common srm27_map_core srm27_map_adapter)

pluginlib_export_plugin_description_file(nav2_core srm27_nav_smoother_plugins.xml)

# 上游头文件随包安装，便于溯源；不放进公共 include 搜索路径。
install(DIRECTORY vendor/src/MinimumSnapOsqp DESTINATION include/${PROJECT_NAME}/vendor)
install(FILES vendor/LICENCE THIRD_PARTY_NOTICES.md PATCHES.md
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_sfc_markers test/test_sfc_markers.cpp)
  target_link_libraries(test_sfc_markers ${PROJECT_NAME})

  ament_add_gtest(test_pluginlib_load test/test_pluginlib_load.cpp)
  target_link_libraries(test_pluginlib_load ${PROJECT_NAME})
  set_tests_properties(test_pluginlib_load PROPERTIES ENVIRONMENT
    "ROS_HOME=${CMAKE_BINARY_DIR}/ros_home;ROS_LOG_DIR=${CMAKE_BINARY_DIR}/ros_home/log")

  # vendored SfcSquare/MinimumSnap 的纯算法用例（P7/P10 原点补丁）。
  ament_add_gtest(test_sfc_origin test/test_sfc_origin.cpp)
  target_include_directories(test_sfc_origin PRIVATE vendor/src/MinimumSnapOsqp)
  target_link_libraries(test_sfc_origin ${PROJECT_NAME})

  ament_add_gtest(test_sfc_bound_frame test/test_sfc_bound_frame.cpp)
  target_include_directories(test_sfc_bound_frame PRIVATE vendor/src/MinimumSnapOsqp)
  target_link_libraries(test_sfc_bound_frame ${PROJECT_NAME})
endif()

# 不启用 ament_lint_auto：vendored 上游源码会触发格式/版权 lint。
ament_auto_package()
