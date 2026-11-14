cmake_minimum_required(VERSION 3.10)
project(tdt_nav_kit)

# 阶段二：YAstar 全局规划器 + MinimumSnap/SfcSquare 平滑器。
# 上游源码含未使用变量等告警，故本包**不启用**「告警即报错」选项
# （对比 srm27_nav_plugins/CMakeLists.txt:9 的写法）。
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(Eigen3 REQUIRED)
find_package(pluginlib REQUIRED)
# OSQP 由 osqp_vendor（ROS 包，apt 可装）提供；OsqpEigen 无 rosdep key，
# 需源码安装（默认前缀 /usr/local），见 README「阶段二依赖」。
find_package(osqp_vendor REQUIRED)
find_package(OsqpEigen REQUIRED)

if(NOT TARGET Eigen3::Eigen)
  message(FATAL_ERROR "tdt_nav_kit 需要 Eigen3::Eigen，未找到可用的 Eigen3 目标。")
endif()

# kinodynamicAstar.cpp 仍不编译：上游自述全局搜索约 2 s，主链路不用（见 PATCHES.md）。
ament_auto_add_library(${PROJECT_NAME} SHARED
  vendor/src/YAstar/yastar.cpp
  vendor/src/MinimumSnapOsqp/minimumSnap.cpp
  vendor/src/MinimumSnapOsqp/sfcSquare.cpp
  src/costmap_converter.cpp
  src/path_utils.cpp
  src/tdt_global_planner.cpp
  src/tdt_smoother.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/vendor/src/YAstar>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/vendor/src/MinimumSnapOsqp>"
  "$<INSTALL_INTERFACE:include>"
)
# ament_auto_add_library 内部通过 ament_target_dependencies() 使用了
# target_link_libraries 的 plain 签名，这里必须保持一致（不能写 PUBLIC/PRIVATE），
# 否则 CMake 报 "all uses ... must be either all-keyword or all-plain"。
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen OsqpEigen::OsqpEigen)
ament_export_dependencies(Eigen3)

# 上游头文件随包安装，便于阶段二接入 MinimumSnap / SfcSquare 与来源溯源。
# 注意：vendor/ 下的头只做安装，不通过 INSTALL_INTERFACE 导出，避免下游误依赖。
install(DIRECTORY vendor/src/ DESTINATION include/${PROJECT_NAME}/vendor)
# 带平滑的行为树：让 controller 拿到平滑后的稠密路径，
# 否则 YAstar 的稀疏路径会被 transformGlobalPlan 裁成单点（详见该文件头注释）。
install(DIRECTORY behavior_trees DESTINATION share/${PROJECT_NAME})
install(FILES
  vendor/LICENCE
  THIRD_PARTY_NOTICES.md
  PATCHES.md
  DESTINATION share/${PROJECT_NAME}
)

# 库名 tdt_nav_kit = XML 的 <library path>，由 CMake 注册到 ament resource index。
pluginlib_export_plugin_description_file(nav2_core tdt_planner_plugin.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_costmap_converter test/test_costmap_converter.cpp)
  target_link_libraries(test_costmap_converter ${PROJECT_NAME})

  ament_add_gtest(test_sfc_grid test/test_sfc_grid.cpp)
  target_link_libraries(test_sfc_grid ${PROJECT_NAME})

  ament_add_gtest(test_sfc_origin test/test_sfc_origin.cpp)
  target_link_libraries(test_sfc_origin ${PROJECT_NAME})

  ament_add_gtest(test_yastar_chain test/test_yastar_chain.cpp)
  target_link_libraries(test_yastar_chain ${PROJECT_NAME})

  ament_add_gtest(test_pluginlib_load test/test_pluginlib_load.cpp)
  target_link_libraries(test_pluginlib_load ${PROJECT_NAME})

  ament_add_gtest(test_path_utils test/test_path_utils.cpp)
  target_link_libraries(test_path_utils ${PROJECT_NAME})
endif()

# 本包不启用 ament_lint_auto：vendored 上游源码会触发格式/版权 lint 失败。
ament_auto_package(INSTALL_TO_SHARE config launch)
