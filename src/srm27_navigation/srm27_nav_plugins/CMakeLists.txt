cmake_minimum_required(VERSION 3.5)
project(srm27_nav_plugins)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(srm27_backup_free_space_behavior SHARED
  src/behaviors/back_up_free_space.cpp
)

ament_auto_add_library(srm27_costmap_layers SHARED
  src/layers/intensity_voxel_layer.cpp
)

# 全向底盘的到点判定：位置 + 平动停稳，角速度不参与（自转不属于导航）。
ament_auto_add_library(srm27_goal_checkers SHARED
  src/goal_checkers/omni_stopped_goal_checker.cpp
)

# 自研 BT 节点：把已经走过去的途径点从 navigate_through_poses 的目标列表里删掉。
# nav2 的 bt_navigator 是按 plugin_lib_names 里的库名 dlopen 这个 .so 让节点自注册的；
# BT_PLUGIN_EXPORT 必须定义，否则 BTCPP_EXPORT 展开成 static，符号不导出、加载时报找不到节点。
ament_auto_add_library(srm27_remove_passed_goals_bt_node SHARED
  src/bt/remove_passed_goals_by_route.cpp
)
target_compile_definitions(srm27_remove_passed_goals_bt_node PRIVATE BT_PLUGIN_EXPORT)

if(BUILD_TESTING)
  set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_SOURCE_DIR}/.clang-format")
  set(ament_cmake_clang_tidy_CONFIG_FILE "${CMAKE_SOURCE_DIR}/.clang-tidy")
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_intensity_voxel_layer test/test_intensity_voxel_layer.cpp)
  target_link_libraries(test_intensity_voxel_layer srm27_costmap_layers)
  set_tests_properties(test_intensity_voxel_layer PROPERTIES ENVIRONMENT
    "ROS_HOME=${CMAKE_BINARY_DIR}/ros_home;ROS_LOG_DIR=${CMAKE_BINARY_DIR}/ros_home/log")
  ament_add_gtest(test_omni_stopped_goal_checker test/test_omni_stopped_goal_checker.cpp)
  target_link_libraries(test_omni_stopped_goal_checker srm27_goal_checkers)
  # 这是会创建 ROS 节点的测试：把 ROS 日志/家目录指到构建目录，保证测试自包含，
  # 不依赖运行者的 ~/.ros 可写（与 srm27_minco_controller 的 test_state_adapter 同样做法）。
  set_tests_properties(test_omni_stopped_goal_checker PROPERTIES ENVIRONMENT
    "ROS_HOME=${CMAKE_BINARY_DIR}/ros_home;ROS_LOG_DIR=${CMAKE_BINARY_DIR}/ros_home/log")
  # 判据单测 + 3 个"八字被切掉"的实测场景回归；最后一条用例还会按库名加载
  # behavior_trees/navigate_through_poses_route_aware.xml 里用到的所有节点，等价于
  # bt_navigator 启动时的校验（不需要跑导航栈）。
  ament_add_gtest(test_remove_passed_goals_by_route
    test/test_remove_passed_goals_by_route.cpp)
  target_link_libraries(test_remove_passed_goals_by_route srm27_remove_passed_goals_bt_node)
  target_compile_definitions(test_remove_passed_goals_by_route PRIVATE
    SRM27_ROUTE_AWARE_BT_XML="${CMAKE_CURRENT_SOURCE_DIR}/../srm27_nav_bringup/behavior_trees/navigate_through_poses_route_aware.xml")
  set_tests_properties(test_remove_passed_goals_by_route PROPERTIES ENVIRONMENT
    "ROS_HOME=${CMAKE_BINARY_DIR}/ros_home;ROS_LOG_DIR=${CMAKE_BINARY_DIR}/ros_home/log")
endif()

pluginlib_export_plugin_description_file(nav2_core behavior_plugin.xml)
pluginlib_export_plugin_description_file(nav2_core goal_checker_plugins.xml)
pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_plugins.xml)

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
)
