cmake_minimum_required(VERSION 3.8)
project(srm27_nav_behavior_plugins)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(srm27_backup_free_space_behavior SHARED
  src/behaviors/back_up_free_space.cpp
)

ament_auto_add_library(srm27_goal_checkers SHARED
  src/goal_checkers/omni_stopped_goal_checker.cpp
)

ament_auto_add_library(srm27_remove_passed_goals_bt_node SHARED
  src/bt/remove_passed_goals_by_route.cpp
)
target_compile_definitions(srm27_remove_passed_goals_bt_node PRIVATE BT_PLUGIN_EXPORT)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_back_up_free_space test/test_back_up_free_space.cpp)
  target_link_libraries(test_back_up_free_space srm27_backup_free_space_behavior)
  set_tests_properties(test_back_up_free_space PROPERTIES ENVIRONMENT
    "ROS_HOME=${CMAKE_BINARY_DIR}/ros_home;ROS_LOG_DIR=${CMAKE_BINARY_DIR}/ros_home/log")

  ament_add_gtest(test_omni_stopped_goal_checker test/test_omni_stopped_goal_checker.cpp)
  target_link_libraries(test_omni_stopped_goal_checker srm27_goal_checkers)
  set_tests_properties(test_omni_stopped_goal_checker PROPERTIES ENVIRONMENT
    "ROS_HOME=${CMAKE_BINARY_DIR}/ros_home;ROS_LOG_DIR=${CMAKE_BINARY_DIR}/ros_home/log")

  ament_add_gtest(test_remove_passed_goals_by_route test/test_remove_passed_goals_by_route.cpp)
  target_link_libraries(test_remove_passed_goals_by_route srm27_remove_passed_goals_bt_node)
  target_compile_definitions(test_remove_passed_goals_by_route PRIVATE
    SRM27_ROUTE_AWARE_BT_XML="${CMAKE_CURRENT_SOURCE_DIR}/test/route_aware_nodes.xml")
  set_tests_properties(test_remove_passed_goals_by_route PROPERTIES ENVIRONMENT
    "ROS_HOME=${CMAKE_BINARY_DIR}/ros_home;ROS_LOG_DIR=${CMAKE_BINARY_DIR}/ros_home/log")
endif()

pluginlib_export_plugin_description_file(nav2_core behavior_plugin.xml)
pluginlib_export_plugin_description_file(nav2_core goal_checker_plugins.xml)
install(FILES LICENSE NOTICE DESTINATION share/${PROJECT_NAME})

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
