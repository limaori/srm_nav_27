cmake_minimum_required(VERSION 3.16)
project(srm27_behavior_core)

# 决策模块的共享底座：无 ROS 节点、无 BehaviorTree 插件注册，
# 只保留服务端与 BT 插件都要用的状态/算法（header-only）。
#   服务端 srm27_behavior ──► 本包
#   BT 插件 srm27_behavior_plugins ──► 本包
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_decision_core test/test_decision_core.cpp)
  target_include_directories(test_decision_core PRIVATE include)
  ament_target_dependencies(
    test_decision_core
    ament_index_cpp behaviortree_cpp geometry_msgs rclcpp tf2 tf2_geometry_msgs)
  target_link_libraries(test_decision_core yaml-cpp)
  set_tests_properties(test_decision_core PROPERTIES ENVIRONMENT
    "ROS_HOME=${CMAKE_BINARY_DIR}/ros_home;ROS_LOG_DIR=${CMAKE_BINARY_DIR}/ros_home/log")
endif()

# header-only：ament_auto_package 负责安装 include/ 并导出 include 与依赖。
ament_auto_package()
