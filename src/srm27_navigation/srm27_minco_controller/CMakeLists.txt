cmake_minimum_required(VERSION 3.10)
project(srm27_minco_controller)

# 与相邻 Nav2 controller 包一致的 C++ 标准与格式/静态检查配置。
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/costmap_adapter.cpp
  src/diagnostics.cpp
  src/minco_mpc_controller.cpp
  src/planning_worker.cpp
  src/state_adapter.cpp
  src/yaw_policy.cpp
)

# Nav2 controller_server 会在插件生命周期回调里调用这些成员；必须导出共享库并
# 通过 pluginlib 描述文件声明，只改 YAML 而漏装 XML 会加载失败（方案 §4.3）。
pluginlib_export_plugin_description_file(nav2_core srm27_minco_controller.xml)

if(BUILD_TESTING)
  set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/.clang-format")
  set(ament_cmake_clang_tidy_CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/.clang-tidy")
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  # 只做不依赖 TF/costmap 的接口与行为测试；pluginlib 用例验证插件确实被导出。
  ament_add_gtest(test_pluginlib_loading test/test_pluginlib_loading.cpp)
  target_link_libraries(test_pluginlib_loading ${PROJECT_NAME})
  ament_add_gtest(test_yaw_policy test/test_yaw_policy.cpp)
  target_link_libraries(test_yaw_policy ${PROJECT_NAME})
  ament_add_gtest(test_diagnostics test/test_diagnostics.cpp)
  target_link_libraries(test_diagnostics ${PROJECT_NAME})
  # 状态适配层需要真实节点/话题/TF，属于集成测试但仍不依赖 Gazebo。
  ament_add_gtest(test_state_adapter test/test_state_adapter.cpp)
  target_link_libraries(test_state_adapter ${PROJECT_NAME})
  ament_target_dependencies(test_state_adapter rclcpp rclcpp_lifecycle nav_msgs tf2 tf2_geometry_msgs)
  # 这是唯一会创建 ROS 节点的测试：把 ROS 日志/家目录指到构建目录，保证测试自包含，
  # 不依赖运行者的 ~/.ros 是否可写。
  if(TARGET test_state_adapter)
    set_tests_properties(
      test_state_adapter
      PROPERTIES ENVIRONMENT
                 "ROS_HOME=${CMAKE_BINARY_DIR}/ros_home;ROS_LOG_DIR=${CMAKE_BINARY_DIR}/ros_home/log"
    )
  endif()
endif()

install(DIRECTORY include/ DESTINATION include)

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
