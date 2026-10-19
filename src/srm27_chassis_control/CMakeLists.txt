cmake_minimum_required(VERSION 3.8)
project(srm27_chassis_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rcl_interfaces REQUIRED)

# ---------------------------------------------------------------- 纯逻辑核心
# 速度限幅与超时判定都不依赖 ROS，便于单元测试。
add_library(srm_chassis_control_core
  src/velocity_mix.cpp
  src/twist_watchdog.cpp
)
target_include_directories(srm_chassis_control_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ---------------------------------------------------------------- 节点
# 独立自转链（rotation_controller / rotation_test_sender）已于 2026-10-19 删除：
# wz 只由上位机（导航栈）产生，仿真链上只剩 srm_cmd_mux 这一个出口节点。
add_executable(cmd_mux src/cmd_mux_node.cpp)
target_link_libraries(cmd_mux
  srm_chassis_control_core
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${diagnostic_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${rcl_interfaces_TARGETS}
)
install(TARGETS cmd_mux DESTINATION lib/${PROJECT_NAME})

install(TARGETS srm_chassis_control_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_velocity_mix test/test_velocity_mix.cpp)
  target_link_libraries(test_velocity_mix srm_chassis_control_core)

  ament_add_gtest(test_twist_watchdog test/test_twist_watchdog.cpp)
  target_link_libraries(test_twist_watchdog srm_chassis_control_core)

  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_cpplint
    ament_cmake_flake8
    ament_cmake_pep257
  )
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(srm_chassis_control_core)

ament_package()
