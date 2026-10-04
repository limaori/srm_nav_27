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
# 速度合成、超时判定和自转波形都不依赖 ROS，便于单元测试。
add_library(srm_chassis_control_core
  src/velocity_mix.cpp
  src/twist_watchdog.cpp
  src/rotation_waveform.cpp
)
target_include_directories(srm_chassis_control_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ---------------------------------------------------------------- 节点
foreach(node_name cmd_mux rotation_controller rotation_test_sender)
  if(node_name STREQUAL "cmd_mux")
    set(node_source src/cmd_mux_node.cpp)
  elseif(node_name STREQUAL "rotation_controller")
    set(node_source src/rotation_controller_node.cpp)
  else()
    set(node_source src/rotation_test_sender_node.cpp)
  endif()
  add_executable(${node_name} ${node_source})
  target_link_libraries(${node_name}
    srm_chassis_control_core
    rclcpp::rclcpp
    ${geometry_msgs_TARGETS}
    ${diagnostic_msgs_TARGETS}
    ${std_srvs_TARGETS}
    ${rcl_interfaces_TARGETS}
  )
  install(TARGETS ${node_name} DESTINATION lib/${PROJECT_NAME})
endforeach()

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

  ament_add_gtest(test_rotation_and_watchdog test/test_rotation_and_watchdog.cpp)
  target_link_libraries(test_rotation_and_watchdog srm_chassis_control_core)

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
