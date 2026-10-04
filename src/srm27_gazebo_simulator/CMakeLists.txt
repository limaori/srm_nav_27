cmake_minimum_required(VERSION 3.8)
project(srm27_gazebo_simulator)

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

# Gazebo Fortress 对应的版本：ignition-gazebo6 / ignition-transport11 / ignition-msgs8
find_package(ignition-cmake2 REQUIRED)
find_package(ignition-gazebo6 REQUIRED)
set(IGN_GAZEBO_VER 6)
find_package(ignition-transport11 REQUIRED)
set(IGN_TRANSPORT_VER 11)
find_package(ignition-msgs8 REQUIRED)
set(IGN_MSGS_VER 8)

# ---------------------------------------------------------------- 底盘速度插件
# SRM 自有的速度执行插件，替换旧步兵的 MecanumDrive2 与 rmoss_gz_base 底盘控制器。
add_library(srm_velocity_system SHARED
  plugins/srm_velocity_system/SrmVelocitySystem.cc
)
target_include_directories(srm_velocity_system PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/plugins/srm_velocity_system
)
target_link_libraries(srm_velocity_system
  ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER}
)

# ---------------------------------------------------------------- 速度适配器
# 订阅 ROS 侧 cmd_vel_sim，校验后转发到 Gazebo Transport，并做单调时钟超时清零。
add_executable(srm_velocity_adapter src/srm_velocity_adapter_node.cpp)
target_link_libraries(srm_velocity_adapter
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${diagnostic_msgs_TARGETS}
  ignition-transport${IGN_TRANSPORT_VER}::core
  ignition-msgs${IGN_MSGS_VER}::ignition-msgs${IGN_MSGS_VER}
)

install(TARGETS srm_velocity_system srm_velocity_adapter
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/srm_velocity_system.dsv.in")

install(DIRECTORY
  config
  launch
  worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_cpplint
    ament_cmake_uncrustify
    ament_cmake_flake8
    ament_cmake_pep257
  )
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
