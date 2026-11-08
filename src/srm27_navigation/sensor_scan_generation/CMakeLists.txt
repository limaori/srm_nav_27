cmake_minimum_required(VERSION 3.10)
project(sensor_scan_generation)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

###########
## Build ##
###########

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN sensor_scan_generation::SensorScanGenerationNode
  EXECUTABLE ${PROJECT_NAME}_node
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  set(ament_cmake_clang_format_CLANG_FORMAT_VERSION 21)
  set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_SOURCE_DIR}/.clang-format")
  set(ament_cmake_clang_tidy_CONFIG_FILE "${CMAKE_SOURCE_DIR}/.clang-tidy")

  # 方案 §4.1.1 的回归：关节角 TF 缺失时 /odometry 仍按点云频率输出。
  find_package(ament_cmake_gtest REQUIRED)
  find_package(tf2_msgs REQUIRED)
  ament_add_gtest(test_odometry_without_joint_tf
    test/test_odometry_without_joint_tf.cpp
  )
  if(TARGET test_odometry_without_joint_tf)
    target_link_libraries(test_odometry_without_joint_tf
      ${PROJECT_NAME}
      ${tf2_msgs_TARGETS}
    )
  endif()

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

#############
## Install ##
#############

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  launch
)
