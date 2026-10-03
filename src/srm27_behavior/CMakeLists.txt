cmake_minimum_required(VERSION 3.16)
project(srm27_behavior)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

###########
## Build ##
###########

ament_auto_add_executable(srm27_behavior_server src/srm27_behavior_server.cpp)

ament_auto_add_library(srm27_behavior_action_client SHARED
  src/srm27_behavior_client.cpp
)

rclcpp_components_register_node(srm27_behavior_action_client
  PLUGIN srm27_behavior::SentryBehaviorClient
  EXECUTABLE srm27_behavior_client
)

####### Conditions #######
ament_auto_add_library(is_game_status SHARED plugins/condition/is_game_status.cpp)
list(APPEND plugin_libs is_game_status)

ament_auto_add_library(is_rfid_detected SHARED plugins/condition/is_rfid_detected.cpp)
list(APPEND plugin_libs is_rfid_detected)

ament_auto_add_library(is_status_ok SHARED plugins/condition/is_status_ok.cpp)
list(APPEND plugin_libs is_status_ok)

ament_auto_add_library(is_attacked SHARED plugins/condition/is_attacked.cpp)
list(APPEND plugin_libs is_attacked)

####### Actions #######
ament_auto_add_library(send_nav2_goal SHARED plugins/action/send_nav2_goal.cpp)
list(APPEND plugin_libs send_nav2_goal)

ament_auto_add_library(pub_nav2_goal SHARED plugins/action/pub_nav2_goal.cpp)
list(APPEND plugin_libs pub_nav2_goal)

ament_auto_add_library(pub_spin_speed SHARED plugins/action/pub_spin_speed.cpp)
list(APPEND plugin_libs pub_spin_speed)

ament_auto_add_library(pub_twist SHARED plugins/action/pub_twist.cpp)
list(APPEND plugin_libs pub_twist)

ament_auto_add_library(pub_gimbal_absolute SHARED plugins/action/pub_gimbal_absolute.cpp)
list(APPEND plugin_libs pub_gimbal_absolute)

ament_auto_add_library(pub_gimbal_velocity SHARED plugins/action/pub_gimbal_velocity.cpp)
list(APPEND plugin_libs pub_gimbal_velocity)

####### Controls #######
ament_auto_add_library(recovery_node SHARED plugins/control/recovery_node.cpp)
list(APPEND plugin_libs recovery_node)

####### Decorators #######
ament_auto_add_library(rate_controller SHARED plugins/decorator/rate_controller.cpp)
list(APPEND plugin_libs rate_controller)

ament_auto_add_library(tick_after_timeout_node SHARED plugins/decorator/tick_after_timeout_node.cpp)
list(APPEND plugin_libs tick_after_timeout_node)

foreach(bt_plugin ${plugin_libs})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

#############
## Testing ##
#############

if(BUILD_TESTING)
  set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_SOURCE_DIR}/.clang-format")
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify
    ament_cmake_flake8
  )
  ament_lint_auto_find_test_dependencies()
endif()

#############
## Install ##
#############

install(
  TARGETS ${plugin_libs}
  LIBRARY DESTINATION share/${PROJECT_NAME}/bt_plugins
  ARCHIVE DESTINATION share/${PROJECT_NAME}/bt_plugins
  RUNTIME DESTINATION share/${PROJECT_NAME}/bt_plugins
)

ament_auto_package(
  INSTALL_TO_SHARE
  behavior_trees
  params
  launch
)
