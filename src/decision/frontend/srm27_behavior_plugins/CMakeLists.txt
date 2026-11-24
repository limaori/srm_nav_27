cmake_minimum_required(VERSION 3.16)
project(srm27_behavior_plugins)

# 决策前端：BehaviorTree.CPP 4.x 插件（动作/条件/控制/装饰器）。
# 只依赖共享底座 srm27_behavior_core，不依赖决策服务端；服务端在运行时
# 从 share/srm27_behavior_plugins/bt_plugins 加载这里的 .so。
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_definitions(-Wall -Werror)

find_package(ament_cmake_auto REQUIRED)
find_package(yaml-cpp REQUIRED)
ament_auto_find_build_dependencies()

# 与拆包前保持一致：每个插件单独一个共享库。
# 原因：behaviortree_ros2 的 ros_node_params.hpp 里 BT_RegisterRosNodeFromPlugin()
# 与 custom_types.hpp 的 convertFromString 特化都是非 inline 定义，把多个插件
# 合进同一个 .so 会报 multiple definition；按插件分库也便于单独替换/排查。
set(SRM27_BEHAVIOR_ACTION_PLUGINS
  calculate_attack_pose
  mux_stop_control
  nav_through_poses
  pub_gimbal_absolute
  pub_gimbal_velocity
  pub_nav2_goal
  pub_rotation_cmd
  pub_spin_speed
  pub_twist
  send_nav2_goal
  set_sentry_posture
)
set(SRM27_BEHAVIOR_CONDITION_PLUGINS
  hysteresis_check
  is_attacked
  is_game_status
  is_hp_dropping
  is_in_semantic_zone
  is_rfid_detected
  is_status_ok
)
set(SRM27_BEHAVIOR_CONTROL_PLUGINS
  recovery_node
)
set(SRM27_BEHAVIOR_DECORATOR_PLUGINS
  navigation_transition_gate
  rate_controller
  tick_after_timeout_node
)

set(SRM27_BEHAVIOR_PLUGIN_LIBS "")
foreach(plugin_name ${SRM27_BEHAVIOR_ACTION_PLUGINS})
  ament_auto_add_library(${plugin_name} SHARED plugins/action/${plugin_name}.cpp)
  target_link_libraries(${plugin_name} yaml-cpp)
  list(APPEND SRM27_BEHAVIOR_PLUGIN_LIBS ${plugin_name})
endforeach()
foreach(plugin_name ${SRM27_BEHAVIOR_CONDITION_PLUGINS})
  ament_auto_add_library(${plugin_name} SHARED plugins/condition/${plugin_name}.cpp)
  target_link_libraries(${plugin_name} yaml-cpp)
  list(APPEND SRM27_BEHAVIOR_PLUGIN_LIBS ${plugin_name})
endforeach()
foreach(plugin_name ${SRM27_BEHAVIOR_CONTROL_PLUGINS})
  ament_auto_add_library(${plugin_name} SHARED plugins/control/${plugin_name}.cpp)
  list(APPEND SRM27_BEHAVIOR_PLUGIN_LIBS ${plugin_name})
endforeach()
foreach(plugin_name ${SRM27_BEHAVIOR_DECORATOR_PLUGINS})
  ament_auto_add_library(${plugin_name} SHARED plugins/decorator/${plugin_name}.cpp)
  list(APPEND SRM27_BEHAVIOR_PLUGIN_LIBS ${plugin_name})
endforeach()

foreach(plugin_lib ${SRM27_BEHAVIOR_PLUGIN_LIBS})
  target_compile_definitions(${plugin_lib} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

# 决策服务端的 plugins 参数按 share/<包>/bt_plugins 目录枚举并 dlopen。
install(
  TARGETS ${SRM27_BEHAVIOR_PLUGIN_LIBS}
  LIBRARY DESTINATION share/${PROJECT_NAME}/bt_plugins
  ARCHIVE DESTINATION share/${PROJECT_NAME}/bt_plugins
  RUNTIME DESTINATION share/${PROJECT_NAME}/bt_plugins
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_set_sentry_posture test/test_set_sentry_posture.cpp)
  target_link_libraries(test_set_sentry_posture set_sentry_posture)
  set_tests_properties(test_set_sentry_posture PROPERTIES ENVIRONMENT
    "ROS_HOME=${CMAKE_BINARY_DIR}/ros_home;ROS_LOG_DIR=${CMAKE_BINARY_DIR}/ros_home/log")
endif()

ament_auto_package()
