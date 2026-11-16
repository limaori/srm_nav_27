# 生成前后端同步观测、会话身份和重置协议。
cmake_minimum_required(VERSION 3.8)
project(localization_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ResetFrontend.srv"
  "msg/FrontendFrame.msg"
  "msg/FrontendOdometry.msg"
  "msg/FrontendSession.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs nav_msgs sensor_msgs
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
ament_export_include_directories(include/${PROJECT_NAME})
ament_export_dependencies(rosidl_default_runtime)
install(FILES LICENSE DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
    add_subdirectory(test)
endif()

ament_package()
