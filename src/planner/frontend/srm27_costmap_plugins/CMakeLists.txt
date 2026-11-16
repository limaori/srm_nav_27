cmake_minimum_required(VERSION 3.8)
project(srm27_costmap_plugins)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(srm27_costmap_layers SHARED
  src/layers/intensity_voxel_layer.cpp
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_intensity_voxel_layer test/test_intensity_voxel_layer.cpp)
  target_link_libraries(test_intensity_voxel_layer srm27_costmap_layers)
  set_tests_properties(test_intensity_voxel_layer PROPERTIES ENVIRONMENT
    "ROS_HOME=${CMAKE_BINARY_DIR}/ros_home;ROS_LOG_DIR=${CMAKE_BINARY_DIR}/ros_home/log")
endif()

pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_plugins.xml)
install(FILES LICENSE NOTICE DESTINATION share/${PROJECT_NAME})

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
