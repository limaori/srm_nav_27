# 构建定位后端核心、ROS 适配器、地图工具及功能回归。
cmake_minimum_required(VERSION 3.16)
if(POLICY CMP0074)
    cmake_policy(SET CMP0074 NEW)
endif()
project(srm27_zlocalization_backend LANGUAGES CXX)
include(${CMAKE_CURRENT_LIST_DIR}/cmake/Warnings.cmake)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(message_filters REQUIRED)
find_package(localization_interfaces REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io kdtree filters)
find_package(Eigen3 REQUIRED)
find_path(NANOFLANN_INCLUDE_DIR nanoflann.hpp REQUIRED)
find_package(Threads REQUIRED)
find_package(TBB REQUIRED COMPONENTS tbb)
find_package(OpenMP REQUIRED COMPONENTS CXX)
find_package(GTSAM 4.2 CONFIG REQUIRED)
if(GTSAM_VERSION VERSION_GREATER_EQUAL 4.3)
    message(FATAL_ERROR "backend targets GTSAM 4.2.x; select its GTSAM_DIR")
endif()

# 核心库不依赖 ROS，节点和工具按各自接口链接。
set(BACKEND_ROS_DEPS
    rclcpp
    std_msgs
    std_srvs
    geometry_msgs
    nav_msgs
    sensor_msgs
    visualization_msgs
    tf2
    tf2_ros
    pcl_conversions
    message_filters
    localization_interfaces
)

# 定位算法与运行状态共用一个非 ROS 核心目标。
add_library(srm27_zlocalization_backend_core STATIC
    src/uwb_alignment.cpp
    src/body.cpp
    src/map_bundle.cpp
    src/graph_calibration.cpp
    src/pose_conditioning.cpp
    src/config.cpp
    src/loop_consistency.cpp
    src/localization.cpp
    src/covariance.cpp
    src/directory.cpp
    src/measurement.cpp
    src/sampling.cpp
    src/descriptor.cpp
    src/graph_estimates.cpp
    src/uwb_alignment_solver.cpp
    src/frame_history.cpp
    src/uwb_fusion.cpp
    src/gprobe/descriptor.cpp
    src/gprobe/geometry.cpp
    src/gprobe/retrieval.cpp
    src/gprobe/storage.cpp
    src/pose_graph.cpp
    src/relocalization_hypotheses.cpp
    src/graph_innovation.cpp
    src/logger.cpp
    src/loop.cpp
    src/keyframe_map.cpp
    src/recovery_matcher.cpp
    src/graph_noise.cpp
    src/observation.cpp
    src/pose_output.cpp
    src/graph_pose.cpp
    src/localization_output.cpp
    src/registration_quality.cpp
    src/localization_recovery.cpp
    src/registration.cpp
    src/reset_coordinator.cpp
    src/scan_context/descriptor.cpp
    src/scan_context/storage.cpp
    src/relocalization.cpp
    src/registration_solver.cpp
    src/localization_state.cpp
    src/cloud_storage.cpp
    src/target_cache.cpp
    src/uwb_timing.cpp
    src/tracking_matcher.cpp
    src/localization_trajectory.cpp
    src/graph_uwb.cpp
    src/relocalization_verification.cpp
)
target_include_directories(srm27_zlocalization_backend_core
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR}
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src ${NANOFLANN_INCLUDE_DIR})
target_link_libraries(srm27_zlocalization_backend_core PUBLIC ${PCL_LIBRARIES} gtsam Threads::Threads
    TBB::tbb OpenMP::OpenMP_CXX)
# Linux 核心消费者优先使用构建选中的库，避免同 SONAME 的其他版本覆盖。
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_link_options(srm27_zlocalization_backend_core INTERFACE "LINKER:--disable-new-dtags")
endif()
# ROS 节点只负责消息、参数、服务与发布适配。
add_executable(srm27_zlocalization_backend_node
    apps/fusion_parameters.cpp
    apps/node_input.cpp
    apps/node_map.cpp
    apps/main.cpp
    apps/frontend_connection.cpp
    apps/frontend_session.cpp
    apps/ros.cpp
    apps/publisher.cpp
    apps/publisher_state.cpp
    apps/publisher_view.cpp
    apps/publisher_legacy.cpp
    apps/node.cpp
    apps/parameters.cpp
    apps/node_setup.cpp
)
target_include_directories(srm27_zlocalization_backend_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
ament_target_dependencies(srm27_zlocalization_backend_node ${BACKEND_ROS_DEPS})
target_link_libraries(srm27_zlocalization_backend_node srm27_zlocalization_backend_core)
set_target_properties(srm27_zlocalization_backend_node PROPERTIES
    BUILD_RPATH "$<TARGET_FILE_DIR:gtsam>"
    INSTALL_RPATH "$<TARGET_FILE_DIR:gtsam>")
# 地图与标定工具保留稳定的命令名称。
add_library(srm27_zlocalization_backend_tool_bundle STATIC tools/bundle.cpp)
target_include_directories(srm27_zlocalization_backend_tool_bundle PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/tools)
target_link_libraries(srm27_zlocalization_backend_tool_bundle PUBLIC srm27_zlocalization_backend_core)
add_executable(srm27_zlocalization_backend_build_pcd_map_bundle tools/build.cpp tools/generate.cpp)
add_executable(srm27_zlocalization_backend_filter_map_bundle tools/filter.cpp)
add_executable(srm27_zlocalization_backend_transform_map_bundle tools/transform.cpp tools/alignment.cpp)
add_executable(srm27_zlocalization_backend_rebuild_scan_context_descriptors tools/describe.cpp)
add_executable(srm27_zlocalization_backend_body_calibration_node tools/calibrate.cpp tools/body_calibration.cpp)
set(BACKEND_TOOLS srm27_zlocalization_backend_build_pcd_map_bundle srm27_zlocalization_backend_filter_map_bundle srm27_zlocalization_backend_transform_map_bundle
                  srm27_zlocalization_backend_rebuild_scan_context_descriptors srm27_zlocalization_backend_body_calibration_node)
# 离线清理只属于工具；核心库和节点不链接这一实现。
add_library(srm27_zlocalization_backend_map_cleanup STATIC tools/cleanup/pipeline.cpp tools/cleanup/visibility.cpp)
target_include_directories(srm27_zlocalization_backend_map_cleanup
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/tools PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(srm27_zlocalization_backend_map_cleanup PUBLIC srm27_zlocalization_backend_core)
add_executable(srm27_zlocalization_backend_clean_map_bundle tools/cleanup.cpp)
target_link_libraries(srm27_zlocalization_backend_clean_map_bundle srm27_zlocalization_backend_map_cleanup)
foreach(tool srm27_zlocalization_backend_build_pcd_map_bundle srm27_zlocalization_backend_filter_map_bundle srm27_zlocalization_backend_transform_map_bundle srm27_zlocalization_backend_clean_map_bundle)
    target_link_libraries(${tool} srm27_zlocalization_backend_tool_bundle)
endforeach()
list(APPEND BACKEND_TOOLS srm27_zlocalization_backend_clean_map_bundle)
foreach(tool IN LISTS BACKEND_TOOLS)
    target_include_directories(${tool} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR})
    target_link_libraries(${tool} srm27_zlocalization_backend_core)
endforeach()
ament_target_dependencies(srm27_zlocalization_backend_body_calibration_node rclcpp geometry_msgs nav_msgs tf2 tf2_ros tf2_eigen)
install(TARGETS srm27_zlocalization_backend_node ${BACKEND_TOOLS}
    RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY rviz docs DESTINATION share/${PROJECT_NAME}
    PATTERN "__pycache__" EXCLUDE PATTERN "*.pyc" EXCLUDE)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
    FILES_MATCHING PATTERN "*.launch.py" PATTERN "__pycache__" EXCLUDE)
install(FILES README.md LICENSE DESTINATION share/${PROJECT_NAME})

localization_enable_warnings()
if(BUILD_TESTING)
    add_subdirectory(test)
endif()

ament_package()
