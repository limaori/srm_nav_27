# 构建激光惯性里程计核心库及包含 Livox 支持的 ROS 2 节点。
cmake_minimum_required(VERSION 3.16)
project(frontend VERSION 0.4.0 LANGUAGES CXX)
# 迁移说明（仿真 LIO 前端替换 Z-LIO 实施方案 §6.1）：本包原先位于
# z-localization/src/frontend，警告规则、算法配置与授权文件都在仓库根目录
# （../../cmake、../../config、../../LICENSE）。并入本工作空间后改为包内自持，
# 因此下面三处路径从 ../../ 收回本包目录，其余构建逻辑与上游一致。
include(${CMAKE_CURRENT_LIST_DIR}/cmake/Warnings.cmake)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)
find_package(OpenMP REQUIRED COMPONENTS CXX)
find_path(NANOFLANN_INCLUDE_DIR nanoflann.hpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(localization_interfaces REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

add_library(zlio
    src/config.cpp
    src/execution.cpp
    src/filter.cpp
    src/geometry.cpp
    src/inertial.cpp
    src/initialization.cpp
    src/input.cpp
    src/map.cpp
    src/observation.cpp
    src/odometry.cpp
    src/plane.cpp
    src/prediction.cpp
    src/preintegration.cpp
    src/registration.cpp
    src/sampling.cpp
    src/state.cpp
    src/trajectory.cpp
    src/uncertainty.cpp
    src/window.cpp
)
target_include_directories(zlio
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE ${NANOFLANN_INCLUDE_DIR}
)
target_compile_features(zlio PUBLIC cxx_std_17)
target_link_libraries(zlio PUBLIC Eigen3::Eigen PRIVATE yaml-cpp Threads::Threads OpenMP::OpenMP_CXX)
add_library(zlio::zlio ALIAS zlio)

add_executable(zlio_node apps/main.cpp apps/node.cpp apps/input.cpp apps/ros.cpp)
target_link_libraries(zlio_node zlio Threads::Threads)
ament_target_dependencies(zlio_node rclcpp sensor_msgs nav_msgs std_srvs tf2_ros
    localization_interfaces livox_ros_driver2)
add_executable(zlio_visualizer apps/visualization.cpp)
target_include_directories(zlio_visualizer PRIVATE include)
target_link_libraries(zlio_visualizer Eigen3::Eigen)
ament_target_dependencies(zlio_visualizer rclcpp sensor_msgs nav_msgs geometry_msgs localization_interfaces)

install(TARGETS zlio_node zlio_visualizer DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME} PATTERN "__pycache__" EXCLUDE)
ament_export_targets(zlioTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 yaml-cpp Threads OpenMP)

# 同时导出纯 C++ 核心目标，消费者无需链接 ROS 节点。
include(CMakePackageConfigHelpers)
configure_package_config_file(cmake/zlioConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/zlioConfig.cmake INSTALL_DESTINATION share/zlio/cmake)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/zlioConfigVersion.cmake
    VERSION ${PROJECT_VERSION} COMPATIBILITY SameMajorVersion)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/zlioConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/zlioConfigVersion.cmake DESTINATION share/zlio/cmake)

install(TARGETS zlio EXPORT zlioTargets
    ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
set(ZLIO_PUBLIC_HEADERS
    include/zlio/config.hpp
    include/zlio/filter.hpp
    include/zlio/inertial.hpp
    include/zlio/initialization.hpp
    include/zlio/input.hpp
    include/zlio/map.hpp
    include/zlio/observation.hpp
    include/zlio/odometry.hpp
    include/zlio/plane.hpp
    include/zlio/prediction.hpp
    include/zlio/sampling.hpp
    include/zlio/state.hpp
    include/zlio/trajectory.hpp
    include/zlio/types.hpp
    include/zlio/voxel.hpp
    include/zlio/window.hpp
)
install(FILES ${ZLIO_PUBLIC_HEADERS} DESTINATION include/zlio)
install(FILES LICENSE DESTINATION share/${PROJECT_NAME})
install(FILES config/lio.yaml DESTINATION share/${PROJECT_NAME}/config)
install(EXPORT zlioTargets NAMESPACE zlio:: DESTINATION share/zlio/cmake)

localization_enable_warnings()
if(BUILD_TESTING)
    add_subdirectory(test)
endif()

ament_package()
