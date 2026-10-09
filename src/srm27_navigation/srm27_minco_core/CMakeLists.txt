cmake_minimum_required(VERSION 3.10)
project(srm27_minco_core)

# 与本项目相邻 Nav2 controller 包一致的 C++ 标准与格式/静态检查配置。
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(srm27_minco_vendor REQUIRED)
find_package(srm27_qpoases_vendor REQUIRED)

add_library(
  ${PROJECT_NAME} SHARED
  src/esdf_2d.cpp
  src/grid_snapshot.cpp
  src/minco_optimizer.cpp
  src/mpc_model.cpp
  src/mpc_solver.cpp
  src/replan_manager.cpp
  src/tracking_reference.cpp
  src/trajectory_2d.cpp
  src/trajectory_initializer.cpp
  src/trajectory_validator.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)

# 上游 GCOPTER 头文件走 SYSTEM，避免其告警进入本项目受维护代码的告警面。
# 必须是 PUBLIC：公开头文件 `minco_optimizer.hpp` 直接 include `<gcopter/minco.hpp>`，
# 下游（本包测试目标、srm27_minco_controller）同样需要这个 include 路径。
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${srm27_minco_vendor_INCLUDE_DIRS})
target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC
    Eigen3::Eigen
    ${srm27_qpoases_vendor_TARGETS}
)

if(BUILD_TESTING)
  set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/.clang-format")
  set(ament_cmake_clang_tidy_CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/.clang-tidy")
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  set(SRM27_MINCO_CORE_TEST_SOURCES
    test/test_esdf_2d.cpp
    test/test_kinematics.cpp
    test/test_minco_optimizer.cpp
    test/test_mpc_solver.cpp
    test/test_mpc_startup.cpp
    test/test_replan_manager.cpp
    test/test_smooth_cost.cpp
    test/test_tracking_reference.cpp
    test/test_trajectory_2d.cpp
    test/test_trajectory_initializer.cpp
    test/test_trajectory_validator.cpp
  )
  foreach(_test_source IN LISTS SRM27_MINCO_CORE_TEST_SOURCES)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${_test_source}")
      get_filename_component(_test_name ${_test_source} NAME_WE)
      ament_add_gtest(${_test_name} ${_test_source})
      target_link_libraries(${_test_name} ${PROJECT_NAME})
    endif()
  endforeach()
endif()

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(Eigen3 srm27_minco_vendor srm27_qpoases_vendor)

ament_package()
