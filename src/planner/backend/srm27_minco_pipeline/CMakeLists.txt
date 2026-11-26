cmake_minimum_required(VERSION 3.10)
project(srm27_minco_pipeline)

# MINCO 的“前端流水线”：局部路径裁剪 / 净空预推 + 后台规划线程。
#
# 2026-11-26 从 srm27_minco_controller 抽出：MINCO 迁到规划器后，规划器与控制器
# （保留 internal 模式做 A/B 与回退）都需要这段代码，放在这里避免复制两份。
# 本包**不依赖 ROS**（只有 Eigen + 地图/优化核心），因此谁都能链接。

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/local_path.cpp
  src/planning_worker.cpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  foreach(_test_name test_local_path test_planning_worker)
    ament_add_gtest(${_test_name} test/${_test_name}.cpp)
    target_link_libraries(${_test_name} ${PROJECT_NAME})
  endforeach()
endif()

ament_auto_package()
