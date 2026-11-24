cmake_minimum_required(VERSION 3.10)
project(srm27_nav_common)

# 规划前端（YAstar）与后端（MinimumSnap/SfcSquare）共用的**路径工具**：
# 加密、折角圆弧化、Path 消息 ↔ 点列转换。
# 2026-11-24 起上游算法源码分别放进各自插件包的 vendor/（本包不再含第三方代码）。
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(Eigen3 REQUIRED)

if(NOT TARGET Eigen3::Eigen)
  message(FATAL_ERROR "srm27_nav_common 需要 Eigen3::Eigen，未找到可用的 Eigen3 目标。")
endif()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/path_conversion.cpp
  src/path_utils.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
# ament_auto_add_library 内部通过 ament_target_dependencies() 使用了
# target_link_libraries 的 plain 签名，这里必须保持一致（不能写 PUBLIC/PRIVATE），
# 否则 CMake 报 "all uses ... must be either all-keyword or all-plain"。
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
ament_export_dependencies(Eigen3)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_path_conversion test/test_path_conversion.cpp)
  target_link_libraries(test_path_conversion ${PROJECT_NAME})

  ament_add_gtest(test_path_utils test/test_path_utils.cpp)
  target_link_libraries(test_path_utils ${PROJECT_NAME})
endif()

ament_auto_package()
