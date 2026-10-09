cmake_minimum_required(VERSION 3.10)
project(srm27_minco_vendor)

# 上游 GCOPTER 的 MINCO / Trajectory / root finder / L-BFGS 头文件。
# 上游文件原样保留（含其 MIT 许可证头），本包只做 include 与依赖导出包装；
# SRM 侧的适配、代价函数和轨迹类型都放在 srm27_minco_core，避免直接改上游。
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

if(NOT TARGET Eigen3::Eigen)
  message(FATAL_ERROR "srm27_minco_vendor 需要 Eigen3::Eigen，未找到可用的 Eigen3 目标。")
endif()

install(DIRECTORY include/ DESTINATION include)
install(FILES LICENSE THIRD_PARTY_NOTICES.md DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_dependencies(Eigen3)
ament_package()
