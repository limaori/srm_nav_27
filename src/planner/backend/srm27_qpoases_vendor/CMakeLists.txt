cmake_minimum_required(VERSION 3.10)
project(srm27_qpoases_vendor)

# 上游 qpOASES（LGPL-2.1）的 CMake 包装。
# 只编译本项目实际使用的稠密求解路径，产出独立共享库，便于满足 LGPL 的动态链接要求。
# 源码原样保留，未做任何修改；排除文件的原因见 THIRD_PARTY_NOTICES.md。
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)

set(SRM27_QPOASES_SOURCES
  src/BLASReplacement.cpp
  src/Bounds.cpp
  src/Constraints.cpp
  src/Flipper.cpp
  src/Indexlist.cpp
  src/LAPACKReplacement.cpp
  src/Matrices.cpp
  src/MessageHandling.cpp
  src/OQPinterface.cpp
  src/Options.cpp
  src/QProblem.cpp
  src/QProblemB.cpp
  src/SQProblem.cpp
  src/SQProblemSchur.cpp
  src/SolutionAnalysis.cpp
  src/SparseSolver.cpp
  src/SubjectTo.cpp
  src/Utils.cpp
)

foreach(_source IN LISTS SRM27_QPOASES_SOURCES)
  if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${_source}")
    message(FATAL_ERROR "srm27_qpoases_vendor 缺少源文件: ${_source}")
  endif()
endforeach()

add_library(srm27_qpoases SHARED ${SRM27_QPOASES_SOURCES})
add_library(${PROJECT_NAME}::srm27_qpoases ALIAS srm27_qpoases)

target_include_directories(
  srm27_qpoases
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)
set_target_properties(srm27_qpoases PROPERTIES OUTPUT_NAME srm27_qpoases VERSION 3.2.1)

install(
  TARGETS srm27_qpoases
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(FILES LICENSE THIRD_PARTY_NOTICES.md DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(srm27_qpoases)

# 自检：确认头文件与源文件都被找到，避免只设置一个未检查的链接标志后继续。
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include/qpOASES.hpp")
  message(FATAL_ERROR "srm27_qpoases_vendor 缺少 include/qpOASES.hpp")
endif()

ament_package()
