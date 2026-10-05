cmake_minimum_required(VERSION 3.8)
project(srm27_robot_description)

find_package(ament_cmake REQUIRED)

# Python 模块（几何读取与模型生成）安装到 site-packages，launch 文件可直接 import。
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/generate_robot_model.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  meshes
  urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # 用普通 Python 测试入口而不是 pytest：本机 python3 环境的 pytest11 插件
  # （launch_testing 与较新 pluggy）不兼容，会让 pytest 在收集阶段直接失败。
  # 测试文件本身仍然保持 pytest 兼容（纯 assert + test_ 前缀）。
  find_program(SRM27_PYTHON3_EXECUTABLE NAMES python3 REQUIRED)
  ament_add_test(test_model_builder
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND ${SRM27_PYTHON3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_model_builder.py
    OUTPUT_FILE ${CMAKE_CURRENT_BINARY_DIR}/test_model_builder.txt
    ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}:$ENV{PYTHONPATH}
  )
endif()

ament_package()
