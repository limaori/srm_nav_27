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

ament_package()
