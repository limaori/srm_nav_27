cmake_minimum_required(VERSION 3.8)
project(srm27_nav_bringup)

find_package(ament_cmake_auto REQUIRED)

install(PROGRAMS
  scripts/simulation_ground_truth_odometry.py
  scripts/srm_regression_metadata.py
  scripts/srm_velocity_monitor.py
  DESTINATION lib/${PROJECT_NAME}
)

# 纯几何模块：真值里程计适配器运行时与几何回归测试都要导入，必须装在脚本同目录。
install(FILES
  scripts/srm_nav_pose_math.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify
    ament_cmake_cpplint
    ament_cmake_flake8
  )
  ament_lint_auto_find_test_dependencies()

  # 几何回归测试用普通 Python 入口而不是 pytest：本机 pytest11 插件与
  # launch_testing/较新 pluggy 不兼容，会在收集阶段直接失败（与
  # srm27_robot_description 的 test_model_builder 保持一致的做法）。
  find_program(SRM27_PYTHON3_EXECUTABLE NAMES python3 REQUIRED)
  find_package(ament_cmake_test REQUIRED)
  foreach(test_name test_pose_math test_ground_truth_odometry_pose)
    ament_add_test(${test_name}
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO
      COMMAND ${SRM27_PYTHON3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/test/${test_name}.py
      OUTPUT_FILE ${CMAKE_CURRENT_BINARY_DIR}/${test_name}.txt
      ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts:$ENV{PYTHONPATH}
    )
  endforeach()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  map
  meshes
  pcd
  config
  urdf
  behavior_trees
  rviz
)
