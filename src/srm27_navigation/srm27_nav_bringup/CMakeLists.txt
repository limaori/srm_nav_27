cmake_minimum_required(VERSION 3.8)
project(srm27_nav_bringup)

find_package(ament_cmake_auto REQUIRED)

install(PROGRAMS
  scripts/simulation_ground_truth_odometry.py
  scripts/srm_minco_real_preflight.py
  scripts/srm_regression_metadata.py
  scripts/srm_speed_limits_check.py
  scripts/srm_velocity_monitor.py
  DESTINATION lib/${PROJECT_NAME}
)

# 纯模块：launch（速度限幅合并）、实车预检与限幅自检脚本都要导入同一个实现，
# 必须装在脚本同目录（launch 通过 get_package_prefix(...)/lib/<包名> 导入）。
install(FILES
  scripts/srm_nav_pose_math.py
  scripts/srm_robot_description.py
  scripts/srm_speed_limits.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # 只保留本分支（小陀螺 / fakeyaw 链路）自己的回归测试。
  # 其余测试已随 main 的 b2d6fc2 一并删除，因此这里不再跑 ament_lint_auto。
  # 用普通 Python 入口而不是 pytest：本机 pytest11 插件与 launch_testing/
  # 较新 pluggy 不兼容，会在收集阶段直接失败。
  find_program(SRM27_PYTHON3_EXECUTABLE NAMES python3 REQUIRED)
  find_package(ament_cmake_test REQUIRED)
  foreach(test_name test_fakeyaw_launch_wiring test_speed_limits)
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
  behavior_trees
  rviz
)
