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
