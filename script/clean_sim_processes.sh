#!/usr/bin/env bash
# 清理本工作空间仿真相关的残留进程。
# 单独放在脚本里执行：如果把 pkill 模式写在同一条 bash 命令行里，
# 模式会匹配到 bash 自身的命令行，导致把自己的 shell 也杀掉。
patterns=(
  'srm27_gazebo_simulator' 'srm27_nav_bringup' 'srm27_chassis_control'
  'srm_velocity_adapter' 'srm_cmd_mux' 'rotation_controller' 'rotation_test_sender'
  'ign gazebo' 'gz sim' 'ruby.*gazebo'
  'ros_gz_bridge' 'ros_gz_sim'
  'static_transform_publisher' 'simulation_ground_truth_odometry'
  'terrainAnalysis' 'map_server' 'lifecycle_manager'
  'controller_server' 'planner_server' 'bt_navigator' 'behavior_server'
  'velocity_smoother' 'waypoint_follower' 'smoother_server'
  'ign_sim_pointcloud_tool' 'sensor_scan_generation' 'robot_state_publisher'
  'point_lio' 'loam_interface' 'small_gicp' 'livox_ros_driver2' 'rviz2'
  'component_container'
)
for _ in 1 2 3; do
  for p in "${patterns[@]}"; do
    pkill -9 -f "$p" 2>/dev/null
  done
  sleep 1
done
left=$(pgrep -af 'srm27|ign gazebo|gz sim|ros_gz|nav2_|terrainAnalysis|component_container|rviz2' 2>/dev/null | grep -v clean_sim.sh | wc -l)
echo "剩余相关进程数: $left"
exit 0
