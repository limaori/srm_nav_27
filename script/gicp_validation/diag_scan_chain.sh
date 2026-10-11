#!/usr/bin/env bash
set -uo pipefail
WS=/home/srm/srm_nav_27
VD=$WS/tmp/gicp_validation
OUT=$VD/logs/diag_scan_$(date +%H%M%S).log
export ROS_DOMAIN_ID=195
set +u
source "$VD/env.sh"
set -u
cd "$WS"
{
  echo "=== $(date)"
  ros2 launch srm27_gazebo_simulator srm_sim.launch.py \
    world_sdf:="$VD/worlds/rmuc_2025_ogre2.sdf" gui:=false run_immediately:=true \
    > "$VD/logs/gz_diag_scan.log" 2>&1 &
  sleep 35
  ros2 launch srm27_gicp_validation validation_stack.launch.py \
    namespace:=red_standard_robot1 params_file:="$VD/nav2_params_srm_test.yaml" \
    use_sim_time:=True use_pcd_localization:=True use_lio_odometry:=False slam:=False \
    > "$VD/logs/perception_diag_scan.log" 2>&1 &
  sleep 45
  echo "--- aft_mapped_to_init 头部"
  timeout 12 ros2 topic echo /red_standard_robot1/aft_mapped_to_init --once 2>&1 | head -8
  echo "--- cloud_registered 头部"
  timeout 12 ros2 topic echo /red_standard_robot1/cloud_registered --once 2>&1 | head -8
  echo "--- loam_interface 输出的 lidar_odometry 头部"
  timeout 12 ros2 topic echo /red_standard_robot1/lidar_odometry --once 2>&1 | head -8
  echo "--- registered_scan 头部"
  timeout 12 ros2 topic echo /red_standard_robot1/registered_scan --once 2>&1 | head -8
  echo "--- odom->base_link (sensor_scan_generation 输出)"
  timeout 12 ros2 topic echo /red_standard_robot1/odometry --once 2>&1 | head -10
  echo "--- sensor_scan_generation 订阅"
  timeout 12 ros2 topic info /red_standard_robot1/aft_mapped_to_init -v 2>&1 | grep -E "Node name|Reliability" | head -8
  echo "--- perception 日志（sensor_scan 相关）"
  grep "sensor_scan" "$VD/logs/perception_diag_scan.log" | grep -v "jump back" | tail -6
  echo "--- 结束"
  pkill -INT -f "srm_sim.launch.py" 2>/dev/null
  pkill -INT -f "validation_stack.launch.py" 2>/dev/null
  pkill -INT -f "ign gazebo" 2>/dev/null
} > "$OUT" 2>&1
cat "$OUT"
