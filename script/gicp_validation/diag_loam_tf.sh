#!/usr/bin/env bash
set -uo pipefail
WS=/home/srm/srm_nav_27
VD=$WS/tmp/gicp_validation
OUT=$VD/logs/diag_loamtf_$(date +%H%M%S).log
export ROS_DOMAIN_ID=186
set +u
source "$VD/env.sh"
set -u
cd "$WS"
{
  echo "=== $(date)"
  ros2 launch srm27_gazebo_simulator srm_sim.launch.py \
    world_sdf:="$VD/worlds/rmuc_2025_ogre2.sdf" gui:=false run_immediately:=true \
    > "$VD/logs/gz_diag_loamtf.log" 2>&1 &
  sleep 32
  "$WS/install/srm27_gicp_validation/bin/gicp_static_tf_republisher" \
    --ros-args -r __ns:=/red_standard_robot1 -p use_sim_time:=true > "$VD/logs/repub_loamtf.log" 2>&1 &
  sleep 10
  echo "--- /red_standard_robot1/tf 发布者"
  timeout 15 ros2 topic info /red_standard_robot1/tf -v 2>&1 | grep -E "Publisher count|Node name" | head -8
  echo "--- 我能否看到 base_link->front_mid360"
  timeout 20 "$WS/install/srm27_gicp_validation/bin/gicp_session_check" tf --duration 5 2>&1
  echo "--- 再起 loam_interface（用工作区参数），看它能否找到 base_link"
  ros2 run loam_interface loam_interface_node --ros-args -r __ns:=/red_standard_robot1 \
    --params-file /dev/null -p use_sim_time:=true -p odom_frame:=odom -p base_frame:=base_link \
    -p lidar_frame:=front_mid360 -p state_estimation_topic:=aft_mapped_to_init \
    -p registered_scan_topic:=cloud_registered -p planarize_pose:=true \
    > "$VD/logs/loam_loamtf.log" 2>&1 &
  sleep 12
  echo "--- loam_interface 日志"
  grep -vE "jump back" "$VD/logs/loam_loamtf.log" | tail -6
  echo "--- 结束"
  pkill -INT -f "srm_sim.launch.py" 2>/dev/null
  pkill -INT -f "gicp_static_tf_republisher" 2>/dev/null
  pkill -INT -f "loam_interface" 2>/dev/null
  pkill -INT -f "ign gazebo" 2>/dev/null
} > "$OUT" 2>&1
cat "$OUT"
