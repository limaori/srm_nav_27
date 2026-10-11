#!/usr/bin/env bash
# 链路诊断：起栈后把 TF 树与关键话题状态写进日志，用于定位 registered_scan 不发布的原因。
set -uo pipefail

WS=/home/srm/srm_nav_27
VD=$WS/tmp/gicp_validation
OUT=$VD/logs/diag_$(date +%H%M%S).log

ROS_DOMAIN_ID=${GICP_DOMAIN_ID:-199}
export ROS_DOMAIN_ID
set +u
source "$VD/env.sh"
set -u
cd "$WS"

{
  echo "=== ROS_DOMAIN_ID=$ROS_DOMAIN_ID 时间 $(date)"
  echo "--- 启动链路"
  bash script/gicp_validation/start_validation_stack.sh > "$VD/logs/stack_diag.log" 2>&1 &
  sleep 60

  echo "--- ros2 node list"
  timeout 15 ros2 node list 2>&1

  echo "--- ros2 topic list"
  timeout 15 ros2 topic list 2>&1

  echo "--- TF 帧对（python 检查）"
  "$WS/install/srm27_gicp_validation/bin/gicp_session_check" tf --duration 5 2>&1

  echo "--- robot_description 的 link/joint 名"
  timeout 15 ros2 param get /red_standard_robot1/robot_state_publisher robot_description 2>&1 \
    | grep -oE '<link name="[^"]+"|<joint name="[^"]+"' | head -20

  echo "--- registered_scan info"
  timeout 15 ros2 topic info /red_standard_robot1/registered_scan -v 2>&1 | head -20

  echo "--- loam_interface 参数"
  for p in base_frame lidar_frame odom_frame state_estimation_topic registered_scan_topic; do
    printf '%s=' "$p"
    timeout 10 ros2 param get /red_standard_robot1/loam_interface "$p" 2>&1 | tail -1
  done

  echo "--- sensor_scan_generation 参数"
  for p in base_frame lidar_frame robot_base_frame publish_tf; do
    printf '%s=' "$p"
    timeout 10 ros2 param get /red_standard_robot1/sensor_scan_generation "$p" 2>&1 | tail -1
  done

  echo "--- point_lio 帧"
  timeout 12 ros2 topic echo /red_standard_robot1/aft_mapped_to_init --once 2>&1 | head -8

  echo "--- perception 日志尾部"
  tail -30 "$VD/logs/perception.log"

  echo "--- 结束"
  pkill -INT -f "srm_sim.launch.py" 2>/dev/null
  pkill -INT -f "validation_stack.launch.py" 2>/dev/null
  pkill -INT -f "ign gazebo" 2>/dev/null
} > "$OUT" 2>&1

cat "$OUT"
