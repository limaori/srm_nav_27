#!/usr/bin/env bash
set -uo pipefail
WS=/home/srm/srm_nav_27
VD=$WS/tmp/gicp_validation
OUT=$VD/logs/diag_odomtf_$(date +%H%M%S).log
export ROS_DOMAIN_ID=193
set +u
source "$VD/env.sh"
set -u
cd "$WS"
{
  echo "=== $(date)"
  ros2 launch srm27_gazebo_simulator srm_sim.launch.py \
    world_sdf:="$VD/worlds/rmuc_2025_ogre2.sdf" gui:=false run_immediately:=true \
    > "$VD/logs/gz_diag_odomtf.log" 2>&1 &
  sleep 35
  ros2 launch srm27_gicp_validation validation_stack.launch.py \
    namespace:=red_standard_robot1 params_file:="$VD/nav2_params_srm_test.yaml" \
    use_sim_time:=True use_pcd_localization:=True use_lio_odometry:=False slam:=False \
    > "$VD/logs/perception_diag_odomtf.log" 2>&1 &
  sleep 55
  echo "--- odom->base_link 发布者进程状态（日志）"
  grep -E "odom_base_tf" "$VD/logs/perception_diag_odomtf.log" | tail -4
  echo "--- lidar_odometry 是否有订阅者（应包含 gicp_odom_base_tf_publisher）"
  timeout 12 ros2 topic info /red_standard_robot1/lidar_odometry -v 2>&1 | grep -E "Node name|Publisher count|Subscription count" | head -8
  echo "--- 独立节点日志（不通过 launch）"
  timeout 15 "$WS/install/srm27_gicp_validation/bin/gicp_odom_base_tf_publisher" \
    --ros-args -r __ns:=/red_standard_robot1 -p use_sim_time:=true \
    -p input_odometry_topic:=lidar_odometry 2>&1 | head -6 &
  sleep 10
  echo "--- 此时 odom->base_link 是否出现"
  timeout 15 "$WS/install/srm27_gicp_validation/bin/gicp_session_check" tf --duration 5 2>&1
  echo "--- 独立节点的日志"
  grep -E "odom_base_tf|过期" "$VD/logs/perception_diag_odomtf.log" | tail -4
  echo "--- 结束"
  pkill -INT -f "srm_sim.launch.py" 2>/dev/null
  pkill -INT -f "validation_stack.launch.py" 2>/dev/null
  pkill -INT -f "gicp_odom_base_tf_publisher" 2>/dev/null
  pkill -INT -f "ign gazebo" 2>/dev/null
} > "$OUT" 2>&1
cat "$OUT"
