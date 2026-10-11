#!/usr/bin/env bash
set -uo pipefail
WS=/home/srm/srm_nav_27
VD=$WS/tmp/gicp_validation
OUT=$VD/logs/diag_tfwhere_$(date +%H%M%S).log
export ROS_DOMAIN_ID=184
set +u
source "$VD/env.sh"
set -u
cd "$WS"
{
  echo "=== $(date)"
  ros2 launch srm27_gazebo_simulator srm_sim.launch.py \
    world_sdf:="$VD/worlds/rmuc_2025_ogre2.sdf" gui:=false run_immediately:=true \
    > "$VD/logs/gz_tfwhere.log" 2>&1 &
  sleep 30
  ros2 launch srm27_gicp_validation validation_stack.launch.py \
    namespace:=red_standard_robot1 params_file:="$VD/nav2_params_srm_test.yaml" \
    use_sim_time:=True use_pcd_localization:=True use_lio_odometry:=False slam:=False \
    > "$VD/logs/perception_tfwhere.log" 2>&1 &
  sleep 60
  echo "--- topics matching tf"
  timeout 15 ros2 topic list 2>&1 | grep -i tf
  echo "--- /tf info"
  timeout 15 ros2 topic info /tf -v 2>&1 | grep -E "Publisher count|Node name|Topic type" | head -14
  echo "--- /red_standard_robot1/tf info"
  timeout 15 ros2 topic info /red_standard_robot1/tf -v 2>&1 | grep -E "Publisher count|Node name|Topic type" | head -14
  echo "--- scan provider 的 frame 统计"
  grep -E "registered_scan\] 已发布" "$VD/logs/perception_tfwhere.log" | tail -2
  echo "--- 结束"
  pkill -INT -f "srm_sim.launch.py" 2>/dev/null
  pkill -INT -f "validation_stack.launch.py" 2>/dev/null
  pkill -INT -f "ign gazebo" 2>/dev/null
} > "$OUT" 2>&1
cat "$OUT"
