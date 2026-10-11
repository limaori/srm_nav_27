#!/usr/bin/env bash
# 诊断 A：直接用本工程文档里的 nav_srm_simulation_launch.py 起栈（不经验证专用 launch），
# 检查 registered_scan 是否发布，用于区分「验证 launch 配置问题」与「上游链路问题」。
set -uo pipefail

WS=/home/srm/srm_nav_27
VD=$WS/tmp/gicp_validation
OUT=$VD/logs/diag_nav_$(date +%H%M%S).log

ROS_DOMAIN_ID=${GICP_DOMAIN_ID:-198}
export ROS_DOMAIN_ID
set +u
source "$VD/env.sh"
set -u
cd "$WS"

{
  echo "=== ROS_DOMAIN_ID=$ROS_DOMAIN_ID 时间 $(date)"
  ros2 launch srm27_gazebo_simulator srm_sim.launch.py \
    world_sdf:="$VD/worlds/rmuc_2025_ogre2.sdf" gui:=false run_immediately:=true \
    > "$VD/logs/gz_diag_nav.log" 2>&1 &
  sleep 30
  ros2 launch srm27_nav_bringup nav_srm_simulation_launch.py \
    namespace:=red_standard_robot1 world:=rmuc_2025 \
    map:="$WS/src/srm27_navigation/srm27_nav_bringup/map/simulation/rmuc_2025.yaml" \
    prior_pcd_file:="$VD/promap_missing.pcd" \
    params_file:="$WS/src/srm27_navigation/srm27_nav_bringup/config/simulation/nav2_params_srm.yaml" \
    slam:=False use_pcd_localization:=False use_lio_odometry:=True \
    use_sim_time:=True use_composition:=False \
    start_simulation:=False start_rotation_sender:=False use_rviz:=False \
    > "$VD/logs/nav_diag.log" 2>&1 &
  sleep 90

  echo "--- node list"
  timeout 15 ros2 node list 2>&1
  echo "--- registered_scan 是否有数据"
  timeout 15 "$WS/install/srm27_gicp_validation/bin/gicp_session_check" ready --timeout 20 2>&1
  echo "--- tf 帧对"
  timeout 20 "$WS/install/srm27_gicp_validation/bin/gicp_session_check" tf --duration 5 2>&1
  echo "--- loam_interface 日志"
  grep "loam_interface" "$VD/logs/nav_diag.log" | tail -5
  echo "--- 结束"
  pkill -INT -f "srm_sim.launch.py" 2>/dev/null
  pkill -INT -f "nav_srm_simulation_launch.py" 2>/dev/null
  pkill -INT -f "ign gazebo" 2>/dev/null
} > "$OUT" 2>&1

cat "$OUT"
