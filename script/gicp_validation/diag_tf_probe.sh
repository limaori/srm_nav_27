#!/usr/bin/env bash
# 诊断 C：最小复现 TF 查询失败——在链路运行时用 tf2_ros Buffer 直接查询 base_link。
set -uo pipefail
WS=/home/srm/srm_nav_27
VD=$WS/tmp/gicp_validation
OUT=$VD/logs/diag_probe_$(date +%H%M%S).log
ROS_DOMAIN_ID=${GICP_DOMAIN_ID:-196}
export ROS_DOMAIN_ID
set +u
source "$VD/env.sh"
set -u
cd "$WS"
{
  echo "=== ROS_DOMAIN_ID=$ROS_DOMAIN_ID 时间 $(date)"
  ros2 launch srm27_gazebo_simulator srm_sim.launch.py \
    world_sdf:="$VD/worlds/rmuc_2025_ogre2.sdf" gui:=false run_immediately:=true \
    > "$VD/logs/gz_diag_probe.log" 2>&1 &
  sleep 35
  ros2 launch srm27_gicp_validation validation_stack.launch.py \
    namespace:=red_standard_robot1 params_file:="$VD/nav2_params_srm_test.yaml" \
    use_sim_time:=True use_pcd_localization:=True use_lio_odometry:=False slam:=False \
    > "$VD/logs/perception_diag_probe.log" 2>&1 &
  sleep 30
  echo "--- 探针（use_sim_time=True，命名空间 /red_standard_robot1）"
  timeout 20 "$WS/install/srm27_gicp_validation/bin/gicp_tf_buffer_probe" \
    --ros-args -r __ns:=/red_standard_robot1 -p use_sim_time:=true 2>&1 | head -40
  echo "--- 探针（根命名空间，use_sim_time=True）"
  timeout 20 "$WS/install/srm27_gicp_validation/bin/gicp_tf_buffer_probe" \
    --ros-args -p use_sim_time:=true 2>&1 | head -40
  echo "--- 结束"
  pkill -INT -f "srm_sim.launch.py" 2>/dev/null
  pkill -INT -f "validation_stack.launch.py" 2>/dev/null
  pkill -INT -f "ign gazebo" 2>/dev/null
} > "$OUT" 2>&1
cat "$OUT"
