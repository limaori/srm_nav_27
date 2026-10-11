#!/usr/bin/env bash
# 诊断 E：一次性 dump TF 图（节点、话题、发布者/订阅者），保存成文件便于分析。
set -uo pipefail
WS=/home/srm/srm_nav_27
VD=$WS/tmp/gicp_validation
OUT=$VD/logs/diag_graph_$(date +%H%M%S).log
export ROS_DOMAIN_ID=187
set +u
source "$VD/env.sh"
set -u
cd "$WS"
{
  echo "=== $(date)"
  ros2 launch srm27_gazebo_simulator srm_sim.launch.py \
    world_sdf:="$VD/worlds/rmuc_2025_ogre2.sdf" gui:=false run_immediately:=true \
    > "$VD/logs/gz_diag_graph.log" 2>&1 &
  sleep 32
  ros2 launch srm27_gicp_validation validation_stack.launch.py \
    namespace:=red_standard_robot1 params_file:="$VD/nav2_params_srm_test.yaml" \
    use_sim_time:=True use_pcd_localization:=True use_lio_odometry:=False slam:=False \
    > "$VD/logs/perception_diag_graph.log" 2>&1 &
  sleep 50
  echo "--- ros2 node list"
  timeout 20 ros2 node list 2>&1
  echo "--- ros2 topic list"
  timeout 20 ros2 topic list 2>&1
  echo "--- /red_standard_robot1/tf info"
  timeout 20 ros2 topic info /red_standard_robot1/tf -v 2>&1
  echo "--- /tf info"
  timeout 20 ros2 topic info /tf -v 2>&1
  echo "--- 当前 TF 帧对（tf2 buffer 视角）"
  timeout 25 "$WS/install/srm27_gicp_validation/bin/gicp_session_check" tf --duration 6 2>&1
  echo "--- perception 日志尾部（非 jump back）"
  grep -vE "jump back|process started|All log files|verbosity" "$VD/logs/perception_diag_graph.log" | tail -25
  echo "--- 结束"
  pkill -INT -f "srm_sim.launch.py" 2>/dev/null
  pkill -INT -f "validation_stack.launch.py" 2>/dev/null
  pkill -INT -f "ign gazebo" 2>/dev/null
} > "$OUT" 2>&1
cat "$OUT"
