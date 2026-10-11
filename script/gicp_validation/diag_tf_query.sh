#!/usr/bin/env bash
# 诊断 B：只起验证专用链路，重复用 tf2_echo 查询 base_link <- front_mid360，
# 判断 loam_interface 的持续失败是「一次性竞态」还是「TF 树里根本没有该帧」。
set -uo pipefail

WS=/home/srm/srm_nav_27
VD=$WS/tmp/gicp_validation
OUT=$VD/logs/diag_tf_$(date +%H%M%S).log

ROS_DOMAIN_ID=${GICP_DOMAIN_ID:-197}
export ROS_DOMAIN_ID
set +u
source "$VD/env.sh"
set -u
cd "$WS"

{
  echo "=== ROS_DOMAIN_ID=$ROS_DOMAIN_ID 时间 $(date)"
  ros2 launch srm27_gazebo_simulator srm_sim.launch.py \
    world_sdf:="$VD/worlds/rmuc_2025_ogre2.sdf" gui:=false run_immediately:=true \
    > "$VD/logs/gz_diag_tf.log" 2>&1 &
  sleep 35

  ros2 launch srm27_gicp_validation validation_stack.launch.py \
    namespace:=red_standard_robot1 params_file:="$VD/nav2_params_srm_test.yaml" \
    use_sim_time:=True use_pcd_localization:=True use_lio_odometry:=False slam:=False \
    > "$VD/logs/perception_diag_tf.log" 2>&1 &
  sleep 40

  echo "--- tf2_echo base_link <- front_mid360（默认，sim 时间 0）"
  timeout 12 ros2 run tf2_ros tf2_echo base_link front_mid360 2>&1 | head -6
  echo "--- tf2_echo base_link <- front_mid360 @ sim 100"
  timeout 12 ros2 run tf2_ros tf2_echo base_link front_mid360 100 2>&1 | head -6
  echo "--- tf2_echo base_link <- front_mid360 @ wall 0"
  timeout 12 ros2 run tf2_ros tf2_echo base_link front_mid360 0 2>&1 | head -6

  echo "--- tf2_echo odom <- base_link"
  timeout 20 ros2 run tf2_ros tf2_echo odom base_link 2>&1 | head -12

  echo "--- python 检查"
  timeout 25 "$WS/install/srm27_gicp_validation/bin/gicp_session_check" tf --duration 5 2>&1

  echo "--- topic info tf_static"
  timeout 15 ros2 topic info /red_standard_robot1/tf_static -v 2>&1 | head -25

  echo "--- tf_static 原始内容"
  timeout 15 ros2 topic echo /red_standard_robot1/tf_static --once 2>&1 | head -14

  echo "--- loam_interface 前 5 条日志"
  grep "loam_interface" "$VD/logs/perception_diag_tf.log" | head -5

  echo "--- 结束"
  pkill -INT -f "srm_sim.launch.py" 2>/dev/null
  pkill -INT -f "validation_stack.launch.py" 2>/dev/null
  pkill -INT -f "ign gazebo" 2>/dev/null
} > "$OUT" 2>&1

cat "$OUT"
