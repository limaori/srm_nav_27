#!/usr/bin/env bash
set -o pipefail
cd /home/srm/srm_nav_27 || exit 1
WS=$PWD
source "$WS/scripts/setup/env_srm27.sh" >/dev/null 2>&1
export ROS_DOMAIN_ID=77
export ROS_LOG_DIR=$WS/.tmp_roslog
ros2 daemon stop >/dev/null 2>&1; sleep 2
ros2 launch srm27_nav_bringup real_mapping_launch.py \
  start_lidar:=False use_sim_time:=True use_rviz:=False save_pcd:=False \
  lio_frontend:=point_lio params_file:="$WS/.diag_tmp/mapping_params_tuned.yaml" \
  > "$WS/.diag_tmp/probe5.log" 2>&1 &
PID=$!
for i in $(seq 1 20); do
  if ros2 node list --no-daemon 2>/dev/null | grep -qx "/slam_toolbox"; then break; fi
  sleep 1
done
sleep 2
echo "=== 节点数(slam): $(ros2 node list --no-daemon 2>/dev/null | grep -c slam_toolbox)"
echo "=== scan_topic: $(ros2 param get /slam_toolbox scan_topic 2>&1 | tail -1)"
echo "=== 调过的参数(应为 0.3): $(ros2 param get /slam_toolbox link_match_minimum_response_fine 2>&1 | tail -1)"
echo "=== /scan: $(ros2 topic info /scan 2>/dev/null | tr '\n' ' ')"
echo "=== 谁在发 /scan: $(ros2 topic info /scan --verbose 2>/dev/null | grep 'Node name' | tr '\n' ' ')"
kill -INT $PID 2>/dev/null; sleep 4
pkill -INT -f "[s]ync_slam_toolbox_node"; pkill -INT -f "[p]ointlio_mapping"; pkill -INT -f "[r]eal_mapping_launch\.py"; sleep 2
