#!/usr/bin/env bash
set -o pipefail
cd /home/srm/srm_nav_27 || exit 1
WS=$PWD
source "$WS/scripts/setup/env_srm27.sh" >/dev/null 2>&1
export ROS_LOG_DIR=$WS/.tmp_roslog
ros2 launch srm27_nav_bringup real_mapping_launch.py \
  start_lidar:=False use_sim_time:=True use_rviz:=False save_pcd:=False \
  lio_frontend:=point_lio params_file:="$WS/.diag_tmp/mapping_params_tuned.yaml" \
  > "$WS/.diag_tmp/probe3.log" 2>&1 &
PID=$!
sleep 20
echo "=== slam_toolbox 节点个数: $(ros2 node list --no-daemon 2>/dev/null | grep -c '^/slam_toolbox$')"
echo "=== param try1: $(ros2 param get /slam_toolbox scan_topic 2>&1 | tail -1)"
echo "=== param try2: $(ros2 param get /slam_toolbox scan_topic 2>&1 | tail -1)"
echo "=== /scan verbose:"; ros2 topic info /scan --verbose 2>/dev/null | grep -E "Node name|Node namespace|Subscription count|Publisher count" | head -8
echo "=== /obstacle_scan verbose:"; ros2 topic info /obstacle_scan --verbose 2>/dev/null | grep -E "Node name|Subscription count|Publisher count" | head -8
echo "=== slam_toolbox subscriptions:"; ros2 node info /slam_toolbox 2>/dev/null | sed -n '/Subscribers/,/Publishers/p'
kill -INT $PID 2>/dev/null; sleep 4
pkill -INT -f "[s]ync_slam_toolbox_node"; pkill -INT -f "[p]ointlio_mapping"; pkill -INT -f "[r]eal_mapping_launch\.py"; sleep 2
