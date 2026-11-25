#!/usr/bin/env bash
set -o pipefail
cd /home/srm/srm_nav_27 || exit 1
WS=$PWD
source "$WS/scripts/setup/env_srm27.sh" >/dev/null 2>&1
export ROS_LOG_DIR=$WS/.tmp_roslog
ros2 launch srm27_nav_bringup real_mapping_launch.py \
  start_lidar:=False use_sim_time:=True use_rviz:=False save_pcd:=False \
  lio_frontend:=point_lio params_file:="$WS/.diag_tmp/mapping_params_tuned.yaml" \
  > "$WS/.diag_tmp/probe2.log" 2>&1 &
PID=$!
sleep 22
echo "=== node list:"; ros2 node list 2>/dev/null
echo "=== scan_topic param:"; ros2 param get /slam_toolbox scan_topic 2>/dev/null
echo "=== odom_frame / map_frame:"; ros2 param get /slam_toolbox odom_frame 2>/dev/null; ros2 param get /slam_toolbox map_frame 2>/dev/null
echo "=== subscribers of /slam_toolbox:"; ros2 node info /slam_toolbox 2>/dev/null | sed -n '/Subscribers/,/Publishers/p'
echo "=== /obstacle_scan:"; ros2 topic info /obstacle_scan 2>/dev/null
echo "=== /scan:"; ros2 topic info /scan 2>/dev/null
kill -INT $PID 2>/dev/null; sleep 5
pkill -INT -f "[s]ync_slam_toolbox_node"; pkill -INT -f "[p]ointlio_mapping"; pkill -INT -f "[r]eal_mapping_launch\.py"; sleep 2
