#!/usr/bin/env bash
set -o pipefail
cd /home/srm/srm_nav_27 || exit 1
WS=$PWD
source "$WS/scripts/setup/env_srm27.sh" >/dev/null 2>&1
export ROS_DOMAIN_ID=77
export ROS_LOG_DIR=$WS/.tmp_roslog
ros2 daemon stop >/dev/null 2>&1; sleep 2
ros2 launch "$WS/.diag_tmp/replay_mapping_launch.py" \
  start_lidar:=False use_sim_time:=True use_rviz:=False save_pcd:=False \
  lio_frontend:=point_lio params_file:="$WS/.diag_tmp/replay_params.yaml" \
  > "$WS/.diag_tmp/probe6.log" 2>&1 &
PID=$!
sleep 22
echo "=== nodes:"; ros2 node list --no-daemon 2>/dev/null | grep -v transform_listener
echo "=== services(slam):"; ros2 service list 2>/dev/null | grep -i slam
echo "=== livox subs:"; ros2 topic info /livox/lidar 2>/dev/null | tail -2
kill -INT $PID 2>/dev/null; sleep 4
pkill -INT -f "[s]ync_slam_toolbox_node"; pkill -INT -f "[p]ointlio_mapping"; pkill -INT -f "[r]eplay_mapping_launch"; sleep 2
