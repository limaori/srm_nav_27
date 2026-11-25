#!/usr/bin/env bash
# 只起链路不放包，看 slam_toolbox 到底有没有订阅 /scan
set -o pipefail
cd /home/srm/srm_nav_27 || exit 1
WS=$PWD
PARAMS=$1
LOG=$WS/.diag_tmp/probe_$(basename "$PARAMS" .yaml).log
source "$WS/scripts/setup/env_srm27.sh" >/dev/null 2>&1
export ROS_LOG_DIR=$WS/.tmp_roslog
ros2 launch srm27_nav_bringup real_mapping_launch.py \
  start_lidar:=False use_sim_time:=True use_rviz:=False save_pcd:=False \
  lio_frontend:=point_lio params_file:="$PARAMS" > "$LOG" 2>&1 &
PID=$!
sleep 25
echo "--- node info /slam_toolbox (subscriptions):"
ros2 node info /slam_toolbox 2>/dev/null | sed -n '/Subscribers/,/Publishers/p' | head -12
echo "--- /scan info:"
ros2 topic info /scan 2>/dev/null
echo "--- pointcloud_to_laserscan lines:"
grep -c "Got a subscriber" "$LOG"
kill -INT $PID 2>/dev/null
sleep 5
pkill -INT -f "[s]ync_slam_toolbox_node" 2>/dev/null
pkill -INT -f "[p]ointlio_mapping" 2>/dev/null
pkill -INT -f "[r]eal_mapping_launch\.py" 2>/dev/null
sleep 2
