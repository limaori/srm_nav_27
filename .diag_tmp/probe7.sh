#!/usr/bin/env bash
# 60 秒迷你回放：只回答一件事 —— /scan 有没有流量、slam_toolbox 有没有发 /map
set -o pipefail
cd /home/srm/srm_nav_27 || exit 1
WS=$PWD
BAG=$HOME/srm_nav_27_data/bags/mapping_bag_202to227_20261124_223712
source "$WS/scripts/setup/env_srm27.sh" >/dev/null 2>&1
export ROS_DOMAIN_ID=77
export ROS_LOG_DIR=$WS/.tmp_roslog
ros2 daemon stop >/dev/null 2>&1; sleep 2

ros2 launch "$WS/.diag_tmp/replay_mapping_launch.py" \
  start_lidar:=False use_sim_time:=True use_rviz:=False save_pcd:=False \
  lio_frontend:=point_lio params_file:="$WS/.diag_tmp/replay_params.yaml" \
  > "$WS/.diag_tmp/probe7.log" 2>&1 &
PID=$!
for i in $(seq 1 30); do
  ros2 node list --no-daemon 2>/dev/null | grep -qx "/replay_slam" && break
  sleep 1
done
sleep 3
echo "scan_topic: $(ros2 param get /replay_slam scan_topic 2>&1 | tail -1)"

ros2 bag play "$BAG" --clock --topics /livox/lidar /livox/imu --rate 1.0 --start-offset 15 >> "$WS/.diag_tmp/probe7.log" 2>&1 &
PLAY=$!
sleep 8
( timeout 45 ros2 topic hz /scan > "$WS/.diag_tmp/probe7_scan_hz.txt" 2>&1 ) &
( timeout 45 ros2 topic hz /map  > "$WS/.diag_tmp/probe7_map_hz.txt" 2>&1 ) &
sleep 47
kill -INT $PLAY 2>/dev/null
echo "=== /scan hz:"; tail -3 "$WS/.diag_tmp/probe7_scan_hz.txt"
echo "=== /map hz:";  tail -3 "$WS/.diag_tmp/probe7_map_hz.txt"
echo "=== drops: $(grep -c 'Message Filter dropping' "$WS/.diag_tmp/probe7.log")"
echo "=== /scan msgs: $(grep -c 'Got a subscriber' "$WS/.diag_tmp/probe7.log")"
kill -INT $PID 2>/dev/null; sleep 4
pkill -INT -f "[s]ync_slam_toolbox_node"; pkill -INT -f "[p]ointlio_mapping"; pkill -INT -f "[r]eplay_mapping_launch"; sleep 2
