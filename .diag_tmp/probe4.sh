#!/usr/bin/env bash
set -o pipefail
cd /home/srm/srm_nav_27 || exit 1
WS=$PWD
source "$WS/scripts/setup/env_srm27.sh" >/dev/null 2>&1
export ROS_DOMAIN_ID=77
export ROS_LOG_DIR=$WS/.tmp_roslog
ros2 launch srm27_nav_bringup real_mapping_launch.py \
  start_lidar:=False use_sim_time:=True use_rviz:=False save_pcd:=False \
  lio_frontend:=point_lio params_file:="$WS/.diag_tmp/mapping_params_tuned.yaml" \
  > "$WS/.diag_tmp/probe4.log" 2>&1 &
PID=$!
sleep 18
echo "=== slam_toolbox 进程命令行:"
ps -eo pid,args | grep "[s]ync_slam_toolbox" | tr ' ' '\n' | grep -E "params-file|__node|^/opt" | head -10
echo "=== scan_topic / map_frame:"
ros2 param get /slam_toolbox scan_topic
echo "=== 参数来源文件里的 scan_topic:"
grep -n "scan_topic" "$WS/.diag_tmp/mapping_params_tuned.yaml"
kill -INT $PID 2>/dev/null; sleep 4
pkill -INT -f "[s]ync_slam_toolbox_node"; pkill -INT -f "[p]ointlio_mapping"; pkill -INT -f "[r]eal_mapping_launch\.py"; sleep 2
