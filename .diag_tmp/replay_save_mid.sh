#!/usr/bin/env bash
# 在放包「途中」存图：此时 /clock 还在走、slam_toolbox 正在发 /map，
# 避开放完之后 sim time 停住、map_saver 收不到消息的问题。
set -o pipefail
cd /home/srm/srm_nav_27 || exit 1
WS=$PWD
BAG=$HOME/srm_nav_27_data/bags/mapping_bag_202to227_20261124_223712
OUT=$WS/maps/202to227_replay/202to227_replay
LOG=$WS/.diag_tmp/replay_save_mid.log
source "$WS/scripts/setup/env_srm27.sh" >/dev/null 2>&1
export ROS_DOMAIN_ID=77
export ROS_LOG_DIR=$WS/.tmp_roslog
ros2 daemon stop >/dev/null 2>&1; sleep 2
mkdir -p "$(dirname "$OUT")"
: > "$LOG"

ros2 launch "$WS/.diag_tmp/replay_mapping_launch.py" \
  start_lidar:=False use_sim_time:=True use_rviz:=False save_pcd:=False \
  lio_frontend:=point_lio params_file:="$WS/.diag_tmp/replay_params.yaml" >> "$LOG" 2>&1 &
PID=$!
for i in $(seq 1 30); do ros2 node list --no-daemon 2>/dev/null | grep -qx "/replay_slam" && break; sleep 1; done
sleep 3
echo "scan_topic: $(ros2 param get /replay_slam scan_topic 2>&1 | tail -1)" | tee -a "$LOG"

ros2 bag play "$BAG" --clock --topics /livox/lidar /livox/imu --rate 1.0 >> "$LOG" 2>&1 &
PLAY=$!
sleep 195
echo "[mid] /map 频率:" | tee -a "$LOG"
timeout 6 ros2 topic hz /map >> "$LOG" 2>&1
echo "[mid] 存图 (service):" | tee -a "$LOG"
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: '$OUT'}}" >> "$LOG" 2>&1
if [ ! -s "$OUT.pgm" ]; then
  echo "[mid] service 没出图, map_saver_cli:" | tee -a "$LOG"
  timeout 40 ros2 run nav2_map_server map_saver_cli -f "$OUT" >> "$LOG" 2>&1
fi
kill -INT $PLAY 2>/dev/null
sleep 1
kill -INT $PID 2>/dev/null; sleep 4
pkill -INT -f "[s]ync_slam_toolbox_node"; pkill -INT -f "[p]ointlio_mapping"; pkill -INT -f "[r]eplay_mapping_launch"; sleep 2
ls -la "$(dirname "$OUT")" | tee -a "$LOG"
[ -s "$OUT.pgm" ] && echo "RESULT=OK" || echo "RESULT=FAIL"
