#!/usr/bin/env bash
# 用包里的「密点云」重建 2D 图：CustomMsg -> PointCloud2 -> pointcloud_to_laserscan -> slam_toolbox
set -o pipefail
cd /home/srm/srm_nav_27 || exit 1
WS=$PWD
BAG=$HOME/srm_nav_27_data/bags/mapping_bag_202to227_20261124_223712
OUT=$WS/maps/202to227_replay/202to227_dense
LOG=$WS/.diag_tmp/replay_dense.log
source "$WS/scripts/setup/env_srm27.sh" >/dev/null 2>&1
export ROS_DOMAIN_ID=78
export ROS_LOG_DIR=$WS/.tmp_roslog
ros2 daemon stop >/dev/null 2>&1; sleep 2
mkdir -p "$(dirname "$OUT")"
: > "$LOG"

echo "[1/6] 起转换节点 (CustomMsg -> PointCloud2)" | tee -a "$LOG"
python3 "$WS/.diag_tmp/custommsg_to_pointcloud2.py" > "$WS/.diag_tmp/conv.log" 2>&1 &
CONV=$!

echo "[2/6] 起建图链路 (scan_cloud_topic=/livox/lidar/pointcloud)" | tee -a "$LOG"
ros2 launch "$WS/.diag_tmp/replay_mapping_launch.py" \
  start_lidar:=False use_sim_time:=True use_rviz:=False save_pcd:=False \
  lio_frontend:=point_lio params_file:="$WS/.diag_tmp/replay_params.yaml" \
  scan_cloud_topic:=/livox/lidar/pointcloud >> "$LOG" 2>&1 &
PID=$!

for i in $(seq 1 40); do ros2 node list --no-daemon 2>/dev/null | grep -qx "/replay_slam" && break; sleep 1; done
sleep 4
echo "[3/6] 自检:" | tee -a "$LOG"
echo "  scan_topic = $(ros2 param get /replay_slam scan_topic 2>&1 | tail -1)" | tee -a "$LOG"
echo "  pointcloud_to_laserscan 订阅: $(ros2 topic info /livox/lidar/pointcloud 2>/dev/null | tr '\n' ' ')" | tee -a "$LOG"
echo "  slam 匹配参数 = $(ros2 param get /replay_slam link_match_minimum_response_fine 2>&1 | tail -1)" | tee -a "$LOG"

echo "[4/6] 放包 (220 s)" | tee -a "$LOG"
ros2 bag play "$BAG" --clock --topics /livox/lidar /livox/imu --rate 1.0 >> "$LOG" 2>&1 &
PLAY=$!
sleep 60
echo "[5/6] 播放 60s 时: /scan 频率" | tee -a "$LOG"
timeout 10 ros2 topic hz /scan --window 20 >> "$LOG" 2>&1
sleep 140
echo "[5b] 播放 200s: /map 频率 + 存图" | tee -a "$LOG"
timeout 8 ros2 topic hz /map --window 5 >> "$LOG" 2>&1
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '$OUT'}}" >> "$LOG" 2>&1
if [ ! -s "$OUT.pgm" ]; then
  echo "  service 没出图, map_saver_cli 兜底" | tee -a "$LOG"
  timeout 40 ros2 run nav2_map_server map_saver_cli -f "$OUT" >> "$LOG" 2>&1
fi
kill -INT $PLAY 2>/dev/null; sleep 1
kill -INT $CONV 2>/dev/null
echo "[6/6] 收尾" | tee -a "$LOG"
kill -INT $PID 2>/dev/null; sleep 4
pkill -INT -f "[s]ync_slam_toolbox_node"; pkill -INT -f "[p]ointlio_mapping"; pkill -INT -f "[r]eplay_mapping_launch"; sleep 2
echo "=== 转换节点统计:" | tee -a "$LOG"; tail -3 "$WS/.diag_tmp/conv.log" | tee -a "$LOG"
ls -la "$(dirname "$OUT")" | tee -a "$LOG"
[ -s "$OUT.pgm" ] && echo "RESULT=OK $OUT" || echo "RESULT=FAIL"
