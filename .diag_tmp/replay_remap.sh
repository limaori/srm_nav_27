#!/usr/bin/env bash
# 方案 A：用同一个包重放建图，slam_toolbox 收紧匹配阈值，重新存 2D 栅格图（不覆盖原图）。
set -o pipefail
cd /home/srm/srm_nav_27 || exit 1

WS=$PWD
BAG=$HOME/srm_nav_27_data/bags/mapping_bag_202to227_20261124_223712
OUT_DIR=$WS/maps/202to227_replay
OUT_PREFIX=$OUT_DIR/202to227_replay
LOG=$WS/.diag_tmp/replay_remap.log
PARAMS=$WS/.diag_tmp/replay_params.yaml
LAUNCH_FILE=$WS/.diag_tmp/replay_mapping_launch.py

mkdir -p "$OUT_DIR"
: > "$LOG"

# 必须用 env_srm27.sh：它剔除 /opt/MVS 的老 libusb（否则 PCL 节点 exit 127），
# 并把 Livox SDK 的 .so 目录补进 LD_LIBRARY_PATH（install/ 里没有这份库）。
source "$WS/scripts/setup/env_srm27.sh" >> "$LOG" 2>&1
export ROS_DOMAIN_ID=77   # 独立域：避开机器上其它/残留的同名 slam_toolbox
export ROS_LOG_DIR="$WS/.tmp_roslog"
mkdir -p "$ROS_LOG_DIR"

# ROS 2 CLI 的 daemon 会缓存旧图；不杀干净时 node list / param get 会返回上一轮
# （甚至机器上别的会话）的幽灵节点，表现为"scan_topic 突然变成 obstacle_scan"。
ros2 daemon stop >/dev/null 2>&1
sleep 2

echo "[1/5] 起建图链路 (point_lio + slam_toolbox, tuned params)" | tee -a "$LOG"
ros2 launch "$LAUNCH_FILE" \
  start_lidar:=False use_sim_time:=True use_rviz:=False save_pcd:=False \
  lio_frontend:=point_lio params_file:="$PARAMS" >> "$LOG" 2>&1 &
LAUNCH_PID=$!

ok=0
for i in $(seq 1 45); do
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "[错误] 链路启动就退出，日志尾部:" | tee -a "$LOG"; tail -25 "$LOG"; exit 1
  fi
  if ros2 node list --no-daemon 2>/dev/null | grep -qx "/replay_slam" \
     && ros2 node list --no-daemon 2>/dev/null | grep -qx "/point_lio" \
     && ros2 topic info /livox/lidar 2>/dev/null | grep -q "Subscription count: [1-9]" \
     && ros2 topic info /livox/imu 2>/dev/null | grep -q "Subscription count: [1-9]" \
     && ros2 service list 2>/dev/null | grep -qx "/slam_toolbox/save_map"; then
    ok=1; echo "[2/5] 链路就绪 (${i}0s 内)" | tee -a "$LOG"; break
  fi
  sleep 2
done
if [ "$ok" != "1" ]; then
  echo "[错误] 等了 90 s 链路没就绪，日志尾部:" | tee -a "$LOG"; tail -25 "$LOG"
  kill -INT "$LAUNCH_PID" 2>/dev/null; sleep 3; exit 1
fi
sleep 3
echo "[2b] 链路自检:" | tee -a "$LOG"
ros2 node list --no-daemon 2>/dev/null | grep -E "replay_slam|slam_toolbox|point_lio|pointcloud_to_laserscan" | tee -a "$LOG"
SC_TOPIC=$(ros2 param get /replay_slam scan_topic 2>/dev/null | tail -1)
echo "     scan_topic = $SC_TOPIC" | tee -a "$LOG"
ros2 topic info /scan 2>/dev/null | tee -a "$LOG"
case "$SC_TOPIC" in
  *"/scan"*) ;;
  *)
    echo "[错误] slam_toolbox 的 scan_topic 不是 /scan（$SC_TOPIC）——图里有同名旧节点在抢，先清掉再跑。" | tee -a "$LOG"
    kill -INT "$LAUNCH_PID" 2>/dev/null; sleep 4
    pkill -INT -f "[s]ync_slam_toolbox_node"; pkill -INT -f "[p]ointlio_mapping"; pkill -INT -f "[r]eal_mapping_launch\.py"
    exit 1 ;;
esac

echo "[3/5] 放包 1x (约 220 s): $BAG" | tee -a "$LOG"
ros2 bag play "$BAG" --clock --topics /livox/lidar /livox/imu --rate 1.0 >> "$LOG" 2>&1 &
PLAY_PID=$!
sleep 70
echo "[3b] 播放中自检 (t≈70s):" | tee -a "$LOG"
timeout 8 ros2 topic echo /map --once --field info.width 2>/dev/null | tee -a "$LOG"
timeout 8 ros2 topic hz /scan --window 10 2>/dev/null | head -2 | tee -a "$LOG"
wait $PLAY_PID
echo "      bag 播放结束" | tee -a "$LOG"
sleep 6

echo "[4/5] 存 2D 图 -> $OUT_PREFIX.{pgm,yaml}" | tee -a "$LOG"
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: '$OUT_PREFIX'}}" >> "$LOG" 2>&1
if [ ! -s "$OUT_PREFIX.pgm" ]; then
  echo "      save_map 没出图, 用 map_saver_cli 兜底" | tee -a "$LOG"
  timeout 30 ros2 run nav2_map_server map_saver_cli -f "$OUT_PREFIX" >> "$LOG" 2>&1
fi
sleep 3

echo "[5/5] 收尾" | tee -a "$LOG"
pkill -INT -u "$(id -u)" -f "[s]ync_slam_toolbox_node" 2>/dev/null
kill -INT "$LAUNCH_PID" 2>/dev/null
sleep 6
pkill -INT -u "$(id -u)" -f "[p]ointlio_mapping" 2>/dev/null
pkill -INT -u "$(id -u)" -f "[r]eal_mapping_launch\.py" 2>/dev/null
sleep 3

ls -la "$OUT_DIR" | tee -a "$LOG"
if [ -s "$OUT_PREFIX.pgm" ] && [ -s "$OUT_PREFIX.yaml" ]; then
  echo "RESULT=OK $OUT_PREFIX" | tee -a "$LOG"
else
  echo "RESULT=FAIL" | tee -a "$LOG"
fi
