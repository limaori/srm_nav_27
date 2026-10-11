#!/usr/bin/env bash
# 先验地图采集（需要 start_validation_stack.sh 已在后台运行）。
#
# 两段式：先记录「雷达系点云 + 对应真值位姿」到 npy 会话目录，再用
# tools/rebuild_prior_map.py 离线重建 PCD / 栅格 / 清单。这样采集回调里没有重活，
# 不会把点云话题速率拖垮。
#
# 用法： bash script/gicp_validation/run_prior_map_capture.sh [seconds] [route] [map_name]
set -uo pipefail

WS=/home/srm/srm_nav_27
VD=$WS/tmp/gicp_validation
CAPTURE_SECONDS=${1:-60}
ROUTE=${2:-map_slow}
MAP_NAME=${3:-promap}
SESSION=$VD/map_session

set +u
source "$VD/env.sh"
set -u
cd "$WS"

rm -rf "$SESSION"
mkdir -p "$SESSION"

echo "[capture] 等待真值就绪"
for _ in $(seq 1 60); do
  if timeout 8 ros2 topic echo /red_standard_robot1/chassis_odometry_gt --once >/dev/null 2>&1; then
    break
  fi
  sleep 1
done

echo "[capture] 记录 ${CAPTURE_SECONDS}s（路线 ${ROUTE}）"
timeout $((CAPTURE_SECONDS + 60)) "$WS/install/srm27_gicp_validation/bin/gicp_map_recorder" \
  --ros-args \
  -p session_dir:="$SESSION" \
  -p capture_seconds:="$CAPTURE_SECONDS" -p start_delay:=5 \
  -p max_odom_gap:=0.08 -p point_stride:=4 \
  > "$VD/logs/map_recorder.log" 2>&1 &
RECORDER_PID=$!

sleep 6
timeout $((CAPTURE_SECONDS + 30)) "$WS/install/srm27_gicp_validation/bin/gicp_cmd_vel_script" \
  --ros-args -p route:="$ROUTE" > "$VD/logs/cmd_vel_map.log" 2>&1 || true

wait "$RECORDER_PID" 2>/dev/null
echo "[capture] 采集结束："
tail -3 "$VD/logs/map_recorder.log"

echo "[capture] 离线重建地图"
python3 "$WS/src/srm27_navigation/srm27_gicp_validation/tools/rebuild_prior_map.py" \
  --session "$SESSION" --output-dir "$VD" --map-name "$MAP_NAME" \
  --leaf-size 0.10 --grid-resolution 0.05 --grid-min-z 0.15 --grid-max-z 1.6 \
  --spawn-x 3.4 --spawn-y 9.5 --spawn-yaw 0.0 \
  --world-name rmuc_2025 --world-sdf "$VD/worlds/rmuc_2025_ogre2.sdf" \
  2>&1 | tail -8
ls -la "$VD"/${MAP_NAME}* 2>/dev/null
