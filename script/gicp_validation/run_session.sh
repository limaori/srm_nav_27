#!/usr/bin/env bash
# 整场 GICP 初始重定位验证会话（方案 §5–§8）的编排脚本。
#
# 必须在**一个**后台作业里运行：DSH 的每次 bash 调用都是独立 PID 命名空间，
# 一次调用结束后其中的子进程随之结束，所以「起栈 → 建图 → 逐轮测试」不能拆到
# 多次调用里。
#
# 阶段：
#   map      仅建先验地图
#   screen   建图（若缺）+ 航向快速筛查
#   s0       建图（若缺）+ S0 回归
#   full     建图（若缺）+ 配置里指定的全量矩阵
#   <路径>   直接把参数当作矩阵文件路径
#
# 用法： bash script/gicp_validation/run_session.sh <phase> [extra runner args...]
set -uo pipefail

WS=/home/srm/srm_nav_27
VD=$WS/tmp/gicp_validation
PHASE=${1:-screen}
shift || true

mkdir -p "$VD/logs" "$VD/results" "$VD/worlds"

# 每次会话用一个独立的 ROS_DOMAIN_ID：DSH 的 bash 调用各自是独立 PID 命名空间，
# 跨调用无法用 pkill 精确清理残留节点（只能看到自己命名空间里的进程）。
# 换域可以让上一次残留的孤儿节点完全不参与本会话的图，保证 TF 只有一个发布者。
ROS_DOMAIN_ID=${GICP_DOMAIN_ID:-$(( (RANDOM % 100) + 100 ))}
export ROS_DOMAIN_ID
echo "[session] ROS_DOMAIN_ID=$ROS_DOMAIN_ID"

set +u
source "$VD/env.sh"
set -u
cd "$WS"

RUN_ID=$(date +%Y%m%dT%H%M%S)
export GICP_RUN_ID="$RUN_ID"
echo "[session] phase=$PHASE run_id=$RUN_ID"

# ---------------------------------------------------------------- 0. 预检
# Gazebo 通过 Gazebo Transport（不依赖 ROS 域）通信：先停掉可能残留的 server，
# 否则会出现重复的 clock/物理世界，TF 树也会出现重复发布者。
echo "[session] 预检：停掉可能残留的 Gazebo server"
timeout 12 ign service -s /server_control --reqtype ignition.msgs.ServerControl   --reptype ignition.msgs.Boolean --timeout 3000 --req 'stop: true' >/dev/null 2>&1 || true
sleep 3

cleanup() {
  echo "[session] 清理"
  pkill -INT -f "srm_sim.launch.py" 2>/dev/null
  pkill -INT -f "ign gazebo" 2>/dev/null
  pkill -INT -f "validation_stack.launch.py" 2>/dev/null
  pkill -INT -f "gicp_reload.launch.py" 2>/dev/null
  pkill -INT -f "gicp_truth_evaluator" 2>/dev/null
  pkill -INT -f "pointlio_mapping" 2>/dev/null
  sleep 4
}
trap cleanup EXIT

# ---------------------------------------------------------------- 1. 起栈
echo "[session] 启动仿真与感知链路"
bash script/gicp_validation/start_validation_stack.sh > "$VD/logs/stack_session.log" 2>&1 &
STACK_PID=$!

# 就绪判据不能只看「话题存在」：loam_interface 在 TF 未就绪时只广告不发布。
# 这里要求真值里程计与 registered_scan 都真的能收到消息。
READY_OUT=$("$WS/install/srm27_gicp_validation/bin/gicp_session_check" ready --timeout 240 2>/dev/null | tail -1)
echo "[session] 就绪检查：$READY_OUT"
if ! printf '%s' "$READY_OUT" | grep -q '"ready": true'; then
  echo "[session] 致命：链路未就绪，保留日志供排查"
  tail -30 "$VD/logs/gz.log" 2>/dev/null
  tail -30 "$VD/logs/perception.log" 2>/dev/null
  exit 1
fi

# TF 检查（方案 §3.2）：odom->base_link 必须存在且唯一。
# 注意要重试：odom→base_link 要等 registered_scan 提供者收到第一帧里程计后才出现，
# 而 registered_scan 也要等 Point-LIO 初始化完成，启动后约 5-30 s 才齐。
tf_ok=0
for attempt in $(seq 1 30); do
  TF_OUT=$("$WS/install/srm27_gicp_validation/bin/gicp_session_check" tf --duration 3 2>/dev/null | tail -1)
  if printf '%s' "$TF_OUT" | grep -q '"has_odom_base": true'; then
    tf_ok=1
    break
  fi
  sleep 3
done
echo "[session] TF 检查（第 $attempt 次）：$TF_OUT"
if [ "$tf_ok" != "1" ]; then
  echo "[session] 致命：缺少 odom->base_link"
  grep -vE "jump back" "$VD/logs/perception.log" | tail -20
  exit 1
fi

# ---------------------------------------------------------------- 2. 建图
if [ "$PHASE" != "nomap" ] && [ ! -f "$VD/promap.pcd" ]; then
  echo "[session] 采集先验地图"
  bash script/gicp_validation/run_prior_map_capture.sh 60 map_slow promap || true
fi

if [ "$PHASE" = "map" ]; then
  echo "[session] 仅建图，结束"
  exit 0
fi

if [ ! -f "$VD/promap.pcd" ]; then
  echo "[session] 致命：缺少先验点云 $VD/promap.pcd，无法继续"
  exit 1
fi

# ---------------------------------------------------------------- 3. 评估器
echo "[session] 启动真值评估器"
"$WS/install/srm27_gicp_validation/bin/gicp_truth_evaluator" \
  --ros-args \
  -p output_dir:="$VD/results" -p run_id:="$RUN_ID" \
  -p spawn_x:=3.4 -p spawn_y:=9.5 -p spawn_yaw:=0.0 \
  > "$VD/logs/evaluator.log" 2>&1 &
sleep 4

# ---------------------------------------------------------------- 4. 矩阵
case "$PHASE" in
  screen) MATRIX="$VD/matrices/quick_screen.yaml" ;;
  s0)     MATRIX="$VD/matrices/s0_regression.yaml" ;;
  s1)     MATRIX="$VD/matrices/s1_core.yaml" ;;
  full)   MATRIX="$VD/matrices/s1_full.yaml" ;;
  *)      MATRIX="$PHASE" ;;
esac

echo "[session] 运行试验矩阵 $MATRIX"
"$WS/install/srm27_gicp_validation/bin/gicp_initialization_runner" \
  --config "$VD/validation_runtime.yaml" \
  --matrix "$MATRIX" \
  --output-dir "$VD/results" \
  --run-id "$RUN_ID" \
  "$@" 2>&1 | tee "$VD/logs/runner_$RUN_ID.log"

echo "[session] 试验结束：results/$RUN_ID"
