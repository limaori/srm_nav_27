#!/usr/bin/env bash
# GICP 重定位「手动挡」常驻会话：起栈 → 拉起被测节点 → 起评估器 → 保持运行。
#
# 与 run_session.sh 的区别：
#   * run_session.sh 是自动批测（矩阵驱动，逐轮重启被测节点）；
#   * 本脚本只负责把一切备好并**保持运行**，偏置由你手动发，方便反复试。
#
# 必须在**一个**后台作业里运行：本 sandbox 每次 bash 调用都是独立挂载命名空间，
# 一次调用结束后其中的子进程全部结束。
#
# 用法：
#   bash script/gicp_validation/run_session_manual.sh
# 然后（另开会话）按 README 里的命令发偏置，见脚本末尾打印的提示。

set -uo pipefail

WS=/home/srm/srm_nav_27
VD=$WS/tmp/gicp_validation
PARAMS=$VD/nav2_params_srm_test.yaml
PCD=$VD/promap.pcd
RUNTIME_CFG=$VD/validation_runtime.yaml
RUN_ID=$(date +%Y%m%dT%H%M%S)

[ -f "$PARAMS" ] || { echo "缺少参数副本 $PARAMS（先跑一次 run_session.sh）"; exit 1; }
[ -f "$PCD" ] || { echo "缺少先验点云 $PCD（先跑一次 run_session.sh screen/map）"; exit 1; }

export GICP_DOMAIN_ID=${GICP_DOMAIN_ID:-175}
set +u
source "$VD/env.sh"
set -u
cd "$WS"

echo "[manual-session] ROS_DOMAIN_ID=$ROS_DOMAIN_ID run_id=$RUN_ID"

cleanup() {
  echo "[manual-session] 清理进程"
  pkill -INT -f "srm_sim.launch.py" 2>/dev/null
  pkill -INT -f "validation_stack.launch.py" 2>/dev/null
  pkill -INT -f "gicp_reload.launch.py" 2>/dev/null
  pkill -INT -f "gicp_truth_evaluator" 2>/dev/null
  pkill -INT -f "ign gazebo" 2>/dev/null
  sleep 4
}
trap cleanup EXIT

# 0) 预检：停掉可能残留的 Gazebo server（Gazebo Transport 不区分 ROS 域）
timeout 12 ign service -s /server_control --reqtype ignition.msgs.ServerControl \
  --reptype ignition.msgs.Boolean --timeout 3000 --req 'stop: true' >/dev/null 2>&1 || true
sleep 3

# 1) 仿真 + 感知链路
echo "[manual-session] 启动仿真与感知链路"
bash script/gicp_validation/start_validation_stack.sh > "$VD/logs/manual_stack.log" 2>&1 &

READY_OUT=$("$WS/install/srm27_gicp_validation/bin/gicp_session_check" ready --timeout 240 2>/dev/null | tail -1)
echo "[manual-session] 就绪检查：$READY_OUT"
printf '%s' "$READY_OUT" | grep -q '"ready": true' || { echo "链路未就绪，退出"; exit 1; }

TF_OUT=""
for attempt in $(seq 1 30); do
  TF_OUT=$("$WS/install/srm27_gicp_validation/bin/gicp_session_check" tf --duration 3 2>/dev/null | tail -1)
  printf '%s' "$TF_OUT" | grep -q '"has_odom_base": true' && break
  sleep 3
done
echo "[manual-session] TF 检查（第 $attempt 次）：$TF_OUT"
printf '%s' "$TF_OUT" | grep -q '"has_odom_base": true' || { echo "缺少 odom→base_link，退出"; exit 1; }

# 2) 被测节点（常驻，日志里能看到每条 gicp_diag JSON）
echo "[manual-session] 拉起被测 GICP 节点"
ros2 launch srm27_gicp_validation gicp_reload.launch.py \
  namespace:=red_standard_robot1 \
  params_file:="$PARAMS" \
  prior_pcd_file:="$PCD" \
  use_sim_time:=True \
  run_id:="$RUN_ID" \
  init_pose:="[0.0,0.0,0.0,0.0,0.0,0.0]" \
  > "$VD/logs/manual_gicp.log" 2>&1 &

# 3) 评估器（常驻）
echo "[manual-session] 拉起真值评估器"
"$WS/install/srm27_gicp_validation/bin/gicp_truth_evaluator" \
  --ros-args \
  -p output_dir:="$VD/results" -p run_id:="$RUN_ID" \
  -p spawn_x:=3.4 -p spawn_y:=9.5 -p spawn_yaw:=0.0 \
  > "$VD/logs/manual_evaluator.log" 2>&1 &
sleep 5

# 4) 确认被测节点真的在工作（有诊断输出才算）
for _ in $(seq 1 40); do
  if grep -q "gicp_diag" "$VD/logs/manual_gicp.log" 2>/dev/null; then
    break
  fi
  sleep 2
done
if grep -q "gicp_diag" "$VD/logs/manual_gicp.log" 2>/dev/null; then
  echo "[manual-session] 被测节点已在输出诊断"
else
  echo "[manual-session] 警告：被测节点还没有诊断输出，检查 $VD/logs/manual_gicp.log"
  tail -15 "$VD/logs/manual_gicp.log"
fi

# 5) 可选：在同一个会话里自动跑一串偏置（沙箱下跨 bash 调用通不了 DDS，
#    所以「手动打命令」必须写进这里；在正常多终端环境里可以直接手敲命令）。
MANUAL_TOOL="$WS/install/srm27_gicp_validation/bin/gicp_manual_initial_pose"
SUMMARY=$VD/results/$RUN_ID/manual_summary.csv
if [ -n "${GICP_MANUAL_CASES:-}" ]; then
  echo "[manual-session] 按 GICP_MANUAL_CASES 依次注入偏置：$GICP_MANUAL_CASES"
  index=0
  IFS=';' read -ra CASES <<< "$GICP_MANUAL_CASES"
  for case in "${CASES[@]}"; do
    index=$((index + 1))
    set -- $case
    dx=${1:-0}; dy=${2:-0}; dyaw=${3:-0}
    echo ""
    echo "########## 偏置 $index/${#CASES[@]}：dx=$dx dy=$dy dyaw=${dyaw}°"
    # 数值统一带小数点，避免 ROS 参数把 0 解析成 INTEGER
    printf -v dxf "%.3f" "$dx"
    printf -v dyf "%.3f" "$dy"
    printf -v dyawf "%.3f" "$dyaw"
    timeout 90 "$MANUAL_TOOL" --config "$RUNTIME_CFG" --ros-args \
      -p dx:=$dxf -p dy:=$dyf -p dyaw_deg:=$dyawf \
      -p watch_seconds:=12 -p summary_csv:="$SUMMARY" 2>&1 | tail -12
    sleep 2
  done
  echo ""
  echo "########## 汇总"
  column -s, -t "$SUMMARY" 2>/dev/null || cat "$SUMMARY"
fi

cat <<EOF

================ 会话已就绪，保持运行中 ================
  被测节点日志（含逐次诊断 JSON）: $VD/logs/manual_gicp.log
  评估器日志                    : $VD/logs/manual_evaluator.log
  结果目录                      : $VD/results/$RUN_ID

  当前会话的 ROS_DOMAIN_ID = $ROS_DOMAIN_ID
  ⚠ 每次 bash 调用都是独立命名空间，**不要**另开 bash 调用去发偏置，
    请把它们写进同一个后台作业（见下面命令行）或者用 RViz 手点。

  手动发偏置（把 <...> 换成实际值）：
    $WS/install/srm27_gicp_validation/bin/gicp_manual_initial_pose \\
      --config $RUNTIME_CFG --ros-args \\
      -p dx:=0.5 -p dy:=0.0 -p dyaw_deg:=30 -p watch_seconds:=12 \\
      -p summary_csv:=$VD/results/$RUN_ID/manual_summary.csv

  RViz 手点：
    rviz2 -d $WS/src/srm27_navigation/srm27_nav_bringup/rviz/nav2_default_view.rviz
    （Fixed Frame 设为 map，用 "2D Pose Estimate" 点；点完看下面这条命令的输出）
=======================================================
EOF

wait
