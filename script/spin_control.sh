#!/usr/bin/env bash
# 手动控制小陀螺（阶段二，方案 §4.1.2 的自转请求进线）。
#
# 原理: 本脚本向 /<ns>/rotation_cmd 发转速请求 -> MINCO(yaw_policy.mode=spin)
#       setSpinRequest -> 导航输出 wz -> fake_vel_transform -> srm_cmd_mux -> 底盘。
#       停发超过 spin_request.timeout(默认 0.25 s) 后 MINCO 按撤回处理，车停止自转。
#
# 2026-10-19: 独立自转链（rotation_controller / rotation_velocity）已删除，请求由
#       上位机直接进 MINCO。原来的 wz_max(2.0) 限幅改由本脚本在本地下发前钳位
#       （--max-wz，默认 2.0；MINCO / mux 侧仍有各自的 wz 上限兜底）。
#       wz 始终由导航产生，符合"上位机控制 wz"的唯一权威。
#
# 前置:
#   1) 导航栈用小陀螺模式启动:  ./script/start_sim_nav.sh --yaw-mode spin ...
#      (yaw_mode=se2 时请求会被航向策略忽略，车不会转 —— 这是迁移契约，不是故障)
#   2) 在已 source install/setup.bash 的终端运行 (或设 SRM27_WS_DIR)。
#
# 用法:
#   script/spin_control.sh on <rad/s>     恒速自转, 如: script/spin_control.sh on 2.0
#   script/spin_control.sh off            停止自转（撤回请求）
#   script/spin_control.sh status         查看请求/实际转速/进线状态
#
# 环境变量: ROBOT_NS (默认 red_standard_robot1), SPIN_MAX_WZ (默认 2.0 rad/s)
set -euo pipefail

NS="${ROBOT_NS:-red_standard_robot1}"
CMD_TOPIC="/${NS}/rotation_cmd"
PID_FILE="${TMPDIR:-/tmp}/srm27_spin_control_${NS}.pid"
MAX_WZ="${SPIN_MAX_WZ:-2.0}"

_ws_setup() {
  local ws="${SRM27_WS_DIR:-$HOME/srm_nav_27}"
  if [ -f "$ws/install/setup.bash" ]; then
    # colcon 的 setup.bash 引用未定义变量（COLCON_TRACE 等），须在 set -u 下临时放开。
    set +u
    # shellcheck disable=SC1091
    source "$ws/install/setup.bash"
    set -u
  fi
}

stop_publisher() {
  if [ -f "$PID_FILE" ]; then
    local pid
    pid="$(cat "$PID_FILE" 2>/dev/null || true)"
    if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
      kill "$pid" 2>/dev/null || true
      sleep 0.3
    fi
    rm -f "$PID_FILE"
  fi
  # 兜底: 清掉本机残留的 rotation_cmd 发布进程 (按话题名匹配，误伤面极小)。
  pkill -f "topic pub.*${CMD_TOPIC}" 2>/dev/null || true
}

# 把请求钳到 ±MAX_WZ（原 rotation_controller.wz_max 的职责）。
_clamp() {
  awk -v v="$1" -v m="$MAX_WZ" 'BEGIN{
    if (m < 0) m = -m;
    if (v > m) v = m;
    if (v < -m) v = -m;
    printf "%.6f", v;
  }'
}

cmd="${1:-status}"
case "$cmd" in
  on)
    rate="${2:?用法: spin_control.sh on <rad/s>}"
    case "$rate" in ''|*[!0-9.eE+-]*) echo "[错误] 转速必须是数字 (rad/s): $rate" >&2; exit 2 ;; esac
    clamped="$(_clamp "$rate")"
    if [ "$clamped" != "$rate" ]; then
      echo "[提示] ${rate} rad/s 超出本地钳位 ±${MAX_WZ}，按 ${clamped} rad/s 下发。"
    fi
    _ws_setup
    stop_publisher
    # 后台常发 (20 Hz): 只要本进程活着请求就"新鲜"；kill 即撤回。
    nohup ros2 topic pub --rate 20 "$CMD_TOPIC" geometry_msgs/msg/Twist \
      "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: ${clamped}}}" \
      >/dev/null 2>&1 &
    echo $! > "$PID_FILE"
    echo "[自转] 请求 ${clamped} rad/s 已发布 (PID $(cat "$PID_FILE"))，minco spin_request 应在 0.3 s 内变新鲜。"
    echo "[提示] 停止: $0 off"
    ;;
  off)
    _ws_setup
    stop_publisher
    # 发一条零速，立刻撤回（否则要等 MINCO 的 spin_request.timeout 0.25 s）。
    timeout 3 ros2 topic pub --once "$CMD_TOPIC" geometry_msgs/msg/Twist \
      "{angular: {z: 0.0}}" >/dev/null 2>&1 || true
    echo "[自转] 已停止（请求撤回 + 零速）。"
    ;;
  status)
    _ws_setup
    echo "== rotation_cmd (上位机自转请求) =="
    timeout 3 ros2 topic echo "$CMD_TOPIC" --once 2>/dev/null | grep -A3 "^angular" || echo "  (无发布 —— 请求未发出)"
    echo "== MINCO 进线状态 =="
    timeout 3 ros2 topic echo "/${NS}/FollowPath/diagnostics" 2>/dev/null \
      | grep -A1 -E "key: (yaw_mode|spin_request|spin_request_fresh|applied_wz)" \
      | grep -E "key:|value:" | head -8 || echo "  (诊断未收到 —— 栈没起或 yaw_mode 不是 spin)"
    echo "== 实际下发 (cmd_vel_sim) =="
    timeout 3 ros2 topic echo "/${NS}/cmd_vel_sim" --once 2>/dev/null | grep -A3 "^angular" || echo "  (无数据)"
    if [ -f "$PID_FILE" ] && kill -0 "$(cat "$PID_FILE" 2>/dev/null)" 2>/dev/null; then
      echo "== 发布进程: 活着 (PID $(cat "$PID_FILE")) =="
    else
      echo "== 发布进程: 不在 (请求已撤回) =="
    fi
    ;;
  *)
    echo "用法: $0 on <rad/s> | off | status" >&2
    exit 2
    ;;
esac
