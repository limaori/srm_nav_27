#!/usr/bin/env bash
# 手动控制迁移链自转（阶段二小陀螺，方案 §4.1.2 的自转请求进线）。
#
# 原理: 本脚本向 /<ns>/rotation_cmd 发转速请求 -> rotation_controller(限幅/斜率)
#       -> rotation_velocity -> MINCO(yaw_policy.mode=spin) setSpinRequest -> 导航 wz。
#       停发超过 spin_request.timeout(默认 0.25 s) 后 MINCO 按撤回处理，车停止自转。
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
# 环境变量: ROBOT_NS (默认 red_standard_robot1)
set -euo pipefail

NS="${ROBOT_NS:-red_standard_robot1}"
CMD_TOPIC="/${NS}/rotation_cmd"
PID_FILE="${TMPDIR:-/tmp}/srm27_spin_control_${NS}.pid"

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

cmd="${1:-status}"
case "$cmd" in
  on)
    rate="${2:?用法: spin_control.sh on <rad/s>}"
    case "$rate" in ''|*[!0-9.eE+-]*) echo "[错误] 转速必须是数字 (rad/s): $rate" >&2; exit 2 ;; esac
    _ws_setup
    stop_publisher
    # 后台常发 (20 Hz): 只要本进程活着请求就"新鲜"；kill 即撤回。
    nohup ros2 topic pub --rate 20 "$CMD_TOPIC" geometry_msgs/msg/Twist \
      "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: ${rate}}}" \
      >/dev/null 2>&1 &
    echo $! > "$PID_FILE"
    if awk -v r="$rate" 'BEGIN{exit !(r<0||r>4)}'; then
      echo "[警告] |${rate}| rad/s 超过 mux 自转上限 4.0，实际会被钳到 4.0。"
    fi
    echo "[自转] 请求 ${rate} rad/s 已发布 (PID $(cat "$PID_FILE"))，minco spin_request 应在 0.3 s 内变新鲜。"
    echo "[提示] 停止: $0 off"
    ;;
  off)
    _ws_setup
    stop_publisher
    # 发一条零速，立刻生效（否则要等 rotation_controller 的 request_timeout 0.5 s）。
    timeout 3 ros2 topic pub --once "$CMD_TOPIC" geometry_msgs/msg/Twist \
      "{angular: {z: 0.0}}" >/dev/null 2>&1 || true
    echo "[自转] 已停止（请求撤回 + 零速）。"
    ;;
  status)
    _ws_setup
    echo "== rotation_cmd (手动请求) =="
    timeout 3 ros2 topic echo "$CMD_TOPIC" --once 2>/dev/null | grep -A3 "^angular" || echo "  (无发布 —— 请求未发出)"
    echo "== rotation_velocity (限幅后) =="
    timeout 3 ros2 topic echo "/${NS}/rotation_velocity" --once 2>/dev/null | grep -A3 "^angular" || echo "  (无数据)"
    echo "== MINCO 进线状态 =="
    timeout 3 ros2 topic echo "/${NS}/FollowPath/diagnostics" 2>/dev/null \
      | grep -A1 -E "key: (yaw_mode|spin_request|spin_request_fresh|applied_wz)" \
      | grep -E "key:|value:" | head -8 || echo "  (诊断未收到 —— 栈没起或 yaw_mode 不是 spin)"
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
