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
    # 数值比较，不能用字符串（"%s" 与 "%.6f" 形态不同，"2.0" 会一直被误判成超限）。
    if ! awk -v a="$clamped" -v b="$rate" 'BEGIN{ exit (a == b) ? 0 : 1 }'; then
      echo "[提示] ${rate} rad/s 超出本地钳位 ±${MAX_WZ}，按 ${clamped} rad/s 下发。"
    fi
    _ws_setup
    stop_publisher
    # 后台常发 (20 Hz): 只要本进程活着请求就"新鲜"；kill 即撤回。
    nohup ros2 topic pub --rate 20 "$CMD_TOPIC" geometry_msgs/msg/Twist \
      "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: ${clamped}}}" \
      >/dev/null 2>&1 &
    echo $! > "$PID_FILE"
    subs="$(timeout 5 ros2 topic info "$CMD_TOPIC" 2>/dev/null | sed -n 's/^Subscription count: //p' || true)"
    echo "[自转] 请求 ${clamped} rad/s 已发布 (PID $(cat "$PID_FILE"))，minco spin_request 应在 0.3 s 内变新鲜。"
    if [ -z "${subs:-}" ]; then
      echo "[警告] 查不到 ${CMD_TOPIC} 的订阅者 —— 栈没起 / 域不同 / ros2 daemon 缓存旧信息。"
    elif [ "$subs" = "0" ]; then
      cat <<WARN
[警告] ${CMD_TOPIC} 现在**没有任何订阅者**，请求发出去也没人听。常见原因：
        · MINCO 的 spin_request.topic 不是 rotation_cmd（用 -c minco 或 -p …minco.yaml 时
          会退回代码默认值 rotation_velocity）
        · 命名空间不一致（本脚本 ROBOT_NS=${NS}）
        · 栈和本终端不在同一个 ROS_DOMAIN_ID
       逐条确认: $0 status
WARN
    else
      echo "[提示] ${CMD_TOPIC} 订阅者 ${subs} 个（MINCO 在听）。停止: $0 off"
    fi
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
    echo "===== 1) 模式与话题（决定请求会不会被消费） ====="
    # nav2 节点在机器人命名空间下（/red_standard_robot1/controller_server）；根命名空间的
    # 部署才用 /controller_server。两种都试，否则永远"读不到"。
    ctrl_node=""
    for candidate in "/${NS}/controller_server" "/controller_server"; do
      if timeout 8 ros2 param get "$candidate" FollowPath.plugin >/dev/null 2>&1; then
        ctrl_node="$candidate"; break
      fi
    done
    if [ -n "$ctrl_node" ]; then
      echo "  controller_server        = ${ctrl_node}"
      mode="$(timeout 8 ros2 param get "$ctrl_node" FollowPath.yaw_policy.mode 2>/dev/null | sed 's/^String value is: //' || true)"
      topic="$(timeout 8 ros2 param get "$ctrl_node" FollowPath.spin_request.topic 2>/dev/null | sed 's/^String value is: //' || true)"
    else
      echo "  controller_server        = <不在线>  ← Nav2 没起来，请求一定没人消费"
      mode=""; topic=""
    fi
    wzmax="$(timeout 8 ros2 param get "/${NS}/srm_cmd_mux" wz_max 2>/dev/null | sed 's/^Double value is: //' || true)"
    echo "  yaw_policy.mode        = ${mode:-<读不到>}   # 必须 spin"
    echo "  spin_request.topic     = ${topic:-<读不到>}   # 必须 rotation_cmd"
    echo "  srm_cmd_mux.wz_max     = ${wzmax:-<读不到>}"

    echo "===== 2) 请求是否真的发出去了 ====="
    echo -n "  话题订阅者: "
    subs_info="$(timeout 5 ros2 topic info "$CMD_TOPIC" 2>/dev/null | sed -n 's/^Subscription count: //p' || true)"
    echo "${subs_info:-查不到}"
    if [ "${subs_info:-0}" = "0" ]; then
      echo "  ⚠ 没人在听。检查 MINCO 是不是在听旧话题名:"
      for alt in rotation_velocity; do
        alt_subs="$(timeout 5 ros2 topic info "/${NS}/${alt}" 2>/dev/null | sed -n 's/^Subscription count: //p' || true)"
        [ -n "${alt_subs:-}" ] && [ "${alt_subs:-0}" != "0" ] \
          && echo "    /${NS}/${alt} 订阅者 ${alt_subs} 个 ← MINCO 在这里听着（spin_request.topic 没配成 rotation_cmd）"
      done
      echo "  也确认 Nav2 是否在跑: ros2 node list | grep -E 'controller_server|planner_server|bt_navigator'"
    fi
    timeout 3 ros2 topic echo "$CMD_TOPIC" --once 2>/dev/null | grep -A3 "^angular" || echo "  (无发布 —— 请求未发出，或 $0 on 的后台进程已死)"

    echo "===== 3) MINCO 有没有消费（目标状态看这里） ====="
    timeout 4 ros2 topic echo "/${NS}/FollowPath/diagnostics" --once 2>/dev/null \
      | grep -A1 -E "key: (yaw_mode|spin_request|spin_request_fresh|applied_wz|rejected_spin_request_count|planning_result|stop_reason)" \
      | grep -E "key:|value:" | paste - - | sed 's/key: //; s/[[:space:]]*value: /=/' | sed 's/^/  /' \
      || echo "  (诊断未收到 —— 没目标时控制器不跑，或 yaw_mode 不是 spin)"

    echo "===== 4) 三层速度（wz 应逐层相同、非零） ====="
    for layer in cmd_vel_nav_fake cmd_vel_nav_body cmd_vel_sim; do
      wz="$(timeout 3 ros2 topic echo "/${NS}/${layer}" --once 2>/dev/null \
            | awk '/^angular:/{f=1} f&&/z:/{print $2; exit}' || true)"
      echo "  ${layer}  wz = ${wz:-<无数据>}"
    done

    echo "===== 5) 发布进程 ====="
    if [ -f "$PID_FILE" ] && kill -0 "$(cat "$PID_FILE" 2>/dev/null)" 2>/dev/null; then
      echo "  活着 (PID $(cat "$PID_FILE"))"
    else
      echo "  不在（请求已撤回）"
    fi
    ;;
  *)
    echo "用法: $0 on <rad/s> | off | status" >&2
    exit 2
    ;;
esac
