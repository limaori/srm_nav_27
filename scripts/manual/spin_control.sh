#!/usr/bin/env bash
# 手动控制小陀螺（阶段二，方案 §4.1.2 的自转请求进线）。
#
# 原理: 本脚本向 /<ns>/rotation_cmd 发转速请求。
#   2026-11-12 起（云台系，仿真 / 实车同一套）：
#       rotation_cmd -> srm_cmd_mux(spin_topic，只取 angular.z) -> cmd_vel_sim/chassis
#       与导航**完全解耦**：请求不进 MINCO（xy_only 会拒绝并计数），停发
#       mux 的 spin_timeout(默认 0.25 s) 后车停止自转；没有导航目标时也照转。
#   旧链（yaw_policy.mode=spin，仅 A/B 回退时）：
#       rotation_cmd -> MINCO setSpinRequest -> 导航输出 wz -> fake -> mux -> 底盘。
#
# 2026-10-19: 独立自转链（rotation_controller / rotation_velocity）已删除，请求由
#       上位机直接发。原来的 wz_max(2.0) 限幅改由本脚本在本地下发前钳位
#       （--max-wz，默认 2.0；mux 的 wz_max / wz_ramp 仍会兜底）。
#
# 前置:
#   1) 导航栈已经起来（仿真 / 实车脚本的默认 xy_only 即可，不需要 --yaw-mode spin）。
#   2) 在已 source install/setup.bash 的终端运行 (或设 SRM27_WS_DIR)。
#
# 用法:
#   script/spin_control.sh on <rad/s>     恒速自转, 如: script/spin_control.sh on 2.0
#   script/spin_control.sh sine <max> <min> <period>
#                                         正弦自转: wz(t)=mid+amp*sin(2*pi*t/T)
#                                         例: spin_control.sh sine 0.5 -0.3 4
#   script/spin_control.sh off            停止自转（撤回请求）
#   script/spin_control.sh status         查看请求/实际转速/进线状态
#
# 环境变量: ROBOT_NS (默认 red_standard_robot1；**设为空串表示根命名空间 = 实车默认**),
#           SPIN_MAX_WZ (默认 2.0 rad/s)
#           SPIN_CMD_TOPIC  自转请求发到哪个话题。默认 <ns>/rotation_cmd ——
#           start_sim_nav.sh --chassis-spin 与 start_real_nav.sh --chassis-spin 用的都是它
#           （云台系：srm_cmd_mux 在听；旧 spin 模式：MINCO 也在听）。
#           start_sim_slam.sh --chassis-spin 会把它指到 <ns>/cmd_vel_nav_fake：
#           建图栈没有 MINCO / mux（rotation_cmd 无人订阅），只能从 fakeyaw 链入口注入，
#           经 fake_vel_transform 到底盘。
#           注意 status 的 1)/3) 段是 MINCO 专用（它的参数与诊断）；云台系下 MINCO
#           只会拒绝自转请求，**真实执行看第 4 段**（mux 输出 / 里程计）。
#           SPIN_EXEC_TOPIC 执行端话题（status 第 4 段用）。默认自动识别：
#           有 cmd_vel_sim（仿真）就用它，否则用 cmd_vel_chassis（实车）。
set -euo pipefail

NS="${ROBOT_NS-red_standard_robot1}"
# 根命名空间（实车默认）与 <ns> 两种部署统一成同一个前缀写法，避免拼出 "//rotation_cmd"。
NS_PREFIX=""
[ -n "$NS" ] && NS_PREFIX="/${NS}"
#: 把"相对名"补成带命名空间的绝对话题名。
_t() { printf '%s/%s' "$NS_PREFIX" "$1"; }

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"

CMD_TOPIC="${SPIN_CMD_TOPIC:-$(_t rotation_cmd)}"
MAX_WZ="${SPIN_MAX_WZ:-2.0}"
# PID 文件按话题区分：rotation_cmd 沿用旧名字（兼容既有用法），其它话题加后缀，
# 免得导航与建图两套自转发布器互相 kill。
if [ "$CMD_TOPIC" = "$(_t rotation_cmd)" ]; then
  PID_FILE="${TMPDIR:-/tmp}/srm27_spin_control_${NS:-root}.pid"
else
  PID_FILE="${TMPDIR:-/tmp}/srm27_spin_control_${NS:-root}_$(printf '%s' "$CMD_TOPIC" | tr -c 'A-Za-z0-9' '_').pid"
fi

_ws_setup() {
  local ws="${SRM27_WS_DIR:-$(cd "$SCRIPT_DIR/../.." && pwd)}"
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
    echo "[自转] 请求 ${clamped} rad/s 已发布到 ${CMD_TOPIC} (PID $(cat "$PID_FILE"))，0.3 s 内应生效。"
    if [ -z "${subs:-}" ]; then
      echo "[警告] 查不到 ${CMD_TOPIC} 的订阅者 —— 栈没起 / 域不同 / ros2 daemon 缓存旧信息。"
    elif [ "$subs" = "0" ]; then
      cat <<WARN
[警告] ${CMD_TOPIC} 现在**没有任何订阅者**，请求发出去也没人听。常见原因：
        · 云台系链路里 mux 的 spin_topic 不是 rotation_cmd
          （srm_chassis_control.launch.py / nav_srm_simulation_launch.py 的 mux_spin_topic）
        · 旧 spin 模式里 MINCO 的 spin_request.topic 不是 rotation_cmd
        · 话题指错了（SPIN_CMD_TOPIC=${CMD_TOPIC}）
        · 命名空间不一致（本脚本 ROBOT_NS='${NS}'，空串=根命名空间）
        · 栈和本终端不在同一个 ROS_DOMAIN_ID
       逐条确认: $0 status
WARN
    else
      echo "[提示] ${CMD_TOPIC} 订阅者 ${subs} 个（云台系: srm_cmd_mux；旧 spin 模式: MINCO）。停止: $0 off"
    fi
    ;;
  sine)
    smax="${2:?用法: spin_control.sh sine <max> <min> <period>}"
    smin="${3:?用法: spin_control.sh sine <max> <min> <period>}"
    speriod="${4:?用法: spin_control.sh sine <max> <min> <period>}"
    for v in "$smax" "$smin" "$speriod"; do
      case "$v" in ''|*[!0-9.eE+-]*) echo "[错误] 参数必须是数字: $v" >&2; exit 2 ;; esac
      awk -v x="$v" 'BEGIN{ exit !(x == x + 0) }' \
        || { echo "[错误] 参数必须是数字: $v" >&2; exit 2; }
    done
    awk -v p="$speriod" 'BEGIN{ exit !(p > 0.0) }' \
      || { echo "[错误] period 必须 > 0 秒: $speriod" >&2; exit 2; }
    awk -v a="$smax" -v b="$smin" 'BEGIN{ exit !(a >= b) }' \
      || { echo "[错误] max 必须 >= min: $smax < $smin" >&2; exit 2; }
    cmax="$(_clamp "$smax")"
    cmin="$(_clamp "$smin")"
    _ws_setup
    stop_publisher
    _sine_log="${TMPDIR:-/tmp}/srm27_spin_sine_${NS:-root}.log"
    nohup python3 "$SCRIPT_DIR/spin_sine.py" --ns "$NS" --topic "$CMD_TOPIC" \
      --max "$cmax" --min "$cmin" --period "$speriod" >"$_sine_log" 2>&1 &
    echo $! > "$PID_FILE"
    subs="$(timeout 5 ros2 topic info "$CMD_TOPIC" 2>/dev/null | sed -n 's/^Subscription count: //p' || true)"
    echo "[自转] 正弦请求已发布到 ${CMD_TOPIC} (PID $(cat "$PID_FILE")):"
    echo "        max ${cmax} / min ${cmin} rad/s, T ${speriod}s, 日志 ${_sine_log}"
    if [ -z "${subs:-}" ]; then
      echo "[警告] 查不到 ${CMD_TOPIC} 的订阅者 —— 栈没起 / 域不同 / ros2 daemon 缓存旧信息。"
    elif [ "$subs" = "0" ]; then
      echo "[警告] ${CMD_TOPIC} 现在没有任何订阅者（云台系下应由 srm_cmd_mux 消费）。"
      echo "       逐条确认: $0 status"
    else
      echo "[提示] ${CMD_TOPIC} 订阅者 ${subs} 个。停止: $0 off"
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
    echo "  请求话题               = ${CMD_TOPIC}"
    if [ "$CMD_TOPIC" != "$(_t rotation_cmd)" ]; then
      echo "  （自定义话题：mux 的 spin_topic / MINCO 的 spin_request.topic 才是 rotation_cmd，下面仅作对照）"
    fi
    # nav2 节点在机器人命名空间下（/red_standard_robot1/controller_server）；根命名空间的
    # 部署才用 /controller_server。两种都试，否则永远"读不到"。
    ctrl_node=""
    for candidate in "$(_t controller_server)" "/controller_server"; do
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
    wzmax="$(timeout 8 ros2 param get "$(_t srm_cmd_mux)" wz_max 2>/dev/null | sed 's/^Double value is: //' || true)"
    echo "  yaw_policy.mode        = ${mode:-<读不到>}   # 云台系应为 xy_only（MINCO 不消费自转）"
    echo "  spin_request.topic     = ${topic:-<读不到>}   # 与 mux 的 spin_topic 都应为 rotation_cmd"
    echo "  srm_cmd_mux.wz_max     = ${wzmax:-<读不到>}"

    echo "===== 2) 请求是否真的发出去了 ====="
    echo -n "  话题订阅者: "
    subs_info="$(timeout 5 ros2 topic info "$CMD_TOPIC" 2>/dev/null | sed -n 's/^Subscription count: //p' || true)"
    echo "${subs_info:-查不到}"
    if [ "${subs_info:-0}" = "0" ]; then
      echo "  ⚠ 没人在听。检查 mux 的 spin_topic 与 MINCO 的 spin_request.topic 是不是旧话题名:"
      for alt in rotation_velocity; do
        alt_subs="$(timeout 5 ros2 topic info "$(_t "${alt}")" 2>/dev/null | sed -n 's/^Subscription count: //p' || true)"
        [ -n "${alt_subs:-}" ] && [ "${alt_subs:-0}" != "0" ] \
          && echo "    $(_t "${alt}") 订阅者 ${alt_subs} 个 ← 有节点在旧话题上听着"
      done
      echo "  也确认 Nav2 是否在跑: ros2 node list | grep -E 'controller_server|planner_server|bt_navigator|srm_cmd_mux'"
    fi
    timeout 3 ros2 topic echo "$CMD_TOPIC" --once 2>/dev/null | grep -A3 "^angular" || echo "  (无发布 —— 请求未发出，或 $0 on 的后台进程已死)"

    echo "===== 3) MINCO 有没有消费（云台系下应显示拒绝；真实执行看第 4 段） ====="
    timeout 4 ros2 topic echo "$(_t FollowPath/diagnostics)" --once 2>/dev/null \
      | grep -A1 -E "key: (yaw_mode|spin_request|spin_request_fresh|applied_wz|rejected_spin_request_count|planning_result|stop_reason)" \
      | grep -E "key:|value:" | paste - - | sed 's/key: //; s/[[:space:]]*value: /=/' | sed 's/^/  /' \
      || echo "  (诊断未收到 —— 没目标时控制器不跑；云台系下这里不作为执行依据)"

    echo "===== 4) 三层速度（云台系: nav 两层 wz=0，执行层 = 请求值） ====="
    # 执行端话题：仿真叫 cmd_vel_sim，实车叫 cmd_vel_chassis（串口节点订阅的话题）。
    # 没显式给 SPIN_EXEC_TOPIC 时按"谁能查到就用谁"自动识别，两种部署都能直接跑。
    exec_topic="${SPIN_EXEC_TOPIC:-}"
    if [ -z "$exec_topic" ]; then
      if timeout 5 ros2 topic list 2>/dev/null | grep -qx "$(_t cmd_vel_sim)"; then
        exec_topic="$(_t cmd_vel_sim)"
      else
        exec_topic="$(_t cmd_vel_chassis)"
      fi
    fi
    for layer in cmd_vel_nav_fake cmd_vel_nav_body "$(basename "$exec_topic")"; do
      wz="$(timeout 3 ros2 topic echo "$(_t "${layer}")" --once 2>/dev/null \
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
