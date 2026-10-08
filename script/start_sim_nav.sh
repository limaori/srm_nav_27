#!/usr/bin/env bash
# =============================================================
# SRM 仿真 NAV 导航 + 独立自转测试 (一键启动)
#
#   标签页 1: SRM Gazebo 仿真 (世界 + SRM 圆柱底盘 + 雷达 + 速度执行)
#   标签页 2: 速度合成 + Nav2 导航栈 + RViz (含独立自转测试发送器)
#   标签页 3: 手柄自转 (可选, 默认关闭; 需要手柄, 输出 remap 到 rotation_cmd)
#
# 速度链路 (与实施方案 3.3 一致):
#   Nav2 controller ─cmd_vel_nav─┐
#                                ├─ srm_cmd_mux ─cmd_vel_sim─► SRM 速度适配器 → Gazebo
#   rotation_test_sender ─ rotation_controller ─ rotation_velocity ┘
#
#   导航只产生 vx/vy; mux 丢弃导航输入的 angular.z。自转由独立自转链路给出。
#
# 模式说明 (本脚本只做导航):
#   slam:=False  use_pcd_localization:=False
#     -> 由 map_server 加载现成的 PGM/YAML 地图, map->odom 为静态 TF,
#        odom->base_link 由 simulation_ground_truth_odometry 给出;
#        不启动 slam_toolbox / Point-LIO / small_gicp_relocalization。
#   如果需要建图或重定位, 请另起 launch (slam:=True 或 use_pcd_localization:=True),
#   不要与本脚本同时运行, 否则多个节点会争抢 map->odom。
#
# 用法:
#   ./script/start_sim_nav.sh                                  # rmuc_2025 + 隧道地图(默认), 不自转
#   ./script/start_sim_nav.sh -m rmuc_2025                     # 换成普通场地地图
#   ./script/start_sim_nav.sh -w srm_empty --run               # 空场, Gazebo 直接开始运行
#   ./script/start_sim_nav.sh --rotation-mode constant --rotation-speed 1.0
#   ./script/start_sim_nav.sh --rotation-mode periodic \
#       --rotation-offset 1.0 --rotation-amplitude 0.5 --rotation-period 4.0
#   ./script/start_sim_nav.sh --no-rviz --no-rotation          # 只跑仿真 + 导航
#   DRY_RUN=1 ./script/start_sim_nav.sh                        # 只打印将执行的命令
#
# 参数:
#   -w, --world   <name>      rmuc_2025 / rmuc_2024 / rmul_2024 / rmul_2025 / srm_empty
#   -m, --map     <名字|绝对路径>  地图名(自动补 .yaml)或 YAML 绝对路径
#   -p, --params  <绝对路径>  Nav2 参数文件, 默认 config/simulation/nav2_params_srm.yaml
#       --rviz / --no-rviz        是否启动 RViz (默认启动)
#       --gui / --no-gui          Gazebo 是否带 GUI (默认带)
#       --run / --no-run           Gazebo 是否直接开始运行 (默认暂停, 手动点播放)
#       --smoother / --no-smoother velocity_smoother 是否串联 (默认串联)
#       --rotation-mode <stop|constant|periodic>   自转模式 (默认 stop)
#       --rotation-speed <rad/s>  恒速模式角速度
#       --rotation-offset <rad/s> 周期模式平均角速度
#       --rotation-amplitude <rad/s> 周期模式变化幅度
#       --rotation-period <s>     周期
#       --rotation-phase <rad>    初相位
#       --rotation-wave <sine|square> 周期波形
#       --rotation / --no-rotation 是否启动自转测试发送器 (默认启动, stop 模式)
#       --teleop / --no-teleop    手柄自转标签页 (默认关闭)
#   -h, --help                显示本帮助
#
# 环境变量:
#   WORLD MAP MAP_NAME PARAMS_FILE USE_RVIZ USE_GUI RUN_IMMEDIATELY
#   USE_VELOCITY_SMOOTHER ROTATION_MODE ROTATION_SPEED ROTATION_OFFSET
#   ROTATION_AMPLITUDE ROTATION_PERIOD ROTATION_PHASE ROTATION_SINE_WAVE
#   START_ROTATION_SENDER ENABLE_TELEOP
#   OPEN_MODE(tab|window) TERMINAL ROBOT_NS USE_COMPOSITION DRY_RUN SRM27_WS_DIR
#
# 说明:
#   - Gazebo 世界、SRM 初始位姿和速度参数统一由
#     srm27_gazebo_simulator/config/srm_sim.yaml 给出; -w 会覆盖其中的 world。
#   - 地图与参数文件一律使用绝对路径; 地图可直接给名字 (在 map/simulation/ 下解析)。
#   - 每个标签页都会先 source 工作空间的 install/setup.bash。
#   - 控制清单：导航速度 cmd_vel_nav、自转请求 rotation_cmd、自转输出
#     rotation_velocity、合成命令 cmd_vel_sim; 诊断在 /<ns>/diagnostics。
# =============================================================

set -euo pipefail

# ---------- 路径 ----------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="${SRM27_WS_DIR:-$(cd "$SCRIPT_DIR/.." && pwd)}"
PKG_SRC="$WS_DIR/src/srm27_navigation/srm27_nav_bringup"
SIM_SRC="$WS_DIR/src/srm27_gazebo_simulator"

# ---------- 可配置项 ----------
OPEN_MODE="${OPEN_MODE:-tab}"                # tab: 同一窗口多标签页; window: 每命令独立窗口
TERMINAL="${TERMINAL:-gnome-terminal}"
ROBOT_NS="${ROBOT_NS:-red_standard_robot1}"  # 与 nav_srm_simulation_launch.py 的 namespace 默认值一致
USE_COMPOSITION="${USE_COMPOSITION:-False}"
USE_RVIZ="${USE_RVIZ:-True}"
USE_GUI="${USE_GUI:-true}"
RUN_IMMEDIATELY="${RUN_IMMEDIATELY:-false}"
USE_VELOCITY_SMOOTHER="${USE_VELOCITY_SMOOTHER:-True}"
ENABLE_TELEOP="${ENABLE_TELEOP:-0}"
DRY_RUN="${DRY_RUN:-0}"

START_ROTATION_SENDER="${START_ROTATION_SENDER:-1}"
ROTATION_MODE="${ROTATION_MODE:-stop}"
ROTATION_SPEED="${ROTATION_SPEED:-0.0}"
ROTATION_OFFSET="${ROTATION_OFFSET:-0.0}"
ROTATION_AMPLITUDE="${ROTATION_AMPLITUDE:-0.0}"
ROTATION_PERIOD="${ROTATION_PERIOD:-4.0}"
ROTATION_PHASE="${ROTATION_PHASE:-0.0}"
ROTATION_SINE_WAVE="${ROTATION_SINE_WAVE:-true}"

MAP_NAME="${MAP_NAME:-}"
MAP_FILE="${MAP:-}"
PARAMS_FILE="${PARAMS_FILE:-}"
WORLD_FROM_USER="${WORLD:-}"
WORLD=""

# ---------- 帮助 ----------
usage() {
  # 打印文件头注释块 (跳过 shebang 与首尾分隔线), 避免帮助文本与代码不同步。
  awk 'NR == 1 { next }
       /^# =+[[:space:]]*$/ { sep++; if (sep == 2) exit; next }
       { sub(/^# ?/, ""); print }' "${BASH_SOURCE[0]}"
}

# ---------- 命令行参数 ----------
while [ $# -gt 0 ]; do
  case "$1" in
    -w|--world)   WORLD_FROM_USER="${2:?--world 需要一个世界名}"; shift 2 ;;
    -m|--map)     MAP_FILE="${2:?--map 需要地图名或绝对路径}"; shift 2 ;;
    -p|--params)  PARAMS_FILE="${2:?--params 需要绝对路径}"; shift 2 ;;
    --rviz)       USE_RVIZ="True"; shift ;;
    --no-rviz)    USE_RVIZ="False"; shift ;;
    --gui)        USE_GUI="true"; shift ;;
    --no-gui)     USE_GUI="false"; shift ;;
    --run)        RUN_IMMEDIATELY="true"; shift ;;
    --no-run)     RUN_IMMEDIATELY="false"; shift ;;
    --smoother)   USE_VELOCITY_SMOOTHER="True"; shift ;;
    --no-smoother) USE_VELOCITY_SMOOTHER="False"; shift ;;
    --rotation-mode)      ROTATION_MODE="${2:?--rotation-mode 需要 stop|constant|periodic}"; shift 2 ;;
    --rotation-speed)     ROTATION_SPEED="${2:?--rotation-speed 需要 rad/s}"; shift 2 ;;
    --rotation-offset)    ROTATION_OFFSET="${2:?--rotation-offset 需要 rad/s}"; shift 2 ;;
    --rotation-amplitude) ROTATION_AMPLITUDE="${2:?--rotation-amplitude 需要 rad/s}"; shift 2 ;;
    --rotation-period)    ROTATION_PERIOD="${2:?--rotation-period 需要秒}"; shift 2 ;;
    --rotation-phase)     ROTATION_PHASE="${2:?--rotation-phase 需要弧度}"; shift 2 ;;
    --rotation-wave)      case "${2:?--rotation-wave 需要 sine|square}" in
                            sine)   ROTATION_SINE_WAVE="true" ;;
                            square) ROTATION_SINE_WAVE="false" ;;
                            *) echo "[错误] --rotation-wave 只支持 sine 或 square" >&2; exit 2 ;;
                          esac
                          shift 2 ;;
    --rotation)     START_ROTATION_SENDER="1"; shift ;;
    --no-rotation)  START_ROTATION_SENDER="0"; shift ;;
    --teleop)     ENABLE_TELEOP="1"; shift ;;
    --no-teleop)  ENABLE_TELEOP="0"; shift ;;
    -h|--help)    usage; exit 0 ;;
    *) echo "[错误] 未知参数: $1 (用 -h 查看用法)" >&2; exit 2 ;;
  esac
done

# ---------- 环境检查 ----------
if [ ! -f "$WS_DIR/install/setup.bash" ]; then
  echo "[错误] 未找到 $WS_DIR/install/setup.bash" >&2
  echo "[提示] 先构建工作空间, 或用 SRM27_WS_DIR=/path/to/ws 指定工作空间。" >&2
  exit 1
fi

if [ "$DRY_RUN" != "1" ] && ! command -v "$TERMINAL" >/dev/null 2>&1; then
  echo "[错误] 未找到终端程序: $TERMINAL" >&2
  echo "[提示] 可用 TERMINAL=konsole 或 TERMINAL=xfce4-terminal 指定。" >&2
  exit 1
fi

if [ "$DRY_RUN" != "1" ] && [ "$USE_GUI" = "true" ] \
   && [ -z "${DISPLAY:-}" ] && [ -z "${WAYLAND_DISPLAY:-}" ]; then
  echo "[警告] 未检测到 DISPLAY / WAYLAND_DISPLAY, Gazebo 与 RViz 的图形界面可能起不来。" >&2
  echo "[警告] 可以改用 --no-gui --no-rviz。" >&2
fi

# ---------- 工作区一致性检查 ----------
# 若当前 shell 还 source 了同机的另一份工作区（例如 ~/srm_nav_27test），同名包会优先解析到
# 那边：底盘限速、参数文件、行为树可能全部来自另一个工作区，而 MINCO 插件只存在于本工作区
# 于是出现"一半新一半旧"的混合环境——现场表现为"改了 v_max=1.5 却仍然被限到 0.5"。
# 这里在启动前逐个核对关键包的解析路径，不一致就直接失败，而不是让混合环境跑起来。
require_pkg_from_ws() {
  local pkg="$1" prefix
  prefix="$(bash -c "source '$WS_DIR/install/setup.bash' >/dev/null 2>&1; ros2 pkg prefix '$pkg' 2>/dev/null" || true)"
  if [ -z "$prefix" ]; then
    echo "[错误] 找不到包 $pkg，请确认 $WS_DIR/install/setup.bash 可用。" >&2
    return 1
  fi
  case "$prefix" in
    "$WS_DIR"/*) return 0 ;;
    *)
      echo "[错误] 包 $pkg 解析到了其它工作区: $prefix" >&2
      echo "       期望前缀: $WS_DIR" >&2
      echo "       当前环境里还 source 了别的工作区（检查 ~/.bashrc 与当前 shell 的 AMENT_PREFIX_PATH），" >&2
      echo "       否则同名包（含底盘限速、Nav2 参数、行为树）会从那边加载。请开新终端只 source 本工作区。" >&2
      return 1
      ;;
  esac
}

if [ "$DRY_RUN" != "1" ]; then
  for _pkg in srm27_nav_bringup srm27_chassis_control srm27_minco_controller; do
    require_pkg_from_ws "$_pkg" || exit 1
  done
fi

if [ "$DRY_RUN" != "1" ] && ! command -v flock >/dev/null 2>&1; then
  echo "[错误] 未找到 flock, 无法保证 Gazebo 单实例启动。" >&2
  exit 1
fi

# ---------- 地图 / 参数文件解析 ----------
# 优先用源码目录 (symlink-install 下与 install 内容一致), 退化到 install。
MAP_DIR="$PKG_SRC/map/simulation"
if [ ! -d "$MAP_DIR" ]; then
  MAP_DIR="$WS_DIR/install/srm27_nav_bringup/share/srm27_nav_bringup/map/simulation"
fi
if [ ! -d "$MAP_DIR" ]; then
  echo "[错误] 未找到仿真地图目录: $MAP_DIR" >&2
  exit 1
fi

if [ -z "$PARAMS_FILE" ]; then
  PARAMS_FILE="$PKG_SRC/config/simulation/nav2_params_srm.yaml"
  if [ ! -f "$PARAMS_FILE" ]; then
    PARAMS_FILE="$WS_DIR/install/srm27_nav_bringup/share/srm27_nav_bringup/config/simulation/nav2_params_srm.yaml"
  fi
fi

# Gazebo 世界与 SRM 初始位姿由 srm27_gazebo_simulator/config/srm_sim.yaml 给出。
SIM_CONFIG="$SIM_SRC/config/srm_sim.yaml"
if [ ! -f "$SIM_CONFIG" ]; then
  SIM_CONFIG="$WS_DIR/install/srm27_gazebo_simulator/share/srm27_gazebo_simulator/config/srm_sim.yaml"
fi
GZ_WORLD=""
if [ -f "$SIM_CONFIG" ]; then
  GZ_WORLD="$(sed -n 's/^world:[[:space:]]*//p' "$SIM_CONFIG" | head -n 1)"
  GZ_WORLD="${GZ_WORLD%\"}"; GZ_WORLD="${GZ_WORLD#\"}"
  GZ_WORLD="${GZ_WORLD%\'}"; GZ_WORLD="${GZ_WORLD#\'}"
fi

WORLD="${WORLD_FROM_USER:-${GZ_WORLD:-rmuc_2025}}"

list_maps() {
  echo "[提示] $MAP_DIR 下可用的地图:" >&2
  (cd "$MAP_DIR" && ls *.yaml 2>/dev/null | sed 's/^/  - /') >&2
}

# 地图名 -> 绝对路径; 也接受绝对路径。
resolve_map() {
  local value="$1"
  if [ -z "$value" ]; then
    return 1
  fi
  if [ "${value#/}" != "$value" ]; then
    if [ -f "$value" ]; then
      printf '%s\n' "$value"
      return 0
    fi
    echo "[错误] 地图文件不存在: $value" >&2
    return 1
  fi
  if [ "${value#*/}" != "$value" ]; then
    echo "[错误] 相对路径地图不受支持, 请用绝对路径: $value" >&2
    return 1
  fi
  local name="$value"
  case "$name" in
    *.yaml) ;;
    *) name="${name}.yaml" ;;
  esac
  if [ -f "$MAP_DIR/$name" ]; then
    printf '%s\n' "$MAP_DIR/$name"
    return 0
  fi
  echo "[错误] 未找到地图: $MAP_DIR/$name" >&2
  list_maps
  return 1
}

if [ -z "$MAP_FILE" ]; then
  if [ -z "$MAP_NAME" ]; then
    for candidate in "${WORLD}_tunnel" "$WORLD"; do
      if [ -f "$MAP_DIR/${candidate}.yaml" ]; then
        MAP_NAME="$candidate"
        break
      fi
    done
  fi
  if [ -z "$MAP_NAME" ]; then
    echo "[错误] 无法为世界 $WORLD 推断默认地图, 请用 -m 指定。" >&2
    list_maps
    exit 1
  fi
  MAP_FILE="$MAP_NAME"
fi

MAP_FILE="$(resolve_map "$MAP_FILE")" || exit 1

if [ ! -f "$PARAMS_FILE" ]; then
  echo "[错误] 参数文件不存在: $PARAMS_FILE" >&2
  exit 1
fi
if [ "${PARAMS_FILE#/}" = "$PARAMS_FILE" ]; then
  echo "[错误] 参数文件必须是绝对路径: $PARAMS_FILE" >&2
  exit 1
fi

# ---------- 布尔值归一化 ----------
norm_bool() {
  case "$(printf '%s' "$1" | tr '[:upper:]' '[:lower:]')" in
    1|true|yes|on)  echo "True" ;;
    0|false|no|off) echo "False" ;;
    *)              echo "$1" ;;
  esac
}
USE_RVIZ="$(norm_bool "$USE_RVIZ")"

norm_switch() {
  case "$(printf '%s' "$1" | tr '[:upper:]' '[:lower:]')" in
    1|true|yes|on)  echo "true" ;;
    0|false|no|off) echo "false" ;;
    *)              echo "$1" ;;
  esac
}
USE_GUI="$(norm_switch "$USE_GUI")"
RUN_IMMEDIATELY="$(norm_switch "$RUN_IMMEDIATELY")"
ROTATION_SINE_WAVE="$(norm_switch "$ROTATION_SINE_WAVE")"

if [ -n "$WORLD_FROM_USER" ] && [ -n "$GZ_WORLD" ] && [ "$GZ_WORLD" != "$WORLD" ]; then
  echo "[提示] 世界由 srm_sim.yaml 的 '$GZ_WORLD' 覆盖为 '$WORLD'。"
fi

case "$ROTATION_MODE" in
  stop|constant|periodic) ;;
  *) echo "[错误] --rotation-mode 只支持 stop / constant / periodic" >&2; exit 2 ;;
esac

if [ "$START_ROTATION_SENDER" = "0" ] || [ "$START_ROTATION_SENDER" = "false" ] || [ "$START_ROTATION_SENDER" = "no" ]; then
  START_ROTATION_SENDER="False"
else
  START_ROTATION_SENDER="True"
fi

if [ "$ENABLE_TELEOP" = "0" ] || [ "$ENABLE_TELEOP" = "false" ] || [ "$ENABLE_TELEOP" = "no" ]; then
  ENABLE_TELEOP=0
else
  ENABLE_TELEOP=1
fi

# ---------- 进程检测 / 清理 ----------
# 方括号包住首字符, 避免 pgrep 匹配到自身命令行。
GAZEBO_PATTERN='(^|/)(gzserver|gzclient|gazebo)([[:space:]]|$)|[i]gn[[:space:]]+gazebo|[g]z[[:space:]]+sim|[r]os2[[:space:]]+launch[[:space:]]+srm27_gazebo_simulator'
GAZEBO_RESIDUAL_PATTERN='[/]ros_gz_bridge/parameter_bridge[[:space:]]+/clock@rosgraph_msgs/msg/Clock\[gz.msgs.Clock|[s]rm_velocity_adapter|[r]obot_state_publisher.*__ns:=/red_standard_robot1'
NAV_PATTERN='[r]os2[[:space:]]+launch[[:space:]]+srm27_nav_bringup[[:space:]]+(nav_srm_simulation_launch|nav_simulation_launch)\.py'
TELEOP_PATTERN='[s]rm27_teleop_twist_joy_node'

GAZEBO_LOCK_FILE="${SRM27_GAZEBO_LOCK_FILE:-${XDG_RUNTIME_DIR:-/tmp}/srm27_sentry_gazebo.lock}"

process_running() {
  pgrep -u "$(id -u)" -f "$1" >/dev/null 2>&1
}

gazebo_lock_held() {
  local lock_fd
  exec {lock_fd}>"$GAZEBO_LOCK_FILE"
  if flock -n "$lock_fd"; then
    flock -u "$lock_fd"
    exec {lock_fd}>&-
    return 1
  fi
  exec {lock_fd}>&-
  return 0
}

# 上次 launch 异常退出时, /clock bridge 与机器人节点可能留在后台;
# 没有 Gazebo 本体却有这些残留时先清掉, 避免新旧时钟并存。
cleanup_orphaned_gazebo() {
  process_running "$GAZEBO_PATTERN" && return 0
  process_running "$GAZEBO_RESIDUAL_PATTERN" || return 0

  echo "[提示] 检测到上次仿真的残留进程, 正在清理..."
  WAIT_SECONDS="${GAZEBO_CLEANUP_WAIT_SECONDS:-5}" "$SCRIPT_DIR/kill_gzb.sh"
}

# ---------- 终端 ----------
open_term() {
  local title="$1"
  local cmd="$2"
  # 先 cd 到工作空间并 source 环境; 命令结束后保留 shell 便于查看输出。
  local full_cmd="cd '$WS_DIR' && source install/setup.bash && ${cmd}; exec bash"

  if [ "$DRY_RUN" = "1" ]; then
    echo "--- [dry-run] 标签页: $title"
    echo "$full_cmd"
    return 0
  fi

  if [ "$OPEN_MODE" = "tab" ]; then
    "$TERMINAL" --tab --title="$title" -- bash -c "$full_cmd"
  else
    "$TERMINAL" --title="$title" -- bash -c "$full_cmd"
  fi
}

start_once() {
  local title="$1"
  local process_pattern="$2"
  local cmd="$3"

  if process_running "$process_pattern"; then
    echo "[跳过] 已检测到正在运行的进程: $title"
    return 0
  fi

  open_term "$title" "$cmd"
}

# ---------- 启动 ----------
printf '========== SRM27 仿真 NAV 导航 (世界: %s) ==========\n' "$WORLD"
printf '工作空间  : %s\n' "$WS_DIR"
printf '地图      : %s\n' "$MAP_FILE"
printf '参数文件  : %s\n' "$PARAMS_FILE"
printf '定位方式  : 静态地图 + 仿真真值里程计 (slam:=False, use_pcd_localization:=False)\n'
printf 'RViz      : %s    Gazebo GUI: %s    直接运行: %s\n' "$USE_RVIZ" "$USE_GUI" "$RUN_IMMEDIATELY"
printf '平滑器    : %s\n' "$USE_VELOCITY_SMOOTHER"
printf '自转测试  : 发送器 %s  模式 %s' "$START_ROTATION_SENDER" "$ROTATION_MODE"
case "$ROTATION_MODE" in
  constant) printf '  角速度 %s rad/s' "$ROTATION_SPEED" ;;
  periodic) printf '  offset %s + amplitude %s, period %s s, phase %s rad, %s' \
      "$ROTATION_OFFSET" "$ROTATION_AMPLITUDE" "$ROTATION_PERIOD" "$ROTATION_PHASE" \
      "$([ "$ROTATION_SINE_WAVE" = "true" ] && echo 正弦 || echo 方波)" ;;
esac
printf '\n终端模式  : %s (%s)\n' "$OPEN_MODE" "$TERMINAL"
echo

SIM_CMD="ros2 launch srm27_gazebo_simulator srm_sim.launch.py world:=${WORLD} gui:=${USE_GUI} run_immediately:=${RUN_IMMEDIATELY}"

if [ "$DRY_RUN" != "1" ]; then
  mkdir -p "$(dirname "$GAZEBO_LOCK_FILE")"
  echo "[1/3] 启动 SRM Gazebo 仿真..."
  cleanup_orphaned_gazebo

  if process_running "$GAZEBO_PATTERN" || gazebo_lock_held; then
    echo "[跳过] 已检测到正在运行或正在启动的 Gazebo 仿真。"
  else
    GAZEBO_LOCK_FILE_Q="$(printf '%q' "$GAZEBO_LOCK_FILE")"
    # 释放 FD 9 后终端才会进入保留 shell, 否则锁会被一直持有,
    # 导致旧 Gazebo 退出后下一次仿真仍起不来。
    GAZEBO_CMD="exec 9>${GAZEBO_LOCK_FILE_Q}; if ! flock -n 9; then echo '[跳过] 另一个启动脚本已占用 Gazebo 锁。'; else ${SIM_CMD}; fi; flock -u 9; exec 9>&-"
    open_term "SRM27 Gazebo 仿真" "$GAZEBO_CMD"
  fi
else
  echo "[1/3] SRM Gazebo 仿真 (dry-run)..."
  open_term "SRM27 Gazebo 仿真" "$SIM_CMD"
fi

echo "[2/3] 启动速度合成 + 导航栈 + RViz..."
MAP_FILE_Q="$(printf '%q' "$MAP_FILE")"
PARAMS_FILE_Q="$(printf '%q' "$PARAMS_FILE")"
NAV_CMD="ros2 launch srm27_nav_bringup nav_srm_simulation_launch.py namespace:=${ROBOT_NS} world:=${WORLD} map:=${MAP_FILE_Q} params_file:=${PARAMS_FILE_Q} slam:=False use_pcd_localization:=False use_composition:=${USE_COMPOSITION} use_sim_time:=True use_rviz:=${USE_RVIZ} use_velocity_smoother:=${USE_VELOCITY_SMOOTHER} start_rotation_sender:=${START_ROTATION_SENDER} rotation_mode:=${ROTATION_MODE} rotation_speed:=${ROTATION_SPEED} rotation_offset:=${ROTATION_OFFSET} rotation_amplitude:=${ROTATION_AMPLITUDE} rotation_period:=${ROTATION_PERIOD} rotation_phase:=${ROTATION_PHASE} rotation_sine_wave:=${ROTATION_SINE_WAVE}"
start_once "SRM27 导航 + 速度合成 + RViz" "$NAV_PATTERN" "$NAV_CMD"

if [ "$ENABLE_TELEOP" = "1" ]; then
  echo "[3/3] 启动手柄自转 (输出 remap 到 rotation_cmd)..."
  start_once \
    "SRM27 手柄自转" \
    "$TELEOP_PATTERN" \
    "ros2 run srm27_teleop_twist_joy srm27_teleop_twist_joy_node --ros-args -r __ns:=/${ROBOT_NS} -r cmd_vel:=rotation_cmd -p use_sim_time:=true"
else
  echo "[3/3] 跳过手柄自转 (--no-teleop)。自转请用 --rotation-mode 选择模式。"
fi

cat <<EOF

启动流程处理完成。
  - 导航栈刚起时打印 "waiting for clock" 属正常, 等 Gazebo 起来后会自行继续。
  - Gazebo 若不是直接运行(--run), 需要点播放后仿真才推进。
  - Gazebo 里车出现后, 在 RViz 用 "2D Goal Pose" / "Nav2 Goal" 下发目标点。

速度链路检查 (新终端):
  cd '$WS_DIR' && source install/setup.bash
  ros2 topic echo /${ROBOT_NS}/cmd_vel_nav --once         # 导航平移速度 (angular.z 应为 0)
  ros2 topic echo /${ROBOT_NS}/rotation_velocity --once   # 独立自转速度
  ros2 topic echo /${ROBOT_NS}/cmd_vel_sim --once         # 合成后的最终执行命令
  ros2 topic hz /${ROBOT_NS}/cmd_vel_sim                  # 应为 200 Hz 左右
  ros2 topic echo /${ROBOT_NS}/odometry --once            # 真值里程计 (含实际 yaw/wz)
  ros2 run tf2_ros tf2_echo map base_link                 # 完整 TF 链: map->odom->base_link
  ros2 topic echo /${ROBOT_NS}/diagnostics --once         # 合成与超时状态

切换自转模式 (无需重启导航):
  ros2 service call /${ROBOT_NS}/rotation_test_sender/disable std_srvs/srv/Trigger
  ros2 param set /${ROBOT_NS}/rotation_test_sender rotation_mode periodic
  ros2 param set /${ROBOT_NS}/rotation_test_sender offset 1.0
  ros2 param set /${ROBOT_NS}/rotation_test_sender amplitude 0.5
  ros2 param set /${ROBOT_NS}/rotation_test_sender period 4.0
  ros2 service call /${ROBOT_NS}/rotation_test_sender/enable std_srvs/srv/Trigger

急停 (清零并保持零速):
  ros2 service call /${ROBOT_NS}/srm_cmd_mux/stop_all std_srvs/srv/Trigger
  ros2 service call /${ROBOT_NS}/srm_cmd_mux/resume_all std_srvs/srv/Trigger

清理残留:
  ./script/kill_gzb.sh && ./script/kill_rviz.sh
EOF
