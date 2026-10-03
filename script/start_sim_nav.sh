#!/usr/bin/env bash
# =============================================================
# 一键启动 rmuc_2025 地图的仿真 NAV 导航 (不做 SLAM 建图)
#
#   标签页 1: Gazebo 仿真 (世界 + 机器人 + 传感器 + 仿真真值里程计)
#   标签页 2: Nav2 导航栈 + RViz (加载已有栅格地图 + 真值里程计)
#   标签页 3: 键鼠控制 (可选, 直接给底盘发 chassis_cmd)
#
# 模式说明 (本脚本只做导航):
#   slam:=False  use_pcd_localization:=False
#     -> 由 map_server 加载现成的 PGM/YAML 地图, map->odom 为静态 TF,
#        odom->base_footprint 由 simulation_ground_truth_odometry 给出;
#        不启动 slam_toolbox / Point-LIO / small_gicp_relocalization。
#   如果需要建图或重定位, 请另起 launch (slam:=True 或 use_pcd_localization:=True),
#   不要与本脚本同时运行, 否则多个节点会争抢 map->odom。
#
# 用法:
#   ./script/start_sim_nav.sh                          # rmuc_2025 + 隧道地图(默认)
#   ./script/start_sim_nav.sh -m rmuc_2025             # 换成普通场地地图
#   ./script/start_sim_nav.sh -m /abs/path/map.yaml    # 用绝对路径地图
#   ./script/start_sim_nav.sh --no-rviz --no-teleop    # 只跑 Gazebo + 导航
#   DRY_RUN=1 ./script/start_sim_nav.sh                # 只打印将执行的命令
#
# 参数:
#   -w, --world   <name>      rmuc_2025 / rmuc_2024 / rmul_2024 / rmul_2025
#   -m, --map     <名字|绝对路径>  地图名(自动补 .yaml)或 YAML 绝对路径
#   -p, --params  <绝对路径>  Nav2 参数文件, 默认 config/simulation/nav2_params.yaml
#       --rviz / --no-rviz        是否启动 RViz (默认启动)
#       --teleop / --no-teleop    是否启动键鼠控制 (默认启动)
#   -h, --help                显示本帮助
#
# 环境变量:
#   WORLD MAP MAP_NAME PARAMS_FILE USE_RVIZ ENABLE_TELEOP
#   OPEN_MODE(tab|window) TERMINAL ROBOT_NS
#   TELEOP_V TELEOP_W USE_COMPOSITION DRY_RUN SRM27_WS_DIR
#
# 说明:
#   - Gazebo 世界由 rmu_gazebo_simulator/config/gz_world.yaml 的 world 字段决定,
#     bringup_sim.launch.py 没有 world 参数; 本脚本会检测 -w 与该文件是否一致。
#   - 地图与参数文件一律使用绝对路径; 地图可直接给名字 (在 map/simulation/ 下解析)。
#   - 每个标签页都会先 source 工作空间的 install/setup.bash。
# =============================================================

set -euo pipefail

# ---------- 路径 ----------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="${SRM27_WS_DIR:-$(cd "$SCRIPT_DIR/.." && pwd)}"
PKG_SRC="$WS_DIR/src/srm27_navigation/srm27_nav_bringup"
SIM_SRC="$WS_DIR/src/rmu_gazebo_simulator/rmu_gazebo_simulator"

# ---------- 可配置项 ----------
OPEN_MODE="${OPEN_MODE:-tab}"                # tab: 同一窗口多标签页; window: 每命令独立窗口
TERMINAL="${TERMINAL:-gnome-terminal}"
ROBOT_NS="${ROBOT_NS:-red_standard_robot1}"  # 与 nav_simulation_launch.py 的 namespace 默认值一致
USE_COMPOSITION="${USE_COMPOSITION:-False}"
TELEOP_V="${TELEOP_V:-0.8}"
TELEOP_W="${TELEOP_W:-0.8}"
USE_RVIZ="${USE_RVIZ:-True}"
ENABLE_TELEOP="${ENABLE_TELEOP:-1}"
DRY_RUN="${DRY_RUN:-0}"

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

if [ "$DRY_RUN" != "1" ] && [ -z "${DISPLAY:-}" ] && [ -z "${WAYLAND_DISPLAY:-}" ]; then
  echo "[警告] 未检测到 DISPLAY / WAYLAND_DISPLAY, Gazebo 与 RViz 的图形界面可能起不来。" >&2
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
  PARAMS_FILE="$PKG_SRC/config/simulation/nav2_params.yaml"
  if [ ! -f "$PARAMS_FILE" ]; then
    PARAMS_FILE="$WS_DIR/install/srm27_nav_bringup/share/srm27_nav_bringup/config/simulation/nav2_params.yaml"
  fi
fi

# Gazebo 实际加载的世界取自 gz_world.yaml, 而不是 launch 参数。
GZ_WORLD_YAML="$SIM_SRC/config/gz_world.yaml"
GZ_WORLD=""
if [ -f "$GZ_WORLD_YAML" ]; then
  GZ_WORLD="$(sed -n 's/^world:[[:space:]]*//p' "$GZ_WORLD_YAML" | head -n 1)"
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

if [ "$GZ_WORLD" != "$WORLD" ]; then
  echo "[警告] Gazebo 实际加载的世界是 '$GZ_WORLD' (来自 $GZ_WORLD_YAML)," >&2
  echo "[警告] 而不是 '$WORLD'。该文件没有 launch 参数, 需要修改:" >&2
  echo "[警告]   $GZ_WORLD_YAML   ->   world: \"$WORLD\"" >&2
fi

if [ "$ENABLE_TELEOP" = "0" ] || [ "$ENABLE_TELEOP" = "false" ] || [ "$ENABLE_TELEOP" = "no" ]; then
  ENABLE_TELEOP=0
else
  ENABLE_TELEOP=1
fi

# ---------- 进程检测 / 清理 ----------
# 方括号包住首字符, 避免 pgrep 匹配到自身命令行。
GAZEBO_PATTERN='(^|/)(gzserver|gzclient|gazebo)([[:space:]]|$)|[i]gn[[:space:]]+gazebo|[g]z[[:space:]]+sim|[r]os2[[:space:]]+launch[[:space:]]+rmu_gazebo_simulator[[:space:]]+bringup_sim\.launch\.py'
GAZEBO_RESIDUAL_PATTERN='[/]ros_gz_bridge/parameter_bridge[[:space:]]+/clock@rosgraph_msgs/msg/Clock\[gz.msgs.Clock|[/]ros_gz_bridge/parameter_bridge.*__ns:=/red_standard_robot1|[/]rmoss_gz_base/rmua19_robot_base.*__ns:=/red_standard_robot1|[r]obot_state_publisher.*__ns:=/red_standard_robot1'
NAV_PATTERN='[r]os2[[:space:]]+launch[[:space:]]+srm27_nav_bringup[[:space:]]+nav_simulation_launch\.py'
TELEOP_PATTERN='[t]est_chassis_cmd\.py'

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
printf 'RViz      : %s    键鼠控制: %s\n' "$USE_RVIZ" "$([ "$ENABLE_TELEOP" = 1 ] && echo 开启 || echo 关闭)"
printf '终端模式  : %s (%s)\n' "$OPEN_MODE" "$TERMINAL"
echo

if [ "$DRY_RUN" != "1" ]; then
  mkdir -p "$(dirname "$GAZEBO_LOCK_FILE")"
  echo "[1/3] 启动 Gazebo 仿真..."
  cleanup_orphaned_gazebo

  if process_running "$GAZEBO_PATTERN" || gazebo_lock_held; then
    echo "[跳过] 已检测到正在运行或正在启动的 Gazebo 仿真。"
  else
    GAZEBO_LOCK_FILE_Q="$(printf '%q' "$GAZEBO_LOCK_FILE")"
    # 释放 FD 9 后终端才会进入保留 shell, 否则锁会被一直持有,
    # 导致旧 Gazebo 退出后下一次仿真仍起不来。
    GAZEBO_CMD="exec 9>${GAZEBO_LOCK_FILE_Q}; if ! flock -n 9; then echo '[跳过] 另一个启动脚本已占用 Gazebo 锁。'; else ros2 launch rmu_gazebo_simulator bringup_sim.launch.py; fi; flock -u 9; exec 9>&-"
    open_term "SRM27 Gazebo 仿真" "$GAZEBO_CMD"
  fi
else
  echo "[1/3] Gazebo 仿真 (dry-run)..."
  open_term "SRM27 Gazebo 仿真" "ros2 launch rmu_gazebo_simulator bringup_sim.launch.py"
fi

echo "[2/3] 启动导航栈 + RViz..."
MAP_FILE_Q="$(printf '%q' "$MAP_FILE")"
PARAMS_FILE_Q="$(printf '%q' "$PARAMS_FILE")"
NAV_CMD="ros2 launch srm27_nav_bringup nav_simulation_launch.py namespace:=${ROBOT_NS} world:=${WORLD} map:=${MAP_FILE_Q} params_file:=${PARAMS_FILE_Q} slam:=False use_pcd_localization:=False use_composition:=${USE_COMPOSITION} use_sim_time:=True use_rviz:=${USE_RVIZ}"
start_once "SRM27 导航 + RViz" "$NAV_PATTERN" "$NAV_CMD"

if [ "$ENABLE_TELEOP" = "1" ]; then
  echo "[3/3] 启动键鼠控制..."
  start_once \
    "SRM27 键鼠控制" \
    "$TELEOP_PATTERN" \
    "ros2 run rmoss_gz_base test_chassis_cmd.py --ros-args -r __ns:=/${ROBOT_NS}/robot_base -p v:=${TELEOP_V} -p w:=${TELEOP_W}"
else
  echo "[3/3] 跳过键鼠控制 (--no-teleop)。"
fi

cat <<EOF

启动流程处理完成。
  - 导航栈刚起时打印 "waiting for clock" 属正常, 等 Gazebo 起来后会自行继续。
  - Gazebo 里车出现后, 在 RViz 用 "2D Goal Pose" / "Nav2 Goal" 下发目标点。

验证 (新终端):
  cd '$WS_DIR' && source install/setup.bash
  ros2 topic hz /${ROBOT_NS}/velodyne_points            # 仿真点云经 ign_sim_pointcloud_tool 转换后
  ros2 topic echo /${ROBOT_NS}/odometry --once          # 仿真真值里程计
  ros2 run tf2_ros tf2_echo map gimbal_yaw_fake         # 完整 TF 链: map->odom->base_footprint->...->gimbal_yaw_fake
  ros2 topic list | grep small_gicp                     # 本模式应为空输出

清理残留:
  ./script/kill_gzb.sh && ./script/kill_rviz.sh
EOF
