#!/usr/bin/env bash
# =============================================================
# 一键启动带重定位的导航仿真：Gazebo 仿真 + RViz 导航 + 键鼠控制
#
# 用法:
#   ./script/start_nav_reloc.sh
#   或
#   SRM27_WS_DIR=/path/to/ws ./script/start_nav_reloc.sh
#   PRIOR_PCD_FILE=/path/to/map.pcd ./script/start_nav_reloc.sh
#   WORLD=rmuc_2025 ./script/start_nav_reloc.sh
#
# 说明:
#   - 默认在同一个窗口里打开 3 个标签页（Gazebo / RViz 导航 / 键鼠控制）。
#   - 若想换成每个命令一个独立窗口, 执行时加环境变量 OPEN_MODE=window 即可。
#   - 每个命令都会先 source 工作空间的 install/setup.bash。
#
# 重定位(Relocalization)说明:
#   本脚本通过 nav_simulation_launch.py 的 use_pcd_localization:=True
#   启用基于先验点云地图的定位:
#     - Point-LIO            : 融合 Lidar + IMU 输出里程计 (odom -> base)
#     - small_gicp_relocalization : 将实时点云与先验 PCD 地图做 GICP 配准,
#                                   估计 map -> odom
#   因此不再使用 Gazebo 真值里程计/静态 map->odom。
#
#   先验地图 PCD 通过 PRIOR_PCD_FILE 指定; 默认用
#   src/srm27_navigation/point_lio/PCD/scans.pcd (仿真扫图生成的扫描地图)。
#   若该文件存在, 则无需额外配置即可使用。
# =============================================================

set -euo pipefail

# 工作目录 = 脚本所在目录的上一级（即 ROS2 工作空间根目录）
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# 脚本所在目录，用于调用同目录下的清理脚本。
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# 允许通过环境变量覆盖
WS_DIR="${SRM27_WS_DIR:-$WS_DIR}"

# 窗口 / 标签页 模式: "tab" 同一窗口多个标签页(默认), "window" 每个命令一个独立窗口
OPEN_MODE="${OPEN_MODE:-tab}"

# 终端程序（可改成 konsole / xfce4-terminal 等）
TERMINAL="${TERMINAL:-gnome-terminal}"

# 世界 / 地图名
WORLD="${WORLD:-rmuc_2025}"

# 先验点云地图 (PCD). 默认优先使用仿真扫图生成的扫描地图。
DEFAULT_PRIOR_PCD="${SRM27_DEFAULT_PRIOR_PCD:-$WS_DIR/src/srm27_navigation/point_lio/PCD/scans.pcd}"
PRIOR_PCD_FILE="${PRIOR_PCD_FILE:-}"
if [ -z "$PRIOR_PCD_FILE" ] && [ -f "$DEFAULT_PRIOR_PCD" ]; then
  PRIOR_PCD_FILE="$DEFAULT_PRIOR_PCD"
fi

# 重定位依赖 small_gicp 的先验 PCD 目标点云，Point-LIO 只负责里程计。
if [ -z "$PRIOR_PCD_FILE" ]; then
  echo "[错误] 未找到先验点云地图。" >&2
  echo "[错误] 请用 PRIOR_PCD_FILE=/绝对路径/map.pcd 指定, 或确保默认路径存在:" >&2
  echo "[错误]   $DEFAULT_PRIOR_PCD" >&2
  echo "[错误] 提示: 仿真扫图通常存放在 src/srm27_navigation/point_lio/PCD/scans.pcd。" >&2
  exit 1
fi
if [ ! -f "$PRIOR_PCD_FILE" ]; then
  echo "[错误] 先验点云地图文件不存在: $PRIOR_PCD_FILE" >&2
  echo "[错误] 请确认路径正确, 且该 PCD 与当前 WORLD=$WORLD 匹配。" >&2
  exit 1
fi

# Gazebo 启动锁由终端里的 ros2 launch 进程持有，直到该仿真退出。
# 这样即使两个脚本几乎同时执行，也最多只有一个 Gazebo 真正启动。
GAZEBO_LOCK_FILE="${SRM27_GAZEBO_LOCK_FILE:-${XDG_RUNTIME_DIR:-/tmp}/srm27_sentry_gazebo.lock}"
GAZEBO_LOCK_DIR="$(dirname "$GAZEBO_LOCK_FILE")"

if ! command -v flock >/dev/null 2>&1; then
  echo "[错误] 未找到 flock，无法保证 Gazebo 单实例启动。" >&2
  exit 1
fi

mkdir -p "$GAZEBO_LOCK_DIR"

if [ ! -f "$WS_DIR/install/setup.bash" ]; then
  echo "[错误] 未找到工作空间的 install/setup.bash, 请确认路径: $WS_DIR" >&2
  exit 1
fi

# 打开一个终端执行指定命令
open_term() {
  local title="$1"
  local cmd="$2"
  # 在终端内先 cd 到工作空间, source 环境, 再执行命令; 结束后保留 shell 便于查看输出
  local full_cmd="cd '$WS_DIR' && source install/setup.bash && ${cmd}; exec bash"

  if [ "$OPEN_MODE" = "tab" ]; then
    "$TERMINAL" --tab --title="$title" -- bash -c "$full_cmd"
  else
    "$TERMINAL" --title="$title" -- bash -c "$full_cmd"
  fi
}

# Gazebo、时钟 bridge 和机器人仿真节点的匹配规则。
# 使用方括号包住首字符，避免 pgrep 把自身的匹配命令算进去。
GAZEBO_PATTERN='(^|/)(gzserver|gzclient|gazebo)([[:space:]]|$)|[i]gn[[:space:]]+gazebo|[g]z[[:space:]]+sim|[r]os2[[:space:]]+launch[[:space:]]+rmu_gazebo_simulator[[:space:]]+bringup_sim\.launch\.py'
GAZEBO_RESIDUAL_PATTERN='[/]ros_gz_bridge/parameter_bridge[[:space:]]+/clock@rosgraph_msgs/msg/Clock\[gz.msgs.Clock|[/]ros_gz_bridge/parameter_bridge.*__ns:=/red_standard_robot1|[/]rmoss_gz_base/rmua19_robot_base.*__ns:=/red_standard_robot1|[r]obot_state_publisher.*__ns:=/red_standard_robot1'

# 重定位(导航)进程的匹配规则。
NAV_RELOC_PATTERN='[r]os2 launch srm27_nav_bringup nav_simulation_launch.py'

# 检查命令是否已经在运行。
# 使用带方括号的正则，避免 pgrep 把自身的匹配命令算进去。
process_running() {
  pgrep -u "$(id -u)" -f "$1" >/dev/null 2>&1
}

# 检查 Gazebo 锁是否已被另一个启动终端持有。
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

# 上一次 launch 异常退出时，/clock bridge 和机器人节点可能留在后台。
# 没有 Gazebo 本体却发现这些残留时，先清掉它们，避免新旧时钟并存。
cleanup_orphaned_gazebo() {
  if process_running "$GAZEBO_PATTERN"; then
    return 0
  fi

  if ! process_running "$GAZEBO_RESIDUAL_PATTERN"; then
    return 0
  fi

  echo "[提示] 检测到上一次仿真的残留进程，正在清理..."
  WAIT_SECONDS="${GAZEBO_CLEANUP_WAIT_SECONDS:-5}" \
    "$SCRIPT_DIR/kill_gzb.sh"
}

start_once() {
  local title="$1"
  local process_pattern="$2"
  local cmd="$3"

  if process_running "$process_pattern"; then
    echo "[跳过] 已检测到正在运行的进程: ${title}"
    return 0
  fi

  open_term "$title" "$cmd"
}

echo "[1/3] 启动 Gazebo 仿真..."
cleanup_orphaned_gazebo

if process_running "$GAZEBO_PATTERN" || gazebo_lock_held; then
  echo "[跳过] 已检测到正在运行或正在启动的 Gazebo 仿真。"
else
  GAZEBO_LOCK_FILE_Q="$(printf '%q' "$GAZEBO_LOCK_FILE")"
  # 释放 FD 9 后，open_term 才会进入保留 shell；否则终端 shell 会一直
  # 持有锁，导致下一次仿真即使旧 Gazebo 已退出也无法启动。
  GAZEBO_CMD="exec 9>${GAZEBO_LOCK_FILE_Q}; if ! flock -n 9; then echo '[跳过] 另一个启动脚本已占用 Gazebo 锁。'; else ros2 launch rmu_gazebo_simulator bringup_sim.launch.py; fi; flock -u 9; exec 9>&-"
  open_term "Gazebo 仿真" "$GAZEBO_CMD"
fi

# 组装重定位导航的启动命令。
if [ -z "$PRIOR_PCD_FILE" ]; then
  echo "[警告] 未指定先验点云地图 (PRIOR_PCD_FILE)，也未在默认位置找到 scans.pcd。" >&2
  echo "[警告] 重定位将使用 nav_simulation_launch.py 的默认路径，请确认该 PCD 存在。" >&2
fi

PRIOR_PCD_FILE_Q="$(printf '%q' "$PRIOR_PCD_FILE")"
NAV_CMD="ros2 launch srm27_nav_bringup nav_simulation_launch.py world:=${WORLD} slam:=False use_pcd_localization:=True use_composition:=False use_sim_time:=True use_rviz:=True prior_pcd_file:=${PRIOR_PCD_FILE_Q}"

echo "[2/3] 启动 RViz 导航 (PCD 重定位)..."
start_once \
  "RViz 重定位导航" \
  "$NAV_RELOC_PATTERN" \
  "$NAV_CMD"

echo "[3/3] 启动键鼠控制..."
start_once \
  "键鼠控制" \
  '[r]os2 run rmoss_gz_base test_chassis_cmd.py' \
  "ros2 run rmoss_gz_base test_chassis_cmd.py --ros-args -r __ns:=/red_standard_robot1/robot_base -p v:=0.8 -p w:=0.8"

echo "启动流程处理完成（终端模式: ${OPEN_MODE}，世界: ${WORLD}，先验地图: ${PRIOR_PCD_FILE:-未指定}）。"
