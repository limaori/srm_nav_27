#!/usr/bin/env bash
# =============================================================
# 一键启动仿真流程：Gazebo 仿真 + RViz 导航 + 键鼠控制
#
# 用法:
#   ./script/start_slam.sh
#   或
#   SRM27_WS_DIR=/path/to/ws ./script/start_slam.sh
#
# 说明:
#   - 默认在同一个窗口里打开 3 个标签页（Gazebo / RViz / 键鼠控制）。
#   - 若想换成每个命令一个独立窗口, 执行时加环境变量 OPEN_MODE=window 即可。
#   - 每个命令都会先 source 工作空间的 install/setup.bash。
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

# Gazebo 启动锁由终端里的 ros2 launch 进程持有，直到该仿真退出。
# 这样即使两个 start_slam.sh 几乎同时执行，也最多只有一个 Gazebo 真正启动。
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

echo "[2/3] 启动 RViz 导航..."
start_once \
  "RViz 导航" \
  '[r]os2 launch srm27_nav_bringup nav_simulation_launch.py' \
  "ros2 launch srm27_nav_bringup nav_simulation_launch.py world:=rmuc_2025 slam:=True use_sim_time:=True use_rviz:=True"

echo "[3/3] 启动键鼠控制..."
start_once \
  "键鼠控制" \
  '[r]os2 run rmoss_gz_base test_chassis_cmd.py' \
  "ros2 run rmoss_gz_base test_chassis_cmd.py --ros-args -r __ns:=/red_standard_robot1/robot_base -p v:=0.8 -p w:=0.8"

echo "启动流程处理完成（终端模式: ${OPEN_MODE}）。"
