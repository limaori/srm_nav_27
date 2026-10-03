#!/usr/bin/env bash
# =============================================================
# 一键启动导航仿真：Gazebo 仿真 + RViz 导航
#
# 用法:
#   ./script/start_nav.sh
#   或
#   SRM27_WS_DIR=/path/to/ws ./script/start_nav.sh
#
# 说明:
#   - 默认在同一个窗口里打开 2 个标签页（Gazebo / RViz）。
#   - 若想换成每个命令一个独立窗口, 执行时加环境变量 OPEN_MODE=window 即可。
#   - 每个命令都会先 source 工作空间的 install/setup.bash。
# =============================================================

set -euo pipefail

# 工作目录 = 脚本所在目录的上一级（即 ROS2 工作空间根目录）
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# 允许通过环境变量覆盖
WS_DIR="${SRM27_WS_DIR:-$WS_DIR}"

# 窗口 / 标签页 模式: "tab" 同一窗口多个标签页(默认), "window" 每个命令一个独立窗口
OPEN_MODE="${OPEN_MODE:-tab}"

# 终端程序（可改成 konsole / xfce4-terminal 等）
TERMINAL="${TERMINAL:-gnome-terminal}"

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

# 检查命令是否已经在运行。
# 使用带方括号的正则，避免 pgrep 把自身的匹配命令算进去。
process_running() {
  pgrep -af "$1" >/dev/null 2>&1
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

echo "[1/2] 启动 Gazebo 仿真..."
start_once \
  "Gazebo 仿真" \
  '[r]os2 launch rmu_gazebo_simulator bringup_sim.launch.py|[i]gn gazebo|[g]z sim' \
  "ros2 launch rmu_gazebo_simulator bringup_sim.launch.py"

echo "[2/2] 启动 RViz 导航..."
start_once \
  "RViz 导航" \
  '[r]os2 launch srm27_nav_bringup nav_simulation_launch.py' \
  "ros2 launch srm27_nav_bringup nav_simulation_launch.py world:=rmuc_2025 slam:=False use_pcd_localization:=False use_composition:=False use_sim_time:=True use_rviz:=True"

echo "启动流程处理完成（终端模式: ${OPEN_MODE}）。"
