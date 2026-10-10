#!/usr/bin/env bash

set -u

# 结束本项目的 Nav2 导航栈。
#
# 为什么需要它：costmap / controller / planner 的参数都是**节点 configure 时**读一次的，
# 其中膨胀半径 (inflation_layer) 在 Humble 里没有动态参数回调 —— 改了参数文件后
# 不重启导航栈，跑出来的还是旧值。start_sim_nav.sh 的启动前清理过去只杀 Gazebo 与 RViz，
# 导航栈则由 start_once 的"检测到在跑就跳过"逻辑留着，于是很容易出现
# "Gazebo 是新的、导航是旧的" 这种错配（改了膨胀半径/限速却看不到变化）。
#
# 清理范围（两遍，先精确后兜底）：
#   1) 本包导航入口 launch 进程（默认 nav_srm_simulation_launch / nav_simulation_launch）
#      以及它的**整棵子进程树** —— 由 ros2 launch 自己负责优雅收尾；
#   2) launch 已被强杀、子进程被 systemd --user 接管的"孤儿"：按可执行文件名
#      + 命名空间过滤（默认 /red_standard_robot1），不会误伤别的工作空间/别命名空间的节点。
#
# 环境变量：
#   WAIT_SECONDS  等待优雅退出的秒数（默认 6）
#   ROBOT_NS      命名空间过滤（默认 red_standard_robot1），用于识别孤儿进程
#   NAV_LAUNCH_PATTERN  覆盖入口 launch 的匹配式（例如要连实车入口一起清时）

WAIT_SECONDS="${WAIT_SECONDS:-8}"
ROBOT_NS="${ROBOT_NS:-red_standard_robot1}"
TARGET_UID="$(id -u)"

if ! [[ "$WAIT_SECONDS" =~ ^[0-9]+$ ]]; then
  echo "[错误] WAIT_SECONDS 必须是非负整数。" >&2
  exit 2
fi

# 本包导航入口 launch（[r]os2 的写法让 pgrep 不去匹配自己这条命令行）。
NAV_LAUNCH_PATTERN="${NAV_LAUNCH_PATTERN:-[r]os2[[:space:]]+launch[[:space:]]+srm27_nav_bringup[[:space:]]+(nav_srm_simulation_launch|nav_simulation_launch)\.py}"

# 孤儿兜底：Nav2 核心节点 + 本仓库导航链上会被 launch 带起来的进程。
ORPHAN_EXEC_PATTERN='(^|/)(controller_server|planner_server|smoother_server|behavior_server|bt_navigator|waypoint_follower|velocity_smoother|map_server|amcl|slam_toolbox|lifecycle_manager[a-z_]*|component_container|static_transform_publisher|terrainAnalysis|terrainAnalysisExt|loam_interface|sensor_scan_generation|small_gicp_relocalization|pointcloud_to_laserscan|ign_sim_pointcloud_tool)([[:space:]]|$)'

cmdline_of() {
  tr '\0' ' ' < "/proc/$1/cmdline" 2>/dev/null || true
}

# 只认"真正的 ros2 CLI 进程"作为根：
# 启动用的终端包装 shell（bash -lc "… ros2 launch …"）命令行里也含同样的字样，
# 把它当根会顺着父链把外层 shell / 终端甚至沙箱进程一起算进来（实测会误杀自己）。
# 用 /proc/<pid>/comm 过滤，只看 python / ros2 解释器进程。
comm_of() {
  cat "/proc/$1/comm" 2>/dev/null || true
}

is_ros2_cli_process() {
  [[ "$(comm_of "$1")" =~ ^python[0-9.]*$|^ros2$ ]]
}

# 递归收集某进程的所有后代：ros2 launch 被杀后，子进程可能被接管成孤儿，
# 所以这里按"父链"整棵收，而不是只杀 launch 自己。
descendants_of() {
  local -a queue=("$@") children
  local pid
  while ((${#queue[@]} > 0)); do
    pid="${queue[0]}"
    queue=("${queue[@]:1}")
    mapfile -t children < <(pgrep -P "$pid" 2>/dev/null || true)
    ((${#children[@]} == 0)) && continue
    printf '%s\n' "${children[@]}"
    queue+=("${children[@]}")
  done
}

find_nav_pids() {
  local -a roots=() found=() candidates=() raw_roots=()
  local pid

  mapfile -t raw_roots < <(pgrep -u "$TARGET_UID" -f "$NAV_LAUNCH_PATTERN" 2>/dev/null || true)
  for pid in "${raw_roots[@]:-}"; do
    [ -n "$pid" ] || continue
    is_ros2_cli_process "$pid" && roots+=("$pid")
  done
  if ((${#roots[@]} > 0)); then
    found+=("${roots[@]}")
    while read -r pid; do
      [ -n "$pid" ] && found+=("$pid")
    done < <(descendants_of "${roots[@]}")
  fi

  # 孤儿：可执行文件名匹配，并且命令行里带着目标命名空间（避免误伤同机其他工作空间）。
  mapfile -t candidates < <(pgrep -u "$TARGET_UID" -f "$ORPHAN_EXEC_PATTERN" 2>/dev/null || true)
  for pid in "${candidates[@]:-}"; do
    [ -n "$pid" ] || continue
    case "$(cmdline_of "$pid")" in
      *"__ns:=/${ROBOT_NS}"*) found+=("$pid") ;;
    esac
  done

  printf '%s\n' "${found[@]:-}" | grep -E '^[0-9]+$' | sort -un || true
}

mapfile -t pids < <(find_nav_pids)

if ((${#pids[@]} == 0)); then
  echo "未发现正在运行的 Nav2 导航栈（命名空间 /${ROBOT_NS}）。"
  exit 0
fi

echo "发现以下 Nav2 相关进程："
ps -o pid=,ppid=,stat=,cmd= -p "$(IFS=,; echo "${pids[*]}")" 2>/dev/null || true

echo "正在请求导航栈退出（SIGINT，让 ros2 launch 走收尾流程）..."
kill -INT "${pids[@]}" 2>/dev/null || true

for ((second = 0; second < WAIT_SECONDS; second++)); do
  mapfile -t remaining < <(find_nav_pids)
  ((${#remaining[@]} == 0)) && break
  sleep 1
done

# launch 收尾（停各生命周期节点）有时比 6 秒慢，先给一次 SIGTERM，
# 再等 2 秒才用 SIGKILL，尽量避免把容器直接打掉。
mapfile -t remaining < <(find_nav_pids)
if ((${#remaining[@]} > 0)); then
  echo "等待 ${WAIT_SECONDS} 秒后仍有残留，先发 SIGTERM：${remaining[*]}"
  kill -TERM "${remaining[@]}" 2>/dev/null || true
  for ((second = 0; second < 2; second++)); do
    mapfile -t remaining < <(find_nav_pids)
    ((${#remaining[@]} == 0)) && break
    sleep 1
  done
fi

mapfile -t remaining < <(find_nav_pids)
if ((${#remaining[@]} > 0)); then
  echo "仍在运行，强制结束：${remaining[*]}"
  kill -KILL "${remaining[@]}" 2>/dev/null || true
fi

sleep 0.2
mapfile -t remaining < <(find_nav_pids)
if ((${#remaining[@]} > 0)); then
  echo "[错误] 以下 Nav2 进程未能结束：${remaining[*]}" >&2
  exit 1
fi

echo "Nav2 导航栈已结束（下次启动会重新读取参数文件）。"
