#!/usr/bin/env bash
# =============================================================
# SRM 航点任务一键启动 (包装 script/waypoint_mission.py)
#
#   ./script/start_waypoints.sh                       # 点选模式: RViz 里点航点, 再调 ~/start
#   ./script/start_waypoints.sh --file m.yaml         # 直接执行航点文件里的航点
#   ./script/start_waypoints.sh --save-file m.yaml    # 只收集航点, 用 ~/save 存盘
#   ./script/start_waypoints.sh --loop --retry 2      # 循环跑, 每个点失败重试 2 次
#
# 所有参数原样透传给 waypoint_mission.py, 完整说明见:
#   ./script/start_waypoints.sh -h
#   (或直接看 script/waypoint_mission.py 头部注释)
#
# 注意:
#   - 需要导航栈在跑(nav2 + bt_navigator), 即先执行 ./script/start_real_nav.sh。
#     本脚本只下发导航目标, 不直接发速度: 速度仍走 Nav2 控制器 → velocity_smoother
#     → fake_vel_transform → /cmd_vel_chassis 这条链路, 底盘限幅/看门狗照旧生效。
#   - RViz 点选请用工具栏的 "2D Goal Pose" (发 /goal_pose) 或 "Publish Point"
#     (发 /clicked_point); 别用 Nav2 面板里直接下发 action 的 "Nav2 Goal" 工具,
#     否则它和本脚本会互相抢目标。
# =============================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="${SRM27_WS_DIR:-$(cd "$SCRIPT_DIR/.." && pwd)}"

if [ ! -f "$WS_DIR/install/setup.bash" ]; then
  echo "[错误] 未找到 $WS_DIR/install/setup.bash" >&2
  echo "[提示] 先编译工作空间, 或用 SRM27_WS_DIR=/path/to/ws 指定。" >&2
  exit 1
fi

# 没起导航栈时只提示不阻断: 点选/查看航点(只记录模式)不需要导航栈,
# 真要下发目标时 waypoint_mission.py 自己会等到超时并给出明确报错。
if ! pgrep -u "$(id -u)" -f '[b]t_navigator|[n]av2_container|[n]av2_stack_launch' >/dev/null 2>&1; then
  echo "[提示] 没检测到导航栈 (bt_navigator / nav2_container)。" >&2
  echo "[提示] 要下发航点请先起: ./script/start_real_nav.sh" >&2
  echo "[提示] 现在只做点选/记录也可以继续。" >&2
fi

# ROS 的 setup.bash 本身不是 set -u 安全的(会引用未绑定的 AMENT_TRACE_SETUP_FILES),
# 在 -u 下 source 会直接报错中止, 所以这里临时关掉 -u。
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$WS_DIR/install/setup.bash"
set -u
cd "$WS_DIR"
exec python3 "$SCRIPT_DIR/waypoint_mission.py" "$@"
