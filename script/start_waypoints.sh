#!/usr/bin/env bash
# =============================================================
# SRM 航点任务一键启动 (包装 script/waypoint_mission.py)
#
#   ./script/start_waypoints.sh                       # 点选模式: RViz 里点航点, 再调 ~/start
#   ./script/start_waypoints.sh --file m.yaml         # 直接执行航点文件里的航点(每个点都停车)
#   ./script/start_waypoints.sh --file m.yaml --pass-through   # 途径点模式: 除最后一个点(终点)
#                                                     # 外都是途径点, 途径点不刹车
#   ./script/start_waypoints.sh --save-file m.yaml    # 只收集航点, 用 ~/save 存盘
#   ./script/start_waypoints.sh --loop --retry 2      # 循环跑, 每个点失败重试 2 次
#
# 所有参数原样透传给 waypoint_mission.py, 完整说明见:
#   ./script/start_waypoints.sh -h
#   (或直接看 script/waypoint_mission.py 头部注释)
#
# 仿真 (Gazebo):
#   ./script/start_sim_nav.sh                         # 先起仿真 + 导航栈(默认命名空间
#                                                     # red_standard_robot1)
#   ./script/start_waypoints.sh --file m.yaml         # 命名空间会被自动探测, 直接这样跑就行
#   ./script/start_waypoints.sh --file m.yaml --namespace red_standard_robot1   # 也可以显式指定
#   说明:
#     - 仿真导航栈跑在命名空间下, 导航 action 在 /<ns>/navigate_to_pose、/<ns>/navigate_through_poses,
#       TF 话题在 /<ns>/tf(cf. nav2 launch 里的 SetRemap("/tf","tf"))。本脚本会自动探测命名空间
#       (ROS 图里根命名空间没有 /navigate_to_pose 而恰好有一个 /<ns>/navigate_to_pose 时),
#       也可以 --namespace 显式给; 显式给时不探测。
#     - 坐标系不用改: 仿真的 params 里 global_frame=map、robot_base_frame=base_link, 没有前缀。
#     - 仿真里**不要**加 --use-sim-time: Gazebo 暂停时脚本会跟着 /clock 冻住; 只有回放 rosbag 才需要。
#     - --pass-through 依赖一组耦合参数 (共享 BT 的 RemovePassedGoals radius=0.35 要小于
#       general_goal_checker.xy_goal_tolerance、又要大于切内弯的偏移量): 实车/仿真两份 Omni
#       参数 2026-10-09 起已都是 容差 0.4 + 前瞻 0.6/0.3/0.6, 两边表现一致。详见
#       waypoint_mission.py 头部注释。
#     - --pass-through 默认开"回头检测"(--backtrack-guard): 实测 nav2 的 RemovePassedGoals
#       没在删已过的途径点, 靠它自己发现"车在往回走"就取消当前航段、只重发还没开过的点。
#       详见 waypoint_mission.py 头部注释。--no-backtrack-guard 可关。
#
# 注意:
#   - 需要导航栈在跑(nav2 + bt_navigator), 即先执行 ./script/start_real_nav.sh (仿真则
#     ./script/start_sim_nav.sh)。
#     本脚本只下发导航目标, 不直接发速度: 速度仍走 Nav2 控制器 → velocity_smoother
#     → fake_vel_transform → /cmd_vel_chassis 这条链路, 底盘限幅/看门狗照旧生效。
#   - --pass-through 用的是 bt_navigator 的 navigate_through_poses 与
#     behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml, 默认导航栈
#     就带; 它只在最后一个点(终点)停车, 途径点不停车。失败时按 --retry 重发还没开过的
#     剩余点, 再用完则按 --on-failure abort/skip。细节见 waypoint_mission.py 头部注释。
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
  echo "[提示] 要下发航点请先起: ./script/start_real_nav.sh (仿真: ./script/start_sim_nav.sh)" >&2
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

# ---------- 命名空间自动探测 (仿真 start_sim_nav.sh 默认跑在 red_standard_robot1 下) ----------
# 只在用户没显式给 --namespace/-n 时探测:
#   根命名空间有 /navigate_to_pose        -> 实车, 不加 --namespace
#   只有唯一一个 /<ns>/navigate_to_pose   -> 仿真, 自动加 --namespace <ns>
#   有多个                                 -> 不猜, 提示用户自己指定
NS_ARGS=()
USER_GAVE_NS=0
for arg in "$@"; do
  case "$arg" in
    --namespace|--namespace=*|-n|-n=*) USER_GAVE_NS=1 ;;
  esac
done
if [ "$USER_GAVE_NS" -eq 0 ]; then
  ACTION_LIST="$(ros2 action list 2>/dev/null || true)"
  if [ -n "$ACTION_LIST" ] && ! grep -qx "/navigate_to_pose" <<<"$ACTION_LIST"; then
    NS_CANDIDATES="$(grep -E '^/[A-Za-z0-9_]+/navigate_to_pose$' <<<"$ACTION_LIST" \
      | sed -E 's#^/##; s#/navigate_to_pose$##' | sort -u || true)"
    NS_COUNT="$(printf '%s\n' "$NS_CANDIDATES" | grep -c . || true)"
    NS_ONE="$(printf '%s\n' "$NS_CANDIDATES" | head -n 1)"
    if [ "$NS_COUNT" = "1" ]; then
      NS_ARGS=(--namespace "$NS_ONE")
      echo "[提示] 检测到带命名空间的导航栈 /$NS_ONE (仿真常见), 已自动加 --namespace $NS_ONE"
    elif [ "$NS_COUNT" -gt 1 ]; then
      echo "[提示] 检测到多个命名空间的导航栈: $(printf '%s' "$NS_CANDIDATES" | tr '\n' ' ')" >&2
      echo "[提示] 请用 --namespace <名字> 指定要控制哪一个。" >&2
    fi
  fi
fi

cd "$WS_DIR"
exec python3 "$SCRIPT_DIR/waypoint_mission.py" "${NS_ARGS[@]}" "$@"
