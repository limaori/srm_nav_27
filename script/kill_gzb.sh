#!/usr/bin/env bash

set -u

WAIT_SECONDS="${WAIT_SECONDS:-5}"

if ! [[ "$WAIT_SECONDS" =~ ^[0-9]+$ ]]; then
  echo "[错误] WAIT_SECONDS 必须是非负整数。" >&2
  exit 2
fi

# 覆盖 Gazebo 本体、启动器，以及本项目启动的 bridge/机器人节点。
#
# ros2 launch 被强制结束后，子进程可能被 systemd --user 接管，继续留在
# 后台。尤其是 /clock bridge 仍会发布时钟，下一次启动仿真时会造成两个
# 时钟源同时写入 /clock。因此这里也要清理这些“孤儿”仿真进程。
PATTERN='(^|/)(gzserver|gzclient|gazebo)([[:space:]]|$)|[i]gn[[:space:]]+gazebo|[g]z[[:space:]]+sim|[r]os2[[:space:]]+launch[[:space:]]+rmu_gazebo_simulator[[:space:]]+bringup_sim\.launch\.py|[/]ros_gz_bridge/parameter_bridge[[:space:]]+/clock@rosgraph_msgs/msg/Clock\[gz.msgs.Clock|[/]ros_gz_bridge/parameter_bridge.*__ns:=/red_standard_robot1|[/]rmoss_gz_base/rmua19_robot_base.*__ns:=/red_standard_robot1|[r]obot_state_publisher.*__ns:=/red_standard_robot1'

TARGET_UID="$(id -u)"

find_gazebo_pids() {
  pgrep -u "$TARGET_UID" -f "$PATTERN" 2>/dev/null | sort -un || true
}

mapfile -t pids < <(find_gazebo_pids)

if ((${#pids[@]} == 0)); then
  echo "未发现正在运行的 Gazebo 仿真或其残留进程。"
  exit 0
fi

echo "发现以下 Gazebo 仿真相关进程："
ps -o pid=,stat=,cmd= -p "$(IFS=,; echo "${pids[*]}")"

echo "正在请求进程退出（SIGINT）..."
kill -INT "${pids[@]}" 2>/dev/null || true

for ((second = 0; second < WAIT_SECONDS; second++)); do
  mapfile -t remaining < <(find_gazebo_pids)
  ((${#remaining[@]} == 0)) && break
  sleep 1
done

mapfile -t remaining < <(find_gazebo_pids)
if ((${#remaining[@]} > 0)); then
  echo "等待 ${WAIT_SECONDS} 秒后仍有残留，正在强制结束：${remaining[*]}"
  kill -KILL "${remaining[@]}" 2>/dev/null || true
fi

sleep 0.2
mapfile -t remaining < <(find_gazebo_pids)
if ((${#remaining[@]} > 0)); then
  echo "[错误] 以下 Gazebo 仿真相关进程未能结束：${remaining[*]}" >&2
  exit 1
fi

echo "Gazebo 仿真及其残留进程已结束。"
