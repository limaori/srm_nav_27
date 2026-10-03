#!/usr/bin/env bash

set -u

# 等待 RViz 自行清理资源并退出的秒数，可在运行脚本时通过环境变量覆盖。
WAIT_SECONDS="${WAIT_SECONDS:-5}"

if ! [[ "$WAIT_SECONDS" =~ ^[0-9]+$ ]]; then
  echo "[错误] WAIT_SECONDS 必须是非负整数。" >&2
  exit 2
fi

TARGET_UID="$(id -u)"

find_rviz_pids() {
  {
    pgrep -u "$TARGET_UID" -x rviz 2>/dev/null || true
    pgrep -u "$TARGET_UID" -x rviz2 2>/dev/null || true
  } | sort -un
}

mapfile -t pids < <(find_rviz_pids)

if ((${#pids[@]} == 0)); then
  echo "未发现当前用户正在运行的 RViz 进程。"
  exit 0
fi

echo "发现以下 RViz 进程："
ps -o pid=,stat=,cmd= -p "$(IFS=,; echo "${pids[*]}")" 2>/dev/null || true

echo "正在请求进程退出（SIGINT）..."
kill -INT "${pids[@]}" 2>/dev/null || true

for ((second = 0; second < WAIT_SECONDS; second++)); do
  mapfile -t remaining < <(find_rviz_pids)
  ((${#remaining[@]} == 0)) && break
  sleep 1
done

mapfile -t remaining < <(find_rviz_pids)
if ((${#remaining[@]} > 0)); then
  echo "等待 ${WAIT_SECONDS} 秒后仍有残留，正在强制结束：${remaining[*]}"
  kill -KILL "${remaining[@]}" 2>/dev/null || true
fi

sleep 0.2
mapfile -t remaining < <(find_rviz_pids)
if ((${#remaining[@]} > 0)); then
  echo "[错误] 以下 RViz 进程未能结束：${remaining[*]}" >&2
  exit 1
fi

echo "RViz 残留进程已清理。"
