#!/usr/bin/env bash
# 评估器离线自检（不依赖 Gazebo）：验证 gicp_truth_evaluator 能按方案 §8.2 正确
# 判定「通过 / 超时失败 / 低 RMSE 假成功」。
#
# 用法： bash script/gicp_validation/selftest_evaluator.sh [good|bad|fake|noisy]
set -uo pipefail

WS=/home/srm/srm_nav_27
SCENARIO=${1:-good}
VD=$WS/tmp/gicp_validation/selftest_$SCENARIO

set +u
export ROS_HOME=/tmp/ros_home_gicp_selftest
export ROS_LOG_DIR=$ROS_HOME/log
export ROS_DOMAIN_ID=91
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export RCUTILS_LOGGING_BUFFERED_STREAM=1
mkdir -p "$ROS_LOG_DIR"
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u
cd "$WS"

rm -rf "$VD"
mkdir -p "$VD"

"$WS/install/srm27_gicp_validation/bin/gicp_truth_evaluator" \
  --ros-args -p output_dir:="$VD" -p run_id:="selftest_$SCENARIO" \
  > "$VD/evaluator.log" 2>&1 &
EVAL_PID=$!
sleep 4

# 被测发布者先跑起来（发送初值事件），评估器在事件之后才开始计时
DURATION=16
[ "$SCENARIO" = "good" ] && DURATION=13
[ "$SCENARIO" = "fake" ] && DURATION=14
[ "$SCENARIO" = "noisy" ] && DURATION=13
timeout "$DURATION" "$WS/install/srm27_gicp_validation/bin/gicp_eval_selftest" \
  --ros-args -p scenario:="$SCENARIO" > "$VD/publisher.log" 2>&1 &
PUB_PID=$!

# 等初值事件真的被评估器收到（日志出现「初值事件」）后再 start_run
for _ in $(seq 1 40); do
  grep -q "初值事件" "$VD/evaluator.log" 2>/dev/null && break
  sleep 0.5
done
timeout 10 ros2 service call /gicp_eval/start_run std_srvs/srv/Trigger "{}" >/dev/null 2>&1
wait "$PUB_PID" 2>/dev/null || true
sleep 1
timeout 10 ros2 service call /gicp_eval/stop_run std_srvs/srv/Trigger "{}" >/dev/null 2>&1
sleep 1
kill -INT "$EVAL_PID" 2>/dev/null
sleep 2

echo "==================== 评估器日志 ===================="
grep -E "判定|可疑|初值事件|真值起点" "$VD/evaluator.log" | tail -10
echo "==================== summary.json ===================="
python3 - "$VD/selftest_$SCENARIO/summary.json" <<'PY'
import json, sys
try:
    data = json.load(open(sys.argv[1]))
except FileNotFoundError:
    print("未生成 summary.json"); raise SystemExit(1)
keys = ["case_id","passed","failure_reason","accepted_count","e_xy_max","e_yaw_max",
        "suspect_count","init_duration_wall","init_error_xy","init_error_yaw"]
for k in keys:
    v = data.get(k)
    if isinstance(v, float):
        v = round(v, 4)
    print(f"  {k}: {v}")
raise SystemExit(0 if data.get("passed") else 1)
PY
