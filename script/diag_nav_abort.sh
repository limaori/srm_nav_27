#!/usr/bin/env bash
# 单次、安全的 SRM 仿真导航诊断运行。
#
# 安全约定（避免再次出现多套仿真栈叠起导致整机卡死）：
#   1. 启动前强制检查并清空所有残留进程，有残留就直接退出；
#   2. 用 setsid 起独立进程组，退出时按进程组 kill，确保 gzserver / bridge /
#      适配器 / Nav2 节点全部回收；
#   3. 日志写到工作空间内的 log_diag/（已 gitignore），重启后仍可查阅；
#   4. 全程 headless（gui:=false），不启动 RViz。
# 不用 set -u：ROS 的 setup.bash 会引用未定义的 AMENT_TRACE_SETUP_FILES。

WS=/home/srm/srm_nav_27
OUT="$WS/log_diag/nav_abort"
NS=red_standard_robot1
MAP="$WS/src/srm27_navigation/srm27_nav_bringup/map/simulation/rmuc_2025_tunnel.yaml"
PARAMS="$WS/src/srm27_navigation/srm27_nav_bringup/config/simulation/nav2_params_srm.yaml"
WORLD="${WORLD:-rmuc_2025}"
GOAL_X="${GOAL_X:-1.5}"
GOAL_Y="${GOAL_Y:-1.5}"

mkdir -p "$OUT"
cd "$WS"

# ---------- 1. 残留检查 ----------
LEFTOVER_PATTERNS=(
  'srm27_gazebo_simulator' 'srm27_nav_bringup' 'srm27_chassis_control'
  'srm_velocity_adapter' 'srm_cmd_mux' 'rotation_controller' 'rotation_test_sender'
  'ign gazebo' 'gz sim' 'ruby.*gazebo' 'ros_gz_bridge'
  'static_transform_publisher' 'simulation_ground_truth_odometry' 'terrainAnalysis'
  'map_server' 'lifecycle_manager' 'controller_server' 'planner_server'
  'bt_navigator' 'behavior_server' 'velocity_smoother' 'waypoint_follower'
  'smoother_server' 'sensor_scan_generation' 'robot_state_publisher'
  'point_lio' 'loam_interface' 'small_gicp' 'rviz2' 'component_container'
)
leftover=""
for p in "${LEFTOVER_PATTERNS[@]}"; do
  hits="$(pgrep -f "$p" 2>/dev/null | tr '\n' ' ')"
  [ -n "$hits" ] && leftover="$leftover $p[$hits]"
done
if [ -n "$leftover" ]; then
  echo "[中止] 检测到残留仿真进程，先清理再运行：$leftover" | tee "$OUT/run.log"
  exit 1
fi
echo "[1/6] 无残留进程，开始。" | tee "$OUT/run.log"

# ---------- 2. 启动（独立进程组） ----------
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set +u

setsid ros2 launch srm27_gazebo_simulator srm_sim.launch.py \
  world:="$WORLD" gui:=false run_immediately:=true robot_name:="$NS" \
  > "$OUT/01_sim.log" 2>&1 &
SIM_PGID=$!
sleep 3
setsid ros2 launch srm27_nav_bringup nav_srm_simulation_launch.py \
  namespace:="$NS" world:="$WORLD" map:="$MAP" params_file:="$PARAMS" \
  slam:=False use_pcd_localization:=False use_composition:=False \
  use_sim_time:=True use_rviz:=False \
  > "$OUT/02_nav.log" 2>&1 &
NAV_PGID=$!

cleanup() {
  kill -INT -"$SIM_PGID" -"$NAV_PGID" 2>/dev/null
  sleep 3
  kill -KILL -"$SIM_PGID" -"$NAV_PGID" 2>/dev/null
  sleep 1
  for p in "${LEFTOVER_PATTERNS[@]}"; do
    pkill -9 -f "$p" 2>/dev/null
  done
  echo "[6/6] 已清理本次运行的进程组。" | tee -a "$OUT/run.log"
}
trap cleanup EXIT

# ---------- 3. 等栈起来 ----------
sleep 45
echo "[2/6] 栈已启动，抓取行为树日志与初始状态。" | tee -a "$OUT/run.log"
timeout 100 ros2 topic echo /$NS/behavior_tree_log --field "events[:]{node_name,previous_status,current_status}" \
  > "$OUT/03_bt_log.txt" 2>&1 &
BT_PID=$!
timeout 6 ros2 topic echo /$NS/odometry --field pose.pose.position --once > "$OUT/04_start_pose.txt" 2>&1
timeout 6 ros2 topic info /$NS/cmd_vel_nav >> "$OUT/run.log" 2>&1

# ---------- 4. 规划一条路径（只看规划是否成功） ----------
echo "[3/6] 规划到 map($GOAL_X, $GOAL_Y)。" | tee -a "$OUT/run.log"
timeout 25 ros2 action send_goal /$NS/compute_path_to_pose nav2_msgs/action/ComputePathToPose \
  "{goal: {header: {frame_id: map}, pose: {position: {x: $GOAL_X, y: $GOAL_Y, z: 0.0}, orientation: {w: 1.0}}}, use_start: false}" \
  > "$OUT/05_plan.log" 2>&1
grep -E "number_of_poses|error_code|Goal finished" "$OUT/05_plan.log" | tail -3 | tee -a "$OUT/run.log"

# ---------- 5. 下发导航目标并观察 ----------
echo "[4/6] 下发 NavigateToPose 目标。" | tee -a "$OUT/run.log"
timeout 150 ros2 action send_goal /$NS/navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: $GOAL_X, y: $GOAL_Y, z: 0.0}, orientation: {w: 1.0}}}}" \
  > "$OUT/06_goal.log" 2>&1 &
GOAL_PID=$!

: > "$OUT/07_track.txt"
for i in $(seq 1 14); do
  sleep 7
  # 内存看门狗：可用内存低于 1.5 GB 时立即结束，避免整机再次卡死。
  avail_mb=$(awk '/MemAvailable/ {print int($2/1024)}' /proc/meminfo)
  if [ "${avail_mb:-0}" -lt 1536 ]; then
    echo "[看门狗] 可用内存仅 ${avail_mb} MB，提前结束诊断运行。" | tee -a "$OUT/run.log"
    break
  fi
  {
    printf "t=%3ds mem_avail=%sMB " $((i*7)) "$avail_mb"
    timeout 5 ros2 topic echo /$NS/odometry --field pose.pose.position --once 2>/dev/null | tr -d '\n' | sed 's/  */ /g'
    printf " | cmd_nav "
    timeout 5 ros2 topic echo /$NS/cmd_vel_nav --once 2>/dev/null | tr -d '\n' | sed 's/  */ /g' | cut -c1-90
    printf " | cmd_sim "
    timeout 5 ros2 topic echo /$NS/cmd_vel_sim --once 2>/dev/null | tr -d '\n' | sed 's/  */ /g' | cut -c1-90
    echo
  } >> "$OUT/07_track.txt" 2>&1
done

wait $GOAL_PID 2>/dev/null
kill -INT $BT_PID 2>/dev/null

# ---------- 6. 汇总 ----------
{
  echo "===== goal 结果 ====="
  grep -E "Goal accepted|Goal finished" "$OUT/06_goal.log" | tail -3
  echo "===== 跟踪曲线 ====="
  cat "$OUT/07_track.txt"
  echo "===== 行为树事件 ====="
  cat "$OUT/03_bt_log.txt"
  echo "===== nav 日志中的 ERROR/WARN ====="
  grep -iE "\[ERROR\]|\[WARN\]" "$OUT/02_nav.log" | grep -v "已清零" | tail -40
} > "$OUT/08_summary.txt" 2>&1

echo "[5/6] 汇总写入 $OUT/08_summary.txt" | tee -a "$OUT/run.log"
exit 0
