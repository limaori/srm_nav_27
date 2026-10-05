#!/usr/bin/env bash
# =============================================================
# SRM 仿真回归运行器 (实施方案 §7 分阶段验证 + §8 测试矩阵)
#
# 设计要点:
#   * 每个 session 只起「一套」仿真栈, 多个场景复用同一套栈, 避免叠栈,
#     session 结束时按进程组回收全部子进程;
#   * 启动前检查残留, 运行中带内存看门狗(可用内存低于 1.5 GB 立即停止);
#   * 每个场景单独落盘: metadata.yaml + 速度链路 CSV + 汇总 + 该场景日志;
#   * 所有速度都走真实链路 —— 平移发 cmd_vel_nav, 自转发 rotation_cmd 参数,
#     cmd_vel_sim 始终只由 srm_cmd_mux 发布。
#
# 场景 (session: field = 空场 srm_empty; nav = 场地 rmuc_2025)
#   field/vx            纯 vx=+0.3          (期望 +x 前进, 实际 vx≈0.3)
#   field/vy            纯 vy=+0.3          (期望 +y 左移, x 不变)
#   field/diagonal      vx=vy=+0.5          (期望按模长限到 0.5, 各 0.354)
#   field/wz_positive   纯 wz=+1.0          (期望逆时针, 实际 wz≈+1.0)
#   field/wz_negative   纯 wz=-1.0          (期望顺时针, 实际 wz≈-1.0)
#   field/overlay_constant   vx=0.25 + wz=+1.0        (平移 + 恒速自转)
#   field/overlay_periodic   vx=0.25 + 周期 offset1 amp0.5 T4 (平移 + 周期自转)
#   field/timeout_zeroing    发 5 s 后停发, 再观察 4 s  (平移清零, 自转保持)
#   nav/plain                导航, 不自转
#   nav/constant_rotation    导航 + 恒速自转
#   nav/periodic_rotation    导航 + 周期自转
#
# 用法:
#   ./script/srm_regression.sh                      # 跑全部
#   ./script/srm_regression.sh --session field      # 只跑空场
#   ./script/srm_regression.sh --session nav --cases plain,constant_rotation
#   RECORD_CLOUDS=1 ./script/srm_regression.sh --session field   # 额外记录点云(体积大)
# =============================================================

set -u

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WS"

SESSION="all"
CASES=""
RECORD_CLOUDS="${RECORD_CLOUDS:-0}"
NAV_NAMESPACE="red_standard_robot1"
MEM_FLOOR_MB="${MEM_FLOOR_MB:-1536}"

while [ $# -gt 0 ]; do
  case "$1" in
    --session) SESSION="${2:?--session 需要 field|nav|all}"; shift 2 ;;
    --cases)   CASES="${2:?--cases 需要逗号分隔的场景名}"; shift 2 ;;
    -h|--help)
      awk 'NR == 1 { next }
           /^# =+[[:space:]]*$/ { sep++; if (sep == 2) exit; next }
           { sub(/^# ?/, ""); print }' "${BASH_SOURCE[0]}"
      exit 0 ;;
    *) echo "[错误] 未知参数: $1" >&2; exit 2 ;;
  esac
done

MAP_FILE="$WS/src/srm27_navigation/srm27_nav_bringup/map/simulation/rmuc_2025_tunnel.yaml"
PARAMS_FILE="$WS/src/srm27_navigation/srm27_nav_bringup/config/simulation/nav2_params_srm.yaml"
STAMP="$(date +%Y-%m-%d_%H-%M-%S)"
OUT_ROOT="$WS/log_diag/regression_$STAMP"

LEFTOVER_PATTERNS=(
  'srm27_gazebo_simulator' 'srm27_nav_bringup' 'srm27_chassis_control'
  'srm_velocity_adapter' 'srm_cmd_mux' 'rotation_controller' 'rotation_test_sender'
  'ign gazebo' 'gz sim' 'ruby.*gazebo' 'ros_gz_bridge'
  'static_transform_publisher' 'simulation_ground_truth_odometry' 'terrainAnalysis'
  'map_server' 'lifecycle_manager' 'controller_server' 'planner_server'
  'bt_navigator' 'behavior_server' 'velocity_smoother' 'waypoint_follower'
  'smoother_server' 'sensor_scan_generation' 'robot_state_publisher'
  'point_lio' 'loam_interface' 'small_gicp' 'rviz2' 'component_container'
  'srm_velocity_monitor'
)

selected() {
  [ -z "$CASES" ] && return 0
  case ",$CASES," in *",$1,"*) return 0 ;; *) return 1 ;; esac
}

check_leftovers() {
  local found=""
  for p in "${LEFTOVER_PATTERNS[@]}"; do
    pgrep -f "$p" >/dev/null 2>&1 && found="$found $p"
  done
  [ -n "$found" ] && { echo "[错误] 检测到残留进程:$found" >&2; echo "[提示] 先运行 ./script/clean_sim_processes.sh" >&2; return 1; }
  return 0
}

kill_leftovers() {
  for _ in 1 2; do
    for p in "${LEFTOVER_PATTERNS[@]}"; do pkill -9 -f "$p" 2>/dev/null; done
    sleep 1
  done
}

mem_avail_mb() { awk '/MemAvailable/ {print int($2/1024)}' /proc/meminfo; }

SIM_PGID=""
NAV_PGID=""
MON_PID=""

stop_session() {
  [ -n "$MON_PID" ] && kill -INT "$MON_PID" 2>/dev/null
  [ -n "$NAV_PGID" ] && { kill -INT -"$NAV_PGID" 2>/dev/null; }
  [ -n "$SIM_PGID" ] && { kill -INT -"$SIM_PGID" 2>/dev/null; }
  sleep 3
  [ -n "$NAV_PGID" ] && kill -KILL -"$NAV_PGID" 2>/dev/null
  [ -n "$SIM_PGID" ] && kill -KILL -"$SIM_PGID" 2>/dev/null
  sleep 1
  kill_leftovers
  SIM_PGID=""; NAV_PGID=""; MON_PID=""
}
trap stop_session EXIT

start_sim() {  # $1 = world
  setsid ros2 launch srm27_gazebo_simulator srm_sim.launch.py \
    world:="$1" gui:=false run_immediately:=true robot_name:="$NAV_NAMESPACE" \
    > "$CASE_DIR/sim.log" 2>&1 &
  SIM_PGID=$!
}

publish_nav() {  # $1 = x  $2 = y  $3 = 持续秒数(后台)
  local seconds="$3"
  timeout "$seconds" ros2 topic pub -r 50 "/$NAV_NAMESPACE/cmd_vel_nav" \
    geometry_msgs/msg/Twist \
    "{linear: {x: $1, y: $2, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
    > /dev/null 2>&1
}

# CLI_TIMEOUT: 每个 ros2 CLI 调用的硬超时。实测在 rosbag 写盘抢占 CPU 时，
# `ros2 param set` 偶发长时间阻塞；不加超时会把整个回归挂死。
CLI_TIMEOUT="${CLI_TIMEOUT:-15}"

set_rotation() {  # $1 mode $2 speed $3 offset $4 amplitude $5 period $6 sine
  timeout "$CLI_TIMEOUT" ros2 service call "/$NAV_NAMESPACE/rotation_test_sender/disable" std_srvs/srv/Trigger >/dev/null 2>&1
  timeout "$CLI_TIMEOUT" ros2 param set "/$NAV_NAMESPACE/rotation_test_sender" rotation_mode "$1" >/dev/null 2>&1
  timeout "$CLI_TIMEOUT" ros2 param set "/$NAV_NAMESPACE/rotation_test_sender" angular_speed "$2" >/dev/null 2>&1
  timeout "$CLI_TIMEOUT" ros2 param set "/$NAV_NAMESPACE/rotation_test_sender" offset "$3" >/dev/null 2>&1
  timeout "$CLI_TIMEOUT" ros2 param set "/$NAV_NAMESPACE/rotation_test_sender" amplitude "$4" >/dev/null 2>&1
  timeout "$CLI_TIMEOUT" ros2 param set "/$NAV_NAMESPACE/rotation_test_sender" period "$5" >/dev/null 2>&1
  timeout "$CLI_TIMEOUT" ros2 param set "/$NAV_NAMESPACE/rotation_test_sender" sine_wave "$6" >/dev/null 2>&1
  timeout "$CLI_TIMEOUT" ros2 service call "/$NAV_NAMESPACE/rotation_test_sender/enable" std_srvs/srv/Trigger >/dev/null 2>&1
  # 读回确认，避免 CLI 静默失败后继续按错误配置跑
  local mode
  mode="$(timeout "$CLI_TIMEOUT" ros2 param get "/$NAV_NAMESPACE/rotation_test_sender" rotation_mode 2>/dev/null | awk '{print $NF}')"
  [ "$mode" = "$1" ] || echo "[警告] 自转模式设置未生效: 期望 $1, 实际 ${mode:-<无响应>}" | tee -a "$RUN_LOG"
}

run_monitor() {  # $1 时长 $2 场景名   -> 结束后汇总写入 summary.txt
  # 额外留 10 s 余量，超时后强制结束，避免监视器卡住不放。
  timeout "$(( $1 + 10 ))" ros2 run srm27_nav_bringup srm_velocity_monitor.py --ros-args \
    -r __ns:="/$NAV_NAMESPACE" -p duration:="$1" \
    -p actual_topic:="${ACTUAL_TOPIC:-odometry}" \
    -p output:="$CASE_DIR/velocity.csv" \
    > "$CASE_DIR/summary.txt" 2>&1
}

# bag 必须在场景动作之前开始录、动作之后再停, 否则录不到测量窗口。
start_bag() {
  local topics=(
    "/clock"
    "/$NAV_NAMESPACE/cmd_vel_nav" "/$NAV_NAMESPACE/cmd_vel_sim"
    "/$NAV_NAMESPACE/rotation_cmd" "/$NAV_NAMESPACE/rotation_velocity"
    "/$NAV_NAMESPACE/odometry" "/$NAV_NAMESPACE/chassis_odometry_gt"
    "/$NAV_NAMESPACE/tf" "/$NAV_NAMESPACE/tf_static"
    "/$NAV_NAMESPACE/diagnostics" "/$NAV_NAMESPACE/livox/imu"
    "/$NAV_NAMESPACE/terrain_map" "/$NAV_NAMESPACE/terrain_map_ext"
  )
  # 点云体积大, 默认不录; RECORD_CLOUDS=1 时按方案 §8 的完整清单记录。
  if [ "$RECORD_CLOUDS" = "1" ]; then
    topics+=(
      "/$NAV_NAMESPACE/livox/lidar"
      "/$NAV_NAMESPACE/velodyne_points"
    )
  fi
  ros2 bag record -o "$CASE_DIR/bag" --compression-mode file \
    --compression-format zstd "${topics[@]}" \
    > "$CASE_DIR/bag_record.log" 2>&1 &
  BAG_PID=$!
  sleep 2
}

write_metadata() {  # $1 case  $2 world
  timeout "$CLI_TIMEOUT" ros2 run srm27_nav_bringup srm_regression_metadata.py \
    --output-dir "$CASE_DIR" \
    --world "$2" --map "$MAP_FILE" --params-file "$PARAMS_FILE" \
    --namespace "$NAV_NAMESPACE" --case "$1" \
    >> "$CASE_DIR/summary.txt" 2>&1
}

stop_bag() {
  if [ -n "${BAG_PID:-}" ]; then
    kill -INT "$BAG_PID" 2>/dev/null
    for _ in 1 2 3 4 5 6; do
      kill -0 "$BAG_PID" 2>/dev/null || break
      sleep 1
    done
    kill -KILL "$BAG_PID" 2>/dev/null
    # ros2 bag record 会再 fork 出 recorder；按命令行精确回收，避免写盘进程残留。
    pkill -9 -f "ros2 bag record -o $CASE_DIR/bag" 2>/dev/null
  fi
  BAG_PID=""
}

# ---------------------------------------------------------------- 空场 session
run_field_case() {  # $1 名称
  local case="$1"
  CASE_DIR="$OUT_ROOT/field_$case"
  mkdir -p "$CASE_DIR"
  echo "[field/$case] 开始" | tee -a "$RUN_LOG"

  set_rotation stop 0 0 0 4 true
  start_bag

  case "$case" in
    vx)
      ( publish_nav 0.3 0.0 9 ) &
      run_monitor 11 "$case"
      ;;
    vy)
      ( publish_nav 0.0 0.3 9 ) &
      run_monitor 11 "$case"
      ;;
    diagonal)
      ( publish_nav 0.5 0.5 9 ) &
      run_monitor 11 "$case"
      ;;
    wz_positive)
      set_rotation constant 1.0 0 0 4 true
      ( publish_nav 0.0 0.0 9 ) &
      run_monitor 11 "$case"
      ;;
    wz_negative)
      set_rotation constant -1.0 0 0 4 true
      ( publish_nav 0.0 0.0 9 ) &
      run_monitor 11 "$case"
      ;;
    overlay_constant)
      set_rotation constant 1.0 0 0 4 true
      ( publish_nav 0.25 0.0 9 ) &
      run_monitor 11 "$case"
      ;;
    overlay_periodic)
      set_rotation periodic 0.0 1.0 0.5 4.0 true
      ( publish_nav 0.25 0.0 13 ) &
      run_monitor 15 "$case"
      ;;
    timeout_zeroing)
      set_rotation constant 1.0 0 0 4 true
      # 先发 5 s, 然后停发, 观察平移超时清零(自转应继续)
      ( publish_nav 0.3 0.0 5 ) &
      run_monitor 11 "$case"
      ;;
    *) echo "[跳过] 未知 field 场景: $case" | tee -a "$RUN_LOG"; return 0 ;;
  esac
  set_rotation stop 0 0 0 4 true
  sleep 1
  stop_bag
  write_metadata "$case" "srm_empty"
  echo "[field/$case] 完成 -> $CASE_DIR" | tee -a "$RUN_LOG"
}

run_field_session() {
  # 空场没有导航栈，没有 odometry；用速度插件的真值里程计作为"实际运动"来源。
  ACTUAL_TOPIC="chassis_odometry_gt"
  CASE_DIR="$OUT_ROOT/_field_session"; mkdir -p "$CASE_DIR"
  start_sim "srm_empty"
  sleep 3
  setsid ros2 launch srm27_chassis_control srm_chassis_control.launch.py \
    namespace:="$NAV_NAMESPACE" use_sim_time:=True rotation_mode:=stop \
    > "$CASE_DIR/chassis.log" 2>&1 &
  NAV_PGID=$!
  sleep 35
  for case in vx vy diagonal wz_positive wz_negative overlay_constant overlay_periodic timeout_zeroing; do
    selected "$case" || continue
    [ "$(mem_avail_mb)" -lt "$MEM_FLOOR_MB" ] && { echo "[看门狗] 内存不足, 停止 field session" | tee -a "$RUN_LOG"; break; }
    run_field_case "$case"
  done
}

# ---------------------------------------------------------------- 场地 session
run_nav_case() {  # $1 名称 $2 目标x $3 目标y $4 自转配置
  local case="$1" gx="$2" gy="$3" rot="$4"
  CASE_DIR="$OUT_ROOT/nav_$case"
  mkdir -p "$CASE_DIR"
  echo "[nav/$case] 开始 (目标 map($gx, $gy), 自转 $rot)" | tee -a "$RUN_LOG"

  case "$rot" in
    none)     set_rotation stop 0 0 0 4 true ;;
    constant) set_rotation constant 1.0 0 0 4 true ;;
    periodic) set_rotation periodic 0.0 1.0 0.5 4.0 true ;;
  esac

  start_bag
  ( run_monitor 90 "$case" ) &
  local mon_bg=$!
  timeout 75 ros2 action send_goal "/$NAV_NAMESPACE/navigate_to_pose" \
    nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: map}, pose: {position: {x: $gx, y: $gy, z: 0.0}, orientation: {w: 1.0}}}}" \
    > "$CASE_DIR/goal.log" 2>&1
  wait $mon_bg 2>/dev/null

  set_rotation stop 0 0 0 4 true
  sleep 1
  stop_bag
  write_metadata "$case" "rmuc_2025"
  grep -E "Goal finished" "$CASE_DIR/goal.log" | tail -1 >> "$CASE_DIR/summary.txt" 2>/dev/null
  echo "[nav/$case] 完成 -> $CASE_DIR" | tee -a "$RUN_LOG"
}

run_nav_session() {
  ACTUAL_TOPIC="odometry"
  CASE_DIR="$OUT_ROOT/_nav_session"; mkdir -p "$CASE_DIR"
  start_sim "rmuc_2025"
  sleep 3
  setsid ros2 launch srm27_nav_bringup nav_srm_simulation_launch.py \
    namespace:="$NAV_NAMESPACE" world:=rmuc_2025 map:="$MAP_FILE" \
    params_file:="$PARAMS_FILE" slam:=False use_pcd_localization:=False \
    use_composition:=False use_sim_time:=True use_rviz:=False \
    start_rotation_sender:=True rotation_mode:=stop \
    > "$CASE_DIR/nav.log" 2>&1 &
  NAV_PGID=$!
  sleep 45
  # 目标点必须落在地图覆盖且可通行区域内: map 原点即机器人出生点。
  selected plain           && run_nav_case plain           1.5  1.5  none
  selected constant_rotation && run_nav_case constant_rotation 0.0 0.0 constant
  selected periodic_rotation && run_nav_case periodic_rotation 1.5 1.5 periodic
}

# ---------------------------------------------------------------- main
if [ ! -f "$WS/install/setup.bash" ]; then
  echo "[错误] 未找到 $WS/install/setup.bash" >&2; exit 1
fi
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set +u

mkdir -p "$OUT_ROOT"
RUN_LOG="$OUT_ROOT/run.log"
: > "$RUN_LOG"
echo "输出目录: $OUT_ROOT" | tee -a "$RUN_LOG"

check_leftovers || exit 1
echo "无残留进程, 开始。" | tee -a "$RUN_LOG"

case "$SESSION" in
  field) run_field_session ;;
  nav)   run_nav_session ;;
  all)   run_field_session; run_nav_session ;;
  *) echo "[错误] --session 只支持 field|nav|all" >&2; exit 2 ;;
esac

echo "全部完成, 结果在 $OUT_ROOT" | tee -a "$RUN_LOG"
exit 0
