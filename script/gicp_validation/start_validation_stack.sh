#!/usr/bin/env bash
# 启动验证用的「长期存活」仿真链路（方案 §5）：
#   Gazebo（软件渲染 ogre2）+ 点云转换 + Point-LIO + loam_interface + sensor_scan_generation
#
# 重要：DSH 的每次 bash 调用都是独立的 PID 命名空间，一次调用结束后其中的子进程
# 会随之结束。因此整场试验（起栈 → 建图 → 逐轮测试）必须放进**同一个后台作业**，
# 由 script/gicp_validation/run_session.sh 统一编排。
#
# 本脚本只负责「起栈并守候」，供 run_session.sh 或人工在后台作业里调用。
#
# 用法： bash script/gicp_validation/start_validation_stack.sh [--no-gazebo]
set -uo pipefail

WS=/home/srm/srm_nav_27
VD=$WS/tmp/gicp_validation
WORLD=$VD/worlds/rmuc_2025_ogre2.sdf
PARAMS=$VD/nav2_params_srm_test.yaml
START_GAZEBO=1
[ "${1:-}" = "--no-gazebo" ] && START_GAZEBO=0

mkdir -p "$VD/worlds" "$VD/logs" "$VD/results"

if [ ! -f "$WORLD" ]; then
  cp "$WS/install/srm27_gazebo_simulator/share/srm27_gazebo_simulator/resource/worlds/rmuc_2025_world.sdf" "$WORLD"
  sed -i 's|<render_engine>ogre</render_engine>|<render_engine>ogre2</render_engine>|' "$WORLD"
  echo "[stack] 生成软件渲染世界副本：$WORLD"
fi
if [ ! -f "$PARAMS" ]; then
  cp "$WS/src/srm27_navigation/srm27_nav_bringup/config/simulation/nav2_params_srm.yaml" "$PARAMS"
  echo "[stack] 生成参数副本：$PARAMS"
fi

set +u
source "$VD/env.sh"
set -u
export LIBGL_ALWAYS_SOFTWARE=1 MESA_LOADER_DRIVER_OVERRIDE=swrast GALLIUM_DRIVER=llvmpipe
cd "$WS"

cleanup() {
  echo "[stack] 收到退出信号，清理子进程"
  pkill -INT -f "srm_sim.launch.py" 2>/dev/null
  pkill -INT -f "ign gazebo" 2>/dev/null
  pkill -INT -f "validation_stack.launch.py" 2>/dev/null
  pkill -INT -f "pointlio_mapping" 2>/dev/null
  sleep 3
  exit 0
}
trap cleanup INT TERM

if [ "$START_GAZEBO" = "1" ]; then
  echo "[stack] 启动 Gazebo"
  ros2 launch srm27_gazebo_simulator srm_sim.launch.py \
    world_sdf:="$WORLD" gui:=false run_immediately:=true \
    > "$VD/logs/gz.log" 2>&1 &
  sleep 25
else
  echo "[stack] 跳过 Gazebo（--no-gazebo）"
fi

echo "[stack] 启动点云转换 + Point-LIO + TF 链路"
ros2 launch srm27_gicp_validation validation_stack.launch.py \
  namespace:=red_standard_robot1 \
  params_file:="$PARAMS" \
  use_sim_time:=True use_pcd_localization:=True use_lio_odometry:=False slam:=False \
  > "$VD/logs/perception.log" 2>&1 &

# 把本会话所有后代的 PID 写进文件，供 --stop / 预检精确清理。
# 只靠 pkill -f 在跨 PID 命名空间时会漏掉子进程（ros2 launch 的子进程会变成孤儿）。
(
  while true; do
    {
      pgrep -f "srm_sim.launch.py|validation_stack.launch.py|ign gazebo|pointlio_mapping|loam_interface|sensor_scan_generation|ign_sim_pointcloud_tool|ros_gz_bridge|robot_state_publisher|srm_velocity_adapter" 2>/dev/null || true
    } >> "$VD/logs/stack_pids.txt"
    sort -u "$VD/logs/stack_pids.txt" -o "$VD/logs/stack_pids.txt"
    sleep 3
  done
) &
PIDFILE_WRITER=$!

echo "[stack] 已启动，等待外部命令；日志见 $VD/logs/"
wait
