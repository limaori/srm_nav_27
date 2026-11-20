#!/usr/bin/env bash
set -u
cd /home/srm/srm_nav_27
export ROS_LOG_DIR=/home/srm/srm_nav_27/.dsh_run/roslog3
mkdir -p "$ROS_LOG_DIR"
set +u; source install/setup.bash; set -u
PKG=install/srm27_nav_bringup/share/srm27_nav_bringup
ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args \
  -r __node:=livox_ros_driver2 \
  --params-file "$PKG/config/real/nav2_params_srm_minco_fakeyaw.yaml" \
  -p user_config_path:=$PKG/config/real/mid360_user_config.json > .dsh_run/driver3.log 2>&1 &
DRV=$!
sleep 6
ros2 run frontend zlio_node --ros-args \
  -p config_file:=$PKG/config/real/zlio_mid360.yaml \
  -p input_type:=livox -p lidar_topic:=livox/lidar -p imu_topic:=livox/imu \
  -p imu_acceleration_scale:=9.81 -p body_frame:=livox_imu \
  -p publish_tf:=False -p world_frame:=odom > .dsh_run/zlio3.log 2>&1 &
ZL=$!
sleep 3
echo "=== 参数确认 ==="
ros2 param get /zlio maximum_buffered_points
ros2 param get /zlio maximum_queue_events
ros2 param get /zlio maximum_imu_samples
echo "=== 线程/CPU 采样 (每秒) ==="
prev_total=0
for i in $(seq 1 20); do
  sleep 1
  read -r total states < <(awk '{s[$2]=1; t+=$14+$15} END {for (k in s) printf "%s", k; printf " %.2f\n", t}' /proc/$ZL/task/*/stat 2>/dev/null | sort -u)
  d=$((total - prev_total)); prev_total=$total
  echo "[$i s] cpu_ticks+=$d 状态集合=$states"
  kill -0 $ZL 2>/dev/null || { echo "zlio 已退出"; break; }
done
echo "=== 线程名 ==="
for t in /proc/$ZL/task/*; do printf '%s %s\n' "$(basename $t)" "$(cat $t/comm 2>/dev/null)"; done
echo "=== zlio3.log ==="
cat .dsh_run/zlio3.log
kill -INT $ZL $DRV 2>/dev/null; sleep 2; kill -9 $ZL $DRV 2>/dev/null
