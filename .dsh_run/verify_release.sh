#!/usr/bin/env bash
set -u
cd /home/srm/srm_nav_27
export ROS_LOG_DIR=/home/srm/srm_nav_27/.dsh_run/roslog8
mkdir -p "$ROS_LOG_DIR"
set +u; source install/setup.bash; set -u
PKG=install/srm27_nav_bringup/share/srm27_nav_bringup
ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args \
  -r __node:=livox_ros_driver2 \
  --params-file "$PKG/config/real/nav2_params_srm_minco_fakeyaw.yaml" \
  -p user_config_path:=$PKG/config/real/mid360_user_config.json > .dsh_run/driver8.log 2>&1 &
DRV=$!
sleep 6
ros2 run frontend zlio_node --ros-args \
  -p config_file:=$PKG/config/real/zlio_mid360.yaml \
  -p input_type:=livox -p lidar_topic:=livox/lidar -p imu_topic:=livox/imu \
  -p imu_acceleration_scale:=9.81 -p body_frame:=livox_imu \
  -p publish_tf:=False -p world_frame:=odom > .dsh_run/zlio8.log 2>&1 &
sleep 1
echo "=== /odometry 频率 (15s) ==="
timeout 15 ros2 topic hz /odometry 2>&1 | tail -3
echo "=== /frontend/frame 频率 (6s) ==="
timeout 6 ros2 topic hz /frontend/frame 2>&1 | tail -3
echo "=== 再等 30 s 看队列是否还溢出 ==="
sleep 30
pgrep -x zlio_node >/dev/null && echo "zlio 存活: 是" || echo "zlio 存活: 否"
echo "=== zlio8.log ==="
cat .dsh_run/zlio8.log
pkill -INT -x zlio_node 2>/dev/null; kill -INT $DRV 2>/dev/null; sleep 2
pkill -9 -x zlio_node 2>/dev/null; kill -9 $DRV 2>/dev/null
