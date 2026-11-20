#!/usr/bin/env bash
set -u
cd /home/srm/srm_nav_27
set +u; source install/setup.bash; set -u
export ROS_LOG_DIR=/home/srm/srm_nav_27/.dsh_run/roslog
mkdir -p "$ROS_LOG_DIR"
PKG=install/srm27_nav_bringup/share/srm27_nav_bringup
echo "=== 起 livox 驱动 ==="
ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args \
  -r __node:=livox_ros_driver2 \
  --params-file "$PKG/config/real/nav2_params_srm_minco_fakeyaw.yaml" \
  -p user_config_path:=$PKG/config/real/mid360_user_config.json > .dsh_run/driver.log 2>&1 &
DRV=$!
sleep 6
echo "=== 话题列表 ==="
ros2 topic list | grep -i livox
echo "=== livox/lidar 频率 ==="
timeout 8 ros2 topic hz livox/lidar 2>&1 | tail -4
echo "=== 起 zlio ==="
ros2 run frontend zlio_node --ros-args \
  -p config_file:=$PKG/config/real/zlio_mid360.yaml \
  -p input_type:=livox -p lidar_topic:=livox/lidar -p imu_topic:=livox/imu \
  -p imu_acceleration_scale:=9.81 -p body_frame:=livox_imu \
  -p publish_tf:=False -p world_frame:=odom --log-level info > .dsh_run/zlio.log 2>&1 &
ZL=$!
for i in $(seq 1 9); do
  sleep 5
  alive=$(kill -0 $ZL 2>/dev/null && echo yes || echo no)
  odom=$(timeout 3 ros2 topic hz /zlio/odometry 2>&1 | tail -4 | tr '\n' ' ')
  echo "[t=$((i*5))s] zlio存活=$alive  odom: $odom"
done
echo "=== zlio.log ==="
cat .dsh_run/zlio.log
echo "=== driver.log tail ==="
tail -5 .dsh_run/driver.log
kill -INT $ZL $DRV 2>/dev/null; sleep 2; kill -9 $ZL $DRV 2>/dev/null
