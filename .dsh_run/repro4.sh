#!/usr/bin/env bash
set -u
cd /home/srm/srm_nav_27
export ROS_LOG_DIR=/home/srm/srm_nav_27/.dsh_run/roslog4
mkdir -p "$ROS_LOG_DIR"
set +u; source install/setup.bash; set -u
PKG=install/srm27_nav_bringup/share/srm27_nav_bringup
ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args \
  -r __node:=livox_ros_driver2 \
  --params-file "$PKG/config/real/nav2_params_srm_minco_fakeyaw.yaml" \
  -p user_config_path:=$PKG/config/real/mid360_user_config.json > .dsh_run/driver4.log 2>&1 &
DRV=$!
sleep 6
ros2 run frontend zlio_node --ros-args \
  -p config_file:=$PKG/config/real/zlio_mid360.yaml \
  -p input_type:=livox -p lidar_topic:=livox/lidar -p imu_topic:=livox/imu \
  -p imu_acceleration_scale:=9.81 -p body_frame:=livox_imu \
  -p publish_tf:=False -p world_frame:=odom > .dsh_run/zlio4.log 2>&1 &
ZL=$!
sleep 2
ZLNODE=$(pgrep -f "build/frontend/zlio_node" | head -1); echo "zlio node pid=$ZLNODE (launcher=$ZL)"; python3 .dsh_run/cpu_probe.py $ZLNODE 22
echo "=== zlio4.log ==="
cat .dsh_run/zlio4.log
kill -INT $ZL $DRV 2>/dev/null; sleep 2; kill -9 $ZL $DRV 2>/dev/null
