#!/usr/bin/env bash
set -u
cd /home/srm/srm_nav_27
export ROS_LOG_DIR=/home/srm/srm_nav_27/.dsh_run/roslog6
mkdir -p "$ROS_LOG_DIR"
set +u; source install/setup.bash; set -u
PKG=install/srm27_nav_bringup/share/srm27_nav_bringup
ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args \
  -r __node:=livox_ros_driver2 \
  --params-file "$PKG/config/real/nav2_params_srm_minco_fakeyaw.yaml" \
  -p user_config_path:=$PKG/config/real/mid360_user_config.json > .dsh_run/driver6.log 2>&1 &
DRV=$!
sleep 6
echo "=== gdb 里跑 zlio, 20 s 后中断看栈 ==="
timeout -s INT 20 gdb -q -batch \
  -ex "set pagination off" -ex "set confirm off" -ex "handle SIGINT stop nopass" \
  -ex run -ex "info threads" -ex "thread apply all bt 14" \
  --args ./install/frontend/lib/frontend/zlio_node --ros-args \
  -p config_file:=$PKG/config/real/zlio_mid360.yaml \
  -p input_type:=livox -p lidar_topic:=livox/lidar -p imu_topic:=livox/imu \
  -p imu_acceleration_scale:=9.81 -p body_frame:=livox_imu \
  -p publish_tf:=False -p world_frame:=odom 2>&1 | grep -vE "New Thread|Thread 0x|\[New LWP|libthread_db|Using host" | head -120
kill -INT $DRV 2>/dev/null; sleep 1; kill -9 $DRV 2>/dev/null
