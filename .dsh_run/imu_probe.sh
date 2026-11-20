#!/usr/bin/env bash
set -u
cd /home/srm/srm_nav_27
export ROS_LOG_DIR=/home/srm/srm_nav_27/.dsh_run/roslog_imu
mkdir -p "$ROS_LOG_DIR"
set +u; source install/setup.bash; set -u
PKG=install/srm27_nav_bringup/share/srm27_nav_bringup
PARAMS=$PKG/config/real/nav2_params_srm_minco_fakeyaw.yaml
ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args \
  -r __node:=livox_ros_driver2 --params-file "$PARAMS" \
  -p user_config_path:=$PKG/config/real/mid360_user_config.json > .dsh_run/imu_driver.log 2>&1 &
DRV=$!
sleep 6
ros2 run srm27_localization_adapter imu_saturation_guard.py > .dsh_run/imu_guard.log 2>&1 &
GRD=$!
sleep 2
echo "=== 20 s 对比 ==="
python3 .dsh_run/imu_stamp_probe.py
echo "=== guard 日志 ==="
tail -5 .dsh_run/imu_guard.log
kill -INT $GRD $DRV 2>/dev/null; sleep 2; kill -9 $GRD $DRV 2>/dev/null
