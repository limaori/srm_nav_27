#!/usr/bin/env bash
set -u
cd /home/srm/srm_nav_27
export ROS_LOG_DIR=/home/srm/srm_nav_27/.dsh_run/roslog2
mkdir -p "$ROS_LOG_DIR"
set +u; source install/setup.bash; set -u
PKG=install/srm27_nav_bringup/share/srm27_nav_bringup
ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args \
  -r __node:=livox_ros_driver2 \
  --params-file "$PKG/config/real/nav2_params_srm_minco_fakeyaw.yaml" \
  -p user_config_path:=$PKG/config/real/mid360_user_config.json > .dsh_run/driver2.log 2>&1 &
DRV=$!
sleep 6

python3 - > .dsh_run/points.log 2>&1 <<'PY' &
import rclpy, time
from rclpy.node import Node
from livox_ros_driver2.msg import CustomMsg
rclpy.init()
n = Node("pt_probe")
last = {}
def cb(msg):
    last["n"] = msg.point_num
    last["t"] = time.time()
n.create_subscription(CustomMsg, "livox/lidar", cb, 10)
end = time.time() + 40
while time.time() < end:
    rclpy.spin_once(n, timeout_sec=0.5)
    if "n" in last:
        print("CustomMsg point_num =", last["n"], flush=True)
        break
PY
PROBE=$!

ros2 run frontend zlio_node --ros-args \
  -p config_file:=$PKG/config/real/zlio_mid360.yaml \
  -p input_type:=livox -p lidar_topic:=livox/lidar -p imu_topic:=livox/imu \
  -p imu_acceleration_scale:=9.81 -p body_frame:=livox_imu \
  -p publish_tf:=False -p world_frame:=odom --log-level debug > .dsh_run/zlio2.log 2>&1 &
ZL=$!
for i in $(seq 1 8); do
  sleep 4
  cpu=$(ps -o %cpu=,etime= -p $ZL 2>/dev/null | tr -s ' ')
  echo "[t=$((i*4))s] zlio pid=$ZL cpu/etime=$cpu" | tee -a .dsh_run/monitor.log
done
echo "=== 点数 ==="; cat .dsh_run/points.log
echo "=== zlio 日志 (前 40 行) ==="; head -40 .dsh_run/zlio2.log
echo "=== zlio 日志 (后 20 行) ==="; tail -20 .dsh_run/zlio2.log
kill -INT $ZL $DRV $PROBE 2>/dev/null; sleep 2; kill -9 $ZL $DRV $PROBE 2>/dev/null
