#!/usr/bin/env bash
set -uo pipefail
WS=/home/srm/srm_nav_27
VD=$WS/tmp/gicp_validation
OUT=$VD/logs/diag_motion_$(date +%H%M%S).log
export ROS_DOMAIN_ID=183
set +u
source "$VD/env.sh"
set -u
cd "$WS"
{
  echo "=== $(date)"
  ros2 launch srm27_gazebo_simulator srm_sim.launch.py \
    world_sdf:="$VD/worlds/rmuc_2025_ogre2.sdf" gui:=false run_immediately:=true \
    > "$VD/logs/gz_motion.log" 2>&1 &
  sleep 45
  echo "--- 静止时的真值 / 点云范围"
  python3 - <<'PY'
import rclpy, time, numpy as np
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from rclpy.qos import qos_profile_sensor_data
rclpy.init(); n = Node('motion_probe')
state={'o':None,'c':None}
def ono(m):
    p=m.pose.pose; state['o']=(p.position.x,p.position.y,p.position.z,m.header.stamp.sec+m.header.stamp.nanosec*1e-9)
def onc(m):
    fm={f.name:f for f in m.fields}
    off=[fm[k].offset for k in ('x','y','z')]
    raw=np.frombuffer(m.data,dtype=np.uint8); ps=m.point_step
    cnt=raw.size//ps; raw=raw[:cnt*ps].reshape(cnt,ps)
    cols=[raw[:,o:o+4].copy().view(np.float32).reshape(-1).astype(np.float64) for o in off]
    pts=np.stack(cols,axis=1)
    state['c']=(pts.min(axis=0).round(2).tolist(), pts.max(axis=0).round(2).tolist(), m.header.frame_id, cnt)
n.create_subscription(Odometry,'/red_standard_robot1/chassis_odometry_gt',ono,20)
n.create_subscription(PointCloud2,'/red_standard_robot1/velodyne_points',onc,qos_profile_sensor_data)
t0=time.monotonic()
while time.monotonic()-t0<8: rclpy.spin_once(n,timeout_sec=0.1)
print('真值(世界系 x,y,z,t):', state['o'])
print('雷达系点云 min/max/frame/count:', state['c'])
rclpy.shutdown()
PY
  echo "--- 前进 4 s（cmd_vel_sim vx=0.5）"
  ros2 run srm27_gicp_validation gicp_cmd_vel_script --ros-args -p route:=forward > "$VD/logs/vel_motion.log" 2>&1 &
  sleep 6
  python3 - <<'PY'
import rclpy, time
from rclpy.node import Node
from nav_msgs.msg import Odometry
rclpy.init(); n = Node('motion_probe2')
last={}
def ono(m):
    p=m.pose.pose
    last['p']=(round(p.position.x,3), round(p.position.y,3), round(p.position.z,3))
n.create_subscription(Odometry,'/red_standard_robot1/chassis_odometry_gt',ono,20)
t0=time.monotonic()
while time.monotonic()-t0<6: rclpy.spin_once(n,timeout_sec=0.1)
print('前进中真值位置:', last.get('p'))
rclpy.shutdown()
PY
  echo "--- 结束"
  pkill -INT -f "srm_sim.launch.py" 2>/dev/null
  pkill -INT -f "gicp_cmd_vel_script" 2>/dev/null
  pkill -INT -f "ign gazebo" 2>/dev/null
} > "$OUT" 2>&1
cat "$OUT"
