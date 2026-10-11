#!/usr/bin/env bash
set -u
WS=/home/srm/srm_nav_27
export ROS_DOMAIN_ID=${GICP_DOMAIN_ID:-181}
set +u
source "$WS/tmp/gicp_validation/env.sh"
set -u
timeout 30 python3 - <<'PY'
import rclpy, time
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import PointCloud2
rclpy.init(); n = Node('scan_probe')
res={}
def mk(name):
    def cb(m):
        e=res.setdefault(name, {'n':0,'w':0})
        e['n']+=1; e['w']=m.width
    return cb
n.create_subscription(PointCloud2,'/red_standard_robot1/registered_scan',mk('sensor'),qos_profile_sensor_data)
rel=QoSProfile(depth=5, history=QoSHistoryPolicy.KEEP_LAST, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.VOLATILE)
n.create_subscription(PointCloud2,'/red_standard_robot1/registered_scan',mk('reliable'),rel)
t0=time.monotonic()
while time.monotonic()-t0<12: rclpy.spin_once(n,timeout_sec=0.1)
print('registered_scan probe:', res)
rclpy.shutdown()
PY
