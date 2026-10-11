#!/usr/bin/env bash
# 诊断 D：起栈后测量关键话题速率、仿真实时率与 TF 树，定位「数据很少/没有」的真实原因。
set -uo pipefail
WS=/home/srm/srm_nav_27
VD=$WS/tmp/gicp_validation
OUT=$VD/logs/diag_rates_$(date +%H%M%S).log
export ROS_DOMAIN_ID=194
set +u
source "$VD/env.sh"
set -u
cd "$WS"
{
  echo "=== $(date)"
  ros2 launch srm27_gazebo_simulator srm_sim.launch.py \
    world_sdf:="$VD/worlds/rmuc_2025_ogre2.sdf" gui:=false run_immediately:=true \
    > "$VD/logs/gz_diag_rates.log" 2>&1 &
  sleep 35
  ros2 launch srm27_gicp_validation validation_stack.launch.py \
    namespace:=red_standard_robot1 params_file:="$VD/nav2_params_srm_test.yaml" \
    use_sim_time:=True use_pcd_localization:=True use_lio_odometry:=False slam:=False \
    > "$VD/logs/perception_diag_rates.log" 2>&1 &
  echo "--- 等待 60 s 让 Point-LIO 稳定"
  sleep 60
  for t in livox/lidar aft_mapped_to_init cloud_registered lidar_odometry registered_scan; do
    printf '%s: ' "$t"
    timeout 8 ros2 topic hz "/red_standard_robot1/$t" 2>&1 | grep "average rate" | head -1
  done
  echo "--- 仿真实时率（sim 秒 / 墙钟秒）"
  python3 - <<'PY'
import rclpy, time
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
rclpy.init(); n = Node('rtf_probe')
st = {'t': None}
n.create_subscription(Clock, '/clock', lambda m: st.__setitem__('t', m.clock.sec + m.clock.nanosec*1e-9), 10)
t0 = time.monotonic(); s0 = None
while time.monotonic() - t0 < 4:
    rclpy.spin_once(n, timeout_sec=0.1)
    if s0 is None and st['t'] is not None: s0 = st['t']
print('sim 经过 %.3f s / 墙钟 %.3f s -> RTF=%.2f' % (st['t']-s0, time.monotonic()-t0, (st['t']-s0)/(time.monotonic()-t0)))
rclpy.shutdown()
PY
  echo "--- TF 树"
  timeout 20 "$WS/install/srm27_gicp_validation/bin/gicp_session_check" tf --duration 5 2>&1
  echo "--- odom->base_link 是否有数据"
  timeout 10 ros2 topic hz /red_standard_robot1/odometry 2>&1 | grep "average rate" | head -1
  echo "--- perception 日志中的告警"
  grep -E "WARN|ERROR" "$VD/logs/perception_diag_rates.log" | grep -v "jump back" | tail -8
  echo "--- 结束"
  pkill -INT -f "srm_sim.launch.py" 2>/dev/null
  pkill -INT -f "validation_stack.launch.py" 2>/dev/null
  pkill -INT -f "ign gazebo" 2>/dev/null
} > "$OUT" 2>&1
cat "$OUT"
