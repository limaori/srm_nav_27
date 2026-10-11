#!/usr/bin/env bash
set -uo pipefail
WS=/home/srm/srm_nav_27
VD=$WS/tmp/gicp_validation
OUT=$VD/logs/diag_tftopic_$(date +%H%M%S).log
export ROS_DOMAIN_ID=192
set +u
source "$VD/env.sh"
set -u
cd "$WS"
{
  echo "=== $(date)"
  ros2 launch srm27_gazebo_simulator srm_sim.launch.py \
    world_sdf:="$VD/worlds/rmuc_2025_ogre2.sdf" gui:=false run_immediately:=true \
    > "$VD/logs/gz_diag_tftopic.log" 2>&1 &
  sleep 30
  "$WS/install/srm27_gicp_validation/bin/gicp_static_tf_republisher" \
    --ros-args -r __ns:=/red_standard_robot1 -p use_sim_time:=true \
    > "$VD/logs/tfrepub_diag.log" 2>&1 &
  sleep 12
  echo "--- /red_standard_robot1/tf 发布者"
  timeout 12 ros2 topic info /red_standard_robot1/tf -v 2>&1 | grep -E "Publisher count|Node name|Reliability|Durability" | head -12
  echo "--- /red_standard_robot1/tf 消息内容（前 2 条）"
  timeout 12 ros2 topic echo /red_standard_robot1/tf 2>&1 | head -24
  echo "--- 用 python 订阅 /red_standard_robot1/tf"
  python3 - <<'PY'
import rclpy, time
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
rclpy.init(); n = Node('tfdump')
pairs = set(); cnt = [0]
def cb(m):
    cnt[0] += 1
    for t in m.transforms:
        pairs.add((t.header.frame_id, t.child_frame_id))
n.create_subscription(TFMessage, '/red_standard_robot1/tf', cb, 100)
t0 = time.monotonic()
while time.monotonic() - t0 < 6:
    rclpy.spin_once(n, timeout_sec=0.1)
print('消息数', cnt[0], '帧对', sorted(pairs))
rclpy.shutdown()
PY
  echo "--- 结束"
  pkill -INT -f "srm_sim.launch.py" 2>/dev/null
  pkill -INT -f "gicp_static_tf_republisher" 2>/dev/null
  pkill -INT -f "ign gazebo" 2>/dev/null
} > "$OUT" 2>&1
cat "$OUT"
