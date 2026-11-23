#!/usr/bin/env python3
# 2026-11-22 北走廊调试: 同时记录三级路径话题的完整消息,定位路径在哪一环被带偏。
# 用法: python3 capture_nav_paths.py <秒数> <输出目录>
# 每条消息存为 <目录>/<话题序号>_<n>.txt, 文件头带收到时间。
import rclpy, sys, os, time
from nav_msgs.msg import Path

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 25.0
OUT = sys.argv[2] if len(sys.argv) > 2 else '/tmp/navcap'
os.makedirs(OUT, exist_ok=True)

TOPICS = [('/plan', 'plan'), ('/plan_smoothed', 'smoothed'),
          ('/FollowPath/planning_input_path', 'input'), ('/local_plan', 'local')]

rclpy.init()
node = rclpy.create_node('nav_path_capture')
counts = {t: 0 for t, _ in TOPICS}
t0 = time.time()

def make_cb(topic, tag, idx):
    def cb(msg):
        counts[topic] += 1
        n = counts[topic]
        if n > 12:
            return
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        with open(f'{OUT}/{idx}_{tag}_{n:02d}.txt', 'w') as f:
            f.write(f't={time.time()-t0:.2f}s frame={msg.header.frame_id} n={len(pts)}\n')
            for x, y in pts:
                f.write(f'{x:.3f} {y:.3f}\n')
    return cb

for i, (topic, tag) in enumerate(TOPICS):
    node.create_subscription(Path, topic, make_cb(topic, tag, i), 10)

end = time.time() + DUR
while rclpy.ok() and time.time() < end:
    rclpy.spin_once(node, timeout_sec=0.2)
print('counts:', counts)
