#!/usr/bin/env python3
"""抓一次导航的「参考路径 vs 实际轨迹」，用于判断拐角是规划切弯还是控制过冲。

用法: python3 .diag_tmp/capture_corner.py <秒数> <输出目录>
产出: <目录>/odom.csv        t,x,y,yaw,vx,vy,wz
      <目录>/paths.jsonl     每帧路径（plan / local_plan / planning_input_path）
"""
import json
import math
import os
import sys
import time

import rclpy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist
from rclpy.node import Node

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 90.0
OUT = sys.argv[2] if len(sys.argv) > 2 else '.diag_tmp/navcap'
os.makedirs(OUT, exist_ok=True)

rclpy.init()
node = Node('corner_capture')
t0 = time.time()
odom_f = open(os.path.join(OUT, 'odom.csv'), 'w', encoding='utf-8')
odom_f.write('t,x,y,yaw,vx,vy,wz\n')
path_f = open(os.path.join(OUT, 'paths.jsonl'), 'w', encoding='utf-8')
counts = {}


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def on_odom(msg):
    p = msg.pose.pose
    v = msg.twist.twist
    odom_f.write(f'{time.time() - t0:.3f},{p.position.x:.4f},{p.position.y:.4f},'
                 f'{yaw_of(p.orientation):.4f},{v.linear.x:.4f},{v.linear.y:.4f},{v.angular.z:.4f}\n')


def make_path_cb(tag):
    def cb(msg):
        counts[tag] = counts.get(tag, 0) + 1
        pts = [[round(pp.pose.position.x, 3), round(pp.pose.position.y, 3)] for pp in msg.poses]
        path_f.write(json.dumps({'t': round(time.time() - t0, 3), 'topic': tag, 'n': len(pts),
                                 'pts': pts}, ensure_ascii=False) + '\n')
    return cb


node.create_subscription(Odometry, 'odometry', on_odom, 20)
for tag, topic in (('plan', 'plan'),
                   ('smoothed', 'plan_smoothed'),
                   ('input', 'FollowPath/planning_input_path'),
                   ('local', 'local_plan')):
    node.create_subscription(Path, topic, make_path_cb(tag), 5)

end = time.time() + DUR
while rclpy.ok() and time.time() < end:
    rclpy.spin_once(node, timeout_sec=0.1)

odom_f.close()
path_f.close()
print('counts:', counts, flush=True)
