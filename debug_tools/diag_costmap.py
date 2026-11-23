#!/usr/bin/env python3
"""一次性打印两张代价地图的实况：尺寸/原点/色值直方图 + 机器人周围 ASCII。

用法（栈起来之后）：
    source install/setup.bash
    python3 debug_tools/diag_costmap.py            # 默认 2.1 m 窗口
    python3 debug_tools/diag_costmap.py --half 10  # 窗口半径（格）

判读：
  * 直方图里没有 100/99，说明这一层里没有任何障碍（或静态层没加载上）；
  * 只有 0，说明整张图是空的 —— 对应 RViz 里"看不到这一层"；
  * -1 很多，说明未知占比大（track_unknown_space 打开且没观测到）。
"""
import argparse
import math
import sys

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener

TOPICS = ('/local_costmap/costmap', '/global_costmap/costmap')
BASE_FRAME = 'gimbal_yaw_fake'


def qos_latched():
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL,
                      history=HistoryPolicy.KEEP_LAST)


class Diag(Node):
    def __init__(self):
        super().__init__('diag_costmap')
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.maps = {}
        for t in TOPICS:
            self.create_subscription(OccupancyGrid, t, lambda m, topic=t: self.maps.__setitem__(topic, m), qos_latched())

    def wait_map(self, topic, timeout=20.0):
        import time
        end = time.time() + timeout
        while time.time() < end and topic not in self.maps:
            rclpy.spin_once(self, timeout_sec=0.2)
        return self.maps.get(topic)


def robot_pose(node):
    import time
    end = time.time() + 10.0
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.tf.can_transform('map', BASE_FRAME, rclpy.time.Time()):
            t = node.tf.lookup_transform('map', BASE_FRAME, rclpy.time.Time(), timeout=Duration(seconds=1.0))
            q = t.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            return (t.transform.translation.x, t.transform.translation.y, yaw)
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--half', type=int, default=21, help='ASCII 窗口半径（格）')
    args = ap.parse_args()

    rclpy.init()
    node = Diag()
    pose = robot_pose(node)
    if pose is None:
        print('TF map->%s 取不到（定位/TF 没起来？）' % BASE_FRAME)
    else:
        print('robot(map): x=%.3f y=%.3f yaw=%.1f deg' % (pose[0], pose[1], math.degrees(pose[2])))
    print()

    for topic in TOPICS:
        msg = node.wait_map(topic)
        if msg is None:
            print('=== %s === 没有收到消息（这一层没发布）' % topic)
            continue
        data = np.array(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
        res, ox, oy = msg.info.resolution, msg.info.origin.position.x, msg.info.origin.position.y
        print('=== %s ===' % topic)
        print('  frame=%s  %dx%d  res=%.3f  origin=(%.2f, %.2f)' % (
            msg.header.frame_id, msg.info.width, msg.info.height, res, ox, oy))
        vals, cnts = np.unique(data, return_counts=True)
        order = np.argsort(-cnts)[:8]
        tot = data.size
        print('  histogram: ' + ', '.join('%d:%.1f%%' % (int(vals[i]), 100.0 * cnts[i] / tot) for i in order))
        print('  free(0)=%.1f%%  lethal(100)=%.1f%%  inscribed(99)=%.1f%%  unknown(-1)=%.1f%%' % (
            100.0 * (data == 0).mean(), 100.0 * (data == 100).mean(),
            100.0 * (data == 99).mean(), 100.0 * (data == -1).mean()))
        if pose is not None:
            cs = int((pose[0] - ox) / res)
            rs = int((pose[1] - oy) / res)
            print('  window %dx%d cells (%.1f m), "#"=99/100  "+"=50..98  "."=free  "?"=unknown' % (
                args.half * 2 + 1, args.half * 2 + 1, (args.half * 2 + 1) * res))
            for r in range(rs + args.half, rs - args.half - 1, -1):
                line = ''
                for c in range(cs - args.half, cs + args.half + 1):
                    if not (0 <= r < data.shape[0] and 0 <= c < data.shape[1]):
                        line += ' '
                        continue
                    v = data[r, c]
                    line += '?' if v < 0 else ('#' if v >= 99 else ('+' if v >= 50 else '.'))
                print('   ' + line)
            print('  center cell = %d' % data[min(max(rs, 0), data.shape[0] - 1), min(max(cs, 0), data.shape[1] - 1)])
        print()

    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
