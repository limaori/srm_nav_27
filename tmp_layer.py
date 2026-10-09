#!/usr/bin/env python3
"""分离"MINCO 规划层切弯"和"跟踪层跟不上":
   /FollowPath/planning_input_path = 截断后的全局路径(局部参考, 进 MINCO 之前)
   /FollowPath/minco_trajectory    = MINCO 输出的轨迹(MPC 要跟的东西)
   /plan                           = 全局路径
比较三者的最大/平均偏离, 就知道偏差是在哪一层产生的。
"""

import math
import time

import rclpy
from nav_msgs.msg import Path
from rclpy.node import Node

NS = "/red_standard_robot1"


class P(Node):
    def __init__(self):
        super().__init__("layer_probe")
        self.g = None
        self.inp = None
        self.minco = None
        self.create_subscription(Path, f"{NS}/plan", lambda m: setattr(self, "g", m), 10)
        self.create_subscription(Path, f"{NS}/FollowPath/planning_input_path",
                                 lambda m: setattr(self, "inp", m), 10)
        self.create_subscription(Path, f"{NS}/FollowPath/minco_trajectory",
                                 lambda m: setattr(self, "minco", m), 10)


def dist_to_polyline(poses, x, y):
    best = 1e9
    for a, b in zip(poses, poses[1:]):
        ax, ay = a.pose.position.x, a.pose.position.y
        bx, by = b.pose.position.x, b.pose.position.y
        dx, dy = bx - ax, by - ay
        seg2 = dx * dx + dy * dy
        if seg2 < 1e-9:
            continue
        t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / seg2))
        best = min(best, math.hypot(x - (ax + t * dx), y - (ay + t * dy)))
    return best


def deviation(a_poses, b_poses):
    """a 上每个点到 b 折线的距离, 返回 (mean, max)"""
    ds = [dist_to_polyline(b_poses, p.pose.position.x, p.pose.position.y) for p in a_poses]
    ds = [d for d in ds if d < 1e8]
    return (sum(ds) / len(ds), max(ds)) if ds else (float("nan"), float("nan"))


def main():
    rclpy.init()
    node = P()
    t0 = time.time()
    m1, m2 = [], []      # MINCO vs 局部参考; 车? 这里只做层间比较
    print(f"{'t':>5} | MINCO vs 局部参考(mean/max) | 局部参考 vs 全局(mean/max)")
    while time.time() - t0 < 40.0:
        rclpy.spin_once(node, timeout_sec=0.05)
        now = time.time() - t0
        if now < 1.0 or node.inp is None or node.minco is None:
            continue
        if (now - t0) % 1.0 > 0.3:
            continue
        a = deviation(node.minco.poses, node.inp.poses)
        b = deviation(node.inp.poses, node.g.poses) if node.g is not None else (float("nan"),) * 2
        m1.append(a)
        m2.append(b)
        print(f"{now:5.1f} | {a[0]:6.2f} / {a[1]:6.2f} m            | "
              f"{b[0]:6.2f} / {b[1]:6.2f} m", flush=True)
        time.sleep(0.7)
    if m1:
        print(f"\nMINCO 轨迹离'局部参考路径'平均 {sum(x[0] for x in m1)/len(m1):.2f} m, "
              f"最大 {max(x[1] for x in m1):.2f} m   ← 这一层就是 MINCO 自己的切弯量")
        print(f"局部参考离'全局路径'平均 {sum(x[0] for x in m2)/len(m2):.2f} m, "
              f"最大 {max(x[1] for x in m2):.2f} m   (应≈0, 它只是全局路径的截断)")
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
