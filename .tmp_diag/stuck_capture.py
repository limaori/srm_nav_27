#!/usr/bin/env python3
"""10~20 s 实时抓取：控制器输出 / fake 变换 / 底盘出口 + 局部路径长度与净空。"""
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node


class Capture(Node):
    def __init__(self):
        super().__init__("stuck_capture")
        self.t0 = time.time()
        self.grid = None
        self.local = None
        self.plan = None
        self.local_count = 0
        self.plan_count = 0
        self.twists = {}
        for name in ("cmd_vel_nav_fake", "cmd_vel_nav_body", "cmd_vel_chassis"):
            self.create_subscription(
                Twist, "/" + name,
                lambda m, n=name: self.twists.__setitem__(n, (time.time() - self.t0, m)), 10)
        self.create_subscription(OccupancyGrid, "/local_costmap/costmap", self.on_grid, 1)
        self.create_subscription(Path, "/FollowPath/planning_input_path", self.on_local, 1)
        self.create_subscription(Path, "/plan", self.on_plan, 1)

    def on_grid(self, msg):
        self.grid = msg

    def on_local(self, msg):
        self.local = msg
        self.local_count += 1

    def on_plan(self, msg):
        self.plan = msg
        self.plan_count += 1


def clearance(points, grid):
    info = grid.info
    data = np.asarray(grid.data, dtype=np.int16).reshape(info.height, info.width)
    occ = (data >= 65) | (data < 0)
    ys, xs = np.nonzero(occ)
    if xs.size == 0:
        return np.full(len(points), np.inf)
    ox = info.origin.position.x + (xs + 0.5) * info.resolution
    oy = info.origin.position.y + (ys + 0.5) * info.resolution
    obs = np.stack([ox, oy], axis=1)
    return np.array([float(np.linalg.norm(obs - p, axis=1).min()) for p in points])


def main():
    rclpy.init()
    node = Capture()
    end = time.time() + 22.0
    last_print = 0.0
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
        now = time.time() - node.t0
        if now - last_print >= 0.5:
            last_print = now
            row = [f"t={now:5.1f}"]
            for name, short in (("cmd_vel_nav_fake", "fake"), ("cmd_vel_nav_body", "body"),
                                ("cmd_vel_chassis", "chas")):
                stamp, msg = node.twists.get(name, (-1.0, None))
                if msg is None:
                    row.append(f"{short}=--")
                else:
                    row.append(f"{short}=({msg.linear.x:+.3f},{msg.linear.y:+.3f})@{stamp:4.1f}")
            row.append(f"local#{node.local_count} plan#{node.plan_count}")
            if node.local is not None and node.grid is not None and node.local.header.frame_id == \
                    node.grid.header.frame_id:
                lp = np.array([[p.pose.position.x, p.pose.position.y] for p in node.local.poses])
                if len(lp) >= 2:
                    length = float(np.sum(np.linalg.norm(np.diff(lp, axis=0), axis=1)))
                    cl = clearance(lp, node.grid)
                    row.append(f"lp_pts={len(lp)} lp_len={length:.2f}m end_clr={cl[-1]:.2f} min={cl.min():.2f}")
            print(" ".join(row), flush=True)
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
