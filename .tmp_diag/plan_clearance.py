#!/usr/bin/env python3
"""量测全局路径（/plan）相对 MINCO 硬门槛 0.38 m 的净空剖面。"""
import math
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


class Probe(Node):
    def __init__(self):
        super().__init__("plan_clearance")
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.gcost = None
        self.lcost = None
        self.plan = None
        self.plan_stamp = 0.0
        self.local = None
        self.local_count = 0
        self.create_subscription(OccupancyGrid, "/global_costmap/costmap", lambda m: setattr(self, "gcost", m), 1)
        self.create_subscription(OccupancyGrid, "/local_costmap/costmap", lambda m: setattr(self, "lcost", m), 1)
        self.create_subscription(Path, "/plan", self.on_plan, 1)
        self.create_subscription(Path, "/FollowPath/planning_input_path", self.on_local, 1)

    def on_plan(self, msg):
        self.plan = msg
        self.plan_stamp = time.time()

    def on_local(self, msg):
        self.local = msg
        self.local_count += 1


def occ_points(grid):
    info = grid.info
    data = np.asarray(grid.data, dtype=np.int16).reshape(info.height, info.width)
    occ = (data >= 65) | (data < 0)
    ys, xs = np.nonzero(occ)
    ox = info.origin.position.x + (xs + 0.5) * info.resolution
    oy = info.origin.position.y + (ys + 0.5) * info.resolution
    return np.stack([ox, oy], axis=1)


def main():
    rclpy.init()
    node = Probe()
    deadline = time.time() + 30.0
    done = False
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.plan is None or node.gcost is None:
            continue
        if done and (time.time() - node.plan_stamp) < 2.0:
            continue
        try:
            tf = node.tf.lookup_transform("map", "gimbal_yaw_fake", rclpy.time.Time(),
                                          timeout=rclpy.duration.Duration(seconds=1.0))
        except Exception:
            continue
        pose = np.array([tf.transform.translation.x, tf.transform.translation.y])
        pts = np.array([[p.pose.position.x, p.pose.position.y] for p in node.plan.poses])
        if len(pts) < 2:
            continue
        obs = occ_points(node.gcost)
        if obs.size == 0:
            continue
        d = np.array([float(np.linalg.norm(obs - p, axis=1).min()) for p in pts])
        seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        s = np.concatenate([[0.0], np.cumsum(seg)])
        i0 = int(np.argmin(np.linalg.norm(pts - pose, axis=1)))
        s_rel = s - s[i0]
        fwd = s_rel >= -0.05
        window = fwd & (s_rel <= 3.0)
        print(f"\n=== /plan #{len(pts)} pts, robot nearest idx={i0} "
              f"dist_to_path={np.linalg.norm(pts[i0]-pose):.2f} m, frame={node.plan.header.frame_id}")
        print(f"clearance at robot pose: {np.linalg.norm(obs-pose, axis=1).min():.3f} m")
        print(f"path clearance within 3 m ahead: min={d[window].min():.3f} m at "
              f"s={s_rel[window][int(np.argmin(d[window]))]:.2f} m")
        for j in np.where(window)[0][::6]:
            flag = "  <-- below 0.38" if d[j] < 0.38 else ""
            print(f"   s={s_rel[j]:5.2f} m  clr={d[j]:5.3f}{flag}")
        below = np.where(window & (d < 0.38))[0]
        if below.size:
            print(f"   first below 0.38 at s={s_rel[below[0]]:.2f} m")
        if node.local is not None and len(node.local.poses) >= 2:
            lp = np.array([[p.pose.position.x, p.pose.position.y] for p in node.local.poses])
            ll = float(np.sum(np.linalg.norm(np.diff(lp, axis=0), axis=1)))
            print(f"   local path: frame={node.local.header.frame_id} pts={len(lp)} len={ll:.2f} m "
                  f"(count={node.local_count})")
        done = True
        break
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
