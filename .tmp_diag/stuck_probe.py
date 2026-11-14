#!/usr/bin/env python3
"""一次性诊断：量测机器人当前位置/全局路径/局部路径的净空与长度。

只读：订阅话题、查 TF，不发布任何东西。
"""
import math
import sys
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


class Probe(Node):
    def __init__(self):
        super().__init__("stuck_probe")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.grid = None
        self.plan = None
        self.local_plan = None
        self.odom = None
        self.create_subscription(OccupancyGrid, "/local_costmap/costmap", self.on_grid, 1)
        self.create_subscription(Path, "/plan", self.on_plan, 1)
        self.create_subscription(Path, "/FollowPath/planning_input_path", self.on_local, 1)
        self.create_subscription(Odometry, "/odometry", self.on_odom, 10)

    def on_grid(self, msg):
        self.grid = msg

    def on_plan(self, msg):
        self.plan = msg

    def on_local(self, msg):
        self.local_plan = msg

    def on_odom(self, msg):
        self.odom = msg


def dist_to_obstacles(points_xy, grid, occ_thresh, unknown_is_obstacle):
    info = grid.info
    w, h = info.width, info.height
    data = np.asarray(grid.data, dtype=np.int16).reshape(h, w)
    occupied = data >= occ_thresh
    if unknown_is_obstacle:
        occupied = occupied | (data < 0)
    ys, xs = np.nonzero(occupied)
    if xs.size == 0:
        return np.full(len(points_xy), np.inf)
    ox = info.origin.position.x + (xs + 0.5) * info.resolution
    oy = info.origin.position.y + (ys + 0.5) * info.resolution
    obstacles = np.stack([ox, oy], axis=1)
    out = np.empty(len(points_xy))
    for i, p in enumerate(points_xy):
        d = np.linalg.norm(obstacles - p, axis=1)
        out[i] = float(d.min())
    return out


def main():
    rclpy.init()
    node = Probe()
    deadline = time.time() + 15.0
    while time.time() < deadline and not (node.grid and node.plan and node.odom):
        rclpy.spin_once(node, timeout_sec=0.2)
    if not node.grid or not node.plan:
        print("FAILED to receive costmap/plan")
        return 1

    grid = node.grid
    print(f"local_costmap: frame={grid.header.frame_id} {grid.info.width}x{grid.info.height} "
          f"res={grid.info.resolution} origin=({grid.info.origin.position.x:.2f},"
          f"{grid.info.origin.position.y:.2f})")

    # 机器人位姿：优先 map 系（TF odom->map 复合）
    pose_xy = None
    frame_note = ""
    try:
        tf = node.tf_buffer.lookup_transform(
            grid.header.frame_id, "gimbal_yaw", rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=2.0))
        pose_xy = np.array([tf.transform.translation.x, tf.transform.translation.y])
        frame_note = f"TF {grid.header.frame_id} <- gimbal_yaw"
    except Exception as exc:  # noqa: BLE001
        pose_xy = np.array([node.odom.pose.pose.position.x, node.odom.pose.pose.position.y])
        frame_note = f"odom only ({exc.__class__.__name__})"
    print(f"robot pose in {grid.header.frame_id}: ({pose_xy[0]:.3f}, {pose_xy[1]:.3f})  [{frame_note}]")

    # 全局路径投到 costmap 系
    plan_pts = np.array([[p.pose.position.x, p.pose.position.y] for p in node.plan.poses])
    print(f"/plan: frame={node.plan.header.frame_id} points={len(plan_pts)}")
    if node.plan.header.frame_id != grid.header.frame_id:
        try:
            tf = node.tf_buffer.lookup_transform(
                grid.header.frame_id, node.plan.header.frame_id, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0))
            t = tf.transform.translation
            q = tf.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z), 1 - 2 * q.z * q.z)
            c, s = math.cos(yaw), math.sin(yaw)
            plan_pts = np.stack([
                c * plan_pts[:, 0] - s * plan_pts[:, 1] + t.x,
                s * plan_pts[:, 0] + c * plan_pts[:, 1] + t.y,
            ], axis=1)
            print(f"  transformed {node.plan.header.frame_id}->{grid.header.frame_id} "
                  f"(yaw={math.degrees(yaw):.1f} deg)")
        except Exception as exc:  # noqa: BLE001
            print(f"  TF plan frame failed: {exc}")
            return 2

    d_plan = dist_to_obstacles(plan_pts, grid, 65, True)
    d_robot = dist_to_obstacles(pose_xy[None, :], grid, 65, True)[0]
    print(f"clearance at robot: {d_robot:.3f} m  (hard floor = robot_radius 0.33 + margin 0.05 = 0.38)")

    # 沿路径累计弧长
    seg = np.linalg.norm(np.diff(plan_pts, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    # 找最近点
    i0 = int(np.argmin(np.linalg.norm(plan_pts - pose_xy, axis=1)))
    print(f"nearest plan index={i0} s={s[i0]:.3f} m, clearance there={d_plan[i0]:.3f} m")
    print("\n  s[m]   clearance[m]   (from robot along global path)")
    printed = 0
    for i in range(i0, len(plan_pts)):
        ds = s[i] - s[i0]
        if ds > 3.2:
            break
        if printed % 4 == 0 or d_plan[i] < 0.38:
            print(f"  {ds:5.2f}   {d_plan[i]:8.3f}{'   <-- BELOW 0.38' if d_plan[i] < 0.38 else ''}")
        printed += 1
        if d_plan[i] < 0.38 and ds > 0.05:
            print(f"\n  first crossing below 0.38 m at s = {ds:.3f} m from robot")
            break

    # 局部路径
    if node.local_plan is not None and len(node.local_plan.poses) >= 2:
        lp = np.array([[p.pose.position.x, p.pose.position.y] for p in node.local_plan.poses])
        if node.local_plan.header.frame_id != grid.header.frame_id:
            try:
                tf = node.tf_buffer.lookup_transform(
                    grid.header.frame_id, node.local_plan.header.frame_id, rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=1.0))
                t = tf.transform.translation
                q = tf.transform.rotation
                yaw = math.atan2(2 * (q.w * q.z), 1 - 2 * q.z * q.z)
                c, s_ = math.cos(yaw), math.sin(yaw)
                lp = np.stack([c * lp[:, 0] - s_ * lp[:, 1] + t.x,
                               s_ * lp[:, 0] + c * lp[:, 1] + t.y], axis=1)
            except Exception:  # noqa: BLE001
                pass
        length = float(np.sum(np.linalg.norm(np.diff(lp, axis=0), axis=1)))
        d_lp = dist_to_obstacles(lp, grid, 65, True)
        print(f"\n/FollowPath/planning_input_path: points={len(lp)} length={length:.3f} m "
              f"end_clearance={d_lp[-1]:.3f} m min_clearance={d_lp.min():.3f} m")
    else:
        print("\n/FollowPath/planning_input_path: no valid sample")

    if node.odom is not None:
        v = node.odom.twist.twist.linear
        print(f"odom twist: vx={v.x:.3f} vy={v.y:.3f} |v|={math.hypot(v.x, v.y):.3f} m/s")
    return 0


if __name__ == "__main__":
    sys.exit(main())
