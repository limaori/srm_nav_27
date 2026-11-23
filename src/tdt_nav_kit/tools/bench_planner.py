#!/usr/bin/env python3
"""ROS 侧基准：连续调用规划动作 N 次，统计墙钟耗时。"""
import argparse
import math
import statistics
import time

import rclpy
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from rclpy.node import Node


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-n", type=int, default=50)
    ap.add_argument("--goal", nargs=2, type=float, default=[12.0, 3.0])
    ap.add_argument("--start", nargs=2, type=float, default=[3.4, 5.0])
    ap.add_argument("--ns", default="red_standard_robot1")
    a = ap.parse_args()

    rclpy.init()
    node = Node("bench_planner")
    client = ActionClient(node, ComputePathToPose, f"/{a.ns}/compute_path_to_pose")
    if not client.wait_for_server(timeout_sec=20.0):
        print("规划动作不可用")
        return 1

    times, sizes = [], []
    for _ in range(a.n):
        g = ComputePathToPose.Goal()
        g.start.header.frame_id = "map"
        g.start.pose.position.x, g.start.pose.position.y = a.start
        g.start.pose.orientation.w = 1.0
        g.goal.header.frame_id = "map"
        g.goal.pose.position.x, g.goal.pose.position.y = a.goal
        g.goal.pose.orientation.w = 1.0
        g.planner_id = "GridBased"

        t0 = time.time()
        send = client.send_goal_async(g)
        rclpy.spin_until_future_complete(node, send, timeout_sec=20.0)
        handle = send.result()
        if handle is None or not handle.accepted:
            print("目标被拒绝")
            return 1
        res = handle.get_result_async()
        rclpy.spin_until_future_complete(node, res, timeout_sec=20.0)
        times.append((time.time() - t0) * 1000.0)
        sizes.append(len(res.result().result.path.poses))

    times.sort()
    n = len(times)
    p95 = times[min(n - 1, int(math.ceil(0.95 * n)) - 1)]
    print(
        f"规划 x{n}：平均 {statistics.mean(times):.1f} ms，中位 {statistics.median(times):.1f} ms，"
        f"p95 {p95:.1f} ms，最大 {times[-1]:.1f} ms，路径点数 {min(sizes)}~{max(sizes)}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
