#!/usr/bin/env python3
"""验证会话的就绪检查与 TF 冲突检查（替代一串 ros2 CLI，快得多）。

用法::

    python3 gicp_session_check.py ready   [--timeout 180]
    python3 gicp_session_check.py tf      [--duration 3]
    python3 gicp_session_check.py pubs
"""

import argparse
import json
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage

NAMESPACE = "red_standard_robot1"


class Checker(Node):
    def __init__(self) -> None:
        super().__init__("gicp_session_check")
        self.gt_count = 0
        self.scan_count = 0
        self.scan_stamp = None
        self.gt_stamp = None
        self.tf_pairs = set()
        self.static_pairs = set()

        self.create_subscription(
            Odometry, f"/{NAMESPACE}/chassis_odometry_gt", self.on_gt, 20
        )
        self.create_subscription(
            PointCloud2,
            f"/{NAMESPACE}/registered_scan",
            self.on_scan,
            qos_profile_sensor_data,
        )
        qos = QoSProfile(
            depth=100,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        # TF 可能同时出现在全局 /tf 与 <ns>/tf 上（不同节点发布落点不同），
        # 两个都订阅，避免漏判。
        for topic in ("/tf", f"/{NAMESPACE}/tf"):
            self.create_subscription(TFMessage, topic, self.on_tf, 200)
        self.create_subscription(TFMessage, f"/{NAMESPACE}/tf_static", self.on_static, qos)
        self.create_subscription(TFMessage, "/tf_static", self.on_static, qos)

    def on_gt(self, msg: Odometry) -> None:
        self.gt_count += 1
        self.gt_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def on_scan(self, msg: PointCloud2) -> None:
        self.scan_count += 1
        self.scan_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def on_tf(self, msg: TFMessage) -> None:
        for transform in msg.transforms:
            self.tf_pairs.add((transform.header.frame_id, transform.child_frame_id))

    def on_static(self, msg: TFMessage) -> None:
        for transform in msg.transforms:
            self.static_pairs.add((transform.header.frame_id, transform.child_frame_id))

    def spin_for(self, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="验证会话就绪/一致性检查")
    parser.add_argument("command", choices=["ready", "tf", "pubs"])
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument("--duration", type=float, default=3.0)
    known, remaining = parser.parse_known_args(argv)

    rclpy.init(args=remaining)
    node = Checker()
    exit_code = 1
    try:
        if known.command == "ready":
            deadline = time.monotonic() + known.timeout
            while time.monotonic() < deadline:
                node.spin_for(1.0)
                if node.gt_count > 3 and node.scan_count > 3:
                    break
            result = {
                "gt_messages": node.gt_count,
                "scan_messages": node.scan_count,
                "gt_stamp": node.gt_stamp,
                "scan_stamp": node.scan_stamp,
                "ready": node.gt_count > 3 and node.scan_count > 3,
            }
            print(json.dumps(result, ensure_ascii=False))
            exit_code = 0 if result["ready"] else 1
        elif known.command == "tf":
            node.spin_for(known.duration)
            result = {
                "dynamic_pairs": sorted(f"{a}->{b}" for a, b in node.tf_pairs),
                "static_pairs": sorted(f"{a}->{b}" for a, b in node.static_pairs),
                # 方案 §3.2：map->odom 只有 GICP，odom->base_link 只有 sensor_scan_generation
                "has_map_odom": any(a == "map" and b == "odom" for a, b in node.tf_pairs),
                "has_odom_base": any(
                    a == "odom" and b == "base_link" for a, b in node.tf_pairs
                ),
            }
            print(json.dumps(result, ensure_ascii=False))
            exit_code = 0
        else:
            node.spin_for(1.0)
            print(json.dumps({"note": "pubs 由外层 ros2 topic info 提供"}, ensure_ascii=False))
            exit_code = 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
