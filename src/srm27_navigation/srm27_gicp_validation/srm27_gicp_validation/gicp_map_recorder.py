#!/usr/bin/env python3
"""先验地图采集：只做「按时间戳配对 + 落盘」，重活交给离线重建工具。

在 ROS 回调里做体素合并会把执行器阻塞到秒级，进而丢掉后续帧、把点云话题速率打到
1 Hz 以下（实测）。因此这里每帧只做：

1. 读雷达系原始点（stride 视图，不做多余拷贝）；
2. 找时间上最近的底盘真值 ``T_W_B``；
3. 存成 ``<session>/frames/NNNNNN.npz``（``points`` = 雷达系点，``odom`` = 4x4 真值）。

离线重建见 ``tools/rebuild_prior_map.py``；采集脚本见
``script/gicp_validation/run_prior_map_capture.sh``。

用法::

    ros2 run srm27_gicp_validation gicp_map_recorder --ros-args \\
        -p session_dir:=/tmp/gicp_validation/map_session \\
        -p capture_seconds:=60
"""

import argparse
import os
import sys
import time
from collections import deque

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from gicp_validation_common import make_transform, quaternion_to_matrix  # noqa: E402


class MapRecorder(Node):
    def __init__(self) -> None:
        super().__init__("gicp_map_recorder")

        self.declare_parameter("lidar_topic", "/red_standard_robot1/livox/lidar")
        self.declare_parameter("odom_topic", "/red_standard_robot1/chassis_odometry_gt")
        self.declare_parameter("session_dir", "/tmp/gicp_validation/map_session")
        self.declare_parameter("capture_seconds", 0)
        self.declare_parameter("start_delay", 4)
        self.declare_parameter("max_odom_gap", 0.08)
        self.declare_parameter("min_frame_points", 200)
        self.declare_parameter("point_stride", 4)

        self.session_dir = self.get_parameter("session_dir").value
        self.capture_seconds = int(self.get_parameter("capture_seconds").value)
        self.start_delay = int(self.get_parameter("start_delay").value)
        self.max_odom_gap = float(self.get_parameter("max_odom_gap").value)
        self.min_frame_points = int(self.get_parameter("min_frame_points").value)
        self.point_stride = max(1, int(self.get_parameter("point_stride").value))

        self.frames_dir = os.path.join(self.session_dir, "frames")
        os.makedirs(self.frames_dir, exist_ok=True)

        self.odom_samples = deque(maxlen=4000)
        self.frame_index = 0
        self.raw_points = 0
        self.dropped = 0
        self.lidar_callbacks = 0
        self.capturing = False
        self.deadline = None
        self.done = False

        self.create_subscription(
            PointCloud2,
            self.get_parameter("lidar_topic").value,
            self.on_cloud,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry, self.get_parameter("odom_topic").value, self.on_odom, 50
        )
        self.timer = self.create_timer(0.2, self.on_timer)

        self.get_logger().info(
            "[map_recorder] 会话目录 %s；%d s 后开始采集 %d s（point_stride=%d）"
            % (self.session_dir, self.start_delay, self.capture_seconds, self.point_stride)
        )

    def on_odom(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        transform = make_transform(
            [pose.position.x, pose.position.y, pose.position.z],
            quaternion_to_matrix(
                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
            ),
        )
        self.odom_samples.append((stamp, transform))

    def nearest_odom(self, stamp: float):
        best = None
        best_gap = None
        for sample_stamp, transform in reversed(self.odom_samples):
            gap = abs(sample_stamp - stamp)
            if best_gap is None or gap < best_gap:
                best, best_gap = transform, gap
            if sample_stamp < stamp - self.max_odom_gap:
                break
        if best is None or best_gap is None or best_gap > self.max_odom_gap:
            return None, best_gap
        return best, best_gap

    @staticmethod
    def read_xyz(msg: PointCloud2, stride: int) -> np.ndarray:
        field_map = {field.name: field for field in msg.fields}
        offsets = [field_map[name].offset for name in ("x", "y", "z")]
        point_step = msg.point_step
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        count = raw.size // point_step
        if count <= 0:
            return np.zeros((0, 3), dtype=np.float32)
        raw = raw[: count * point_step].reshape(count, point_step)[::stride]
        columns = [
            raw[:, offset : offset + 4].copy().view(np.float32).reshape(-1) for offset in offsets
        ]
        points = np.stack(columns, axis=1)
        return points[np.isfinite(points).all(axis=1)]

    def on_cloud(self, msg: PointCloud2) -> None:
        self.lidar_callbacks += 1
        if not self.capturing or self.done:
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        odom, gap = self.nearest_odom(stamp)
        if odom is None:
            self.dropped += 1
            return
        points = self.read_xyz(msg, self.point_stride)
        if points.shape[0] < self.min_frame_points:
            self.dropped += 1
            return
        path = os.path.join(self.frames_dir, f"{self.frame_index:06d}.npz")
        np.savez_compressed(
            path,
            points=points,
            odom=odom.astype(np.float64),
            stamp=np.float64(stamp),
            odom_gap=np.float64(gap if gap is not None else -1.0),
        )
        self.frame_index += 1
        self.raw_points += points.shape[0]

    def on_timer(self) -> None:
        if not self.capturing and not self.done:
            if time.monotonic() - self.node_start >= self.start_delay and self.odom_samples:
                self.capturing = True
                started = time.monotonic()
                self.deadline = started + self.capture_seconds if self.capture_seconds > 0 else None
                self.get_logger().info("[map_recorder] 开始采集")
            return
        if self.capturing and self.deadline is not None and time.monotonic() >= self.deadline:
            self.stop()

    node_start = 0.0

    def stop(self) -> None:
        self.capturing = False
        self.done = True
        self.get_logger().info(
            "[map_recorder] 采集结束：保存 %d 帧（%d 点），丢弃 %d 帧，雷达回调 %d"
            % (self.frame_index, self.raw_points, self.dropped, self.lidar_callbacks)
        )
        with open(os.path.join(self.session_dir, "recorder_summary.txt"), "w", encoding="utf-8") as stream:
            stream.write(f"frames={self.frame_index}\n")
            stream.write(f"points={self.raw_points}\n")
            stream.write(f"dropped={self.dropped}\n")
            stream.write(f"lidar_callbacks={self.lidar_callbacks}\n")
            stream.write(f"max_odom_gap={self.max_odom_gap}\n")
            stream.write(f"point_stride={self.point_stride}\n")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="先验地图采集（只落盘）")
    known, remaining = parser.parse_known_args(argv)
    del known
    rclpy.init(args=remaining)
    node = MapRecorder()
    node.node_start = time.monotonic()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        if not node.done:
            node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
