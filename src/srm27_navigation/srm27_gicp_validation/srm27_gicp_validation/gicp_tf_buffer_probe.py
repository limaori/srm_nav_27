#!/usr/bin/env python3
"""TF 缓冲诊断：最小复现 loam_interface / tf2_echo 的 base_link 查询失败。

用法::

    python3 gicp_tf_buffer_probe.py --ros-args -p use_sim_time:=true
"""

import argparse
import time

import rclpy
import tf2_ros
from rclpy.node import Node


class Probe(Node):
    def __init__(self) -> None:
        super().__init__("gicp_tf_buffer_probe")
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self, spin_thread=False)
        self.create_timer(2.0, self.report)
        self.iteration = 0

    def report(self) -> None:
        self.iteration += 1
        frames = None
        try:
            frames = self.buffer.all_frames_as_string()
        except Exception as error:  # noqa: BLE001
            frames = f"<all_frames_as_string 失败：{error}>"
        print(f"--- 第 {self.iteration} 次检查（{time.strftime('%H:%M:%S')}）")
        print("all_frames_as_string:")
        print(frames if frames else "<空>")
        for target, source in (
            ("base_link", "front_mid360"),
            ("odom", "base_link"),
            ("base_link", "base_link"),
        ):
            try:
                transform = self.buffer.lookup_transform(target, source, rclpy.time.Time())
                print(
                    f"lookup {target} <- {source}: OK t=({transform.transform.translation.x:.3f}, "
                    f"{transform.transform.translation.y:.3f}, {transform.transform.translation.z:.3f})"
                )
            except Exception as error:  # noqa: BLE001
                print(f"lookup {target} <- {source}: 失败 {type(error).__name__}: {error}")
        print(f"clock now={self.get_clock().now().nanoseconds * 1e-9:.3f}")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="TF 缓冲区诊断")
    known, remaining = parser.parse_known_args(argv)
    del known
    rclpy.init(args=remaining)
    node = Probe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
