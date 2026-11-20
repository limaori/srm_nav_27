#!/usr/bin/env python3
"""对比 livox/imu 与 livox/imu_saturated 的 header.stamp 是否单调。"""
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu

class Probe(Node):
    def __init__(self):
        super().__init__("imu_stamp_probe")
        qos = QoSProfile(depth=1000, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.state = {}
        for topic in ("livox/imu", "livox/imu_saturated"):
            self.state[topic] = {"last": None, "n": 0, "back": 0, "dups": 0, "gaps": 0, "examples": []}
            self.create_subscription(Imu, topic, self.make_cb(topic), qos)

    def make_cb(self, topic):
        st = self.state[topic]
        def cb(msg):
            t = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
            st["n"] += 1
            if st["last"] is not None:
                d = t - st["last"]
                if d < 0:
                    st["back"] += 1
                    if len(st["examples"]) < 5:
                        st["examples"].append((st["last"], t, d))
                elif d == 0:
                    st["dups"] += 1
                elif d > 10_000_000:
                    st["gaps"] += 1
            st["last"] = t
        return cb

def main():
    rclpy.init()
    node = Probe()
    end = node.get_clock().now().nanoseconds + 20 * 10**9
    while node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    for topic, st in node.state.items():
        print(f"{topic:24s} 收到 {st['n']:6d} 帧, 时间回退 {st['back']}, 重复 {st['dups']}, 间隔>10ms {st['gaps']}")
        for last, t, d in st["examples"]:
            print(f"    回退示例: {last} -> {t}  (Δ={d/1e6:.3f} ms)")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    sys.exit(main())
