#!/usr/bin/env python3
"""把底盘到雷达的外参以外参 TF 的形式**周期性**发布（默认 10 Hz）。

为什么需要它：本工程仿真里 ``robot_state_publisher`` 只把
``base_link -> front_mid360`` 放在 ``/tf_static`` 上发布一次（transient-local）。
在本环境实测：**晚加入**的订阅者（``loam_interface`` / ``sensor_scan_generation`` /
``tf2_echo`` / 任何 ``TransformListener``）拿不到这条 latch 消息，于是它们的 TF
缓冲区里永远没有 ``base_link``，``registered_scan`` 一个都不发布，整条定位链路起不来。
（同一 QoS 的普通订阅者可以收到，问题出在 tf2 的静态帧缓存上。）

这个节点用与 URDF 完全相同的位姿，把该外参以带仿真时间戳的**动态** TF 周期发布，
从根上避免依赖 latch 语义。参数默认值来自
``srm27_robot_description/config/srm27_sentry_geometry.yaml`` 的 ``lidar`` 段。

用法::

    ros2 run srm27_gicp_validation gicp_static_tf_republisher --ros-args \\
        -p base_frame:=base_link -p lidar_frame:=front_mid360 \\
        -p xyz:=[0.15,-0.15,0.22] -p rpy:=[-0.06981317007977318,0.0,-1.5707963267948966]
"""

import argparse
import math
import sys

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from tf2_msgs.msg import TFMessage

sys.path.insert(0, __file__.rsplit("/", 1)[0])

from gicp_validation_common import matrix_to_quaternion, make_transform, rotation_z  # noqa: E402


def rpy_to_matrix(roll: float, pitch: float, yaw: float):
    """URDF 约定的 R = Rz(yaw) · Ry(pitch) · Rx(roll)。"""
    import numpy as np

    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=float)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=float)
    return rotation_z(yaw) @ ry @ rx


# tf2 的 /tf 话题 QoS（Humble 的 rclpy 没有 tf2_ros.qos_profile_tf 常量）
TF_QOS = QoSProfile(
    depth=100,
    history=QoSHistoryPolicy.KEEP_LAST,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
)


class StaticTfRepublisher(Node):
    def __init__(self) -> None:
        super().__init__("gicp_static_tf_republisher")

        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("lidar_frame", "front_mid360")
        # 默认值与 srm27_robot_description/config/srm27_sentry_geometry.yaml 的
        # lidar.xyz / lidar.rpy 一致。
        self.declare_parameter("xyz", [0.15, -0.15, 0.22])
        self.declare_parameter(
            "rpy", [-0.06981317007977318, 0.0, -1.5707963267948966]
        )
        self.declare_parameter("rate", 10.0)
        self.declare_parameter("tf_topic", "/tf")
        self.declare_parameter("namespace_frames", True)

        self.base_frame = self.get_parameter("base_frame").value
        self.lidar_frame = self.get_parameter("lidar_frame").value
        xyz = [float(v) for v in self.get_parameter("xyz").value]
        rpy = [float(v) for v in self.get_parameter("rpy").value]
        rate = float(self.get_parameter("rate").value)

        self.transform = make_transform(xyz, rpy_to_matrix(*rpy))
        # 用参数指定的**绝对**话题名建发布者，避免 rclpy TransformBroadcaster
        # 的 "tf" 相对名被命名空间改写后落点不确定。
        # 话题类型必须是 tf2_msgs/msg/TFMessage：往 /tf 发单条 TransformStamped
        # 会让话题出现两种类型，所有 tf2 订阅者都收不到。
        self.tf_publisher = self.create_publisher(
            TFMessage, self.get_parameter("tf_topic").value, TF_QOS
        )
        self.timer = self.create_timer(1.0 / rate, self.publish)
        self.count = 0

        self.get_logger().info(
            "[static_tf_republisher] %s -> %s xyz=%s rpy=%s 以 %.1f Hz 周期发布"
            % (self.base_frame, self.lidar_frame, xyz, rpy, rate)
        )

    def publish(self) -> None:
        message = TransformStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.base_frame
        message.child_frame_id = self.lidar_frame
        message.transform.translation.x = float(self.transform[0, 3])
        message.transform.translation.y = float(self.transform[1, 3])
        message.transform.translation.z = float(self.transform[2, 3])
        x, y, z, w = matrix_to_quaternion(self.transform[:3, :3])
        message.transform.rotation.x = x
        message.transform.rotation.y = y
        message.transform.rotation.z = z
        message.transform.rotation.w = w
        envelope = TFMessage()
        envelope.transforms.append(message)
        self.tf_publisher.publish(envelope)
        self.count += 1


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="周期发布底盘->雷达外参 TF")
    known, remaining = parser.parse_known_args(argv)
    del known
    rclpy.init(args=remaining)
    node = StaticTfRepublisher()
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
