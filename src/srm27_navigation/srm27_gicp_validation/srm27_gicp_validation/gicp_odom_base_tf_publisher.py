#!/usr/bin/env python3
"""发布 ``odom → base_link``（方案 §3.2 里由 ``sensor_scan_generation`` 承担的角色）。

为什么单独写这个节点：``sensor_scan_generation`` 用 ``message_filters`` 的
**ExactTime** 策略同步 ``lidar_odometry`` 与 ``registered_scan``。在本环境实测
Point-LIO 的 ``aft_mapped_to_init`` 与 ``cloud_registered`` 时间戳相差约 1 s
（远超同步窗口），同步器永远不触发回调，于是 ``odom → base_link`` 一个都不发布，
GICP 拿不到 ``T_O_B``，初值换算也就无从进行。

这里改成「以 ``loam_interface`` 发布的 ``lidar_odometry`` 为唯一输入」：
``T_O_B = T_O_L · T_L_B``，其中 ``T_L_B = inverse(T_B_L)``，``T_B_L`` 与
``gicp_static_tf_republisher`` 用的是同一份几何外参。对平地仿真而言底盘
roll/pitch 可忽略（方案 §1 本轮只评价 x/y/yaw）。

用法::

    ros2 run srm27_gicp_validation gicp_odom_base_tf_publisher --ros-args \\
        -p input_odometry_topic:=lidar_odometry
"""

import argparse
import os
import sys

import rclpy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from gicp_validation_common import (  # noqa: E402
    invert,
    make_transform,
    matrix_to_quaternion,
    quaternion_to_matrix,
)
from gicp_static_tf_republisher import rpy_to_matrix  # noqa: E402


# tf2 默认的 TF 话题 QoS（Humble 的 rclpy 里没有 tf2_ros.qos_profile_tf，
# 这里显式写出来，保证与 C++ 侧 tf2_ros::DynamicBroadcaster 一致）。
TF_QOS = QoSProfile(
    depth=100,
    history=QoSHistoryPolicy.KEEP_LAST,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
)


class OdomBaseTfPublisher(Node):
    def __init__(self) -> None:
        super().__init__("gicp_odom_base_tf_publisher")

        self.declare_parameter("input_odometry_topic", "lidar_odometry")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        # 与 srm27_sentry_geometry.yaml 的 lidar.xyz / lidar.rpy 一致
        self.declare_parameter("lidar_xyz", [0.15, -0.15, 0.22])
        self.declare_parameter("lidar_rpy", [-0.06981317007977318, 0.0, -1.5707963267948966])
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("max_sample_age", 1.0)

        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        xyz = [float(v) for v in self.get_parameter("lidar_xyz").value]
        rpy = [float(v) for v in self.get_parameter("lidar_rpy").value]
        rate = float(self.get_parameter("publish_rate").value)
        self.max_sample_age = float(self.get_parameter("max_sample_age").value)

        # T_L_B = inverse(T_B_L)
        self.t_lidar_base = invert(make_transform(xyz, rpy_to_matrix(*rpy)))

        self.latest_odom = None
        self.latest_stamp = None
        self.latest_wall = None
        self.publish_count = 0
        self.stale_count = 0

        self.create_subscription(
            Odometry, self.get_parameter("input_odometry_topic").value, self.on_odom, 50
        )
        # 显式用**绝对**话题名建发布者：rclpy 的 TransformBroadcaster 用相对名 "tf"
        # 时会被节点命名空间改写成 <ns>/tf，而本工程其余节点（robot_state_publisher
        # 等）都在 <ns>/tf 上收发；用绝对名可以避免对 -r tf:=... 重映射的依赖。
        self.tf_publisher = self.create_publisher(
            TransformStamped, "/red_standard_robot1/tf", TF_QOS
        )
        self.timer = self.create_timer(1.0 / rate, self.publish)

        self.get_logger().info(
            "[odom_base_tf] 输入 %s → 发布 %s → %s（%.1f Hz，T_L_B 平移=%s）"
            % (
                self.get_parameter("input_odometry_topic").value,
                self.odom_frame,
                self.base_frame,
                rate,
                [round(float(v), 4) for v in self.t_lidar_base[:3, 3]],
            )
        )

    def on_odom(self, msg: Odometry) -> None:
        self.recv_count = getattr(self, "recv_count", 0) + 1
        if self.recv_count == 1:
            self.get_logger().info("[odom_base_tf] 收到第一条 %s" % msg.header.frame_id)
        pose = msg.pose.pose
        self.latest_odom = make_transform(
            [pose.position.x, pose.position.y, pose.position.z],
            quaternion_to_matrix(
                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
            ),
        )
        sim_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.latest_stamp = sim_time
        self.latest_wall = self.get_clock().now().nanoseconds * 1e-9

    def publish(self) -> None:
        if self.latest_odom is None or self.latest_stamp is None:
            self.wait_count = getattr(self, "wait_count", 0) + 1
            if self.wait_count % 100 == 1:
                self.get_logger().warn(
                    "[odom_base_tf] 尚未收到 %s 上的里程计，暂不发布（第 %d 次定时）"
                    % (self.get_parameter("input_odometry_topic").value, self.wait_count)
                )
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.max_sample_age > 0.0 and abs(now - self.latest_stamp) > self.max_sample_age:
            self.stale_count += 1
            if self.stale_count % 200 == 1:
                self.get_logger().warn(
                    "[odom_base_tf] 输入过期 %.3f s，暂停发布 %s → %s"
                    % (abs(now - self.latest_stamp), self.odom_frame, self.base_frame)
                )
            return

        transform = self.latest_odom @ self.t_lidar_base
        message = TransformStamped()
        # 用当前仿真时间发布，避免下游按扫描时间戳查询时外推
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.odom_frame
        message.child_frame_id = self.base_frame
        message.transform.translation.x = float(transform[0, 3])
        message.transform.translation.y = float(transform[1, 3])
        message.transform.translation.z = float(transform[2, 3])
        x, y, z, w = matrix_to_quaternion(transform[:3, :3])
        message.transform.rotation.x = x
        message.transform.rotation.y = y
        message.transform.rotation.z = z
        message.transform.rotation.w = w
        self.tf_publisher.publish(message)
        self.publish_count += 1
        if self.publish_count == 1:
            self.get_logger().info(
                "[odom_base_tf] 首条 %s → %s 已发布（stamp=%.3f，T_O_B 平移=[%.3f, %.3f, %.3f]）"
                % (
                    self.odom_frame,
                    self.base_frame,
                    message.header.stamp.sec + message.header.stamp.nanosec * 1e-9,
                    transform[0, 3],
                    transform[1, 3],
                    transform[2, 3],
                )
            )


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="发布 odom -> base_link")
    known, remaining = parser.parse_known_args(argv)
    del known
    rclpy.init(args=remaining)
    node = OdomBaseTfPublisher()
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
