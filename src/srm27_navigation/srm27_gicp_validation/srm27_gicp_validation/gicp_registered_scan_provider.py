#!/usr/bin/env python3
"""**验证专用**的 ``registered_scan`` + ``odom → base_link`` 单一提供者。

## 为什么需要它

方案 §3.2 规定 ``registered_scan`` 由 ``loam_interface`` 给出、``odom → base_link``
由 ``sensor_scan_generation`` 给出。在本环境实测这两个上游节点都不可用：

* ``loam_interface`` 用带时间戳的 ``lookupTransform(base_link, front_mid360, 扫描时间)``
  做一次性初始化，在本环境下持续失败（``base_link ... does not exist``），于是
  ``registered_scan`` 一个都不发布，整条链路起不来；
* ``sensor_scan_generation`` 用 ``message_filters`` 的 **ExactTime** 策略同步
  ``lidar_odometry`` 与 ``registered_scan``，本环境下两者时间戳相差约 1 s，
  回调永不触发。

本节点用**同一份已确认的几何**自己完成这两件事，输入只有两个稳定话题：

* ``velodyne_points``（雷达系原始点，``ign_sim_pointcloud_tool`` 输出）
* ``aft_mapped_to_init``（Point-LIO 的 ``T_camera_init_body`` 里程计）

输出：

* ``registered_scan``：把雷达系点云按当前里程计位姿变换到 ``odom`` 系；
* ``<ns>/tf``：``odom → front_mid360`` 以及 ``odom → base_link``。

变换关系（``T_A_B`` 表示把 B 系点变换到 A 系）：

    T_O_L = T_C_B · T_B_L          （``T_B_L`` 是底盘到雷达的固定外参）
    T_O_B = T_O_L · T_L_B = T_O_L · inverse(T_B_L)

对平地仿真，底盘 roll/pitch 可忽略（方案 §1 本轮只评价 x/y/yaw）。
"""

import argparse
import os
import sys
import time

import numpy as np
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
    qos_profile_sensor_data,
)
from sensor_msgs.msg import PointCloud2, PointField
from tf2_msgs.msg import TFMessage

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from gicp_static_tf_republisher import rpy_to_matrix  # noqa: E402
from gicp_validation_common import (  # noqa: E402
    invert,
    make_transform,
    matrix_to_quaternion,
    quaternion_to_matrix,
)

# tf2 的 /tf 话题 QoS（Humble 的 rclpy 没有 tf2_ros.qos_profile_tf 常量）
TF_QOS = QoSProfile(
    depth=100,
    history=QoSHistoryPolicy.KEEP_LAST,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
)


class RegisteredScanProvider(Node):
    def __init__(self) -> None:
        super().__init__("gicp_registered_scan_provider")

        self.declare_parameter("input_cloud_topic", "velodyne_points")
        self.declare_parameter("input_odometry_topic", "aft_mapped_to_init")
        self.declare_parameter("output_cloud_topic", "registered_scan")
        self.declare_parameter("tf_topic", "/red_standard_robot1/tf")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("lidar_frame", "front_mid360")
        self.declare_parameter("base_frame", "base_link")
        # 与 srm27_sentry_geometry.yaml 的 lidar.xyz / lidar.rpy 一致
        self.declare_parameter("lidar_xyz", [0.15, -0.15, 0.22])
        self.declare_parameter("lidar_rpy", [-0.06981317007977318, 0.0, -1.5707963267948966])
        self.declare_parameter("publish_rate", 5.0)
        self.declare_parameter("max_points", 12000)
        self.declare_parameter("max_odom_age", 1.5)

        self.odom_frame = self.get_parameter("odom_frame").value
        self.lidar_frame = self.get_parameter("lidar_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.max_points = int(self.get_parameter("max_points").value)
        self.max_odom_age = float(self.get_parameter("max_odom_age").value)
        rate = float(self.get_parameter("publish_rate").value)
        xyz = [float(v) for v in self.get_parameter("lidar_xyz").value]
        rpy = [float(v) for v in self.get_parameter("lidar_rpy").value]

        self.t_base_lidar = make_transform(xyz, rpy_to_matrix(*rpy))
        self.t_lidar_base = invert(self.t_base_lidar)

        self.latest_odom = None
        self.latest_odom_stamp = None
        self.latest_cloud = None
        self.latest_cloud_stamp = None

        self.scan_count = 0
        self.skipped_no_odom = 0
        self.skipped_no_cloud = 0
        self.skipped_stale = 0

        self.create_subscription(
            PointCloud2,
            self.get_parameter("input_cloud_topic").value,
            self.on_cloud,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry, self.get_parameter("input_odometry_topic").value, self.on_odom, 50
        )
        self.cloud_publisher = self.create_publisher(
            PointCloud2, self.get_parameter("output_cloud_topic").value, qos_profile_sensor_data
        )
        # 用参数指定的话题名直接建发布者（rclpy 的 TransformBroadcaster 只发
        # 相对名 "tf"，会被节点命名空间改写，导致实际发到全局 /tf 而不是
        # <ns>/tf）。这里显式用参数里的绝对名，避免依赖命名空间语义。
        # /tf 的话题类型必须是 tf2_msgs/msg/TFMessage：往同名话题发单条
        # TransformStamped 会让话题出现两种类型，所有 tf2 订阅者都收不到。
        self.tf_publisher = self.create_publisher(
            TFMessage, self.get_parameter("tf_topic").value, TF_QOS
        )
        self.timer = self.create_timer(1.0 / rate, self.publish)
        self.report_timer = self.create_timer(10.0, self.report)

        self.get_logger().info(
            "[registered_scan] %s + %s → registered_scan(%s) 与 tf(%s→%s, %s→%s)，%.1f Hz"
            % (
                self.get_parameter("input_cloud_topic").value,
                self.get_parameter("input_odometry_topic").value,
                self.odom_frame,
                self.odom_frame,
                self.lidar_frame,
                self.odom_frame,
                self.base_frame,
                rate,
            )
        )

    # ------------------------------------------------------------------ 输入

    def on_odom(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        self.latest_odom = make_transform(
            [pose.position.x, pose.position.y, pose.position.z],
            quaternion_to_matrix(
                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
            ),
        )
        self.latest_odom_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def on_cloud(self, msg: PointCloud2) -> None:
        self.latest_cloud = msg
        self.latest_cloud_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    @staticmethod
    def read_xyz(msg: PointCloud2) -> np.ndarray:
        """按 point_step 做 stride 视图取 x/y/z（字段类型可能不一致）。"""
        field_map = {field.name: field for field in msg.fields}
        offsets = [field_map[name].offset for name in ("x", "y", "z")]
        point_step = msg.point_step
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        count = raw.size // point_step
        if count <= 0:
            return np.zeros((0, 3), dtype=np.float64)
        raw = raw[: count * point_step].reshape(count, point_step)
        columns = [
            raw[:, offset : offset + 4].copy().view(np.float32).reshape(-1).astype(np.float64)
            for offset in offsets
        ]
        points = np.stack(columns, axis=1)
        return points[np.isfinite(points).all(axis=1)]

    # ------------------------------------------------------------------ 输出

    def report(self) -> None:
        self.get_logger().info(
            "[registered_scan] 已发布 %d 帧；跳过：无里程计 %d / 无点云 %d / 数据过期 %d"
            % (self.scan_count, self.skipped_no_odom, self.skipped_no_cloud, self.skipped_stale)
        )

    def publish(self) -> None:
        if self.latest_odom is None:
            self.skipped_no_odom += 1
            return
        if self.latest_cloud is None:
            self.skipped_no_cloud += 1
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        if (
            self.max_odom_age > 0.0
            and (abs(now - self.latest_odom_stamp) > self.max_odom_age
                 or abs(now - self.latest_cloud_stamp) > self.max_odom_age)
        ):
            self.skipped_stale += 1
            if self.skipped_stale % 100 == 1:
                self.get_logger().warn(
                    "[registered_scan] 输入过期（odom %.3f s / cloud %.3f s），暂停发布"
                    % (abs(now - self.latest_odom_stamp), abs(now - self.latest_cloud_stamp))
                )
            return

        # T_O_L = T_C_B · T_B_L
        t_odom_lidar = self.latest_odom @ self.t_base_lidar

        points = self.read_xyz(self.latest_cloud)
        if points.shape[0] == 0:
            self.skipped_no_cloud += 1
            return
        if self.max_points > 0 and points.shape[0] > self.max_points:
            step = max(1, points.shape[0] // self.max_points)
            points = points[::step]

        homogeneous = np.hstack([points, np.ones((points.shape[0], 1), dtype=np.float64)])
        transformed = (t_odom_lidar @ homogeneous.T).T[:, :3].astype(np.float32)

        message = PointCloud2()
        # 用**当前仿真时间**：GICP 只把 header.stamp 当作「最新扫描时间」，
        # 并要求点云在 odom 系；用输入点云时间会让下游 TF 时间戳落后于时钟。
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.odom_frame
        message.height = 1
        message.width = int(transformed.shape[0])
        message.is_bigendian = False
        message.point_step = 12
        message.row_step = 12 * message.width
        message.is_dense = True
        message.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        message.data = np.ascontiguousarray(transformed).tobytes()
        self.cloud_publisher.publish(message)
        self.scan_count += 1

        stamp = message.header.stamp
        # odom → front_mid360（等价于 sensor_scan_generation 的 odom→base 语义链）
        self.send_tf(stamp, self.odom_frame, self.lidar_frame, t_odom_lidar)
        # odom → base_link
        self.send_tf(stamp, self.odom_frame, self.base_frame, t_odom_lidar @ self.t_lidar_base)

    def send_tf(self, stamp, parent: str, child: str, transform: np.ndarray) -> None:
        message = TransformStamped()
        message.header.stamp = stamp
        message.header.frame_id = parent
        message.child_frame_id = child
        message.transform.translation.x = float(transform[0, 3])
        message.transform.translation.y = float(transform[1, 3])
        message.transform.translation.z = float(transform[2, 3])
        x, y, z, w = matrix_to_quaternion(transform[:3, :3])
        message.transform.rotation.x = x
        message.transform.rotation.y = y
        message.transform.rotation.z = z
        message.transform.rotation.w = w
        # 注意：这里**不能**用 TransformBroadcaster.sendTransform（它发 TFMessage 到相对名
        # "tf"，本环境会被命名空间改写导致落点不确定）。改用普通 PointCloud 同款发布 +
        # 显式话题名。为了与 tf2 订阅者类型一致，仍然发 TFMessage 聚合消息。
        envelope = TFMessage()
        envelope.transforms.append(message)
        self.tf_publisher.publish(envelope)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="验证专用 registered_scan / odom-base TF 提供者")
    known, remaining = parser.parse_known_args(argv)
    del known
    rclpy.init(args=remaining)
    node = RegisteredScanProvider()
    started = time.monotonic()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            "[registered_scan] 退出：运行 %.1f s，发布 %d 帧"
            % (time.monotonic() - started, node.scan_count)
        )
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
