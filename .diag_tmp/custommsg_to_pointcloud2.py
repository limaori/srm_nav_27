#!/usr/bin/env python3
"""把 /livox/lidar (livox_ros_driver2/CustomMsg) 转成 /livox/lidar/pointcloud (PointCloud2)。

为什么需要：实车建图时 2D 栅格默认吃驱动发的 `/livox/lidar/pointcloud`（~20k 点/帧），
回放时驱动不在，包里只有 CustomMsg，pointcloud_to_laserscan 就只能退到 LIO 输出的
`registered_scan`（0.5 m 体素后 ~1.7k 点/帧）——图会稀成一根根射线。这个节点把包里
那份**同样密**的点补回 PointCloud2，回放建图就能和实车同密度。

字段：x y z intensity (float32, point_step=16)，header 原样拷贝（stamp + frame_id）。
"""
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField

from livox_ros_driver2.msg import CustomMsg


class CustomMsgToPointCloud2(Node):
    def __init__(self):
        super().__init__('custommsg_to_pointcloud2')
        self.declare_parameter('in_topic', '/livox/lidar')
        self.declare_parameter('out_topic', '/livox/lidar/pointcloud')
        in_topic = self.get_parameter('in_topic').value
        out_topic = self.get_parameter('out_topic').value

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(PointCloud2, out_topic, qos)
        self.sub = self.create_subscription(CustomMsg, in_topic, self.on_msg, qos)
        self.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        self.frames = 0
        self.points = 0
        self.create_timer(30.0, self.report)
        self.get_logger().info(f'{in_topic} -> {out_topic} 已启动')

    def report(self):
        if self.frames:
            self.get_logger().info(
                f'已转发 {self.frames} 帧, 平均 {self.points / self.frames:.0f} 点/帧')

    def on_msg(self, msg):
        n = len(msg.points)
        if n == 0:
            return
        arr = np.empty((n, 4), dtype=np.float32)
        # 逐点读取（rclpy 已反序列化成对象列表），20k 点约 10~20 ms
        arr[:, 0] = [p.x for p in msg.points]
        arr[:, 1] = [p.y for p in msg.points]
        arr[:, 2] = [p.z for p in msg.points]
        arr[:, 3] = [float(p.reflectivity) for p in msg.points]

        cloud = PointCloud2()
        cloud.header = msg.header
        cloud.height = 1
        cloud.width = n
        cloud.fields = self.fields
        cloud.is_bigendian = False
        cloud.point_step = 16
        cloud.row_step = 16 * n
        cloud.is_dense = True
        cloud.data = arr.tobytes()
        self.pub.publish(cloud)

        self.frames += 1
        self.points += n


def main():
    rclpy.init()
    node = CustomMsgToPointCloud2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
