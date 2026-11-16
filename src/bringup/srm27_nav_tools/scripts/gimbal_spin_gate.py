#!/usr/bin/env python3
# Copyright 2026 SRM
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""LIO 模式下的云台自转门控：等 Point-LIO 初始化完成后，才开始让云台转。

为什么需要它
============
Point-LIO 把"雷达 + IMU 所在的那块刚体"当作自己的机体系，而这两者都装在云台上。
所以在 LIO 模式下云台**可以**转 —— 前提是：

  1. 云台系就是 LIO 的机体系：本工程已经是这样（`loam_interface` 的
     `base_frame: gimbal_yaw`，把 LIO 位姿当作云台位姿）；
  2. 车体位姿靠 TF 换算：`sensor_scan_generation` 每帧实时查
     `front_mid360 -> base_link`（这条链会随云台关节变化），再发 `odom -> base_link`；
  3. **LIO 初始化期间云台必须静止**：Point-LIO 初始化要求传感器静止若干秒来估
     gravity/bias，2 rad/s 的自转会让它一开始就发散 —— 实测现象是
     "RViz 里车直接飞出去，Gazebo 里车原地不动"。

本节点负责第 3 条：
  * LIO 还没开始时，持续发 0（把云台锁住，不会自由漂）；
  * 收到足够多的 LIO 里程计、并稳定运行 `settle_sec` 之后，才开始按 `spin_rate` 发；
  * 里程计一旦中断（>`odom_timeout`），立刻回 0，避免"没有定位还在转"。

命令怎么到 Gazebo
=================
`/<ns>/cmd_gimbal_vel` (std_msgs/Float64, rad/s) --ros_gz_bridge--> gz
`<model>/joint/gimbal_yaw_joint/0/cmd_vel` --> gz `JointController` --> 云台关节。
对应 SDF 里 `gimbal_driver=external`（此时自有速度插件不碰云台关节）。

与真值里程计模式的关系
======================
真值模式（不加 `--lio`）不需要本节点：云台转速由 SDF 里的 `gimbal_spin_rate`
直接交给 SrmVelocitySystem，起来就匀速转。本节点只在 LIO 模式下启动。
"""

import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from nav_msgs.msg import Odometry
from std_msgs.msg import Float64


class GimbalSpinGate(Node):
    """等 LIO 稳定之后再让云台转。"""

    def __init__(self):
        super().__init__("gimbal_spin_gate")

        self.declare_parameter("spin_rate", 2.0)
        self.declare_parameter("odometry_topic", "lidar_odometry")
        self.declare_parameter("command_topic", "cmd_gimbal_vel")
        self.declare_parameter("settle_sec", 2.0)
        self.declare_parameter("min_messages", 10)
        self.declare_parameter("odom_timeout", 1.0)
        self.declare_parameter("publish_hz", 10.0)

        self.spin_rate = float(self.get_parameter("spin_rate").value)
        odom_topic = str(self.get_parameter("odometry_topic").value)
        command_topic = str(self.get_parameter("command_topic").value)
        self.settle_sec = float(self.get_parameter("settle_sec").value)
        self.min_messages = int(self.get_parameter("min_messages").value)
        self.odom_timeout = float(self.get_parameter("odom_timeout").value)
        publish_hz = float(self.get_parameter("publish_hz").value)
        if not (publish_hz > 0.0):
            publish_hz = 10.0

        self.messages = 0
        self.first_stamp = None
        self.last_recv = None
        self.spinning = False

        self.publisher = self.create_publisher(Float64, command_topic, 10)
        self.subscription = self.create_subscription(
            Odometry, odom_topic, self._on_odometry, qos_profile_sensor_data
        )
        self.timer = self.create_timer(1.0 / publish_hz, self._on_timer)

        self.get_logger().info(
            f"云台自转门控已启动: 等 {odom_topic} 至少 {self.min_messages} 帧"
            f"且稳定 {self.settle_sec:.1f} s 后, 按 {self.spin_rate:.2f} rad/s 发 "
            f"{command_topic}；此前持续发 0（云台锁定）"
        )

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_odometry(self, msg: Odometry):
        del msg
        self.messages += 1
        now = self._now()
        if self.first_stamp is None:
            self.first_stamp = now
        self.last_recv = now

    def _should_spin(self):
        if self.messages < self.min_messages:
            return False
        if self.first_stamp is None:
            return False
        if self._now() - self.first_stamp < self.settle_sec:
            return False
        # 里程计还在来：否则说明 LIO 挂了/停了，不能转。
        if self.last_recv is None or self._now() - self.last_recv > self.odom_timeout:
            return False
        return True

    def _publish(self, rate):
        msg = Float64()
        msg.data = float(rate)
        self.publisher.publish(msg)

    def _on_timer(self):
        want = self._should_spin()
        if want != self.spinning:
            self.spinning = want
            if want:
                self.get_logger().info(
                    f"LIO 已稳定（{self.messages} 帧里程计），云台开始以 "
                    f"{self.spin_rate:.2f} rad/s 匀速自转"
                )
            else:
                reason = (
                    "LIO 里程计中断"
                    if self.messages >= self.min_messages
                    else "LIO 尚未稳定"
                )
                self.get_logger().warn(f"{reason}，云台回 0（锁定）")
        self._publish(self.spin_rate if self.spinning else 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = GimbalSpinGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 退出前显式归零：JointController 不会因为发布者消失就停。
        try:
            node._publish(0.0)
        except Exception:  # noqa: BLE001 - 退出路径上尽力而为
            pass
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
