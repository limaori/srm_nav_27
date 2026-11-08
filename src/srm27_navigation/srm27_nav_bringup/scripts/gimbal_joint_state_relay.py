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

"""云台关节角 -> /joint_states 中继（实车云台自转的前置条件）。

为什么需要它
------------
实车雷达装在 yaw 云台上，云台由**下位机**驱动自转，所以

    base_link yaw = 云台世界 yaw − 云台关节角

`robot_state_publisher` 用 `gimbal_joint_type:=revolute` 时，`base_link -> gimbal_yaw`
这条 TF 角**只来自 `/joint_states`**（见 `srm_robot_description.py` 的模型说明）。
而当前《SRM 校内赛导航通信协议》下行只有 vx/vy/wz、上行只有比赛状态类数据，
**没有云台关节角字段**，因此实车现在没有任何节点发布 `/joint_states`：

* 云台不转时没问题（关节角恒 0，`revolute` 与 `fixed` 等价）；
* 云台一转，`sensor_scan_generation` 反推出的 `odom -> base_link` 会把云台的转动
  当成底盘的转动 —— 车在 RViz/代价地图里会"自己转身"，导航立刻跑偏。

本节点只做一件事：把"下位机回传的关节角"（任何话题）按时转成标准
`sensor_msgs/JointState`，让上面那条 TF 边有数据。协议补上字段后，把
`input_topic` / `input_type` 指过去即可，**不需要改导航栈**。

输入格式（--input-type）
-----------------------
* `float32`（默认）：`data` = 云台相对底盘的关节角（rad，顺时针为负，与 URDF 的
  revolute 正方向一致）。消息类型由 `float_msg_type` 选：`std_msgs`（默认）或
  `example_interfaces` —— 同一话题**只能选一种**，两个都订会直接抛 RCLError。
* `joint_state`：`sensor_msgs/msg/JointState`，取其 `name` 里等于 `input_joint_name`
  的那一项 position（可同时给 velocity）。

输出
----
`/<ns>/joint_states`（`sensor_msgs/msg/JointState`），关节名固定 `joint_name`（默认
`gimbal_yaw_joint`，必须与 URDF 一致），按 `publish_rate` 定频重发最近一次的角
（`robot_state_publisher` 是消息驱动的，不发就不更新 TF）。

失效策略（重要）
----------------
输入超过 `timeout_sec` 没有更新时**停止发布并告警**，而不是发 0：
发 0 等于告诉 TF"云台回零了"，而真实云台可能还在转 —— 那会把误差藏起来。
宁可让 TF 停在最后一帧（RViz 里能看出来），也不要伪造一个"看起来正常"的角度。

用法
----
    ros2 run srm27_nav_bringup gimbal_joint_state_relay.py \\
        --ros-args -p input_topic:=gimbal_joint_angle -p publish_rate:=100.0

或由实车入口带起（推荐）::

    ros2 launch srm27_nav_bringup nav_srm_real_launch.py \\
        gimbal_joint_topic:=gimbal_joint_angle

⚠ 本节点只是"接口就绪"：它不产生角度。协议下行/上行补字段之前，
实车请保持云台固定（`--no-gimbal-spin` 或不下发云台自转），否则 yaw 必错。
详见 docs/仿真链迁移实车实施记录(ai).md。
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import JointState
from std_msgs.msg import Float32

try:  # 部分下位机桥接直接用 example_interfaces 的 Float32（与 std_msgs 是不同类型）
    from example_interfaces.msg import Float32 as ExampleFloat32
except ImportError:  # pragma: no cover - 一定装得上
    ExampleFloat32 = None


class GimbalJointStateRelay(Node):
    """把云台关节角转成 JointState，供 robot_state_publisher 发布动态 TF。"""

    def __init__(self):
        super().__init__("gimbal_joint_state_relay")

        self.declare_parameter("input_topic", "gimbal_joint_angle")
        self.declare_parameter("input_type", "float32")
        # constant: 不订阅任何话题，按固定角发布(暂位模式, 见文件头说明)
        self.declare_parameter("constant_angle", 0.0)
        # 同一话题只能有一种消息类型：std_msgs/Float32 与 example_interfaces/Float32 是
        # 两个不同的类型，同时订阅会直接抛 RCLError（invalid allocator），节点当场退出。
        self.declare_parameter("float_msg_type", "std_msgs")
        self.declare_parameter("input_joint_name", "gimbal_yaw_joint")
        self.declare_parameter("joint_name", "gimbal_yaw_joint")
        self.declare_parameter("output_topic", "joint_states")
        self.declare_parameter("publish_rate", 100.0)
        self.declare_parameter("timeout_sec", 0.3)
        self.declare_parameter("angle_offset", 0.0)
        self.declare_parameter("wrap_to_pi", True)

        input_topic = str(self.get_parameter("input_topic").value)
        self.input_type = str(self.get_parameter("input_type").value).strip().lower()
        self.input_joint_name = str(self.get_parameter("input_joint_name").value)
        self.float_msg_type = str(self.get_parameter("float_msg_type").value).strip().lower()
        self.joint_name = str(self.get_parameter("joint_name").value)
        self.angle_offset = float(self.get_parameter("angle_offset").value)
        self.wrap_to_pi = bool(self.get_parameter("wrap_to_pi").value)
        rate = float(self.get_parameter("publish_rate").value)
        self.timeout_sec = float(self.get_parameter("timeout_sec").value)

        if rate <= 0.0 or not math.isfinite(rate):
            raise ValueError("publish_rate 必须是正数")
        if self.timeout_sec <= 0.0 or not math.isfinite(self.timeout_sec):
            raise ValueError("timeout_sec 必须是正数")
        if self.input_type not in ("float32", "joint_state", "constant"):
            raise ValueError("input_type 只支持 float32 / joint_state / constant")
        if self.float_msg_type not in ("std_msgs", "example_interfaces"):
            raise ValueError("float_msg_type 只支持 std_msgs / example_interfaces")

        self.angle = None
        self.velocity = None
        self.last_input_steady = 0.0
        self.input_count = 0
        self.stale_count = 0
        self.non_finite_count = 0
        self.publish_count = 0

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.publisher = self.create_publisher(JointState, str(self.get_parameter("output_topic").value), qos)

        if self.input_type == "constant":
            # 暂位模式: 没有云台角回传时也要让 base_link->gimbal_yaw 这条 TF 存在,
            # 否则 robot_state_publisher 一个动态 TF 都不发, 整棵树断开。
            angle = float(self.get_parameter("constant_angle").value)
            if self.wrap_to_pi:
                angle = math.atan2(math.sin(angle), math.cos(angle))
            self.angle = angle + self.angle_offset
            self.last_input_steady = _steady()
        elif self.input_type == "joint_state":
            self.create_subscription(JointState, input_topic, self._joint_state_callback, qos)
        elif self.float_msg_type == "example_interfaces":
            if ExampleFloat32 is None:
                raise ValueError("float_msg_type=example_interfaces 但该消息包不可用")
            self.create_subscription(ExampleFloat32, input_topic, self._float_callback, qos)
        else:
            self.create_subscription(Float32, input_topic, self._float_callback, qos)

        self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info(
            "云台关节角中继已启动: %s(%s) -> /%s, 关节名=%s, %.0f Hz, 超时 %.2f s"
            % (
                input_topic,
                self.input_type,
                str(self.get_parameter("output_topic").value),
                self.joint_name,
                rate,
                self.timeout_sec,
            )
        )
        if self.angle_offset:
            self.get_logger().info("角度偏置 angle_offset=%.4f rad（已加到回传角上）" % self.angle_offset)

    # ---- 输入 ----
    def _float_callback(self, msg):
        self._accept(float(msg.data), None)

    def _joint_state_callback(self, msg):
        if self.input_joint_name not in msg.name:
            return
        index = msg.name.index(self.input_joint_name)
        if index >= len(msg.position):
            return
        velocity = None
        if index < len(msg.velocity):
            velocity = float(msg.velocity[index])
        self._accept(float(msg.position[index]), velocity)

    def _accept(self, angle, velocity):
        if not math.isfinite(angle):
            self.non_finite_count += 1
            self.get_logger().warn(
                "收到非有限关节角，已忽略（累计 %d 次）" % self.non_finite_count,
                throttle_duration_sec=2.0,
            )
            return
        if self.wrap_to_pi:
            angle = math.atan2(math.sin(angle), math.cos(angle))
        self.angle = angle + self.angle_offset
        self.velocity = velocity
        # 新鲜度用墙钟：仿真时钟暂停/回放时不该被当成"数据过期"。
        self.last_input_steady = _steady()
        self.input_count += 1

    # ---- 输出 ----
    def _publish(self):
        if self.angle is None:
            return
        if self.input_type != "constant" and _steady() - self.last_input_steady > self.timeout_sec:
            self.stale_count += 1
            self.angle = None  # 停止发布，等新数据；不要发 0 冒充"回零"
            self.get_logger().warn(
                "关节角超过 %.2f s 未更新，已停止发布 TF（累计 %d 次）；"
                "云台自转期间这会让底盘 yaw 变错，请检查下位机回传" % (self.timeout_sec, self.stale_count),
                throttle_duration_sec=2.0,
            )
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [self.joint_name]
        msg.position = [self.angle]
        if self.velocity is not None and math.isfinite(self.velocity):
            msg.velocity = [self.velocity]
        self.publisher.publish(msg)
        self.publish_count += 1


def _steady():
    """墙钟单调时间：关节角的新鲜度判断不能依赖仿真时钟。"""
    return time.monotonic()


def main(args=None):
    rclpy.init(args=args)
    node = GimbalJointStateRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
