#!/usr/bin/env python3

# Copyright 2026 Shiyu
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

"""仿真真值里程计适配器：把 Gazebo 真值三维位姿变成导航链路用的里程计。

约定：

* ``odom -> base_link`` 的 TF 由本节点**单独拥有**（``sensor_scan_generation`` 的
  ``publish_tf`` 在仿真里为 False），因此这里发布的位姿就是整条链路的真值。
* odom 系是**重力对齐**的：原点取出生位置，姿态只取出生航向，z 轴始终竖直，
  高度以出生点地面高度为基准，不会随每条消息把 z 归零。
* ``odom -> base_link`` 保留完整三维姿态（含 roll/pitch），雷达位姿由完整底盘位姿
  组合外参得到；二维导航需要的 x/y/yaw 由下游显式提取。
"""

import math
import os
import sys
import time


import rclpy
from geometry_msgs.msg import Transform, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from tf2_ros import TransformBroadcaster

# 同目录的纯几何模块（无 ROS 依赖，便于单元测试）。显式加入脚本所在目录，兼容
# ``--symlink-install`` 且 Python 3.11+ 会把 ``sys.path[0]`` 解析成源码目录的情况。
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from srm_nav_pose_math import (  # noqa: E402 - 必须在 sys.path 调整之后导入
    compose_pose,
    inverse_pose,
    normalize_angle,
    pose_from_ros_pose,
    pose_from_transform,
    rpy_to_quaternion,
    yaw_from_quaternion,
)


POSITION_JITTER_THRESHOLD = 0.002
YAW_JITTER_THRESHOLD = 0.002


def load_lidar_extrinsic():
    """从 srm27_robot_description 的几何 YAML 读取 base_link -> 雷达外参。

    外参只在几何 YAML 里定义一次，避免这里再写一份硬编码数值与模型漂移。
    读取失败时回退到 SRM 的当前安装值，并打印告警。
    """
    fallback = {
        "link_name": "front_mid360",
        "xyz": [0.15, -0.15, 0.22],
        "rpy": [-0.06981317007977318, 0.0, -1.5707963267948966],
    }
    try:
        from srm27_robot_description import load_geometry

        lidar = load_geometry()["lidar"]
        return {
            "link_name": lidar["link_name"],
            "xyz": [float(value) for value in lidar["xyz"]],
            "rpy": [float(value) for value in lidar["rpy"]],
        }
    except Exception as error:  # noqa: BLE001 - 回退必须给出可见原因
        print(
            "[warning] 无法从 srm27_robot_description 读取雷达外参"
            f"（{error}），使用内置 SRM 默认值。",
            flush=True,
        )
        return fallback


class SimulationGroundTruthOdometry(Node):
    def __init__(self):
        super().__init__("simulation_ground_truth_odometry")
        # 导航速度参考系：SRM 使用真实随底盘自转的 base_link。
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        lidar_extrinsic = load_lidar_extrinsic()
        self.lidar_frame = self.declare_parameter(
            "lidar_frame", lidar_extrinsic["link_name"]
        ).value
        # base_link -> 雷达外参位姿（几何 YAML 是唯一来源，避免这里再写一份硬编码）。
        self.base_to_lidar = (
            tuple(lidar_extrinsic["xyz"]),
            rpy_to_quaternion(*lidar_extrinsic["rpy"]),
        )
        self.initial_pose = None
        self.zero_pose_since = None
        self.zero_pose_grace_ns = 2_000_000_000
        self.last_publish_time = None
        self.last_callback_time = None
        self.latest_message = None
        self.filtered_pose = None
        self.publish_period_ns = 20_000_000  # 50 Hz is sufficient for Nav2.
        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_publisher = self.create_publisher(Odometry, "odometry", 10)
        self.lidar_odom_publisher = self.create_publisher(Odometry, "lidar_odometry", 10)
        # Gazebo's mecanum plugin can publish odometry at the physics rate
        # (800+ Hz).  Keep only the newest sample in DDS so Python does not
        # build a backlog of stale poses while Nav2 only needs 50 Hz.
        odometry_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(
            Odometry, "chassis_odometry_gt", self._odometry_callback, odometry_qos
        )
        # Gazebo may deliver odometry at the physics rate (800+ Hz).  Keep the
        # subscription callback deliberately trivial and do pose conversion at
        # the fixed Nav2 rate below; otherwise Python spends most of a CPU
        # core calculating and publishing samples that are immediately
        # superseded.
        self.create_timer(self.publish_period_ns * 1e-9, self._process_latest)

    def _odometry_callback(self, message):
        self.latest_message = message

    def _process_latest(self):
        message = self.latest_message
        if message is None:
            return
        now_monotonic = time.monotonic_ns()
        self.last_callback_time = now_monotonic
        # Gazebo 真值给出**完整三维位姿**：位置含高度，姿态含 roll/pitch/yaw。这里刻意
        # 不做二维压平——odom -> base_link 的 TF、雷达位姿与点云变换都由完整位姿算出；
        # 上坡时忽略俯仰会让车前方 1 m 的点产生约 0.2 m 的高度误差。二维导航需要的
        # x/y/yaw 由下游（状态适配层）显式提取，不在感知链路上提前丢掉。
        world_pose = pose_from_ros_pose(message.pose.pose)
        world_position, world_orientation = world_pose
        yaw = yaw_from_quaternion(world_orientation)
        if self.initial_pose is None:
            # Gazebo can publish one zero-valued odometry sample while the
            # robot entity is still being spawned. Do not use that placeholder
            # as the odometry origin, or the real spawn pose will appear
            # outside the Nav2 map.
            is_zero_pose = (
                abs(world_position[0]) < 1e-6
                and abs(world_position[1]) < 1e-6
                and abs(world_position[2]) < 1e-6
                and abs(yaw) < 1e-6
            )
            if is_zero_pose:
                # A paused Gazebo world can repeatedly publish the placeholder
                # pose while /clock is still zero.  Never initialize from it:
                # the first real pose after unpausing may be the configured
                # spawn pose, far away from the world origin.
                if message.header.stamp.sec == 0 and message.header.stamp.nanosec == 0:
                    self.zero_pose_since = None
                    return
                now_ns = now_monotonic
                if self.zero_pose_since is None:
                    self.zero_pose_since = now_ns
                # Some Gazebo versions emit zero odometry until the model's
                # odometry plugin has settled. Give a real pose priority, but
                # do not leave Nav2 without an odom TF forever when zero is
                # the intended relative-odometry origin.
                if now_ns - self.zero_pose_since < self.zero_pose_grace_ns:
                    return
            # 里程计原点 = 出生位置 + 出生航向；roll/pitch 不进入原点定义，因此 odom 系
            # 始终重力对齐（z 轴竖直），高度基准就是出生点地面高度，不会被强制归零。
            self.initial_pose = (world_position, rpy_to_quaternion(0.0, 0.0, yaw))
            self.get_logger().info(
                "Ground-truth odometry origin initialized: "
                f"z={world_position[2]:.3f} m, yaw={math.degrees(yaw):.1f} deg"
            )

        # Use the node's ROS clock for every derived message.  Gazebo bridge
        # messages can carry a stale source timestamp after /clock starts;
        # forwarding it makes Nav2 request transforms from the distant past.
        # The bridge supplies a Gazebo /clock timestamp.  Reuse it so this
        # adapter does not need to subscribe to the high-rate /clock topic.
        stamp = rclpy.time.Time.from_msg(message.header.stamp)
        publish_time_ns = time.monotonic_ns()
        if (
            self.last_publish_time is not None
            and publish_time_ns - self.last_publish_time < self.publish_period_ns
        ):
            return
        self.last_publish_time = publish_time_ns

        # 相对位姿 T_odom_base = T_odom_initial⁻¹ ∘ T_world_base。odom 系原点在出生点、
        # 姿态只保留出生航向，所以 z 轴始终竖直、高度有明确基准。
        base_pose = self._suppress_jitter(
            compose_pose(inverse_pose(self.initial_pose), world_pose)
        )

        transform = self._transform_from_pose(base_pose, stamp, "odom", self.base_frame)
        self.tf_broadcaster.sendTransform(transform)

        odometry = Odometry()
        odometry.header = transform.header
        odometry.child_frame_id = transform.child_frame_id
        # 位姿与 TF 用**同一个完整三维位姿**：同名 frame 在 TF 与消息里必须表示同一件
        # 事，否则下游（点云变换、地形分析）会按平面位姿解释三维真值。
        odometry.pose.pose.position.x = base_pose[0][0]
        odometry.pose.pose.position.y = base_pose[0][1]
        odometry.pose.pose.position.z = base_pose[0][2]
        odometry.pose.pose.orientation = transform.transform.rotation
        odometry.twist = message.twist
        # twist 保持平面：速度指令本身就是平面的，Nav2 消费的也是平面底盘里程计；
        # 高度方向与 roll/pitch 角速度属于感知侧信息，保留在 Gazebo 真值话题里，
        # 不进入导航接口，避免下游把三维分量误当平面速度。
        odometry.twist.twist.linear.z = 0.0
        odometry.twist.twist.angular.x = 0.0
        odometry.twist.twist.angular.y = 0.0
        # 抑制静止时的物理噪声：否则控制器与 RViz 里的车会在停住后持续抽动。
        if abs(odometry.twist.twist.linear.x) < 0.002:
            odometry.twist.twist.linear.x = 0.0
        if abs(odometry.twist.twist.linear.y) < 0.002:
            odometry.twist.twist.linear.y = 0.0
        if abs(odometry.twist.twist.angular.z) < 0.002:
            odometry.twist.twist.angular.z = 0.0
        self.odom_publisher.publish(odometry)

        # sensor_scan_generation consumes a lidar pose together with the cloud.
        # Publish the same stable pose with the lidar child frame so this path
        # works without Point-LIO/loam_interface in simulation odometry mode.
        # 外参取自 srm27_robot_description 的几何 YAML（base_link -> front_mid360），
        # 由完整三维底盘位姿组合而成：底盘一旦有 roll/pitch，只有三维组合才正确。
        lidar_transform = TransformStamped()
        lidar_transform.header = transform.header
        lidar_transform.child_frame_id = self.lidar_frame
        lidar_transform.transform = self._compose(
            transform.transform, self._pose_to_transform(self.base_to_lidar)
        )
        lidar_odom = Odometry()
        lidar_odom.header = lidar_transform.header
        lidar_odom.child_frame_id = lidar_transform.child_frame_id
        lidar_odom.pose.pose.position.x = lidar_transform.transform.translation.x
        lidar_odom.pose.pose.position.y = lidar_transform.transform.translation.y
        lidar_odom.pose.pose.position.z = lidar_transform.transform.translation.z
        lidar_odom.pose.pose.orientation = lidar_transform.transform.rotation
        self.lidar_odom_publisher.publish(lidar_odom)

    def _suppress_jitter(self, pose):
        """静止时保持亚阈值抖动，同时不吞掉累积运动。

        保持判据仍是**平面**位移与航向（与二维导航调参一致）；判据成立时整个三维位姿
        （含高度与 roll/pitch）一起保持，避免静止时 RViz/代价地图的模型抽动；判据不
        成立时位姿整体更新，因此上坡过程中的高度与俯仰不会被吞掉。
        """
        if self.filtered_pose is None:
            self.filtered_pose = pose
            return self.filtered_pose

        position, orientation = pose
        last_position, last_orientation = self.filtered_pose
        yaw_delta = normalize_angle(
            yaw_from_quaternion(orientation) - yaw_from_quaternion(last_orientation)
        )
        if (
            math.hypot(
                position[0] - last_position[0], position[1] - last_position[1]
            )
            < POSITION_JITTER_THRESHOLD
            and abs(yaw_delta) < YAW_JITTER_THRESHOLD
        ):
            return self.filtered_pose

        self.filtered_pose = pose
        return self.filtered_pose

    @staticmethod
    def _pose_to_transform(pose):
        """把 ``(position, orientation)`` 位姿写成不带时间戳/坐标系的 ``Transform``。"""
        position, orientation = pose
        transform = Transform()
        transform.translation.x = position[0]
        transform.translation.y = position[1]
        transform.translation.z = position[2]
        (
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w,
        ) = orientation
        return transform

    @staticmethod
    def _transform_from_pose(pose, stamp, parent_frame, child_frame):
        """把 ``(position, orientation)`` 位姿写成 ``TransformStamped``。"""
        transform = TransformStamped()
        transform.header.stamp = stamp.to_msg()
        transform.header.frame_id = parent_frame
        transform.child_frame_id = child_frame
        transform.transform = SimulationGroundTruthOdometry._pose_to_transform(pose)
        return transform

    @staticmethod
    def _compose(first, second):
        """组合两个刚体变换：``T_out = T_first ∘ T_second``。

        平移必须用 **second 的平移**参与 first 的旋转
        （``t_out = t_first + R_first · t_second``），旋转用四元数乘法
        （``q_out = q_first ⊗ q_second``），两者使用互相独立的变量。

        历史缺陷：这里把 second 的**四元数分量**当成了平移代入叉乘，于是底盘转动时
        外参平移完全不跟着转。最小复现（first 只 yaw 90°、second 只有平移
        ``(0.15, -0.15, 0.22)`` 且旋转为单位四元数）：正确结果 ``(0.15, 0.15, 0.22)``，
        缺陷结果 ``(0.15, -0.15, 0.22)``，相差 0.30 m。
        """
        composed = compose_pose(pose_from_transform(first), pose_from_transform(second))
        return SimulationGroundTruthOdometry._pose_to_transform(composed)


def main(args=None):
    rclpy.init(args=args)
    node = SimulationGroundTruthOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        # launch 用 SIGINT 收尾时，rclpy 的信号处理会先把 context 关掉，
        # spin 于是抛 ExternalShutdownException 而不是 KeyboardInterrupt。
        # 不捕获会以退出码 1 结束，在 launch 日志里被记成 "[ERROR] process has died"，
        # 掩盖真正的异常；这里按正常退出处理。
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
