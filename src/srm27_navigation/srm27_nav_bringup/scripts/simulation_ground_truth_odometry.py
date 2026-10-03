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

import math
import time


import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from tf2_ros import TransformBroadcaster


POSITION_JITTER_THRESHOLD = 0.002
YAW_JITTER_THRESHOLD = 0.002


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class SimulationGroundTruthOdometry(Node):
    def __init__(self):
        super().__init__("simulation_ground_truth_odometry")
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
        pose = message.pose.pose
        yaw = yaw_from_quaternion(pose.orientation)
        if self.initial_pose is None:
            # Gazebo can publish one zero-valued odometry sample while the
            # robot entity is still being spawned. Do not use that placeholder
            # as the odometry origin, or the real spawn pose will appear
            # outside the Nav2 map.
            is_zero_pose = (
                abs(pose.position.x) < 1e-6
                and abs(pose.position.y) < 1e-6
                and abs(pose.position.z) < 1e-6
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
            self.initial_pose = (pose.position.x, pose.position.y, yaw)
            self.get_logger().info("Ground-truth odometry origin initialized")

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

        initial_x, initial_y, initial_yaw = self.initial_pose
        world_dx = pose.position.x - initial_x
        world_dy = pose.position.y - initial_y
        cos_yaw = math.cos(initial_yaw)
        sin_yaw = math.sin(initial_yaw)
        x = cos_yaw * world_dx + sin_yaw * world_dy
        y = -sin_yaw * world_dx + cos_yaw * world_dy
        relative_yaw = math.atan2(
            math.sin(yaw - initial_yaw), math.cos(yaw - initial_yaw)
        )
        x, y, relative_yaw = self._suppress_jitter(x, y, relative_yaw)

        transform = TransformStamped()
        transform.header.stamp = stamp.to_msg()
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_footprint"
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.rotation.z = math.sin(relative_yaw / 2.0)
        transform.transform.rotation.w = math.cos(relative_yaw / 2.0)
        self.tf_broadcaster.sendTransform(transform)

        odometry = Odometry()
        odometry.header = transform.header
        odometry.child_frame_id = transform.child_frame_id
        odometry.pose.pose.position.x = x
        odometry.pose.pose.position.y = y
        odometry.pose.pose.orientation = transform.transform.rotation
        odometry.twist = message.twist
        # The Gazebo plugin reports full-3D twist in the chassis frame.  Nav2
        # consumes a planar base odometry; discard vertical/roll/pitch motion
        # and suppress tiny physics noise that otherwise keeps the controller
        # and RViz model twitching while stopped.
        odometry.twist.twist.linear.z = 0.0
        odometry.twist.twist.angular.x = 0.0
        odometry.twist.twist.angular.y = 0.0
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
        # Fixed model extrinsic: base_footprint -> chassis (0, 0, 0.076),
        # chassis -> front_mid360 (0.16, 0, 0.18), roll=30 deg, yaw=90 deg.
        lidar_tf = TransformStamped().transform
        lidar_tf.translation.x = 0.16
        lidar_tf.translation.z = 0.256
        lidar_tf.rotation.x = 0.1830127
        lidar_tf.rotation.y = 0.1830127
        lidar_tf.rotation.z = 0.6830127
        lidar_tf.rotation.w = 0.6830127
        lidar_transform = TransformStamped()
        lidar_transform.header = transform.header
        lidar_transform.child_frame_id = "front_mid360"
        lidar_transform.transform = self._compose(transform.transform, lidar_tf)
        lidar_odom = Odometry()
        lidar_odom.header = lidar_transform.header
        lidar_odom.child_frame_id = lidar_transform.child_frame_id
        lidar_odom.pose.pose.position.x = lidar_transform.transform.translation.x
        lidar_odom.pose.pose.position.y = lidar_transform.transform.translation.y
        lidar_odom.pose.pose.position.z = lidar_transform.transform.translation.z
        lidar_odom.pose.pose.orientation = lidar_transform.transform.rotation
        self.lidar_odom_publisher.publish(lidar_odom)

    def _suppress_jitter(self, x, y, yaw):
        """Hold sub-threshold stationary noise while preserving accumulated motion."""
        if self.filtered_pose is None:
            self.filtered_pose = (x, y, yaw)
            return self.filtered_pose

        last_x, last_y, last_yaw = self.filtered_pose
        yaw_delta = math.atan2(math.sin(yaw - last_yaw), math.cos(yaw - last_yaw))
        if (
            math.hypot(x - last_x, y - last_y) < POSITION_JITTER_THRESHOLD
            and abs(yaw_delta) < YAW_JITTER_THRESHOLD
        ):
            return self.filtered_pose

        self.filtered_pose = (x, y, yaw)
        return self.filtered_pose

    @staticmethod
    def _compose(first, second):
        """Compose two rigid transforms using only standard-library math."""
        qx, qy, qz, qw = first.rotation.x, first.rotation.y, first.rotation.z, first.rotation.w
        sx, sy, sz, sw = second.rotation.x, second.rotation.y, second.rotation.z, second.rotation.w
        # Rotate the second translation by the first quaternion.
        tx = 2.0 * (qy * sz - qz * sy)
        ty = 2.0 * (qz * sx - qx * sz)
        tz = 2.0 * (qx * sy - qy * sx)
        output = TransformStamped().transform
        output.translation.x = first.translation.x + (
            second.translation.x + qw * tx + (qy * tz - qz * ty))
        output.translation.y = first.translation.y + (
            second.translation.y + qw * ty + (qz * tx - qx * tz))
        output.translation.z = first.translation.z + (
            second.translation.z + qw * tz + (qx * ty - qy * tx))
        output.rotation.x = qw * sx + qx * sw + qy * sz - qz * sy
        output.rotation.y = qw * sy - qx * sz + qy * sw + qz * sx
        output.rotation.z = qw * sz + qx * sy - qy * sx + qz * sw
        output.rotation.w = qw * sw - qx * sx - qy * sy - qz * sz
        return output


def main(args=None):
    rclpy.init(args=args)
    node = SimulationGroundTruthOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
