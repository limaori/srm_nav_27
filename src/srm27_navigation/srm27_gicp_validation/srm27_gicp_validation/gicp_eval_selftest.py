#!/usr/bin/env python3
"""评估器离线自检：合成真值 + TF + 诊断，验证验收判据能正确判通过/失败。

不依赖 Gazebo：直接用配置的出生点生成自洽的
``chassis_odometry_gt``、``map → odom``、``odom → base_link`` 与 GICP 诊断 JSON，
让 ``gicp_truth_evaluator`` 在没有仿真的环境下被端到端验证一次。

场景：

* ``good``：位姿误差在容差内持续 >3 s → 期望 passed=true；
* ``bad``：位姿误差 1.0 m 持续 >10 s → 期望 passed=false（观察超时）；
* ``fake``：RMSE 很低但位置错到另一个区域 → 期望记入 suspicious，判失败。

用法::

    ros2 run srm27_gicp_validation gicp_eval_selftest --ros-args -p scenario:=good
"""

import argparse
import json
import math
import sys
import time

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String

sys.path.insert(0, __file__.rsplit("/", 1)[0])

from gicp_validation_common import (  # noqa: E402
    invert,
    make_transform,
    matrix_to_quaternion,
    quaternion_to_matrix,
    transform_from_pose,
    yaw_from_matrix,
)


class SelfTest(Node):
    def __init__(self, scenario: str, spawn, error_xy: float, error_yaw_deg: float, rate: float):
        super().__init__("gicp_eval_selftest")
        self.scenario = scenario
        self.spawn = spawn
        self.t_map_world = invert(spawn)
        self.rate = rate
        self.seq = 0

        self.gt_pub = self.create_publisher(Odometry, "chassis_odometry_gt", 20)
        self.diag_pub = self.create_publisher(String, "gicp_diagnostics", 50)
        self.init_pub = self.create_publisher(PoseWithCovarianceStamped, "initialpose", 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.good_pose = self.t_map_world @ spawn  # T_M_B 真值
        error = transform_from_pose(error_xy, 0.0, 0.0, math.radians(error_yaw_deg))
        self.est_pose_map = self.t_map_world @ spawn @ error  # T_M_B 估计
        # 静止机器人：odom → base_link 固定为 T_M_W · T_W_B(初值时刻)
        self.t_o_b_fixed = invert(self.t_map_world @ spawn)
        self.timer = self.create_timer(1.0 / rate, self.step)

        self.start_wall = time.monotonic()
        self.initial_sent = False
        self.get_logger().info(
            "[selftest][debug] spawn=%s est_pose_map=%s map_odom=%s"
            % (
                np.round(self.spawn[:3, 3], 4).tolist(),
                np.round(self.est_pose_map[:3, 3], 4).tolist(),
                np.round((self.est_pose_map @ invert(self.t_o_b_fixed))[:3, 3], 4).tolist(),
            )
        )
        self.get_logger().info(
            "[selftest] scenario=%s 位置误差=%.2f m 航向误差=%.1f°"
            % (scenario, error_xy, error_yaw_deg)
        )

    def step(self) -> None:
        now = self.get_clock().now().to_msg()
        # 真值：T_W_B
        gt = Odometry()
        gt.header.stamp = now
        gt.header.frame_id = "odom"
        gt.child_frame_id = "base_link"
        gt.pose.pose.position.x = float(self.spawn[0, 3])
        gt.pose.pose.position.y = float(self.spawn[1, 3])
        gt.pose.pose.position.z = 0.0
        qx, qy, qz, qw = matrix_to_quaternion(self.spawn[:3, :3])
        gt.pose.pose.orientation.x = qx
        gt.pose.pose.orientation.y = qy
        gt.pose.pose.orientation.z = qz
        gt.pose.pose.orientation.w = qw
        self.gt_pub.publish(gt)

        # 被测链路：odom → base_link 是**固定**的（静止机器人），map → odom 由被测节点给出。
        # 关键：T_O_B 必须在初值时刻定死，否则每帧重算会把注入的误差抵消掉
        # （T_M_O_est · T_O_B = T_M_B_est 恒成立，误差就消失了）。
        t_o_b = self.t_o_b_fixed
        map_odom = self.est_pose_map @ invert(t_o_b)
        for parent, child, transform in (
            ("map", "odom", map_odom),
            ("odom", "base_link", t_o_b),
        ):
            message = TransformStamped()
            message.header.stamp = now
            message.header.frame_id = parent
            message.child_frame_id = child
            message.transform.translation.x = float(transform[0, 3])
            message.transform.translation.y = float(transform[1, 3])
            message.transform.translation.z = float(transform[2, 3])
            qx, qy, qz, qw = matrix_to_quaternion(transform[:3, :3])
            message.transform.rotation.x = qx
            message.transform.rotation.y = qy
            message.transform.rotation.z = qz
            message.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(message)

        if not self.initial_sent and time.monotonic() - self.start_wall > 1.0:
            self.initial_sent = True
            guess = PoseWithCovarianceStamped()
            guess.header.frame_id = "map"
            guess.header.stamp = now
            guess.pose.pose.position.x = float(self.t_map_world[0, 3])
            guess.pose.pose.position.y = float(self.t_map_world[1, 3])
            guess.pose.pose.orientation.w = 1.0
            self.init_pub.publish(guess)

            event = {
                "seq": self.seq,
                "run_id": "selftest",
                "event": "initial_pose_applied",
                "node_wall_ns": self.get_clock().now().nanoseconds,
                "scan_stamp_ns": 0,
                "branch": "initial_pose_applied",
                "reason": "",
            }
            message = String()
            message.data = json.dumps(event)
            self.diag_pub.publish(message)

        if self.initial_sent:
            self.seq += 1
            scan_stamp_ns = self.get_clock().now().nanoseconds
            rmse = 0.02 if self.scenario != "noisy" else 0.25
            record = {
                "seq": self.seq,
                "run_id": "selftest",
                "node_wall_ns": scan_stamp_ns,
                "scan_stamp_ns": scan_stamp_ns,
                "scan_age": 0.01,
                "has_scan": True,
                "raw_points": 20000,
                "source_points": 3000,
                "converged": True,
                "iterations": 12,
                "accepted": True,
                "branch": "accepted",
                "reason": "",
                "inlier_ratio": 0.92 if self.scenario != "fake" else 0.94,
                "rmse": rmse,
                "dpos": 0.01,
                "dangle": 0.01,
                "duration_ms": 12.0,
                "result_x": float(self.est_pose_map[0, 3]),
                "result_y": float(self.est_pose_map[1, 3]),
                "result_yaw": float(yaw_from_matrix(self.est_pose_map[:3, :3])),
            }
            message = String()
            message.data = json.dumps(record)
            self.diag_pub.publish(message)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="评估器离线自检")
    parser.add_argument("--scenario", default="good", choices=["good", "bad", "fake", "noisy"])
    parser.add_argument("--error-xy", type=float, default=None)
    parser.add_argument("--error-yaw-deg", type=float, default=None)
    parser.add_argument("--rate", type=float, default=20.0)
    # 兼容两种调用方式：`--scenario bad`（直接命令行）与
    # `--ros-args -p scenario:=bad`（ros2 run 风格）。注意 argparse 的
    # `--scenario bad` 不会被 ROS 解析，所以必须在这里显式读取 ROS 参数。
    known, remaining = parser.parse_known_args(argv)
    ros_scenario = None
    for index, token in enumerate(remaining):
        if token in ("-p", "--param") and index + 1 < len(remaining):
            key_value = remaining[index + 1]
            if key_value.startswith("scenario:="):
                ros_scenario = key_value.split(":=", 1)[1]
    if ros_scenario is not None:
        known.scenario = ros_scenario

    defaults = {
        "good": (0.05, 1.0),
        "bad": (1.0, 0.0),
        "fake": (5.0, 30.0),
        "noisy": (0.10, 2.0),
    }
    error_xy, error_yaw = defaults[known.scenario]
    if known.error_xy is not None:
        error_xy = known.error_xy
    if known.error_yaw_deg is not None:
        error_yaw = known.error_yaw_deg

    spawn = transform_from_pose(3.4, 9.5, 0.0, 0.0)
    rclpy.init(args=remaining)
    node = SelfTest(known.scenario, spawn, error_xy, error_yaw, known.rate)
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
