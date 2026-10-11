#!/usr/bin/env python3
"""手动/半自动注入初值偏置，观察 GICP 重定位效果。

它是 ``gicp_initialization_runner`` 的「手动挡」版本：**不重启任何节点**，
只做三件事——

1. 周期校验评估器里评估器/真值是否就绪；
2. 按给定偏置算出「机器人位姿猜测」并发布 ``/initialpose``；
3. 轮询评估器状态，把「是否成功、耗时、误差」打印出来（可选转成 CSV）。

坐标关系（方案 §5）::

    T_M_O_init = T_M_B_guess · T_O_B(t_init)^-1

其中 ``T_M_B_guess`` 是**机器人位姿**的猜测（真值 + 人为偏置），
``T_O_B(t_init)`` 由本工具用同一份真值算出，因此注入的偏置就是
「场景里看到的初始误差」。偏置坐标系为 **map 系**（不是车体系）。

用法（必须和栈在同一个 bash 调用里，见 script/gicp_validation/run_session_manual.sh）::

    gicp_manual_initial_pose --ros-args \\
        -p dx:=1.0 -p dy:=0.0 -p dyaw_deg:=90 -p watch_seconds:=12
"""

import argparse
import csv
import json
import math
import os
import sys
import time

import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from gicp_validation_common import (  # noqa: E402
    invert,
    make_transform,
    matrix_to_quaternion,
    quaternion_to_matrix,
    transform_from_pose,
    wrap_to_pi,
    yaw_from_matrix,
)

SUMMARY_FIELDS = [
    "label",
    "dx",
    "dy",
    "dyaw_deg",
    "passed",
    "failure_reason",
    "accepted_count",
    "suspect_count",
    "e_xy_max",
    "e_yaw_max",
    "t_first_accepted_s",
    "init_duration_wall",
    "note",
]


class ManualInitialPose(Node):
    def __init__(self, config: dict) -> None:
        # 关键：ROS 参数里 `-p dx:=0` 会被解析成 INTEGER，若 dx 已声明为 DOUBLE
        # 就会抛 InvalidParameterTypeException。开启
        # automatically_declare_parameters_from_overrides 让覆盖值按自己的类型自动声明，
        # 之后 get_parameter 再统一转 float。
        super().__init__(
            "gicp_manual_initial_pose",
            automatically_declare_parameters_from_overrides=True,
        )

        if not self.has_parameter("dx"):
            self.declare_parameter("dx", 0.0)
        if not self.has_parameter("dy"):
            self.declare_parameter("dy", 0.0)
        if not self.has_parameter("dyaw_deg"):
            self.declare_parameter("dyaw_deg", 0.0)
        if not self.has_parameter("label"):
            self.declare_parameter("label", "")
        if not self.has_parameter("watch_seconds"):
            self.declare_parameter("watch_seconds", 12.0)
        if not self.has_parameter("settle_seconds"):
            self.declare_parameter("settle_seconds", 1.0)
        if not self.has_parameter("repeats"):
            self.declare_parameter("repeats", 1)
        if not self.has_parameter("publish_twice"):
            self.declare_parameter("publish_twice", True)
        if not self.has_parameter("summary_csv"):
            self.declare_parameter("summary_csv", "")
        if not self.has_parameter("note"):
            self.declare_parameter("note", "")

        self.dx = float(self.get_parameter("dx").value or 0.0)
        self.dy = float(self.get_parameter("dy").value or 0.0)
        self.dyaw_deg = float(self.get_parameter("dyaw_deg").value or 0.0)
        self.label = (self.get_parameter("label").value or "") or (
            "dx%.2f_dy%.2f_dyaw%+.0f" % (self.dx, self.dy, self.dyaw_deg)
        )
        self.watch_seconds = float(self.get_parameter("watch_seconds").value)
        self.settle_seconds = float(self.get_parameter("settle_seconds").value)
        self.repeats = int(self.get_parameter("repeats").value)
        self.publish_twice = bool(self.get_parameter("publish_twice").value)
        self.summary_csv = self.get_parameter("summary_csv").value
        self.note = self.get_parameter("note").value

        self.namespace = config.get("namespace", "red_standard_robot1")
        spawn = config.get("spawn", {})
        self.t_map_world = invert(
            transform_from_pose(
                float(spawn.get("x", 0.0)),
                float(spawn.get("y", 0.0)),
                0.0,
                float(spawn.get("yaw", 0.0)),
            )
        )

        self.latest_gt = None
        self.gt_count = 0
        self.status = {}

        self.create_subscription(
            Odometry,
            "/%s/chassis_odometry_gt" % self.namespace,
            self.on_gt,
            50,
        )
        self.create_subscription(String, "gicp_eval/status", self.on_status, 20)
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/%s/initialpose" % self.namespace, 10
        )

        self.get_logger().info(
            "[manual] 就绪：namespace=%s 偏置 dx=%.3f dy=%.3f dyaw=%+.1f° label=%s"
            % (self.namespace, self.dx, self.dy, self.dyaw_deg, self.label)
        )

    def on_gt(self, msg: Odometry) -> None:
        self.gt_count += 1
        pose = msg.pose.pose
        self.latest_gt = make_transform(
            [pose.position.x, pose.position.y, pose.position.z],
            quaternion_to_matrix(
                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
            ),
        )

    def on_status(self, msg: String) -> None:
        try:
            self.status = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    # ------------------------------------------------------------------ 动作

    def make_guess(self):
        """返回 (T_M_B_guess, T_M_O_init)；偏置在 map 系。"""
        if self.latest_gt is None:
            raise RuntimeError("还没有真值 chassis_odometry_gt")
        truth_map = self.t_map_world @ self.latest_gt
        yaw_truth = yaw_from_matrix(truth_map[:3, :3])
        guess = transform_from_pose(
            truth_map[0, 3] + self.dx,
            truth_map[1, 3] + self.dy,
            0.0,
            wrap_to_pi(yaw_truth + math.radians(self.dyaw_deg)),
        )
        # T_O_B(t_init) = inverse(T_M_W · T_W_B)
        t_o_b = invert(self.t_map_world @ self.latest_gt)
        return guess, guess @ invert(t_o_b)

    def publish_guess(self, guess) -> None:
        message = PoseWithCovarianceStamped()
        message.header.frame_id = "map"
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.pose.position.x = float(guess[0, 3])
        message.pose.pose.position.y = float(guess[1, 3])
        message.pose.pose.position.z = float(guess[2, 3])
        x, y, z, w = matrix_to_quaternion(guess[:3, :3])
        message.pose.pose.orientation.x = x
        message.pose.pose.orientation.y = y
        message.pose.pose.orientation.z = z
        message.pose.pose.orientation.w = w
        self.pose_pub.publish(message)

    def spin_for(self, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

    def wait_for_truth(self, timeout: float = 60.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.latest_gt is not None:
                return True
        return False

    def run_once(self, index: int) -> dict:
        guess, map_to_odom = self.make_guess()

        self.get_logger().info(
            "[manual] 第 %d 次：猜测位姿 (%.3f, %.3f, yaw=%+.3f°) → T_M_O_init "
            "(%.3f, %.3f, yaw=%+.3f°)"
            % (
                index,
                guess[0, 3],
                guess[1, 3],
                math.degrees(yaw_from_matrix(guess[:3, :3])),
                map_to_odom[0, 3],
                map_to_odom[1, 3],
                math.degrees(yaw_from_matrix(map_to_odom[:3, :3])),
            )
        )

        self.publish_guess(guess)
        if self.publish_twice:
            self.spin_for(0.2)
            self.publish_guess(guess)

        deadline = time.monotonic() + self.watch_seconds
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.status.get("success") or self.status.get("failure_reason"):
                break
            if self.status.get("active") is False and self.status.get("diag_seen"):
                # 评估器没在跑轮次也能从状态里读到进度，继续等到超时
                pass

        result = {
            "label": "%s#%d" % (self.label, index),
            "dx": self.dx,
            "dy": self.dy,
            "dyaw_deg": self.dyaw_deg,
            "passed": bool(self.status.get("success")),
            "failure_reason": self.status.get("failure_reason", ""),
            "accepted_count": self.status.get("accepted_diag_seen", ""),
            "suspect_count": self.status.get("suspicious", ""),
            "e_xy_max": "",
            "e_yaw_max": "",
            "t_first_accepted_s": "",
            "init_duration_wall": "",
            "note": self.note,
        }
        self.get_logger().info(
            "[manual] 结果：success=%s failure=%s accepted=%s suspicious=%s"
            % (
                result["passed"],
                result["failure_reason"] or "-",
                result["accepted_count"],
                result["suspect_count"],
            )
        )
        return result

    def run(self) -> int:
        if not self.wait_for_truth():
            self.get_logger().error("[manual] 等不到真值 chassis_odometry_gt")
            return 1

        # 让评估器复位并开始计时（评估器在轮次里才算稳定窗）
        self.reset_evaluator()
        self.spin_for(self.settle_seconds)

        results = []
        for index in range(1, self.repeats + 1):
            results.append(self.run_once(index))
            if index < self.repeats:
                self.get_logger().info("[manual] 重启被测节点以便下一轮从干净状态开始")
                self.restart_gicp()
                self.reset_evaluator()
                self.spin_until_gicp_ready()
                self.spin_for(self.settle_seconds)

        if self.summary_csv:
            self.write_csv(results)
        return 0 if all(item["passed"] for item in results) else 1

    # ------------------------------------------------------------------ 辅助

    def call_trigger(self, name: str, timeout: float = 5.0) -> bool:
        from std_srvs.srv import Trigger

        client = self.create_client(Trigger, name)
        if not client.wait_for_service(timeout_sec=timeout):
            self.destroy_client(client)
            return False
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        self.destroy_client(client)
        return future.done()

    def reset_evaluator(self) -> None:
        self.call_trigger("gicp_eval/stop_run")
        self.status = {}
        self.call_trigger("gicp_eval/start_run")

    def restart_gicp(self) -> None:
        """通过重建 launch 之外的方式重启被测节点：这里交给外层的重启脚本。

        为保持本工具「只发初值」的职责单一，重启由 run_session_manual.sh 提供；
        若外部没有重启，则退化为「运行中重发初值」口径（方案 §6.8 允许，但要标注）。
        """
        flag = os.environ.get("GICP_MANUAL_RESTART_CMD")
        if not flag:
            self.get_logger().warn(
                "[manual] 未配置 GICP_MANUAL_RESTART_CMD，本轮按「运行中重置初值」口径"
            )
            return
        import subprocess

        subprocess.run(["bash", "-lc", flag], check=False, timeout=60)
        self.spin_for(3.0)

    def spin_until_gicp_ready(self, timeout: float = 30.0) -> None:
        deadline = time.monotonic() + timeout
        before = self.status.get("diag_seen", 0)
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.status.get("diag_seen", 0) > before:
                return

    def write_csv(self, results) -> None:
        os.makedirs(os.path.dirname(self.summary_csv), exist_ok=True)
        write_header = not os.path.isfile(self.summary_csv)
        with open(self.summary_csv, "a", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=SUMMARY_FIELDS)
            if write_header:
                writer.writeheader()
            for row in results:
                writer.writerow(row)
        self.get_logger().info("[manual] 结果已追加到 %s" % self.summary_csv)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="手动注入初值偏置并观察重定位效果")
    parser.add_argument("--config", required=True, help="验证配置 YAML（取 namespace/spawn）")
    known, remaining = parser.parse_known_args(argv)

    with open(known.config, "r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)

    rclpy.init(args=remaining)
    node = ManualInitialPose(config)
    try:
        return node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
