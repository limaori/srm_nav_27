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

"""fakeyaw 链路的 P0 自检（方案 §5 P0，不需要 Gazebo / Nav2）。

被测对象是 **真实节点** `fake_vel_transform`（`transform_mode=nav_se2`），不是数学库：
本脚本自己造人工里程计与人工速度命令，检查下面四件事。

1. 没有里程计时必须输出**零速**（不允许沿用旧 yaw）；
2. 四个人工 yaw（0、±π/2、π）下 `v_base = R(-psi) * v_fake` 的方向正确，
   平移模长保持，且 `wz` **逐条透传**（误差 < 1e-6，方案 §5 P0 门槛）；
3. `base_link -> base_link_fake` 的 TF yaw 等于 `-psi`，且平移为零；
4. 里程计超时后重新输出零速；诊断里 `wz_error_max == 0`。

用法::

    ./script/check_wz_fakeyaw_chain.py
    ROS_DOMAIN_ID=77 ./script/check_wz_fakeyaw_chain.py   # 换域，避免与在跑的仿真串味

退出码 0 = 全部通过。
"""

import argparse
import math
import os
import signal
import subprocess
import sys
import tempfile
import time

# rclpy 在 init() 时会往 ROS_LOG_DIR 写日志文件；只读的 HOME（CI / 沙箱）会让它直接抛
# RuntimeError: Failed opening file ... for writing。这里先兜一个可写的临时目录再导入 rclpy。
if not os.environ.get("ROS_LOG_DIR"):
    os.environ["ROS_LOG_DIR"] = tempfile.mkdtemp(prefix="fakeyaw_p0_log_")

import rclpy  # noqa: E402 - 必须在 ROS_LOG_DIR 就绪之后导入
import tf2_ros
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

#: 方案 §5 P0 的门槛。
TOLERANCE = 1.0e-6

#: 四个基准 yaw：yaw -> (qz, qw)。
YAW_CASES = [
    ("0", 0.0, 0.0, 1.0),
    ("+pi/2", math.pi / 2.0, math.sin(math.pi / 4.0), math.cos(math.pi / 4.0)),
    ("pi", math.pi, 1.0, 0.0),
    ("-pi/2", -math.pi / 2.0, -math.sin(math.pi / 4.0), math.cos(math.pi / 4.0)),
]

#: 输入固定为 vx=0.3, vy=0.2, wz=0.5；期望输出按方案 §3.1 手工算好（不复用被测公式）。
INPUT_VX, INPUT_VY, INPUT_WZ = 0.3, 0.2, 0.5
EXPECTED = {
    "0": (0.3, 0.2),
    "+pi/2": (0.2, -0.3),
    "pi": (-0.3, -0.2),
    "-pi/2": (-0.2, 0.3),
}


class Checker(Node):
    """人工里程计 / 人工速度命令 + 结果采样。"""

    def __init__(self, odom_timeout):
        super().__init__("fakeyaw_p0_check")
        self.odom_timeout = odom_timeout
        self.yaw = 0.0
        self.publish_odom = False
        self.publish_cmd = False

        reliable = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.odom_pub = self.create_publisher(Odometry, "odometry", reliable)
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel_nav_fake", reliable)
        self.body_sub = self.create_subscription(
            Twist, "cmd_vel_nav_body", self._on_body, reliable
        )
        self.diag_sub = self.create_subscription(
            DiagnosticArray, "diagnostics", self._on_diagnostics, reliable
        )
        self.latest_body = None
        self.latest_diagnostics = {}
        self.create_timer(0.02, self._tick)

    # ---- 回调 ----
    def _on_body(self, msg):
        self.latest_body = msg

    def _on_diagnostics(self, msg):
        for status in msg.status:
            for entry in status.values:
                self.latest_diagnostics[entry.key] = entry.value

    def _tick(self):
        if self.publish_odom:
            odometry = Odometry()
            odometry.header.stamp = self.get_clock().now().to_msg()
            odometry.header.frame_id = "odom"
            odometry.child_frame_id = "base_link"
            odometry.pose.pose.orientation.z = math.sin(0.5 * self.yaw)
            odometry.pose.pose.orientation.w = math.cos(0.5 * self.yaw)
            self.odom_pub.publish(odometry)
        if self.publish_cmd:
            command = Twist()
            command.linear.x = INPUT_VX
            command.linear.y = INPUT_VY
            command.angular.z = INPUT_WZ
            self.cmd_pub.publish(command)

    # ---- 测试辅助 ----
    def spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_for_publisher(self, topic, timeout=10.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.count_publishers(topic) > 0:
                return True
            rclpy.spin_once(self, timeout_sec=0.05)
        return False


def near(lhs, rhs, tolerance=TOLERANCE):
    return abs(lhs - rhs) <= tolerance


def main():
    parser = argparse.ArgumentParser(description="fakeyaw 链路 P0 自检")
    parser.add_argument("--odom-timeout", type=float, default=0.5, help="被测节点的里程计超时（s）")
    parser.add_argument("--keep-node", action="store_true", help="退出时不杀掉被测节点")
    args = parser.parse_args()

    rclpy.init()
    checker = Checker(args.odom_timeout)
    node_process = subprocess.Popen(
        [
            "ros2", "run", "fake_vel_transform", "fake_vel_transform_node",
            "--ros-args",
            "-r", "__node:=p0_fake_vel_transform",
            "-p", "transform_mode:=nav_se2",
            "-p", "robot_base_frame:=base_link",
            "-p", "fake_robot_base_frame:=base_link_fake",
            "-p", "odom_topic:=odometry",
            "-p", "input_cmd_vel_topic:=cmd_vel_nav_fake",
            "-p", "output_cmd_vel_topic:=cmd_vel_nav_body",
            "-p", f"odom_timeout:={args.odom_timeout}",
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.STDOUT,
    )

    failures = []
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer, checker, spin_thread=False)

    def check(condition, message):
        print(("  [PASS] " if condition else "  [FAIL] ") + message)
        if not condition:
            failures.append(message)

    try:
        if not checker.wait_for_publisher("/cmd_vel_nav_body"):
            print("[错误] 被测节点没有发布 /cmd_vel_nav_body，无法继续。")
            return 1
        tf_listener  # noqa: B018 - 保持引用，避免被 GC

        print("--- 1) 无里程计：必须输出零速 ---")
        checker.publish_cmd = True
        checker.spin_for(0.5)
        body = checker.latest_body
        check(body is not None, "收到了 /cmd_vel_nav_body 输出")
        if body is not None:
            check(
                near(body.linear.x, 0.0) and near(body.linear.y, 0.0) and near(body.angular.z, 0.0),
                f"无里程计时输出零速 (vx={body.linear.x}, vy={body.linear.y}, wz={body.angular.z})",
            )

        print("--- 2) 四个基准 yaw 的速度方向与 wz 透传 ---")
        checker.publish_odom = True
        for name, yaw, qz, qw in YAW_CASES:
            checker.yaw = yaw
            # 等 yaw 稳定（节点只认最新一条里程计）。
            checker.spin_for(0.2)
            checker.latest_body = None
            checker.publish_cmd = True
            checker.spin_for(0.25)
            body = checker.latest_body
            if body is None:
                check(False, f"yaw={name}: 没有收到输出")
                continue
            expected_vx, expected_vy = EXPECTED[name]
            speed_in = math.hypot(INPUT_VX, INPUT_VY)
            speed_out = math.hypot(body.linear.x, body.linear.y)
            check(
                near(body.linear.x, expected_vx) and near(body.linear.y, expected_vy),
                f"yaw={name}: v_base=({body.linear.x:.9f}, {body.linear.y:.9f}) "
                f"期望 ({expected_vx}, {expected_vy})",
            )
            check(
                near(speed_out, speed_in),
                f"yaw={name}: 平移模长保持 ({speed_out:.9f} vs {speed_in:.9f})",
            )
            check(
                near(body.angular.z, INPUT_WZ),
                f"yaw={name}: wz 逐条透传 ({body.angular.z:.9f} vs {INPUT_WZ})",
            )

            print(f"--- 3) yaw={name}: TF base_link -> base_link_fake 的 yaw 必须是 -psi ---")
            try:
                transform = tf_buffer.lookup_transform("base_link", "base_link_fake", rclpy.time.Time())
                q = transform.transform.rotation
                tf_yaw = math.atan2(
                    2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                )
                delta = math.remainder(tf_yaw - (-yaw), 2.0 * math.pi)
                check(near(delta, 0.0, 1e-6), f"TF yaw={tf_yaw:.9f} 期望 {-yaw:.9f}")
                check(
                    near(transform.transform.translation.x, 0.0)
                    and near(transform.transform.translation.y, 0.0)
                    and near(transform.transform.translation.z, 0.0),
                    "TF 平移为零（fake 系与 base_link 原点重合）",
                )
            except Exception as error:  # noqa: BLE001 - 任何 TF 失败都算不通过
                check(False, f"TF 查询失败: {error}")

        print("--- 4) 里程计超时后必须重新输出零速 ---")
        checker.publish_odom = False
        checker.publish_cmd = True
        checker.spin_for(args.odom_timeout + 0.4)
        checker.latest_body = None
        checker.spin_for(0.3)
        body = checker.latest_body
        check(
            body is not None and near(body.linear.x, 0.0) and near(body.angular.z, 0.0),
            "里程计超时后输出零速"
            + ("" if body is None else f" (vx={body.linear.x}, wz={body.angular.z})"),
        )

        print("--- 5) 诊断计数 ---")
        checker.spin_for(1.0)
        diagnostics = checker.latest_diagnostics
        for key in (
            "mode", "yaw_valid", "input_count", "output_count", "zero_output_count",
            "no_odom_count", "stale_odom_count", "wz_error_max", "wz_error_mean",
        ):
            print(f"      {key:<22} {diagnostics.get(key, '?')}")
        check(diagnostics.get("mode") == "nav_se2", "诊断 mode=nav_se2")
        wz_error = float(diagnostics.get("wz_error_max", "1"))
        check(near(wz_error, 0.0), f"wz 透传误差上限 {wz_error} <= 1e-6")
        check(
            float(diagnostics.get("zero_output_count", "0")) > 0.0,
            "零速输出被计数（无里程计 / 超时路径确实生效）",
        )
    finally:
        if not args.keep_node:
            node_process.send_signal(signal.SIGINT)
            try:
                node_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                node_process.kill()
        tf_listener = None
        checker.destroy_node()
        rclpy.shutdown()

    print()
    if failures:
        print(f"=== P0 有 {len(failures)} 项不通过 ===")
        for message in failures:
            print(f"  - {message}")
        return 1
    print("=== P0 全部通过 ===")
    return 0


if __name__ == "__main__":
    sys.exit(main())
