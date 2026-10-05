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

"""速度链路监视器（实施方案 §8 的指标采集）。

同时订阅三路速度并按时间对齐写 CSV，保证记录里能区分：

  * 导航请求的速度   cmd_vel_nav       （nav_vx / nav_vy / nav_wz）
  * 合成后的执行命令 cmd_vel_sim       （cmd_vx / cmd_vy / cmd_wz）
  * 实际运动速度     odometry.twist    （act_vx / act_vy / act_wz）与位姿（x / y / yaw）

同时统计导航输入中**非零 angular.z 的次数**，作为"导航误发 wz"的话题级诊断
（方案 §9 的风险项之一）。

用法：
  ros2 run srm27_nav_bringup srm_velocity_monitor.py --ros-args \\
      -r __ns:=/red_standard_robot1 -p duration:=60 -p output:=/tmp/run.csv
"""

import csv
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class VelocityMonitor(Node):
    def __init__(self):
        super().__init__("srm_velocity_monitor")
        # 采样参数走 ROS 参数（不是 argparse），便于用 -p 从命令行或 launch 传入。
        # dynamic_typing: `-p duration:=11` 传进来的是 INTEGER，不做动态类型会被拒绝。
        numeric = ParameterDescriptor(dynamic_typing=True)
        self.duration = float(
            self.declare_parameter("duration", 0.0, numeric).value
        )
        self.output = str(self.declare_parameter("output", "").value)
        rate = float(self.declare_parameter("rate", 50.0, numeric).value)
        # 实际速度来源：导航侧相对里程计用 odometry；空场（无导航栈）用
        # chassis_odometry_gt。必须显式指定，不能在运行中回退，否则轨迹会在
        # 两套坐标之间跳变。
        self.actual_topic = str(
            self.declare_parameter("actual_topic", "odometry").value
        )
        self.robot_ns = self.get_namespace().strip("/")

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(Twist, "cmd_vel_nav", self._on_nav, qos)
        self.create_subscription(Twist, "cmd_vel_sim", self._on_cmd, qos)
        self.create_subscription(Twist, "rotation_velocity", self._on_rotation, qos)
        self.create_subscription(
            Odometry, self.actual_topic, self._on_odom, qos
        )

        self.nav = None
        self.cmd = None
        self.rotation = None
        self.odom = None
        self.rows = []
        self.marks = []
        self.period = 1.0 / max(rate, 1.0)
        # 运行期统计量
        self.nav_count = 0
        self.cmd_count = 0
        self.nav_wz_nonzero = 0

        self.create_timer(self.period, self._tick)

        # 供外部脚本触发一次阶段性标记（例如"开始叠加自转"）。
        self.create_service(Trigger, "~/mark", self._on_mark)

        self.get_logger().info(
            f"速度监视器已启动: 命名空间 /{self.robot_ns}, "
            f"采样 {rate:g} Hz, 时长 {self.duration:g}s, "
            f"实际速度来源 {self.actual_topic}, 输出 {self.output or '(未指定)'}"
        )

    # ------------------------------------------------------------------ 回调
    def _on_nav(self, msg):
        self.nav = msg
        if abs(msg.angular.z) > 1e-9:
            self.nav_wz_nonzero += 1
        self.nav_count += 1

    def _on_cmd(self, msg):
        self.cmd = msg
        self.cmd_count += 1

    def _on_rotation(self, msg):
        self.rotation = msg

    def _on_odom(self, msg):
        self.odom = msg

    def _on_mark(self, _request, response):
        self.marks.append(len(self.rows))
        response.success = True
        response.message = f"已标记第 {len(self.rows)} 个采样点"
        return response

    # ------------------------------------------------------------------ 采样
    def _tick(self):
        stamp = self.get_clock().now()
        row = {
            "t_sim": stamp.nanoseconds * 1e-9,
            "t_wall": time.time(),
            "nav_vx": self.nav.linear.x if self.nav else "",
            "nav_vy": self.nav.linear.y if self.nav else "",
            "nav_wz": self.nav.angular.z if self.nav else "",
            "rot_wz": self.rotation.angular.z if self.rotation else "",
            "cmd_vx": self.cmd.linear.x if self.cmd else "",
            "cmd_vy": self.cmd.linear.y if self.cmd else "",
            "cmd_wz": self.cmd.angular.z if self.cmd else "",
        }
        if self.odom is not None:
            pose = self.odom.pose.pose
            twist = self.odom.twist.twist
            row.update(
                {
                    "actual_src": self.actual_topic,
                    "x": pose.position.x,
                    "y": pose.position.y,
                    "yaw": yaw_from_quaternion(pose.orientation),
                    "act_vx": twist.linear.x,
                    "act_vy": twist.linear.y,
                    "act_wz": twist.angular.z,
                }
            )
        self.rows.append(row)

    # ------------------------------------------------------------------ 汇总
    def _column(self, name):
        values = [row[name] for row in self.rows if row.get(name) not in ("", None)]
        return [float(value) for value in values]

    def summary(self):
        lines = []
        lines.append(f"采样点数            : {len(self.rows)}")
        if getattr(self, "csv_mode", False):
            lines.append(f"有 nav 数据的采样点 : {self.nav_count}")
            lines.append(f"有 cmd 数据的采样点 : {self.cmd_count}")
        else:
            lines.append(f"cmd_vel_nav 消息数  : {self.nav_count}")
            lines.append(f"cmd_vel_sim 消息数  : {self.cmd_count}")
        lines.append(
            f"导航输入中非零 wz   : {self.nav_wz_nonzero} 次"
            + ("（符合 wz=0 约定）" if self.nav_wz_nonzero == 0 else "（异常，应恒为 0）")
        )

        for name, label in (
            ("cmd_wz", "cmd_vel_sim.wz"),
            ("rot_wz", "rotation_velocity.wz"),
            ("act_wz", "实际 wz"),
        ):
            values = self._column(name)
            if values:
                lines.append(
                    f"{label:<20}: max {max(values, key=abs):+.3f}  "
                    f"均值 {sum(values) / len(values):+.3f}"
                )

        # 平移只统计"确实下发了平移命令"的采样点：停止后的滑行段命令为 0，
        # 把它算进来会把命令均值拉低，得出"实际比命令大"的错误结论。
        def has_translation_command(row):
            if row.get("cmd_vx") in ("", None):
                return False
            return (
                abs(float(row["cmd_vx"])) > 1e-6
                or abs(float(row.get("cmd_vy") or 0.0)) > 1e-6
            )

        active = [row for row in self.rows if has_translation_command(row)]
        for axis in ("vx", "vy"):
            if not active:
                continue
            cmd = [float(row[f"cmd_{axis}"]) for row in active]
            act = [
                float(row[f"act_{axis}"])
                for row in active
                if row.get(f"act_{axis}") not in ("", None)
            ]
            n = min(len(cmd), len(act))
            if n == 0:
                continue
            err = [abs(cmd[i] - act[i]) for i in range(n)]
            lines.append(
                f"平移 {axis}（命令段 {n} 点）: 命令均值 "
                f"{sum(cmd[:n]) / n:+.3f} / 实际均值 {sum(act[:n]) / n:+.3f} m/s，"
                f"最大偏差 {max(err):.3f}"
            )

        # 命令结束后是否真的停下：只看"行尾连续无平移命令"的那一段，
        # 不能把开头的空闲采样也算进来，否则位移里混进了整段运动。
        def is_idle(row):
            return (
                row.get("cmd_vx") not in ("", None)
                and row.get("x") not in ("", None)
                and not has_translation_command(row)
            )

        idle = []
        for row in reversed(self.rows):
            if is_idle(row):
                idle.append(row)
            else:
                break
        idle.reverse()
        if idle:
            xs = [float(row["x"]) for row in idle if "x" in row]
            ys = [float(row["y"]) for row in idle if "y" in row]
            if len(xs) > 1:
                moved = math.hypot(xs[-1] - xs[0], ys[-1] - ys[0])
                tail = max(1, len(xs) // 10)
                tail_moved = math.hypot(
                    xs[-1] - xs[-1 - tail], ys[-1] - ys[-1 - tail]
                )
                lines.append(
                    f"命令结束后（尾段 {len(idle)} 点）: 位移 {moved:.3f} m，"
                    f"最后 {tail} 点位移 {tail_moved:.4f} m（应接近 0）"
                )

        sources = {row.get("actual_src") for row in self.rows if row.get("actual_src")}
        if sources:
            lines.append(f"实际速度来源        : {', '.join(sorted(sources))}")

        xs, ys = self._column("x"), self._column("y")
        if xs and ys:
            distance = sum(
                math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1])
                for i in range(1, len(xs))
            )
            lines.append(
                f"轨迹                : 起点 ({xs[0]:+.3f}, {ys[0]:+.3f}) → "
                f"终点 ({xs[-1]:+.3f}, {ys[-1]:+.3f})，路程 {distance:.3f} m"
            )

        yaws = self._column("yaw")
        if yaws:
            total = sum(
                wrap_angle(yaws[i] - yaws[i - 1]) for i in range(1, len(yaws))
            )
            lines.append(f"累计 yaw 变化       : {total:+.3f} rad")

        for index in self.marks:
            lines.append(f"标记点            : 第 {index} 个采样")
        return lines

    def write_csv(self, path):
        if not self.rows:
            return
        fields = []
        for row in self.rows:
            for key in row:
                if key not in fields:
                    fields.append(key)
        with open(path, "w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()
            writer.writerows(self.rows)


def summarize_csv(path):
    """从已保存的 CSV 重新生成汇总，便于在不重跑仿真的前提下统一统计口径。"""
    with open(path, "r", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        rows = []
        for raw in reader:
            row = {}
            for key, value in raw.items():
                if value in ("", None):
                    row[key] = ""
                    continue
                try:
                    row[key] = float(value)
                except ValueError:
                    row[key] = value
            rows.append(row)

    stub = VelocityMonitor.__new__(VelocityMonitor)
    stub.rows = rows
    stub.nav_count = sum(1 for row in rows if row.get("nav_vx") not in ("", None))
    stub.cmd_count = sum(1 for row in rows if row.get("cmd_vx") not in ("", None))
    stub.nav_wz_nonzero = sum(
        1
        for row in rows
        if row.get("nav_wz") not in ("", None) and abs(float(row["nav_wz"])) > 1e-9
    )
    stub.marks = []
    stub.csv_mode = True
    print("\n===== 速度链路汇总（由 CSV 重新生成） =====", flush=True)
    for line in stub.summary():
        print(line, flush=True)
    return 0


def main(argv=None):
    if argv is None:
        argv = sys.argv[1:]
    # 支持 `--csv <文件>`：离线重算汇总，不启动 ROS。
    if "--csv" in argv:
        index = argv.index("--csv")
        return summarize_csv(argv[index + 1])

    rclpy.init(args=argv)
    node = VelocityMonitor()
    try:
        deadline = (
            time.monotonic() + node.duration if node.duration > 0 else None
        )
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if deadline is not None and time.monotonic() >= deadline:
                break
    except KeyboardInterrupt:
        pass
    finally:
        print("\n===== 速度链路汇总 =====", flush=True)
        for line in node.summary():
            print(line, flush=True)
        if node.output:
            node.write_csv(node.output)
            print(f"CSV: {node.output}", flush=True)
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
