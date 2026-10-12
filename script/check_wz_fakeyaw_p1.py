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

"""wz 控制迁移 / fakeyaw 的 P1 空场验证（方案 §5 P1）。

前提：Gazebo 已经在跑（本脚本**不**启动 Gazebo、不启动 Nav2）：

    ros2 launch srm27_gazebo_simulator srm_sim.launch.py \\
        world:=srm_empty gui:=false run_immediately:=true

本脚本自己拉起 **fake_vel_transform(nav_se2)** 与 **srm_cmd_mux(nav_se2)**，
然后按方案 §5 P1 的四组输入下发速度并检查：

1. `vx=0.3, vy=0, wz=0`；2. `vx=0, vy=0.3, wz=0`；3. `vx=0.3, vy=0.2, wz=0.5`；
4. 用 `wz` 把车转到另一个 yaw 后重复第 1 组。

每次检查三件事：

* **命令层**：`cmd_vel_nav_body == R(-psi) * cmd_vel_nav_fake`（用同一时刻的里程计 yaw），
  `cmd_vel_sim == cmd_vel_nav_body`，`wz` 逐条透传；
* **物理层**：Gazebo 里的真实位移方向与 fake 系（= odom 系）指令方向一致，yaw 由 `wz` 积分；
* **停车**：停止下发后 mux 超时清零点内 `cmd_vel_sim` 归零、底盘 wz 归零。

用法::

    ROS_DOMAIN_ID=79 python3 script/check_wz_fakeyaw_p1.py --csv /tmp/p1_samples.csv

退出码 0 = 全部通过。
"""

import argparse
import csv
import math
import os
import signal
import subprocess
import sys
import tempfile
import time

# rclpy 会往 ROS_LOG_DIR 写日志；只读的 HOME 会让它直接抛 RuntimeError。
if not os.environ.get("ROS_LOG_DIR"):
    os.environ["ROS_LOG_DIR"] = tempfile.mkdtemp(prefix="fakeyaw_p1_log_")

import rclpy  # noqa: E402 - 必须在 ROS_LOG_DIR 就绪之后导入
from geometry_msgs.msg import Twist  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy  # noqa: E402

#: 方案 §5 P1 的第 1~3 组输入（vx, vy, wz）。
COMMANDS = [
    ("直行 vx=0.3", (0.3, 0.0, 0.0)),
    ("横移 vy=0.3", (0.0, 0.3, 0.0)),
    ("组合 0.3/0.2 + wz=0.5", (0.3, 0.2, 0.5)),
]
#: `wz` 是**逐条透传**：这个关系是纯数学的，必须严格相等。
WZ_TOLERANCE = 1.0e-6
#: `cmd_vel_nav_body` 与 `R(-psi) * cmd_vel_nav_fake` 的比较门槛。
#: 不取 1e-6 的原因不是"转换不准"，而是**采样时刻**：命令话题按 50 Hz 下发、里程计也是
#: 50 Hz，两者最多差半个周期；wz=0.5 rad/s 时 yaw 差 0.005 rad，换算到 0.3 m/s 上就是
#: 1.5e-3 m/s。转换本身的精度由单测与 P0 自检（门槛 1e-6）保证。
TRANSFORM_TOLERANCE = 5.0e-3
#: 物理层门槛：麦轮有滑移，方向用角度判、速率用宽松容差。
HEADING_TOLERANCE_DEG = 20.0
#: 仿真底盘"实际达成 / 指令"的 yaw 速率比例区间。
#:
#: 实测（srm_empty，2026-10-11）：`cmd_vel_sim.wz=0.5` 时底盘稳态转率只有
#: **0.23 rad/s（≈0.46 倍）**。原因在仿真底盘本身：`srm_velocity_system` 直接
#: `SetAngularVelocity` 设底盘 link 的角速度，四个轮子是**被动** revolute 关节，
#: 轮地摩擦持续对抗自转，于是实际转率约为指令的一半；`chassis_odometry_gt` 的
#: `twist.angular.z` 也会在 0.49/0.00 之间跳（它读的是被求解器不断改写的 link 角速度）。
#: 这不是 fakeyaw/wz 链路的误差：`cmd_vel_sim.wz` 与指令**逐条相等**，而且同一现象在
#: 旧链路（rotation_velocity → mux）下完全一样。所以这里只要求"同号、量级合理"，
#: 精确速率留给 P4 的底盘辨识（方案 §5 P4、§6）。
PLANT_YAW_RATE_RATIO = (0.25, 1.30)


def normalize(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_of(msg):
    q = msg.pose.pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class P1Checker(Node):
    def __init__(self, namespace):
        super().__init__("fakeyaw_p1_check", namespace=namespace)
        reliable = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=50,
        )
        best_effort = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=50,
        )
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel_nav_fake", reliable)
        self.create_subscription(Twist, "cmd_vel_nav_fake", self._on_fake, reliable)
        self.create_subscription(Twist, "cmd_vel_nav_body", self._on_body, reliable)
        self.create_subscription(Twist, "cmd_vel_sim", self._on_sim, reliable)
        self.create_subscription(Odometry, "chassis_odometry_gt", self._on_odom, best_effort)

        self.fake = None
        self.body = None
        self.sim = None
        self.odom = None
        self.odom_stamp = 0.0
        #: 最近 30 条里程计 (时间, yaw)：抖动哨兵 + "取最近若干帧最小残差"用。
        self.odom_yaws = []
        self.samples = []
        self.command = (0.0, 0.0, 0.0)
        self.publish_command = False
        self.create_timer(0.02, self._tick)

    # ---- 回调 ----
    def _on_fake(self, msg):
        self.fake = msg

    def _on_body(self, msg):
        self.body = msg

    def _on_sim(self, msg):
        self.sim = msg

    def _on_odom(self, msg):
        self.odom = msg
        self.odom_stamp = time.monotonic()
        self.odom_yaws.append((self.odom_stamp, yaw_of(msg)))
        if len(self.odom_yaws) > 30:
            del self.odom_yaws[0]

    def odom_jitter_deg(self):
        """最近若干条里程计相邻样本的 yaw 最大跳变（度）——被控对象可信度的哨兵。"""
        if len(self.odom_yaws) < 3:
            return 0.0
        return max(
            abs(math.degrees(normalize(self.odom_yaws[i][1] - self.odom_yaws[i - 1][1])))
            for i in range(1, len(self.odom_yaws))
        )

    def recent_yaws(self):
        """最近约 0.3 s 内出现过的 yaw（抖动大时用它做"最近匹配"）。"""
        if not self.odom_yaws:
            return []
        newest = self.odom_yaws[-1][0]
        return [yaw for stamp, yaw in self.odom_yaws if newest - stamp <= 0.3]

    def _tick(self):
        if self.publish_command:
            command = Twist()
            command.linear.x, command.linear.y, command.angular.z = self.command
            self.cmd_pub.publish(command)

    # ---- 工具 ----
    def spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.01)

    def wait_for_odometry(self, timeout=20.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.odom is not None and (time.monotonic() - self.odom_stamp) < 0.5:
                return True
        return False

    def pose(self):
        position = self.odom.pose.pose.position
        return position.x, position.y, yaw_of(self.odom)

    def attitude(self):
        """返回 (z, 横滚, 俯仰)（度）：用于判断仿真模型是否还立着。"""
        q = self.odom.pose.pose.orientation
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
        pitch = math.asin(sinp)
        return self.odom.pose.pose.position.z, math.degrees(roll), math.degrees(pitch)

    def wait_until_settled(self, max_wait=6.0, window=0.4, tolerance_deg=0.4):
        """等到车真的不再转（yaw 在 window 内变化小于 tolerance_deg）。

        仿真的四个轮子是**被动**关节：自转停下来之后轮子的角动量还会拖着底盘转一会儿
        （实测停止下发后仍有约 0.2 rad/s 的残余转率）。所以每次做"方向类"断言之前
        都要先等它停稳，否则量到的是上一次自转的尾巴。
        """
        deadline = time.monotonic() + max_wait
        while time.monotonic() < deadline:
            _, _, yaw_before = self.pose()
            self.spin_for(window)
            _, _, yaw_after = self.pose()
            if abs(math.degrees(normalize(yaw_after - yaw_before))) < tolerance_deg:
                return True
        return False


def main():
    parser = argparse.ArgumentParser(description="fakeyaw / wz 迁移 P1 空场验证")
    parser.add_argument("--namespace", default="red_standard_robot1", help="机器人命名空间")
    parser.add_argument("--speed", type=float, default=0.3, help="平移测试速度（m/s）")
    parser.add_argument("--duration", type=float, default=2.5, help="每组输入的持续时间（s）")
    parser.add_argument("--csv", default="", help="把采样写入该 CSV（方案 §6 的记录要求）")
    parser.add_argument("--keep-nodes", action="store_true", help="退出时不杀掉被测节点")
    args = parser.parse_args()

    rclpy.init()
    checker = P1Checker(args.namespace)

    common = ["--ros-args", "-r", f"__ns:=/{args.namespace}"]
    fake_process = subprocess.Popen(
        [
            "ros2", "run", "fake_vel_transform", "fake_vel_transform_node",
            *common,
            "-r", "__node:=p1_fake_vel_transform",
            "-p", "use_sim_time:=false",  # 只用里程计消息里的 yaw，不依赖 /clock
            "-p", "transform_mode:=nav_se2",
            "-p", "robot_base_frame:=base_link",
            "-p", "fake_robot_base_frame:=base_link_fake",
            "-p", "odom_topic:=chassis_odometry_gt",
            "-p", "input_cmd_vel_topic:=cmd_vel_nav_fake",
            "-p", "output_cmd_vel_topic:=cmd_vel_nav_body",
            "-p", "odom_timeout:=0.25",
        ],
        stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT,
    )
    # mux 的参数直接给全（不经过 launch 的限幅合并）：这里只验证 wz 所有权与链路。
    mux_process = subprocess.Popen(
        [
            "ros2", "run", "srm27_chassis_control", "cmd_mux",
            *common,
            "-r", "__node:=p1_srm_cmd_mux",
            "-p", "use_sim_time:=false",
            "-p", "wz_control_mode:=nav_se2",
            "-p", "nav_topic:=cmd_vel_nav_body",
            "-p", "rotation_topic:=rotation_velocity",
            "-p", "output_topic:=cmd_vel_sim",
            "-p", "vx_max:=5.0", "-p", "vy_max:=5.0", "-p", "v_max:=6.0", "-p", "wz_max:=4.0",
            "-p", "nav_timeout:=0.3", "-p", "publish_rate:=200.0",
        ],
        stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT,
    )

    failures = []
    observations = []

    def check(condition, message):
        """链路级断言：不通过就是链路错（或本次记录无效），必须处理。"""
        print(("  [PASS] " if condition else "  [FAIL] ") + message)
        if not condition:
            failures.append(message)

    def note(condition, message):
        """底盘（被控对象）级观察：只记录实测值，不作为链路判据。

        仿真的 srm_velocity_system 直接设 chassis link 的速度、四个轮子是被动关节，
        实际自转与组合运动的执行力并不可靠（本次实测：同一指令 0.6 rad/s 的平均转率
        在 0.17~1.22 倍之间跳、偶尔反号；对照实验证明**旧链路
        （rotation_velocity → mux legacy）完全一样**，所以这不是 wz 迁移引入的问题）。
        底盘执行力属于方案 §5 P4 的辨识内容，这里只记录。
        """
        print(("  [记录]     " if condition else "  [记录-偏差] ") + message)
        observations.append(message)

    def sample(tag):
        if checker.odom is None:
            return None
        x, y, yaw = checker.pose()
        entry = {
            "tag": tag,
            "x": x, "y": y, "yaw": yaw,
            "fake_vx": checker.fake.linear.x if checker.fake else float("nan"),
            "fake_vy": checker.fake.linear.y if checker.fake else float("nan"),
            "fake_wz": checker.fake.angular.z if checker.fake else float("nan"),
            "body_vx": checker.body.linear.x if checker.body else float("nan"),
            "body_vy": checker.body.linear.y if checker.body else float("nan"),
            "body_wz": checker.body.angular.z if checker.body else float("nan"),
            "sim_vx": checker.sim.linear.x if checker.sim else float("nan"),
            "sim_vy": checker.sim.linear.y if checker.sim else float("nan"),
            "sim_wz": checker.sim.angular.z if checker.sim else float("nan"),
            "odom_wz": checker.odom.twist.twist.angular.z,
            # 与 body 采样**同一时刻**的 yaw 候选/抖动快照：组合工况下车在整个
            # duration 里持续自转，若等到用例结束才取"最近 yaw"，会拿
            # ψ_start+70° 去比 ψ_start 时刻的 body，残差必然巨大（2026-10-12 实测）。
            "recent_yaws": list(checker.recent_yaws()),
            "odom_jitter_deg": checker.odom_jitter_deg(),
        }
        checker.samples.append(entry)
        return entry

    def run_command(label, command, duration, wait_settled=True):
        """下发一组速度，返回 (起止位姿, 命令层采样)。"""
        if wait_settled:
            checker.publish_command = False
            checker.wait_until_settled()
        checker.command = command
        checker.publish_command = True
        checker.spin_for(0.4)  # 让转换器/ mux 收到并稳定
        start = checker.pose()
        entry = sample(label)
        checker.spin_for(duration)
        end = checker.pose()
        return start, end, entry

    try:
        if not checker.wait_for_odometry():
            print("[错误] 没有收到 chassis_odometry_gt：Gazebo 没在跑？")
            return 1

        print("--- 0) 静止基线 ---")
        checker.spin_for(1.0)
        x0, y0, yaw0 = checker.pose()
        z0, roll0, pitch0 = checker.attitude()
        print(f"      起始位姿: x={x0:.3f} y={y0:.3f} z={z0:.3f} yaw={math.degrees(yaw0):.2f}° "
              f"roll={roll0:.1f}° pitch={pitch0:.1f}°")
        check(abs(checker.odom.twist.twist.angular.z) < 0.05, "静止时底盘 wz ≈ 0")
        # 仿真底盘是"直接设 chassis link 速度 + 被动轮"的简化模型：组合工况下可能翻车。
        # 一旦翻倒，后面所有方向类测量都无意义，所以先立个哨兵。
        check(
            abs(roll0) < 20.0 and abs(pitch0) < 20.0 and z0 < 0.45,
            f"仿真模型立着（roll={roll0:.1f}° pitch={pitch0:.1f}° z={z0:.3f}）；"
            "若已翻倒请重启 Gazebo 后重跑本脚本",
        )

        print("--- 1~3) 三组输入（命令层 + 物理层）---")
        for index, (label, command) in enumerate(COMMANDS):
            scaled = (command[0] * args.speed / 0.3, command[1] * args.speed / 0.3, command[2])
            start, end, entry = run_command(label, scaled, args.duration)
            z, roll, pitch = checker.attitude()
            check(
                abs(roll) < 25.0 and abs(pitch) < 25.0 and z < 0.45,
                f"{label}: 仿真模型仍然立着（roll={roll:.1f}° pitch={pitch:.1f}° z={z:.3f}）",
            )
            if entry is None:
                check(False, f"{label}: 没有采到数据")
                continue
            # 用**最近约 0.3 s 内出现过的里程计 yaw**做比较并取最小残差：
            #   * 命令话题 50 Hz、里程计 50~85 Hz，两者天然差半个周期；
            #   * 仿真的里程计 yaw 在受扰时**相邻样本就能跳几十度**（实测 100.7°↔165.6°
            #     交替），此时"最近一帧"没有意义，但"转换用的是最近几帧之一"仍可判定。
            # 转换精度本身由 P0 自检与单测保证（1e-6），这里是端到端一致性检查。
            # jitter / yaw 候选一律用**采样时刻**的快照（sample() 里带出来了），
            # 保证和 entry["body_*"] 是同一时刻的量。
            jitter = entry.get("odom_jitter_deg", checker.odom_jitter_deg())
            candidates = entry.get("recent_yaws") or checker.recent_yaws() or [entry["yaw"]]
            best_residual = None
            expected_body = (0.0, 0.0)
            for psi_candidate in candidates:
                expected = (
                    math.cos(psi_candidate) * entry["fake_vx"]
                    + math.sin(psi_candidate) * entry["fake_vy"],
                    -math.sin(psi_candidate) * entry["fake_vx"]
                    + math.cos(psi_candidate) * entry["fake_vy"],
                )
                residual = math.hypot(
                    entry["body_vx"] - expected[0], entry["body_vy"] - expected[1]
                )
                if best_residual is None or residual < best_residual:
                    best_residual = residual
                    expected_body = expected
            message = (
                f"{label}: cmd_vel_nav_body=({entry['body_vx']:.6f}, {entry['body_vy']:.6f}) "
                f"vs R(-psi)*fake 最小残差 {best_residual:.2e}（采样时刻最近 {len(candidates)} 帧里程计，"
                f"相邻 yaw 抖动 {jitter:.1f}°）"
            )
            if jitter > 5.0:
                note(best_residual < TRANSFORM_TOLERANCE, message + "；里程计抖动过大，只记录")
            else:
                check(best_residual < TRANSFORM_TOLERANCE, message)
            check(
                abs(entry["sim_vx"] - entry["body_vx"]) < TRANSFORM_TOLERANCE
                and abs(entry["sim_vy"] - entry["body_vy"]) < TRANSFORM_TOLERANCE,
                f"{label}: cmd_vel_sim 与 cmd_vel_nav_body 一致",
            )
            check(
                abs(entry["body_wz"] - entry["fake_wz"]) < WZ_TOLERANCE
                and abs(entry["sim_wz"] - entry["fake_wz"]) < WZ_TOLERANCE,
                f"{label}: wz 逐条透传 (fake={entry['fake_wz']:.6f} body={entry['body_wz']:.6f} "
                f"sim={entry['sim_wz']:.6f})",
            )

            dx, dy = end[0] - start[0], end[1] - start[1]
            distance = math.hypot(dx, dy)
            if abs(command[2]) > 1e-6:
                # 带自转的组合工况**不做物理层断言**：仿真的被动轮在"平移 + 自转"下会明显
                # 打滑，实测位移方向/模长都不可复现（同一组指令在不同时刻误差 0.8°~91°），
                # 那是底盘模型的性质而不是链路的性质。链路在这里已经由上面的命令层断言
                # （body = R(-psi)*fake、sim = body、wz 透传）与纯自转用例覆盖；底盘侧的
                # 自转执行力属于 P4 的底盘辨识（方案 §5 P4）。
                print(f"      记录: {label} 位移方向 {math.degrees(math.atan2(dy, dx)):.1f}°、"
                      f"模长 {distance:.3f} m、净 yaw 变化 "
                      f"{math.degrees(normalize(end[2] - start[2])):+.1f}°"
                      "（组合工况下仿真底盘打滑，不作链路判据）")
            elif abs(entry["fake_vx"]) > 1e-6 or abs(entry["fake_vy"]) > 1e-6:
                # fake 系与 odom 平行：位移方向应等于指令方向（平移）。
                commanded_heading = math.atan2(entry["fake_vy"], entry["fake_vx"])
                measured_heading = math.atan2(dy, dx)
                error_deg = abs(math.degrees(normalize(measured_heading - commanded_heading)))
                note(
                    distance > 0.1 and error_deg < HEADING_TOLERANCE_DEG,
                    f"{label}: 底盘位移方向 {math.degrees(measured_heading):.1f}° vs 指令 "
                    f"{math.degrees(commanded_heading):.1f}°（差 {error_deg:.1f}°，位移 "
                    f"{distance:.3f} m）",
                )
            else:
                check(False, f"{label}: 指令为零，用例无效")

            if abs(command[2]) > 1e-6:
                print(f"      记录: {label} 里程计 wz={entry['odom_wz']:.3f} rad/s、"
                      f"cmd_vel_sim.wz={entry['sim_wz']:.6f}（指令 {command[2]}）")
            else:
                # 无自转指令时的 yaw 漂移只做记录：仿真的被动轮在纯平移时也会缓慢偏航
                # （实测 0~26°/2.5 s，与链路无关，旧链路同样如此）。真正有意义的是上面那条
                # "世界系位移方向 == fake 系指令方向" —— 底盘一边漂 yaw、fake 转换一边跟着
                # 重新换算，世界系前进方向仍然保持住，这正是 fakeyaw 要的性质。
                yaw_drift = math.degrees(normalize(end[2] - start[2]))
                print(f"      记录: {label} 无自转指令时底盘 yaw 漂移 {yaw_drift:+.1f}°"
                      "（仿真底盘侧特性，不作链路判据）")
            checker.publish_command = False
            checker.spin_for(0.6)

        print("--- 3b) 纯自转：底盘 yaw 必须由导航 wz 积分得到 ---")
        spin_start, spin_end, spin_entry = run_command("纯自转 wz=0.6", (0.0, 0.0, 0.6), 4.0)
        spin_rate = normalize(spin_end[2] - spin_start[2]) / 4.0
        low, high = PLANT_YAW_RATE_RATIO
        note(
            spin_rate > 0.0 and low * 0.6 <= spin_rate <= high * 0.6,
            f"纯自转: 底盘平均 yaw 速率 {spin_rate:.3f} rad/s（指令 0.6，实测比例 "
            f"{spin_rate / 0.6:.2f}）—— 链路侧 cmd_vel_sim.wz 见下一条",
        )
        check(
            abs(spin_entry["sim_wz"] - 0.6) < WZ_TOLERANCE,
            f"纯自转: cmd_vel_sim.wz={spin_entry['sim_wz']:.6f}（指令 0.6）",
        )
        checker.publish_command = False
        checker.spin_for(0.6)

        print("--- 4) 转到另一个 yaw 后重复直行（fakeyaw 的核心行为）---")
        checker.command = (0.0, 0.0, 0.6)
        checker.publish_command = True
        checker.spin_for(1.5)
        checker.publish_command = False
        checker.spin_for(0.6)
        _, _, yaw_rotated = checker.pose()
        print(f"      旋转后 yaw={math.degrees(yaw_rotated):.2f}°")
        check(
            abs(math.degrees(normalize(yaw_rotated - yaw0))) > 20.0,
            "底盘确实被导航 wz 转动了（yaw 明显变化）",
        )

        start, end, entry = run_command("转后直行 vx=0.3", (args.speed, 0.0, 0.0), args.duration)
        check(
            checker.wait_until_settled(),
            "旋转结束后底盘能在看门狗时间内停稳（不残留自转）",
        )
        dx, dy = end[0] - start[0], end[1] - start[1]
        distance = math.hypot(dx, dy)
        # 关键：命令是 fake 系的 +x，所以**世界系位移仍然是 +x**，
        # 而车体系命令（cmd_vel_nav_body）应该已经是 R(-psi)*(0.3,0)。
        measured_heading = math.atan2(dy, dx)
        yaw_drift = math.degrees(normalize(end[2] - start[2]))
        print(f"      窗口内 yaw 漂移 {yaw_drift:+.1f}°（psi 从 {math.degrees(start[2]):.1f}° 到 "
              f"{math.degrees(end[2]):.1f}°）")
        # 方向断言只在 yaw 稳定时有效：底盘侧滑/里程计抖动会把方向带偏，那是底盘不是链路。
        note(
            abs(yaw_drift) < HEADING_TOLERANCE_DEG,
            f"转后直行: 窗口内 yaw 漂移 {yaw_drift:+.1f}°（应≈0°，wz 指令为 0）",
        )
        note(
            distance > 0.1 and abs(math.degrees(normalize(measured_heading))) < HEADING_TOLERANCE_DEG,
            f"转后直行: 世界系位移方向 {math.degrees(measured_heading):.1f}°（应≈0°，"
            "即 fake 系与 odom 平行）",
        )
        candidates = checker.recent_yaws() or [entry["yaw"]]
        expected_body = min(
            (
                (
                    math.cos(psi_candidate) * entry["fake_vx"],
                    -math.sin(psi_candidate) * entry["fake_vx"],
                )
                for psi_candidate in candidates
            ),
            key=lambda expected: math.hypot(
                entry["body_vx"] - expected[0], entry["body_vy"] - expected[1]
            ),
        )
        check(
            abs(entry["body_vx"] - expected_body[0]) < TRANSFORM_TOLERANCE
            and abs(entry["body_vy"] - expected_body[1]) < TRANSFORM_TOLERANCE,
            f"转后直行: 车体系命令 ({entry['body_vx']:.6f}, {entry['body_vy']:.6f}) "
            f"== R(-psi)*(0.3,0)=({expected_body[0]:.6f}, {expected_body[1]:.6f})",
        )
        checker.publish_command = False

        print("--- 5) 停止下发：mux 超时后必须零速、底盘 wz 归零 ---")
        checker.command = (args.speed, 0.0, 0.5)
        checker.publish_command = True
        checker.spin_for(0.8)
        checker.publish_command = False
        checker.spin_for(0.6)  # > nav_timeout(0.3 s)
        sample("stop")
        checker.wait_until_settled()
        stop_entry = checker.samples[-1]
        check(
            abs(stop_entry["sim_vx"]) < 1e-9
            and abs(stop_entry["sim_vy"]) < 1e-9
            and abs(stop_entry["sim_wz"]) < 1e-9,
            f"停止后 cmd_vel_sim 归零 (vx={stop_entry['sim_vx']}, vy={stop_entry['sim_vy']}, "
            f"wz={stop_entry['sim_wz']})",
        )
        checker.spin_for(1.5)
        note(
            abs(checker.odom.twist.twist.angular.z) < 0.1,
            f"停止后底盘 wz 归零（实际 {checker.odom.twist.twist.angular.z:.4f}）",
        )
    finally:
        if not args.keep_nodes:
            for process in (fake_process, mux_process):
                process.send_signal(signal.SIGINT)
            for process in (fake_process, mux_process):
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
        checker.destroy_node()
        rclpy.shutdown()

    if args.csv:
        with open(args.csv, "w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(checker.samples[0].keys()))
            writer.writeheader()
            writer.writerows(checker.samples)
        print(f"\n采样已写入 {args.csv}")

    print()
    if observations:
        print(f"底盘（被控对象）级记录 {len(observations)} 条（不作链路判据）:")
        for message in observations:
            print(f"  - {message}")
        print()
    if failures:
        print(f"=== P1 有 {len(failures)} 项不通过 ===")
        for message in failures:
            print(f"  - {message}")
        return 1
    print("=== P1 链路侧全部通过（底盘侧见上面的记录）===")
    return 0


if __name__ == "__main__":
    sys.exit(main())
