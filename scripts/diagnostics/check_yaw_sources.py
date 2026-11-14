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

"""对照两个 yaw 来源：导航用的里程计 vs Gazebo 真值（仿真的"下位机 yaw"）。

**为什么需要它**：实车上底盘 yaw 由下位机给、或者由 LIO 反推；仿真里没有下位机，
唯一可信的高频底盘姿态是 Gazebo 速度插件发的 `/chassis_odometry_gt`（真值）。
两者一比，就直接量出"整条 yaw 链路"的总误差：

    LIO 传感器 yaw  →(TF, 减去云台角)→  底盘 yaw  →  fake 系 → 导航

也就是说，这个差值同时覆盖了：LIO 精度、云台角是否正确、TF 时间对齐、
以及 fake_vel_transform 的 odom 新鲜度。**自转时最敏感**（转速 × 延迟）。

用法::

    # 静态看 10 s（先起好仿真与导航栈）
    ./script/check_yaw_sources.py

    # 一边自转一边比（会往 rotation_cmd 发 2.0 rad/s，需要 --yaw-mode spin）
    ./script/check_yaw_sources.py --spin 2.0 --duration 12

    # 换命名空间 / 放宽判据
    ROBOT_NS=xx ./script/check_yaw_sources.py
    ./script/check_yaw_sources.py --tol-deg 3.0

判据：两源在窗口内的**转角增量**差值（真值里程计与 LIO 里程计各自的 odom 原点不同，
所以只能比增量，不能比绝对值）。超过 `--tol-deg` 打印警告并给出最可能的病根。
"""

import argparse
import math
import os
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


def yaw_of(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class YawCollector:
    """累加一个话题的转角增量、位置漂移与角速度样本。"""

    def __init__(self):
        self.samples = 0
        self.first_position = None
        self.last_position = None
        self.path_length = 0.0
        self.travel = 0.0        # 累计|转角|（rad）
        self.signed_travel = 0.0  # 带符号累计转角（rad）
        self.wz = []
        self.last_yaw = None

    def add(self, quaternion, wz, position=None):
        if position is not None:
            if self.first_position is None:
                self.first_position = position
            elif self.last_position is not None:
                self.path_length += math.dist(self.last_position, position)
            self.last_position = position
        yaw = yaw_of(quaternion)
        if self.last_yaw is not None:
            delta = wrap(yaw - self.last_yaw)
            self.travel += abs(delta)
            self.signed_travel += delta
        self.last_yaw = yaw
        self.samples += 1
        self.wz.append(wz)

    def mean_wz(self):
        return sum(self.wz) / len(self.wz) if self.wz else 0.0

    def displacement(self):
        """窗口内首尾位移向量 (dx, dy)。"""
        if self.first_position is None or self.last_position is None:
            return None
        return (self.last_position[0] - self.first_position[0],
                self.last_position[1] - self.first_position[1])


class YawSourceChecker(Node):
    def __init__(self, namespace, spin_wz):
        super().__init__("yaw_source_checker")
        self.nav = YawCollector()
        self.truth = YawCollector()
        self.nav_stamps = 0
        self.truth_stamps = 0

        qos = QoSProfile(
            depth=50,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(
            Odometry, f"/{namespace}/odometry", self._on_nav, qos
        )
        self.create_subscription(
            Odometry, f"/{namespace}/chassis_odometry_gt", self._on_truth, qos
        )
        self._stop = threading.Event()
        if spin_wz:
            self.spin_wz = spin_wz
            self.spin_pub = self.create_publisher(
                Twist, f"/{namespace}/rotation_cmd", 10
            )
            threading.Thread(target=self._spin_loop, daemon=True).start()
        else:
            self.spin_wz = 0.0
            self.spin_pub = None

    def _on_nav(self, message):
        self.nav_stamps += 1
        pos = message.pose.pose.position
        self.nav.add(message.pose.pose.orientation, message.twist.twist.angular.z, (pos.x, pos.y))

    def _on_truth(self, message):
        self.truth_stamps += 1
        pos = message.pose.pose.position
        self.truth.add(message.pose.pose.orientation, message.twist.twist.angular.z, (pos.x, pos.y))

    def _spin_loop(self):
        message = Twist()
        message.angular.z = self.spin_wz
        while not self._stop.is_set():
            if self.spin_pub is not None:
                self.spin_pub.publish(message)
            time.sleep(0.05)

    def stop(self):
        self._stop.set()


def main():
    parser = argparse.ArgumentParser(description="对照两个 yaw 来源")
    parser.add_argument(
        "--ns",
        default=os.environ.get("ROBOT_NS", "red_standard_robot1"),
        help="机器人命名空间（默认 red_standard_robot1）",
    )
    parser.add_argument("--duration", type=float, default=10.0, help="采样窗口（秒）")
    parser.add_argument(
        "--spin",
        type=float,
        default=0.0,
        help="窗口内往 rotation_cmd 发该角速度（rad/s，0=不发；需要 --yaw-mode spin）",
    )
    parser.add_argument(
        "--tol-deg",
        type=float,
        default=2.0,
        help="两源转角增量差值的容差（度），超过就报可疑",
    )
    parser.add_argument(
        "--tol-m",
        type=float,
        default=0.5,
        help="两源位移差值的容差（米）；超过说明里程计漂移（跑飞的直接指标）",
    )
    args = parser.parse_args()

    rclpy.init()
    node = YawSourceChecker(args.ns, args.spin)
    print(f"采样 {args.duration:.1f} s（命名空间 /{args.ns}"
          f"{'，同时发 rotation_cmd=' + str(args.spin) + ' rad/s' if args.spin else ''}）...")
    deadline = time.time() + args.duration
    try:
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.stop()
        time.sleep(0.2)
        try:
            rclpy.spin_once(node, timeout_sec=0.1)
        except Exception:  # noqa: BLE001 - 收尾期间的 QoS 中断无所谓
            pass

    ok = True
    print()
    print("== 采样 ==")
    print(f"  /{args.ns}/odometry               {node.nav_stamps:5d} 帧"
          f"   (导航用的底盘里程计)")
    print(f"  /{args.ns}/chassis_odometry_gt    {node.truth_stamps:5d} 帧"
          f"   (Gazebo 真值 = 仿真的\"下位机 yaw\")")

    if node.nav.samples < 5:
        print("\n[错误] 导航里程计样本太少：栈没起，或命名空间不对。")
        rclpy.shutdown()
        return 1
    if node.truth_stamps < 5:
        print("\n[提示] 没有 /chassis_odometry_gt：")
        print("       仿真里应有（ros_gz_bridge 一直桥接）；实车上没有这个真值源，")
        print("       那时只能拿最小转弯半径/地面标记做对照。")
        print("\n== 导航里程计自身 ==")
        print(f"  转角增量 {math.degrees(node.nav.signed_travel):+.2f}°"
              f" | 累计 {math.degrees(node.nav.travel):.2f}°"
              f" | wz 均值 {node.nav.mean_wz():+.3f} rad/s")
        rclpy.shutdown()
        return 0

    nav_delta = math.degrees(node.nav.signed_travel)
    truth_delta = math.degrees(node.truth.signed_travel)
    diff = nav_delta - truth_delta
    print()
    print("== 转角增量对照（同窗口，正值=逆时针）==")
    print(f"  导航里程计  : {nav_delta:+8.2f}°   wz 均值 {node.nav.mean_wz():+.3f} rad/s")
    print(f"  真值(下位机): {truth_delta:+8.2f}°   wz 均值 {node.truth.mean_wz():+.3f} rad/s")
    print(f"  差值        : {diff:+8.2f}°   （容差 {args.tol_deg:.1f}°）")

    nav_disp = node.nav.displacement()
    truth_disp = node.truth.displacement()
    if nav_disp is not None and truth_disp is not None:
        disp_err = math.dist(nav_disp, truth_disp)
        print()
        print("== 位移对照（同窗口；里程计漂移 = 两源位移向量之差）==")
        print(f"  导航里程计  : Δ=({nav_disp[0]:+.2f}, {nav_disp[1]:+.2f}) m"
              f"   路径长 {node.nav.path_length:.2f} m")
        print(f"  真值(下位机): Δ=({truth_disp[0]:+.2f}, {truth_disp[1]:+.2f}) m"
              f"   路径长 {node.truth.path_length:.2f} m")
        print(f"  漂移误差    : {disp_err:.2f} m   （容差 {args.tol_m:.2f} m）")
        if disp_err > args.tol_m:
            ok = False
            print("  [可疑] 里程计漂移超容差 —— 导航会跑飞。空场世界(srm_empty)只有地面,")
            print("         点云在 x/y 上不可观, LIO 只能靠 IMU 推算, 必然漂; 换有墙/障碍的世界,")
            print("         或改用 --no-lio(真值) / --gicp(先验图重定位)。")

    if abs(diff) > args.tol_deg:
        ok = False
        print("\n[可疑] 两源转角不一致。按可能性排查：")
        print("  1) 云台角没进 TF / 不新鲜: 雷达在云台上，底盘 yaw = LIO 传感器 yaw − 云台角；")
        print("     云台关节是 fixed 时该角恒为 0（此时本条不适用），一旦放成 revolute")
        print("     就必须有 ≥100 Hz 的 /joint_states，否则底盘 yaw 直接差一个云台角。")
        print("  2) LIO 精度/时间戳: 看 point_lio 是否报 'IMU/雷达时间不同步'，")
        print("     以及 time_diff_lidar_to_imu 是否需要标定。")
        print("  3) fake_vel_transform 的 odom 太旧: 自转时表现为按转速放大的角度偏差；")
        print("     查它的诊断 stale_odom_count / last_odom_age。")
        print("  4) 静态就不一致（不自转也差很多）: 外参/重力装错，先核对 gravity 与 lidar_rpy。")
    else:
        print("\n[通过] 两源转角在容差内一致 —— yaw 链路（含云台角与 TF 时间对齐）可信。")

    if args.spin:
        print(
            f"\n提示: 本次同时发了 rotation_cmd={args.spin} rad/s；"
            f"若两源都不转，先确认栈是以 --yaw-mode spin 启动、"
            f"且 FollowPath.yaw_policy.mode=spin。"
        )

    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
