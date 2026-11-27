#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""云台旋转链路复检：用**刚性不变**的口径判断桥接到底有没有"吃掉旋转"。

为什么要另写一个工具
====================
`diag_gimbal_chain.py` 用"累计 yaw"当转角：

    net(seq) = Σ wrap(yaw_{i+1} − yaw_i)

这对**原始**位姿成立（ZLIO 的世界系是重力对齐的，雷达绕世界竖直轴转 ψ，yaw 就线性
跟着转）。但对**桥接输出**不成立：桥接做的是

    pose_out(k) = T_off · R_k ,   T_off = ground · M · R_0⁻¹   （M = gimbal←livox 安装外参）

它是**刚性左乘**。刚性左乘把整条轨迹共轭到另一个参考系：绕轴 n 转 ψ 变成绕轴
(T_off·n) 转 ψ，**转角 ψ 本身一位不变**。所以

    angle(R_k⁻¹ R_{k+1})  ==  angle(R_out(k)⁻¹ R_out(k+1))        （恒等，与 M 无关）

而 yaw 只是这个旋转在"竖直轴"上的投影：轴一旦不是竖直的，yaw 就不再随 ψ 线性变化，
累计出来是几百几十度的任意值。用 2026-11-26 那份真实外参
q=(0.8236,−0.2207,−0.1352,0.5047)、R_0≈Rx(−29.3°) 算，桥接输出的旋转轴与竖直差 33.7°，
于是"累计 yaw"和真实转角根本不是同一个量。**0.54 这类比值是这种投影误差，不是丢旋转。**

本工具改成三列对照：

  1. `累计相对旋转角`  —— Σ angle(R_k⁻¹R_{k+1})；刚性不变，**唯一能判"跟不跟"的量**
  2. `去安装后 yaw`    —— 对 M⁻¹·pose 再取 yaw（把固定安装旋转抵掉，便于和关节角比对）
  3. `旧口径 yaw`      —— 复现 diag_gimbal_chain 的算法，用来看"那个数是怎么来的"

用法::

    ./debug_tools/check_bridge_rotation.py                 # 起来后转云台 ≥360°
    ./debug_tools/check_bridge_rotation.py --min-spin 720 --timeout 90
    ./debug_tools/check_bridge_rotation.py --raw /aft_mapped_to_init   # point_lio 场合

退出码：0 = 桥接与原始位姿一致（刚性判据）；1 = 真的对不上；3 = 没等到转够。
"""

import argparse
import csv
import math
import os
import sys
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener

JOINT_NAME = "gimbal_yaw_joint"
RAW_CANDIDATES = ("/zlio/odometry", "/aft_mapped_to_init")
BRIDGE_TOPIC = "/lidar_odometry"


def _quat_R(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n <= 0:
        raise ValueError("退化四元数")
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def _msg_R(msg):
    q = msg.pose.pose.orientation
    return _quat_R(q.x, q.y, q.z, q.w)


def _yaw(R):
    return math.atan2(R[1, 0], R[0, 0])


def _angle(R):
    return math.acos(max(-1.0, min(1.0, (np.trace(R) - 1.0) / 2.0)))


def _wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _accumulate(seq, step):
    """对每对相邻样本累加 step(a, b)；样本不足返回 None。"""
    if len(seq) < 10:
        return None
    return sum(step(a, b) for a, b in zip(seq, seq[1:]))


def _rpy(R):
    return (math.degrees(math.atan2(R[2, 1], R[2, 2])),
            math.degrees(-math.asin(max(-1.0, min(1.0, R[2, 0])))),
            math.degrees(math.atan2(R[1, 0], R[0, 0])))


class ChainChecker(Node):
    """同时录原始位姿、桥接位姿与关节角，等云台转够后按刚性判据对账。"""

    def __init__(self, raw_topic):
        super().__init__("check_bridge_rotation")
        qos = QoSProfile(depth=2000, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        self.raw_topic = raw_topic
        self.raw = []
        self.out = []
        self.joint = []
        self.raw_rpy = []
        self.out_rpy = []
        self.create_subscription(Odometry, raw_topic,
                                 lambda m: self._odom(m, self.raw, self.raw_rpy), qos)
        self.create_subscription(Odometry, BRIDGE_TOPIC,
                                 lambda m: self._odom(m, self.out, self.out_rpy), qos)
        self.create_subscription(JointState, "/joint_states", self._joint, qos)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.mount = None

    def _odom(self, msg, seq, rpy_seq):
        try:
            R = _msg_R(msg)
        except ValueError:
            return
        seq.append(R)
        rpy_seq.append(_rpy(R))

    def _joint(self, msg):
        if JOINT_NAME in msg.name:
            self.joint.append(float(msg.position[msg.name.index(JOINT_NAME)]))

    def sample_mount(self):
        """抓一次 gimbal_yaw <- livox_imu（固定安装外参，与桥接用的那份同源）。"""
        try:
            tf = self.buffer.lookup_transform("gimbal_yaw", "livox_imu",
                                              rclpy.time.Time(seconds=0))
        except Exception:  # noqa: BLE001 - TF 还没就绪就下次再试
            return
        q = tf.transform.rotation
        self.mount = _quat_R(q.x, q.y, q.z, q.w)

    def raw_joint_travel(self):
        """关节角累计转角（弧度，取绝对值）。"""
        total = _accumulate(self.joint, lambda a, b: abs(_wrap(b - a)))
        return total

    def report(self, elapsed, spin_threshold_deg):
        joint_travel = self.raw_joint_travel()
        if joint_travel is None or math.degrees(joint_travel) < spin_threshold_deg:
            print("[未完成] 云台只转了 %.1f°（要求 ≥ %.0f°），再转一会儿"
                  % (math.degrees(joint_travel or 0.0), spin_threshold_deg))
            return 3
        if self.mount is None:
            print("[错误] 没拿到 gimbal_yaw←livox_imu 外参（TF 没起来？）")
            return 1

        R_mount = self.mount
        rows = []
        for name, seq, rpy in (("原始 %s" % self.raw_topic, self.raw, self.raw_rpy),
                               ("桥接 %s" % BRIDGE_TOPIC, self.out, self.out_rpy)):
            rigid = _accumulate(seq, lambda a, b: _angle(a.T @ b))
            old_yaw = _accumulate(seq, lambda a, b: _wrap(_yaw(b) - _yaw(a)))
            demount = _accumulate(seq, lambda a, b: _wrap(
                _yaw(R_mount.T @ b) - _yaw(R_mount.T @ a)))
            rows.append((name, rigid, old_yaw, demount, len(seq), rpy))

        print("\n================ 云台旋转链路复检 ================")
        print("窗口 %.1f s｜关节角累计转角 %.1f°｜样本 原始 %d / 桥接 %d"
              % (elapsed, math.degrees(joint_travel), len(self.raw), len(self.out)))
        print("安装外参 M = gimbal_yaw←livox_imu: RPY %+.1f / %+.1f / %+.1f" % _rpy(R_mount))
        print()
        print("%-24s %14s %12s %14s" % ("话题", "累计相对旋转角", "去安装 yaw", "旧口径 yaw"))
        for name, rigid, old_yaw, demount, n, _ in rows:
            def show(v):
                return "—" if v is None else "%+8.1f°" % math.degrees(v)
            print("%-24s %14s %12s %14s" % (name, show(rigid), show(demount), show(old_yaw)))
        print()
        print("判据（刚性不变）：累计相对旋转角 应与关节角同量级，且桥接 == 原始。")
        print("      （旧口径 yaw 不是转角：旋转轴不竖直时它只是投影，别拿它判定。）")
        print()

        expected = math.degrees(joint_travel)
        ok = True
        rigid_raw = rows[0][1]
        rigid_out = rows[1][1]
        for name, rigid in (("原始", rigid_raw), ("桥接", rigid_out)):
            if rigid is None:
                print("[跳过] %s 样本不足" % name)
                continue
            ratio = math.degrees(rigid) / expected if expected > 0 else float("nan")
            good = abs(ratio - 1.0) <= 0.03
            ok = ok and good
            print("  %s 相对旋转角 %.1f° / 关节 %.1f° = %.4f  %s"
                  % (name, math.degrees(rigid), expected, ratio, "OK" if good else "✗ 不跟"))
        if rigid_raw is not None and rigid_out is not None:
            diff = abs(math.degrees(rigid_out - rigid_raw))
            identical = diff <= 0.5
            print("  桥接 vs 原始 相对旋转角差 %.3f° → %s"
                  % (diff, "一致（刚性合成，符合预期）" if identical else "不一致（桥接真有问题）"))
            ok = ok and identical

        # 姿态有没有随云台摆动（雷达斜装 30°，roll 应基本不变）
        for name, _, _, _, n, rpy in rows:
            if len(rpy) > 20:
                roll = [r[0] for r in rpy]
                pitch = [r[1] for r in rpy]
                print("  %-22s roll %.1f…%.1f°（摆幅 %.1f°） pitch %.1f…%.1f°（摆幅 %.1f°）"
                      % (name, min(roll), max(roll), max(roll) - min(roll),
                         min(pitch), max(pitch), max(pitch) - min(pitch)))
        print()
        verdict = ("PASS：桥接与原始位姿的旋转完全一致；'丢旋转'是旧 yaw 口径的投影误差"
                   if ok else
                   "FAIL：桥接真的没跟上原始位姿，按上面两行定位")
        print("[结论] %s" % verdict)
        print("=" * 50)
        return 0 if ok else 1

    def dump_csv(self, path):
        with open(path, "w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(["i", "raw_yaw_deg", "out_yaw_deg", "joint_rad"])
            for i in range(max(len(self.raw), len(self.out))):
                writer.writerow([
                    i,
                    "%.3f" % math.degrees(_yaw(self.raw[i])) if i < len(self.raw) else "",
                    "%.3f" % math.degrees(_yaw(self.out[i])) if i < len(self.out) else "",
                    "%.6f" % self.joint[i] if i < len(self.joint) else "",
                ])


def main():
    parser = argparse.ArgumentParser(description="云台旋转链路复检（刚性判据）")
    parser.add_argument("--raw", default=RAW_CANDIDATES[0],
                        help="原始位姿话题（默认 %s；point_lio 用 /aft_mapped_to_init）"
                             % RAW_CANDIDATES[0])
    parser.add_argument("--min-spin", type=float, default=360.0,
                        help="云台累计转过这么多度才算数（默认 360）")
    parser.add_argument("--timeout", type=float, default=180.0,
                        help="最长等待秒数（默认 180）")
    parser.add_argument("--csv", default="",
                        help="把三路 yaw 存成 CSV（默认不存）")
    args = parser.parse_args()

    rclpy.init()
    node = ChainChecker(args.raw)
    print("[等待] 现在把云台转起来（累计 ≥ %.0f°）；最多等 %.0f s" % (args.min_spin, args.timeout))
    deadline = time.time() + args.timeout
    code = 3
    started = time.time()
    try:
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.mount is None:
                node.sample_mount()
            joint = node.raw_joint_travel()
            if joint is not None and math.degrees(joint) >= args.min_spin:
                # 多收 0.5 s，保证最后一段也算进去
                end = time.time() + 0.5
                while time.time() < end:
                    rclpy.spin_once(node, timeout_sec=0.1)
                code = node.report(time.time() - started, args.min_spin)
                break
    except KeyboardInterrupt:  # noqa: BLE001 - 允许中途 Ctrl-C 出报告
        code = node.report(time.time() - started, 0.0)
    finally:
        if args.csv:
            os.makedirs(os.path.dirname(os.path.abspath(args.csv)), exist_ok=True)
            node.dump_csv(args.csv)
            print("[CSV] %s" % args.csv)
        node.destroy_node()
        rclpy.shutdown()
    return code


if __name__ == "__main__":
    sys.exit(main())
