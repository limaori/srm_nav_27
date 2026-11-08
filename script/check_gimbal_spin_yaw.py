#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""云台单转、车不动 —— 三方 yaw 对照（RViz 里那根箭头的数值版）。

背景
====
云台自转、底盘静止时, 三个 yaw 应该满足一条**恒等式**::

    yaw(odom -> base_link) ≈ yaw(raw LIO) − θ_joint

其中

  * ``yaw(raw LIO)``   = ``/aft_mapped_to_init`` 的 yaw（point_lio 在 ``camera_init`` 里的
    原始输出; 换 zlio 前端时是 ``/zlio/odometry``）。雷达装在云台上, 所以云台一转,
    **这一项一定会跟着转**——它转是对的, 不是 bug。
  * ``θ_joint``        = ``/joint_states`` 里 ``gimbal_yaw_joint`` 的角, 也就是
    ``base_link -> gimbal_yaw`` 那条 TF 的 yaw（URDF: parent=base_link, child=gimbal_yaw）。
  * ``yaw(odom -> base_link)`` = ``/odometry``（sensor_scan_generation 反推、已减掉关节角）。
    车没动时**它应当基本不变**。

RViz 里看箭头只能看出"转没转", 这个脚本直接把三者同时打出来, 并把"转了多少度"
从箭头变成数字, 顺带把最常见的三种失败模式自动判掉:

  ============================  ==================================================
  Δbase ≈ 0                    补偿正确（Δraw ≈ Δθ）
  Δbase ≈ Δθ                   TF 里**根本没有关节角**（云台角没进 /joint_states,
                               或 relay 超时停发后 TF 冻在最后一帧）
  Δbase ≈ 2Δθ                  关节角**符号反了**（gimbal_joint_sign 该翻）
  其它                          Δraw 与 Δθ 不同步: 外参 / 时间戳 / LIO 自己在漂
  ============================  ==================================================

用法
====
车上已在跑导航栈（point_lio + robot_state_publisher + relay）::

    ./script/check_gimbal_spin_yaw.py                 # 采 15 s
    # 然后让云台慢慢转 30~45°（转回来更好, 能顺带看 ±180 归一化）
    ./script/check_gimbal_spin_yaw.py --seconds 20

    # ZLIO 前端（raw 话题换了名字, 其余不变）
    ./script/check_gimbal_spin_yaw.py --raw-topic /zlio/odometry

    # 只做建图链路（real_mapping_launch）时没有 /odometry, 用 TF 版对比:
    ./script/check_gimbal_spin_yaw.py --base-topic /lidar_odometry

退出码
======
  0 = 通过（云台在转、base_link 没跟着转）
  1 = 不通过（按打印出来的病根排查）
  2 = 用法/环境错误
  3 = 没采到有效运动（云台没转够 / 话题没数据）—— 先确认链路, 别改参数
"""

import argparse
import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32


def yaw_of(quaternion):
    """四元数取 yaw（rad），与全仓库其它脚本同一套公式。"""
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def deg(angle):
    return math.degrees(angle)


class YawCollector(Node):
    def __init__(self, args):
        super().__init__("check_gimbal_spin_yaw")
        self.args = args
        self.raw_yaw = None
        self.raw_stamp = None
        self.base_yaw = None
        self.base_stamp = None
        self.joint_yaw = None
        self.joint_stamp = None
        self.joint_source = ""
        self._warned_joint_name = False

        # 里程计一律用 best_effort 订阅: point_lio / sensor_scan_generation 发的是
        # reliable, best_effort 订阅者同样能收；反过来（发布者 best_effort、订阅者 reliable）
        # 才会 QoS 不兼容。只为了看数, 不必强求 reliable。
        qos = QoSProfile(
            depth=10,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(Odometry, args.raw_topic, self.raw_cb, qos)
        self.create_subscription(Odometry, args.base_topic, self.base_cb, qos)
        self.create_subscription(JointState, args.joint_topic, self.joint_cb, qos)
        # relay 没起来 / 只跑了 srm27_nav_protocol 时, 用中继的**输入**顶一下,
        # 两者的量纲一样（rad）, 只是少了 relay 的重发与超时保护。
        self.create_subscription(Float32, args.fallback_joint_topic, self.joint_float_cb, qos)

    # ---- 回调 ----
    def raw_cb(self, msg):
        self.raw_yaw = yaw_of(msg.pose.pose.orientation)
        self.raw_stamp = self.get_clock().now()

    def base_cb(self, msg):
        self.base_yaw = yaw_of(msg.pose.pose.orientation)
        self.base_stamp = self.get_clock().now()

    def joint_cb(self, msg):
        if self.args.joint_name not in msg.name:
            if not self._warned_joint_name:
                print(
                    "[警告] %s 里没有关节 %s（现有: %s）"
                    % (self.args.joint_topic, self.args.joint_name, ",".join(msg.name)),
                    flush=True,
                )
                self._warned_joint_name = True
            return
        self._set_joint(msg.position[msg.name.index(self.args.joint_name)], "joint_states")

    def joint_float_cb(self, msg):
        # JointState 在发时以它为准（relay 的超时保护才是链路的真实状态）。
        if self.joint_source == "joint_states":
            return
        self._set_joint(float(msg.data), "gimbal_joint_angle")

    def _set_joint(self, value, source):
        self.joint_yaw = wrap(value)
        self.joint_stamp = self.get_clock().now()
        self.joint_source = source

    # ---- 取值 ----
    def age(self, stamp):
        if stamp is None:
            return float("inf")
        return (self.get_clock().now() - stamp).nanoseconds / 1e9


def fmt(value, width=7):
    return ("%" + str(width) + ".1f") % value


def main():
    parser = argparse.ArgumentParser(
        description="云台单转（车不动）时 raw LIO / 关节角 / odom->base_link 的 yaw 对照",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--raw-topic", default="/aft_mapped_to_init",
                        help="雷达算法原始里程计（zlio 前端换成 /zlio/odometry）")
    parser.add_argument("--base-topic", default="/odometry",
                        help="已减掉云台角的底盘里程计（建图链路可用 /lidar_odometry 代替）")
    parser.add_argument("--joint-topic", default="/joint_states",
                        help="robot_state_publisher 用的关节角话题")
    parser.add_argument("--joint-name", default="gimbal_yaw_joint",
                        help="关节名（必须与 URDF 一致）")
    parser.add_argument("--fallback-joint-topic", default="/gimbal_joint_angle",
                        help="joint_states 缺席时改读 relay 的输入（Float32, rad）")
    parser.add_argument("--seconds", type=float, default=15.0, help="采样时长（秒）")
    parser.add_argument("--period", type=float, default=0.5, help="打印周期（秒）")
    parser.add_argument("--tol-deg", type=float, default=2.0,
                        help="判定容差（度）: 静止底盘 yaw 的允许漂移")
    parser.add_argument("--min-spin-deg", type=float, default=8.0,
                        help="云台至少要转过这么多度才有判定意义")
    parser.add_argument("--quiet", action="store_true", help="只打最后的结论")

    args = parser.parse_args()
    if args.seconds <= 0 or args.period <= 0:
        print("[错误] --seconds / --period 必须为正数", file=sys.stderr)
        return 2

    rclpy.init()
    node = YawCollector(args)
    print(
        "采集 %.1f s: raw=%s  base=%s  joint=%s(%s)"
        % (args.seconds, args.raw_topic, args.base_topic, args.joint_topic, args.joint_name),
        flush=True,
    )
    if not args.quiet:
        print(
            "%6s %9s %9s %9s %10s %9s %9s"
            % ("t/s", "θ/°", "raw/°", "base/°", "raw-θ/°", "Δθ/°", "Δbase/°"),
            flush=True,
        )

    base_theta = base_raw = base_base = None
    rows = 0
    max_base_dev = 0.0
    joint_stale_warned = False

    deadline = time.monotonic() + args.seconds
    next_print = time.monotonic()
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            if now < next_print:
                continue
            next_print = now + args.period

            if node.raw_yaw is None or node.base_yaw is None:
                continue

            joint_age = node.age(node.joint_stamp)
            if node.joint_yaw is None:
                if not args.quiet:
                    print(
                        "%6.1f %9s %9s %9s %10s %9s %9s   (还没有关节角)"
                        % (args.seconds - (deadline - now), "-", fmt(deg(node.raw_yaw)),
                           fmt(deg(node.base_yaw)), "-", "-", "-"),
                        flush=True,
                    )
                continue

            # 关节角停更 = relay 超时（它刻意不发 0 冒充回零），此时 TF 冻在最后一帧,
            # base yaw 会开始跟着 raw 漂 —— 这是"数据断了", 不是符号问题, 单独提示。
            if joint_age > 1.0 and not joint_stale_warned:
                print(
                    "[警告] 关节角已 %.1f s 没更新（relay 超时后会停发）, 后面几行 base/° 会跟着 raw 漂。"
                    % joint_age,
                    flush=True,
                )
                joint_stale_warned = True

            if base_theta is None:
                base_theta, base_raw, base_base = node.joint_yaw, node.raw_yaw, node.base_yaw
                t0 = now

            d_theta = wrap(node.joint_yaw - base_theta)
            d_raw = wrap(node.raw_yaw - base_raw)
            d_base = wrap(node.base_yaw - base_base)
            residual = wrap(node.raw_yaw - node.joint_yaw - node.base_yaw)
            max_base_dev = max(max_base_dev, abs(deg(d_base)))
            rows += 1

            if not args.quiet:
                print(
                    "%6.1f %9s %9s %9s %10s %9s %9s"
                    % (now - t0, fmt(deg(node.joint_yaw)), fmt(deg(node.raw_yaw)),
                       fmt(deg(node.base_yaw)), fmt(deg(residual)),
                       fmt(deg(d_theta)), fmt(deg(d_base))),
                    flush=True,
                )
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

    print("", flush=True)
    if base_theta is None or rows == 0:
        print("[结论] 没采到有效数据: 至少 raw / base / joint 三路要同时有数据。", flush=True)
        print("       自查: ros2 topic hz %s ; ros2 topic hz %s ; ros2 topic echo --once %s"
              % (args.raw_topic, args.base_topic, args.joint_topic), flush=True)
        print("       没有 /joint_states: 看 %s 有没有数据（relay 没起时才需要看它）"
              % args.fallback_joint_topic, flush=True)
        print("       没有 %s: 建图链路（real_mapping_launch）里用 --base-topic /lidar_odometry"
              % args.base_topic, flush=True)
        return 3

    d_theta = deg(wrap(node.joint_yaw - base_theta))
    d_raw = deg(wrap(node.raw_yaw - base_raw))
    d_base = deg(wrap(node.base_yaw - base_base))
    residual = deg(wrap(node.raw_yaw - node.joint_yaw - node.base_yaw))
    tol = args.tol_deg

    print("窗口 %.1f s, 有效样本 %d, 关节角来源: %s"
          % (node.args.seconds, rows, node.joint_source), flush=True)
    print("  Δθ(关节) = %+.1f°   Δraw(LIO) = %+.1f°   Δbase(odom->base) = %+.1f°"
          % (d_theta, d_raw, d_base), flush=True)
    print("  raw-θ-base 的残差 = %+.1f°（理论 0；它不随时间变，就说明三路对齐）" % residual,
          flush=True)
    print("  base yaw 最大偏移 = %.1f°（容差 %.1f°）" % (max_base_dev, tol), flush=True)
    print("", flush=True)

    if abs(d_theta) < args.min_spin_deg:
        print("[结论] 云台只转了 %.1f°（门槛 %.1f°），测不出东西。" % (d_theta, args.min_spin_deg))
        print("       让云台慢慢转 30~45° 再跑一次。")
        return 3

    if abs(d_base) <= tol:
        if abs(d_raw - d_theta) <= max(tol, 0.15 * abs(d_theta)):
            print("[结论] PASS：云台转了 %+.1f°，raw 跟着转 %+.1f°，底盘 yaw 只动了 %+.1f°。"
                  % (d_theta, d_raw, d_base))
            print("       关节角进 TF 且符号正确（gimbal_joint_sign 不用动）。")
            return 0
        print("[结论] base 没动但 raw 与关节角不同步（Δraw-Δθ = %+.1f°）。" % (d_raw - d_theta))
        print("       云台在转、车没动, 两者应当相等 —— 查: 关节轴与云台是否同轴 /")
        print("       雷达外参 roll/pitch / IMU 与点云时间戳。")
        return 1

    if abs(d_base - 2.0 * d_theta) <= max(tol, 0.15 * abs(d_theta)):
        print("[结论] FAIL：Δbase ≈ 2Δθ，**关节角符号反了**（减成了加）。")
        print("       改 srm27_nav_protocol.yaml 的 gimbal_joint_sign（+1.0 <-> -1.0）后重启链路。")
        return 1

    if abs(d_base - d_theta) <= max(tol, 0.15 * abs(d_theta)):
        print("[结论] FAIL：Δbase ≈ Δθ，**TF 里没有云台关节角**（补偿完全没发生）。")
        print("       查: /joint_states 有没有 gimbal_yaw_joint、relay 是否超时停发、")
        print("       robot_state_publisher 是不是 gimbal_joint_type:=revolute。")
        return 1

    print("[结论] FAIL：Δbase / Δθ / Δraw 三者对不上, 不是单纯的符号或缺失。")
    print("       常见原因: 关节角有延迟或按别的周期更新、LIO 自己在漂、或云台角被算了两遍。")
    return 1


if __name__ == "__main__":
    sys.exit(main())
