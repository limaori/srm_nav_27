#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""云台系 yaw 链路定位：一次定位"转角是在哪一环丢掉的"。

背景（2026-11-09 实测）：云台自转、车不动时，三路 yaw 应该满足

    /zlio/odometry（或 /aft_mapped_to_init，雷达原始位姿）
        └─ zlio_bridge/loam_interface ─► /lidar_odometry（仍是**雷达**位姿）
              └─ sensor_scan_generation ─► /odometry（**车**=云台位姿，child=gimbal_yaw）

云台转 φ 时，前两者都应转 φ（雷达就在云台上），`/joint_states` 的
`gimbal_yaw_joint` 应转 -φ（树反转后它表示"底盘相对云台"），而 `/odometry` 也应转 φ
（车头跟着云台），**只是它的 xy 不该跟着画圆**（那是本工具不判的部分，
用 `capture_gimbal_spin.py` 判）。

用法::

    # 先起导航栈，然后运行本工具，再让云台转起来（>=360°，约 3 s @150°/s）
    ./debug_tools/diag_gimbal_chain.py
    ./debug_tools/diag_gimbal_chain.py --min-spin 720 --timeout 120

退出码：0 = 三路一致；1 = 定位到断点；3 = 没等到云台转够。
"""

import argparse
import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

RAW_CANDIDATES = ("/zlio/odometry", "/aft_mapped_to_init")
LIDAR_TOPIC = "/lidar_odometry"
ROBOT_TOPIC = "/odometry"
JOINT_TOPIC = "/joint_states"
JOINT_NAME = "gimbal_yaw_joint"


def wrap(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class Diag(Node):
    def __init__(self):
        super().__init__("diag_gimbal_chain")
        # 关键：`livox_imu <- gimbal_yaw` 必须是**静态外参**。若它在云台转时跟着变，
        # 说明外参是被"动态 TF"拼出来的（例如 livox_imu 挂到了 base_link 那一侧）。
        self.buffer = Buffer(cache_time=Duration(seconds=20.0))
        self.listener = TransformListener(self.buffer, self)
        self.extrinsic_yaw = []
        self.extrinsic = None          # T(gimbal_yaw <- livox_imu) 最新一次
        self.lidar_pose = []
        self.rpy = {}
        self.robot_pose = []
        qos = QoSProfile(depth=500, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        self.data = {t: [] for t in RAW_CANDIDATES + (LIDAR_TOPIC, ROBOT_TOPIC)}
        self.joint = []
        for topic in self.data:
            self.create_subscription(
                Odometry, topic, lambda m, t=topic: self._odom(m, t), qos)
        self.create_subscription(JointState, JOINT_TOPIC, self._joint, qos)

    def _odom(self, msg, topic):
        q = msg.pose.pose.orientation
        p = msg.pose.pose.position
        self.data[topic].append(
            math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        rpy = _rpy_of((q.x, q.y, q.z, q.w))
        self.rpy.setdefault(topic, []).append(rpy)
        if topic == LIDAR_TOPIC:
            self.lidar_pose.append(((p.x, p.y, p.z), (q.x, q.y, q.z, q.w)))
        if topic == ROBOT_TOPIC:
            self.robot_pose.append(((p.x, p.y, p.z), (q.x, q.y, q.z, q.w)))

    def _joint(self, msg):
        if JOINT_NAME in msg.name:
            self.joint.append(msg.position[msg.name.index(JOINT_NAME)])

    def sample_extrinsic(self):
        try:
            tf = self.buffer.lookup_transform(
                "gimbal_yaw", "livox_imu", rclpy.time.Time(seconds=0, nanoseconds=0))
        except Exception:  # noqa: BLE001 - 还没建好就跳过
            return
        q = tf.transform.rotation
        t = tf.transform.translation
        self.extrinsic = ((t.x, t.y, t.z), (q.x, q.y, q.z, q.w))
        self.extrinsic_yaw.append(
            math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def net(self, seq):
        if len(seq) < 10:
            return None
        total = 0.0
        for a, b in zip(seq, seq[1:]):
            total += wrap(b - a)
        return math.degrees(total)


def main():
    parser = argparse.ArgumentParser(description="云台系 yaw 链路断点定位")
    parser.add_argument("--min-spin", type=float, default=360.0,
                        help="关节角累计转过这么多度才算数（默认 360）")
    parser.add_argument("--timeout", type=float, default=300.0, help="最多等多少秒")
    parser.add_argument("--tolerance", type=float, default=0.10,
                        help="各路净转角相对关节角的允许比例误差（默认 10%%）")
    args = parser.parse_args()

    rclpy.init()
    node = Diag()
    print("等待云台转动（关节角累计 >= %.0f°，最多 %.0f s）…… 现在让云台转起来"
          % (args.min_spin, args.timeout), flush=True)
    deadline = time.monotonic() + args.timeout
    joint_base = None
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if len(node.joint) % 20 == 0:
            node.sample_extrinsic()
        if len(node.joint) < 2:
            continue
        if joint_base is None:
            joint_base = node.joint[0]
        travelled = abs(math.degrees(_wrap_total(node.joint)))
        if travelled >= args.min_spin:
            break
        if int(time.monotonic()) % 5 == 0:
            pass
    time.sleep(0.5)  # 收一下尾巴
    spin = abs(math.degrees(_wrap_total(node.joint)))
    joints = node.net(node.joint)
    results = {t: node.net(v) for t, v in node.data.items()}
    rclpy.shutdown()

    ext = node.extrinsic_yaw
    extrinsic_drift_deg = (
        math.degrees(max(abs(wrap(a - ext[0])) for a in ext)) if len(ext) > 20 else None
    )

    if joints is None or spin < args.min_spin:
        print("[结论] 云台只转了 %.0f°（门槛 %.0f°），测不出东西。" % (spin, args.min_spin))
        for topic, value in results.items():
            print("  %-20s %s" % (topic, "无数据" if value is None else "%+.1f°" % value))
        return 3

    reference = -joints  # 树反转后关节角 = 底盘相对云台 = -φ
    print()
    print("关节角净转 %+.1f° ⇒ 云台(车)应转 %+.1f°；下位机云台转速与之一致即正常"
          % (joints, reference))
    print()
    print("%-22s %19s %19s" % ("话题（姿态）", "开始 roll/pitch", "结束 roll/pitch"))
    for topic, seq in node.rpy.items():
        if len(seq) < 5:
            continue
        a, b = seq[0], seq[-1]
        print("%-22s %9.1f / %-8.1f %9.1f / %-8.1f  Δ=%+.0f°"
              % (topic, a[0], a[1], b[0], b[1], b[0] - a[0]))
    if extrinsic_drift_deg is not None:
        print("外参 livox_imu <- gimbal_yaw 的 yaw 漂移 %.1f°（应≈0：固定安装外参）"
              % extrinsic_drift_deg)
    print("%-22s %12s %10s %s" % ("话题", "净转角", "占应转比例", "判定"))
    bad = []
    for topic, value in results.items():
        if value is None:
            print("%-22s %12s %10s 无数据" % (topic, "-", "-"))
            continue
        ratio = value / reference if abs(reference) > 1e-6 else float("nan")
        verdict = "OK" if abs(ratio - 1.0) <= args.tolerance else "✗ 不跟"
        if abs(ratio - 1.0) > args.tolerance:
            bad.append(topic)
        print("%-22s %10.1f° %10.2f  %s" % (topic, value, ratio, verdict))

    ext = node.extrinsic_yaw
    if False:
        drift = max(abs(wrap(a - ext[0])) for a in ext)
        print("外参 livox_imu <- gimbal_yaw 的 yaw 漂移 %.1f°（应≈0：它是固定安装外参）"
              % math.degrees(drift))
        if math.degrees(drift) > 5.0:
            print("[结论] 外参在动！livox_imu 相对 gimbal_yaw 不是静态的 ——")
            print("       sensor_scan_generation 用这份外参做补偿，自然把云台转角又减掉了。")
            print("       查 robot_state_publisher 是否带 gimbal_is_root:=true、")
            print("       URDF 里 livox_frame 的 parent 是否为 gimbal_yaw。")
            rclpy.shutdown()
            return 1
    print()
    if extrinsic_drift_deg is not None and extrinsic_drift_deg > 5.0:
        print("[结论] 外参在动！livox_imu 相对 gimbal_yaw 不是静态的 ——")
        print("       sensor_scan_generation 拿这份外参做补偿，会把云台转角再减一次。")
        print("       查 robot_state_publisher 是否带 gimbal_is_root:=true。")
        return 1
    recomposed = _recompose(node)
    if recomposed is not None:
        print("Python 侧用同一份外参复算 [lidar ∘ E] 的净转角 = %+.1f°（应≈云台转角 %+.1f°）"
              % (recomposed, reference))
        tilt = getattr(_recompose, "tilt", None)
        if tilt is not None:
            print("复算出来的姿态 roll 最大 %.1f°、pitch 最大 %.1f°（正常应 <20°：车体是水平的）"
                  % tilt)
            if tilt[0] > 45.0 or tilt[1] > 45.0:
                print("[结论] 组合出来的姿态是**斜的** ⇒ 雷达位姿的姿态约定和外参不匹配，")
                print("       yaw 提取因此病态（转轴几乎躺在水平面里，yaw 不跟着转）。")
                print("       查 /lidar_odometry 的 roll/pitch：正常应≈雷达安装角(30°/0°)，")
                print("       现在若≈-88° 就是 zlio_bridge 的位姿约定（R_0 抵消）弄错了。")
                return 1
        if abs(recomposed - reference) <= 0.15 * max(1.0, abs(reference)) and bad:
            print("[结论] 复算结果跟着云台转、但 /odometry 没转 → sensor_scan_generation 输出层的问题")
            print("       （不是外参、不是树方向）：查它缓存的固定外参、planarize/suppressJitter、")
            print("       以及是否在跑本次编译的二进制。")
            return 1

    if not bad:
        print("[结论] 三路都跟着云台转 → yaw 链路没有断点。")
        print("       剩下只需确认 xy 不画圈：./debug_tools/capture_gimbal_spin.py --no-plot")
        return 0

    if any(t in bad for t in RAW_CANDIDATES):
        print("[结论] 雷达原始位姿就不跟云台转 → LIO 前端本身的问题（不是本链路）。")
        return 1
    if LIDAR_TOPIC in bad:
        print("[结论] 原始位姿转了、%s 不转 → zlio_bridge/loam_interface 这一环丢了旋转。" % LIDAR_TOPIC)
        return 1
    if ROBOT_TOPIC in bad:
        print("[结论] %s 转了、%s 不转 → sensor_scan_generation 这一环丢了旋转：" % (LIDAR_TOPIC, ROBOT_TOPIC))
        print("       查 robot_base_frame 是否 gimbal_yaw、/joint_states 是否在发、")
        print("       以及 TF 里 livox_imu <- gimbal_yaw 是否是**静态**外参。")
        return 1
    print("[结论] 有话题不跟云台转：%s" % ", ".join(bad))
    return 1


def _rpy_of(q):
    r = _r_of(q)
    roll = math.degrees(math.atan2(r[2][1], r[2][2]))
    pitch = math.degrees(-math.asin(max(-1.0, min(1.0, r[2][0]))))
    yaw = math.degrees(math.atan2(r[1][0], r[0][0]))
    return roll, pitch, yaw


def _r_of(q):
    x, y, z, w = q
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


def _yaw_of(r):
    return math.atan2(r[1][0], r[0][0])


def _recompose(node):
    """用 Python 复算 base = lidar ∘ E，返回它的净转角（度）；数据不够返回 None。"""
    if node.extrinsic is None or len(node.lidar_pose) < 20:
        return None
    (tx, ty, tz), (qx, qy, qz, qw) = node.extrinsic
    # T(lidar <- gimbal) = E^-1
    rg = _r_of((qx, qy, qz, qw))
    rt = [[rg[j][i] for j in range(3)] for i in range(3)]
    t_lg = [-sum(rt[i][k] * (tx, ty, tz)[k] for k in range(3)) for i in range(3)]

    def compose(p, q):
        rl = _r_of(q)
        yaw = _yaw_of([
            [sum(rl[i][k] * rt[k][j] for k in range(3)) for j in range(3)]
            for i in range(3)])
        return yaw

    yaws = [compose(p, q) for p, q in node.lidar_pose]
    total = 0.0
    for a, b in zip(yaws, yaws[1:]):
        total += wrap(b - a)

    # 组合出来的到底是不是"水平的车体姿态"？roll/pitch 大 = 姿态约定不匹配，
    # 此时 yaw 提取是病态的（转轴几乎躺在水平面里 -> yaw 不跟转）。
    tilts = [_tilt_of(p, q, rt) for p, q in node.lidar_pose]
    rolls = [t[0] for t in tilts]
    pitches = [t[1] for t in tilts]
    _recompose.tilt = (max(abs(r) for r in rolls), max(abs(p) for p in pitches))
    return math.degrees(total)


def _tilt_of(p, q, rt):
    rl = _r_of(q)
    r = [[sum(rl[i][k] * rt[k][j] for k in range(3)) for j in range(3)]
         for i in range(3)]
    roll = math.degrees(math.atan2(r[2][1], r[2][2]))
    pitch = math.degrees(-math.asin(max(-1.0, min(1.0, r[2][0]))))
    return roll, pitch


def _wrap_total(seq):
    total = 0.0
    for a, b in zip(seq, seq[1:]):
        total += wrap(b - a)
    return total


if __name__ == "__main__":
    sys.exit(main())
