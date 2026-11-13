#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""自动抓「车不动、只转云台」这一段，出 CSV + 曲线图 + 结论。

为什么要这个工具
================
**先看 `/odometry` 的 `child_frame_id`，它决定判据**（2026-11-08 云台系改造后有两种树）：

  * ``gimbal_yaw``（**新，实车默认**）：导航的"车"就是云台，`/odometry` 是
    ``odom -> gimbal_yaw``。云台转，它**必须**跟着转，关节角**不许**进来。
  * ``base_link``（旧树，仿真/旧配置）：`/odometry` 是 ``odom -> base_link``，
    云台转时它**必须**几乎不动（关节角被减掉了）。

旧树的恒等式（云台自转、底盘静止）::

    yaw(odom -> base_link) ≈ yaw(raw LIO) − θ_joint

  * ``raw LIO``  = ``/aft_mapped_to_init``（point_lio）或 ``/zlio/odometry``（ZLIO 前端）。
    雷达装在云台上，**云台一转它一定跟着转**——这是物理事实，不是 bug。
  * ``θ_joint``  = ``/joint_states`` 里的 ``gimbal_yaw_joint``（缺省时退回 relay 的输入
    ``/gimbal_joint_angle``），也就是 ``base_link -> gimbal_yaw`` 那条 TF 的 yaw。
  * ``odom -> base_link`` = ``/odometry``（sensor_scan_generation 反推、已减掉关节角）。
    车没动时**它应当基本不动**，这才是判据。

RViz 里那根箭头只能看出"转没转"。本工具把三路同时录下来，用"逐样本增量的回归斜率"
判定（**不依赖首末两点**，所以"转过去再转回来"也能判），最后落到四个文件::

    user_test_log/gimbal_spin_<YYYYmmdd_HHMMSS>.csv    逐样本原始数据
    user_test_log/gimbal_spin_<YYYYmmdd_HHMMSS>.png    三条曲线 + 残差
    user_test_log/gimbal_spin_<YYYYmmdd_HHMMSS>.txt    结论（同时打在终端上）
    user_test_log/gimbal_spin_<YYYYmmdd_HHMMSS>/raw/   --bag 时额外录的 rosbag

判据（斜率 s = Σ(Δyaw·Δyaw')/Σ(Δyaw'²)，逐样本增量回归）
=========================================================
旧树（`child_frame_id = base_link`）:  s_base 对 θ 的斜率
  ==================  ==========================================================
  s_base ≈ 0          PASS：关节角进了 TF 且符号正确
  s_base ≈ 1          TF 里**没有云台关节角**（/joint_states 没发、relay 超时停发、
                      或 robot_state_publisher 不是 revolute）
  s_base ≈ 2          **关节角符号反了**（gimbal_joint_sign 该翻）
  s_raw ≈ 0           LIO 没在跟踪（话题停更 / 点云或 IMU 断），与补偿无关
  ==================  ==========================================================

新树（`child_frame_id = gimbal_yaw`，实车）:  s_base 对 raw 的斜率 + 三条附加判据
  ================================  ============================================
  s_base_vs_raw ≈ 1 且残差恒定       PASS：定位位姿被完整继承，关节角没混进来
  锁相残差半径 < --residual-tol      PASS 的另一半：云台绕轴转不该让车"走"
      （默认 0.04 m；⚠ 别设成 0.02：前端自己估到的绕轴平移只有物理臂长的
        60%~92%，外参完全正确也会剩 2~5 cm。外参对不对要用"点云 vs 实物"验）
  s_base_vs_raw ≈ 1 − Δθ/Δraw       FAIL：还有一处把关节角减掉了（旧逻辑/旧配置）
  s_base_vs_raw ≈ 0                 FAIL：`odom -> gimbal_yaw` 没跟云台转
                                    （TF 反转没生效，或 /odometry 还是底盘位姿）
  s_raw ≈ 0                         LIO 没在跟踪，与补偿无关
  ================================  ============================================

用法
====
车上已经在跑 `start_real_nav.sh`（或建图链路），数据链路稳定后再开始转云台::

    ./debug_tools/capture_gimbal_spin.py                # 一直录，Ctrl-C 出结论
    ./debug_tools/capture_gimbal_spin.py --duration 60  # 录 60 s 自动收尾
    # 然后让云台慢慢转 30~45°，最好再转回来（本工具对往返同样有效）

    ./debug_tools/capture_gimbal_spin.py --bag          # 顺带录 rosbag（事后可复算）
    ./debug_tools/capture_gimbal_spin.py --no-plot      # 车上没图形界面时

  · 话题**全自动识别**：先有数据的那条胜出（point_lio / zlio / 建图链路都认），
    终端会打印最终用的是哪三路。
  · 启动时导航栈还没起来也没关系：会先等，等到三路都有数据才开始录（--wait 默认 300 s）。
  · 只用一条命令就能把"我转了云台"这件事变成可追溯的证据；结论同时写进 .txt。

退出码
======
  0 = PASS
  1 = 不通过（终端里给了病根）
  2 = 用法/环境错误
  3 = 没采到有效数据（等超时 / 云台没转够）
"""

import argparse
import math
import os
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_OUT_DIR = REPO_ROOT / "user_test_log"

RAW_CANDIDATES = ("/aft_mapped_to_init", "/zlio/odometry")
BASE_CANDIDATES = ("/odometry", "/lidar_odometry")
JOINT_TOPIC = "/joint_states"
JOINT_FALLBACK_TOPIC = "/gimbal_joint_angle"
JOINT_NAME = "gimbal_yaw_joint"
#: 下位机回传的车速（m/s，云台系）—— 独立于 LIO 的第三方证据
BOARD_SPEED_TOPIC = "/chassis_speed"


def yaw_of(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def deg(angle):
    return math.degrees(angle)


# ----------------------------------------------------------------- 采集


class Collector:
    """ROS 侧的最小封装：只在有 rclpy 时才建节点，方便先 --demo 验证画图链路。"""

    def __init__(self, rate_hz, forced_base_topic=""):
        import rclpy
        from geometry_msgs.msg import TwistStamped
        from nav_msgs.msg import Odometry
        from rclpy.node import Node
        from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float32

        # 里程计一律 best_effort：reliable 发布者也能被 best_effort 订阅者收到，
        # 反过来才会 QoS 不兼容，这样点云链路 / 建图链路上都不会收不到。
        qos = QoSProfile(
            depth=10,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        rclpy.init()
        self.rclpy = rclpy
        self.node = Node("capture_gimbal_spin")
        self.raw_topic = self.base_topic = self.joint_topic = None
        # 显式指定的"车"话题（非空时只认它）
        self.forced_base_topic = forced_base_topic or ""
        # /odometry 的 child_frame_id 决定判据（gimbal_yaw = 新树 / base_link = 旧树）。
        self.base_child = None
        self.base_pose = None
        self.base_twist = None
        # raw LIO 自己的平移（判断"LIO 有没有估到雷达绕云台轴的平移"）
        self.raw_pose = None
        # 下位机回传的车速（独立于 LIO，用来确认"车真的没动"）
        self.board_cmd = None
        self.raw_yaw = self.base_yaw = self.joint_yaw = None
        # 完整四元数（w,x,y,z）：只有 yaw 的话，30° 斜装会让"xy 投影半径"本身带歧义，
        # 事后复算外参必须靠四元数（CSV 里新增了 rq*/bq* 四列）。
        self.raw_quat = self.base_quat = None
        self.raw_child = None
        self.raw_stamp = self.base_stamp = self.joint_stamp = None
        self.counts = {topic: 0 for topic in RAW_CANDIDATES + BASE_CANDIDATES}
        self.counts[JOINT_TOPIC] = 0
        self.counts[JOINT_FALLBACK_TOPIC] = 0
        self._warned_joint_name = False
        # TF：只为了在采集结束时量一次"模型里的臂杆" T(base_child <- 雷达)（静态外参），
        # 判读要拿它当"物理上应有的臂长"。查不到不影响采集，只是判读少一个参照。
        self.tf_buffer = None
        self.tf_listener = None
        try:
            from tf2_ros import Buffer, TransformListener

            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self.node)
        except Exception:  # noqa: BLE001 - 没装 tf2_ros 时降级
            self.tf_buffer = None
        # raw 到达时回调一次：用"雷达算法的输出"当采样时钟，比固定定时器干净
        # （定时器和 10 Hz 里程计不同步时会出现"这拍没新数据、下拍跳两步"的锯齿）。
        self.on_raw = None

        for topic in RAW_CANDIDATES:
            self.node.create_subscription(
                Odometry, topic, lambda msg, t=topic: self._odom(msg, t, "raw"), qos
            )
        for topic in BASE_CANDIDATES:
            self.node.create_subscription(
                Odometry, topic, lambda msg, t=topic: self._odom(msg, t, "base"), qos
            )
        self.node.create_subscription(JointState, JOINT_TOPIC, self._joint_state, qos)
        # 下位机的车速反馈（TwistStamped）：云台转、车没动时它应该≈0。
        # 用它把"LIO 在编速度"和"车真的在动"区分开。
        self.node.create_subscription(
            TwistStamped, BOARD_SPEED_TOPIC, self._board_speed, qos)
        self.node.create_subscription(Float32, JOINT_FALLBACK_TOPIC, self._joint_float, qos)

    # ---- 回调 ----
    def _stamp(self):
        return self.node.get_clock().now()

    def _odom(self, msg, topic, which):
        self.counts[topic] = self.counts.get(topic, 0) + 1
        if which == "base" and self.forced_base_topic and topic != self.forced_base_topic:
            return
        current = getattr(self, which + "_topic")
        if which == "base" and self.forced_base_topic == topic:
            current = None  # 强制话题：直接锁，不走"防抖切换"
        # 2026-11-09：/lidar_odometry 是**雷达**位姿，锁到它上面对"车在不在动"毫无意义。
        # 所以只要 /odometry 出现，就从 /lidar_odometry 换过去（无条件）。
        if which == "base" and current == "/lidar_odometry" and topic == "/odometry":
            current = None
        # 已经锁定一条就别来回切；只有当前这条断流超过 2 s 才允许换源。
        if current is not None and current != topic:
            age = self.age(getattr(self, which + "_stamp"))
            if age < 2.0:
                return
        setattr(self, which + "_topic", topic)
        orientation = msg.pose.pose.orientation
        setattr(self, which + "_yaw", yaw_of(orientation))
        setattr(self, which + "_quat",
                (orientation.w, orientation.x, orientation.y, orientation.z))
        setattr(self, which + "_stamp", self._stamp())
        position = msg.pose.pose.position
        if which == "base":
            self.base_child = msg.child_frame_id
            self.base_pose = (position.x, position.y)
            twist = msg.twist.twist.linear
            self.base_twist = (twist.x, twist.y)
        else:
            self.raw_pose = (position.x, position.y)
            self.raw_child = msg.child_frame_id
        if which == "raw" and self.on_raw is not None:
            self.on_raw()

    def _board_speed(self, msg):
        self.counts[BOARD_SPEED_TOPIC] = self.counts.get(BOARD_SPEED_TOPIC, 0) + 1
        self.board_cmd = (msg.twist.linear.x, msg.twist.linear.y)

    def _joint_state(self, msg):
        self.counts[JOINT_TOPIC] += 1
        if JOINT_NAME not in msg.name:
            if not self._warned_joint_name:
                print(
                    "[警告] %s 里没有关节 %s（现有: %s）"
                    % (JOINT_TOPIC, JOINT_NAME, ",".join(msg.name)),
                    flush=True,
                )
                self._warned_joint_name = True
            return
        if self.joint_topic != JOINT_TOPIC:
            self.joint_topic = JOINT_TOPIC
        self.joint_yaw = wrap(msg.position[msg.name.index(JOINT_NAME)])
        self.joint_stamp = self._stamp()

    def _joint_float(self, msg):
        self.counts[JOINT_FALLBACK_TOPIC] += 1
        if self.joint_topic == JOINT_TOPIC:  # relay 在发时以 JointState 为准
            return
        self.joint_topic = JOINT_FALLBACK_TOPIC
        self.joint_yaw = wrap(float(msg.data))
        self.joint_stamp = self._stamp()

    # ---- 工具 ----
    def age(self, stamp):
        if stamp is None:
            return float("inf")
        return (self.node.get_clock().now() - stamp).nanoseconds / 1e9

    def ready(self):
        return None not in (self.raw_yaw, self.base_yaw, self.joint_yaw)

    def mount(self):
        """量一次模型里的 T(base_child <- 雷达)（静态外参）；拿不到返回 None。

        返回 ``(rot 3x3 行主序 list, trans 3 元组)``。它和 zlio_bridge /
        sensor_scan_generation 用的是**同一条** TF，所以判读里的"模型臂长"就是运行时
        真正在用的那个数（不再写死 0.1335）。
        """
        if self.tf_buffer is None:
            print("[警告] 没有 tf2_ros，量不到模型外参（判读里「模型臂长」会缺失）", file=sys.stderr)
            return None
        if not self.base_child or not self.raw_child:
            print("[警告] 还没拿到 base/raw 的 child_frame_id，量不到模型外参", file=sys.stderr)
            return None
        if self.base_child == self.raw_child:
            # base 锁到了 /lidar_odometry（child=雷达帧）时两者同名，量出来是单位变换，
            # 报"臂长 0"只会误导 —— 直接说量不到。
            print("[警告] base 用的是雷达位姿（child=%s），量不到臂杆；请用 /odometry"
                  % self.base_child, file=sys.stderr)
            return None
        from rclpy.time import Time as RosTime

        try:
            stamped = self.tf_buffer.lookup_transform(
                self.base_child, self.raw_child, RosTime())
        except Exception as error:  # noqa: BLE001 - 查不到只是判读少一个参照
            print("[警告] 量 T(%s ← %s) 失败: %s" % (self.base_child, self.raw_child, error),
                  file=sys.stderr)
            return None
        translation = stamped.transform.translation
        rotation = stamped.transform.rotation
        return (
            quat_to_matrix((rotation.w, rotation.x, rotation.y, rotation.z)),
            (translation.x, translation.y, translation.z),
        )

    def spin_once(self, timeout=0.02):
        self.rclpy.spin_once(self.node, timeout_sec=timeout)

    def shutdown(self):
        self.node.destroy_node()
        self.rclpy.shutdown()


# ----------------------------------------------------------------- 分析


def _corr(a, b):
    """相关系数；任一路完全不动时 std=0，直接给 nan（numpy 会刷 RuntimeWarning）。"""
    import numpy as np

    if float(np.std(a)) < 1e-9 or float(np.std(b)) < 1e-9:
        return float("nan")
    return float(np.corrcoef(a, b)[0, 1])


def quat_to_matrix(q):
    """(w, x, y, z) -> 3x3 行主序 list（判读用，不依赖 numpy）。"""
    w, x, y, z = [float(v) for v in q]
    norm = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ]


def _locked_vector(points, theta, center=None):
    """把 xy 轨迹按云台转角 θ 做**锁相分解**，返回 (锁相分量 d, 锁相残差 rms, 平均半径)。

    车不动、云台绕轴转时，任何"跟着云台转"的向量都满足

        p(θ) = c₀ + R(−θ)·d          （云台世界 yaw φ = −θ）

    `d` 就是那个常量向量，表达在 **θ=0 的云台系**（= 模型里的 gimbal_yaw 零位系）；
    `c₀` 是圆的圆心（与"车有没有真的动"无关的那部分常量）。

    ⚠ 不能"先减质心再平均"：云台只往返扫一段时，质心不在圆心，振幅会被吃掉
    （2026-11-13 实测：±0.6 rad 的往返扫掠上，先减质心只剩 12% 的振幅）。所以这里
    把 c₀ 和 d 一起做一次线性最小二乘 —— 对"转满圈"和"只扫一段"都成立。
    """
    import numpy as np

    points = np.asarray(points, dtype=float)
    theta = np.asarray(theta, dtype=float)
    if center is not None:
        delta = points - np.asarray(center, dtype=float)
        c0 = np.asarray(center, dtype=float)
    else:
        rows = np.zeros((2 * points.shape[0], 4))
        rhs = np.zeros(2 * points.shape[0])
        for index, (point, angle) in enumerate(zip(points, theta)):
            cos_a, sin_a = math.cos(-angle), math.sin(-angle)
            rows[2 * index] = (1.0, 0.0, cos_a, -sin_a)
            rows[2 * index + 1] = (0.0, 1.0, sin_a, cos_a)
            rhs[2 * index:2 * index + 2] = point
        solution, *_ = np.linalg.lstsq(rows, rhs, rcond=None)
        c0 = solution[:2]
        delta = points - c0
    rotated = None if center is None else np.array([
        [math.cos(-angle) * delta[i, 0] - math.sin(-angle) * delta[i, 1],
         math.sin(-angle) * delta[i, 0] + math.cos(-angle) * delta[i, 1]]
        for i, angle in enumerate(theta)
    ])
    if rotated is None:
        d = np.asarray(solution[2:])
    else:
        d = rotated.mean(axis=0)
    residual = delta - np.array([
        [math.cos(-angle) * d[0] - math.sin(-angle) * d[1],
         math.sin(-angle) * d[0] + math.cos(-angle) * d[1]]
        for angle in theta
    ])
    rms = float(np.linalg.norm(residual, axis=1).std())
    return np.asarray(d), rms, float(np.linalg.norm(delta, axis=1).mean())


def analyse(samples, mount=None):
    """用逐样本增量做回归，得到 s_base / s_raw；对"转过去再转回来"同样有效。"""
    import numpy as np

    if len(samples) < 5:
        return None

    t = np.array([s["t"] for s in samples])
    # ⚠ 必须先 unwrap：云台快转一圈以上时，已 wrap 的角度做 diff 会出现 ±360 的
    # 假增量，回归斜率会被彻底带偏（2026-11-09 实测：unwrap 前 s_base_vs_raw=0.45，
    # unwrap 后 1.0026）。theta/raw/base 都是"角度（度）"。
    # 样本里是**弧度**（yaw_of 的返回），所以 unwrap 的 period 是 2π。
    theta = np.unwrap(np.array([s["theta"] for s in samples]), period=2 * np.pi)
    raw = np.unwrap(np.array([s["raw"] for s in samples]), period=2 * np.pi)
    base = np.unwrap(np.array([s["base"] for s in samples]), period=2 * np.pi)

    dt = np.diff(t)
    d_theta = np.diff(theta)
    d_raw = np.diff(raw)
    d_base = np.diff(base)

    # 只用"相邻两个样本间隔正常"的增量：卡顿后跳一大步会污染斜率。
    good = (dt > 0.0) & (dt < 0.5)
    if good.sum() < 5:
        return None
    d_theta, d_raw, d_base = d_theta[good], d_raw[good], d_base[good]

    energy = float(np.sum(d_theta ** 2))
    used = {"n": int(good.sum()), "energy": energy}
    used["travel_deg"] = float(np.degrees(np.sum(np.abs(d_theta))))
    used["t_start"], used["t_end"] = float(t[0]), float(t[-1])

    used["rot_ratio"] = (
        float((base[-1] - base[0]) / (raw[-1] - raw[0]))
        if abs(raw[-1] - raw[0]) > 1e-9 else None
    )
    used["raw_per_theta"] = (
        float((raw[-1] - raw[0]) / (theta[-1] - theta[0]))
        if abs(theta[-1] - theta[0]) > 1e-9 else None
    )
    if energy < 1e-6:
        used["s_base"] = used["s_raw"] = None
        return used

    used["s_base"] = float(np.sum(d_base * d_theta) / energy)
    used["s_raw"] = float(np.sum(d_raw * d_theta) / energy)
    raw_energy = float(np.sum(d_raw ** 2))
    # 新树：odom->gimbal_yaw 该跟着 raw 走（斜率 1），而不是跟着关节角走。
    used["s_base_vs_raw"] = (
        float(np.sum(d_base * d_raw) / raw_energy) if raw_energy > 1e-6 else None
    )
    # 新树：raw − base 是纯外参常数，波动越小越好；关节角混进来时它会跟着 θ 走。
    resid_new = np.degrees(raw - base)
    used["resid_span_deg"] = float(np.max(resid_new) - np.min(resid_new))
    # 新树：云台绕轴转时车不该"走"——xy 跨度与 /odometry 线速度都要接近 0。
    poses = np.array([[s["bx"], s["by"]] for s in samples if s.get("bx") is not None])
    if len(poses) > 1:
        used["base_xy_span"] = float(np.max(np.linalg.norm(poses - poses[0], axis=1)))
        centre = poses.mean(axis=0)
        used["base_xy_radius"] = float(np.mean(np.linalg.norm(poses - centre, axis=1)))
        # ---- 锁相分解（判"平移"只看这一组；xy 跨度/圆拟合半径都太粗）----
        theta_pts = np.unwrap(
            np.array([s["theta"] for s in samples if s.get("bx") is not None]),
            period=2 * np.pi,
        )
        c_base, rms_base, mean_base = _locked_vector(poses, theta_pts)
        used["base_locked_xy"] = (float(c_base[0]), float(c_base[1]))
        used["base_locked_radius"] = float(np.linalg.norm(c_base))
        used["base_locked_azimuth_deg"] = float(
            np.degrees(np.arctan2(c_base[1], c_base[0])))
        used["base_locked_rms"] = rms_base
        used["base_locked_mean_radius"] = mean_base
        # 与云台相位是否"锁相"：圆心相对向量的角度 vs 云台转过的角度。
        # 斜率≈±1 说明它在跟着云台画圆（LIO 漏掉了绕轴平移）；
        # 斜率≈0 或散乱说明是漂移/噪声。
        rel = poses - centre
        phase = np.degrees(np.unwrap(np.arctan2(rel[:, 1], rel[:, 0])))
        phi = np.degrees(-theta)  # 云台 yaw（度）与关节角反号（底盘静止时）；
        # 必须与 phase 同单位，否则斜率会差一个 57.3 的因子。
        phi0 = phi - phi[0]
        energy = float(np.sum((phi0 - phi0.mean()) ** 2))
        if energy > 1e-6:
            used["xy_phase_slope"] = float(
                np.sum((phase - phase.mean()) * (phi0 - phi0.mean())) / energy
            )
        else:
            used["xy_phase_slope"] = None
    else:
        used["base_xy_span"] = None
        used["base_xy_radius"] = None
        used["xy_phase_slope"] = None
    speeds = [
        float(np.hypot(s["bvx"], s["bvy"]))
        for s in samples
        if s.get("bvx") is not None
    ]
    used["base_v_max"] = max(speeds) if speeds else None

    # ---- raw LIO 自己的平移：云台绕轴转时，雷达**物理上**在以 0.1335 m 半径画圆。
    #      LIO 若估到了，raw 的 xy 就该是一个半径≈0.1335 m、与云台锁相的圆；
    #      若半径≈0，说明 LIO 把雷达当成"原地转"，odom→gimbal 必然多出一个圆。
    rawpts = np.array([[s["rx"], s["ry"]] for s in samples if s.get("rx") is not None])
    if len(rawpts) > 1:
        c = rawpts.mean(axis=0)
        used["raw_xy_radius"] = float(np.mean(np.linalg.norm(rawpts - c, axis=1)))
        # 锁相分解：|c| = 前端**自己**估到的臂长（与任何外参无关，靠转角锁相测出来）
        theta_raw = np.unwrap(
            np.array([s["theta"] for s in samples if s.get("rx") is not None]),
            period=2 * np.pi,
        )
        c_raw, rms_raw, _ = _locked_vector(rawpts, theta_raw)
        used["raw_locked_xy"] = (float(c_raw[0]), float(c_raw[1]))
        used["raw_locked_radius"] = float(np.linalg.norm(c_raw))
        used["raw_locked_azimuth_deg"] = float(
            np.degrees(np.arctan2(c_raw[1], c_raw[0])))
        used["raw_locked_rms"] = rms_raw
        rel = rawpts - c
        phase = np.degrees(np.unwrap(np.arctan2(rel[:, 1], rel[:, 0])))
        phi0 = np.degrees(-theta) - np.degrees(-theta[0])
        energy = float(np.sum((phi0 - phi0.mean()) ** 2))
        used["raw_xy_phase_slope"] = (
            float(np.sum((phase - phase.mean()) * (phi0 - phi0.mean())) / energy)
            if energy > 1e-6 else None
        )
    else:
        used["raw_xy_radius"] = None
        used["raw_xy_phase_slope"] = None
    boards = [float(np.hypot(s["cvx"], s["cvy"])) for s in samples if s.get("cvx") is not None]
    used["board_v_max"] = max(boards) if boards else None
    # 累积转角比：⚠ 端点比值在"转过去再转回来"时毫无意义（净转角≈0 → 比值 ±50），
    # 改成**行程比** Σ|d_base|/Σ|d_raw|：单方向时与端点比值等价，往返时仍然有效
    # （两者都按 1:1 跟随时，往返的 |增量| 之和也相等）。
    raw_total = float(raw[-1] - raw[0])
    base_total = float(base[-1] - base[0])
    theta_total = float(theta[-1] - theta[0])
    used["raw_total_deg"] = np.degrees(raw_total)
    used["base_total_deg"] = np.degrees(base_total)
    rot_ratio_net = (base_total / raw_total) if abs(raw_total) > 1e-6 else None
    used["rot_ratio_net"] = rot_ratio_net
    travel_raw = float(np.sum(np.abs(d_raw)))
    travel_base = float(np.sum(np.abs(d_base)))
    # 单方向转：用**端点比**（Σ|d| 会被雷达 30° 斜装带来的 yaw 非线性和采样噪声抬高，
    #   2026-11-13 实测抬了 8%，把本来 PASS 的判成 FAIL）；
    # 往返转：端点比 ≈ 0/0 无意义，只能用行程比（此时两者按 1:1 跟随，|增量| 之和也相等）。
    net_ratio = (abs(raw_total) / travel_raw) if travel_raw > 1e-9 else 0.0
    if net_ratio >= 0.5:
        used["rot_ratio"] = rot_ratio_net
        used["rot_ratio_kind"] = "net"
    else:
        used["rot_ratio"] = (travel_base / travel_raw) if travel_raw > 1e-9 else rot_ratio_net
        used["rot_ratio_kind"] = "travel"
    used["raw_per_theta"] = (raw_total / theta_total) if abs(theta_total) > 1e-6 else None

    residual = np.degrees(raw - theta - base)  # 应当是一条水平线
    used["residual_deg"] = float(np.mean(residual))
    used["residual_span_deg"] = float(np.max(residual) - np.min(residual))
    resid_new = np.degrees(raw - base)
    resid_new = resid_new - resid_new[0]
    used["resid_span_deg"] = float(np.max(resid_new) - np.min(resid_new))
    used["base_dev_deg"] = float(np.degrees(np.max(np.abs(base - base[0]))))
    used["raw_dev_deg"] = float(np.degrees(np.max(np.abs(raw - raw[0]))))
    used["corr_raw"] = _corr(d_raw, d_theta)
    used["corr_base"] = _corr(d_base, d_theta)
    # ---- 外参（模型里的臂杆）：从 TF 量一次，用来区分"外参错"和"前端错" ----
    # 残差 c_base = a_前端 − a_外参（都在云台零位系里）。所以：
    #   * |c_base| 就是两者的失配量；单看采集**分不出**是谁错（2026-11-13 的教训）；
    #   * 但把 a_前端 = a_外参 + c_base 反解出来，就能和前端的绕轴半径（raw 锁相 |c|）对照，
    #     也能直接算出"要补偿多少"给 zlio_bridge 的 pose_body_offset_xyz。
    if mount is not None:
        rot, trans = mount
        used["arm_model_xyz"] = [float(v) for v in trans]
        used["arm_model_radius"] = float(math.hypot(trans[0], trans[1]))
        used["arm_model_azimuth_deg"] = float(
            math.degrees(math.atan2(trans[1], trans[0])))
        if "base_locked_xy" in used:
            d = np.array([used["base_locked_xy"][0], used["base_locked_xy"][1], 0.0])
            # pose_pub = pose · T(offset)：令 odom→gimbal_yaw 的锁相分量归零
            #   ⇒ offset = −R_Aᵀ · 残差(水平)   （R_A = T(云台←雷达) 的旋转）
            offset = -np.asarray(rot).T @ d
            used["suggested_offset"] = [float(v) for v in offset]
            a_lio = np.array([trans[0], trans[1], 0.0]) + d
            model_r = used["arm_model_radius"]
            if model_r > 1e-6:
                used["arm_lio_ratio"] = float(np.linalg.norm(a_lio[:2]) / model_r)
                used["arm_angle_deg"] = float(np.degrees(np.arctan2(
                    trans[0] * a_lio[1] - trans[1] * a_lio[0],
                    trans[0] * a_lio[0] + trans[1] * a_lio[1])))

    used["samples"] = samples
    return used


def _verdict_gimbal_tree(stats, tol_deg):
    """新树（nav 的"车" = 云台）的判据。

    设计要点（方案 §4.1 + 2026-11-08 补漏）：云台绕轴转时，
      * `odom -> gimbal_yaw` 的 yaw **必须**跟着 raw LIO 走（斜率 1，残差是外参常数）；
      * 关节角**不许**再进这一项（否则就是还有一处减了 θ）；
      * 云台在轴上转，车不该"走"：xy 跨度与 /odometry 线速度都要接近 0
        （LIO 若把"雷达转动"当成"车在动"，xy 会画出半径≈雷达偏轴 0.133 m 的圆、
         线速度出现 0.2~0.3 m/s 的假值 —— 正是 2026-11-08 现场那次的症状）。
    """
    s_raw = stats["s_raw"]
    s_br = stats.get("s_base_vs_raw")
    rot_ratio = stats.get("rot_ratio")
    resid_span = stats.get("resid_span_deg", float("nan"))
    xy_span = stats.get("base_xy_span")
    v_max = stats.get("base_v_max")
    # 判据提前算：列表和早退分支都要用。
    # 2026-11-13：平移判据从"xy 跨度 < 0.02 m"换成**锁相残差半径**。老判据在这套链上
    # 根本达不到 —— 就算外参完全正确，前端自己的绕轴平移误差（实测只估到物理臂长的
    # 60%~92%）也会留下 0.02~0.06 m 的残差。阈值默认 0.04 m，可用 --residual-tol 调。
    residual_r = stats.get("base_locked_radius")
    tol = float(stats.get("residual_tol", 0.04))
    pose_ok = rot_ratio is not None and abs(rot_ratio - 1.0) <= 0.03
    if residual_r is not None:
        motion_ok = residual_r < tol
    else:
        # 旧 CSV（没有锁相值）：退回老判据，只做参考
        motion_ok = (xy_span is None or xy_span < 0.02) and (v_max is None or v_max < 0.03)

    lines = [
        "  树: odom -> %s（**新树：云台就是车**）" % stats.get("base_child"),
        "  s_raw        = %+.2f   （理论 |1|：云台转多少，raw LIO 就该转多少；"
        "底盘静止、只转云台时对 θ 是 -1）" % s_raw,
        "  s_base_vs_raw= %+.2f   （逐样本回归，仅供参考；多圈旋转下会被采样抖动衰减）"
        % (s_br if s_br is not None else float("nan")),
        "  转角比 base/raw = %+.4f（%s；**判据**：应 ≈ 1，容差 0.03）"
        % (rot_ratio if rot_ratio is not None else float("nan"),
           "端点比" if stats.get("rot_ratio_kind") == "net" else "行程比"),
        "  残差 raw-base 摆动 %.1f°（**正常就该摆动**：raw 是雷达 yaw、base 是云台 yaw，"
        "雷达斜装 30°）" % resid_span,
        "  云台累计转角 %.1f°，raw 最大偏移 %.1f°（odom 的偏移应当与它同量级："
        "%.1f°）" % (stats["travel_deg"], stats["raw_dev_deg"], stats["base_dev_deg"]),
    ]
    if stats.get("raw_xy_radius") is not None:
        if stats.get("raw_locked_radius") is not None:
            lines.append("  前端自己的绕轴臂长 %.4f m（锁相 rms %.4f m；模型臂长 %s）"
                         % (stats["raw_locked_radius"],
                            stats.get("raw_locked_rms", float("nan")),
                            ("%.4f m" % stats["arm_model_radius"])
                            if stats.get("arm_model_radius") is not None else "未知"))
        else:
            lines.append("  raw LIO 的雷达平移半径 %.4f m（≈0 就是 LIO 把雷达当成原地转）"
                         % stats["raw_xy_radius"])
    if stats.get("board_v_max") is not None:
        lines.append("  下位机 /chassis_speed 的车速峰值 %.3f m/s（车其实没动时应≈0）"
                     % stats["board_v_max"])
    if stats.get("base_locked_radius") is not None:
        lines.append("  odom->gimbal_yaw **锁相残差** %.4f m（判据 < %.3f m），方位 %+.0f°，"
                     "锁相 rms %.4f m"
                     % (stats["base_locked_radius"], tol,
                        stats.get("base_locked_azimuth_deg", float("nan")),
                        stats.get("base_locked_rms", float("nan"))))
    if xy_span is not None:
        lines.append("  参考：xy 跨度 %.3f m / 圆拟合半径 %.3f m / 相位斜率 %s"
                     % (xy_span, stats.get("base_xy_radius") or float("nan"),
                        ("%+.2f" % stats["xy_phase_slope"])
                        if stats.get("xy_phase_slope") is not None else "nan"))
    if v_max is not None:
        lines.append("  /odometry 线速度峰值 %.3f m/s（≈ ω×锁相残差，与残差判据等价；"
                     "不单独判）" % v_max)

    if xy_span is None or v_max is None:
        if pose_ok:
            return 3, lines + [
                "[结论] 旋转这一半已经过了（base/raw=%.4f），但**没有平移数据**，判不了。" % rot_ratio,
                "       这份 CSV 是旧版本录的（没有 base_x/base_y 列）。用当前版本重录一次：",
                "       ./debug_tools/capture_gimbal_spin.py --duration 40 --no-plot",
            ]
        return 1, lines + [
            "[结论] FAIL：base/raw=%.4f（应 ≈1），odom->gimbal_yaw 没完整继承雷达位姿。"
            % (rot_ratio if rot_ratio is not None else float("nan")),
        ]

    if abs(s_raw) < 0.35:
        return 1, lines + [
            "[结论] FAIL：raw 几乎不跟云台转 → LIO 没在跟踪，先查数据链路，与坐标系无关。",
            "       看: ros2 topic hz /livox/lidar /livox/imu %s"
            % (stats.get("raw_topic", "")),
        ]

    # ⚠ 残差不要当判据：雷达斜装 30°，`raw` 是**雷达**的 yaw、`base` 是**云台**的 yaw，
    #   两者之差天然随转角摆动几十度（实测峰峰 ~56°）。它只用来"看形状"，不用来判死活。

    if pose_ok and motion_ok:
        return 0, lines + [
            "[结论] PASS：云台转了 %.0f°，odom->gimbal_yaw 一比一跟上，且关节角没进这一项。"
            % stats["travel_deg"],
            "       车的平移没有假速度、xy 没画圈 —— 方案 §6.2 第 1 步通过。",
        ]

    if not pose_ok and xy_span is not None and xy_span < 0.02 and stats.get('s_base') is not None \
            and abs(stats['s_base']) > 0.35:
        return 1, lines + [
            "[结论] FAIL：yaw 跟上了，但残差随关节角变化 → 还有一处把关节角减掉了。",
            "       查: 是不是跑的还是旧参数/旧二进制（sensor_scan_generation 的",
            "       robot_base_frame 应为 gimbal_yaw、publish_robot_base_tf 应为 true）。",
        ]

    if s_br is not None and abs(s_br) <= 0.35:
        # 分两种：base 完全不动（没跟 raw）vs base 跟着 −θ 走（还减了一处关节角）。
        if abs(stats["s_base"]) > 0.35:
            return 1, lines + [
                "[结论] FAIL：odom->gimbal_yaw 跟着「负的关节角」在走（s_base=%+.2f）。"
                % stats["s_base"],
                "       说明还有一处把关节角减掉了（旧逻辑/旧配置仍在跑）：",
                "       确认 sensor_scan_generation 的 robot_base_frame=gimbal_yaw、",
                "       publish_robot_base_tf=true，且用的是本次编译的二进制。",
            ]
        return 1, lines + [
            "[结论] FAIL：odom->gimbal_yaw 没跟云台转（s_base_vs_raw≈0）。",
            "       说明 TF 反转没生效、或 /odometry 还是底盘位姿（sensor_scan_generation",
            "       的 robot_base_frame 不是 gimbal_yaw / publish_robot_base_tf 没开）。",
        ]

    if s_br is not None and s_br < 0.65:
        return 1, lines + [
            "[结论] FAIL：odom->gimbal_yaw 只跟了一部分（s_base_vs_raw=%+.2f）。" % s_br,
            "       典型是「跟了 raw、又减了一半关节角」：查是不是同时开了两处补偿。",
        ]

    if not motion_ok:
        detail = [
            "[结论] FAIL：yaw 对了，但 odom->gimbal_yaw 还在画圈"
            "（锁相残差 %.3f m，阈值 %.3f m）。"
            % (residual_r if residual_r is not None else float("nan"), tol),
        ]
        if stats.get("base_locked_xy") is not None:
            detail.append(
                "       残差向量（云台零位系）(%+.3f, %+.3f) m，方位 %+.0f°，锁相 rms %.3f m"
                % (stats["base_locked_xy"][0], stats["base_locked_xy"][1],
                   stats.get("base_locked_azimuth_deg", float("nan")),
                   stats.get("base_locked_rms", float("nan"))))
        if stats.get("raw_locked_radius") is not None:
            detail.append(
                "       前端自己估到的绕轴臂长 %.3f m（锁相 rms %.3f m）"
                % (stats["raw_locked_radius"], stats.get("raw_locked_rms", float("nan"))))
        if stats.get("arm_model_radius") is not None:
            detail.append(
                "       模型里的臂长 %.3f m、方位 %+.0f°；残差与模型臂差 %+.0f°"
                % (stats["arm_model_radius"],
                   stats.get("arm_model_azimuth_deg", float("nan")),
                   stats.get("arm_angle_deg", float("nan"))))
        detail += [
            "       ⚠ 残差 = |a_前端 − a_外参|，**单凭本采集分不出**是外参错还是前端错。",
            "       两步定性（顺序别反）：",
            "       ① 验外参：停车对着墙/箱子，RViz 固定帧 odom，看 /sensor_scan 里的墙与",
            "          实物的方位是否一致；一致 ⇒ 外参对，残差属于前端。",
            "       ② 验前端：同模型下换 --lio-frontend point_lio 再采一次，圆变了就是前端的。",
        ]
        if stats.get("suggested_offset") is not None:
            off = stats["suggested_offset"]
            detail += [
                "       若①已确认外参正确：把残差当作前端的随体偏置补掉（位姿+点云一起改，",
                "       链条自洽、避障不受影响；但点云相对旧地图会变 ⇒ 要重跑一次建图，",
                "       或先用 --reloc 让 GICP 重新对齐）—— 加到实车 nav2 参数的 zlio_bridge 段：",
                "         pose_body_offset_xyz: [%.4f, %.4f, %.4f]" % (off[0], off[1], off[2]),
                "       ⚠ 先多跑 2~3 次确认稳定（2026-11-13 两次实测：前端臂 0.104@+152° /",
                "         0.125@+156° ⇒ 稳定，差异是 LIO 自身幅度噪声）；散得很开就别补，去修前端。",
                "       ⚠ 别设 pose_body_offset_apply_to_cloud=false 当长期配置：那只改位姿，",
                "         障碍物会相对车位姿永久偏 |offset|（≈0.3 m、随车头转），会破坏避障。",
                "       改完再采一次，锁相残差应 < %.3f m。" % tol,
            ]
        return 1, lines + detail

    return 1, lines + [
        "[结论] FAIL：s_base_vs_raw=%+.2f、残差波动 %.1f°，不是单一路径的问题。"
        % (s_br if s_br is not None else float("nan"), resid_span),
        "       常见原因: 关节角延迟/不同周期、LIO 自己在漂、云台角被算了两遍。",
    ]


def verdict(stats, tol_deg, min_travel_deg):
    """返回 (退出码, [结论行])。"""
    if stats is not None:
        child = str(stats.get("base_child") or "")
        topic = str(stats.get("base_topic") or "")
        if child.lower().startswith(("livox", "front_mid360")) or \
                (topic == "/lidar_odometry" and child not in ("gimbal_yaw", "base_link")):
            return 3, [
                "[结论] 判据失效：base 选到了**雷达**位姿（%s，child=%s），不是\"车\"。" % (topic, child),
                "       云台转时雷达本来就要绕轴画圆，拿它判\"车有没有动\"必然 FAIL。",
                "       用 --base-topic /odometry 重跑，或先启动导航栈让 /odometry 起来。",
            ]
    if stats is None or stats.get("s_base") is None:
        return 3, [
            "[结论] 没采到有效数据：三路要同时有数据，且云台要真的转起来。",
            "       自查: ros2 topic hz /odometry  /joint_states  /aft_mapped_to_init",
        ]

    if stats["travel_deg"] < min_travel_deg:
        return 3, [
            "[结论] 云台累计只转了 %.1f°（门槛 %.0f°），测不出东西。"
            % (stats["travel_deg"], min_travel_deg),
            "       让云台慢慢转 30~45°（转回来也行）再跑一次。",
        ]

    s_base = stats["s_base"]
    s_raw = stats["s_raw"]
    base_child = str(stats.get("base_child") or "base_link")
    gimbal_tree = base_child == "gimbal_yaw" or base_child.endswith("_fake")

    if gimbal_tree:
        return _verdict_gimbal_tree(stats, tol_deg)

    lines = [
        "  树: odom -> %s（旧树：底盘就是车）" % base_child,
        "  s_base = %+.2f   （理论 0：0.0 对、1.0 没减关节角、2.0 符号反）" % s_base,
        "  s_raw  = %+.2f   （理论 +1：云台转多少，raw LIO 就该转多少）" % s_raw,
        "  残差 raw-θ-base 波动 %.1f°（理论恒定；它跟着转就说明三路没对齐）"
        % stats["residual_span_deg"],
        "  云台累计转角 %.1f°，raw 最大偏移 %.1f°，base 最大偏移 %.1f°（容差 %.1f°）"
        % (stats["travel_deg"], stats["raw_dev_deg"], stats["base_dev_deg"], tol_deg),
    ]

    if abs(s_raw) < 0.35:
        return 1, lines + [
            "[结论] FAIL：raw 几乎不跟云台转 → LIO 没在跟踪，先查数据链路，与补偿无关。",
            "       看: ros2 topic hz /livox/lidar  /livox/imu  %s" % (stats.get("raw_topic", "")),
        ]

    if abs(s_base) <= 0.35 and abs(s_raw - 1.0) <= 0.35:
        return 0, lines + [
            "[结论] PASS：云台转了 %.0f°，raw 跟着转了，底盘 yaw 基本没动。"
            % stats["travel_deg"],
            "       关节角进了 TF 且符号正确，gimbal_joint_sign 不用改。",
        ]

    if 0.65 <= s_base <= 1.35:
        return 1, lines + [
            "[结论] FAIL：s_base ≈ 1 → TF 里**没有云台关节角**（补偿完全没发生）。",
            "       查: /joint_states 有没有 gimbal_yaw_joint、relay 是否超时停发、",
            "       robot_state_publisher 是不是 gimbal_joint_type:=revolute。",
        ]

    if 1.65 <= s_base <= 2.35:
        return 1, lines + [
            "[结论] FAIL：s_base ≈ 2 → **关节角符号反了**（减成了加）。",
            "       改 srm27_nav_protocol.yaml 的 gimbal_joint_sign（+1.0 <-> -1.0）后重启链路。",
        ]

    if abs(s_base) <= 0.35:
        return 1, lines + [
            "[结论] FAIL：base 没怎么动，但 raw 与关节角不同步（s_raw=%+.2f）。" % s_raw,
            "       查: 关节轴与云台是否同轴 / 雷达外参 roll-pitch / 点云与 IMU 时间戳。",
        ]

    return 1, lines + [
        "[结论] FAIL：s_base=%+.2f 不是 0/1/2 里的任何一个，不是单纯的符号或缺失。"
        % s_base,
        "       常见原因: 关节角有延迟或按别的周期更新、LIO 自己在漂、云台角被算了两遍。",
    ]


# ----------------------------------------------------------------- 输出


def plot(stats, args, png_path, title_note):
    import matplotlib

    if args.headless or not os.environ.get("DISPLAY"):
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np

    from _common import setup_cjk_font  # 放在这里：只有要画图时才碰 matplotlib

    setup_cjk_font()
    plt.rcParams["axes.unicode_minus"] = False

    samples = stats["samples"]
    t = np.array([s["t"] for s in samples])
    theta = np.degrees(np.unwrap(np.array([s["theta"] for s in samples])))
    raw = np.degrees(np.unwrap(np.array([s["raw"] for s in samples])))
    base = np.degrees(np.unwrap(np.array([s["base"] for s in samples])))
    residual = raw - theta - base

    fig, (ax1, ax2) = plt.subplots(
        2, 1, figsize=(12, 7), sharex=True, gridspec_kw={"height_ratios": [3, 1]}
    )
    ax1.plot(t, theta, color="#2ca02c", linewidth=2.0, label="关节角 θ (gimbal_yaw_joint)")
    ax1.plot(t, raw, color="#1f77b4", linewidth=1.6, label="raw LIO (camera_init)")
    ax1.plot(t, base, color="#d62728", linewidth=1.6, label="odom -> base_link")
    ax1.set_ylabel("yaw / deg")
    ax1.grid(alpha=0.3)
    ax1.legend(loc="best", fontsize=9)
    ax1.set_title(
        "云台单转 yaw 对照  %s\nθ=%s  raw=%s  base=%s"
        % (
            title_note,
            stats.get("joint_topic", "?"),
            stats.get("raw_topic", "?"),
            stats.get("base_topic", "?"),
        ),
        fontsize=10,
    )

    ax2.plot(t, residual, color="#9467bd", linewidth=1.4)
    ax2.axhline(0.0, color="gray", linewidth=0.8, linestyle="--")
    ax2.set_ylabel("raw-θ-base / deg")
    ax2.set_xlabel("t / s")
    ax2.grid(alpha=0.3)
    ax2.set_title(
        "残差（理论恒定）。波动 %.1f°，s_base=%+.2f，s_raw=%+.2f"
        % (
            stats.get("residual_span_deg", float("nan")),
            stats.get("s_base", float("nan")),
            stats.get("s_raw", float("nan")),
        ),
        fontsize=9,
    )
    fig.tight_layout()
    fig.savefig(png_path, dpi=130, bbox_inches="tight")
    plt.close(fig)


def start_bag(out_dir, rate_topic_hint):
    topics = list(RAW_CANDIDATES) + list(BASE_CANDIDATES) + [
        JOINT_TOPIC,
        JOINT_FALLBACK_TOPIC,
        "/tf",
        "/tf_static",
    ]
    bag_dir = out_dir / "raw"
    log_path = out_dir / "bag.log"
    try:
        handle = open(log_path, "w", encoding="utf-8")
    except OSError:
        handle = subprocess.DEVNULL
    print("[录包] ros2 bag record -> %s（收尾要等它写完 metadata.yaml）" % bag_dir, flush=True)
    try:
        return subprocess.Popen(
            ["ros2", "bag", "record", "-o", str(bag_dir)] + topics,
            stdout=handle,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
    except OSError as exc:
        print("[警告] 起不了 ros2 bag record: %s（继续只记 CSV）" % exc, flush=True)
        return None


def stop_bag(proc, wait_s=25.0):
    if proc is None or proc.poll() is not None:
        return
    print("[录包] 通知收尾（SIGINT，最多等 %.0f s 写 metadata.yaml）..." % wait_s, flush=True)
    try:
        proc.send_signal(signal.SIGINT)
        proc.wait(timeout=wait_s)
        print("[录包] 已收尾。", flush=True)
    except subprocess.TimeoutExpired:
        proc.terminate()
        print("[警告] 录包进程 %.0f s 内没退完，已 terminate；缺 metadata.yaml 的包无法回放。"
              % wait_s, flush=True)
    except OSError:
        pass


# ----------------------------------------------------------------- main


def main():
    from _common import prepare_matplotlib_env

    prepare_matplotlib_env()  # 家目录只读时把 MPLCONFIGDIR 挪到 /tmp，免得刷警告

    parser = argparse.ArgumentParser(
        description="自动抓「车不动、云台转」的三路 yaw，出 CSV/曲线图/结论",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--duration", type=float, default=0.0,
                        help="采集时长（秒）；0 = 一直录到 Ctrl-C")
    parser.add_argument("--rate", type=float, default=20.0, help="采样频率（Hz）")
    parser.add_argument("--wait", type=float, default=300.0,
                        help="最多等多久让三路话题出现（秒）")
    parser.add_argument("--out-dir", default=str(DEFAULT_OUT_DIR), help="产物目录")
    parser.add_argument("--tag", default="gimbal_spin", help="产物文件名前缀")
    parser.add_argument(
        "--base-topic", default="",
        help="显式指定'车'的里程计话题（默认自动；**有 /odometry 时优先用它**）。"
             "不要填 /lidar_odometry —— 那是雷达位姿，云台转时它本来就要绕轴画圆",
    )
    parser.add_argument("--tol-deg", type=float, default=3.0,
                        help="底盘 yaw 允许偏移（只用于结论里那句话）")
    parser.add_argument("--min-travel-deg", type=float, default=15.0,
                        help="云台累计转角门槛，低于它不出结论")
    parser.add_argument("--residual-tol", type=float, default=0.04,
                        help="锁相残差半径的通过阈值（米），默认 0.04。别设成 0.02："
                             "前端自己估到的绕轴平移只有物理臂长的 60%%~92%%，"
                             "外参完全正确也会剩 2~5 cm；外参对不对要用"
                             "「点云 vs 实物」单独验。")
    parser.add_argument("--bag", action="store_true", help="同时录 rosbag（事后可复算）")
    parser.add_argument("--no-plot", action="store_true", help="不出图（车上没图形界面）")
    parser.add_argument("--headless", action="store_true", help="强制不弹窗")
    parser.add_argument("--demo", action="store_true", help="合成数据自测（不需要 ROS）")
    args = parser.parse_args()

    if args.rate <= 0 or args.wait < 0 or args.duration < 0:
        print("[错误] --rate 必须 > 0，--wait/--duration 不能为负", file=sys.stderr)
        return 2

    out_dir = Path(args.out_dir).expanduser()
    try:
        out_dir.mkdir(parents=True, exist_ok=True)
    except OSError as exc:
        print("[错误] 建不了产物目录 %s: %s" % (out_dir, exc), file=sys.stderr)
        return 2
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    base_path = out_dir / ("%s_%s" % (args.tag, stamp))

    if args.demo:
        samples = _demo_samples()
        return _finish(args, samples, base_path, {}, None, demo=True)

    try:
        import rclpy  # noqa: F401
    except ImportError:
        print("[错误] 没有 rclpy，先 source /opt/ros/humble/setup.bash 与 install/setup.bash",
              file=sys.stderr)
        return 2

    from _common import CsvLog  # 与其它 debug_tools 一致的落盘方式

    collector = Collector(args.rate, args.base_topic)
    samples = []
    csv_log = None
    bag = None
    try:
        # ---- 等三路话题就绪 ----
        print(
            "[等待] raw(%s) / base(%s) / joint(%s 或 %s)，最多 %.0f s..."
            % (" | ".join(RAW_CANDIDATES), " | ".join(BASE_CANDIDATES),
               JOINT_TOPIC, JOINT_FALLBACK_TOPIC, args.wait),
            flush=True,
        )
        deadline = time.monotonic() + args.wait
        next_status = time.monotonic()
        while not collector.ready():
            if time.monotonic() > deadline:
                print("[错误] 等不到数据，退出。", file=sys.stderr)
                print("       自查: ros2 node list | grep -E 'point_lio|zlio' ; "
                      "ros2 topic list | grep -E 'odometry|joint'", file=sys.stderr)
                return 3
            collector.spin_once(0.05)
            if time.monotonic() >= next_status:
                next_status = time.monotonic() + 3.0
                print(
                    "[等待] raw:%s base:%s joint:%s"
                    % (
                        "✓ %s" % collector.raw_topic if collector.raw_yaw is not None else "…",
                        "✓ %s" % collector.base_topic if collector.base_yaw is not None else "…",
                        "✓ %s" % collector.joint_topic if collector.joint_yaw is not None else "…",
                    ),
                    flush=True,
                )

        print(
            "[开始] raw=%s  base=%s  joint=%s  采样上限 %.0f Hz（由 raw 触发）  产物: %s.*"
            % (collector.raw_topic, collector.base_topic, collector.joint_topic,
               args.rate, base_path),
            flush=True,
        )

        csv_log = CsvLog(
            base_path.with_suffix(".csv"),
            ("t", "theta_deg", "raw_deg", "base_deg", "residual_deg",
             "base_x", "base_y", "base_vx", "base_vy", "raw_x", "raw_y",
             # 四元数（2026-11-13 加）：只有 yaw 时，30° 斜装会让"xy 投影半径"带歧义，
             # 事后复算外参/随体偏置必须靠完整姿态。
             "raw_qw", "raw_qx", "raw_qy", "raw_qz",
             "base_qw", "base_qx", "base_qy", "base_qz"),
        )
        if args.bag:
            bag = start_bag(base_path.with_suffix(""), args.rate)

        # ---- 采集 ----
        # 采样时钟 = raw 里程计的到达时刻（见 Collector.on_raw）；主循环只负责
        # spin、看 --duration、以及每 2 s 打一行状态。
        t0 = time.monotonic()
        deadline = t0 + args.duration if args.duration > 0 else None
        min_gap = 0.5 / args.rate
        state = {"last": -1e9, "next_status": t0 + 2.0}

        state_mount = {"last": -1e9, "value": None}

        def grab_mount():
            """采集中周期性抓一次静态外参（T(base<-雷达)）：收尾那一刻查经常已经晚了。"""
            now = time.monotonic()
            if state_mount["value"] is not None or now - state_mount["last"] < 2.0:
                return
            state_mount["last"] = now
            state_mount["value"] = collector.mount()

        def take_sample():
            now = time.monotonic()
            if now - state["last"] < min_gap or not collector.ready():
                return
            state["last"] = now
            sample = {
                "t": now - t0,
                "theta": collector.joint_yaw,
                "raw": collector.raw_yaw,
                "base": collector.base_yaw,
                # 采样当时用的是哪条话题：base 会在 /lidar_odometry -> /odometry 之间切换，
                # 切换前后的样本语义不同（雷达位姿 vs 云台位姿），统计时必须按最终话题过滤。
                "bt": collector.base_topic,
                "rt": collector.raw_topic,
            }
            if collector.base_pose is not None:
                sample["bx"], sample["by"] = collector.base_pose
            if collector.base_twist is not None:
                sample["bvx"], sample["bvy"] = collector.base_twist
            if collector.raw_pose is not None:
                sample["rx"], sample["ry"] = collector.raw_pose
            if collector.raw_quat is not None:
                sample["rq"] = collector.raw_quat
            if collector.base_quat is not None:
                sample["bq"] = collector.base_quat
            if collector.board_cmd is not None:
                sample["cvx"], sample["cvy"] = collector.board_cmd
            sample["residual"] = wrap(sample["raw"] - sample["theta"] - sample["base"])
            samples.append(sample)
            csv_log.write(
                (
                    "%.3f" % sample["t"],
                    "%.4f" % deg(sample["theta"]),
                    "%.4f" % deg(sample["raw"]),
                    "%.4f" % deg(sample["base"]),
                    "%.4f" % deg(sample["residual"]),
                    "%.5f" % sample.get("bx", float("nan")),
                    "%.5f" % sample.get("by", float("nan")),
                    "%.5f" % sample.get("bvx", float("nan")),
                    "%.5f" % sample.get("bvy", float("nan")),
                    "%.5f" % sample.get("rx", float("nan")),
                    "%.5f" % sample.get("ry", float("nan")),
                ) + tuple("%.6f" % v for v in (
                    sample.get("rq", (float("nan"),) * 4) + sample.get("bq", (float("nan"),) * 4)
                ))
            )
            grab_mount()
            if now >= state["next_status"]:
                state["next_status"] = now + 2.0
                print(
                    "  t=%6.1fs  θ=%8.1f°  raw=%8.1f°  base=%7.2f°  "
                    "raw-θ=%7.1f°  样本 %d"
                    % (sample["t"], deg(sample["theta"]), deg(sample["raw"]),
                       deg(sample["base"]), deg(wrap(sample["raw"] - sample["theta"])),
                       len(samples)),
                    flush=True,
                )

        collector.on_raw = take_sample

        while True:
            collector.spin_once(0.05)
            if deadline is not None and time.monotonic() >= deadline:
                break

    except KeyboardInterrupt:
        print("\n[停止] 收到 Ctrl-C，收尾中...", flush=True)
    finally:
        if csv_log is not None:
            csv_log.close()
        stop_bag(bag)
        meta = {
            "raw_topic": collector.raw_topic,
            "base_topic": collector.base_topic,
            "joint_topic": collector.joint_topic,
            "base_child": collector.base_child,
            "counts": dict(collector.counts),
        }
        # 收尾时量一次静态外参 T(gimbal_yaw <- 雷达)：判读要用"模型臂长"当参照，
        # 并据此算出建议的 pose_body_offset_xyz。查不到只是少这一块，不影响结论。
        try:
            # 静态 TF 要等订阅匹配上（实测 0.2 s 不够，2~3 s 才稳）
            deadline_mount = time.monotonic() + 3.0
            while time.monotonic() < deadline_mount:
                collector.spin_once(0.02)
            meta["mount"] = state_mount["value"] or collector.mount()
        except Exception:  # noqa: BLE001
            meta["mount"] = None
        try:
            collector.shutdown()
        except Exception:  # noqa: BLE001 关闭失败不该影响出结论
            pass
        rc = _finish(args, samples, base_path, meta, None)
    return rc


def _demo_samples():
    """合成一段"转过去再转回来"的数据，验证分析与画图链路。"""
    samples = []
    for i in range(400):
        t = i * 0.05
        theta = wrap(0.7 * math.sin(2.0 * math.pi * t / 12.0))
        # 带一点真实感：raw 滞后 1~2 个采样、底盘有 0.2° 的抖动
        raw = wrap(0.7 * math.sin(2.0 * math.pi * (t - 0.08) / 12.0)
                   + math.radians(0.4))
        base = wrap(math.radians(0.2) * math.sin(2.0 * math.pi * t / 1.7))
        samples.append({"t": t, "theta": theta, "raw": raw, "base": base,
                        "residual": wrap(raw - theta - base),
                        "bx": 0.002 * math.sin(t), "by": 0.0,
                        "bvx": 0.0, "bvy": 0.0})
    return samples


def _finish(args, samples, base_path, meta, _unused, demo=False):
    # ⚠ 按**最终**话题过滤：采集早期 base 可能还锁在 /lidar_odometry（雷达位姿），
    # 那几拍的 yaw/xy 语义完全不同，混进来会把 travel/net/锁相全带偏（2026-11-13 实测：
    # 端点比 0.913、行程比 1.084，其实是被早期 lidar 位姿污染的）。
    final_base = meta.get("base_topic")
    final_raw = meta.get("raw_topic")
    kept = [s for s in samples
            if (s.get("bt") in (None, final_base)) and (s.get("rt") in (None, final_raw))]
    if len(kept) != len(samples):
        print("  注意: %d/%d 个样本用的是切换前的话题（%s/%s），已剔除"
              % (len(samples) - len(kept), len(samples), final_raw, final_base), flush=True)
    stats = analyse(kept, meta.get("mount"))
    if stats is not None:
        stats["raw_topic"] = meta.get("raw_topic", "?")
        stats["base_topic"] = meta.get("base_topic", "?")
        stats["joint_topic"] = meta.get("joint_topic", "?")
        stats["base_child"] = meta.get("base_child") or "base_link"
        stats["residual_tol"] = float(getattr(args, "residual_tol", 0.04))
    rc, lines = verdict(stats, args.tol_deg, args.min_travel_deg)

    print("", flush=True)
    print("================ 云台单转采集结论 ================", flush=True)
    if stats is not None and "samples" in stats:
        print("  窗口 %.1f s，有效样本 %d" % (stats["t_end"], stats["n"]), flush=True)
    for line in lines:
        print(line, flush=True)

    # /lidar_odometry 是 loam_interface/zlio_bridge 的输出，**没有**减云台角；
    # 拿它当 base 时 s_base≈1 是必然的，不能据此判"关节角没进 TF"。
    if meta.get("base_topic") == "/lidar_odometry":
        print("  注意: base 用的是 /lidar_odometry（未经 sensor_scan_generation 减云台角），",
              flush=True)
        print("        s_base≈1 属正常，不能据此判断关节角！优先用默认的 /odometry。",
              flush=True)

    txt = base_path.with_suffix(".txt")
    try:
        with open(txt, "w", encoding="utf-8") as stream:
            stream.write("云台单转采集 %s\n" % base_path.name)
            stream.write("raw=%s  base=%s  joint=%s\n"
                         % (meta.get("raw_topic"), meta.get("base_topic"),
                            meta.get("joint_topic")))
            stream.write("消息计数: %s\n\n" % meta.get("counts"))
            for line in lines:
                stream.write(line.strip() + "\n")
        print("  结论已写入 %s" % txt, flush=True)
    except OSError as exc:
        print("[警告] 写不了 %s: %s" % (txt, exc), flush=True)

    if not args.no_plot and stats is not None and "samples" in stats:
        png = base_path.with_suffix(".png")
        try:
            plot(stats, args, png, "[demo]" if demo else "")
            print("  曲线已保存 %s" % png, flush=True)
        except Exception as exc:  # noqa: BLE001 画图失败不影响结论
            print("[警告] 出图失败: %s" % exc, flush=True)

    csv_path = base_path.with_suffix(".csv")
    if csv_path.exists():
        print("  数据: %s" % csv_path, flush=True)
    print("  前提: 采集期间底盘没动（没发导航目标、没人推车）—— 底盘自己在转时这套判据不成立。",
          flush=True)
    print("==================================================", flush=True)
    return rc


if __name__ == "__main__":
    sys.exit(main())
