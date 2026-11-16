#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# =============================================================
# SRM LIO vs 真值 航向(yaw) 波形调试工具 (debug_tools #2)
#
# 用途：回答"LIO 算出来的车头朝向，和 Gazebo 真值差多少、什么时候开始差"。
#   一张图 4 个面板：
#     ① yaw 曲线        ── 真值(GT) 与 导航用的 /odometry，两条线叠在一起
#     ② yaw 误差        ── 差值(°)，已折到 ±180，0 就是完全对齐
#     ③ 位置轨迹 (x-y)  ── 真值 vs 导航；云台转时若出现小圆，这里一眼可见
#     ④ 速度符号        ── /odometry 的 twist.x 与 /cmd_vel_sim 的 linear.x
#                          （判断"前进变后退"是 LIO 认错方向，还是命令本身反了）
#
# -------------------------------------------------------------
# 数据从哪来（只读，不往任何 cmd_vel 写东西）
# -------------------------------------------------------------
#   真值:  <ns>/chassis_odometry_gt   (nav_msgs/Odometry, Gazebo 插件直报)
#   LIO :  <ns>/odometry              (nav_msgs/Odometry, loam_interface +
#                                       sensor_scan_generation 算出来的车体位姿)
#   命令:  <ns>/cmd_vel_sim           (geometry_msgs/Twist, mux 的唯一出口)
#   可选:  <ns>/lidar_odometry        (odometry 之前那一级，加 --with-raw 才画)
#
#   注意：真值话题的 yaw 是"车体在 Gazebo 世界系里的朝向"；/odometry 的 yaw 是
#   "车体在 odom 系里的朝向"。两者在 map→odom 为静态单位阵时可以直接比较；
#   若你改了 MAP2ODOM_YAW，这条曲线里的差值里就会包含那个常量（这是有意义的，
#   因为它正是导航"以为"的朝向误差）。
#
# -------------------------------------------------------------
# 用法
# -------------------------------------------------------------
#   # 1) 不带启动器跑（先看工具本身能不能出图）
#   ./debug_tools/plot_lio_gt_yaw.py --demo
#
#   # 2) 真实录制（默认一直录，Ctrl-C 结束时出图）
#   ./debug_tools/plot_lio_gt_yaw.py --headless --save /tmp/lio_yaw.png
#   ./debug_tools/plot_lio_gt_yaw.py --ros-args -r __ns:=/red_standard_robot1
#
#   # 3) 想固定时长也行
#   ./debug_tools/plot_lio_gt_yaw.py --duration 60 --headless --save yaw.png
#
#   # 4) 只想看统计数字（不画图）
#   ./debug_tools/plot_lio_gt_yaw.py --duration 30 --stats-only
#
#   典型流程（复现"前进变后退"）：
#     终端A: ./scripts/bringup/start_sim_nav.sh -w rmuc_2025 -m rmuc_2025 --yaw-mode spin --lio
#     终端B: ./debug_tools/plot_lio_gt_yaw.py --duration 60 --headless --save /tmp/lio_yaw.png
#     终端C: 发目标/发速度；看图上 ② 什么时候开始离开 0、④ 两条线是否同号
#
# 判读要点：
#   · ② 常年贴着 0（±2°）→ 坐标换算没问题，问题在下游（控制/执行）
#   · ② 在"开始动/云台转起来"之后才拉开 → LIO 在动态下配准不住（点云太稀）
#   · ④ 里 /odometry.twist.x 与 /cmd_vel_sim.linear.x 长期不同号 → 速度符号/坐标约定反了
#   · ③ 里导航轨迹画小圆而真值不动 → 云台离轴杠杆臂（本车约 13 cm）残差
#
# =============================================================
# --zlio：里程计前端换成 ZLIO 之后，多出 3 个面板 + 一组专属统计
# =============================================================
# 原来的 5 个面板对"谁产生里程计"是不敏感的，换成 ZLIO 之后真正要盯的是三件事：
#
#   ⑥ 累计位移（m）与**位移夸大倍数**  ← 方案 §8.4 的头号 A/B 指标
#        三条线：真值 / 导航用 / ZLIO 原始。倍数 = 导航累计路程 / 真值累计路程。
#        Point-LIO 在仿真里的历史基线是 6.5× / 8.0× / 10.7×（见
#        nav2_params_srm_fakeyaw.yaml 的 point_filter_num 那段注释）。
#        若 ZLIO 也出现 3~10 倍，第一嫌疑是 point_time_scale（把秒当纳秒 → 去畸变失效）。
#
#   ⑦ 点云点数/帧 与 频率
#        ZLIO 的 FrameResult.cloud 是**自适应采样后**的点（1000~4000 目标），再按
#        output_resolution=0.10 体素化，而 Point-LIO 的 cloud_registered 是 ~16,890 点/帧。
#        terrain_analysis 的 scanVoxelSize 是 0.05 —— 点数掉太多时地形图会变稀，
#        表现是"避障没坏但明显迟钝"。这条曲线就是那件事的直接证据。
#
#   ⑧ T_off 一致性（原始 → 修正 的那个固定变换）
#        数学上 T_off(k) = T_导航(k) · T_原始(k)⁻¹ 必须**恒定**（bridge 只查一次 TF）。
#        两条线（yaw / |平移|）若在时间上漂，说明有人每帧重新查了 TF（云台角被引进 odom），
#        那会让 sensor_scan_generation 再减一次，姿态翻倍。
#
#   另外：换代（epoch）时刻在 ②⑥⑦ 上画竖线。epoch 变了而位姿跳一大截 = 锚定没生效，
#   terrain_analysis 的地形图会整体错位（方案 §4.3 的必做项）。
#
#   原始位姿默认取 `zlio/lidar_odometry`（ZLIO 自己发的、未做 T_off 修正的那一份）；
#   想改别的话题用 --raw-topic。
#
# 用法（ZLIO）：
#   ./debug_tools/plot_lio_gt_yaw.py --zlio --headless            # 一直录，Ctrl-C 出图
#   ./debug_tools/plot_lio_gt_yaw.py --zlio --duration 60 --headless
#   ./debug_tools/plot_lio_gt_yaw.py --zlio --demo --headless     # 先看工具本身
#   ./debug_tools/plot_lio_gt_yaw.py ... --no-caption             # 图面只要曲线，图注打到终端
# =============================================================

import argparse
import csv
import math
import os
import sys
import time

import numpy as np

import signal

try:  # 只在真正录制时才需要 rclpy；--demo 模式不依赖 ROS
    from rclpy.executors import ExternalShutdownException
except Exception:  # noqa: BLE001 - 没有 source ROS 时用占位异常
    class ExternalShutdownException(Exception):
        pass



# ROS 专有参数：带值的要连带吃掉它的值，不带值的直接丢。
_ROS_FLAGS_WITH_VALUE = ("-r", "--remap", "-p", "--param", "-e", "--enclave",
                         "--log-level", "--log-config-file", "--params-file")
_ROS_FLAGS_BARE = ("--ros-args", "--", "-a")


def _filter_ros_args(argv):
    """把 ROS 专有参数从命令行里剔出来（返回 (自定义参数, ROS 参数)）。"""
    custom, ros = [], []
    i = 0
    while i < len(argv):
        a = argv[i]
        if a in _ROS_FLAGS_WITH_VALUE and i + 1 < len(argv):
            ros.extend([a, argv[i + 1]])
            i += 2
            continue
        if a in _ROS_FLAGS_BARE or a.startswith("__"):
            ros.append(a)
            i += 1
            continue
        custom.append(a)
        i += 1
    return custom, ros


def wrap_deg(angle_deg):
    """把角度折到 (-180, 180]。"""
    return (angle_deg + 180.0) % 360.0 - 180.0


def quat_to_yaw(x, y, z, w):
    """四元数 -> 航向角（度）。"""
    return math.degrees(2.0 * math.atan2(z, w))


def quat_to_roll_pitch(x, y, z, w):
    """四元数 -> (roll, pitch)，单位度。

    为什么需要它：航向由陀螺决定，通常很准；**平移**靠加速度计 + 激光匹配，
    一旦 LIO 的姿态基准有几度误差，重力就会漏进水平加速度（g·sinθ），
    被积分成"车在匀速/加速漂走"——此时 yaw 完好、位置发散。
    """
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    return math.degrees(roll), math.degrees(pitch)


# ---- 位姿复合（只给 --zlio 的 T_off 一致性面板用，避免为 6 行数学引入依赖） ----
def _qmul(a, b):
    """四元数乘法，(w, x, y, z) 顺序。"""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    q = (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )
    n = math.sqrt(sum(v * v for v in q)) or 1.0
    return tuple(v / n for v in q)


def _qconj(q):
    w, x, y, z = q
    return (w, -x, -y, -z)


def _qrot(q, v):
    """用单位四元数旋转向量。"""
    w, x, y, z = q
    vx, vy, vz = v
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx))


def pose_from_msg(odom_msg):
    """nav_msgs/Odometry -> ((w,x,y,z), (x,y,z))。"""
    p = odom_msg.pose.pose
    return (
        (p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z),
        (p.position.x, p.position.y, p.position.z),
    )


def relative_pose(nav_pose, raw_pose):
    """T_off = T_nav · T_raw⁻¹，返回 (yaw 度, 平移模长 m)。

    bridge 只查一次 T_off 并缓存，所以这个量在**时间上必须恒定**。
    """
    qn, tn = nav_pose
    qr, tr = raw_pose
    q_inv = _qconj(qr)
    t_inv = _qrot(q_inv, (-tr[0], -tr[1], -tr[2]))
    q = _qmul(qn, q_inv)
    rotated = _qrot(qn, t_inv)
    t = (tn[0] + rotated[0], tn[1] + rotated[1], tn[2] + rotated[2])
    return quat_to_yaw(q[1], q[2], q[3], q[0]), math.sqrt(sum(v * v for v in t))


class Recorder(object):
    """订阅两个里程计 + 速度命令，按时间戳存成序列。"""

    def __init__(self, args):
        import rclpy
        from rclpy.node import Node  # noqa: F401
        from nav_msgs.msg import Odometry
        from geometry_msgs.msg import Twist

        self.args = args
        self.rows = []
        self.t0 = None

        rclpy.init(args=[])  # args=[] 而不是 None：否则自定义命令行开关会被当成 ROS 参数解析
        # ★ 2026-11-06：必须用**仿真时间**。仿真比实时慢（实测 RTF≈0.77），拿墙钟去对
        #   仿真时间的 stamp，所有"延迟/年龄"都会差 1/RTF 倍（约 30%）。
        #   先建一个探测节点看有没有 /clock，再按 --sim-time 决定正式节点用哪个时钟。
        use_sim = self._detect_sim_time(rclpy, args.sim_time)
        overrides = []
        if use_sim:
            # ⚠ 这里要的是 **rclpy.parameter.Parameter**（吃 Python 原生值），
            # 不是 rcl_interfaces.msg.Parameter（那个只吃 ParameterValue 子消息）——
            # 两者同名，混用会分别抛 AssertionError / TypeError（都实测过）。
            from rclpy.parameter import Parameter
            overrides = [Parameter("use_sim_time", value=True)]
        self.node = rclpy.create_node("plot_lio_gt_yaw", parameter_overrides=overrides)
        if use_sim:
            self.node.get_logger().info(
                "本工具使用**仿真时间**（use_sim_time=True，来源: %s）"
                % ("--sim-time on" if args.sim_time == "on" else "检测到 /clock"))
        elif args.sim_time == "off":
            self.node.get_logger().info("按 --sim-time off 使用墙钟（仿真在跑时数字会差 1/RTF 倍）")
        else:
            self.node.get_logger().warn(
                "未检测到 /clock：本工具退回墙钟。若仿真在跑，请确认 --sim-time 或 ROS_DOMAIN_ID")
        self.ns = (args.namespace or "").strip("/")
        if not self.ns:
            self.ns = self._autodetect_namespace(rclpy, args.gt_topic)

        def full(topic):
            if not self.ns or topic.startswith("/"):
                return topic
            return "/" + self.ns.strip("/") + "/" + topic.strip("/")

        gt_topic = full(args.gt_topic)
        lio_topic = full(args.nav_topic)
        cmd_topic = full(args.cmd_topic)
        raw_topic = full(args.raw_topic)

        self.gt = None
        self.lio = None
        self.cmd = None
        self.raw = None
        self.imu = None
        # --zlio：修正后的**雷达**位姿（bridge 的 lidar_odometry）。T_off 只能拿它跟原始
        # 位姿配对；拿 /odometry（车体、已过 sensor_scan_generation 的云台补偿）去配，
        # 云台每转一圈就会画出一个锯齿 —— 那是"配错了对象"，不是 T_off 在漂。
        self.bridge = None
        self.matches = None
        self.events = []          # [(t 绝对秒, level, name)]，--zlio 且话题存在时才有
        self.yaw_debug = None     # (预测 yaw, odom yaw, 积分量) rad
        self.t0_abs = None
        # --zlio 专属：点云点数、话题频率、换代代次（epoch）
        self.scan = None
        self.epoch = -1
        self.epoch_changes = []          # [(t 秒, 新 epoch)]
        self._recv = {}                  # 话题 -> 最近若干次到达时刻（算频率用）
        self._raw_by_stamp = None        # zlio: stamp -> 原始位姿消息（配对用）
        self._bridge_by_stamp = None     # zlio: stamp -> bridge 修正后位姿（配对用）
        self._last_row_epoch = -1        # 上一次写进 rows 的 epoch（判定"这一行是不是换代点"）

        self.node.create_subscription(
            Odometry, gt_topic, self._on_gt, 20)
        self.node.create_subscription(
            Odometry, lio_topic, self._on_lio, 20)
        self.node.create_subscription(
            Twist, cmd_topic, self._on_cmd, 20)
        if args.with_raw or args.zlio:
            self.node.create_subscription(
                Odometry, raw_topic, self._on_raw, 20)
        from sensor_msgs.msg import Imu
        self.node.create_subscription(
            Imu, full(args.imu_topic), self._on_imu, 50)
        if args.yaw_debug_topic:
            from geometry_msgs.msg import Vector3
            self.node.create_subscription(
                Vector3, full(args.yaw_debug_topic), self._on_yaw_debug, 50)

        if args.zlio:
            from sensor_msgs.msg import PointCloud2
            from std_msgs.msg import UInt32, UInt64
            scan_topic = full(args.scan_topic)
            epoch_topic = full(args.epoch_topic)
            bridge_topic = full(args.bridge_topic)
            self.node.create_subscription(
                PointCloud2, scan_topic, self._on_scan, 10)
            self.node.create_subscription(
                UInt64, epoch_topic, self._on_epoch, 10)
            self.node.create_subscription(
                Odometry, bridge_topic, self._on_bridge, 20)
            if args.matches_topic:
                self.node.create_subscription(
                    UInt32, full(args.matches_topic), self._on_matches, 20)
            if args.events_topic:
                from diagnostic_msgs.msg import DiagnosticArray
                self.node.create_subscription(
                    DiagnosticArray, full(args.events_topic), self._on_events, 50)
            self.node.get_logger().info(
                "ZLIO 模式: RAW=%s  BRIDGE=%s  SCAN=%s  EPOCH=%s  MATCHES=%s  EVENTS=%s"
                % (raw_topic, bridge_topic, scan_topic, epoch_topic,
                   full(args.matches_topic) if args.matches_topic else "(关)",
                   full(args.events_topic) if args.events_topic else "(关)"))

        self.node.get_logger().info(
            "录制中: GT=%s  NAV=%s  CMD=%s  %s" %
            (gt_topic, lio_topic, cmd_topic,
             "时长=一直（Ctrl-C 结束）" if args.duration <= 0.0
             else "时长=%.0f s" % args.duration))

    @staticmethod
    def _detect_sim_time(rclpy, mode):
        """判断是否该用仿真时间：mode = auto / on / off。"""
        if mode == "on":
            return True
        if mode == "off":
            return False
        probe = rclpy.create_node("plot_lio_gt_yaw_clock_probe")
        try:
            deadline = time.time() + 2.0
            while time.time() < deadline:
                rclpy.spin_once(probe, timeout_sec=0.1)
                if "/clock" in dict(probe.get_topic_names_and_types()):
                    return True
            return False
        finally:
            probe.destroy_node()

    def _autodetect_namespace(self, rclpy, topic_leaf):
        """没给 --namespace 时，自动找带命名空间的同名话题（如 /red_standard_robot1/xxx）。

        这样 `./debug_tools/plot_lio_gt_yaw.py` 直接就能用，不必每次敲 __ns:=...

        注意：仿真刚起来时话题是**分批出现**的（Gazebo 先起、再 spawn 机器人、桥才建映射），
        所以这里必须耐心等（默认 10 s），不能等 2 s 就下结论 —— 否则会出现
        "明明在跑却报没数据"（实际是抢跑）。
        """
        deadline = time.time() + max(1.0, self.args.discover_timeout)
        announced = False
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.2)
            names = list(dict(self.node.get_topic_names_and_types()))
            if not announced and any(n.endswith("/" + topic_leaf) for n in names):
                break
            if not announced and time.time() > deadline - 8.0:
                self.node.get_logger().info(
                    "等待话题 /<ns>/%s 出现…（仿真刚起来时话题会分批出现，最多等 %.0f s）"
                    % (topic_leaf, self.args.discover_timeout))
                announced = True
        for _ in range(3):
            rclpy.spin_once(self.node, timeout_sec=0.1)
        candidates = [
            name.rsplit("/", 1)[0].strip("/")
            for name in dict(self.node.get_topic_names_and_types())
            if name.endswith("/" + topic_leaf.strip("/")) and name.count("/") > 1
        ]
        candidates = [c for c in candidates if c]
        if len(set(candidates)) == 1:
            ns = candidates[0]
            self.node.get_logger().info("自动识别命名空间: /%s（可用 --namespace 覆盖）" % ns)
            return ns
        if len(set(candidates)) > 1:
            self.node.get_logger().warn(
                "发现多个命名空间都有 %s: %s —— 请用 --namespace 指定"
                % (topic_leaf, sorted(set(candidates))))
            return ""
        names = sorted(dict(self.node.get_topic_names_and_types()))
        self.node.get_logger().warn(
            "等了 %.0f s 仍没发现 /<ns>/%s。当前共有 %d 个话题，前 12 个是："
            % (self.args.discover_timeout, topic_leaf, len(names)))
        for n in names[:12]:
            self.node.get_logger().warn("   %s" % n)
        self.node.get_logger().warn(
            "若仿真刚启动，请等 Gazebo 里的车出现后再运行；若话题确实带命名空间，"
            "用 --namespace 或 --ros-args -r __ns:=/xxx 指定")
        return ""

    # ---- 回调：只存最新值，采样由定时器统一做（三条曲线天然同时间轴） ----
    def _mark(self, key):
        """记下一条消息的到达时刻（算话题频率用）。"""
        import collections

        bucket = self._recv.get(key)
        if bucket is None:
            bucket = collections.deque(maxlen=40)
            self._recv[key] = bucket
        bucket.append(self.node.get_clock().now().nanoseconds * 1e-9)

    def _hz(self, key):
        """最近若干条消息的实测频率（Hz）；样本不足返回 nan。"""
        bucket = self._recv.get(key)
        if not bucket or len(bucket) < 3:
            return float("nan")
        span = bucket[-1] - bucket[0]
        if span <= 0.0:
            return float("nan")
        return (len(bucket) - 1) / span

    def _on_yaw_debug(self, msg):
        """fake_vel_transform 的外推诊断：x=预测 yaw, y=odom yaw, z=积分量。"""
        self.yaw_debug = (msg.x, msg.y, msg.z)

    def _on_gt(self, msg):
        self.gt = msg

    def _on_lio(self, msg):
        self._mark("nav")
        self.lio = msg

    def _on_cmd(self, msg):
        self.cmd = msg

    def _on_raw(self, msg):
        self._mark("raw")
        self.raw = msg
        # 按 stamp 存一份：bridge 的 lidar_odometry 与 ZLIO 的原始位姿**同戳**
        # （方案 §4.2 要求），按时间戳配对才能把 T_off 的一致性量准；
        # 拿"最新"去配会让两帧之间的运动混进来，看着像 T_off 在漂。
        import collections

        if self._raw_by_stamp is None:
            self._raw_by_stamp = collections.OrderedDict()
        key = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
        self._raw_by_stamp[key] = msg
        while len(self._raw_by_stamp) > 256:
            self._raw_by_stamp.popitem(last=False)

    def _on_imu(self, msg):
        self.imu = msg

    def _on_bridge(self, msg):
        self.bridge = msg
        if self._bridge_by_stamp is None:
            import collections

            self._bridge_by_stamp = collections.OrderedDict()
        key = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
        self._bridge_by_stamp[key] = msg
        while len(self._bridge_by_stamp) > 256:
            self._bridge_by_stamp.popitem(last=False)

    def _on_matches(self, msg):
        self.matches = int(msg.data)

    def _on_events(self, msg):
        """bridge 的诊断事件：只记时刻与等级，用来在图上画竖线。

        ⚠ `DiagnosticStatus.level` 在 .msg 里是 `byte`，ROS 2 的 Python 绑定把它映射成**单字节
        bytes**（`b'\x01'`），不是 int —— 直接 `int(...)` 会抛
        `ValueError: invalid literal for int() with base 10: b'\x01'`（2026-11-05 实跑踩到）。
        这里统一转成 int（0=OK 1=WARN 2=ERROR 3=STALE）。
        """
        now = self.node.get_clock().now().nanoseconds * 1e-9
        for status in msg.status:
            level = status.level
            if isinstance(level, (bytes, bytearray)):
                level = level[0] if len(level) else 0
            self.events.append((now, int(level), str(status.name)))

    def _on_scan(self, msg):
        self._mark("scan")
        self.scan = msg

    def _on_epoch(self, msg):
        """换代通知：只在数值变化时记一笔（bridge 在每次换代发一次）。"""
        import rclpy

        value = int(msg.data)
        if value == self.epoch:
            return
        self.epoch = value
        now = self.node.get_clock().now().nanoseconds * 1e-9
        self.epoch_changes.append((now if self.t0 is None else now - self.t0, value))

    def _snapshot(self):
        if self.gt is None or self.lio is None:
            return
        import rclpy
        now = self.node.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = now
            self.t0_abs = now
        g, o = self.gt.pose.pose.orientation, self.lio.pose.pose.orientation
        gp, op = self.gt.pose.pose.position, self.lio.pose.pose.position
        yaw_gt = quat_to_yaw(g.x, g.y, g.z, g.w)
        yaw_lio = quat_to_yaw(o.x, o.y, o.z, o.w)
        roll_gt, pitch_gt = quat_to_roll_pitch(g.x, g.y, g.z, g.w)
        roll_lio, pitch_lio = quat_to_roll_pitch(o.x, o.y, o.z, o.w)

        # --zlio：原始位姿（未做 T_off 修正）与 T_off 一致性
        raw_x = raw_y = float("nan")
        raw_yaw = float("nan")
        raw_roll = raw_pitch = float("nan")
        bridge_roll = bridge_pitch = float("nan")
        off_yaw = off_t = float("nan")
        raw_msg = self.raw
        bridge_msg = self.bridge
        if self._raw_by_stamp is not None:
            # 两路同戳（bridge 原样复制 ZLIO 的 stamp），按 stamp 配对才量得准
            key = (bridge_msg.header.stamp.sec * 1000000000
                   + bridge_msg.header.stamp.nanosec) if bridge_msg is not None else None
            if key is not None:
                raw_msg = self._raw_by_stamp.get(key, self.raw)
        if raw_msg is not None:
            rp = raw_msg.pose.pose.position
            raw_x, raw_y = rp.x, rp.y
            ro = raw_msg.pose.pose.orientation
            raw_yaw = quat_to_yaw(ro.x, ro.y, ro.z, ro.w)
            # ★ 2026-11-05：raw 的 roll/pitch 是识别"双算安装倾角"的关键量。
            #   ZLIO 的世界系被 alignGravity() 扶正，所以 raw 的 roll 应当 ≈ 雷达安装倾角
            #   （仿真 30°）。bridge 输出的 roll 也应当 ≈ 安装倾角；两者**同时**偏说明是
            #   ZLIO 侧的事，只有 bridge/nav 侧偏才是 bridge 的 T_off 问题。
            raw_roll, raw_pitch = quat_to_roll_pitch(ro.x, ro.y, ro.z, ro.w)
            # T_off = T_bridge(lidar, odom) · T_raw(zlio odom, lidar)⁻¹ —— 这是 bridge
            # 缓存的那个固定变换，必须恒定。
            if bridge_msg is not None:
                bo = bridge_msg.pose.pose.orientation
                bridge_roll, bridge_pitch = quat_to_roll_pitch(bo.x, bo.y, bo.z, bo.w)
                off_yaw, off_t = relative_pose(
                    pose_from_msg(bridge_msg), pose_from_msg(raw_msg))

        scan_points = float("nan")
        if self.scan is not None:
            scan_points = float(self.scan.width) * float(self.scan.height)

        # 换代点标记：bridge 只在换代时发一次 epoch，所以"值变了"就是换代点。
        # 第一帧 epoch 通知（从"未知"变成 N）也算 —— 那就是第一次换代。
        epoch_changed = 0.0
        if self.epoch >= 0 and self.epoch != self._last_row_epoch:
            epoch_changed = 1.0
            self._last_row_epoch = self.epoch

        self.rows.append(
            dict(
                t=now - self.t0,
                yaw_gt=yaw_gt,
                yaw_lio=yaw_lio,
                err=wrap_deg(yaw_lio - yaw_gt),
                gt_x=gp.x, gt_y=gp.y,
                lio_x=op.x, lio_y=op.y,
                lio_vx=self.lio.twist.twist.linear.x,
                cmd_vx=(self.cmd.linear.x if self.cmd is not None else float("nan")),
                roll_gt=roll_gt, pitch_gt=pitch_gt,
                roll_lio=roll_lio, pitch_lio=pitch_lio,
                imu_ax=(self.imu.linear_acceleration.x if self.imu is not None else float("nan")),
                imu_ay=(self.imu.linear_acceleration.y if self.imu is not None else float("nan")),
                imu_az=(self.imu.linear_acceleration.z if self.imu is not None else float("nan")),
                raw_yaw=raw_yaw,
                raw_roll=raw_roll, raw_pitch=raw_pitch,
                bridge_roll=bridge_roll, bridge_pitch=bridge_pitch,
                # --- ZLIO 专属列（非 zlio 模式下是 nan / -1，CSV 列保持稳定）---
                raw_x=raw_x, raw_y=raw_y,
                bridge_x=(bridge_msg.pose.pose.position.x
                          if bridge_msg is not None else float("nan")),
                bridge_y=(bridge_msg.pose.pose.position.y
                          if bridge_msg is not None else float("nan")),
                off_yaw=off_yaw, off_t=off_t,
                scan_points=scan_points,
                scan_hz=self._hz("scan"),
                nav_hz=self._hz("nav"),
                raw_hz=self._hz("raw"),
                epoch=float(self.epoch),
                epoch_changed=epoch_changed,
                matches=float(self.matches) if self.matches is not None else float("nan"),
                pred_yaw=(math.degrees(self.yaw_debug[0]) if self.yaw_debug else float("nan")),
                yaw_odom_fvt=(math.degrees(self.yaw_debug[1]) if self.yaw_debug else float("nan")),
                wz_integral=(math.degrees(self.yaw_debug[2]) if self.yaw_debug else float("nan")),
            )
        )

    def run(self):
        """录制：--duration<=0 时一直录，直到 Ctrl-C。"""
        import rclpy
        period = 1.0 / self.args.rate
        start = time.time()
        last_report = start
        unlimited = self.args.duration <= 0.0
        print(
            "开始录制（%s）。%s"
            % (
                "一直录，按 Ctrl-C 结束并出图" if unlimited
                else "录 %.0f s" % self.args.duration,
                "" if unlimited else "想提前结束也可以随时 Ctrl-C（已录部分照样出图）",
            )
        )
        stop = {"flag": False}

        def _on_sigint(_signum, _frame):
            stop["flag"] = True

        old_handler = signal.signal(signal.SIGINT, _on_sigint)
        try:
            last_sample = 0.0
            while (unlimited or (time.time() - start) < self.args.duration) and not stop["flag"]:
                try:
                    rclpy.spin_once(self.node, timeout_sec=period)
                except KeyboardInterrupt:
                    stop["flag"] = True
                    continue
                except ExternalShutdownException:
                    stop["flag"] = True
                    continue
                # 采样节流：--rate 必须真的生效。原来每次 spin 回来就存一个点，
                # 在以 30~200 Hz 到达的消息面前等于 500 Hz 采样（2026-11-05 实测
                # "5 s 录了 2493 个点"），既浪费 CPU 又让 CSV 膨胀。
                if time.time() - last_sample >= period:
                    last_sample = time.time()
                    self._snapshot()
                now = time.time()
                if now - last_report >= 5.0:
                    last_report = now
                    print(
                        "  已录 %5.1f s, %4d 个采样点%s"
                        % (
                            now - start,
                            len(self.rows),
                            "" if self.rows else "  <-- 还没有数据，检查仿真/命名空间/话题名",
                        )
                    )
        finally:
            signal.signal(signal.SIGINT, old_handler)
            if stop["flag"]:
                print("")
                print("收到 Ctrl-C：停止录制，开始出图/统计")
            self.node.destroy_node()
            try:
                rclpy.shutdown()
            except Exception:  # noqa: BLE001 - 可能已 shutdown
                pass


def demo_rows(duration=20.0, rate=20.0, zlio=False):
    """合成数据。

    非 zlio（原样保留）：前 8 秒完全对齐，之后 LIO 逐渐漂 + 偶发跳变，并画一个 13 cm 小圆。
    zlio：再额外造一份"像 ZLIO"的数据 —— 位移夸大 3×、~1500 点/帧、T_off 恒定 63°/60 mm、
    第 12 秒换代一次（锚定生效，位姿连续）。这样 `--demo --zlio` 能自检 ⑥⑦⑧ 三个面板。
    """
    rows = []
    t = 0.0
    over = 3.0        # zlio demo 的位移夸大倍数
    while t < duration:
        yaw_gt = 15.0 * math.sin(0.25 * t)
        drift = 0.0 if t < 8.0 else 6.0 * (t - 8.0) / duration * 10.0
        jump = 40.0 if abs((t % 6.0) - 3.0) < 0.06 else 0.0
        yaw_lio = yaw_gt + drift + jump
        r = 0.133  # 云台离轴杠杆臂（m）
        row = dict(
            t=t,
            yaw_gt=yaw_gt,
            yaw_lio=wrap_deg(yaw_lio),
            err=wrap_deg(yaw_lio - yaw_gt),
            gt_x=0.0, gt_y=0.0,
            lio_x=r * math.sin(2.0 * t), lio_y=r * (1 - math.cos(2.0 * t)),
            # demo 里造一个 3° 的 roll 偏差：等效重力泄漏 ≈ 0.51 m/s²
            roll_gt=0.0, pitch_gt=0.0,
            roll_lio=3.0, pitch_lio=0.5,
            imu_ax=float("nan"), imu_ay=float("nan"), imu_az=float("nan"),
            lio_vx=0.2 * math.cos(0.5 * t),
            cmd_vx=0.2,
            raw_yaw=float("nan"),
            raw_x=float("nan"), raw_y=float("nan"),
            bridge_x=float("nan"), bridge_y=float("nan"),
            off_yaw=float("nan"), off_t=float("nan"),
            raw_roll=float("nan"), raw_pitch=float("nan"),
            bridge_roll=float("nan"), bridge_pitch=float("nan"),
            scan_points=float("nan"),
            scan_hz=float("nan"), nav_hz=float("nan"), raw_hz=float("nan"),
            epoch=-1.0,
            epoch_changed=0.0,
            matches=float("nan"),
            pred_yaw=float("nan"), yaw_odom_fvt=float("nan"), wz_integral=float("nan"),
        )
        if zlio:
            # 真值沿 x 走、ZLIO 走 3 倍；再叠上那个 13 cm 小圆（云台杠杆臂）
            row.update(
                gt_x=0.4 * t, gt_y=0.0,
                lio_x=over * 0.4 * t + r * math.sin(2.0 * t),
                lio_y=r * (1 - math.cos(2.0 * t)),
                raw_yaw=wrap_deg(yaw_lio - 63.0),
                raw_x=over * 0.4 * t, raw_y=0.0,
                # demo 里模拟"云台 30° roll 安装 + 已抵消"：raw 与 bridge 都是 30°
                raw_roll=30.0, raw_pitch=0.0,
                bridge_roll=30.0, bridge_pitch=0.0,
                bridge_x=over * 0.4 * t + 0.06, bridge_y=0.0,
                off_yaw=63.0 + 0.05 * math.sin(0.7 * t),
                off_t=0.06 + 0.001 * math.sin(1.3 * t),
                scan_points=1200.0 + 300.0 * math.sin(0.9 * t),
                scan_hz=19.5, nav_hz=19.0, raw_hz=19.0,
                epoch=0.0 if t < 12.0 else 1.0,
                # 第 12 s 换代一次（锚定生效 → 位姿连续，所以只有竖线没有跳变）
                epoch_changed=1.0 if 12.0 <= t < 12.0 + 1.0 / rate else 0.0,
                # 8~10 s 造一段几何退化：匹配点掉到 5 个（正对单面墙的样子）
                matches=(5.0 if 8.0 <= t < 10.0 else 300.0 + 80.0 * math.sin(1.7 * t)),
            )
        rows.append(row)
        t += 1.0 / rate
    return rows


def print_stats(rows, events=None, event_origin=None):
    err = np.array([r["err"] for r in rows], dtype=float)
    jump = np.abs(np.diff(np.array([r["yaw_lio"] for r in rows], dtype=float)))
    jump = np.where(jump > 180.0, 360.0 - jump, jump)
    print("")
    print("================ 统计 ================")
    print("采样点数      : %d" % len(rows))
    print("平均误差      : %+7.2f°   (导航 - 真值)" % err.mean())
    print("误差标准差    : %7.2f°" % err.std())
    print("误差范围      : %+7.2f° ~ %+7.2f°" % (err.min(), err.max()))
    print("单帧跳变>5°   : %d 次, 最大 %.1f°" % (int((jump > 5.0).sum()), jump.max()))
    cmd = np.array([r["cmd_vx"] for r in rows], dtype=float)
    vx = np.array([r["lio_vx"] for r in rows], dtype=float)
    mask = np.isfinite(cmd) & (np.abs(cmd) > 0.05)
    if mask.sum() > 5:
        same_sign = float(np.mean(np.sign(vx[mask]) == np.sign(cmd[mask]))) * 100.0
        print("命令 vs LIO 速度同号比例: %.0f%%  (低 <70%% 就是速度方向反了)" % same_sign)
    else:
        print("命令 vs LIO 速度同号比例: 样本太少（录制期间没有明显速度命令）")
    rg = np.array([r.get("roll_gt", np.nan) for r in rows], dtype=float)
    rl = np.array([r.get("roll_lio", np.nan) for r in rows], dtype=float)
    pg = np.array([r.get("pitch_gt", np.nan) for r in rows], dtype=float)
    pl = np.array([r.get("pitch_lio", np.nan) for r in rows], dtype=float)
    if np.isfinite(rl).any():
        droll = float(np.nanmean(rl - rg))
        dpitch = float(np.nanmean(pl - pg))
        tilt = math.sqrt(droll * droll + dpitch * dpitch)
        print("姿态偏差      : roll %+.2f°, pitch %+.2f°, 合成 %.2f°"
              % (droll, dpitch, tilt))
        print("  -> 等效重力泄漏加速度 ≈ %.2f m/s²  "
              "(= 9.81*sin(%.2f°)，越大越会漂)" % (9.81 * math.sin(math.radians(tilt)), tilt))

    # ---- ZLIO 专属统计（没有 zlio 列时自动跳过）----
    off_yaw = np.array([r.get("off_yaw", np.nan) for r in rows], dtype=float)
    off_t = np.array([r.get("off_t", np.nan) for r in rows], dtype=float)
    scan_points = np.array([r.get("scan_points", np.nan) for r in rows], dtype=float)
    scan_hz = np.array([r.get("scan_hz", np.nan) for r in rows], dtype=float)
    nav_hz = np.array([r.get("nav_hz", np.nan) for r in rows], dtype=float)
    raw_hz = np.array([r.get("raw_hz", np.nan) for r in rows], dtype=float)
    if np.isfinite(off_yaw).any() or np.isfinite(scan_points).any():
        print("---- ZLIO ----")
    pred_yaw = np.array([r.get("pred_yaw", np.nan) for r in rows], dtype=float)
    yaw_gt = np.array([r.get("yaw_gt", np.nan) for r in rows], dtype=float)
    if np.isfinite(pred_yaw).any():
        mask = np.isfinite(pred_yaw) & np.isfinite(yaw_gt)
        if mask.sum() > 10:
            err_pred = np.abs(
                np.array([wrap_deg(a - b) for a, b in zip(pred_yaw[mask], yaw_gt[mask])]))
            err_odom = np.abs(
                np.array([wrap_deg(a - b) for a, b in zip(
                    np.array([r["yaw_lio"] for r in rows], dtype=float)[mask],
                    yaw_gt[mask])]))
            integral = np.array([r.get("wz_integral", np.nan) for r in rows], dtype=float)
            print("外推 yaw      : |pred-GT| 均值 %.2f° 最大 %.2f°；"
                  "|odom-GT| 均值 %.2f° 最大 %.2f°"
                  % (err_pred.mean(), err_pred.max(), err_odom.mean(), err_odom.max()))
            print("  外推量       : 均值 %.2f°，最大 %.2f°（= ∫wz dt，仿真时间积分）"
                  % (float(np.nanmean(integral)), float(np.nanmax(np.abs(integral)))))
            if err_pred.mean() < err_odom.mean():
                print("  → 外推**有改善**：平均航向误差 %.2f° → %.2f°"
                      % (err_odom.mean(), err_pred.mean()))
            else:
                print("  → 外推**没有改善**（差值 %.2f°）：检查 actuation_delay 标定 / "
                      "chassis_cmd_vel_topic 是否真的是 mux 输出"
                      % (err_pred.mean() - err_odom.mean()))

    raw_roll = np.array([r.get("raw_roll", np.nan) for r in rows], dtype=float)
    raw_pitch = np.array([r.get("raw_pitch", np.nan) for r in rows], dtype=float)
    bridge_roll = np.array([r.get("bridge_roll", np.nan) for r in rows], dtype=float)
    bridge_pitch = np.array([r.get("bridge_pitch", np.nan) for r in rows], dtype=float)
    if np.isfinite(raw_roll).any() or np.isfinite(bridge_roll).any():
        print("姿态(raw/bridge): roll %+.2f°/%+.2f°, pitch %+.2f°/%+.2f°"
              "（都应≈雷达安装倾角；nav 的 roll/pitch 被 planarize 清零，看不出来）"
              % (float(np.nanmean(raw_roll)), float(np.nanmean(bridge_roll)),
                 float(np.nanmean(raw_pitch)), float(np.nanmean(bridge_pitch))))
    if np.isfinite(off_yaw).any():
        print("T_off 一致性  : yaw %+7.2f° ± %.3f°, |t| %.1f ± %.1f mm"
              % (float(np.nanmean(off_yaw)), float(np.nanstd(off_yaw)),
                 float(np.nanmean(off_t)) * 1000.0, float(np.nanstd(off_t)) * 1000.0))
        print("  -> 标准差应 ≈0（bridge 只查一次 TF）；yaw 均值应等于 URDF 里"
              " base_link←front_mid360 的安装角")
    if np.isfinite(scan_points).any():
        print("点云点数/帧   : 均值 %.0f, 最小 %.0f, 最大 %.0f"
              % (float(np.nanmean(scan_points)), float(np.nanmin(scan_points)),
                 float(np.nanmax(scan_points))))
        print("  -> 对照 Point-LIO 的 ~16890 点/帧；明显更少时 terrain_map 会变稀")
    matches = np.array([r.get("matches", np.nan) for r in rows], dtype=float)
    if np.isfinite(matches).any():
        print("匹配面观测点  : 均值 %.0f, 最小 %.0f, <30 的帧占 %.0f%%"
              % (float(np.nanmean(matches)), float(np.nanmin(matches)),
                 100.0 * float(np.mean(matches < 30))))
        print("  -> 掉到几十以下 = 几何退化（正对单面墙/空旷）；ZLIO 只会在 matches=0 时才"
              "拒绝更新，所以它看起来仍然\"正常\"")
    for name, series in (("扫描 Hz", scan_hz), ("桥接 odom Hz", nav_hz),
                         ("原始 odom Hz", raw_hz)):
        if np.isfinite(series).any():
            print("%-13s : 均值 %.1f Hz" % (name, float(np.nanmean(series))))
    # 累计位移与倍数（方案 §8.4 的头号指标）
    gt_path, nav_path, raw_path, bridge_path = cumulative_paths(rows)
    if gt_path > 0.05:
        print("累计位移      : GT %.2f m, 导航 %.2f m, ZLIO 原始 %.2f m, bridge %.2f m"
              % (gt_path, nav_path, raw_path, bridge_path))
        print("  分层放大倍数: bridge/GT %.2f×, 导航/bridge %.2f× "
              "(前者=ZLIO 自己的问题，后者=云台补偿/帧链路的问题)"
              % (bridge_path / gt_path if gt_path > 0 else float("nan"),
                 nav_path / bridge_path if bridge_path > 0 else float("nan")))
        print("★ 位移夸大倍数: %.2f×  (= 导航 / GT；Point-LIO 仿真基线 6.5×/8.0×/10.7×)"
              % (nav_path / gt_path))
    else:
        print("累计位移      : GT 几乎没动（%.3f m），位移倍数无意义" % gt_path)
    gt_end = (rows[-1]["gt_x"], rows[-1]["gt_y"])
    nav_end = (rows[-1]["lio_x"], rows[-1]["lio_y"])
    print("终点位置偏差  : %.3f m" % math.hypot(nav_end[0] - gt_end[0],
                                              nav_end[1] - gt_end[1]))
    events = events or []
    if events:
        errors = [e for e in events if e[1] >= 2]
        warns = [e for e in events if e[1] == 1]
        print("诊断事件      : ERROR %d 条, WARN %d 条" % (len(errors), len(warns)))
        if errors:
            first = errors[0]
            offset = (first[0] - event_origin) if event_origin is not None else first[0]
            print("  ★ 第一条 ERROR 在 %.1f s: %s —— 这就是「开始坏」的时刻"
                  % (offset, first[2]))
        for level, label in ((2, "ERROR"), (1, "WARN")):
            group = {}
            for event in events:
                if event[1] == level:
                    group[event[2]] = group.get(event[2], 0) + 1
            for name, count in sorted(group.items(), key=lambda kv: -kv[1])[:4]:
                print("    %-5s %-34s %d 次" % (label, name, count))

    changes = epoch_list(rows)
    if changes:
        print("换代          : %d 次，代次 %s（图上竖线就是它们）"
              % (len(changes),
                 ",".join(str(epoch) for epoch in sorted({ep for _t, ep in changes}))))
    print("CSV 已写出    : %s" % os.path.abspath(CSV_PATH))
    print("=====================================")


def cumulative_paths(rows):
    """按采样点累加三条轨迹的路程（m）。

    采样是固定频率的，所以只要三条曲线用**同一个采样间隔**，比值就是可信的
    （20 Hz 对 0.2~1.0 m/s 的运动足够密，折线近似误差 <1%）。
    """
    def path(key_x, key_y):
        total = 0.0
        last = None
        for row in rows:
            x, y = row.get(key_x, float("nan")), row.get(key_y, float("nan"))
            if not (math.isfinite(x) and math.isfinite(y)):
                continue
            if last is not None:
                total += math.hypot(x - last[0], y - last[1])
            last = (x, y)
        return total

    gt = path("gt_x", "gt_y")
    nav = path("lio_x", "lio_y")
    raw = path("raw_x", "raw_y")
    bridge = path("bridge_x", "bridge_y")
    return gt, nav, raw, bridge


def cum_series(rows, key_x, key_y):
    """逐采样点的累计路程（m），给 ⑥ 面板画曲线用。"""
    out = []
    total = 0.0
    last = None
    for row in rows:
        x, y = row.get(key_x, float("nan")), row.get(key_y, float("nan"))
        if not (math.isfinite(x) and math.isfinite(y)):
            out.append(float("nan") if last is None else total)
            continue
        if last is not None:
            total += math.hypot(x - last[0], y - last[1])
        last = (x, y)
        out.append(total)
    return np.array(out, dtype=float)


def epoch_list(rows):
    """换代时刻列表 [(t 秒, 新 epoch)]。

    用 `epoch_changed` 标记列而不是"相邻两行比较"：bridge 只在**换代时**才发一次
    `zlio_bridge/epoch`，所以第一帧通知本身就是一次换代（无法从行里比较出来）。
    """
    changes = []
    for row in rows:
        if float(row.get("epoch_changed", 0.0) or 0.0) >= 0.5:
            changes.append((row["t"], int(row.get("epoch", -1))))
    return changes


CSV_PATH = "/tmp/lio_gt_yaw.csv"  # 运行时按 --save/--csv 覆盖
# 工程根 = debug_tools/ 的上一级；默认把测试产物写进 <工程根>/user_test_log/
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def save_csv(rows, path=None):
    global CSV_PATH
    if path:
        CSV_PATH = path
    with open(CSV_PATH, "w", newline="") as fp:
        w = csv.DictWriter(fp, fieldnames=list(rows[0].keys()))
        w.writeheader()
        for r in rows:
            w.writerow(r)


def _pyplot(headless):
    """延迟导入 pyplot：先决定后端，避免在有 DISPLAY 但连不上 X 时卡住。"""
    import matplotlib
    if headless or not os.environ.get("DISPLAY"):
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    _CJK_CANDIDATES = (
        "Noto Sans CJK SC", "Noto Sans CJK JP", "WenQuanYi Zen Hei",
        "Droid Sans Fallback", "AR PL UMing CN", "Source Han Sans SC",
    )
    _installed = {f.name for f in font_manager.fontManager.ttflist}
    for _name in _CJK_CANDIDATES:
        if _name in _installed:
            plt.rcParams["font.sans-serif"] = [_name, "DejaVu Sans"]
            break
    plt.rcParams["axes.unicode_minus"] = False
    return plt


def plot(rows, save_path, title_suffix="", headless=False, zlio=False, caption=True,
         events=None, event_origin=None):
    plt = _pyplot(headless)
    t = np.array([r["t"] for r in rows])
    yg = np.array([r["yaw_gt"] for r in rows])
    yl = np.array([r["yaw_lio"] for r in rows])
    er = np.array([r["err"] for r in rows])
    gx = np.array([r["gt_x"] for r in rows])
    gy = np.array([r["gt_y"] for r in rows])
    lx = np.array([r["lio_x"] for r in rows])
    ly = np.array([r["lio_y"] for r in rows])
    vx = np.array([r["lio_vx"] for r in rows])
    cvx = np.array([r["cmd_vx"] for r in rows])
    ry = np.array([r["raw_yaw"] for r in rows])

    rg = np.array([r.get("roll_gt", np.nan) for r in rows], dtype=float)
    pg = np.array([r.get("pitch_gt", np.nan) for r in rows], dtype=float)
    rl = np.array([r.get("roll_lio", np.nan) for r in rows], dtype=float)
    pl = np.array([r.get("pitch_lio", np.nan) for r in rows], dtype=float)

    # --zlio 时在 5 个基础面板下面再接 3 个 ZLIO 专属面板（⑥⑦⑧）
    n_panels = 8 if zlio else 5
    fig_h = 13.5 + (3.0 * (n_panels - 5))
    fig, axes = plt.subplots(n_panels, 1, figsize=(13, fig_h))
    events = events or []
    epoch_t = [ct for (ct, _value) in sorted(epoch_list(rows))]
    # 诊断事件统一换算到与波形同一时间轴：事件记的是**绝对**时钟，波形记的是**相对**秒，
    # 所以必须减掉录制起点 t0_abs（不能拿 rows[0]["t"] 当原点，它本身就是相对时间 —— 踩过）。
    t_start = event_origin if event_origin is not None else 0.0
    error_t = sorted({round(ts - t_start, 2) for ts, level, _n in events if level >= 2})
    warn_t = sorted({round(ts - t_start, 2) for ts, level, _n in events if level == 1})

    # 所有文字一律放到轴外：标题只留"编号 + 名字"，长解释统一收集到图底图注里。
    # 之前长句子直接当 axes 标题，字号一小就以为什么都没挡，实际看图时最上面那
    # 一段正是曲线最容易顶到的位置（尤其 ① 的 raw 线和 ② 的 ±2° 带）。
    notes = []

    def panel_title(axis, short, note=None):
        axis.set_title(short, loc="left", fontsize=11, pad=8)
        if note:
            notes.append(note)

    def header_legend(axis, handles=None, labels=None, ncol=None):
        """图例放到轴外（标题同一行的右侧），不再压在数据上。

        条目多时自动排成两列：一行图例太高会顶到上一个面板的 x 轴标题上。
        """
        if handles is None:
            handles, labels = axis.get_legend_handles_labels()
        if ncol is None:
            ncol = 2 if len(handles) >= 3 else 1
        axis.legend(handles, labels, loc="lower right", bbox_to_anchor=(1.0, 1.0),
                    frameon=False, fontsize=8.5, ncol=ncol, borderaxespad=0.12,
                    handlelength=1.8, columnspacing=1.4)

    def mark_epochs(axis):
        """换代 + 诊断事件画竖线。

        * 紫色点线 = 换代（epoch 变化）：epoch 变了而位姿跳一大截 = 锚定没生效；
        * 红色实线 = bridge 报的 ERROR（单帧跳变等）→ "开始坏"的确切时刻；
        * 橙色细线 = WARN（匹配点过少 / 更新被拒）。
        """
        for index, ts in enumerate(epoch_t):
            axis.axvline(ts, color="#9467bd", lw=1.0, ls=":", alpha=0.9,
                         label="换代 (epoch 变化)" if index == 0 else None)
        for index, ts in enumerate(error_t):
            axis.axvline(ts, color="#e31a1c", lw=1.2, alpha=0.85,
                         label="bridge ERROR" if index == 0 else None)
        for index, ts in enumerate(warn_t[:40]):   # WARN 可能很密，最多画 40 条
            axis.axvline(ts, color="#ff9f1c", lw=0.6, alpha=0.35,
                         label="bridge WARN" if index == 0 else None)

    # 上面板做 unwrap：否则 yaw 越过 ±180 会画成"跳一大截"，看不出真实趋势。
    yg_u = np.degrees(np.unwrap(np.radians(yg)))
    yl_u = np.degrees(np.unwrap(np.radians(yl)))
    ax = axes[0]
    ax.plot(t, yg_u, color="#1f77b4", lw=1.8, label="真值 GT (chassis_odometry_gt)")
    ax.plot(t, yl_u, color="#d62728", lw=1.4, ls="--", label="导航 /odometry (LIO)")
    if np.isfinite(ry).any():
        ry_mask = np.isfinite(ry)
        ry_u = np.degrees(np.unwrap(np.radians(ry[ry_mask])))
        ax.plot(t[ry_mask], ry_u, color="#2ca02c", lw=1.0, alpha=0.8,
                label="原始 lidar_odometry")
    # fake_vel_transform 实际用于转换的 yaw（= odom yaw + ∫wz dt，外推到指令生效时刻）。
    # 它才是"命令真正被旋转时用的那个角"，和真值贴得比 odom yaw 更近就说明外推起效。
    pred_yaw = np.array([r.get("pred_yaw", np.nan) for r in rows], dtype=float)
    if np.isfinite(pred_yaw).any():
        mask = np.isfinite(pred_yaw)
        ax.plot(t[mask], np.degrees(np.unwrap(np.radians(pred_yaw[mask]))),
                color="#e377c2", lw=1.4, label="fake_vel_transform 预测 yaw（外推后）")
    ax.set_ylabel("yaw (°)")
    panel_title(
        ax, "① 航向曲线" + title_suffix,
        "① 航向：LIO 算的 yaw 与真值对比（做了 unwrap，跨 ±180° 不断线）；"
        "两条线越早分开，方向估计出问题越早。")
    ax.grid(alpha=0.3)
    header_legend(ax)

    ax = axes[1]
    ax.plot(t, er, color="#ff7f0e", lw=1.6)
    ax.axhline(0.0, color="k", lw=0.8)
    ax.axhspan(-2, 2, color="#2ca02c", alpha=0.15, label="±2° 正常带")
    mark_epochs(ax)
    ax.set_ylabel("误差 (°)")
    panel_title(
        ax, "② 航向误差" + ("（含 ZLIO 换代）" if zlio else ""),
        "② 误差 = 导航 − 真值，折到 ±180°；绿色带 = ±2° 正常范围，"
        "紫色竖线 = ZLIO 换代时刻。")
    ax.grid(alpha=0.3)
    header_legend(ax)

    ax = axes[2]
    ax.plot(gx, gy, color="#1f77b4", lw=1.8, label="真值轨迹")
    ax.plot(lx, ly, color="#d62728", lw=1.2, ls="--", label="导航轨迹")
    bx = np.array([r.get("bridge_x", np.nan) for r in rows], dtype=float)
    by = np.array([r.get("bridge_y", np.nan) for r in rows], dtype=float)
    if np.isfinite(bx).any():
        ax.plot(bx, by, color="#2ca02c", lw=1.0, alpha=0.9,
                label="bridge 输出（雷达位姿，修正后）")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    panel_title(
        ax, "③ 位置轨迹",
        "③ 轨迹：云台转动时若导航轨迹画出小圆，就是离轴杠杆臂残差（本车 ≈13 cm）。")
    ax.grid(alpha=0.3)
    ax.axis("equal")
    header_legend(ax)

    ax = axes[3]
    ax.plot(t, cvx, color="#1f77b4", lw=1.6, label="cmd_vel_sim.linear.x (命令)")
    ax.plot(t, vx, color="#d62728", lw=1.4, ls="--", label="/odometry.twist.x (LIO 认为的速度)")
    ax.axhline(0.0, color="k", lw=0.8)
    ax.set_xlabel("时间 (s)")
    ax.set_ylabel("x 方向速度 (m/s)")
    panel_title(
        ax, "④ 速度符号",
        "④ 速度符号：cmd_vel_sim.linear.x 与 /odometry.twist.x 长期不同号 "
        "⇒ 速度方向/坐标约定反了。")
    ax.grid(alpha=0.3)
    header_legend(ax)

    ax = axes[4]
    ax.plot(t, rg, color="#1f77b4", lw=1.6, label="真值 roll")
    ax.plot(t, rl, color="#1f77b4", lw=1.2, ls="--", label="导航 roll（planarize 后≈0）")
    ax.plot(t, pg, color="#ff7f0e", lw=1.6, label="真值 pitch")
    ax.plot(t, pl, color="#ff7f0e", lw=1.2, ls="--", label="导航 pitch（planarize 后≈0）")
    raw_roll = np.array([r.get("raw_roll", np.nan) for r in rows], dtype=float)
    raw_pitch = np.array([r.get("raw_pitch", np.nan) for r in rows], dtype=float)
    bridge_roll = np.array([r.get("bridge_roll", np.nan) for r in rows], dtype=float)
    bridge_pitch = np.array([r.get("bridge_pitch", np.nan) for r in rows], dtype=float)
    if np.isfinite(raw_roll).any():
        # ★ 这两组是"倾斜"的直接证据：正确时都 ≈ 雷达安装倾角（仿真 30° roll）。
        #   旧 bug（T_off 少乘 R_0⁻¹）会把 bridge 的倾角再叠一层，而 raw 不变。
        ax.plot(t, raw_roll, color="#9467bd", lw=1.2, label="ZLIO 原始 roll（应≈安装倾角）")
        ax.plot(t, raw_pitch, color="#9467bd", lw=1.0, ls=":", label="ZLIO 原始 pitch")
    if np.isfinite(bridge_roll).any():
        ax.plot(t, bridge_roll, color="#2ca02c", lw=1.2, ls="--",
                label="bridge roll（应≈安装倾角）")
        ax.plot(t, bridge_pitch, color="#2ca02c", lw=1.0, ls=":",
                label="bridge pitch")
    ax.set_xlabel("时间 (s)")
    ax.set_ylabel("角度 (°)")
    panel_title(
        ax, "⑤ 姿态 roll/pitch",
        "⑤ 姿态：LIO 与真值若长期差几度 ⇒ 重力会漏进水平加速度"
        "（1° ≈ 0.17 m/s²），位置被积分成漂移。")
    ax.grid(alpha=0.3)
    header_legend(ax, ncol=2)

    if zlio:
        gt_path, nav_path, raw_path, bridge_path = cumulative_paths(rows)
        ratio = (nav_path / gt_path) if gt_path > 0.05 else float("nan")

        # ⑥ 累计位移 + 位移夸大倍数（方案 §8.4 头号指标）
        ax = axes[5]
        ax.plot(t, cum_series(rows, "gt_x", "gt_y"), color="#1f77b4", lw=1.8,
                label="真值累计位移")
        ax.plot(t, cum_series(rows, "lio_x", "lio_y"), color="#d62728", lw=1.6, ls="--",
                label="导航累计位移 (/odometry)")
        raw_cum = cum_series(rows, "raw_x", "raw_y")
        if np.isfinite(raw_cum).any():
            ax.plot(t, raw_cum, color="#2ca02c", lw=1.0, alpha=0.85,
                    label="ZLIO 原始累计位移 (zlio/lidar_odometry)")
        bridge_cum = cum_series(rows, "bridge_x", "bridge_y")
        if np.isfinite(bridge_cum).any():
            ax.plot(t, bridge_cum, color="#17becf", lw=1.2, ls=":",
                    label="bridge 累计位移（雷达位姿）")
            bridge_path = float(bridge_cum[-1])
            ax.axhline(bridge_path, color="#17becf", lw=0.7, alpha=0.4)
        mark_epochs(ax)
        ax.set_xlabel("时间 (s)")
        ax.set_ylabel("累计路程 (m)")
        panel_title(
            ax, "⑥ 累计位移（夸大 %.2f×）" % ratio,
            "⑥ 累计位移：★ = 导航 / 真值（大于 1 = 里程被夸大）；"
            "Point-LIO 仿真基线 6.5× / 8.0× / 10.7×。")
        ax.grid(alpha=0.3)
        header_legend(ax)

        # ⑦ 点云点数/帧 + 频率（方案 §5.1：ZLIO 输出的是采样后的点）
        ax = axes[6]
        points = np.array([r.get("scan_points", np.nan) for r in rows], dtype=float)
        ax.plot(t, points, color="#d62728", lw=1.2, label="registered_scan 点数/帧")
        matches = np.array([r.get("matches", np.nan) for r in rows], dtype=float)
        if np.isfinite(matches).any():
            ax.plot(t, matches, color="#9467bd", lw=1.0, alpha=0.85,
                    label="匹配面观测点 matches")
            ax.axhline(30.0, color="#9467bd", lw=0.8, ls=":", alpha=0.7)
        ax.set_xlabel("时间 (s)")
        ax.set_ylabel("点数/帧")
        ax.grid(alpha=0.3)
        mark_epochs(ax)
        ax_rt = ax.twinx()
        ax_rt.plot(t, np.array([r.get("scan_hz", np.nan) for r in rows], dtype=float),
                   color="#1f77b4", lw=1.4, label="扫描频率")
        ax_rt.plot(t, np.array([r.get("nav_hz", np.nan) for r in rows], dtype=float),
                   color="#ff7f0e", lw=1.2, ls="--", label="桥接 odom 频率")
        ax_rt.set_ylabel("频率 (Hz)")
        lines = ax.get_lines() + ax_rt.get_lines()
        header_legend(ax, lines, [line.get_label() for line in lines], ncol=2)
        if np.isfinite(points).any():
            panel_title(
                ax, "⑦ 点数/帧与频率（点数均值 %.0f）" % float(np.nanmean(points)),
                "⑦ 点数/频率：ZLIO 输出的是采样后的点，点数明显低于 Point-LIO "
                "（对照 ≈16890 点/帧）时先查降采样。")
        else:
            panel_title(
                ax, "⑦ 点数/帧与频率",
                "⑦ 没收到 registered_scan，检查 --scan-topic。")

        # ⑧ T_off 一致性：原始 → 修正 的固定变换必须恒定
        ax = axes[7]
        off_yaw = np.array([r.get("off_yaw", np.nan) for r in rows], dtype=float)
        off_t = np.array([r.get("off_t", np.nan) for r in rows], dtype=float)
        ax.plot(t, off_yaw, color="#9467bd", lw=1.4, label="T_off 的 yaw (§)")
        ax.set_xlabel("时间 (s)")
        ax.set_ylabel("yaw (°)")
        ax.grid(alpha=0.3)
        mark_epochs(ax)
        ax_rt = ax.twinx()
        ax_rt.plot(t, off_t * 1000.0, color="#8c564b", lw=1.2, ls="--",
                   label="T_off 的 |平移| (mm)")
        ax_rt.set_ylabel("|平移| (mm)")
        lines = ax.get_lines() + ax_rt.get_lines()
        header_legend(ax, lines, [line.get_label() for line in lines], ncol=2)
        if np.isfinite(off_yaw).any():
            panel_title(
                ax, "⑧ T_off 一致性（yaw %+.2f° ± %.2f°, |t| %.0f mm）"
                % (float(np.nanmean(off_yaw)), float(np.nanstd(off_yaw)),
                   float(np.nanmean(off_t)) * 1000.0),
                "⑧ T_off：原始 → 修正的固定变换必须恒定；一旦漂移，说明有节点"
                "每帧重查 TF（白吃 CPU，还可能引入相位滞后）。")
        else:
            panel_title(
                ax, "⑧ T_off 一致性",
                "⑧ 没有原始位姿，检查 --raw-topic。")

    # 图注放图底（轴外）：每行一条，和面板编号对应，永远不压曲线。
    # 底部要同时容纳"图注 + 空隙 + 最后一个面板的刻度标签/x 轴标题"，
    # 只按图注高度留空间的话，最后一行会盖住 x 轴标题（踩过）。
    if notes and caption:
        line_pt = 10.6                      # 7.5pt 字号 + linespacing 1.35
        fig_pt = fig_h * 72.0
        caption_pt = len(notes) * line_pt
        gap_pt = 14.0
        bottom_frac = (caption_pt + gap_pt) / fig_pt + 0.045
        fig.subplots_adjust(left=0.078, right=0.988, top=0.966,
                            bottom=bottom_frac, hspace=0.42)
        fig.text(0.012, (caption_pt / 2.0 + 6.0) / fig_pt, "\n".join(notes),
                 ha="left", va="center", fontsize=7.5, color="#444444",
                 linespacing=1.35)
    else:
        fig.tight_layout()
        if notes:                     # --no-caption：信息别丢，打到终端
            print("图注（--no-caption 时不画进图里）：")
            for note in notes:
                print("  " + note)
    fig.savefig(save_path, dpi=130, bbox_inches="tight")
    print("图已保存: %s" % save_path)
    if os.environ.get("DISPLAY") and not headless:
        plt.show()


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="LIO vs 真值 的 yaw 波形（debug_tools #2）",
        add_help=False,
    )
    parser.add_argument("-h", "--help", action="store_true")
    parser.add_argument(
        "--duration", type=float, default=0.0,
        help="录制时长（秒）。不写或写 0 = 一直录，按 Ctrl-C 结束并出图（默认）")
    parser.add_argument("--rate", type=float, default=20.0, help="采样频率（Hz）")
    parser.add_argument("--gt-topic", default="chassis_odometry_gt", help="真值话题")
    parser.add_argument("--nav-topic", default="odometry", help="导航用车体位姿话题")
    parser.add_argument("--cmd-topic", default="cmd_vel_sim", help="速度命令话题")
    parser.add_argument(
        "--raw-topic", default="",
        help="原始（未修正）里程计话题。--zlio 时默认 zlio/lidar_odometry，"
             "否则默认 lidar_odometry")
    parser.add_argument(
        "--with-raw", action="store_true",
        help="同时画原始里程计的 yaw（--zlio 时自动开启）")
    parser.add_argument(
        "--zlio", action="store_true",
        help="里程计前端是 ZLIO: 额外订阅原始位姿 + registered_scan(点数/频率) + "
             "zlio_bridge/epoch(换代), 并多画 3 个面板、打印★位移夸大倍数 与 T_off 一致性")
    parser.add_argument(
        "--scan-topic", default="registered_scan",
        help="--zlio 时统计点数/频率的点云话题（默认 registered_scan）")
    parser.add_argument(
        "--epoch-topic", default="zlio_bridge/epoch",
        help="--zlio 时的换代诊断话题（默认 zlio_bridge/epoch）")
    parser.add_argument(
        "--bridge-topic", default="lidar_odometry",
        help="--zlio 时 bridge 发布的**修正后雷达位姿**（默认 lidar_odometry）。"
             "T_off 一致性只能用它与原始位姿配对；用 /odometry（车体位姿）配会画出"
             "每圈云台一个锯齿的假象")
    parser.add_argument(
        "--events-topic", default="zlio_bridge/events",
        help="--zlio 时的诊断事件话题（diagnostic_msgs/DiagnosticArray，默认 "
             "zlio_bridge/events）。ERROR/WARN 会画成竖线，用来定位\"第几秒开始坏的\"。"
             "没有就留空：--events-topic ''")
    parser.add_argument(
        "--matches-topic", default="zlio_bridge/matches",
        help="--zlio 时的匹配点诊断话题（std_msgs/UInt32，默认 zlio_bridge/matches）。"
             "没有就留空：--matches-topic ''")
    parser.add_argument("--imu-topic", default="livox/imu", help="IMU 话题（用于看重力的泄漏）")
    parser.add_argument(
        "--discover-timeout", type=float, default=10.0,
        help="自动找命名空间时最多等多久（秒）。仿真刚起来时话题会分批出现，默认 10 s")
    parser.add_argument("--namespace", default="", help="命名空间，如 red_standard_robot1")
    parser.add_argument(
        "--sim-time", choices=("auto", "on", "off"), default="auto",
        help="用哪个时钟（默认 auto：检测到 /clock 就用仿真时间）。"
             "仿真比实时慢，用墙钟对仿真时间戳会让所有延迟差 1/RTF 倍")
    parser.add_argument(
        "--yaw-debug-topic", default="yaw_debug",
        help="fake_vel_transform 的外推诊断话题（geometry_msgs/Vector3："
             "x=预测 yaw, y=odom yaw, z=积分量，单位 rad）。留空则不看")
    parser.add_argument("--demo", action="store_true", help="用合成数据出图（不需要 ROS）")
    parser.add_argument(
        "--no-caption", action="store_true",
        help="不把图注画进图里（保证图面干净；解释会打到终端）")
    parser.add_argument("--headless", action="store_true", help="不弹窗，只存文件")
    parser.add_argument(
        "--save", default="",
        help="PNG 保存路径。不写则自动放到 <工程根>/user_test_log/ 下，"
             "文件名带时间戳（如 lio_yaw_20261105_0231.png）")
    parser.add_argument(
        "--csv", default="",
        help="CSV 保存路径（默认与 --save 同目录同名，只把后缀换成 .csv）")
    parser.add_argument("--stats-only", action="store_true", help="只打印统计，不画图")

    # 兼容 ros2 run 风格：把 ROS 专有参数剔除，其余照常解析。
    # 关键：不按 --ros-args 截断 —— 否则写在它后面的自定义开关（如 --duration）会被吞掉，
    # 表现为"明明给了时长却一直录"（踩过）。
    if argv is None:
        argv = sys.argv[1:]
    argv, ros_args = _filter_ros_args(argv)
    for i, a in enumerate(ros_args):
        if a.startswith("__ns:="):
            pass
    args = parser.parse_args(argv)
    for a in ros_args:
        if a.startswith("__ns:="):
            args.namespace = a.split(":=", 1)[1]
    for i, a in enumerate(ros_args):
        if a in ("-r", "--remap") and i + 1 < len(ros_args) \
                and ros_args[i + 1].startswith("__ns:="):
            args.namespace = ros_args[i + 1].split(":=", 1)[1]
    if args.help:
        print(__doc__)
        return 0
    # 原始位姿话题按模式取默认：ZLIO 的原始位姿在 zlio/ 子命名空间里（方案 §5.5），
    # 而 lidar_odometry 在 ZLIO 模式下已经是 bridge 修正后的那一份。
    if not args.raw_topic:
        args.raw_topic = "zlio/lidar_odometry" if args.zlio else "lidar_odometry"
    if args.zlio:
        args.with_raw = True


    if args.demo:
        if args.zlio:
            print("使用合成数据（--demo --zlio）：位移夸大 3×、1500 点/帧、T_off 恒定 63°/60 mm、"
                  "第 12 s 换代一次")
        else:
            print("使用合成数据（--demo）：前 8 秒对齐，之后漂移+跳变，并画一个 13 cm 小圆")
        rows = demo_rows(duration=(args.duration if args.duration > 0 else 20.0),
                         zlio=args.zlio)
    else:
        rec = Recorder(args)
        rec.run()
        rows = rec.rows
        if not rows:
            print("[错误] 没有采到数据：确认仿真在跑、命名空间对、话题名对（--gt-topic/--nav-topic）。")
            return 2

    if args.save:
        png_path = os.path.abspath(args.save)
    else:
        out_dir = os.path.join(REPO_ROOT, "user_test_log")
        os.makedirs(out_dir, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M")
        prefix = "zlio_scope" if args.zlio else "lio_yaw"
        png_path = os.path.join(out_dir, "%s_%s.png" % (prefix, stamp))
    if args.csv:
        csv_path = os.path.abspath(args.csv)
    else:
        csv_path = os.path.splitext(png_path)[0] + ".csv"
    save_csv(rows, csv_path)
    print("")
    print("保存位置（都是绝对路径）:")
    print("  图  : %s" % png_path)
    print("  数据: %s" % csv_path)
    events = None if args.demo else getattr(rec, "events", [])
    event_origin = None if args.demo else getattr(rec, "t0_abs", None)
    print_stats(rows, events, event_origin)
    if not args.stats_only:
        plot(rows, png_path, headless=args.headless, zlio=args.zlio,
             caption=not args.no_caption, events=events, event_origin=event_origin)
    return 0


if __name__ == "__main__":
    sys.exit(main())
