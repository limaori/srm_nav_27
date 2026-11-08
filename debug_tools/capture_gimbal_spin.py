#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""自动抓「车不动、只转云台」这一段，出 CSV + 曲线图 + 结论。

为什么要这个工具
================
云台自转、底盘静止时，三路 yaw 满足一条恒等式::

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

判据（斜率 s = Σ(Δyaw·Δθ)/Σ(Δθ²)）
==================================
  ==================  ==========================================================
  s_base ≈ 0          PASS：关节角进了 TF 且符号正确
  s_base ≈ 1          TF 里**没有云台关节角**（/joint_states 没发、relay 超时停发、
                      或 robot_state_publisher 不是 revolute）
  s_base ≈ 2          **关节角符号反了**（gimbal_joint_sign 该翻）
  s_raw ≈ 0           LIO 没在跟踪（话题停更 / 点云或 IMU 断），与补偿无关
  ==================  ==========================================================

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

    def __init__(self, rate_hz):
        import rclpy
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
        self.raw_yaw = self.base_yaw = self.joint_yaw = None
        self.raw_stamp = self.base_stamp = self.joint_stamp = None
        self.counts = {topic: 0 for topic in RAW_CANDIDATES + BASE_CANDIDATES}
        self.counts[JOINT_TOPIC] = 0
        self.counts[JOINT_FALLBACK_TOPIC] = 0
        self._warned_joint_name = False
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
        self.node.create_subscription(Float32, JOINT_FALLBACK_TOPIC, self._joint_float, qos)

    # ---- 回调 ----
    def _stamp(self):
        return self.node.get_clock().now()

    def _odom(self, msg, topic, which):
        self.counts[topic] = self.counts.get(topic, 0) + 1
        current = getattr(self, which + "_topic")
        # 已经锁定一条就别来回切；只有当前这条断流超过 2 s 才允许换源。
        if current is not None and current != topic:
            age = self.age(getattr(self, which + "_stamp"))
            if age < 2.0:
                return
        setattr(self, which + "_topic", topic)
        setattr(self, which + "_yaw", yaw_of(msg.pose.pose.orientation))
        setattr(self, which + "_stamp", self._stamp())
        if which == "raw" and self.on_raw is not None:
            self.on_raw()

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


def analyse(samples):
    """用逐样本增量做回归，得到 s_base / s_raw；对"转过去再转回来"同样有效。"""
    import numpy as np

    if len(samples) < 5:
        return None

    t = np.array([s["t"] for s in samples])
    theta = np.array([s["theta"] for s in samples])
    raw = np.array([s["raw"] for s in samples])
    base = np.array([s["base"] for s in samples])

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

    if energy < 1e-6:
        used["s_base"] = used["s_raw"] = None
        return used

    used["s_base"] = float(np.sum(d_base * d_theta) / energy)
    used["s_raw"] = float(np.sum(d_raw * d_theta) / energy)
    residual = np.degrees(raw - theta - base)  # 应当是一条水平线
    used["residual_deg"] = float(np.mean(residual))
    used["residual_span_deg"] = float(np.max(residual) - np.min(residual))
    used["base_dev_deg"] = float(np.degrees(np.max(np.abs(base - base[0]))))
    used["raw_dev_deg"] = float(np.degrees(np.max(np.abs(raw - raw[0]))))
    used["corr_raw"] = _corr(d_raw, d_theta)
    used["corr_base"] = _corr(d_base, d_theta)
    used["samples"] = samples
    return used


def verdict(stats, tol_deg, min_travel_deg):
    """返回 (退出码, [结论行])。"""
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
    lines = [
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
    parser.add_argument("--tol-deg", type=float, default=3.0,
                        help="底盘 yaw 允许偏移（只用于结论里那句话）")
    parser.add_argument("--min-travel-deg", type=float, default=15.0,
                        help="云台累计转角门槛，低于它不出结论")
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

    collector = Collector(args.rate)
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
            ("t", "theta_deg", "raw_deg", "base_deg", "residual_deg"),
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
            }
            sample["residual"] = wrap(sample["raw"] - sample["theta"] - sample["base"])
            samples.append(sample)
            csv_log.write(
                (
                    "%.3f" % sample["t"],
                    "%.4f" % deg(sample["theta"]),
                    "%.4f" % deg(sample["raw"]),
                    "%.4f" % deg(sample["base"]),
                    "%.4f" % deg(sample["residual"]),
                )
            )
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
            "counts": dict(collector.counts),
        }
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
                        "residual": wrap(raw - theta - base)})
    return samples


def _finish(args, samples, base_path, meta, _unused, demo=False):
    stats = analyse(samples)
    if stats is not None:
        stats["raw_topic"] = meta.get("raw_topic", "?")
        stats["base_topic"] = meta.get("base_topic", "?")
        stats["joint_topic"] = meta.get("joint_topic", "?")
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
