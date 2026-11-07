#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# =============================================================
# SRM 底盘 vx/vy/wz 波形调试工具 (debug_tools #1)
#
# 一张图三行，每行一个通道 (vx / vy / wz)，每行两条曲线：
#   ── 串口发送：真正写进 19 字节帧、发到下位机的那三个 float
#   ┈┈ 下位机反馈：下位机 0x0010 包回传的底盘速度观测值（实测值，不是回显）
# 三个通道共用一条时间轴，发送/反馈叠在同一子图里，方便直接看"指令 vs 实际"。
#
# -------------------------------------------------------------
# 数据来源（重要，别把它当成串口抓包工具）
# -------------------------------------------------------------
# 串口发送:
#   本工具**不打开串口**（端口被 srm27_nav_protocol 独占），而是订阅它的唯一输入
#   `cmd_vel_chassis`，然后**复现**节点内部 fillSpeedVector() 的处理顺序：
#       看门狗 (cmd_timeout_sec 内没更新 -> 归零)
#         -> 乘缩放系数 (linear_velocity_scale / angular_velocity_scale)
#         -> 按 max_vx / max_vy / max_wz 逐轴限幅
#         -> 写进帧
#   得到的值就是下位机那一帧里实际收到的数字。这四项参数默认**直接读**
#   src/srm27_nav_protocol/config/srm27_nav_protocol.yaml，改 yaml 这里自动跟，
#   不用改代码；命令行也留了覆盖开关（--max-vx / --linear-scale ...）。
#   注意：这是"按节点逻辑复算"，不是逐字节抓串口。真要看字节，用
#   srm27_nav_protocol 的 debug_print_hex，或后续在 debug_tools 里加串口嗅探。
#
# 下位机反馈:
#   下位机按 0x0010 包回传底盘速度（4 个 float32: vx, vy, wz, yaw_deg），
#   srm27_nav_protocol 解出来发到 `chassis_speed`（geometry_msgs/TwistStamped：
#   twist.linear.x/y = vx/vy，twist.angular.z = wz；yaw 另发 `chassis_yaw_deg`）。
#   本工具订阅它，红色虚线就是**底盘自己观测到的**速度，和蓝线相减即跟随误差：
#     · 默认 --feedback-type twist_stamped --feedback-topic chassis_speed
#     · 上游若改发 geometry_msgs/Twist，用 --feedback-type twist
#     · 若改发 nav_msgs/Odometry，用 --feedback-type odom（读 twist.twist）
#   收不到时图上显示"等待 ..."，先按提示查 publish_chassis_speed 与串口链路。
#
# -------------------------------------------------------------
# 用法
# -------------------------------------------------------------
#   ./debug_tools/plot_vxvywz.py                         # 默认 cmd_vel_chassis + chassis_speed
#   ./debug_tools/plot_vxvywz.py --ros-args -r __ns:=/red_standard_robot1   # 带命名空间时
#   ./debug_tools/plot_vxvywz.py -r __ns:=/red_standard_robot1              # 简写也认
#   ./debug_tools/plot_vxvywz.py --window 60             # 时间窗 60 s
#   ./debug_tools/plot_vxvywz.py --csv /tmp/run.csv      # 同时录 CSV
#   ./debug_tools/plot_vxvywz.py --duration 30 --save /tmp/run.png   # 跑 30 s 后存图退出
#   ./debug_tools/plot_vxvywz.py --demo                  # 不连 ROS，假数据看波形效果
#   ./debug_tools/plot_vxvywz.py --headless --duration 10 --save /tmp/x.png  # 无显示器/ssh
#
# 运行前先 source 环境（只用到标准消息，source 基础环境即可）：
#   source /opt/ros/humble/setup.bash
#   source install/setup.bash        # 若要用本工作空间的消息
#
# -------------------------------------------------------------
# 参数
# -------------------------------------------------------------
#   --send-topic <话题>       串口节点的输入，默认 cmd_vel_chassis（命名空间由 __ns 决定）
#   --feedback-topic <话题>   下位机反馈速度话题，默认 chassis_speed（相对名，随 __ns）
#   --feedback-type twist_stamped|twist|odom
#                             反馈消息类型，默认 twist_stamped（=0x0010 的原生口径）
#   --config <yaml>           srm27_nav_protocol 参数文件，默认自动定位到 src 下那份
#   --max-vx/--max-vy/--max-wz      覆盖 yaml 里的限幅值（默认读 yaml）
#   --linear-scale/--angular-scale  覆盖 yaml 里的缩放系数（默认读 yaml）
#   --cmd-timeout <秒>        覆盖 yaml 里的看门狗超时（默认读 yaml）
#   --window <秒>             波形时间窗，默认 20
#   --sample-hz <Hz>          采样并记录频率，默认 50（串口 100 Hz 发，50 Hz 采样够看形状）
#   --plot-hz <Hz>            界面刷新频率，默认 20
#   --duration <秒>           运行时长，0 = 一直跑到关窗口 / Ctrl-C（默认 0）
#   --csv <路径>              录制 CSV（t, wall, s_vx.., f_vx.., watchdog, clamped）
#   --save <路径>             退出时保存一张 PNG
#   --demo                    合成假数据（不初始化 ROS），用于验证画图链路
#   --headless                不弹窗（Agg 后端），配合 --duration/--save 用
#   -h/--help                 显示本帮助
#
# -------------------------------------------------------------
# 图上怎么看
# -------------------------------------------------------------
#   · 蓝实线：下位机那一帧里实际收到的 vx/vy/wz
#   · 红虚线：下位机 0x0010 回传的实测速度（收不到时留空，并提示在等哪个话题）
#   · 底部状态行：缩放系数、限幅值、看门狗超时、收包计数、限幅次数
#   · 曲线掉到 0 且底部出现"看门狗归零"：说明上游 0.5 s 没发新指令，
#     串口节点在按协议 §6.3 重发零速 —— 这本身就是常见故障，别当成车的问题
#   · 蓝线贴着 max_vx/max_vy 的水平线：限幅生效，限幅次数会累加
# =============================================================

from __future__ import annotations

import argparse
import math
import os
import re
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path

from _common import prepare_matplotlib_env, print_header_help, setup_cjk_font

CHANNELS = ("vx", "vy", "wz")
TITLES = {"vx": "vx  [m/s]", "vy": "vy  [m/s]", "wz": "wz  [rad/s]"}
# 自动缩放时每个通道的最小纵轴跨度，避免"全零时纵轴放大到 1e-18"的鬼畜画面
MIN_SPAN = {"vx": 0.2, "vy": 0.2, "wz": 0.5}

SCRIPT_DIR = Path(__file__).resolve().parent
WORKSPACE_DIR = SCRIPT_DIR.parent
DEFAULT_CONFIG = (
    WORKSPACE_DIR / "src" / "srm27_nav_protocol" / "config" / "srm27_nav_protocol.yaml"
)

# 串口节点 fillSpeedVector() 的默认值，与 srm27_nav_protocol.yaml 一致。
# yaml 读不到时用这一份，并在启动日志里明说"没读到配置"。
PROTOCOL_DEFAULTS = {
    "cmd_timeout_sec": 0.5,
    "max_vx": 4.5,
    "max_vy": 4.5,
    "max_wz": 4.0,
    "linear_velocity_scale": 1.0,
    "angular_velocity_scale": 1.0,
}

CSV_FIELDS = (
    "t_elapsed", "t_wall",
    "s_vx", "s_vy", "s_wz",
    "f_vx", "f_vy", "f_wz",
    "watchdog", "clamped",
)


# =============================================================
# 配置：从 srm27_nav_protocol.yaml 读"串口出口"的真实参数
# =============================================================
def load_protocol_config(path):
    """读 srm27_nav_protocol.yaml 里决定出口数值的那几个参数。

    返回 (values, 说明文字)。任何一步失败都退回 PROTOCOL_DEFAULTS —— 这是调试
    工具，不该因为路径不对就罢工；但一定要把"用的是默认值"说清楚，否则
    波形和车的行为对不上时很难查。
    """
    values = dict(PROTOCOL_DEFAULTS)
    path = Path(path)
    if not path.is_file():
        return values, f"未找到 {path}，使用内置默认值"

    text = ""
    try:
        text = path.read_text(encoding="utf-8")
    except OSError as exc:
        return values, f"读取 {path} 失败 ({exc})，使用内置默认值"

    loaded = False
    try:
        import yaml  # 有 PyYAML 就正经解析
        data = yaml.safe_load(text) or {}
        params = data.get("/srm27_nav_protocol", {}).get("ros__parameters", {})
        for key in values:
            if key in params:
                try:
                    values[key] = float(params[key])
                    loaded = True
                except (TypeError, ValueError):
                    pass
    except Exception:
        # PyYAML 缺失或文件格式异常：退回正则，只认这几个扁平键。
        loaded = False

    if not loaded:
        for key in values:
            match = re.search(rf"^\s*{key}\s*:\s*([-+0-9.eE]+)", text, re.MULTILINE)
            if match:
                try:
                    values[key] = float(match.group(1))
                    loaded = True
                except ValueError:
                    pass

    if not loaded:
        return PROTOCOL_DEFAULTS, f"{path.name} 里没找到这些键，使用内置默认值"
    return values, f"参数来自 {path}"


# =============================================================
# 数据结构
# =============================================================
@dataclass
class Settings:
    send_topic: str = "cmd_vel_chassis"
    feedback_topic: str = "chassis_speed"
    feedback_type: str = "twist_stamped"
    window_sec: float = 20.0
    sample_hz: float = 50.0
    plot_hz: float = 20.0
    duration: float = 0.0
    csv_path: str = ""
    save_path: str = ""
    demo: bool = False
    headless: bool = False
    protocol: dict = field(default_factory=lambda: dict(PROTOCOL_DEFAULTS))
    protocol_note: str = ""

    # 这几个是"串口出口"的口径，波形标题和复算都用它，不要用写死的数字
    @property
    def cmd_timeout(self):
        return float(self.protocol["cmd_timeout_sec"])

    @property
    def max_v(self):
        return (float(self.protocol["max_vx"]),
                float(self.protocol["max_vy"]),
                float(self.protocol["max_wz"]))

    @property
    def scale(self):
        return (float(self.protocol["linear_velocity_scale"]),
                float(self.protocol["linear_velocity_scale"]),
                float(self.protocol["angular_velocity_scale"]))


def reconstruct_serial(raw, age, settings):
    """复现 srm27_nav_protocol::fillSpeedVector()：看门狗 -> 缩放 -> 逐轴限幅。

    raw  : (vx, vy, wz) 最近一次 cmd_vel_chassis 的原始值，None 表示还没收到过
    age  : 距离上一次收到 cmd 的秒数
    返回 (vx, vy, wz, watchdog, clamped) —— 前三个就是写进 19 字节帧的 float。
    """
    if raw is None or age > settings.cmd_timeout:
        # 还没收到 / 超时：节点按协议 §6.3 发零速帧
        return 0.0, 0.0, 0.0, True, False

    vx, vy, wz = raw
    vx *= settings.scale[0]
    vy *= settings.scale[1]
    wz *= settings.scale[2]

    clamped = False
    out = []
    for value, limit in zip((vx, vy, wz), settings.max_v):
        if not math.isfinite(value):
            value, clamped = 0.0, True
        elif abs(value) > limit:
            value, clamped = math.copysign(limit, value), True
        out.append(value)
    return out[0], out[1], out[2], False, clamped


class Recorder:
    """线程安全的样本缓冲 + CSV 落盘 + 统计量。

    ROS 定时器线程写、matplotlib 主线程读，所以所有访问都过锁；锁里只做
    和 list/deque 相关的操作，计算都放在锁外。
    """

    def __init__(self, settings):
        self.settings = settings
        self.lock = threading.Lock()
        self.samples = deque()
        self.stats = {
            "send_msgs": 0,
            "feedback_msgs": 0,
            "clamped": 0,
            "watchdog_episodes": 0,
            "samples": 0,
        }
        self.last_send = (0.0, 0.0, 0.0)
        self.last_feedback = None      # (vx, vy, wz) 或 None
        self.watchdog_active = False
        self._csv = None
        self._csv_rows_since_flush = 0
        if settings.csv_path:
            self._open_csv(settings.csv_path)

    # ------------------------------------------------------------------ CSV
    def _open_csv(self, path):
        path = Path(path)
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            self._csv = path.open("w", newline="", encoding="utf-8")
            self._csv.write(",".join(CSV_FIELDS) + "\n")
        except OSError as exc:
            print(f"[警告] 无法写 CSV {path}: {exc}", file=sys.stderr)
            self._csv = None

    @staticmethod
    def _fmt(value):
        # 没有反馈时写空列，别写 nan 让 Excel 当成文本
        if value is None or (isinstance(value, float) and math.isnan(value)):
            return ""
        return f"{value:.6f}"

    def _write_csv_row(self, elapsed, send, feedback, watchdog, clamped):
        if self._csv is None:
            return
        fb = feedback if feedback is not None else (float("nan"),) * 3
        row = (
            f"{elapsed:.6f}", f"{time.time():.6f}",
            self._fmt(send[0]), self._fmt(send[1]), self._fmt(send[2]),
            self._fmt(fb[0]), self._fmt(fb[1]), self._fmt(fb[2]),
            "1" if watchdog else "0", "1" if clamped else "0",
        )
        self._csv.write(",".join(row) + "\n")
        self._csv_rows_since_flush += 1
        if self._csv_rows_since_flush >= 50:      # 50 Hz 下约每秒落盘一次
            self._csv.flush()
            self._csv_rows_since_flush = 0

    # ------------------------------------------------------------------ 写入
    def bump(self, key, delta=1):
        """统计量自增。三处会写 stats（订阅回调 / 采样定时器 / 绘图），统一走锁。"""
        with self.lock:
            self.stats[key] = self.stats.get(key, 0) + delta

    def push(self, elapsed, send, feedback, watchdog, clamped):
        with self.lock:
            self.samples.append(
                (elapsed, send[0], send[1], send[2],
                 *(feedback if feedback is not None else (float("nan"),) * 3))
            )
            # 时间窗多留 5 s，画图只裁窗口那一段；这样临时拉大窗口还能看到历史
            horizon = elapsed - self.settings.window_sec - 5.0
            while len(self.samples) > 2 and self.samples[0][0] < horizon:
                self.samples.popleft()
            self.stats["samples"] += 1
            self.last_send = send
            # 过期就置空：图上不能出现"下位机已经断链，数值还挂在那里"的假象
            self.last_feedback = feedback
            self.watchdog_active = watchdog
            self._write_csv_row(elapsed, send, feedback, watchdog, clamped)

    def snapshot(self):
        with self.lock:
            return list(self.samples)

    def snapshot_stats(self):
        with self.lock:
            return dict(self.stats)

    def close(self):
        if self._csv is not None:
            try:
                self._csv.flush()
                self._csv.close()
            except OSError:
                pass
            self._csv = None


# =============================================================
# 数据源 1：ROS（订阅 cmd_vel_chassis + 下位机反馈话题）
# =============================================================
class RosSource:
    """rclpy 侧：只收不发，绝不往 /cmd_vel_* 上写东西。"""

    def __init__(self, settings, recorder):
        from geometry_msgs.msg import Twist, TwistStamped
        from nav_msgs.msg import Odometry
        from rclpy.node import Node
        from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

        self.settings = settings
        self.recorder = recorder
        self._cmd = None            # (vx, vy, wz)
        self._cmd_stamp = None
        self._feedback = None       # (vx, vy, wz)
        self._feedback_stamp = None
        self._lock = threading.Lock()
        self._t0 = None
        self._last_t = None
        self._static_streak = 0
        self._use_wall_clock = False
        self._wall_t0 = time.monotonic()
        self._was_fresh = False
        self._started = time.monotonic()
        self._no_cmd_warned = False
        self._first_cmd_logged = False
        self._diag_timer = None

        self.node = Node("chassis_velocity_plot")
        # BEST_EFFORT：上游无论是 reliable 还是 best_effort 都兼容，
        # 而且速度指令丢一帧无所谓，不要让 QoS 不匹配变成"图上一条线都没有"。
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.node.create_subscription(Twist, settings.send_topic, self._on_cmd, qos)

        if settings.feedback_type == "odom":
            self.node.create_subscription(
                Odometry, settings.feedback_topic, self._on_odom, qos)
        elif settings.feedback_type == "twist_stamped":
            self.node.create_subscription(
                TwistStamped, settings.feedback_topic, self._on_feedback_stamped, qos)
        else:
            self.node.create_subscription(
                Twist, settings.feedback_topic, self._on_feedback, qos)

        self._period = 1.0 / max(settings.sample_hz, 1.0)
        self.node.create_timer(self._period, self._sample)
        # 5 s 后问一次"对端在不在"。话题名能查到、但 publisher 数为 0 是现场最容易
        # 误判的情况：话题之所以存在，正是因为这个订阅自己把它挂上了（ros2 topic
        # list 里看得见），于是"有话题、没频率"看起来像工具坏了，其实是节点没起。
        self._diag_timer = self.node.create_timer(5.0, self._diagnose_endpoints)

        # 相对话题名在订阅那一刻已经按命名空间解析过了；把解析结果写回 settings，
        # 图标题和提示里显示的就是**真正在收**的那个话题，排查命名空间时少绕一圈。
        settings.send_topic = self.node.resolve_topic_name(settings.send_topic)
        settings.feedback_topic = self.node.resolve_topic_name(settings.feedback_topic)
        print(f"[话题] 发送 = {settings.send_topic}    "
              f"反馈 = {settings.feedback_topic} ({settings.feedback_type})")

    # ------------------------------------------------------------------ 回调
    def _diagnose_endpoints(self):
        """启动 5 s 后查一次两条话题有没有 publisher，只在没有时说话。"""
        if self._diag_timer is not None:
            self.node.destroy_timer(self._diag_timer)
            self._diag_timer = None
        for label, topic in (("下位机反馈", self.settings.feedback_topic),
                             ("发送指令", self.settings.send_topic)):
            try:
                endpoints = self.node.get_publishers_info_by_topic(topic)
            except Exception:
                endpoints = []
            if endpoints:
                continue
            if label == "下位机反馈":
                print(f"[提示] {topic} 上没有任何 publisher —— 底盘串口节点没在跑"
                      f"（或跑在别的 ROS_DOMAIN_ID / 命名空间）。")
                print("       话题在 ros2 topic list 里看得见，是因为本工具订阅了它，"
                      "不代表有人在发。")
                print("       起节点: ros2 launch srm27_nav_protocol "
                      "srm27_nav_protocol.launch.py")
            else:
                print(f"[提示] {topic} 上没有任何 publisher —— 上游速度出口"
                      f"（srm_cmd_mux）没起，蓝线会一直是 0。")

    def _on_cmd(self, msg):
        with self._lock:
            self._cmd = (msg.linear.x, msg.linear.y, msg.angular.z)
            self._cmd_stamp = time.monotonic()
        self.recorder.bump("send_msgs")
        # 限幅次数按"收到的指令"算：采样定时器 50 Hz 复算同一帧，
        # 在那里计数会把一条指令数成几十次。
        _vx, _vy, _wz, _wd, clamped = reconstruct_serial(
            (msg.linear.x, msg.linear.y, msg.angular.z), 0.0, self.settings)
        if clamped:
            self.recorder.bump("clamped")

    def _on_feedback(self, msg):
        with self._lock:
            self._feedback = (msg.linear.x, msg.linear.y, msg.angular.z)
            self._feedback_stamp = time.monotonic()
        self.recorder.bump("feedback_msgs")

    def _on_feedback_stamped(self, msg):
        # 0x0010 的原生口径：TwistStamped，速度在 msg.twist 里
        twist = msg.twist
        with self._lock:
            self._feedback = (twist.linear.x, twist.linear.y, twist.angular.z)
            self._feedback_stamp = time.monotonic()
        self.recorder.bump("feedback_msgs")

    def _on_odom(self, msg):
        twist = msg.twist.twist
        with self._lock:
            self._feedback = (twist.linear.x, twist.linear.y, twist.angular.z)
            self._feedback_stamp = time.monotonic()
        self.recorder.bump("feedback_msgs")

    # ------------------------------------------------------------------ 时钟
    def _clock_seconds(self):
        """优先用 ROS 时钟（use_sim_time 时自动跟 /clock）；时钟不动就用墙钟。

        为什么还要管这个：开了 use_sim_time 却没有 /clock 时，ROS 时钟恒为 0，
        横轴会退化成一条竖线，看起来像"工具坏了"。这种情况直接退回墙钟，
        并且只说一次 —— 宁可时间基不完美，也别让波形看不见。
        """
        if self._use_wall_clock:
            return time.monotonic() - self._wall_t0

        stamp = self.node.get_clock().now().nanoseconds * 1e-9
        if self._t0 is None:
            self._t0 = stamp
        elapsed = stamp - self._t0

        if self._last_t is not None and abs(elapsed - self._last_t) < 1e-12:
            self._static_streak += 1
        else:
            self._static_streak = 0
        self._last_t = elapsed

        if self._static_streak > 20:               # 连续 20 次采样时间没动
            self._use_wall_clock = True
            self._wall_t0 = time.monotonic() - elapsed
            print("[提示] ROS 时钟不前进（use_sim_time 开了但没有 /clock？），"
                  "时间轴改用墙钟。")
            return elapsed
        return elapsed

    # ------------------------------------------------------------------ 采样
    def _sample(self):
        now = time.monotonic()
        elapsed = self._clock_seconds()

        with self._lock:
            cmd = self._cmd
            cmd_stamp = self._cmd_stamp
            fb = self._feedback
            fb_stamp = self._feedback_stamp

        age = float("inf") if cmd_stamp is None else (now - cmd_stamp)
        vx, vy, wz, watchdog, clamped = reconstruct_serial(cmd, age, self.settings)

        # 反馈也要过期：否则下位机断链后红色虚线会平着无限延伸，误导性极强
        feedback = None
        if fb is not None and fb_stamp is not None and (now - fb_stamp) <= self.settings.cmd_timeout:
            feedback = fb

        # 看门狗"事件"只数 新鲜->失效 的跳变。启动时还没收到过 cmd，
        # 节点本来就在发零速帧，那不是故障，不该记一笔。
        if watchdog and self._was_fresh:
            self.recorder.bump("watchdog_episodes")
        self._was_fresh = not watchdog

        self.recorder.push(elapsed, (vx, vy, wz), feedback, watchdog, clamped)

        # 只提示"启动到现在一条都没收到"，收到第一条就报个就绪 —— 发现延迟
        # 常常要一两秒，拿"3 s 内没有"当故障会在启动瞬间误报。
        if cmd_stamp is None:
            if not self._no_cmd_warned and (now - self._started) > 5.0:
                self._no_cmd_warned = True
                print(f"[提示] 启动 5 s 还没收到 {self.settings.send_topic}，"
                      f"确认底盘串口节点在跑、命名空间对得上"
                      f"（ros2 topic hz {self.settings.send_topic}）。")
        elif not self._first_cmd_logged:
            self._first_cmd_logged = True
            print(f"[就绪] 已收到 {self.settings.send_topic}，开始画发送曲线。")

    # ------------------------------------------------------------------ 生命周期
    def spin_in_background(self):
        from rclpy.executors import SingleThreadedExecutor
        # executor 留个引用：退出时要先让它停，再销毁节点。顺序反了就是
        # "terminate called without an active exception" —— 节点在 spin 线程
        # 里被销毁，rclpy 会直接 abort（这里踩过一次）。
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self.node)
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

    def _spin(self):
        try:
            self._executor.spin()
        except Exception:
            pass

    def shutdown(self):
        try:
            self._executor.shutdown(timeout_sec=1.0)
        except Exception:
            pass
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        try:
            self._executor.remove_node(self.node)
        except Exception:
            pass
        try:
            self.node.destroy_node()
        except Exception:
            pass


# =============================================================
# 数据源 2：演示模式（不依赖 ROS，验证画图/CSV/存图链路）
# =============================================================
class DemoSource:
    """合成一组"看着像那么回事"的速度：加了延迟、衰减和噪声的反馈。"""

    def __init__(self, settings, recorder):
        self.settings = settings
        self.recorder = recorder
        self._stop = threading.Event()
        self._thread = None
        self._history = deque()

    def spin_in_background(self):
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        period = 1.0 / max(self.settings.sample_hz, 1.0)
        start = time.monotonic()
        while not self._stop.is_set():
            elapsed = time.monotonic() - start
            send = (
                0.62 * math.sin(2.0 * math.pi * elapsed / 9.0),
                0.38 * math.cos(2.0 * math.pi * elapsed / 6.5),
                0.9 * math.sin(2.0 * math.pi * elapsed / 4.0 + 0.6),
            )
            # 限幅照抄真实逻辑，让演示里也能看到"贴住上限"的样子
            max_v = self.settings.max_v
            send = tuple(
                math.copysign(min(abs(value), limit), value)
                for value, limit in zip(send, max_v)
            )
            self._history.append((elapsed, send))
            while self._history and self._history[0][0] < elapsed - 0.16:
                self._history.popleft()
            # 反馈 = 约 0.15 s 前的发送 × 增益 + 噪声（模拟下位机跟随滞后与稳态增益）
            delayed = self._history[0][1] if self._history else send
            feedback = tuple(
                value * gain + noise
                for value, gain, noise in zip(
                    delayed, (0.88, 0.92, 0.80), (0.008, -0.006, 0.012))
            )
            self.recorder.push(elapsed, send, feedback, False, False)
            # 演示模式下没有真实报文，计数也补上，免得小结里显示"0 条"让人误会
            self.recorder.bump("send_msgs")
            self.recorder.bump("feedback_msgs")
            time.sleep(period)

    def shutdown(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)


# =============================================================
# 画图
# =============================================================
class Plotter:
    """三个子图（vx / vy / wz），每个子图两条线：串口发送 + 下位机反馈。"""

    SEND_STYLE = dict(color="#1f77b4", linewidth=1.8, label="串口发送 (19B 帧内)")
    FEEDBACK_STYLE = dict(color="#d62728", linewidth=1.6, linestyle="--",
                          label="下位机反馈")

    def __init__(self, settings, recorder, cjk_font):
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation

        self.plt = plt
        self.FuncAnimation = FuncAnimation
        self.settings = settings
        self.recorder = recorder

        self.fig, axes = plt.subplots(
            3, 1, figsize=(11.0, 8.2), sharex=True,
            gridspec_kw={"hspace": 0.28},
        )
        self.axes = list(axes)
        self.lines_send = []
        self.lines_feedback = []
        self.current_texts = []

        for axis, channel in zip(self.axes, CHANNELS):
            (line_send,) = axis.plot([], [], **self.SEND_STYLE)
            (line_feedback,) = axis.plot([], [], **self.FEEDBACK_STYLE)
            axis.set_ylabel(TITLES[channel])
            axis.grid(True, alpha=0.3)
            axis.axhline(0.0, color="#888888", linewidth=0.8, alpha=0.6)
            axis.tick_params(labelbottom=(channel == CHANNELS[-1]))
            self.lines_send.append(line_send)
            self.lines_feedback.append(line_feedback)
            self.current_texts.append(
                axis.text(
                    0.995, 0.88, "", transform=axis.transAxes,
                    ha="right", va="top", fontsize=9,
                    bbox=dict(boxstyle="round,pad=0.3", facecolor="white",
                              edgecolor="#cccccc", alpha=0.85),
                )
            )

        self.axes[0].legend(loc="lower left", fontsize=9, framealpha=0.9)
        self.axes[-1].set_xlabel("时间 [s]")

        self.title = self.fig.suptitle("", fontsize=12, x=0.5, ha="center")
        self.status = self.fig.text(
            0.5, 0.008, "", ha="center", va="bottom", fontsize=9,
            color="#333333",
        )
        self.note = self.fig.text(
            0.5, 0.048, "", ha="center", va="bottom", fontsize=9,
            color="#b25c00", style="italic",
        )
        self._has_feedback = False
        self._cjk_font = cjk_font
        self.fig.subplots_adjust(left=0.08, right=0.985, top=0.90, bottom=0.16)

    # ------------------------------------------------------------------ 绘制
    def redraw(self, _frame=None):
        import numpy as np

        samples = self.recorder.snapshot()
        stats = self.recorder.snapshot_stats()
        settings = self.settings

        if samples:
            data = np.asarray(samples, dtype=float)
            t = data[:, 0]
            window_start = max(t[-1] - settings.window_sec, 0.0)
            mask = t >= window_start
            t = t[mask]
            send = data[mask, 1:4]
            feedback = data[mask, 4:7]
        else:
            t = np.empty(0)
            send = np.empty((0, 3))
            feedback = np.empty((0, 3))

        self._has_feedback = bool(stats["feedback_msgs"])

        for index, channel in enumerate(CHANNELS):
            self.lines_send[index].set_data(t, send[:, index])
            self.lines_feedback[index].set_data(t, feedback[:, index])

            values = np.concatenate((send[:, index], feedback[:, index]))
            finite = values[np.isfinite(values)]
            peak = float(np.max(np.abs(finite))) if finite.size else 0.0
            span = max(MIN_SPAN[channel], peak * 1.25)
            self.axes[index].set_ylim(-span, span)

            send_now = self.recorder.last_send[index]
            fb_now = self.recorder.last_feedback
            fb_value = fb_now[index] if fb_now is not None else float("nan")
            self.current_texts[index].set_text(
                f"发送 {send_now:+.3f}   反馈 "
                + ("   —    " if not np.isfinite(fb_value) else f"{fb_value:+.3f}")
            )

        if t.size:
            self.axes[-1].set_xlim(max(window_start - 0.5, 0.0), t[-1] + 0.5)
        else:
            self.axes[-1].set_xlim(0.0, settings.window_sec)

        max_v = settings.max_v
        scale = settings.scale
        self.title.set_text(
            "底盘 vx/vy/wz 波形 —— "
            f"串口发送: {settings.send_topic}   |   "
            f"下位机反馈: {settings.feedback_topic}"
        )
        self.status.set_text(
            f"缩放 {scale[0]:g}/{scale[2]:g}   限幅 {max_v[0]:g}/{max_v[1]:g}/{max_v[2]:g}"
            f"   看门狗 {settings.cmd_timeout:g} s"
            f"   |   发送 {stats['send_msgs']} 条   反馈 {stats['feedback_msgs']} 条"
            f"   限幅 {stats['clamped']} 次   看门狗归零 {stats['watchdog_episodes']} 次"
        )
        if not self._has_feedback:
            self.note.set_text(
                f"等待下位机速度反馈 {settings.feedback_topic} ({settings.feedback_type})"
                "：查 publish_chassis_speed、串口链路、命名空间"
            )
        else:
            self.note.set_text("")

        self.fig.canvas.draw_idle()
        return ()

    def save(self, path):
        path = Path(path)
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            self.redraw()
            self.fig.savefig(path, dpi=130, bbox_inches="tight")
            print(f"[输出] 波形已保存: {path}")
        except (OSError, ValueError) as exc:
            print(f"[警告] 保存图片失败 {path}: {exc}", file=sys.stderr)

    def close(self):
        self.plt.close(self.fig)


# =============================================================
# 帮助 / 参数
# =============================================================
def print_help():
    """把文件头注释打出来（公共实现，见 _common.py）。"""
    print_header_help(__file__)


def build_settings(argv):
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("-h", "--help", action="store_true")
    parser.add_argument("--send-topic", default="cmd_vel_chassis")
    parser.add_argument("--feedback-topic", default="chassis_speed")
    parser.add_argument("--feedback-type",
                        choices=("twist_stamped", "twist", "odom"),
                        default="twist_stamped")
    parser.add_argument("--config", default=str(DEFAULT_CONFIG))
    parser.add_argument("--max-vx", type=float, default=None)
    parser.add_argument("--max-vy", type=float, default=None)
    parser.add_argument("--max-wz", type=float, default=None)
    parser.add_argument("--linear-scale", type=float, default=None)
    parser.add_argument("--angular-scale", type=float, default=None)
    parser.add_argument("--cmd-timeout", type=float, default=None)
    parser.add_argument("--window", type=float, default=20.0)
    parser.add_argument("--sample-hz", type=float, default=50.0)
    parser.add_argument("--plot-hz", type=float, default=20.0)
    parser.add_argument("--duration", type=float, default=0.0)
    parser.add_argument("--csv", default="")
    parser.add_argument("--save", default="")
    parser.add_argument("--demo", action="store_true")
    parser.add_argument("--headless", action="store_true")

    # --ros-args 之后是 ROS 自己的参数（__ns / use_sim_time / remap），argparse 不认，
    # 先剥出来交给 rclpy.init。`-r`/`-p` 这种 ros2 CLI 的简写也认，免得照着
    # `ros2 run ... -r __ns:=x` 的习惯写过来直接报 unrecognized arguments ——
    # 但只剥"标记 + 一个值"这一对，后面本脚本自己的参数照常解析。
    pair_markers = ("-r", "-p", "--remap", "--param", "--params-file")
    plain_argv, ros_argv = [], []
    index = 0
    while index < len(argv):
        token = argv[index]
        if token == "--ros-args":
            ros_argv.extend(argv[index:])        # 之后的全部交给 ROS
            break
        if token in pair_markers and index + 1 < len(argv):
            ros_argv.extend(argv[index:index + 2])
            index += 2
            continue
        plain_argv.append(token)
        index += 1
    if ros_argv and ros_argv[0] != "--ros-args":
        ros_argv.insert(0, "--ros-args")         # rcl 只解析 --ros-args 之后的部分

    args = parser.parse_args(plain_argv)
    if args.help:
        print_help()
        raise SystemExit(0)

    protocol, note = load_protocol_config(args.config)
    for key, value in (
        ("max_vx", args.max_vx), ("max_vy", args.max_vy), ("max_wz", args.max_wz),
        ("linear_velocity_scale", args.linear_scale),
        ("angular_velocity_scale", args.angular_scale),
        ("cmd_timeout_sec", args.cmd_timeout),
    ):
        if value is not None:
            protocol[key] = float(value)

    settings = Settings(
        send_topic=args.send_topic,
        feedback_topic=args.feedback_topic,
        feedback_type=args.feedback_type,
        window_sec=max(args.window, 1.0),
        sample_hz=max(args.sample_hz, 1.0),
        plot_hz=max(args.plot_hz, 1.0),
        duration=max(args.duration, 0.0),
        csv_path=args.csv,
        save_path=args.save,
        demo=args.demo,
        headless=args.headless,
        protocol=protocol,
        protocol_note=note,
    )
    return settings, ros_argv


def describe_settings(settings):
    scale = settings.scale
    max_v = settings.max_v
    lines = [
        "=" * 66,
        " SRM 底盘 vx/vy/wz 波形调试 (debug_tools/plot_vxvywz.py)",
        "=" * 66,
        f"  串口发送来源 : {settings.send_topic}  (复现 fillSpeedVector: 看门狗->缩放->限幅)",
        f"  下位机反馈   : {settings.feedback_topic}  ({settings.feedback_type})"
        + ("  [演示数据]" if settings.demo else "  [0x0010 实测速度，收到就画]"),
        f"  出口口径     : 缩放 {scale[0]:g}/{scale[2]:g}"
        f"   限幅 {max_v[0]:g}/{max_v[1]:g}/{max_v[2]:g} m/s·rad/s"
        f"   看门狗 {settings.cmd_timeout:g} s",
        f"  参数来源     : {settings.protocol_note}",
        f"  时间窗/采样  : {settings.window_sec:g} s 窗口, "
        f"{settings.sample_hz:g} Hz 采样, {settings.plot_hz:g} Hz 刷新",
    ]
    if settings.csv_path:
        lines.append(f"  CSV          : {settings.csv_path}")
    if settings.save_path:
        lines.append(f"  退出存图     : {settings.save_path}")
    if settings.duration > 0:
        lines.append(f"  运行时长     : {settings.duration:g} s")
    lines.append("=" * 66)
    return "\n".join(lines)


def print_summary(recorder):
    """退出时的文字小结：ssh 里没有窗口时靠它判断波形该长什么样。"""
    samples = recorder.snapshot()
    stats = recorder.snapshot_stats()
    print("\n===== 波形小结 =====")
    print(f"采样点            : {len(samples)}  "
          f"(cmd_vel {stats['send_msgs']} 条, 反馈 {stats['feedback_msgs']} 条)")
    print(f"限幅次数          : {stats['clamped']}")
    print(f"看门狗归零次数    : {stats['watchdog_episodes']}")
    if not samples:
        print("一条样本都没有：检查话题名 / 命名空间，或先用 --demo 验证画图链路。")
        return
    import numpy as _np  # 只在这里用，避免和图后端抢初始化顺序

    data = _np.asarray(samples, dtype=float)
    for index, channel in enumerate(CHANNELS):
        send = data[:, 1 + index]
        send = send[_np.isfinite(send)]
        feedback = data[:, 4 + index]
        feedback = feedback[_np.isfinite(feedback)]
        text = (f"{channel:<3} 发送: 峰值 {_np.max(_np.abs(send)):+.3f} "
                f"均值 {_np.mean(send):+.3f}" if send.size else f"{channel:<3} 发送: 无")
        if feedback.size:
            text += (f"   反馈: 峰值 {_np.max(_np.abs(feedback)):+.3f} "
                     f"均值 {_np.mean(feedback):+.3f}")
        else:
            text += "   反馈: 无数据（等 chassis_speed）"
        print(text)


# =============================================================
# 主流程
# =============================================================
def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    settings, ros_argv = build_settings(argv)

    print(describe_settings(settings))

    prepare_matplotlib_env()
    gui_ok = bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
    headless = settings.headless or not gui_ok
    if not gui_ok and not settings.headless:
        print("[提示] 没有 DISPLAY，自动切到 headless（不弹窗，用 --save 出图）。")

    import matplotlib
    if headless:
        matplotlib.use("Agg")
    else:
        try:
            matplotlib.use("TkAgg")
        except ImportError:
            matplotlib.use("Agg")
            headless = True
            print("[提示] 没有 tkinter/TkAgg，自动切到 headless。")
    import matplotlib.pyplot as plt

    cjk_font = setup_cjk_font()
    if not cjk_font:
        print("[提示] 没找到中文字体，图上中文可能显示成方块（不影响数据）。")

    recorder = Recorder(settings)

    rclpy_module = None
    source = None
    if settings.demo:
        source = DemoSource(settings, recorder)
        source.spin_in_background()
    else:
        try:
            import rclpy
        except ImportError:
            print("[错误] 没有 rclpy。先 source /opt/ros/humble/setup.bash", file=sys.stderr)
            return 2
        rclpy_module = rclpy
        rclpy.init(args=[sys.argv[0]] + ros_argv)
        try:
            source = RosSource(settings, recorder)
        except Exception as exc:                     # 话题类型不对等
            print(f"[错误] 初始化订阅失败: {exc}", file=sys.stderr)
            rclpy.try_shutdown()
            return 1
        source.spin_in_background()

    plotter = Plotter(settings, recorder, cjk_font)
    stop_flag = threading.Event()
    deadline = time.monotonic() + settings.duration if settings.duration > 0 else None
    animation = None

    try:
        if headless:
            # 无窗口：自己按 plot_hz 重画，ssh / 无 DISPLAY 下也能拿到 PNG
            period = 1.0 / settings.plot_hz
            while not stop_flag.is_set():
                plotter.redraw()
                if deadline is not None and time.monotonic() >= deadline:
                    break
                time.sleep(period)
        else:
            def _update(_frame):
                if deadline is not None and time.monotonic() >= deadline:
                    plt.close(plotter.fig)          # 到点自己关窗，退出 plt.show()
                    return ()
                return plotter.redraw(_frame)

            # 不要 plt.ion()：交互模式下 show() 会立刻返回，窗口一闪就没了。
            # 阻塞的 show() 由 Tk 事件循环驱动 FuncAnimation，这是标准用法。
            animation = plotter.FuncAnimation(
                plotter.fig, _update, interval=int(1000.0 / settings.plot_hz),
                blit=False, cache_frame_data=False,
            )
            plt.show(block=True)
        if settings.save_path:
            plotter.save(settings.save_path)
    except KeyboardInterrupt:
        print("\n[中断] Ctrl-C，保存结果后退出。")
        if settings.save_path:
            plotter.save(settings.save_path)
    finally:
        stop_flag.set()
        if animation is not None:
            try:
                animation.event_source.stop()
            except Exception:
                pass
        print_summary(recorder)
        if source is not None:
            source.shutdown()
        if rclpy_module is not None:
            try:
                rclpy_module.try_shutdown()
            except Exception:
                pass
        recorder.close()
        if settings.csv_path:
            print(f"[输出] CSV: {settings.csv_path}")
        try:
            plotter.close()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
