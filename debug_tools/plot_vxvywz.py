#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# =============================================================
# SRM 底盘 vx/vy/wz 波形调试工具 (debug_tools #1)
#
# 一张图四行：
#   第 1~3 行 vx / vy / wz，每行两条曲线：
#     ── 串口发送：真正写进 19 字节帧、发到下位机的那三个 float
#     ┈┈ 下位机反馈：下位机 0x0010 包回传的底盘速度观测值（实测值，不是回显）
#   第 4 行 云台偏角，两条曲线（都是**度**，共用一条轴，方向关系一眼能看出来）：
#     ── chassis_to_gimbal_yaw：下位机原值，底盘相对云台（0x0010 第 4 个 float32）
#     ┈┈ gimbal_yaw_joint：真正接进 URDF/TF 的关节角（云台相对底盘），
#        由 gimbal_joint_sign × 原值 × π/180 得到，这里再换回度来画
#     默认 sign = -1，所以两条线应该**关于 0 镜像**；哪天改成 +1 就会重合。
#     这也是现场核对符号最快的地方：手把底盘逆时针一转，绿色往上、橙色往下就对。
# 四个通道共用一条时间轴，发送/反馈叠在同一子图里，方便直接看"指令 vs 实际"。
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
#   下位机按 0x0010 包回传底盘速度（4 个 float32: vx, vy, wz, chassis_to_gimbal_yaw_deg），
#   srm27_nav_protocol 解出来发到 `chassis_speed`（geometry_msgs/TwistStamped：
#   twist.linear.x/y = vx/vy（底盘系），twist.angular.z = wz；另有
#   `chassis_to_gimbal_yaw_deg`(度) 与 `gimbal_joint_angle`(弧度) 两条 Float32）。
#   本工具订阅全部三条，红色虚线就是**底盘自己观测到的**速度，和蓝线相减即跟随误差：
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
#   ./debug_tools/plot_vxvywz.py --duration 30 --save    # 跑 30 s，图存进 user_test_log/
#   ./debug_tools/plot_vxvywz.py --duration 30 --save /tmp/run.png   # 存到指定路径
#   ./debug_tools/plot_vxvywz.py --demo                  # 不连 ROS，假数据看波形效果
#   ./debug_tools/plot_vxvywz.py --headless --duration 10 --save     # 无显示器/ssh
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
#   --angle-topic <话题>      底盘相对云台（度）话题，默认 chassis_to_gimbal_yaw_deg
#   --joint-topic <话题>      云台关节角（弧度）话题，默认 gimbal_joint_angle
#   --angle-timeout <秒>      偏角/关节角多久没更新就断线，默认 0.5
#                             （0x0010 约 33 Hz；断流时宁可空着，不要画一条平线）
#   --config <yaml>           srm27_nav_protocol 参数文件，默认自动定位到 src 下那份
#   --max-vx/--max-vy/--max-wz      覆盖 yaml 里的限幅值（默认读 yaml）
#   --linear-scale/--angular-scale  覆盖 yaml 里的缩放系数（默认读 yaml）
#   --cmd-timeout <秒>        覆盖 yaml 里的看门狗超时（默认读 yaml）
#   --window <秒>             屏幕上的波形时间窗，默认 20（**不影响存图**：存的是整段）
#   --history <秒>            历史保留上限，默认 0 = 全程保留；只想留最近一段又嫌内存
#                             大时给个上限（例如 600）。存图/退出小结用的是保留的那段
#   --sample-hz <Hz>          采样并记录频率，默认 50（串口 100 Hz 发，50 Hz 采样够看形状）
#   --plot-hz <Hz>            界面刷新频率，默认 20
#   --duration <秒>           运行时长，0 = 一直跑到关窗口 / Ctrl-C（默认 0）
#   --csv <路径>              录制 CSV（t, wall, s_vx.., f_vx.., watchdog, clamped,
#                             a_chassis_to_gimbal_deg, a_joint_rad）
#   --save [路径]             录制结束（关窗口 / Ctrl-C / --duration 到点）存一张 **整段录制** 的
#                             PNG：横轴铺满从启动到退出的全部时间，不是屏幕上那个 --window。
#                             不带路径 -> user_test_log/chassis_vxvywz_<YYYYmmdd_HHMM>.png
#                             （同一分钟重复跑自动加 _2/_3，不覆盖）；给了路径就按路径存
#   --demo                    合成假数据（不初始化 ROS），用于验证画图链路
#   --headless                不弹窗（Agg 后端），配合 --duration/--save 用
#   -h/--help                 显示本帮助
#
# -------------------------------------------------------------
# 图上怎么看
# -------------------------------------------------------------
#   · 蓝实线：下位机那一帧里实际收到的 vx/vy/wz
#   · 红虚线：下位机 0x0010 回传的实测速度（收不到时留空，并提示在等哪个话题）
#   · 第 4 行绿实线 = chassis_to_gimbal_yaw（度，下位机原值），
#     橙虚线 = gimbal_yaw_joint（度，真正进 TF 的关节角）
#       - sign = -1（默认）时两条线**关于 0 镜像**；改成 +1 会重合
#       - 手把底盘逆时针转（云台不动）：绿线涨到 +45 左右、橙线到 -45 左右
#       - 两条线在 ±180 处同时断一下再回来，是角度跨界的正常折叠，不是故障
#       - 某一侧断流：那一条线只断自己（按 --angle-timeout），不会拉平线骗人
#   · 屏幕上横轴只显示最近 --window 秒（实时看形状）；**--save 出来的图是全程**
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

# 第 4 行（云台偏角）的纵轴：最小 ±30°，最多 ±180°（角度本身就被 wrap 在这个范围里）
ANGLE_TITLE = "云台偏角 [deg]"
ANGLE_MIN_SPAN = 30.0
ANGLE_MAX_SPAN = 180.0

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
    # 2026-11-08 追加：第 4 行云台偏角。旧列顺序不动，只往后加，读旧 CSV 的脚本不受影响。
    # a_chassis_to_gimbal_deg = 下位机原值（度）；a_joint_rad = 接进 TF 的关节角（弧度）
    "a_chassis_to_gimbal_deg", "a_joint_rad",
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
    # 第 4 行：下位机原值（度）与换算后的关节角（弧度）两条话题
    angle_topic: str = "chassis_to_gimbal_yaw_deg"
    joint_topic: str = "gimbal_joint_angle"
    angle_timeout: float = 0.5
    # 历史保留时长（秒）。0 = 全程保留：--save 存的是**整段录制**，不是当前窗口。
    # 只想要最近一段又嫌内存大时用 --history 设个上限（例如 600）。
    history_sec: float = 0.0
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
            "angle_msgs": 0,
            "joint_msgs": 0,
            "clamped": 0,
            "watchdog_episodes": 0,
            "samples": 0,
        }
        self.last_send = (0.0, 0.0, 0.0)
        self.last_feedback = None      # (vx, vy, wz) 或 None
        # 第 4 行：(底盘相对云台 度, 关节角 弧度)，任一没数据就是 nan
        self.last_angle = (float("nan"), float("nan"))
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

    def _write_csv_row(self, elapsed, send, feedback, watchdog, clamped, angle):
        if self._csv is None:
            return
        fb = feedback if feedback is not None else (float("nan"),) * 3
        angle_deg, joint_rad = angle
        row = (
            f"{elapsed:.6f}", f"{time.time():.6f}",
            self._fmt(send[0]), self._fmt(send[1]), self._fmt(send[2]),
            self._fmt(fb[0]), self._fmt(fb[1]), self._fmt(fb[2]),
            "1" if watchdog else "0", "1" if clamped else "0",
            self._fmt(angle_deg), self._fmt(joint_rad),
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

    def push(self, elapsed, send, feedback, watchdog, clamped, angle=None):
        if angle is None:
            angle = (float("nan"), float("nan"))
        with self.lock:
            self.samples.append(
                (elapsed, send[0], send[1], send[2],
                 *(feedback if feedback is not None else (float("nan"),) * 3),
                 angle[0], angle[1])
            )
            # 默认**全程保留**（--save 要存整段录制，不能只存当前窗口）。
            # 只想看最近一段又想省内存时用 --history <秒> 给个上限。
            if self.settings.history_sec > 0.0:
                horizon = elapsed - self.settings.history_sec
                while len(self.samples) > 2 and self.samples[0][0] < horizon:
                    self.samples.popleft()
            self.stats["samples"] += 1
            self.last_send = send
            # 过期就置空：图上不能出现"下位机已经断链，数值还挂在那里"的假象
            self.last_feedback = feedback
            self.last_angle = angle
            self.watchdog_active = watchdog
            self._write_csv_row(elapsed, send, feedback, watchdog, clamped, angle)

    def snapshot(self, window_sec=None):
        """取**当前窗口**内的样本（屏幕实时图用）。window_sec=None 用设置里的值。

        缓冲里存的是全程数据，这里只从尾部往回挑窗口那一段，
        所以窗口再大也不会把整段历史复制一遍。
        """
        window = self.settings.window_sec if window_sec is None else window_sec
        with self.lock:
            if not self.samples:
                return []
            horizon = self.samples[-1][0] - window
            picked = []
            for row in reversed(self.samples):
                if row[0] < horizon:
                    break
                picked.append(row)
            picked.reverse()
            return picked

    def snapshot_history(self):
        """取**整段录制**（--save 存图与退出小结用）。"""
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
        from std_msgs.msg import Float32

        self.settings = settings
        self.recorder = recorder
        self._cmd = None            # (vx, vy, wz)
        self._cmd_stamp = None
        self._feedback = None       # (vx, vy, wz)
        self._feedback_stamp = None
        self._angle_deg = None      # 底盘相对云台（度，下位机原值）
        self._angle_stamp = None
        self._joint_rad = None      # 云台关节角（弧度，接进 TF 的那个）
        self._joint_stamp = None
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

        # 第 4 行：0x0010 第 4 个字段的两条 Float32 话题（下位机原值 + 换算后的关节角）。
        # 两条都订：原值要用来看下位机口径，关节角是真正进 TF 的量，缺一不可。
        self.node.create_subscription(
            Float32, settings.angle_topic, self._on_angle, qos)
        self.node.create_subscription(
            Float32, settings.joint_topic, self._on_joint, qos)

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
        settings.angle_topic = self.node.resolve_topic_name(settings.angle_topic)
        settings.joint_topic = self.node.resolve_topic_name(settings.joint_topic)
        print(f"[话题] 发送 = {settings.send_topic}    "
              f"反馈 = {settings.feedback_topic} ({settings.feedback_type})")
        print(f"[话题] 云台偏角 = {settings.angle_topic}    "
              f"关节角 = {settings.joint_topic}")

    # ------------------------------------------------------------------ 回调
    def _diagnose_endpoints(self):
        """启动 5 s 后查一次四条话题有没有 publisher，只在没有时说话。"""
        if self._diag_timer is not None:
            self.node.destroy_timer(self._diag_timer)
            self._diag_timer = None
        checks = (
            ("下位机反馈", self.settings.feedback_topic,
             "底盘串口节点没在跑（或跑在别的 ROS_DOMAIN_ID / 命名空间）。\n"
             "       话题在 ros2 topic list 里看得见，是因为本工具订阅了它，不代表有人在发。\n"
             "       起节点: ros2 launch srm27_nav_protocol srm27_nav_protocol.launch.py"),
            ("发送指令", self.settings.send_topic,
             "上游速度出口（srm_cmd_mux）没起，蓝线会一直是 0。"),
            ("云台偏角", self.settings.angle_topic,
             "串口节点版本旧（还没发这两条话题）或话题名/命名空间不对，第 4 行会一直空着。"),
            ("云台关节角", self.settings.joint_topic,
             "同上；顺带核对 yaml 的 gimbal_joint_topic 与启动参数是否一致。"),
        )
        for label, topic, hint in checks:
            try:
                endpoints = self.node.get_publishers_info_by_topic(topic)
            except Exception:
                endpoints = []
            if endpoints:
                continue
            print(f"[提示] {topic} 上没有任何 publisher —— {label}缺失：")
            print(f"       {hint}")

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

    def _on_angle(self, msg):
        # 0x0010 第 4 个字段的原值：底盘相对云台，单位**度**
        with self._lock:
            self._angle_deg = float(msg.data)
            self._angle_stamp = time.monotonic()
        self.recorder.bump("angle_msgs")

    def _on_joint(self, msg):
        # 换算后的 gimbal_yaw_joint：云台相对底盘，单位**弧度**
        with self._lock:
            self._joint_rad = float(msg.data)
            self._joint_stamp = time.monotonic()
        self.recorder.bump("joint_msgs")

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
            angle_deg = self._angle_deg
            angle_stamp = self._angle_stamp
            joint_rad = self._joint_rad
            joint_stamp = self._joint_stamp

        age = float("inf") if cmd_stamp is None else (now - cmd_stamp)
        vx, vy, wz, watchdog, clamped = reconstruct_serial(cmd, age, self.settings)

        # 反馈也要过期：否则下位机断链后红色虚线会平着无限延伸，误导性极强
        feedback = None
        if fb is not None and fb_stamp is not None and (now - fb_stamp) <= self.settings.cmd_timeout:
            feedback = fb

        # 第 4 行同理：两条话题各自判新鲜度，缺一条就只断那一条（不画平线冒充有效角）。
        # 角度本身是 33 Hz，超时给 angle_timeout；跨过 ±180 的折叠不算断流。
        def _fresh(value, stamp):
            if value is None or stamp is None:
                return float("nan")
            if (now - stamp) > self.settings.angle_timeout:
                return float("nan")
            return float(value)

        angle = (_fresh(angle_deg, angle_stamp), _fresh(joint_rad, joint_stamp))

        # 看门狗"事件"只数 新鲜->失效 的跳变。启动时还没收到过 cmd，
        # 节点本来就在发零速帧，那不是故障，不该记一笔。
        if watchdog and self._was_fresh:
            self.recorder.bump("watchdog_episodes")
        self._was_fresh = not watchdog

        self.recorder.push(elapsed, (vx, vy, wz), feedback, watchdog, clamped, angle)

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
            # 第 4 行也合成一份：底盘相对云台在 ±60° 内来回摆，关节角按默认 sign=-1 取反。
            # 两条线镜像，和真机上"绿上橙下"的观感一致，顺带验证换度画图这段逻辑。
            angle_deg = 60.0 * math.sin(2.0 * math.pi * elapsed / 11.0)
            joint_rad = math.radians(-angle_deg)
            self.recorder.push(elapsed, send, feedback, False, False,
                               (angle_deg, joint_rad))
            # 演示模式下没有真实报文，计数也补上，免得小结里显示"0 条"让人误会
            self.recorder.bump("send_msgs")
            self.recorder.bump("feedback_msgs")
            self.recorder.bump("angle_msgs")
            self.recorder.bump("joint_msgs")
            time.sleep(period)

    def shutdown(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)


# =============================================================
# 画图
# =============================================================
class Plotter:
    """四个子图：vx / vy / wz（串口发送 + 下位机反馈）+ 云台偏角（原值 + 关节角）。"""

    SEND_STYLE = dict(color="#1f77b4", linewidth=1.8, label="串口发送 (19B 帧内)")
    FEEDBACK_STYLE = dict(color="#d62728", linewidth=1.6, linestyle="--",
                          label="下位机反馈")
    # 第 4 行：绿实线 = 下位机原值（底盘相对云台，度）；橙虚线 = 进 TF 的关节角（换回度）
    ANGLE_STYLE = dict(color="#2ca02c", linewidth=1.8,
                       label="chassis_to_gimbal_yaw [deg]（底盘相对云台）")
    JOINT_STYLE = dict(color="#ff7f0e", linewidth=1.6, linestyle="--",
                       label="gimbal_yaw_joint [deg]（云台相对底盘，进 TF）")

    def __init__(self, settings, recorder, cjk_font):
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation

        self.plt = plt
        self.FuncAnimation = FuncAnimation
        self.settings = settings
        self.recorder = recorder

        self.fig, axes = plt.subplots(
            4, 1, figsize=(11.0, 9.8), sharex=True,
            gridspec_kw={"hspace": 0.28, "height_ratios": (1.0, 1.0, 1.0, 1.05)},
        )
        self.axes = list(axes)
        self.axis_angle = self.axes[-1]
        self.lines_send = []
        self.lines_feedback = []
        self.current_texts = []

        for axis, channel in zip(self.axes, CHANNELS):
            (line_send,) = axis.plot([], [], **self.SEND_STYLE)
            (line_feedback,) = axis.plot([], [], **self.FEEDBACK_STYLE)
            axis.set_ylabel(TITLES[channel])
            axis.grid(True, alpha=0.3)
            axis.axhline(0.0, color="#888888", linewidth=0.8, alpha=0.6)
            axis.tick_params(labelbottom=False)      # 只有最下面一行显示横轴刻度
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

        # 第 4 行：云台偏角 / 关节角（度）
        (self.line_angle,) = self.axis_angle.plot([], [], **self.ANGLE_STYLE)
        (self.line_joint,) = self.axis_angle.plot([], [], **self.JOINT_STYLE)
        self.axis_angle.set_ylabel(ANGLE_TITLE)
        self.axis_angle.grid(True, alpha=0.3)
        self.axis_angle.axhline(0.0, color="#888888", linewidth=0.8, alpha=0.6)
        self.axis_angle.tick_params(labelbottom=True)
        self.axis_angle.set_xlabel("时间 [s]")
        self.axis_angle.legend(loc="lower left", fontsize=9, framealpha=0.9)
        self.angle_text = self.axis_angle.text(
            0.995, 0.88, "", transform=self.axis_angle.transAxes,
            ha="right", va="top", fontsize=9,
            bbox=dict(boxstyle="round,pad=0.3", facecolor="white",
                      edgecolor="#cccccc", alpha=0.85),
        )

        self.axes[0].legend(loc="lower left", fontsize=9, framealpha=0.9)

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
        self._has_angle = False
        self._cjk_font = cjk_font
        self.fig.subplots_adjust(left=0.08, right=0.985, top=0.92, bottom=0.14)

    # ------------------------------------------------------------------ 绘制
    def redraw(self, _frame=None, samples=None, window_sec=None):
        import numpy as np

        # samples / window_sec 是给 --save 用的：存图时传入整段录制与全程窗口，
        # 屏幕上的实时图仍然只显示最近 --window 秒。
        if samples is None:
            samples = self.recorder.snapshot()
        stats = self.recorder.snapshot_stats()
        settings = self.settings
        window = settings.window_sec if window_sec is None else window_sec

        if samples:
            data = np.asarray(samples, dtype=float)
            t = data[:, 0]
            window_start = max(t[-1] - window, 0.0)
            mask = t >= window_start
            t = t[mask]
            send = data[mask, 1:4]
            feedback = data[mask, 4:7]
            angle_deg = data[mask, 7]
            joint_rad = data[mask, 8]
        else:
            t = np.empty(0)
            send = np.empty((0, 3))
            feedback = np.empty((0, 3))
            angle_deg = np.empty(0)
            joint_rad = np.empty(0)

        self._has_feedback = bool(stats["feedback_msgs"])
        self._has_angle = bool(stats["angle_msgs"] or stats["joint_msgs"])

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

        # 第 4 行：云台偏角。关节角存的是弧度，画之前换回度，两条线才能同轴比方向。
        joint_deg = np.degrees(joint_rad)
        self.line_angle.set_data(t, angle_deg)
        self.line_joint.set_data(t, joint_deg)
        angle_values = np.concatenate((angle_deg, joint_deg))
        finite = angle_values[np.isfinite(angle_values)]
        peak = float(np.max(np.abs(finite))) if finite.size else 0.0
        angle_span = min(max(ANGLE_MIN_SPAN, peak * 1.25), ANGLE_MAX_SPAN)
        self.axis_angle.set_ylim(-angle_span, angle_span)

        angle_now, joint_now = self.recorder.last_angle
        self.angle_text.set_text(
            "偏角 " + ("   —   " if not np.isfinite(angle_now) else f"{angle_now:+.1f}°")
            + "   关节 "
            + ("   —   " if not np.isfinite(joint_now)
               else f"{math.degrees(joint_now):+.1f}°")
        )

        if t.size:
            self.axes[-1].set_xlim(max(window_start - 0.5, 0.0), t[-1] + 0.5)
        else:
            self.axes[-1].set_xlim(0.0, window)

        max_v = settings.max_v
        scale = settings.scale
        self.title.set_text(
            "底盘 vx/vy/wz + 云台偏角 —— "
            f"发送: {settings.send_topic}   |   反馈: {settings.feedback_topic}   |   "
            f"偏角: {settings.angle_topic} / {settings.joint_topic}"
        )
        self.status.set_text(
            f"缩放 {scale[0]:g}/{scale[2]:g}   限幅 {max_v[0]:g}/{max_v[1]:g}/{max_v[2]:g}"
            f"   看门狗 {settings.cmd_timeout:g} s"
            f"   |   发送 {stats['send_msgs']} 条   反馈 {stats['feedback_msgs']} 条"
            f"   偏角 {stats['angle_msgs']}/{stats['joint_msgs']} 条"
            f"   限幅 {stats['clamped']} 次   看门狗归零 {stats['watchdog_episodes']} 次"
        )
        if not self._has_feedback:
            self.note.set_text(
                f"等待下位机速度反馈 {settings.feedback_topic} ({settings.feedback_type})"
                "：查 publish_chassis_speed、串口链路、命名空间"
            )
        elif not self._has_angle:
            self.note.set_text(
                f"等待云台偏角 {settings.angle_topic} / {settings.joint_topic}"
                "：第 4 行留空（节点没发这两条话题，或话题名/命名空间不对）"
            )
        else:
            self.note.set_text("")

        self.fig.canvas.draw_idle()
        return ()

    def save(self, path):
        """存**整段录制**（从启动到退出），不是屏幕上当前显示的那个窗口。

        屏幕图只留最近 --window 秒（实时看形状方便），存图要能复盘全程，
        所以这里用 snapshot_history() 里的全部样本，把横轴铺满整个录制时长。
        """
        path = Path(path)
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            history = self.recorder.snapshot_history()
            if history:
                span = max(history[-1][0] - history[0][0], 1.0)
                self.redraw(samples=history, window_sec=span + 0.5)
            else:
                self.redraw()
            self.fig.savefig(path, dpi=130, bbox_inches="tight")
            if history:
                print(f"[输出] 波形已保存（整段 {span:.1f} s, "
                      f"{len(history)} 个采样点）: {path}")
            else:
                print(f"[输出] 波形已保存（没有采样点）: {path}")
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


def resolve_save_path(requested, demo=False):
    """把 --save 解析成最终落点（录制结束/关窗口/Ctrl-C 时存这一张）。

    · 没写 --save          -> ""（纯粹当实时示波器，不留文件）
    · --save（不带路径）    -> <工程根>/user_test_log/chassis_vxvywz_<时间戳>.png
    · --save /path/x.png   -> 原样使用（相对路径按当前目录）

    时间戳格式与 plot_lio_gt_yaw.py 保持一致（%Y%m%d_%H%M），落在同一个
    user_test_log/ 里方便一起翻；同一分钟内重复跑自动加 _2/_3，不覆盖旧图。
    """
    if requested is None:
        return ""
    if requested:
        return str(Path(requested).expanduser())

    out_dir = WORKSPACE_DIR / "user_test_log"
    prefix = "chassis_vxvywz_demo" if demo else "chassis_vxvywz"
    stamp = time.strftime("%Y%m%d_%H%M")
    candidate = out_dir / f"{prefix}_{stamp}.png"
    index = 2
    while candidate.exists():
        candidate = out_dir / f"{prefix}_{stamp}_{index}.png"
        index += 1
    return str(candidate)


def build_settings(argv):
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("-h", "--help", action="store_true")
    parser.add_argument("--send-topic", default="cmd_vel_chassis")
    parser.add_argument("--feedback-topic", default="chassis_speed")
    parser.add_argument("--feedback-type",
                        choices=("twist_stamped", "twist", "odom"),
                        default="twist_stamped")
    parser.add_argument("--angle-topic", default="chassis_to_gimbal_yaw_deg")
    parser.add_argument("--joint-topic", default="gimbal_joint_angle")
    parser.add_argument("--angle-timeout", type=float, default=0.5)
    # 0 = 全程保留（--save 才能存整段）；只想要最近一段时给个上限，省内存
    parser.add_argument("--history", type=float, default=0.0)
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
    # --save 不带路径 = 自动落到 <工程根>/user_test_log/（同 plot_lio_gt_yaw.py 的约定），
    # 带路径就按路径存。default=None 用来区分"没要存图"和"要存但没给路径"。
    parser.add_argument("--save", nargs="?", const="", default=None)
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
        angle_topic=args.angle_topic,
        joint_topic=args.joint_topic,
        angle_timeout=max(args.angle_timeout, 0.05),
        history_sec=max(args.history, 0.0),
        window_sec=max(args.window, 1.0),
        sample_hz=max(args.sample_hz, 1.0),
        plot_hz=max(args.plot_hz, 1.0),
        duration=max(args.duration, 0.0),
        csv_path=args.csv,
        save_path=resolve_save_path(args.save, args.demo),
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
        f"  云台偏角     : {settings.angle_topic} (度) -> {settings.joint_topic} (弧度, 进 TF)"
        + ("  [演示数据]" if settings.demo else f"  [超时 {settings.angle_timeout:g} s 断线]"),
        f"  出口口径     : 缩放 {scale[0]:g}/{scale[2]:g}"
        f"   限幅 {max_v[0]:g}/{max_v[1]:g}/{max_v[2]:g} m/s·rad/s"
        f"   看门狗 {settings.cmd_timeout:g} s",
        f"  参数来源     : {settings.protocol_note}",
        f"  时间窗/采样  : {settings.window_sec:g} s 窗口（仅屏幕）, "
        f"{settings.sample_hz:g} Hz 采样, {settings.plot_hz:g} Hz 刷新",
    ]
    lines.append(
        "  历史保留     : "
        + (f"最近 {settings.history_sec:g} s" if settings.history_sec > 0
           else "全程（--save 存整段录制）")
    )
    if settings.csv_path:
        lines.append(f"  CSV          : {settings.csv_path}")
    if settings.save_path:
        lines.append(f"  退出存图     : {settings.save_path}（整段录制，不是屏幕窗口）")
    if settings.duration > 0:
        lines.append(f"  运行时长     : {settings.duration:g} s")
    lines.append("=" * 66)
    return "\n".join(lines)


def print_summary(recorder):
    """退出时的文字小结：ssh 里没有窗口时靠它判断波形该长什么样。"""
    samples = recorder.snapshot_history()
    stats = recorder.snapshot_stats()
    print("\n===== 波形小结 =====")
    print(f"采样点            : {len(samples)}  "
          f"(cmd_vel {stats['send_msgs']} 条, 反馈 {stats['feedback_msgs']} 条, "
          f"偏角 {stats['angle_msgs']}/{stats['joint_msgs']} 条)")
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

    # 第 4 行：云台偏角。关节角画/打印都换回度，和偏角同一口径。
    angle_deg = data[:, 7]
    joint_rad = data[:, 8]
    angle_finite = angle_deg[_np.isfinite(angle_deg)]
    joint_finite = joint_rad[_np.isfinite(joint_rad)]
    if angle_finite.size or joint_finite.size:
        text = "云台偏角          :"
        if angle_finite.size:
            peak = _np.max(_np.abs(angle_finite))
            text += (f" 底盘相对云台 最新 {angle_finite[-1]:+.1f}° 峰值 {peak:.1f}°")
        else:
            text += " 底盘相对云台: 无数据"
        if joint_finite.size:
            text += (f"   关节(进 TF) 最新 {math.degrees(joint_finite[-1]):+.1f}°"
                     f" 峰值 {math.degrees(_np.max(_np.abs(joint_finite))):.1f}°")
        else:
            text += "   关节(进 TF): 无数据"
        print(text)
    else:
        print("云台偏角          : 无数据（等 chassis_to_gimbal_yaw_deg / gimbal_joint_angle）")


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
