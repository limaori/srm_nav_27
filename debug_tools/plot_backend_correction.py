#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# =============================================================
# Z-Localization backend: map→odom 纠正监视器（先验配准 vs 回环）
#
# 回答一个问题：backend 到底有没有在修正 map→odom？
#   · 先验配准（scan-to-map）修了多少、多久修一次？
#   · 回环（loop closure）闭合时 map→odom 跳了多少？
#   · 这两类修正合起来，把 map→odom 从"刚启动"推到了哪里？
#
# 本工具只读，不发任何控制/服务，不依赖 backend 打开 visualization。
# 它订阅的是 backend 本来就有的四类输出：
#
#   /tf                                   map -> odom 的动态 TF（唯一权威值）
#   /localization/odometry                map 系传感器位姿（画 2D 轨迹 / 把修正箭头
#                                         放到机器人实际位置）
#   <ns>/backend_node/localization_state  状态、epoch、reason（做状态带和统计）
#   /rosout                               backend 的 INFO 日志，用来区分"这次修正
#                                         是先验还是回环"
#
# 为什么必须看 /rosout
# --------------------
# map→odom 是一条连续曲线，先验配准和回环都会改它，但 TF 本身不带"谁改的"。
# backend 在提交修正时会打两类结构化日志：
#
#   先验: global correction committed: runtime=..., sources=prior_scan_to_map, ...,
#         delta_t=...m, delta_r_deg=...
#   回环: loop scan-to-map accepted: latest=..., target=..., ...,
#         delta_t=...m, delta_r_deg=...
#
# 本工具解析这些行拿到"来源 + 日志报告的修正量"，再用 /tf 在事件附近实测
# map→odom 的前后变化。两列会同时打印/存盘：日志报告值 vs TF 实测值。
# 如果两列差很多，通常不是工具算错，而是事件窗口里混进了别的修正，或者
# map→odom 在两次 TF 之间被图优化连续改了几次——这时看面板①/②的趋势，
# 不要只盯单次数字。
#
# UWB
# ---
# UWB 单独修正与 prior_scan_to_map+uwb 联合修正分别标记，联合量不能拆成独立贡献。
# 先验、回环、UWB 由独立开关选择，可同时启用。
# 只有收到明确的 loop=disabled / effective=false 日志才标回环关闭；
# 没有回环事件不代表回环关闭，也可能尚未重访或候选未通过门控。
# --demo 里仍合成两次回环，只为验证本工具的解析/画图链路。
#
# ⚠ map→odom 是**纠正面**，不是定位输出误差
# ------------------------------------------------
# map→odom 会随前端 odom 反向变化：前端在自转/激烈运动下漂了 0.1 m，
# map→odom 就要跟着改 0.1 m，输出位姿（/localization/odometry 或
# TF map→base_link）反而可能是平滑的。判断“定位稳不稳”看输出位姿，
# 不要只看 ①/② 的 map→odom 台阶；本工具 ①/② 是纠正面视图。
#
# -------------------------------------------------------------
# 四个面板
# -------------------------------------------------------------
#   ① map→odom 平移 x/y/z（m）       —— 看纠正后原点被推到哪里、是否连续
#   ② map→odom 姿态 yaw/roll/pitch（°）—— yaw 是平面导航最敏感的一项
#                                       （roll/pitch 用细线，正常应接近 0）
#   ③ 单次修正量（柱=日志报告值）      —— 蓝=先验，橙=回环；实心=平移，×=旋转
#   ④ map 系机器人轨迹 + 修正箭头      —— 箭头从"修正前位置"指向"修正后位置"，
#                                       长度就是这次修正对车实际位置的影响
#
# 竖直虚线 = 回环发生的时刻（先验太多，不画竖线，只画 ③ 的柱）。
# 浅色背景带 = 非 NORMAL 状态（OBS_DEGRADED/SUSPECT/LOST），红色越深越严重。
#
# -------------------------------------------------------------
# 用法
# -------------------------------------------------------------
#   # 1) 先看工具本身（合成数据，不需要 ROS）
#   ./debug_tools/plot_backend_correction.py --demo
#   ./debug_tools/plot_backend_correction.py --demo --headless --duration 25 --save
#
#   # 2) 实车/仿真：backend 已经在跑，另开终端
#   source /opt/ros/humble/setup.bash
#   source install/setup.bash
#   ./debug_tools/plot_backend_correction.py
#   ./debug_tools/plot_backend_correction.py --ros-args -r __ns:=/red_standard_robot1
#   ./debug_tools/plot_backend_correction.py --duration 120 --headless --save /tmp/bc.png
#
#   # 3) 只想要数字，不要图
#   ./debug_tools/plot_backend_correction.py --duration 60 --stats-only
#
#   # 4) backend 没有日志（比如被 --log-level WARN 压掉）时，至少还能看 TF：
#   ./debug_tools/plot_backend_correction.py --rosout-topic ""
#
# 默认结束方式：不带 --duration 就一直录，Ctrl-C 结束时出整段图 + CSV。
#
# -------------------------------------------------------------
# 输出
# -------------------------------------------------------------
#   PNG : <工程根>/user_log/backend_correction_<时间戳>.png
#   CSV : 同名 _samples.csv（map→odom / 里程计 / 状态逐行）
#         同名 _events.csv（每次先验/回环修正一行，含 TF 实测值）
#   终端: 每 5 s 一行当前摘要；结束时打印按来源分组的统计和未归属跳变。
#
# -------------------------------------------------------------
# 判读
# -------------------------------------------------------------
#   · ③ 里先验柱很密（约 2 Hz）、幅度小（mm~cm）→ 正常：先验在持续拉回漂移。
#   · ③ 里回环柱突然很高（dm~m）→ 一次回环闭合，map→odom 被显著修正；
#     同时看 ④ 的箭头方向和 ① 的台阶是否一致。
#   · ④ 箭头方向来回摆、幅度都差不多 → 先验在"追"一个不稳定的观测，
#     常见于先验地图与当前场景不一致、或雷达外参/时间戳有问题。
#   · ① 长期单向漂移、③ 几乎为 0 → 先验没有真正进入图（检查状态是否 NORMAL、
#     看日志 `prior scan-to-map factor accepted` 是否持续出现）。
#   · ② yaw 有台阶但 ③ 没对应柱 → 不是这两类修正造成的，查 reset/UWB/恢复。
#   · "未归属跳变" > 0 → TF 上出现了没有日志对应的大跳，优先查 reset、LOST 冻结
#     或 /rosout 是否漏收。
#
# 注意：工具启动时 backend 可能已经运行很久，所以"首末变化"是**本工具录制窗口内**
# 的变化，不是从 backend 启动那一刻算起。要看完整过程就尽早启动本工具。
# =============================================================

from __future__ import annotations

import argparse
import bisect
import math
import os
import re
import sys
import time
from collections import Counter, deque
from dataclasses import dataclass
from pathlib import Path
from typing import Optional
from urllib.parse import unquote

import numpy as np

from _common import CsvLog, prepare_matplotlib_env, print_header_help, setup_cjk_font

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
DEFAULT_OUT_DIR = REPO_ROOT / "user_log"

DEG2RAD = math.pi / 180.0
# ④ 面板的大致宽高比（数据坐标）；只用来算 x/y 的最小视野，让图面不留大片空白。
XY_ASPECT = 3.2

# 日志里 delta_t / delta_r_deg 的格式由 C++ ostringstream 默认精度决定，
# 可能是 0.012345、1.2e-05 这种形式，统一按浮点解析。
_RE_DELTA_T = re.compile(r"delta_t=([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)\s*m")
_RE_DELTA_R = re.compile(
    r"delta_r_deg=([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)")
_RE_RUNTIME = re.compile(r"runtime=(\d+)")
_RE_SOURCES = re.compile(r"sources=([A-Za-z0-9_+]+)")
_RE_LOOP_INDEX = re.compile(r"latest=(\d+),\s*target=(\d+)")
_RE_NIS = re.compile(r"(?:joint_)?nis=([-+]?[0-9.eE]+)")

SOURCE_COLORS = {
    "prior": "#1f77b4",   # 蓝：先验配准
    "loop": "#ff7f0e",    # 橙：回环
    "uwb": "#7f7f7f",     # 灰：UWB
    "prior_uwb": "#17becf",  # 青：先验与 UWB 联合提交
    "recovery": "#9467bd",  # 紫：LOST/SUSPECT 恢复（不是本次重点）
    "other": "#8c564b",
}
SOURCE_NAMES = {
    "prior": "先验配准",
    "loop": "回环检测",
    "uwb": "UWB",
    "prior_uwb": "先验+UWB 联合",
    "recovery": "恢复重定位",
    "other": "其它",
}
SOURCE_MARKERS = {
    "prior": "o",
    "loop": "^",
    "uwb": "s",
    "prior_uwb": "P",
    "recovery": "D",
    "other": "x",
}


# ---------------------------------------------------------------------------
# 数学 / 格式
# ---------------------------------------------------------------------------
def quaternion_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """单位四元数 -> 3×3 旋转矩阵（行主序 numpy）。"""
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 0.0:
        return np.eye(3)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)],
            [2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)],
            [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)],
        ]
    )


def matrix_yaw_deg(matrix: np.ndarray) -> float:
    """旋转矩阵（或 4×4）的 yaw，单位度。"""
    rotation = matrix[:3, :3]
    return math.degrees(math.atan2(rotation[1, 0], rotation[0, 0]))


def matrix_roll_pitch_deg(matrix: np.ndarray) -> tuple[float, float]:
    """旋转矩阵（或 4×4）的 roll/pitch，单位度。"""
    rotation = matrix[:3, :3]
    roll = math.atan2(rotation[2, 1], rotation[2, 2])
    pitch = math.asin(max(-1.0, min(1.0, -rotation[2, 0])))
    return math.degrees(roll), math.degrees(pitch)


def wrap_deg(angle: float) -> float:
    """把角度折到 (-180, 180]。"""
    return (angle + 180.0) % 360.0 - 180.0


def pose_matrix(x: float, y: float, z: float, qx: float, qy: float, qz: float,
                qw: float) -> np.ndarray:
    """位置 + 四元数 -> 4×4 齐次矩阵。"""
    matrix = np.eye(4)
    matrix[:3, :3] = quaternion_matrix(qx, qy, qz, qw)
    matrix[:3, 3] = (x, y, z)
    return matrix


def fmt(value: float, digits: int = 3) -> str:
    """终端/图上的定宽数字；nan 显示成 --。"""
    if value is None or not math.isfinite(value):
        return "--"
    return f"{value:+.{digits}f}"


def unique_path(path: Path) -> Path:
    """同一分钟内重复运行不覆盖旧图：_2 / _3 ..."""
    if not path.exists():
        return path
    stem, suffix = path.stem, path.suffix
    for index in range(2, 1000):
        candidate = path.with_name(f"{stem}_{index}{suffix}")
        if not candidate.exists():
            return candidate
    return path.with_name(f"{stem}_{int(time.time())}{suffix}")


def _ensure_writable_dir(directory: Path) -> bool:
    """确保目录存在且可写；项目只读/权限不足时返回 False，由调用方回退。"""
    try:
        directory.mkdir(parents=True, exist_ok=True)
    except OSError:
        return False
    return os.access(str(directory), os.W_OK)


def _fallback_output(requested: Path) -> Path:
    """把不可写的输出路径回退到项目 user_log/，只保留原文件名。"""
    fallback = unique_path(DEFAULT_OUT_DIR / requested.name)
    print("[警告] 无法写入 %s（权限/目录不可写），改用 %s"
          % (requested, fallback), file=sys.stderr)
    return fallback


def resolve_png_path(requested: str, default_name: str) -> Path:
    """解析 PNG 输出路径。

    规则：
      * 不写 `--save`：`<工程根>/user_log/<时间戳>.png`；
      * 只给文件名（`--save pic.png`）：落到 `<工程根>/user_log/`；
      * 给绝对路径/带目录的相对路径：尊重用户，但目录不可写时回退到 `user_log/`。
    这样 `--save /user_log/pic.png` 这种把项目目录误写成根目录的写法不会崩，
    也不会把现场数据丢到不可写的地方。
    """
    if not _ensure_writable_dir(DEFAULT_OUT_DIR):
        raise OSError("无法创建/写入输出目录 %s" % DEFAULT_OUT_DIR)
    if not requested:
        return unique_path(DEFAULT_OUT_DIR / default_name)
    path = Path(requested).expanduser()
    if path.is_dir() or requested.endswith(os.sep):
        path = path / default_name
    elif not path.is_absolute() and path.parent == Path("."):
        # 裸文件名默认进项目 user_log/
        path = DEFAULT_OUT_DIR / path.name
    elif not path.is_absolute():
        path = Path.cwd() / path
    if not path.suffix:
        path = path.with_suffix(".png")
    if not _ensure_writable_dir(path.parent):
        return _fallback_output(path)
    return unique_path(path)


def resolve_csv_base(requested: str, png_path: Path) -> Path:
    """解析 CSV 基础路径（最终写 <base>_samples.csv / <base>_events.csv）。"""
    if not requested:
        return png_path.with_suffix("")
    path = Path(requested).expanduser()
    if path.is_dir() or requested.endswith(os.sep):
        path = path / png_path.stem
    elif not path.is_absolute() and path.parent == Path("."):
        path = DEFAULT_OUT_DIR / path.name
    elif not path.is_absolute():
        path = Path.cwd() / path
    if path.suffix:
        path = path.with_suffix("")
    if not _ensure_writable_dir(path.parent):
        return _fallback_output(path)
    return path


# ---------------------------------------------------------------------------
# 数据模型
# ---------------------------------------------------------------------------
@dataclass
class MapOdomSample:
    """一次 map→odom TF 快照。t 是到达本工具的墙钟时刻（秒）。"""

    t: float
    stamp_ns: int
    T: np.ndarray
    x: float
    y: float
    z: float
    roll_deg: float
    pitch_deg: float
    yaw_deg: float


@dataclass
class PoseSample:
    """一次 /localization/odometry 的 map 系传感器位姿。"""

    t: float
    stamp_ns: int
    T: np.ndarray
    x: float
    y: float
    z: float
    yaw_deg: float


@dataclass
class EventEffect:
    """在事件附近由 TF 实测到的 map→odom 前后变化（已换算到机器人位置）。"""

    dxy: float
    dyaw_deg: float
    before_xy: tuple[float, float]
    after_xy: tuple[float, float]
    step_t: float
    step_gap: float
    step_index: int = -1
    # 台阶对 map→odom 本身的增量（不是机器人位置上的位移）。分解用。
    map_dx: float = 0.0
    map_dy: float = 0.0


@dataclass
class CorrectionEvent:
    """一条先验/回环/UWB/恢复修正事件。"""

    t: float
    source: str
    delta_t: float
    delta_r_deg: float
    detail: str
    raw: str
    stamp_ns: int = 0
    effect: Optional[EventEffect] = None


@dataclass
class StateSample:
    """一次 localization_state 快照。"""

    t: float
    status: str
    epoch: int
    reason: str
    text: str


# ---------------------------------------------------------------------------
# /rosout 日志解析
# ---------------------------------------------------------------------------
def parse_backend_log(text: str) -> Optional[CorrectionEvent]:
    """把 backend 的一条**已提交修正**日志解析成事件；不是则返回 None。

    只认三种固定前缀：
      * `loop scan-to-map accepted:`：回环真正写入图；
      * `global correction committed:`：先验/UWB 因子真正写入图；
      * `localization state ->` 且带 delta_t：恢复/重定位真正提交。
    不能再按 "日志里出现 prior/delta_t" 模糊匹配：`prior scan-to-map factor
    conflicted` / `physical correction cap` 也带 delta_t，但它们是**被拒绝**的
    观测，图没有变化，误当修正会把统计放大好几倍（现场实测 0.2~0.5 m 的假事件）。
    """
    delta_t_match = _RE_DELTA_T.search(text)
    if delta_t_match is None:
        return None
    delta_r_match = _RE_DELTA_R.search(text)
    delta_t = float(delta_t_match.group(1))
    delta_r = float(delta_r_match.group(1)) if delta_r_match else 0.0

    if "loop scan-to-map accepted:" in text:
        source = "loop"
        index_match = _RE_LOOP_INDEX.search(text)
        detail = "回环"
        if index_match:
            detail = "latest=%s target=%s" % (index_match.group(1), index_match.group(2))
    elif "global correction committed:" in text:
        sources_match = _RE_SOURCES.search(text)
        sources = sources_match.group(1) if sources_match else ""
        if "prior_scan_to_map" in sources and "uwb" in sources:
            source = "prior_uwb"
        elif "prior_scan_to_map" in sources:
            source = "prior"
        elif "uwb" in sources:
            source = "uwb"
        else:
            source = "other"
        runtime_match = _RE_RUNTIME.search(text)
        detail = "runtime=%s" % (runtime_match.group(1) if runtime_match else "?")
        if sources:
            detail += " sources=%s" % sources
        nis_match = _RE_NIS.search(text)
        if nis_match:
            detail += " nis=%s" % nis_match.group(1)
    elif "localization state ->" in text:
        # 恢复/重定位日志：
        # localization state -> NORMAL (recovered via ..., delta_t=..., delta_r_deg=...)
        source = "recovery"
        detail = text.split("(", 1)[1].rstrip(")")[:160] if "(" in text else "恢复"
    else:
        return None

    return CorrectionEvent(
        t=0.0,
        source=source,
        delta_t=delta_t,
        delta_r_deg=delta_r,
        detail=detail,
        raw=text,
    )


def parse_state_text(text: str) -> Optional[StateSample]:
    """解析 backend 的 `status=...;epoch=...;reason=...` 文本。"""
    fields = {}
    for part in text.split(";"):
        if "=" not in part:
            continue
        key, value = part.split("=", 1)
        fields[key.strip()] = value
    status = fields.get("status")
    if not status:
        return None
    try:
        epoch = int(fields.get("epoch", "0"))
    except ValueError:
        epoch = 0
    reason = unquote(fields.get("reason", ""))
    return StateSample(t=0.0, status=status, epoch=epoch, reason=reason, text=text)


# ---------------------------------------------------------------------------
# 录制容器（demo 与 live 共用）
# ---------------------------------------------------------------------------
class Recording:
    """保存 TF / 里程计 / 事件 / 状态，并提供统计与绘图需要的派生量。"""

    def __init__(self, args):
        self.args = args
        self.start_wall = time.monotonic()
        self.samples: list[MapOdomSample] = []
        self.sample_times: list[float] = []
        self.poses: list[PoseSample] = []
        self.pose_times: list[float] = []
        self.events: list[CorrectionEvent] = []
        self.states: list[StateSample] = []
        self.latest_state: Optional[StateSample] = None
        self.counts = Counter()
        self.seen_logs = deque(maxlen=400)
        self.last_print_wall = 0.0
        # 事件 → TF 台阶的唯一分配结果（索引对应 build_tf_steps 的返回）；
        # 统计"未归属跳变"时用，避免同一台阶被两个事件重复认领。
        self.assigned_steps: set[int] = set()
        # backend 启动时若把 loop.enabled 从 true 覆盖成 false，会在 /rosout 打一条
        # "configuration override: loop.enabled ..."。不根据先验事件推断开关。
        self.loop_override: Optional[str] = None

    def observe_backend_configuration(self, message: str) -> bool:
        """仅使用明确配置日志更新关闭标记，允许新启动的 enabled 状态清除旧标记。"""
        if "unified backend ready:" in message:
            self.loop_override = message if "loop=disabled" in message else None
            return True
        if "configuration override: loop.enabled" in message and "effective=false" in message:
            self.loop_override = message
            return True
        return False

    def now_rel(self) -> float:
        return time.monotonic() - self.start_wall

    def add_map_odom(self, t: float, stamp_ns: int, T: np.ndarray) -> None:
        roll, pitch = matrix_roll_pitch_deg(T)
        sample = MapOdomSample(
            t=t,
            stamp_ns=stamp_ns,
            T=T,
            x=float(T[0, 3]),
            y=float(T[1, 3]),
            z=float(T[2, 3]),
            roll_deg=roll,
            pitch_deg=pitch,
            yaw_deg=matrix_yaw_deg(T),
        )
        self.samples.append(sample)
        self.sample_times.append(t)

    def add_pose(self, t: float, stamp_ns: int, T: np.ndarray) -> None:
        # backend 的 /localization/odometry 可能接近 100 Hz；事件定位和 2D 轨迹
        # 用 20 Hz 足够。降采样只影响本工具的内存/CSV，不改变修正统计。
        odom_hz = getattr(self.args, "odom_hz", 0.0)
        if odom_hz and odom_hz > 0.0 and self.poses:
            if t - self.poses[-1].t < 0.95 / odom_hz:
                return
        sample = PoseSample(
            t=t,
            stamp_ns=stamp_ns,
            T=T,
            x=float(T[0, 3]),
            y=float(T[1, 3]),
            z=float(T[2, 3]),
            yaw_deg=matrix_yaw_deg(T),
        )
        self.poses.append(sample)
        self.pose_times.append(t)

    def add_state(self, t: float, state: StateSample) -> None:
        state.t = t
        self.latest_state = state
        # 状态话题通常跟 TF 同频；只在变化或 0.5 s 以上时存一条，图/CSV 都够用。
        if (not self.states
                or self.states[-1].status != state.status
                or self.states[-1].epoch != state.epoch
                or t - self.states[-1].t >= 0.5):
            self.states.append(state)
            self.counts["state"] += 1

    def add_event(self, event: CorrectionEvent, *, verbose: bool = False,
                  print_delta: float = 0.02) -> None:
        event.t = self.now_rel()
        self.events.append(event)
        self.counts["event_" + event.source] += 1
        if event.source == "loop":
            print("[回环] t=%7.2fs  Δp=%6.3f m  Δyaw=%6.3f°  %s"
                  % (event.t, event.delta_t, event.delta_r_deg, event.detail))
        elif event.source == "uwb":
            print("[UWB ] t=%7.2fs  Δp=%6.3f m  Δyaw=%6.3f°  %s"
                  % (event.t, event.delta_t, event.delta_r_deg, event.detail))
        elif event.source == "recovery":
            print("[恢复] t=%7.2fs  Δp=%6.3f m  Δyaw=%6.3f°  %s"
                  % (event.t, event.delta_t, event.delta_r_deg, event.detail))
        elif verbose or event.delta_t >= print_delta or abs(event.delta_r_deg) >= 1.0:
            print("[%s] t=%7.2fs  Δp=%6.3f m  Δyaw=%6.3f°  %s"
                  % (SOURCE_NAMES.get(event.source, event.source), event.t,
                     event.delta_t, event.delta_r_deg, event.detail))

    def print_status(self) -> None:
        now = time.monotonic()
        if now - self.last_print_wall < 5.0:
            return
        self.last_print_wall = now
        if not self.samples:
            print("[等待] 还没有 map→odom TF；确认 backend 已启动并在发布 "
                  "map->odom（ros2 run tf2_ros tf2_echo map odom）")
            return
        sample = self.samples[-1]
        state = self.latest_state
        state_text = "state=?"
        if state is not None:
            state_text = "%s epoch=%d" % (state.status, state.epoch)
        print("[t=%7.2fs] map→odom x=%s y=%s yaw=%s° | 含先验 %d 回环 %d | %s"
              % (sample.t - self.samples[0].t, fmt(sample.x), fmt(sample.y),
                 fmt(sample.yaw_deg), self.counts.get("event_prior", 0) + self.counts.get("event_prior_uwb", 0),
                 self.counts.get("event_loop", 0), state_text))


# ---------------------------------------------------------------------------
# 事件 → TF 实测
# ---------------------------------------------------------------------------
def _nearest_pose(recording: Recording, t: float) -> Optional[PoseSample]:
    """取事件时刻最近的机器人位姿；没有 /localization/odometry 时返回 None。"""
    if not recording.poses:
        return None
    times = recording.pose_times
    index = bisect.bisect_left(times, t)
    candidates = []
    if index < len(times):
        candidates.append(recording.poses[index])
    if index > 0:
        candidates.append(recording.poses[index - 1])
    if not candidates:
        return None
    best = min(candidates, key=lambda pose: abs(pose.t - t))
    if abs(best.t - t) > 0.6:
        return None
    return best


@dataclass
class TfStep:
    """map→odom 相邻两个 TF 样本之间的台阶（可能是修正，也可能是图优化抖动）。"""

    index: int
    t: float
    start_t: float
    end_t: float
    dxy_origin: float
    dyaw_deg: float
    gap: float


def build_tf_steps(recording: Recording) -> list[TfStep]:
    """把所有非零的相邻 TF 台阶列出来，供事件唯一分配使用。"""
    steps = []
    for index, (first, second) in enumerate(
            zip(recording.samples, recording.samples[1:])):
        gap = second.t - first.t
        if gap <= 0.0 or gap > 1.0:
            continue
        dxy = math.hypot(second.x - first.x, second.y - first.y)
        dyaw = wrap_deg(second.yaw_deg - first.yaw_deg)
        score = dxy + 0.01 * abs(dyaw)
        if score <= 1e-6:
            continue
        steps.append(TfStep(index=index, t=0.5 * (first.t + second.t),
                            start_t=first.t, end_t=second.t,
                            dxy_origin=dxy, dyaw_deg=dyaw, gap=gap))
    return steps


def effect_for_step(recording: Recording, event: CorrectionEvent,
                    step: TfStep) -> EventEffect:
    """把一次 TF 台阶换算成"机器人当时位置"的修正向量。"""
    first = recording.samples[step.index]
    second = recording.samples[step.index + 1]
    pose = _nearest_pose(recording, event.t)
    if pose is not None:
        before = first.T @ pose.T
        after = second.T @ pose.T
        dxy = math.hypot(after[0, 3] - before[0, 3], after[1, 3] - before[1, 3])
        before_xy = (float(before[0, 3]), float(before[1, 3]))
        after_xy = (float(after[0, 3]), float(after[1, 3]))
    else:
        dxy = step.dxy_origin
        before_xy = (first.x, first.y)
        after_xy = (second.x, second.y)
    delta = second.T @ np.linalg.inv(first.T)
    return EventEffect(
        dxy=dxy,
        dyaw_deg=step.dyaw_deg,
        before_xy=before_xy,
        after_xy=after_xy,
        step_t=step.t,
        step_gap=step.gap,
        step_index=step.index,
        map_dx=float(delta[0, 3]),
        map_dy=float(delta[1, 3]),
    )


def analyze_effects(recording: Recording) -> None:
    """给每条事件唯一分配一次 TF 台阶。

    不能简单地"在窗口内取最大台阶"：先验和回环可能在同一帧附近提交，
    一个大台阶会被两条事件重复认领。这里用全局贪心分配：

        cost = 2×时间差 + |TF实测平移 − 日志报告平移| + 0.02×旋转差

    时间差保证选离事件最近的台阶；量级项保证同时间的大台阶优先给回环、
    小台阶优先给先验。每个台阶最多被一条事件认领，认领不到的记 None。
    """
    before = getattr(recording.args, "match_before", 0.25)
    after = getattr(recording.args, "match_after", 0.35)
    for event in recording.events:
        event.effect = None
    steps = build_tf_steps(recording)
    if not steps or not recording.events:
        recording.assigned_steps = set()
        return
    # 用台阶的“区间”而不是中点匹配：TF 偶尔有 0.5 s 的传输空档，
    # 一次 0.17 m 的修正在样本上会表现为 85.14->85.65 的长台阶，
    # 中点 85.39 会落在事件 t=85.65 的 0.25 s 窗口外，导致漏归属。
    step_starts = [step.start_t for step in steps]
    step_ends = [step.end_t for step in steps]
    candidates = []
    for event_index, event in enumerate(recording.events):
        lo = bisect.bisect_left(step_ends, event.t - before)
        hi = bisect.bisect_right(step_starts, event.t + after)
        for step_index in range(lo, hi):
            step = steps[step_index]
            # 事件之前必须已经有 TF 样本作为基线。启动时的恢复/初始化事件
            # 常常在第一条 map→odom 之前提交，前面根本没有 before 位姿；
            # 这时不能拿后面的正常台阶硬凑一个“TF 实测”。
            if recording.samples[step.index].t > event.t + 0.02:
                continue
            effect = effect_for_step(recording, event, step)
            # 事件落在台阶区间内时间差为 0；否则取到区间端点的距离。
            if event.t < step.start_t:
                time_delta = step.start_t - event.t
            elif event.t > step.end_t:
                time_delta = event.t - step.end_t
            else:
                time_delta = 0.0
            # 平移量级权重 5.0：同一次 transaction 里回环的大台阶和小先验台阶
            # 可能只差几十毫秒，若时间权重压过量级，回环的大台阶会被先验认领。
            cost = (2.0 * time_delta
                    + 5.0 * abs(effect.dxy - event.delta_t)
                    + 0.02 * abs(abs(effect.dyaw_deg) - abs(event.delta_r_deg)))
            candidates.append((cost, event_index, step_index, effect))
    candidates.sort(key=lambda item: item[0])

    used_events: set[int] = set()
    used_steps: set[int] = set()
    for _cost, event_index, step_index, effect in candidates:
        if event_index in used_events or step_index in used_steps:
            continue
        recording.events[event_index].effect = effect
        used_events.add(event_index)
        used_steps.add(step_index)
    # candidates 里存的是 steps 列表下标；unassigned_steps 用的是样本对下标
    # （TfStep.index），这里转换一次，别把两种下标混用。
    recording.assigned_steps = {steps[index].index for index in used_steps}


def unassigned_steps(recording: Recording) -> list[tuple[float, float, float]]:
    """TF 上存在、但没有任何修正事件认领的大台阶。

    返回 [(t, dxy, dyaw_deg)]。用来发现 reset/LOST 冻结或漏收 /rosout。
    """
    if len(recording.samples) < 2:
        return []
    args = recording.args
    assigned = getattr(recording, "assigned_steps", set())
    result = []
    for step in build_tf_steps(recording):
        if step.index in assigned:
            continue
        if step.dxy_origin < args.unassigned_translation and \
                abs(step.dyaw_deg) < args.unassigned_yaw:
            continue
        result.append((step.t, step.dxy_origin, abs(step.dyaw_deg)))
    return result


# ---------------------------------------------------------------------------
# 统计
# ---------------------------------------------------------------------------
def _source_events(recording: Recording, source: str) -> list[CorrectionEvent]:
    return [event for event in recording.events if event.source == source]


def print_summary(recording: Recording) -> None:
    """终端统计：先验/回环各自的事件数、日志量、TF 实测量、未归属跳变。"""
    samples = recording.samples
    print("")
    print("=" * 78)
    print("map→odom 纠正统计（本工具录制窗口内）")
    print("=" * 78)
    if not samples:
        print("没有 map→odom 样本。")
        return

    duration = samples[-1].t - samples[0].t
    rate = (len(samples) - 1) / duration if duration > 1e-9 else float("nan")
    print("样本: %d 个 map→odom TF，时长 %.2f s（%.1f Hz），里程计 %d 条，状态 %d 条"
          % (len(samples), duration, rate, len(recording.poses), len(recording.states)))

    first, last = samples[0], samples[-1]
    net = last.T @ np.linalg.inv(first.T)
    net_xy = math.hypot(net[0, 3], net[1, 3])
    net_yaw = wrap_deg(last.yaw_deg - first.yaw_deg)
    print("当前 map→odom: x=%s y=%s z=%s yaw=%s°"
          % (fmt(last.x), fmt(last.y), fmt(last.z), fmt(last.yaw_deg)))
    print("首末变化    : |Δxy|=%.4f m, Δyaw=%s°（首=%.3f/%.3f/%.3f°）"
          % (net_xy, fmt(net_yaw), first.x, first.y, first.yaw_deg))

    for source in ("prior", "prior_uwb", "loop", "uwb", "recovery", "other"):
        events = _source_events(recording, source)
        if not events and source not in ("prior", "loop"):
            continue
        logged_t = np.array([event.delta_t for event in events], dtype=float)
        logged_r = np.array([event.delta_r_deg for event in events], dtype=float)
        effects = [event.effect for event in events if event.effect is not None]
        measured_t = np.array([effect.dxy for effect in effects], dtype=float)
        measured_r = np.array([abs(effect.dyaw_deg) for effect in effects], dtype=float)
        label = SOURCE_NAMES.get(source, source)
        if not events:
            print("%s: 0 次" % label)
            continue
        line = ("%s: %d 次 | 日志 |Δp| 平均 %.4f m / 最大 %.4f m / Σ %.3f m"
                % (label, len(events), float(np.mean(logged_t)), float(np.max(logged_t)),
                   float(np.sum(logged_t))))
        if len(measured_t):
            line += (" | TF实测(%d/%d) 平均 %.4f m / 最大 %.4f m"
                     % (len(effects), len(events), float(np.mean(measured_t)),
                        float(np.max(measured_t))))
        else:
            line += " | TF实测 未捕获"
        print(line)
        if len(logged_r):
            print("      Δyaw 日志: 平均 %.3f° / 最大 %.3f°%s"
                  % (float(np.mean(np.abs(logged_r))), float(np.max(np.abs(logged_r))),
                     (" | TF实测 最大 %.3f°" % float(np.max(measured_r)))
                     if len(measured_r) else ""))

    uwb_events = _source_events(recording, "uwb")
    if not uwb_events:
        print("UWB 单独修正: 0 次（不代表关闭；先验+UWB 联合提交另计）")
    else:
        print("UWB 单独修正: 检测到 %d 次事件"
              % len(uwb_events))

    # 状态占比按消息条数近似时间占比：状态话题与 TF 同频时足够判断。
    if recording.states:
        counter = Counter(state.status for state in recording.states)
        total = sum(counter.values())
        text = ", ".join("%s %.0f%%" % (name, 100.0 * count / total)
                         for name, count in counter.most_common())
        epochs = sorted({state.epoch for state in recording.states})
        print("状态: %s | epoch: %s" % (text, ",".join(map(str, epochs))))
        if recording.latest_state is not None:
            print("最近 reason: %s" % recording.latest_state.reason[:180])

    steps = unassigned_steps(recording)
    if steps:
        worst = max(steps, key=lambda item: item[1] + 0.01 * item[2])
        print("未归属跳变: %d 次（最大 t=%.2fs |Δxy|=%.3f m |Δyaw|=%.2f°）"
              % (len(steps), worst[0] - samples[0].t, worst[1], worst[2]))
        print("            这些台阶在 /rosout 里没有对应修正日志，查 reset/LOST 冻结/日志级别")
    else:
        print("未归属跳变: 0 次（TF 上所有大台阶都有日志对应）")

    if not recording.events:
        print("")
        print("未捕获到任何修正事件。确认：")
        print("  1) backend 处于 NORMAL 且先验配准在持续成功：")
        print("     ros2 topic echo /rosout | grep -E 'prior scan-to-map|global correction|loop scan'")
        print("  2) 日志级别没有被压到 WARN 以上（本工具依赖 INFO 行）")
        print("  3) 节点名匹配（--node-match backend；backend 的日志器名是 backend，不是 backend_node）")
        if recording.loop_override:
            print("另：已收到明确的回环关闭日志：%s" % recording.loop_override)
    elif not _source_events(recording, "loop"):
        print("")
        if recording.loop_override:
            print("本次没有回环事件：已收到回环关闭日志。")
            print("  %s" % recording.loop_override)
        else:
            print("本次没有回环事件，不能仅凭先验事件判断开关。检查 --zloc-loop / 启动能力日志，")
            print("以及轨迹是否重访、exclude_recent、candidate_max_distance 和几何/PCM 门控。")


# ---------------------------------------------------------------------------
# 绘图
# ---------------------------------------------------------------------------
class ReportFigure:
    """4 面板报告图；live 时反复 update，结束时 save。"""

    def __init__(self, args):
        import matplotlib.pyplot as plt

        self.plt = plt
        self.args = args
        self.fig = plt.figure(figsize=(13.6, 12.8))

    def update(self, recording: Recording, caption: bool = True,
               window: float = 0.0) -> None:
        from matplotlib.lines import Line2D
        from matplotlib.patches import FancyArrowPatch

        self.fig.clf()
        if not recording.samples:
            self.fig.text(0.5, 0.5, "等待 map→odom TF ...", ha="center", va="center",
                          fontsize=14)
            return

        samples = recording.samples
        events = recording.events
        poses = recording.poses
        states = recording.states
        if window > 0.0:
            cutoff = samples[-1].t - window
            samples = [sample for sample in samples if sample.t >= cutoff]
            events = [event for event in events if event.t >= cutoff]
            poses = [pose for pose in poses if pose.t >= cutoff]
            states = [state for state in states if state.t >= cutoff - 1.0]
        t0 = samples[0].t
        times = np.array([sample.t - t0 for sample in samples])
        xs = np.array([sample.x for sample in samples])
        ys = np.array([sample.y for sample in samples])
        zs = np.array([sample.z for sample in samples])
        yaws = np.array([sample.yaw_deg for sample in samples])
        rolls = np.array([sample.roll_deg for sample in samples])
        pitches = np.array([sample.pitch_deg for sample in samples])

        grid = self.fig.add_gridspec(
            4, 1, height_ratios=[1.15, 0.85, 1.10, 1.35], hspace=0.36)
        ax_t = self.fig.add_subplot(grid[0])
        ax_y = self.fig.add_subplot(grid[1], sharex=ax_t)
        ax_c = self.fig.add_subplot(grid[2], sharex=ax_t)
        ax_xy = self.fig.add_subplot(grid[3])

        # 非 NORMAL 状态带：状态话题只有变化时存，所以相邻两条之间整段涂色。
        status_colors = {
            "OBS_DEGRADED": "#ffe08a",
            "SUSPECT": "#ffb0a0",
            "LOST": "#ff6b6b",
            "INITIALIZING": "#b7d7ff",
        }
        for index, state in enumerate(states[:-1]):
            color = status_colors.get(state.status)
            if color is None:
                continue
            start = max(state.t - t0, 0.0)
            end = max(states[index + 1].t - t0, start)
            for axis in (ax_t, ax_y, ax_c):
                axis.axvspan(start, end, color=color, alpha=0.16, linewidth=0)

        # ① map→odom 平移
        ax_t.plot(times, xs, color="#1f77b4", lw=1.2, label="x")
        ax_t.plot(times, ys, color="#ff7f0e", lw=1.2, label="y")
        ax_t.plot(times, zs, color="#2ca02c", lw=1.0, alpha=0.8, label="z")
        ax_t.set_ylabel("map→odom 平移 [m]")
        ax_t.grid(True, alpha=0.28)
        ax_t.legend(loc="upper left", ncol=3, fontsize=8, framealpha=0.9)
        ax_t.set_title("① map→odom 平移：纠正后 odom 原点在地图里的位置", fontsize=10,
                       loc="left")

        # ② map→odom 姿态
        ax_y.plot(times, yaws, color="#9467bd", lw=1.3, label="yaw")
        ax_y.plot(times, rolls, color="#8c564b", lw=0.8, alpha=0.45, label="roll")
        ax_y.plot(times, pitches, color="#17becf", lw=0.8, alpha=0.45, label="pitch")
        ax_y.set_ylabel("map→odom 姿态 [deg]")
        ax_y.grid(True, alpha=0.28)
        ax_y.legend(loc="upper left", ncol=3, fontsize=8, framealpha=0.9)
        ax_y.set_title("② map→odom 姿态：yaw 是平面导航最敏感的一项", fontsize=10,
                       loc="left")

        # 回环时刻竖线（先验太多，不画）
        loop_events = [event for event in events if event.source == "loop"]
        prior_events = [event for event in events if event.source in ("prior", "prior_uwb")]
        loop_disabled = recording.loop_override is not None
        for event in loop_events:
            for axis in (ax_t, ax_y, ax_c):
                axis.axvline(event.t - t0, color=SOURCE_COLORS["loop"], lw=0.9,
                             ls="--", alpha=0.55)

        # ③ 单次修正量
        ax_c_right = ax_c.twinx()
        handles = []
        for source in ("prior", "prior_uwb", "loop", "uwb", "recovery"):
            source_events = [event for event in events if event.source == source]
            if not source_events:
                continue
            event_times = np.array([event.t - t0 for event in source_events])
            delta_t = np.array([event.delta_t for event in source_events])
            delta_r = np.array([event.delta_r_deg for event in source_events])
            color = SOURCE_COLORS[source]
            marker = SOURCE_MARKERS[source]
            ax_c.vlines(event_times, 0.0, delta_t, color=color, lw=0.9, alpha=0.35)
            ax_c.scatter(event_times, delta_t, s=18, color=color, marker=marker,
                         alpha=0.85, zorder=3)
            ax_c_right.scatter(event_times, np.abs(delta_r), s=14, color=color,
                               marker="x", alpha=0.55, zorder=2)
            handles.append(
                Line2D([0], [0], color=color, marker=marker, lw=0, markersize=5,
                       label="%s 平移" % SOURCE_NAMES[source]))
            handles.append(
                Line2D([0], [0], color=color, marker="x", lw=0, markersize=5,
                       label="%s 旋转" % SOURCE_NAMES[source]))
        if handles:
            ax_c.legend(handles=handles, loc="upper left", ncol=2, fontsize=7.5,
                        framealpha=0.9)
        else:
            empty_text = ("未捕获到修正日志\n"
                          "确认 backend 处于 NORMAL、日志级别为 INFO、--node-match 正确")
            if loop_disabled:
                empty_text += "\n回环：配置日志明确关闭"
            ax_c.text(0.5, 0.55, empty_text,
                      transform=ax_c.transAxes, ha="center", va="center", fontsize=10,
                      color="#555555")
            # 没有事件时不要让纵轴自动撑到 0~1 m，否则看起来像“有数据但没画”。
            ax_c.set_ylim(0.0, 0.1)
            ax_c_right.set_ylim(0.0, 2.0)
        ax_c.set_ylabel("单次平移修正 |Δp| [m]")
        ax_c_right.set_ylabel("单次旋转修正 |Δyaw| [deg]")
        ax_c.grid(True, alpha=0.28)
        ax_c.set_title("③ 单次修正量：柱=日志报告值；蓝=先验，橙=回环；×=旋转量",
                       fontsize=10, loc="left")

        # ④ map 系轨迹 + 修正箭头
        if poses:
            pose_x = np.array([pose.x for pose in poses])
            pose_y = np.array([pose.y for pose in poses])
            traj_x, traj_y = pose_x, pose_y
            ax_xy.plot(pose_x, pose_y, color="#444444", lw=1.0, alpha=0.65,
                       label="机器人轨迹")
            ax_xy.scatter(pose_x[0], pose_y[0], s=55, color="#2ca02c", marker="o",
                          zorder=5, label="起点")
            ax_xy.scatter(pose_x[-1], pose_y[-1], s=110, color="#000000", marker="*",
                          zorder=5, label="当前")
            panel_title = "④ map 系机器人轨迹 + 单次修正（箭头=修正前→后）"
        else:
            traj_x, traj_y = xs, ys
            ax_xy.plot(xs, ys, color="#444444", lw=1.0, alpha=0.65,
                       label="map→odom 原点轨迹")
            ax_xy.scatter(xs[0], ys[0], s=55, color="#2ca02c", marker="o", zorder=5,
                          label="起点")
            ax_xy.scatter(xs[-1], ys[-1], s=110, color="#000000", marker="*",
                          zorder=5, label="当前")
            panel_title = "④ map→odom 原点轨迹（未收到 /localization/odometry，箭头画在原点）"

        arrow_order = sorted(
            [event for event in events if event.effect is not None],
            key=lambda event: event.t,
        )
        drawn_prior = 0
        drawn_loop = 0
        for event in arrow_order:
            effect = event.effect
            if effect is None:
                continue
            if event.source in ("prior", "prior_uwb") and effect.dxy < self.args.arrow_min:
                continue
            if event.source not in ("prior", "prior_uwb", "loop"):
                continue
            color = SOURCE_COLORS[event.source]
            lw = 0.9 if event.source in ("prior", "prior_uwb") else 1.8
            alpha = 0.55 if event.source in ("prior", "prior_uwb") else 0.95
            arrow = FancyArrowPatch(
                effect.before_xy, effect.after_xy,
                arrowstyle="-|>", mutation_scale=8 if event.source in ("prior", "prior_uwb") else 14,
                lw=lw, color=color, alpha=alpha, zorder=4,
                shrinkA=0.0, shrinkB=0.0,
            )
            ax_xy.add_patch(arrow)
            if event.source == "loop":
                drawn_loop += 1
                ax_xy.annotate(
                    "loop %.0fs\n%.2fm/%.1f°" % (event.t - t0, effect.dxy, effect.dyaw_deg),
                    xy=effect.after_xy, xytext=(8, 8), textcoords="offset points",
                    fontsize=7.5, color=color,
                    bbox=dict(boxstyle="round,pad=0.2", fc="white", ec=color, alpha=0.85),
                )
            else:
                drawn_prior += 1

        ax_xy.set_xlabel("map x [m]")
        ax_xy.set_ylabel("map y [m]")
        ax_xy.grid(True, alpha=0.28)
        ax_xy.legend(loc="best", fontsize=8, framealpha=0.9)
        ax_xy.set_title(panel_title, fontsize=10, loc="left")

        # 给 ④ 一个最小视野。机器人静止时 /localization/odometry 有 mm 级抖动，
        # 自动缩放到 1 cm 会画成一团“毛线球”，反而看不清修正箭头；最小 2 m
        # 视野下，停车抖动只是一个点，而 cm 级修正箭头仍然可见。
        draw_x = list(traj_x)
        draw_y = list(traj_y)
        for event in events:
            if event.effect is None:
                continue
            draw_x.extend([event.effect.before_xy[0], event.effect.after_xy[0]])
            draw_y.extend([event.effect.before_xy[1], event.effect.after_xy[1]])
        if draw_x:
            xmin, xmax = min(draw_x), max(draw_x)
            ymin, ymax = min(draw_y), max(draw_y)
            data_span = max(xmax - xmin, ymax - ymin)
            center_x = 0.5 * (xmin + xmax)
            center_y = 0.5 * (ymin + ymax)
            if data_span < 0.5:
                # 静止/几乎不动：等比例放大到 1 cm 会变成毛线球，这里用最小视野，
                # 并允许 x/y 像素比例不同（2 m 视野铺满宽面板），只求“点在哪”。
                span = max(data_span * 1.08, self.args.xy_min_span, 1.0e-3)
                ax_xy.set_xlim(center_x - 0.5 * span, center_x + 0.5 * span)
                ax_xy.set_ylim(center_y - 0.5 * span, center_y + 0.5 * span)
                ax_xy.set_aspect("auto")
            else:
                # 有实际运动：保持 1:1，按面板宽高比算最小 x 跨度；显式设限 + box，
                # 避免 datalim 把 x 轴撑到十几米、把轨迹压成中间一小团。
                x_span = max(xmax - xmin, self.args.xy_min_span * XY_ASPECT,
                             self.args.xy_min_span, 1.0e-3) * 1.08
                y_span = max(ymax - ymin, self.args.xy_min_span, 1.0e-3) * 1.08
                ax_xy.set_xlim(center_x - 0.5 * x_span, center_x + 0.5 * x_span)
                ax_xy.set_ylim(center_y - 0.5 * y_span, center_y + 0.5 * y_span)
                ax_xy.set_aspect("equal", adjustable="box")
        else:
            ax_xy.set_aspect("auto")

        text_lines = [
            "含先验修正: %d 次" % len(prior_events),
            "回环检测: %d 次" % len(loop_events),
            "箭头(≥%.0fmm): 先验 %d / 回环 %d"
            % (self.args.arrow_min * 1000.0, drawn_prior, drawn_loop),
        ]
        uwb_events = [event for event in events if event.source == "uwb"]
        if uwb_events:
            text_lines.append("UWB 单独: %d 次" % len(uwb_events))
        if loop_disabled and not loop_events:
            text_lines.append("回环: 配置日志明确关闭")
        if len(times) > 1:
            text_lines.append("map→odom 首末: %.3f m / %.2f°"
                              % (math.hypot(xs[-1] - xs[0], ys[-1] - ys[0]),
                                 wrap_deg(yaws[-1] - yaws[0])))
        ax_xy.text(0.015, 0.975, "\n".join(text_lines), transform=ax_xy.transAxes,
                   ha="left", va="top", fontsize=8,
                   bbox=dict(boxstyle="round,pad=0.35", fc="white", ec="#999999",
                             alpha=0.88))

        state = recording.latest_state
        state_text = "state=?"
        if state is not None:
            state_text = "%s  epoch=%d" % (state.status, state.epoch)
        scope_text = ("最近 %.0f s" % window) if window > 0.0 else "全程"
        if loop_disabled:
            scope_text += " · loop=off(config)"
        self.fig.suptitle(
            "Z-Localization backend · map→odom 纠正监视（先验配准 vs 回环）   |   %s   |   %s"
            % (state_text, scope_text),
            fontsize=13, y=0.985,
        )
        if caption:
            caption_text = (
                "判读：③ 的柱来自 backend /rosout 的 "
                "\"global correction committed ... delta_t/delta_r_deg\" 与 "
                "\"loop scan-to-map accepted ...\"；④ 的箭头是同一事件在 /tf map→odom "
                "上实测的前后变化，画在机器人当时的位置。\n"
                "箭头长度=这次修正对车实际位置的平移影响；旋转量看 ③ 的 × 和②的台阶。"
                "先验+UWB 联合提交单独标记；其总修正量不能拆成两路独立贡献。"
                "状态带：黄=OBS_DEGRADED，红=SUSPECT/LOST。"
            )
            self.fig.text(0.012, 0.008, caption_text, fontsize=8.2, va="bottom",
                          ha="left", color="#333333")
            self.fig.subplots_adjust(left=0.065, right=0.935, top=0.935, bottom=0.075)
        else:
            self.fig.subplots_adjust(left=0.065, right=0.935, top=0.945, bottom=0.035)

    def save(self, path: Path) -> bool:
        """保存 PNG；失败只报错，不让现场工具在收尾时崩掉。"""
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            self.fig.savefig(str(path), dpi=135, bbox_inches="tight")
            return True
        except OSError as error:
            print("[错误] 图保存失败 %s: %s" % (path, error), file=sys.stderr)
            return False


# ---------------------------------------------------------------------------
# CSV
# ---------------------------------------------------------------------------
def save_csv(recording: Recording, samples_path: Path,
             events_path: Path) -> tuple[bool, bool]:
    """两个 CSV：逐帧样本 / 每次事件。写不进去只警告，不影响出图。

    返回 (samples_ok, events_ok)。`CsvLog.enabled` 必须在其 close() 之前读，
    close() 会把句柄置空，之后所有人都会看起来像失败。
    """
    sample_log = CsvLog(
        samples_path,
        [
            "kind", "t", "stamp_ns", "x", "y", "z", "roll_deg", "pitch_deg", "yaw_deg",
            "status", "epoch",
        ],
    )
    for sample in recording.samples:
        sample_log.write(
            [
                "map_odom", "%.6f" % sample.t, str(sample.stamp_ns),
                "%.9f" % sample.x, "%.9f" % sample.y, "%.9f" % sample.z,
                "%.9f" % sample.roll_deg, "%.9f" % sample.pitch_deg,
                "%.9f" % sample.yaw_deg, "", "",
            ]
        )
    for pose in recording.poses:
        sample_log.write(
            [
                "localization_odometry", "%.6f" % pose.t, str(pose.stamp_ns),
                "%.9f" % pose.x, "%.9f" % pose.y, "%.9f" % pose.z,
                "", "", "%.9f" % pose.yaw_deg, "", "",
            ]
        )
    for state in recording.states:
        sample_log.write(
            [
                "state", "%.6f" % state.t, "", "", "", "", "", "", "",
                state.status, str(state.epoch),
            ]
        )
    samples_ok = sample_log.enabled
    sample_log.close()

    event_log = CsvLog(
        events_path,
        [
            "t", "source", "delta_t_m", "delta_r_deg", "tf_dxy_m", "tf_dyaw_deg",
            "tf_step_t", "before_x", "before_y", "after_x", "after_y", "detail",
        ],
    )
    for event in recording.events:
        effect = event.effect
        event_log.write(
            [
                "%.6f" % event.t,
                event.source,
                "%.9f" % event.delta_t,
                "%.9f" % event.delta_r_deg,
                "" if effect is None else "%.9f" % effect.dxy,
                "" if effect is None else "%.9f" % effect.dyaw_deg,
                "" if effect is None else "%.6f" % effect.step_t,
                "" if effect is None else "%.9f" % effect.before_xy[0],
                "" if effect is None else "%.9f" % effect.before_xy[1],
                "" if effect is None else "%.9f" % effect.after_xy[0],
                "" if effect is None else "%.9f" % effect.after_xy[1],
                event.detail.replace(",", " "),
            ]
        )
    events_ok = event_log.enabled
    event_log.close()
    return samples_ok, events_ok


# ---------------------------------------------------------------------------
# Demo：不连 ROS，验证画图/统计链路
# ---------------------------------------------------------------------------
def make_demo(args) -> Recording:
    """合成一段 map→odom：2 Hz 小先验修正 + 两次明显回环。"""
    import random

    duration = args.duration if args.duration > 0 else 20.0
    recording = Recording(args)
    random.seed(7)

    # 初始 map→odom 带一点平移/航向，模拟"地图原点与 odom 原点本来就不重合"。
    T_map_odom = np.eye(4)
    T_map_odom[:3, 3] = (0.12, -0.08, 0.01)
    yaw = 2.5 * DEG2RAD
    T_map_odom[:3, :3] = np.array(
        [[math.cos(yaw), -math.sin(yaw), 0.0],
         [math.sin(yaw), math.cos(yaw), 0.0],
         [0.0, 0.0, 1.0]]
    )

    step = 0.05
    total_steps = int(round(duration / step))
    next_prior = 0.5
    next_loop = 8.0
    state_period = 0.5
    next_state = 0.0

    def robot_odom_pose(t: float) -> np.ndarray:
        # 机器人在 odom 里走一个 8 字形，保证轨迹不重合、2D 面板好看。
        x = 1.4 * math.sin(0.22 * t)
        y = 0.9 * math.sin(0.44 * t)
        heading = math.atan2(1.4 * 0.22 * math.cos(0.22 * t),
                             0.9 * 0.44 * math.cos(0.44 * t))
        pose = np.eye(4)
        pose[:3, :3] = np.array(
            [[math.cos(heading), -math.sin(heading), 0.0],
             [math.sin(heading), math.cos(heading), 0.0],
             [0.0, 0.0, 1.0]]
        )
        pose[:3, 3] = (x, y, 0.02)
        return pose

    def apply_correction(dx: float, dy: float, dyaw_deg: float) -> tuple[float, float]:
        nonlocal T_map_odom
        before_pose = T_map_odom @ robot_odom_pose(t)
        delta = np.eye(4)
        angle = dyaw_deg * DEG2RAD
        delta[:3, :3] = np.array(
            [[math.cos(angle), -math.sin(angle), 0.0],
             [math.sin(angle), math.cos(angle), 0.0],
             [0.0, 0.0, 1.0]]
        )
        delta[:3, 3] = (dx, dy, 0.0)
        T_map_odom = delta @ T_map_odom
        after_pose = T_map_odom @ robot_odom_pose(t)
        dxy = math.hypot(after_pose[0, 3] - before_pose[0, 3],
                         after_pose[1, 3] - before_pose[1, 3])
        dyaw = wrap_deg(matrix_yaw_deg(after_pose) - matrix_yaw_deg(before_pose))
        return dxy, dyaw

    for index in range(total_steps):
        t = index * step
        while t >= next_prior:
            dxy, dyaw = apply_correction(random.gauss(0.0, 0.004),
                                         random.gauss(0.0, 0.004),
                                         random.gauss(0.0, 0.15))
            event = CorrectionEvent(
                t=t,
                source="prior",
                delta_t=dxy,
                delta_r_deg=dyaw,
                detail="runtime=%d sources=prior_scan_to_map" % index,
                raw="demo prior",
            )
            recording.events.append(event)
            next_prior += 0.5
        while t >= next_loop:
            dxy, dyaw = apply_correction(0.18 if next_loop < 12.0 else -0.12,
                                         0.05, 1.8 if next_loop < 12.0 else -1.2)
            event = CorrectionEvent(
                t=t,
                source="loop",
                delta_t=dxy,
                delta_r_deg=dyaw,
                detail="latest=%d target=%d" % (index, max(0, index - 80)),
                raw="demo loop",
            )
            recording.events.append(event)
            next_loop += 8.0
        while t >= next_state:
            recording.add_state(t, StateSample(t=t, status="NORMAL", epoch=0,
                                               reason="demo", text="status=NORMAL"))
            next_state += state_period

        T_sample = T_map_odom.copy()
        recording.add_map_odom(t, int(t * 1e9), T_sample)
        recording.add_pose(t, int(t * 1e9), T_map_odom @ robot_odom_pose(t))

    return recording


# ---------------------------------------------------------------------------
# Live ROS 录制
# ---------------------------------------------------------------------------
_ROS_FLAGS_WITH_VALUE = ("-r", "--remap", "-p", "--param", "-e", "--enclave",
                         "--log-level", "--log-config-file", "--params-file")
_ROS_FLAGS_BARE = ("--ros-args", "--", "-a")


def filter_ros_args(argv: list[str]) -> tuple[list[str], list[str]]:
    """把 ROS 专有参数剔出来，自定义参数照常解析。"""
    custom, ros = [], []
    index = 0
    while index < len(argv):
        token = argv[index]
        if token in _ROS_FLAGS_WITH_VALUE and index + 1 < len(argv):
            ros.extend([token, argv[index + 1]])
            index += 2
            continue
        if token in _ROS_FLAGS_BARE or token.startswith("__"):
            ros.append(token)
            index += 1
            continue
        custom.append(token)
        index += 1
    return custom, ros


class Recorder(Recording):
    """真正的 ROS 节点：订阅 TF / 里程计 / 状态 / rosout。"""

    def __init__(self, args, ros_args: Optional[list[str]] = None):
        super().__init__(args)
        self.args = args
        self.node = None
        self._rclpy = None
        self._setup_ros(ros_args or [])

    @staticmethod
    def _strip_namespace_args(ros_args: list[str]) -> list[str]:
        """把 __ns:=... 从 rclpy.init 的参数里去掉。

        命名空间已经由 --namespace / __ns 解析到 self.args.namespace，
        状态话题也按它显式拼好了；再让 rclpy 解析一遍裸 `__ns:=` 反而可能
        在部分 Humble 版本上报 "unrecognized arguments"。
        """
        result = []
        index = 0
        while index < len(ros_args):
            token = ros_args[index]
            if token in ("-r", "--remap") and index + 1 < len(ros_args):
                if ros_args[index + 1].startswith("__ns:="):
                    index += 2
                    continue
                result.extend([token, ros_args[index + 1]])
                index += 2
                continue
            if token.startswith("__ns:="):
                index += 1
                continue
            result.append(token)
            index += 1
        return result

    def _setup_ros(self, ros_args: list[str]) -> None:
        import rclpy
        from rcl_interfaces.msg import Log
        from nav_msgs.msg import Odometry
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String
        from tf2_msgs.msg import TFMessage

        self._rclpy = rclpy
        # 不能把 None 传给 rclpy.init：它会去解析 sys.argv 里的 --duration 等
        # 自定义参数，直接报 unrecognized arguments。ROS 参数已经单独筛出来了。
        rclpy.init(args=self._strip_namespace_args(ros_args))
        namespace = (self.args.namespace or "").strip("/")
        self.node = rclpy.create_node("backend_correction_monitor",
                                      namespace=namespace if namespace else "")

        tf_qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT)
        odom_qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT)
        rosout_qos = QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE)
        # 状态话题用 volatile：backend 的发布者是 transient_local（更强），
        # volatile 订阅者能连上；反过来若发布者是 volatile（测试/旧版本），
        # transient_local 订阅者会被 QoS 判为不兼容，一条都收不到。
        state_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)

        self.node.create_subscription(TFMessage, self.args.tf_topic, self._on_tf, tf_qos)
        self.node.create_subscription(Odometry, self.args.odom_topic, self._on_odometry,
                                      odom_qos)
        if self.args.rosout_topic:
            self.node.create_subscription(Log, self.args.rosout_topic, self._on_rosout,
                                          rosout_qos)
        state_topic = self._resolve_state_topic()
        if state_topic:
            self.node.create_subscription(String, state_topic, self._on_state, state_qos)

        self.node.get_logger().info(
            "map→odom 纠正监视 | TF=%s odom=%s state=%s rosout=%s"
            % (self.args.tf_topic, self.args.odom_topic, state_topic or "(关)",
               self.args.rosout_topic or "(关)")
        )

    def _resolve_state_topic(self) -> str:
        topic = (self.args.state_topic or "").strip()
        if not topic:
            return ""
        if topic.startswith("/"):
            return topic
        namespace = (self.args.namespace or "").strip("/")
        if namespace:
            return "/%s/%s" % (namespace, topic.strip("/"))

        # 没给 namespace 时，短等一会儿自动找 <ns>/backend_node/localization_state。
        leaf = "/" + topic.strip("/")
        deadline = time.monotonic() + max(0.5, self.args.discover_timeout)
        while time.monotonic() < deadline:
            names = [name for name in dict(self.node.get_topic_names_and_types())
                     if name.endswith(leaf)]
            if names:
                if len(names) > 1:
                    self.node.get_logger().warn(
                        "发现多个 %s: %s，用第一个；可用 --namespace 指定"
                        % (topic, sorted(names)))
                return names[0]
            self._rclpy.spin_once(self.node, timeout_sec=0.1)
        return "/" + topic.strip("/")

    # ---- 回调 -------------------------------------------------------------
    def _on_tf(self, message) -> None:
        for transform in message.transforms:
            if (transform.header.frame_id.strip("/") != self.args.map_frame
                    or transform.child_frame_id.strip("/") != self.args.odom_frame):
                continue
            stamp_ns = (int(transform.header.stamp.sec) * 1_000_000_000
                        + int(transform.header.stamp.nanosec))
            # 同一帧被重复广播时按 stamp 去重，避免样本列表里出现零间隔台阶。
            if self.samples and stamp_ns and stamp_ns == self.samples[-1].stamp_ns:
                return
            quat = transform.transform.rotation
            translation = transform.transform.translation
            matrix = pose_matrix(translation.x, translation.y, translation.z,
                                 quat.x, quat.y, quat.z, quat.w)
            self.add_map_odom(self.now_rel(), stamp_ns, matrix)
            self.counts["tf"] += 1
            return

    def _on_odometry(self, message) -> None:
        pointer = message.pose.pose
        stamp_ns = (int(message.header.stamp.sec) * 1_000_000_000
                    + int(message.header.stamp.nanosec))
        matrix = pose_matrix(pointer.position.x, pointer.position.y, pointer.position.z,
                             pointer.orientation.x, pointer.orientation.y,
                             pointer.orientation.z, pointer.orientation.w)
        self.add_pose(self.now_rel(), stamp_ns, matrix)
        self.counts["odom"] += 1

    def _on_state(self, message) -> None:
        state = parse_state_text(message.data)
        if state is None:
            return
        self.add_state(self.now_rel(), state)

    def _on_rosout(self, message) -> None:
        if self.args.node_match and self.args.node_match not in message.name:
            return
        if self.observe_backend_configuration(message.msg):
            if self.loop_override:
                print("[信息] backend 已关闭回环: %s" % self.loop_override)
            return
        event = parse_backend_log(message.msg)
        if event is None:
            return
        stamp_ns = (int(message.stamp.sec) * 1_000_000_000
                    + int(message.stamp.nanosec))
        key = (stamp_ns, message.msg[:120])
        if key in self.seen_logs:
            return
        self.seen_logs.append(key)
        event.stamp_ns = stamp_ns
        self.add_event(event, verbose=self.args.verbose,
                       print_delta=self.args.print_delta)

    # ---- 生命周期 ---------------------------------------------------------
    def close(self) -> None:
        if self._rclpy is None:
            return
        if self.node is not None:
            try:
                self.node.destroy_node()
            except Exception:  # noqa: BLE001 - 退出路径不让清理异常盖住统计
                pass
            self.node = None
        try:
            if self._rclpy.ok():
                self._rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass


def run_live(args, ros_args: list[str]) -> tuple[Recording, Optional[ReportFigure]]:
    """自旋并（可选）实时刷新窗口；Ctrl-C / --duration 结束后返回录制数据。"""
    import matplotlib.pyplot as plt

    recording = Recorder(args, ros_args)
    report = None
    headless = args.headless or args.stats_only or not os.environ.get("DISPLAY")
    if not headless:
        plt.ion()
        report = ReportFigure(args)

    start_wall = time.monotonic()
    last_draw = 0.0
    received_first = False
    try:
        while True:
            try:
                import rclpy

                if not rclpy.ok():
                    break
                rclpy.spin_once(recording.node, timeout_sec=0.02)
            except Exception as error:  # noqa: BLE001
                print("[错误] ROS 自旋失败: %s" % error, file=sys.stderr)
                break

            if recording.samples and not received_first:
                received_first = True
                print("[OK] 收到 map→odom TF，开始记录（Ctrl-C 结束）")

            elapsed = time.monotonic() - start_wall
            if args.duration > 0 and elapsed >= args.duration:
                break
            recording.print_status()

            if report is not None and time.monotonic() - last_draw >= 1.0 / args.plot_hz:
                analyze_effects(recording)
                report.update(recording, caption=not args.no_caption,
                              window=args.window)
                report.fig.canvas.draw()
                report.fig.canvas.flush_events()
                last_draw = time.monotonic()
    except KeyboardInterrupt:
        print("\n[中断] 正在收尾并保存已录数据 ...")
    finally:
        recording.close()
    return recording, report


# ---------------------------------------------------------------------------
# 命令行
# ---------------------------------------------------------------------------
def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Z-Localization backend 的 map→odom 纠正监视（先验 vs 回环）",
        add_help=False,
    )
    parser.add_argument("-h", "--help", action="store_true")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="录制时长（秒）；0 = 一直录到 Ctrl-C（默认）")
    parser.add_argument("--plot-hz", type=float, default=5.0,
                        help="实时窗口刷新频率（Hz），默认 5")
    parser.add_argument("--window", type=float, default=120.0,
                        help="实时窗口只显示最近多少秒，默认 120；0 = 全程。"
                             "最终保存的 PNG 始终是全程")
    parser.add_argument("--tf-topic", default="/tf", help="TF 话题（默认 /tf）")
    parser.add_argument("--odom-topic", default="/localization/odometry",
                        help="backend 的 map 系里程计（默认 /localization/odometry）")
    parser.add_argument("--odom-hz", type=float, default=20.0,
                        help="里程计降采样频率（Hz），默认 20；0 = 全存")
    parser.add_argument("--state-topic", default="backend_node/localization_state",
                        help="定位状态话题；相对名会按 --namespace / __ns 展开。"
                             "传空串关闭")
    parser.add_argument("--rosout-topic", default="/rosout",
                        help="修正日志来源（默认 /rosout）；传空串则只看 TF、不解析事件")
    parser.add_argument("--map-frame", default="map", help="地图坐标系名")
    parser.add_argument("--odom-frame", default="odom", help="里程计坐标系名")
    parser.add_argument("--node-match", default="backend",
                        help="只接受名字里包含该子串的 /rosout 日志（默认 backend，同时匹配 backend 与 backend_node）")
    parser.add_argument("--namespace", default="",
                        help="命名空间（如 red_standard_robot1）；也可用 --ros-args -r __ns:=...")
    parser.add_argument("--discover-timeout", type=float, default=3.0,
                        help="没给命名空间时，自动找状态话题最多等多久（秒）")
    parser.add_argument("--match-before", type=float, default=0.25,
                        help="事件匹配 TF 台阶时向前看的窗口（秒），默认 0.25")
    parser.add_argument("--match-after", type=float, default=0.35,
                        help="事件匹配 TF 台阶时向后看的窗口（秒），默认 0.35")
    parser.add_argument("--arrow-min", type=float, default=0.01,
                        help="④ 里先验修正箭头的最小长度（米），默认 0.01；回环始终画")
    parser.add_argument("--xy-min-span", type=float, default=2.0,
                        help="④ 的最小视野边长（米），默认 2；静止抖动不会把图放大成毛线球")
    parser.add_argument("--unassigned-translation", type=float, default=0.02,
                        help="统计未归属跳变的平移阈值（米），默认 0.02")
    parser.add_argument("--unassigned-yaw", type=float, default=0.5,
                        help="统计未归属跳变的旋转阈值（度），默认 0.5")
    parser.add_argument("--print-delta", type=float, default=0.02,
                        help="终端实时打印先验事件的最小平移（米），默认 0.02；回环始终打印")
    parser.add_argument("--verbose", action="store_true", help="实时打印每一条先验事件")
    parser.add_argument("--demo", action="store_true", help="用合成数据出图（不需要 ROS）")
    parser.add_argument("--headless", action="store_true", help="不弹窗，只存文件")
    parser.add_argument("--no-caption", action="store_true",
                        help="不把解释画进图里（解释仍打印到终端）")
    parser.add_argument("--stats-only", action="store_true", help="只打印统计，不出图")
    parser.add_argument("--save", default="",
                        help="PNG 路径；不写则自动放到 <工程根>/user_log/。"
                             "只给文件名也进 user_log/；目录不可写时回退到 user_log/")
    parser.add_argument("--csv", default="",
                        help="CSV 基础路径；实际写 <base>_samples.csv 与 <base>_events.csv。不写则与 PNG 同名同目录")
    return parser


def main(argv: Optional[list[str]] = None) -> int:
    if argv is None:
        argv = sys.argv[1:]
    custom_args, ros_args = filter_ros_args(argv)
    parser = build_parser()
    args = parser.parse_args(custom_args)
    if args.help:
        print_header_help(__file__)
        return 0

    # 兼容 ros2 run 风格：__ns:=/xxx 既可以是独立参数，也可以跟 -r/--remap。
    for index, token in enumerate(ros_args):
        if token.startswith("__ns:="):
            args.namespace = token.split(":=", 1)[1]
        elif token in ("-r", "--remap") and index + 1 < len(ros_args):
            next_token = ros_args[index + 1]
            if next_token.startswith("__ns:="):
                args.namespace = next_token.split(":=", 1)[1]

    prepare_matplotlib_env()
    headless = args.headless or args.stats_only or not os.environ.get("DISPLAY")
    import matplotlib
    if headless:
        matplotlib.use("Agg")

    cjk_font = setup_cjk_font()
    if not cjk_font:
        print("[提示] 没找到中文字体，图上中文可能显示为方块；数字/曲线不受影响。",
              file=sys.stderr)

    if args.demo:
        print("使用合成数据（--demo）：约 2 Hz 先验小修正 + 两次回环大修正")
        recording = make_demo(args)
        report = None
        if not args.stats_only:
            report = ReportFigure(args)
    else:
        if args.duration <= 0:
            print("开始录制（没有 --duration，按 Ctrl-C 结束并出图）")
        else:
            print("开始录制 %.1f s ..." % args.duration)
        try:
            recording, report = run_live(args, ros_args)
        except Exception as error:  # noqa: BLE001 - 现场工具优先给出可读的下一步
            print("[错误] ROS 录制启动失败: %s" % error, file=sys.stderr)
            print("       先 source /opt/ros/humble/setup.bash 和 install/setup.bash，"
                  "再确认 backend 在跑。", file=sys.stderr)
            return 2

    if not recording.samples:
        print("[错误] 没有 map→odom 样本。确认 backend 正在跑，并且：", file=sys.stderr)
        print("       ros2 run tf2_ros tf2_echo map odom", file=sys.stderr)
        print("       若 frame 名不是 map/odom，用 --map-frame / --odom-frame 指定",
              file=sys.stderr)
        return 2

    analyze_effects(recording)

    # 输出路径：默认全部进 <工程根>/user_log/；显式 --save/--csv 尊重用户，
    # 但目录不可写时回退到 user_log/，不再抛 PermissionError。
    default_name = ("backend_correction_%s.png" % time.strftime("%Y%m%d_%H%M"))
    try:
        png_path = resolve_png_path(args.save, default_name)
        base = resolve_csv_base(args.csv, png_path)
    except OSError as error:
        print("[错误] 无法准备输出目录: %s" % error, file=sys.stderr)
        return 2
    samples_csv = base.with_name(base.name + "_samples.csv")
    events_csv = base.with_name(base.name + "_events.csv")

    samples_ok, events_ok = save_csv(recording, samples_csv, events_csv)
    print_summary(recording)
    print("")
    print("保存位置：")
    if samples_ok:
        print("  数据: %s" % samples_csv)
    else:
        print("  数据: 未写出（权限/磁盘），目标 %s" % samples_csv)
    if events_ok:
        print("  事件: %s" % events_csv)
    else:
        print("  事件: 未写出（权限/磁盘），目标 %s" % events_csv)
    if not args.stats_only:
        if report is None:
            report = ReportFigure(args)
        report.update(recording, caption=not args.no_caption, window=0.0)
        if report.save(png_path):
            print("  图  : %s" % png_path)
        else:
            print("  图  : 未写出（权限/磁盘），目标 %s" % png_path)
    if samples_ok:
        print("")
        print("按来源分解（先验/UWB/回环/恢复/总效果）：")
        print("  ./debug_tools/backend_contribution_report.py --samples %s"
              % samples_csv)
    return 0


if __name__ == "__main__":
    sys.exit(main())
