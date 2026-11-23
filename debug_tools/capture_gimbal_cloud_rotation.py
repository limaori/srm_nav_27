#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""抓「底盘不动、只转云台」这一段，量出**点云在世界系里到底跟着转不转**，并打包给他人分析。

现象与要回答的问题
==================
云台自转时底盘定位看着是准的，但 RViz 里整片点云跟着云台来回摆。责任可能在两层：

  A. **点云内容真的在 odom 里转**：上游把云台角带进了世界坐标（漏补、补反、补两次、
     或者点云其实在车体系却标成了 odom）。
  B. **点云没转，转的是显示**：RViz 的 Fixed Frame 不是 map、TF 里同一 child 有两个
     parent、静态外参随云台动、或者你看的根本是另一路点云。

只盯 RViz 判断不了。本工具把「云台转了多少」和「点云在 odom 里转了多少」同时录下来，
用**相邻两帧的二维配准**直接量出点云的世界系转角，再对云台角做增量回归：

    斜率 ≈  0  -> 点云在世界系里是稳的（云台没进点云）
    斜率 ≈ +1  -> 云台角被原样加进了世界点云（同向）
    斜率 ≈ -1  -> 应该减掉的云台角没减掉 / 符号反了（反向）
    斜率 ≈ ±2  -> 同一个云台角被算了两次（双计）

斜率落在哪一档，故障层就基本定了。回归用「过原点的增量回归」，和
`capture_gimbal_spin.py` 的判据同口径，所以「转过去再转回来」也能判。
逐帧增量只受相邻两帧影响，不靠首末两点；另外再做一路「每帧直接对齐锚点帧」的
漂移无关校验，两路斜率应当一致。

为什么把 TF 也一起录
====================
点云 header 里的 frame_id 可能是 odom、gimbal_yaw、livox_frame…… **同一个话题在不同层
看到的转角不一样**。工具按 `header.stamp` 查 TF（不是拿最新值硬套），把每帧变换到
分析系（默认 odom，可 `--analysis-frame map`），同时记录整条链的 yaw 分解：

    map -> odom -> gimbal_yaw -> livox_frame
                     └-> base_link

于是报告里能直接看出「云台角进了哪一段」：是 TF 把它转回来的（正常），还是点云内容
自己在转（上游问题）。还会核对 `odom -> lidar` 与 `/lidar_odometry` 是否同源——两者
不一致时，点云和位姿就不是同一时刻的同一件事。

录什么
======
  * 多路点云（自动识别，默认最多 6 路）：registered_scan / sensor_scan / terrain_map /
    terrain_map_ext / cloud_registered / livox 原始点云
  * 云台角三路：/joint_states 的 gimbal_yaw_joint、/chassis_to_gimbal_yaw_deg、
    以及 TF 算出的 yaw(odom->gimbal_yaw) − yaw(odom->base_link)（判据用最后这路）
  * 位姿：/odometry（导航用，child=gimbal_yaw）、/lidar_odometry（雷达位姿）
  * TF：map->odom、odom->base_link、base_link->gimbal_yaw、gimbal_yaw->livox_*
  * TF 父子关系审计（同一 child 被多个 parent 发布时报警）
  * 相关节点的参数（尽力而为，只读）

输出是一个**自解释的数据包**，可以整个拷给别人：

    user_test_log/gimbal_cloud_<时间戳>/
      manifest.json          环境/话题/帧/参数/git 版本/TF 快照
      samples.csv            20 Hz 标量时序（云台角、TF 各段 yaw、底盘速度…）
      raw/<话题>.npz         每帧点云 + 逐帧 TF（分析系、雷达位姿、map->odom）
      clouds/*.pcd           关键帧（已变换到分析系，CloudCompare 直接看）
      clouds/index.csv       每个 PCD 是第几帧、云台多少度
      analysis/pairs.csv     每对相邻帧的配准结果（斜率/滞后/内点率都在这里复算）
      analysis/plots.png     六面板总图
      analysis/cloud_overlay.png  云台两端角度的点云叠加（墙是不是分成了两层）
      report.md             结论 + 证据 + 按斜率给出的排查方向
      bag/                   --bag 时额外录的 rosbag（默认不录，点云很占地方）

用法
====
先起导航（`./scripts/bringup/start_real_nav.sh ...`），等话题稳定，再开本工具：

    # 一直录到 Ctrl-C（推荐：先开始录，再慢慢转云台 ±30~45°，最后转回中间）
    ./debug_tools/capture_gimbal_cloud_rotation.py

    # 固定 60 s
    ./debug_tools/capture_gimbal_cloud_rotation.py --duration 60

    # 只录不分析（数据包回头再算）
    ./debug_tools/capture_gimbal_cloud_rotation.py --duration 60 --no-analyze

    # 对已有数据包重算（不需要 ROS）
    ./debug_tools/capture_gimbal_cloud_rotation.py --analyze user_test_log/gimbal_cloud_20261124_001234

    # 自检：不连 ROS，造 4 路点云（正确 / 车体系 / 漏补 / 双计）跑通全流程
    ./debug_tools/capture_gimbal_cloud_rotation.py --demo

  · 工具**只读**：不打开串口、不发控制、不调服务、不写任何 /cmd_vel_* 或 TF。
  · 云台请用手慢慢转：一次 30~45°，转过去再转回来；转太快时相邻帧间隔角变大，
    配准的搜索范围会不够（可用 --yaw-range 放宽）。
  · 底盘必须基本静止：工具会检查，底盘在跑时报告会标出来，斜率读数作废。
  · 有命名空间时按 ROS 习惯加 `--ros-args -r __ns:=/red_standard_robot1`。

退出码
======
  0 = 采到数据且给出判定
  1 = 数据不足（云台没转够/没点云/底盘在动）
  2 = 用法或环境错误（没 source ROS、没有 scipy 等）
  3 = 分析失败
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import platform
import signal
import subprocess
import sys
import tarfile
import tempfile
import textwrap
import time
import traceback
from dataclasses import dataclass, field, replace
from datetime import datetime
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent
DEBUG_DIR = Path(__file__).resolve().parent
DEFAULT_OUT_DIR = REPO_ROOT / "user_test_log"

DEFAULT_MAP_FRAME = "map"
DEFAULT_ODOM_FRAME = "odom"
DEFAULT_BASE_FRAME = "base_link"
DEFAULT_GIMBAL_FRAME = "gimbal_yaw"

#: 自动识别点云话题时按这个顺序优先；比较的是去掉命名空间后的后缀。
CLOUD_CANDIDATES = (
    "registered_scan",
    "sensor_scan",
    "terrain_map",
    "terrain_map_ext",
    "cloud_registered",
    "cloud_registered_body",
    "cloud_map",
    "livox/lidar/pointcloud",
    "velodyne_points",
)

SAMPLE_FIELDS = (
    "t_stamp", "t_wall", "joint_deg", "c2g_deg", "gimbal_phys_deg",
    "yaw_map_odom_deg", "yaw_odom_base_deg", "yaw_base_gimbal_deg",
    "yaw_gimbal_lidar_deg", "yaw_odom_lidar_tf_deg", "yaw_odom_lidar_odom_deg",
    "yaw_odom_nav_deg", "base_x", "base_y", "base_vx", "base_vy", "base_wz",
    "tf_ok",
)

PAIR_FIELDS = (
    "topic", "prev_idx", "idx", "prev_stamp", "stamp", "dt_s",
    "dtheta_gimbal_deg", "dtheta_base_deg", "dyaw_map_odom_deg",
    "psi_pair_align_deg", "dpsi_cloud_deg", "shift_m", "inlier", "rmse_m",
    "psi_anchor_align_deg", "dpsi_cloud_anchor_deg", "anchor_inlier", "anchor_shift_m",
)

VERDICT_TEXT = {
    "stable": "点云在分析系里稳定（云台角没有进点云）",
    "follow_pos": "云台角被原样加进点云（同向，斜率 ≈ +1）",
    "follow_neg": "云台角没被正确减掉（反向跟随，斜率 ≈ -1）",
    "follow_2x": "云台角被算了两次（斜率 ≈ ±2）",
    "partial": "部分跟随（斜率 0.2~0.7，像是滞后或只有部分帧受影响）",
    "translation_orbit": "yaw 稳定，但世界点云存在平移绕圈（障碍物会跟着绕）",
    "other": "斜率异常，先看 pairs.csv 里的散点",
    "unclear": "相关性偏低，斜率不可信（先看内点率与点位分布）",
    "insufficient": "云台行程或有效帧不足，无法判定",
}


# =============================================================
# 小工具：环境、数学、文件
# =============================================================
def ensure_ros_log_dir():
    """~/.ros/log 不可写时把 ROS 日志挪到临时目录。

    本机实测 `~/.ros` 在只读家目录下，`ros2` 命令和 rclpy 都会因为写不了日志直接崩
    （`Failed opening file ... Read-only file system`）。必须在 import rclpy 之前设置。
    """
    if os.environ.get("ROS_LOG_DIR"):
        return
    candidate = Path.home() / ".ros" / "log"
    try:
        candidate.mkdir(parents=True, exist_ok=True)
        if os.access(candidate, os.W_OK):
            return
    except OSError:
        pass
    fallback = Path(tempfile.gettempdir()) / f"roslog-{os.getuid()}"
    try:
        fallback.mkdir(parents=True, exist_ok=True)
        os.environ["ROS_LOG_DIR"] = str(fallback)
    except OSError:
        pass


def wrap_deg(value):
    """折到 (-180, 180]。"""
    return (float(value) + 180.0) % 360.0 - 180.0


def wrap_deg_array(values):
    values = np.asarray(values, dtype=np.float64)
    with np.errstate(invalid="ignore"):
        return np.mod(values + 180.0, 360.0) - 180.0


def quat_to_matrix(x, y, z, w):
    norm = x * x + y * y + z * z + w * w
    if norm < 1e-12:
        return np.eye(3)
    s = 2.0 / norm
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    return np.array([
        [1.0 - (yy + zz), xy - wz, xz + wy],
        [xy + wz, 1.0 - (xx + zz), yz - wx],
        [xz - wy, yz + wx, 1.0 - (xx + yy)],
    ])


def rigid_matrix(translation, quat_xyzw):
    matrix = np.eye(4)
    matrix[:3, :3] = quat_to_matrix(*quat_xyzw)
    matrix[:3, 3] = np.asarray(translation, dtype=np.float64)
    return matrix


def yaw_deg_of(matrix):
    return math.degrees(math.atan2(float(matrix[1, 0]), float(matrix[0, 0])))


def transform_points(points, matrix):
    if len(points) == 0:
        return points
    return points @ matrix[:3, :3].T + matrix[:3, 3]


def rotate_xy(points_xy, angle, center_xy=None):
    cos_a, sin_a = math.cos(angle), math.sin(angle)
    if center_xy is None:
        center_xy = np.zeros(2)
    delta = points_xy - center_xy
    rotated = np.empty_like(delta)
    rotated[:, 0] = cos_a * delta[:, 0] - sin_a * delta[:, 1]
    rotated[:, 1] = sin_a * delta[:, 0] + cos_a * delta[:, 1]
    return rotated + center_xy


def linear_fit(x, y):
    """过原点的增量回归 + Pearson 相关；返回 (slope, corr, n)。"""
    x = np.asarray(x, dtype=np.float64)
    y = np.asarray(y, dtype=np.float64)
    if len(x) < 3:
        return float("nan"), float("nan"), int(len(x))
    denom = float(np.dot(x, x))
    if denom < 1e-12:
        return float("nan"), float("nan"), int(len(x))
    slope = float(np.dot(x, y) / denom)
    x_centered = x - x.mean()
    y_centered = y - y.mean()
    denom_corr = math.sqrt(float(np.dot(x_centered, x_centered)) *
                           float(np.dot(y_centered, y_centered)))
    corr = float(np.dot(x_centered, y_centered) / denom_corr) if denom_corr > 1e-12 else 0.0
    return slope, corr, int(len(x))


def robust_fit(x, y):
    """先拟合、再剔 3σ 外的点，最多两轮；返回 slope, corr, n, mask。"""
    x = np.asarray(x, dtype=np.float64)
    y = np.asarray(y, dtype=np.float64)
    mask = np.ones(len(x), dtype=bool)
    slope, corr, n = linear_fit(x, y)
    for _ in range(2):
        if n < 5 or not np.isfinite(slope):
            break
        residual = np.abs(y - slope * x)
        threshold = max(3.0 * float(np.std(residual[mask])), 0.5)
        updated = mask & (residual <= threshold)
        if updated.sum() == mask.sum() or updated.sum() < 5:
            break
        mask = updated
        slope, corr, n = linear_fit(x[mask], y[mask])
    return slope, corr, n, mask


def downsample_points(points, voxel, max_points, seed=0):
    if len(points) == 0:
        return points
    if voxel and voxel > 0.0:
        keys = np.floor(points / voxel).astype(np.int64)
        _, first = np.unique(keys, axis=0, return_index=True)
        points = points[np.sort(first)]
    if max_points and len(points) > max_points:
        rng = np.random.default_rng(seed)
        picked = rng.choice(len(points), size=max_points, replace=False)
        points = points[np.sort(picked)]
    return points


def write_pcd(path, points, intensity=None):
    """二进制 PCD（CloudCompare / pcl_viewer 都能直接读）。"""
    n = len(points)
    points = np.asarray(points, dtype="<f4")
    if intensity is not None and len(intensity) == n:
        fields, sizes, types, counts = "x y z intensity", "4 4 4 4", "F F F F", "1 1 1 1"
        payload = np.empty((n, 4), dtype="<f4")
        payload[:, :3] = points
        payload[:, 3] = np.asarray(intensity, dtype="<f4")
    else:
        fields, sizes, types, counts = "x y z", "4 4 4", "F F F", "1 1 1"
        payload = points
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        f"FIELDS {fields}\n"
        f"SIZE {sizes}\n"
        f"TYPE {types}\n"
        f"COUNT {counts}\n"
        f"WIDTH {n}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n}\n"
        "DATA binary\n"
    )
    with open(path, "wb") as handle:
        handle.write(header.encode("ascii"))
        handle.write(payload.tobytes())


def safe_topic_name(topic):
    return topic.strip("/").replace("/", "__") or "root"


def human_duration(seconds):
    seconds = float(seconds)
    if seconds < 120:
        return f"{seconds:.1f} s"
    return f"{seconds / 60.0:.1f} min"


def describe_exception(exc):
    text = "".join(traceback.format_exception_only(type(exc), exc)).strip()
    return text.splitlines()[-1] if text else type(exc).__name__


def find_free_dir(base):
    candidate = Path(base)
    if not candidate.exists():
        return candidate
    index = 2
    while True:
        alternative = candidate.with_name(f"{candidate.name}_{index}")
        if not alternative.exists():
            return alternative
        index += 1


def git_info():
    info = {"commit": "", "branch": "", "dirty": None}
    try:
        info["commit"] = subprocess.run(
            ["git", "-C", str(REPO_ROOT), "rev-parse", "HEAD"],
            capture_output=True, text=True, timeout=3).stdout.strip()
        info["branch"] = subprocess.run(
            ["git", "-C", str(REPO_ROOT), "rev-parse", "--abbrev-ref", "HEAD"],
            capture_output=True, text=True, timeout=3).stdout.strip()
        status = subprocess.run(
            ["git", "-C", str(REPO_ROOT), "status", "--porcelain"],
            capture_output=True, text=True, timeout=5).stdout
        info["dirty"] = bool(status.strip())
    except Exception:  # noqa: BLE001 - 版本信息只是附注，拿不到不影响采集
        pass
    return info


def ros_environment():
    env = {}
    for key in ("ROS_DISTRO", "ROS_VERSION", "RMW_IMPLEMENTATION", "ROS_DOMAIN_ID",
                "ROS_LOCALHOST_ONLY"):
        if os.environ.get(key):
            env[key] = os.environ[key]
    try:
        import rclpy
        env["rclpy"] = getattr(rclpy, "__file__", "")
    except Exception:  # noqa: BLE001
        env["rclpy"] = ""
    return env


def dump_parameters(settings, node_names):
    """尽力而为地 dump 相关节点参数（只读服务，失败就跳过）。"""
    if not settings.param_nodes.strip():
        return {}
    wanted = [item.strip() for item in settings.param_nodes.split(",") if item.strip()]
    result = {}
    for target in wanted:
        matches = [name for name in node_names
                   if name.rstrip("/").endswith("/" + target) or name.rstrip("/") == target]
        for node in matches:
            try:
                completed = subprocess.run(
                    ["ros2", "param", "dump", node],
                    capture_output=True, text=True, timeout=4,
                    env=os.environ.copy())
                if completed.returncode == 0 and completed.stdout.strip():
                    result[node] = completed.stdout
            except Exception:  # noqa: BLE001
                continue
    return result


# =============================================================
# 配置
# =============================================================
@dataclass
class Settings:
    # 采集
    duration: float = 0.0
    rate_hz: float = 5.0
    sample_hz: float = 20.0
    wait_s: float = 60.0
    max_points: int = 4000
    voxel: float = 0.10
    max_topics: int = 6
    cloud_topics: tuple = ()
    tf_cache: float = 0.0
    sim_time: str = "auto"
    bag: bool = False
    param_nodes: str = "zlio_bridge,sensor_scan_generation,terrain_analysis,terrain_analysis_ext"
    # 坐标
    map_frame: str = DEFAULT_MAP_FRAME
    odom_frame: str = DEFAULT_ODOM_FRAME
    base_frame: str = DEFAULT_BASE_FRAME
    gimbal_frame: str = DEFAULT_GIMBAL_FRAME
    lidar_frame: str = ""
    analysis_frame: str = DEFAULT_ODOM_FRAME
    # 分析
    min_points: int = 200
    min_range: float = 0.5
    max_range: float = 20.0
    align_points: int = 900
    inlier_dist: float = 0.15
    max_shift: float = 0.6
    yaw_range: float = 60.0
    coarse_step: float = 3.0
    fine_step: float = 0.30
    pair_gap: float = 0.6
    min_travel: float = 10.0
    min_inlier: float = 0.45
    min_dtheta: float = 0.15
    anomaly_deg: float = 1.0
    orbit_min: float = 0.03
    lag_max: float = 0.4
    lag_step: float = 0.05
    anchor_stride: int = 2
    # 产物
    out_dir: Path = DEFAULT_OUT_DIR
    tag: str = "gimbal_cloud"
    note: str = ""
    no_pcd: bool = False
    pcd_max: int = 40
    no_plot: bool = False
    no_analyze: bool = False
    make_tar: bool = False
    analyze_dir: str = ""
    demo: bool = False


# =============================================================
# 数据结构
# =============================================================
@dataclass
class TopicRecording:
    """一路点云的全部逐帧记录（TF 已按 header.stamp 查好）。"""
    topic: str
    stamps: np.ndarray
    frame_ids: list
    points: list
    intensity: list = field(default_factory=list)
    T_odom_frame: np.ndarray = field(default_factory=lambda: np.zeros((0, 4, 4)))
    T_odom_lidar: np.ndarray = field(default_factory=lambda: np.zeros((0, 4, 4)))
    T_map_odom: np.ndarray = field(default_factory=lambda: np.zeros((0, 4, 4)))
    gimbal_deg: np.ndarray = field(default_factory=lambda: np.zeros(0))
    base_deg: np.ndarray = field(default_factory=lambda: np.zeros(0))
    tf_ok: np.ndarray = field(default_factory=lambda: np.zeros(0, dtype=bool))

    def __len__(self):
        return int(len(self.stamps))


@dataclass
class PairRecord:
    prev_idx: int
    idx: int
    prev_stamp: float
    stamp: float
    dt: float
    dtheta_gimbal: float
    dtheta_base: float
    dyaw_map_odom: float
    dpsi_cloud: float
    psi_align: float
    shift: float
    inlier: float
    rmse: float
    anchor_yaw: float = float("nan")
    anchor_inlier: float = float("nan")
    anchor_shift: float = float("nan")
    anchor_shift_x: float = float("nan")
    anchor_shift_y: float = float("nan")


@dataclass
class TopicResult:
    topic: str
    frames: int
    valid: int
    frame_id: str
    travel_deg: float
    gimbal_min: float
    gimbal_max: float
    pairs: list
    n_pairs: int
    slope: float
    corr: float
    slope_anchor: float
    corr_anchor: float
    lag: float
    lag_corr: float
    lag_slope: float
    median_inlier: float
    median_shift: float
    median_points: int
    angle_source: str
    kind: str
    notes: list = field(default_factory=list)
    orbit_radius: float = float("nan")
    orbit_residual: float = float("nan")
    base_orbit_radius: float = float("nan")

    @property
    def verdict(self):
        return VERDICT_TEXT.get(self.kind, self.kind)


@dataclass
class Analysis:
    bundle: Path
    settings: Settings
    manifest: dict
    samples: dict
    recordings: dict
    results: list
    events: list
    anomalies: list = field(default_factory=list)


# =============================================================
# 配准：点云世界系转角
# =============================================================
class Aligner:
    """把当前帧二维配准到参考帧，返回「当前帧被转了多少度才对齐」。

    约定：返回的 psi 是**作用在当前帧上**、使其与参考帧重合的 yaw（弧度）。
    所以点云在世界系里的实际转角是 -psi。

    两段式：先用 0.2 m 占用栅格在 ±yaw_range 内做粗搜（很便宜），再在粗解附近用
    KD 树的截断最近邻代价细化 yaw，并迭代修正平移。稀疏点云下栅格的膨胀保证了粗解
    不会因为采样空隙掉到错误的峰上。
    """

    def __init__(self, settings):
        from scipy.spatial import cKDTree  # noqa: F401 - 延迟导入，--analyze 才需要

        self.settings = settings
        self.reference = None
        self.tree = None
        self.grid = None
        self.grid_origin = None
        self.grid_resolution = 0.20
        self.margin = 2.0
        self._cKDTree = cKDTree

    def set_reference(self, points_xyz, lidar_xy, seed=0):
        from scipy.ndimage import binary_dilation

        reference = self._prepare(points_xyz, lidar_xy, seed)
        self.reference = reference
        if len(reference) < 20:
            self.tree = None
            self.grid = None
            return 0
        self.tree = self._cKDTree(reference)
        low = reference.min(axis=0) - self.margin
        high = reference.max(axis=0) + self.margin
        shape = (max(int((high[1] - low[1]) / self.grid_resolution) + 2, 3),
                 max(int((high[0] - low[0]) / self.grid_resolution) + 2, 3))
        occupied = np.zeros(shape, dtype=bool)
        cells = ((reference - low) / self.grid_resolution).astype(np.int64)
        valid = ((cells[:, 0] >= 0) & (cells[:, 0] < shape[1]) &
                 (cells[:, 1] >= 0) & (cells[:, 1] < shape[0]))
        occupied[cells[valid, 1], cells[valid, 0]] = True
        self.grid = binary_dilation(occupied, iterations=1)
        self.grid_origin = low
        return len(reference)

    def align(self, points_xyz, lidar_xy, seed=0):
        settings = self.settings
        if self.tree is None or self.grid is None:
            return None
        current = self._prepare(points_xyz, lidar_xy, seed + 1)
        if len(current) < 20:
            return None

        best_psi = self._coarse_yaw(current, lidar_xy)
        if best_psi is None:
            return None
        best_psi = self._kd_refine_yaw(current, lidar_xy, best_psi)
        move, best_psi, inlier, rmse = self._refine_translation(
            current, lidar_xy, best_psi)
        return {
            "psi": best_psi,
            "shift": move,
            "shift_norm": float(np.linalg.norm(move)),
            "inlier": inlier,
            "rmse": rmse,
            "n_reference": len(self.reference),
            "n_current": len(current),
        }

    # -- 内部 ---------------------------------------------------------
    def _prepare(self, points_xyz, lidar_xy, seed):
        settings = self.settings
        points = np.asarray(points_xyz, dtype=np.float64)
        if len(points) == 0:
            return np.zeros((0, 2))
        delta = points[:, :2] - np.asarray(lidar_xy, dtype=np.float64)
        ranges = np.linalg.norm(delta, axis=1)
        mask = (ranges >= settings.min_range) & (ranges <= settings.max_range)
        points = points[mask]
        if len(points) > settings.align_points:
            rng = np.random.default_rng(seed)
            picked = rng.choice(len(points), size=settings.align_points, replace=False)
            points = points[np.sort(picked)]
        return points[:, :2].astype(np.float64)

    def _score(self, current, lidar_xy, yaws):
        grid = self.grid
        low = self.grid_origin
        resolution = self.grid_resolution
        scores = np.empty(len(yaws), dtype=np.float64)
        height, width = grid.shape
        for index, psi in enumerate(yaws):
            points = rotate_xy(current, psi, lidar_xy)
            cells = ((points - low) / resolution).astype(np.int64)
            valid = ((cells[:, 0] >= 0) & (cells[:, 0] < width) &
                     (cells[:, 1] >= 0) & (cells[:, 1] < height))
            if int(valid.sum()) < 20:
                scores[index] = -1.0
            else:
                scores[index] = float(grid[cells[valid, 1], cells[valid, 0]].mean())
        return scores

    def _coarse_yaw(self, current, lidar_xy):
        settings = self.settings
        coarse = np.radians(np.arange(-settings.yaw_range, settings.yaw_range + 1e-9,
                                      settings.coarse_step))
        scores = self._score(current, lidar_xy, coarse)
        if not np.isfinite(scores).any():
            return None
        best = float(coarse[int(np.argmax(scores))])
        fine = np.radians(np.arange(math.degrees(best) - settings.coarse_step,
                                    math.degrees(best) + settings.coarse_step + 1e-9,
                                    settings.fine_step))
        fine_scores = self._score(current, lidar_xy, fine)
        if np.isfinite(fine_scores).any():
            best = float(fine[int(np.argmax(fine_scores))])
        return best

    def _kd_cost(self, current, lidar_xy, psi, shift=None):
        points = rotate_xy(current, psi, lidar_xy)
        if shift is not None:
            points = points + shift
        distances, _ = self.tree.query(points, k=1, workers=1)
        return float(np.minimum(distances, 0.6).mean())

    def _kd_refine_yaw(self, current, lidar_xy, best):
        settings = self.settings
        candidates = best + np.radians(np.arange(-0.75, 0.751, 0.15))
        costs = [self._kd_cost(current, lidar_xy, psi) for psi in candidates]
        return float(candidates[int(np.argmin(costs))])

    def _refine_translation(self, current, lidar_xy, psi):
        settings = self.settings
        shift = np.zeros(2, dtype=np.float64)
        for _ in range(2):
            points = rotate_xy(current, psi, lidar_xy) + shift
            distances, indices = self.tree.query(points, k=1, workers=1)
            mask = distances < max(0.35, settings.inlier_dist * 2.5)
            if int(mask.sum()) < 20:
                break
            delta = np.median(self.reference[indices[mask]] - points[mask], axis=0)
            delta = np.clip(delta, -settings.max_shift, settings.max_shift)
            shift = shift + delta
            if float(np.linalg.norm(delta)) < 0.002:
                break
        points = rotate_xy(current, psi, lidar_xy) + shift
        distances, _ = self.tree.query(points, k=1, workers=1)
        mask = distances < settings.inlier_dist
        inlier = float(mask.mean())
        rmse = float(np.sqrt(np.mean(np.square(distances[mask])))) if int(mask.sum()) else float("nan")
        return shift, psi, inlier, rmse


# =============================================================
# 采集
# =============================================================
class RosCapture:
    """rclpy 采集端：只订阅，不发布。"""

    def __init__(self, settings):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
        from rclpy.time import Time
        from rclpy.duration import Duration
        from rclpy.parameter import Parameter
        from tf2_ros import Buffer, TransformListener
        from sensor_msgs.msg import JointState, PointCloud2
        from nav_msgs.msg import Odometry
        from std_msgs.msg import Float32
        from tf2_msgs.msg import TFMessage
        from sensor_msgs_py import point_cloud2

        self._rclpy = rclpy
        self._Time = Time
        self._Duration = Duration
        self._point_cloud2 = point_cloud2

        self.settings = settings
        self.node = Node("gimbal_cloud_capture")
        self.logger = self.node.get_logger()

        if settings.sim_time == "on" or (
                settings.sim_time == "auto" and self._topic_exists("/clock")):
            try:
                self.node.set_parameters(
                    [Parameter("use_sim_time", Parameter.Type.BOOL, True)])
                print("[时间] 检测到 /clock，已切到仿真时间。")
            except Exception as exc:  # noqa: BLE001
                print(f"[警告] 设置 use_sim_time 失败: {describe_exception(exc)}")

        cache = settings.tf_cache if settings.tf_cache > 0 else max(
            90.0, (settings.duration if settings.duration > 0 else 600.0) + 30.0)
        self.tf_buffer = Buffer(cache_time=Duration(seconds=cache))
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        print(f"[TF] 缓存 {cache:.0f} s（超过这个时长的帧会被跳过）。")

        self._detect_frames()
        self.cloud_topics = []
        self._warned_missing_topics = set()
        self._detect_cloud_topics()

        self.raw_topics = {}
        self.cloud_counts = {}
        self.cloud_subs = {}
        self.parse_failures = 0
        self.cloud_received = {}
        self._cloud_last_stamp = {}
        self._cloud_qos = QoSProfile(
            depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST)
        self._PointCloud2 = PointCloud2
        self.subscribe_cloud_topics()

        self.joint_deg = float("nan")
        self.joint_stamp = float("nan")
        self.c2g_deg = float("nan")
        self.odom_msg = None
        self.lidar_odom_msg = None
        self.samples = []
        self.tf_parents = {}
        self.tf_static_pairs = set()
        self.tf_dynamic_pairs = set()
        self.static_map_odom = None
        self.missing_tf = {}
        self.recorded_duration = 0.0

        # 机器人的话题用相对名，这样 __ns 重映射能生效；TF 是全局的，用绝对名。
        self.node.create_subscription(JointState, "joint_states", self._on_joint, 20)
        self.node.create_subscription(
            Float32, "chassis_to_gimbal_yaw_deg", self._on_c2g, 20)
        self.node.create_subscription(Odometry, "odometry", self._on_odom, 20)
        self.node.create_subscription(
            Odometry, "lidar_odometry", self._on_lidar_odom, 20)
        self.node.create_subscription(
            TFMessage, "/tf", lambda msg: self._on_tf(msg, static=False), 200)
        # /tf_static 是 TRANSIENT_LOCAL 的 latched 话题：用 VOLATILE 订阅会收不到
        # 「启动前已经发过」的静态外参，导致多发布者/静态 map->odom 检查漏检。
        static_qos = QoSProfile(
            depth=100, reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(
            TFMessage, "/tf_static", lambda msg: self._on_tf(msg, static=True), static_qos)

        self._last_status = 0.0
        self._last_sample = 0.0
        self._sample_period = 1.0 / max(settings.sample_hz, 1.0)

    # -- 发现 ----------------------------------------------------------
    def _topic_exists(self, name):
        try:
            return any(name in names for _, names in self.node.get_topic_names_and_types())
        except Exception:  # noqa: BLE001
            return False

    def _detect_frames(self):
        settings = self.settings
        self.odom_frame = settings.odom_frame or DEFAULT_ODOM_FRAME
        self.map_frame = settings.map_frame
        self.base_frame = settings.base_frame
        self.gimbal_frame = settings.gimbal_frame
        self.lidar_frame = settings.lidar_frame

        # 从 /lidar_odometry 的 header 里补全 odom / lidar 帧名（比拍脑袋写死可靠）
        try:
            from nav_msgs.msg import Odometry
            target = self.node.resolve_topic_name("lidar_odometry")
            if any(name == target for name, _ in self.node.get_topic_names_and_types()):
                holder = {"msg": None}

                def _probe(msg, holder=holder):
                    holder["msg"] = msg

                subscription = self.node.create_subscription(Odometry, target, _probe, 10)
                deadline = time.monotonic() + 3.0
                while time.monotonic() < deadline and holder["msg"] is None:
                    self._rclpy.spin_once(self.node, timeout_sec=0.05)
                self.node.destroy_subscription(subscription)
                if holder["msg"] is not None:
                    if holder["msg"].header.frame_id:
                        self.odom_frame = holder["msg"].header.frame_id
                    if holder["msg"].child_frame_id:
                        self.lidar_frame = holder["msg"].child_frame_id
        except Exception as exc:  # noqa: BLE001
            print(f"[警告] 探测 lidar_odometry 帧名失败: {describe_exception(exc)}")

        if not self.lidar_frame:
            for candidate in ("livox_imu", "front_mid360", "livox_frame"):
                try:
                    self.tf_buffer.lookup_transform(
                        self.odom_frame, candidate, self._Time())
                    self.lidar_frame = candidate
                    break
                except Exception:  # noqa: BLE001
                    continue
        if not self.lidar_frame:
            self.lidar_frame = "livox_imu"
        print(f"[帧] map={self.map_frame} odom={self.odom_frame} base={self.base_frame} "
              f"gimbal={self.gimbal_frame} lidar={self.lidar_frame}")

    def _detect_cloud_topics(self):
        settings = self.settings
        try:
            graph = self.node.get_topic_names_and_types()
        except Exception:  # noqa: BLE001
            graph = []
        available = {name.lstrip("/"): name for name, types in graph
                     if "sensor_msgs/msg/PointCloud2" in types}

        if settings.cloud_topics:
            resolved = []
            for item in settings.cloud_topics:
                item = item.strip().lstrip("/")
                match = available.get(item)
                if match is None:
                    for name in available.values():
                        if name.lstrip("/").endswith(item):
                            match = name
                            break
                if match is None:
                    # wait_ready 会高频重扫 graph；同一个缺失话题只提示一次。
                    if item not in self._warned_missing_topics:
                        self._warned_missing_topics.add(item)
                        print(f"[警告] 找不到点云话题 {item}（忽略）。")
                elif match not in resolved:
                    resolved.append(match)
            found = resolved
        else:
            found = []
            for stripped, name in available.items():
                for candidate in CLOUD_CANDIDATES:
                    if stripped == candidate or stripped.endswith("/" + candidate):
                        found.append((CLOUD_CANDIDATES.index(candidate), name))
                        break
            found = [name for _, name in sorted(set(found))]
            found = found[:max(settings.max_topics, 1)]

        if found != self.cloud_topics:
            self.cloud_topics = found
            if found:
                print("[点云] " + ", ".join(found))
            else:
                print("[警告] 没有自动识别到点云话题。用 ros2 topic list 确认后加 --cloud-topics。")

    def subscribe_cloud_topics(self):
        """幂等：为当前 cloud_topics 里还没订阅的话题建订阅。"""
        for topic in self.cloud_topics:
            if topic in self.cloud_subs:
                continue
            self.raw_topics[topic] = {"stamps": [], "frames": [], "points": [],
                                      "intensity": []}
            self.cloud_counts[topic] = 0
            self.cloud_subs[topic] = self.node.create_subscription(
                self._PointCloud2, topic,
                lambda msg, name=topic: self._on_cloud(msg, name),
                self._cloud_qos)

    # -- 订阅回调 ------------------------------------------------------
    def _on_cloud(self, msg, topic):
        if msg.width * msg.height == 0:
            return
        frame = msg.header.frame_id.strip()
        if not frame:
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.cloud_received[topic] = self.cloud_received.get(topic, 0) + 1
        # 按消息时间戳限频（--rate）：源话题 10 Hz 时存 5 Hz 就够，配准精度不变，
        # 内存和分析时间减半。用 0.9 的容差吸收 10 Hz 源的抖动。
        min_period = 0.9 / max(self.settings.rate_hz, 0.1)
        last_stamp = self._cloud_last_stamp.get(topic)
        if last_stamp is not None and stamp - last_stamp < min_period:
            return
        self._cloud_last_stamp[topic] = stamp
        points, intensity = read_cloud_points(msg, self._point_cloud2)
        if points is None or len(points) < 10:
            self.parse_failures += 1
            if self.parse_failures in (1, 20, 200):
                print(f"[警告] {topic} 点云解析失败 {self.parse_failures} 次"
                      f"（frame_id={frame!r}, fields="
                      f"{[field.name for field in msg.fields]}）")
            return
        seed = int(self.cloud_counts.get(topic, 0)) + 17
        points = downsample_points(points, self.settings.voxel,
                                   self.settings.max_points, seed)
        record = self.raw_topics[topic]
        record["stamps"].append(stamp)
        record["frames"].append(frame)
        record["points"].append(np.asarray(points, dtype=np.float32))
        record["intensity"].append(None if intensity is None
                                   else np.asarray(intensity, dtype=np.float32))
        self.cloud_counts[topic] += 1

    def _on_joint(self, msg):
        for name, position in zip(msg.name, msg.position):
            if name == "gimbal_yaw_joint":
                self.joint_deg = math.degrees(float(position))
                self.joint_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                return

    def _on_c2g(self, msg):
        self.c2g_deg = float(msg.data)

    def _on_odom(self, msg):
        self.odom_msg = msg

    def _on_lidar_odom(self, msg):
        self.lidar_odom_msg = msg

    def _on_tf(self, msg, static=False):
        for transform in msg.transforms:
            child = transform.child_frame_id
            parent = transform.header.frame_id
            if not (child and parent):
                continue
            self.tf_parents.setdefault(child, set()).add(parent)
            pair = (parent, child)
            (self.tf_static_pairs if static else self.tf_dynamic_pairs).add(pair)
            if static and parent == self.map_frame and child == self.odom_frame:
                rotation = transform.transform.rotation
                translation = transform.transform.translation
                self.static_map_odom = {
                    "x": translation.x, "y": translation.y,
                    "yaw_deg": math.degrees(math.atan2(
                        2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
                        1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z))),
                }

    # -- 采样 ----------------------------------------------------------
    def _lookup(self, target, source, stamp=None, timeout=0.0, record=True):
        if target == source:
            return np.eye(4)
        try:
            if stamp is None:
                at = self._Time()
            elif hasattr(stamp, "sec"):
                at = self._Time.from_msg(stamp)
            else:
                at = self._Time(seconds=float(stamp))
            return matrix_from_transform(
                self.tf_buffer.lookup_transform(
                    target, source, at, self._Duration(seconds=timeout)))
        except Exception:  # noqa: BLE001
            if record:
                key = f"{target}<-{source}"
                self.missing_tf[key] = self.missing_tf.get(key, 0) + 1
            return None

    def sample_once(self):
        row = {name: float("nan") for name in SAMPLE_FIELDS}
        row["t_stamp"] = self.node.get_clock().now().nanoseconds * 1e-9
        row["t_wall"] = time.time()
        row["joint_deg"] = self.joint_deg
        row["c2g_deg"] = self.c2g_deg

        base = self._lookup(self.odom_frame, self.base_frame)
        gimbal = self._lookup(self.odom_frame, self.gimbal_frame)
        base_gimbal = self._lookup(self.base_frame, self.gimbal_frame)
        gimbal_lidar = self._lookup(self.gimbal_frame, self.lidar_frame)
        map_odom = self._lookup(self.map_frame, self.odom_frame)
        odom_lidar = self._lookup(self.odom_frame, self.lidar_frame)

        ok = base is not None
        if map_odom is not None:
            row["yaw_map_odom_deg"] = yaw_deg_of(map_odom)
        if base is not None:
            row["yaw_odom_base_deg"] = yaw_deg_of(base)
            row["base_x"] = float(base[0, 3])
            row["base_y"] = float(base[1, 3])
        if base_gimbal is not None:
            row["yaw_base_gimbal_deg"] = yaw_deg_of(base_gimbal)
        if gimbal_lidar is not None:
            row["yaw_gimbal_lidar_deg"] = yaw_deg_of(gimbal_lidar)
        if odom_lidar is not None:
            row["yaw_odom_lidar_tf_deg"] = yaw_deg_of(odom_lidar)
        if gimbal is not None and base is not None:
            row["gimbal_phys_deg"] = wrap_deg(yaw_deg_of(gimbal) - yaw_deg_of(base))
        elif base_gimbal is not None:
            row["gimbal_phys_deg"] = wrap_deg(yaw_deg_of(base_gimbal))
        elif np.isfinite(self.joint_deg):
            row["gimbal_phys_deg"] = wrap_deg(-self.joint_deg)

        if self.odom_msg is not None:
            pose = self.odom_msg.pose.pose
            row["yaw_odom_nav_deg"] = math.degrees(yaw_from_pose(pose))
            row["base_vx"] = float(self.odom_msg.twist.twist.linear.x)
            row["base_vy"] = float(self.odom_msg.twist.twist.linear.y)
            row["base_wz"] = float(self.odom_msg.twist.twist.angular.z)
        if self.lidar_odom_msg is not None:
            row["yaw_odom_lidar_odom_deg"] = math.degrees(
                yaw_from_pose(self.lidar_odom_msg.pose.pose))
        row["tf_ok"] = 1.0 if ok else 0.0
        self.samples.append(row)

    def wait_ready(self, timeout):
        """等「至少一路点云真的进数据」+「TF 链可用」再开始计时录。

        光看 graph 里有没有话题不够：ROS 2 的发现是异步的，节点刚起来时通常只看到
        本机一部分发布者；而且话题存在 != 有数据（QoS 不匹配、驱动还没出帧）。
        """
        deadline = time.monotonic() + max(timeout, 1.0)
        last_note = 0.0
        previous_topics = list(self.cloud_topics)
        topic_change = time.monotonic()
        while self._rclpy.ok() and time.monotonic() < deadline:
            self._rclpy.spin_once(self.node, timeout_sec=0.05)
            self._detect_cloud_topics()
            self.subscribe_cloud_topics()
            now = time.monotonic()
            if self.cloud_topics != previous_topics:
                # ROS 2 的发现是分批到达的：话题列表稳定 1 s 再开始，避免漏掉晚到的那几路。
                previous_topics = list(self.cloud_topics)
                topic_change = now
            frames = sum(self.cloud_counts.values())
            # ⚠ 不能直接 all(...)：_lookup 成功时返回 4x4 ndarray，ndarray 的真值
            # 是含糊的（ValueError）。这里显式判 None。
            tf_ready = all(
                self._lookup(target, source, record=False) is not None
                for target, source in (
                    (self.odom_frame, self.base_frame),
                    (self.odom_frame, self.gimbal_frame),
                    (self.odom_frame, self.lidar_frame)))
            if frames > 0 and tf_ready and now - topic_change >= 1.0:
                print(f"[就绪] 点云 {frames} 帧 / {len(self.cloud_counts)} 路，TF 链可用；"
                      "开始计时采集。")
                return True
            if now - last_note > 5.0:
                last_note = now
                print(f"[等待] 点云 {frames} 帧，TF {'可用' if tf_ready else '未就绪'}，"
                      f"继续等（上限 {timeout:.0f} s）…")
            time.sleep(0.05)
        return False

    def spin(self):
        settings = self.settings
        stop = {"flag": False}

        def _on_signal(_signum, _frame):
            stop["flag"] = True

        for sig in (signal.SIGINT, signal.SIGTERM,
                    getattr(signal, "SIGHUP", signal.SIGTERM)):
            try:
                signal.signal(sig, _on_signal)
            except (ValueError, OSError):
                pass

        print("=" * 68)
        print(" 云台点云转动抓包：现在开始**慢慢**转云台（±30~45°），转过去再转回来")
        print(" 底盘保持静止；转完按 Ctrl-C 结束（或等 --duration 到点）")
        print("=" * 68)

        started = time.monotonic()
        deadline = started + settings.duration if settings.duration > 0 else None
        discover_until = started + 5.0
        last_discover = started
        while self._rclpy.ok() and not stop["flag"]:
            try:
                self._rclpy.spin_once(self.node, timeout_sec=0.02)
            except KeyboardInterrupt:
                stop["flag"] = True
                break
            now = time.monotonic()
            if now < discover_until and now - last_discover > 1.0:
                last_discover = now
                self._detect_cloud_topics()
                self.subscribe_cloud_topics()
            if now - self._last_sample >= self._sample_period:
                self._last_sample = now
                try:
                    self.sample_once()
                except Exception as exc:  # noqa: BLE001
                    print(f"[警告] 采样失败: {describe_exception(exc)}")
            if now - self._last_status >= 1.0:
                self._last_status = now
                self._print_status(now - started)
            if deadline is not None and now >= deadline:
                break
        self.recorded_duration = time.monotonic() - started

    def _print_status(self, elapsed):
        angle = self._current_angle()
        frames = sum(self.cloud_counts.values())
        speed = self._base_speed()
        print(f"[{elapsed:6.1f}s] 云台 {angle:+6.1f}° | 点云帧 {frames:4d}"
              f" | 底盘 |v| {speed:4.2f} m/s | 样本 {len(self.samples):5d}")

    def _current_angle(self):
        gimbal = self._lookup(self.odom_frame, self.gimbal_frame, record=False)
        base = self._lookup(self.odom_frame, self.base_frame, record=False)
        if gimbal is not None and base is not None:
            return wrap_deg(yaw_deg_of(gimbal) - yaw_deg_of(base))
        if np.isfinite(self.joint_deg):
            return -self.joint_deg
        return float("nan")

    def _base_speed(self):
        if len(self.samples) < 2:
            return 0.0
        last, first = self.samples[-1], self.samples[0]
        if not (np.isfinite(last["base_x"]) and np.isfinite(first["base_x"])):
            return 0.0
        dt = max(last["t_stamp"] - first["t_stamp"], 1e-6)
        return math.hypot(last["base_x"] - first["base_x"],
                          last["base_y"] - first["base_y"]) / dt

    # -- 收尾 ----------------------------------------------------------
    def finalize(self):
        recordings = {}
        samples = samples_to_arrays(self.samples)
        for topic, record in self.raw_topics.items():
            count = len(record["stamps"])
            if count == 0:
                print(f"[警告] {topic} 没收到点云，跳过。")
                continue
            T_odom_frame = np.tile(np.eye(4), (count, 1, 1))
            T_odom_lidar = np.tile(np.eye(4), (count, 1, 1))
            T_map_odom = np.tile(np.eye(4), (count, 1, 1))
            gimbal = np.full(count, np.nan)
            base = np.full(count, np.nan)
            tf_ok = np.zeros(count, dtype=bool)
            for index, (stamp, frame) in enumerate(zip(record["stamps"], record["frames"])):
                frame_matrix = self._lookup(self.odom_frame, frame, stamp, 0.0)
                if frame_matrix is None:
                    continue
                lidar_matrix = self._lookup(self.odom_frame, self.lidar_frame, stamp, 0.0)
                map_matrix = self._lookup(self.map_frame, self.odom_frame, stamp, 0.0)
                base_matrix = self._lookup(self.odom_frame, self.base_frame, stamp, 0.0)
                gimbal_matrix = self._lookup(self.odom_frame, self.gimbal_frame, stamp, 0.0)
                T_odom_frame[index] = frame_matrix
                if lidar_matrix is not None:
                    T_odom_lidar[index] = lidar_matrix
                if map_matrix is not None:
                    T_map_odom[index] = map_matrix
                if base_matrix is not None:
                    base[index] = yaw_deg_of(base_matrix)
                if gimbal_matrix is not None and base_matrix is not None:
                    gimbal[index] = wrap_deg(
                        yaw_deg_of(gimbal_matrix) - yaw_deg_of(base_matrix))
                tf_ok[index] = True
            # TF 是按 cloud.header.stamp 查的；base/gimbal 那两段可能比点云晚到几毫秒，
            # 按最坏情况会丢掉 5~10% 的帧。缺的用 20 Hz 标量采样插值补上（云台角是
            # 低频量，插值误差远小于逐帧信号），别让这些帧白丢。
            stamps = np.asarray(record["stamps"], dtype=np.float64)
            if np.isnan(gimbal).any():
                fill = interpolate_at(stamps, samples, "gimbal_phys_deg", clamp=True)
                gimbal = np.where(np.isnan(gimbal), fill, gimbal)
            if np.isnan(base).any():
                fill = interpolate_at(stamps, samples, "yaw_odom_base_deg", clamp=True)
                base = np.where(np.isnan(base), fill, base)
            recordings[topic] = TopicRecording(
                topic=topic,
                stamps=np.asarray(record["stamps"], dtype=np.float64),
                frame_ids=list(record["frames"]),
                points=record["points"],
                intensity=record["intensity"],
                T_odom_frame=T_odom_frame,
                T_odom_lidar=T_odom_lidar,
                T_map_odom=T_map_odom,
                gimbal_deg=gimbal,
                base_deg=base,
                tf_ok=tf_ok,
            )
            print(f"[TF] {topic}: {int(tf_ok.sum())}/{count} 帧在 header.stamp 上查到了 TF；"
                  f"云台角覆盖 {int(np.isfinite(gimbal).sum())}/{count}")

        if self.missing_tf:
            items = sorted(self.missing_tf.items(), key=lambda item: -item[1])[:8]
            print("[TF] 查询失败统计: " + ", ".join(f"{k}×{v}" for k, v in items))
        return recordings, samples, self._manifest_extra()

    def _manifest_extra(self):
        duplicates = {
            child: sorted(parents)
            for child, parents in self.tf_parents.items() if len(parents) > 1
        }
        try:
            frames_yaml = self.tf_buffer.all_frames_as_yaml()
        except Exception:  # noqa: BLE001
            frames_yaml = ""
        try:
            node_names = sorted(set(self.node.get_node_names()))
        except Exception:  # noqa: BLE001
            node_names = []
        conflicts = sorted(self.tf_static_pairs & self.tf_dynamic_pairs)
        publishers = {}
        for topic in ("/tf", "/tf_static"):
            try:
                publishers[topic] = sorted({endpoint.node_name
                                            for endpoint in
                                            self.node.get_publishers_info_by_topic(topic)})
            except Exception:  # noqa: BLE001
                publishers[topic] = []
        return {
            "tf_parents": {child: sorted(parents)
                           for child, parents in self.tf_parents.items()},
            "tf_duplicate_parents": duplicates,
            "tf_static_dynamic_conflicts": [
                {"parent": parent, "child": child} for parent, child in conflicts],
            "tf_static_map_odom": self.static_map_odom,
            "tf_publishers": publishers,
            "tf_snapshot": frames_yaml,
            "missing_tf": dict(self.missing_tf),
            "cloud_counts": dict(self.cloud_counts),
            "cloud_received": dict(self.cloud_received),
            "node_names": node_names,
        }


#: PointField.datatype -> numpy 格式（ROS 2 里 7 是 FLOAT32，8 是 FLOAT64）。
_POINT_FIELD_FORMATS = {1: "i1", 2: "u1", 3: "i2", 4: "u2",
                        5: "i4", 6: "u4", 7: "f4", 8: "f8"}


def read_cloud_points(msg, point_cloud2_module=None):
    """PointCloud2 -> (N,3) float32 + 可选 intensity。

    ⚠ **不能直接用 sensor_msgs_py.read_points_numpy**：它要求所有字段同类型
    （`assert all(cloud.fields[0].datatype == field.datatype ...)`），而 Livox 的
    `/livox/lidar/pointcloud` 是 float32 x/y/z/intensity + uint8 tag/line + float64
    timestamp，一进来就断言失败。早先版本把这个异常吞成"没有点"，表现是话题在发、
    工具却一帧都收不到。这里按 fields 的 offset 自己拼结构化 dtype，混合类型也能读。
    """
    by_name = {field.name: field for field in msg.fields}
    if not all(axis in by_name for axis in ("x", "y", "z")):
        return None, None
    count = int(msg.width) * int(msg.height)
    if count <= 0:
        return None, None

    names, formats, offsets = [], [], []
    for field in msg.fields:
        fmt = _POINT_FIELD_FORMATS.get(int(field.datatype))
        if fmt is None or int(field.count) != 1:
            continue
        names.append(field.name)
        formats.append(fmt)
        offsets.append(int(field.offset))
    if not names:
        return None, None
    dtype = np.dtype({"names": names, "formats": formats, "offsets": offsets,
                      "itemsize": int(msg.point_step)})
    if getattr(msg, "is_bigendian", False):
        dtype = dtype.newbyteorder(">")

    try:
        cloud = np.frombuffer(msg.data, dtype=dtype, count=count)
    except (TypeError, ValueError):
        # 极端情况下 data 长度和 point_step*cnt 不一致，退回 ROS 的逐点读法（慢但稳）。
        try:
            packed = point_cloud2_module.read_points(
                msg, field_names=("x", "y", "z"), skip_nans=True)
            xyz = np.asarray(list(packed), dtype=np.float32)
            if xyz.size == 0:
                return None, None
            return xyz.reshape(-1, 3), None
        except Exception:  # noqa: BLE001
            return None, None

    xyz = np.column_stack([cloud["x"], cloud["y"], cloud["z"]]).astype(np.float32)
    intensity = (np.asarray(cloud["intensity"], dtype=np.float32)
                 if "intensity" in names else None)
    mask = np.isfinite(xyz).all(axis=1)
    if not mask.all():
        xyz = xyz[mask]
        if intensity is not None:
            intensity = intensity[mask]
    if len(xyz) == 0:
        return None, None
    return xyz, intensity


def yaw_from_pose(pose):
    q = pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def matrix_from_transform(transform_stamped):
    transform = transform_stamped.transform
    translation = transform.translation
    rotation = transform.rotation
    return rigid_matrix(
        (translation.x, translation.y, translation.z),
        (rotation.x, rotation.y, rotation.z, rotation.w))


def samples_to_arrays(rows):
    arrays = {}
    for name in SAMPLE_FIELDS:
        values = []
        for row in rows:
            value = row.get(name, float("nan"))
            try:
                values.append(float(value))
            except (TypeError, ValueError):
                values.append(float("nan"))
        arrays[name] = np.asarray(values, dtype=np.float64)
    return arrays


# =============================================================
# 数据包读写
# =============================================================
def topic_to_npz_arrays(recording):
    offsets = [0]
    chunks = []
    intensity_chunks = []
    has_intensity = False
    for index, points in enumerate(recording.points):
        points = np.asarray(points, dtype=np.float32)
        chunks.append(points)
        offsets.append(offsets[-1] + len(points))
        intensity = None
        if recording.intensity and index < len(recording.intensity):
            intensity = recording.intensity[index]
        if intensity is not None and len(intensity) == len(points):
            intensity_chunks.append(np.asarray(intensity, dtype=np.float32))
            has_intensity = True
        else:
            intensity_chunks.append(np.zeros(len(points), dtype=np.float32))
    points_flat = (np.concatenate(chunks, axis=0) if chunks
                   else np.zeros((0, 3), np.float32))
    arrays = {
        "topic": np.asarray(recording.topic, dtype="<U160"),
        "stamps": np.asarray(recording.stamps, dtype=np.float64),
        "frame_ids": np.asarray(recording.frame_ids, dtype="<U48"),
        "points": points_flat,
        "offsets": np.asarray(offsets, dtype=np.int64),
        "T_odom_frame": np.asarray(recording.T_odom_frame, dtype=np.float64),
        "T_odom_lidar": np.asarray(recording.T_odom_lidar, dtype=np.float64),
        "T_map_odom": np.asarray(recording.T_map_odom, dtype=np.float64),
        "gimbal_deg": np.asarray(recording.gimbal_deg, dtype=np.float64),
        "base_deg": np.asarray(recording.base_deg, dtype=np.float64),
        "tf_ok": np.asarray(recording.tf_ok, dtype=bool),
    }
    if has_intensity:
        arrays["intensity"] = np.concatenate(intensity_chunks, axis=0)
    return arrays


def recording_from_npz(arrays):
    offsets = arrays["offsets"]
    points_flat = arrays["points"]
    points = [points_flat[offsets[i]:offsets[i + 1]] for i in range(len(offsets) - 1)]
    intensity_flat = arrays.get("intensity")
    intensity = [None] * len(points)
    if intensity_flat is not None:
        intensity = [intensity_flat[offsets[i]:offsets[i + 1]]
                     for i in range(len(offsets) - 1)]
    return TopicRecording(
        topic=str(arrays["topic"]),
        stamps=np.asarray(arrays["stamps"], dtype=np.float64),
        frame_ids=[str(item) for item in arrays["frame_ids"]],
        points=points,
        intensity=intensity,
        T_odom_frame=np.asarray(arrays["T_odom_frame"], dtype=np.float64),
        T_odom_lidar=np.asarray(arrays["T_odom_lidar"], dtype=np.float64),
        T_map_odom=np.asarray(arrays["T_map_odom"], dtype=np.float64),
        gimbal_deg=np.asarray(arrays["gimbal_deg"], dtype=np.float64),
        base_deg=np.asarray(arrays["base_deg"], dtype=np.float64),
        tf_ok=np.asarray(arrays["tf_ok"], dtype=bool),
    )


def write_bundle(bundle, samples, recordings, manifest):
    bundle = Path(bundle)
    (bundle / "raw").mkdir(parents=True, exist_ok=True)
    (bundle / "clouds").mkdir(parents=True, exist_ok=True)
    (bundle / "analysis").mkdir(parents=True, exist_ok=True)

    write_samples_csv(bundle / "samples.csv", samples)
    for topic, recording in recordings.items():
        np.savez_compressed(bundle / "raw" / (safe_topic_name(topic) + ".npz"),
                            **topic_to_npz_arrays(recording))
    (bundle / "manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")
    tf_snapshot = manifest.get("capture", {}).get("tf_snapshot", "")
    if tf_snapshot:
        (bundle / "tf_snapshot.yaml").write_text(str(tf_snapshot), encoding="utf-8")
    return bundle


def write_samples_csv(path, samples):
    count = len(samples.get("t_stamp", []))
    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(SAMPLE_FIELDS)
        for index in range(count):
            row = []
            for name in SAMPLE_FIELDS:
                value = float(samples[name][index])
                row.append("" if not np.isfinite(value) else f"{value:.6f}")
            writer.writerow(row)


def read_samples_csv(path):
    values = {name: [] for name in SAMPLE_FIELDS}
    with open(path, newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            for name in SAMPLE_FIELDS:
                text = (row.get(name) or "").strip()
                try:
                    values[name].append(float(text) if text else float("nan"))
                except ValueError:
                    values[name].append(float("nan"))
    return {name: np.asarray(column, dtype=np.float64) for name, column in values.items()}


def load_bundle(bundle):
    bundle = Path(bundle).expanduser().resolve()
    manifest_path = bundle / "manifest.json"
    if not manifest_path.exists():
        raise FileNotFoundError(f"不是有效数据包（缺 manifest.json）: {bundle}")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    samples = read_samples_csv(bundle / "samples.csv")
    recordings = {}
    for path in sorted((bundle / "raw").glob("*.npz")):
        with np.load(path) as loaded:
            arrays = {name: loaded[name] for name in loaded.files}
        recording = recording_from_npz(arrays)
        recordings[recording.topic] = recording
    return bundle, manifest, samples, recordings


def export_clouds(bundle, recordings, settings):
    """按云台角均匀挑关键帧，落成 PCD（分析系表达）。"""
    if settings.no_pcd:
        return []
    rows = []
    keys = ("topic", "idx", "stamp", "frame_id", "gimbal_deg", "points", "pcd",
            "expressed_in")
    for topic, recording in recordings.items():
        valid = [index for index in range(len(recording))
                 if recording.tf_ok[index]
                 and len(recording.points[index]) >= settings.min_points]
        if not valid:
            continue
        angles = np.array([recording.gimbal_deg[index] for index in valid])
        if np.isfinite(angles).sum() >= 2:
            order = np.argsort(np.where(np.isfinite(angles), angles, 0.0))
            valid = [valid[int(position)] for position in order]
        wanted = max(2, min(settings.pcd_max, len(valid)))
        picked = np.unique(np.linspace(0, len(valid) - 1, wanted).astype(int))
        name = safe_topic_name(topic)
        for position in picked:
            frame_index = int(valid[int(position)])
            points = recording.points[frame_index]
            matrix = recording.T_odom_frame[frame_index]
            if settings.analysis_frame == settings.map_frame:
                matrix = recording.T_map_odom[frame_index] @ matrix
            world = transform_points(np.asarray(points, dtype=np.float64), matrix)
            angle = recording.gimbal_deg[frame_index]
            angle_text = "nan" if not np.isfinite(angle) else f"{angle:+06.1f}"
            filename = f"{name}_{frame_index:04d}_g{angle_text}.pcd"
            intensity = None
            if recording.intensity and frame_index < len(recording.intensity):
                intensity = recording.intensity[frame_index]
            write_pcd(bundle / "clouds" / filename, world, intensity)
            rows.append({
                "topic": topic,
                "idx": frame_index,
                "stamp": f"{recording.stamps[frame_index]:.6f}",
                "frame_id": recording.frame_ids[frame_index],
                "gimbal_deg": "" if not np.isfinite(angle) else f"{angle:.3f}",
                "points": len(points),
                "pcd": filename,
                "expressed_in": settings.analysis_frame,
            })
    with open(bundle / "clouds" / "index.csv", "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)
    return rows


# =============================================================
# 分析
# =============================================================
def effective_angles(recording, samples):
    """优先用 TF 口径的物理云台角；缺失时退回 -joint（URDF 轴为 -z 时两者等价）。"""
    angles = np.asarray(recording.gimbal_deg, dtype=np.float64).copy()
    if np.isfinite(angles).sum() >= 3:
        return angles, "TF: yaw(odom->gimbal) - yaw(odom->base)"
    joint = samples.get("joint_deg", np.zeros(0))
    times = samples.get("t_stamp", np.zeros(0))
    if len(joint) >= 2 and len(times) == len(joint) and np.isfinite(joint).sum() >= 2:
        mask = np.isfinite(joint) & np.isfinite(times)
        if mask.sum() >= 2:
            return np.interp(recording.stamps, times[mask], -joint[mask]), \
                "退化: -joint_states（TF 不可用）"
    return angles, "无可用云台角"


def interpolate_at(stamps, samples, name, clamp=False):
    """按时间插值到给定时间戳。

    clamp=True 时区间外取首/末样本的值（就绪门期间抓到的云帧早于第一个标量采样，
    那时云台一般还没动，用端点值补比丢帧好）；默认区间外保持 NaN，避免拿外推值
    冒充真实数据。
    """
    values = samples.get(name, np.zeros(0))
    times = samples.get("t_stamp", np.zeros(0))
    result = np.full(len(stamps), np.nan)
    if len(values) != len(times) or len(times) < 2:
        return result
    mask = np.isfinite(values) & np.isfinite(times)
    if mask.sum() < 2:
        return result
    if clamp:
        return np.interp(stamps, times[mask], values[mask])
    inside = (stamps >= times[mask][0]) & (stamps <= times[mask][-1])
    result[inside] = np.interp(stamps[inside], times[mask], values[mask])
    return result


def lidar_xy_of(recording, index):
    matrix = recording.T_odom_lidar[index]
    return np.array([matrix[0, 3], matrix[1, 3]], dtype=np.float64)


def analyze_topic(topic, recording, samples, settings):
    frames = len(recording)
    valid = [index for index in range(frames) if recording.tf_ok[index]]
    angles, angle_source = effective_angles(recording, samples)
    finite_angles = angles[np.isfinite(angles)]
    travel = float(finite_angles.max() - finite_angles.min()) if len(finite_angles) else 0.0
    gimbal_min = float(finite_angles.min()) if len(finite_angles) else float("nan")
    gimbal_max = float(finite_angles.max()) if len(finite_angles) else float("nan")

    world_points = {}
    for index in valid:
        matrix = recording.T_odom_frame[index]
        if settings.analysis_frame == settings.map_frame:
            matrix = recording.T_map_odom[index] @ matrix
        world_points[index] = transform_points(
            np.asarray(recording.points[index], dtype=np.float64), matrix)

    usable = [index for index in valid
              if len(recording.points[index]) >= settings.min_points]
    if not usable:
        return TopicResult(
            topic=topic, frames=frames, valid=len(valid),
            frame_id=recording.frame_ids[0] if frames else "",
            travel_deg=travel, gimbal_min=gimbal_min, gimbal_max=gimbal_max,
            pairs=[], n_pairs=0, slope=float("nan"), corr=float("nan"),
            slope_anchor=float("nan"), corr_anchor=float("nan"),
            lag=float("nan"), lag_corr=float("nan"), lag_slope=float("nan"),
            median_inlier=float("nan"), median_shift=float("nan"),
            median_points=0, angle_source=angle_source, kind="insufficient",
            notes=["没有足够点数的帧"])

    map_yaw_at_frames = interpolate_at(recording.stamps, samples, "yaw_map_odom_deg")
    pairs = []
    aligner = Aligner(settings)
    previous_index = None
    for index in usable:
        if previous_index is not None:
            aligner.set_reference(world_points[previous_index],
                                  lidar_xy_of(recording, previous_index),
                                  seed=previous_index)
            result = aligner.align(world_points[index], lidar_xy_of(recording, index),
                                   seed=index)
            if result is not None:
                dt = float(recording.stamps[index] - recording.stamps[previous_index])
                if dt <= settings.pair_gap:
                    pairs.append(PairRecord(
                        prev_idx=previous_index,
                        idx=index,
                        prev_stamp=float(recording.stamps[previous_index]),
                        stamp=float(recording.stamps[index]),
                        dt=dt,
                        dtheta_gimbal=_delta(angles[previous_index], angles[index]),
                        dtheta_base=_delta(recording.base_deg[previous_index],
                                           recording.base_deg[index]),
                        dyaw_map_odom=_delta(map_yaw_at_frames[previous_index],
                                             map_yaw_at_frames[index]),
                        dpsi_cloud=math.degrees(-result["psi"]),
                        psi_align=math.degrees(result["psi"]),
                        shift=result["shift_norm"],
                        inlier=result["inlier"],
                        rmse=result["rmse"],
                    ))
        previous_index = index

    # 锚点帧取「云台角离中位最近」的那帧，而不是第一帧：这样锚点离两端都近，
    # 相对角不会超过搜索范围。范围也按总行程放宽（最多 180°）。
    finite_usable = [(index, angles[index]) for index in usable
                     if np.isfinite(angles[index])]
    if finite_usable:
        median_angle = float(np.median([value for _, value in finite_usable]))
        anchor_index = min(
            usable, key=lambda index: abs(angles[index] - median_angle)
            if np.isfinite(angles[index]) else 1e9)
    else:
        anchor_index = usable[0]
    anchor_settings = replace(
        settings, yaw_range=max(settings.yaw_range, min(180.0, travel * 0.9 + 10.0)))
    anchor_aligner = Aligner(anchor_settings)
    anchor_aligner.set_reference(world_points[anchor_index],
                                 lidar_xy_of(recording, anchor_index),
                                 seed=anchor_index)
    anchor_values = {}
    anchor_relative = []
    for position, index in enumerate(usable):
        if position % max(settings.anchor_stride, 1) != 0:
            continue
        relative = _delta(angles[anchor_index], angles[index])
        if not np.isfinite(relative) or abs(relative) > 0.9 * anchor_settings.yaw_range:
            continue
        result = anchor_aligner.align(world_points[index],
                                      lidar_xy_of(recording, index), seed=index)
        if result is None or result["inlier"] < 0.5:
            continue
        anchor_relative.append(abs(relative))
        anchor_values[index] = (math.degrees(-result["psi"]), result["inlier"],
                                result["shift_norm"], result["shift"][0],
                                result["shift"][1])
    for pair in pairs:
        if pair.idx in anchor_values:
            (pair.anchor_yaw, pair.anchor_inlier, pair.anchor_shift,
             pair.anchor_shift_x, pair.anchor_shift_y) = anchor_values[pair.idx]

    # 平移绕圈：把锚点平移向量对云台角做谐波拟合；同时用车体位姿做同样的拟合，
    # 区分“点云在绕”还是“车体也在绕”。yaw 斜率≈0 时这一项才是有意义的判据。
    orbit_radius, orbit_residual = fit_translation_orbit(
        [angles[index] for index in anchor_values],
        [anchor_values[index][3] for index in anchor_values],
        [anchor_values[index][4] for index in anchor_values])
    base_x_at_frames = interpolate_at(recording.stamps, samples, "base_x", clamp=True)
    base_y_at_frames = interpolate_at(recording.stamps, samples, "base_y", clamp=True)
    reference = usable[0]
    base_orbit_radius, _ = fit_translation_orbit(
        [angles[index] for index in usable],
        [base_x_at_frames[index] - base_x_at_frames[reference] for index in usable],
        [base_y_at_frames[index] - base_y_at_frames[reference] for index in usable])

    good = [pair for pair in pairs
            if np.isfinite(pair.dtheta_gimbal)
            and np.isfinite(pair.dpsi_cloud)
            and pair.inlier >= settings.min_inlier
            and abs(pair.dtheta_gimbal) >= settings.min_dtheta]
    x = np.array([pair.dtheta_gimbal for pair in good], dtype=np.float64)
    y = np.array([pair.dpsi_cloud for pair in good], dtype=np.float64)
    slope, corr, n_pairs, _ = robust_fit(x, y)

    # 锚点校验用的是「相对锚点帧的绝对转角」对「相对锚点帧的绝对云台角差」，
    # 不能拿它去对相邻帧的 Δθ 回归（量纲不同，会把斜率算成一半）。
    notes = []
    anchor_good = [pair for pair in good if np.isfinite(pair.anchor_yaw)]
    anchor_angle = angles[anchor_index]
    anchor_median_inlier = (float(np.median([pair.anchor_inlier for pair in anchor_good]))
                            if anchor_good else float("nan"))
    if len(anchor_good) >= 8 and np.isfinite(anchor_angle) and \
            np.isfinite(anchor_median_inlier) and anchor_median_inlier >= 0.55:
        ax = np.array([_delta(anchor_angle, angles[pair.idx]) for pair in anchor_good])
        ay = np.array([pair.anchor_yaw for pair in anchor_good])
        anchor_mask = np.isfinite(ax) & np.isfinite(ay)
        if anchor_mask.sum() >= 8:
            slope_anchor, corr_anchor, _ = linear_fit(ax[anchor_mask], ay[anchor_mask])
        else:
            slope_anchor, corr_anchor = float("nan"), float("nan")
    else:
        slope_anchor, corr_anchor = float("nan"), float("nan")
        notes.append("锚点校验不可靠（有效相对角/内点率不足），只以相邻帧斜率为准")

    lag, lag_corr, lag_slope = estimate_lag(recording, angles, good, settings)

    median_inlier = float(np.median([pair.inlier for pair in pairs])) if pairs else float("nan")
    median_shift = float(np.median([pair.shift for pair in pairs])) if pairs else float("nan")
    median_points = int(np.median([len(recording.points[index]) for index in usable]))

    kind = classify(slope, corr, n_pairs, travel, settings.min_travel)
    if kind == "stable" and np.isfinite(orbit_radius) \
            and orbit_radius >= settings.orbit_min \
            and orbit_residual <= max(0.5 * orbit_radius, 0.02):
        kind = "translation_orbit"
        notes.append(
            f"yaw 斜率≈0，但锚点平移存在 ≈{orbit_radius * 100:.1f} cm 的绕圈"
            f"（残差 {orbit_residual * 100:.1f} cm；车体位姿同法 R="
            f"{base_orbit_radius * 100:.1f} cm）")
    if travel < settings.min_travel:
        notes.append(f"云台行程只有 {travel:.1f}°（阈值 {settings.min_travel:.0f}°）")
    if n_pairs < 8:
        notes.append(f"有效配准对只有 {n_pairs} 对")
    if np.isfinite(median_inlier) and median_inlier < 0.55:
        notes.append(f"中位内点率 {median_inlier:.2f} 偏低，稀疏/动态场景下斜率要打折看")
    if np.isfinite(lag) and abs(lag) >= 2 * settings.lag_step and np.isfinite(lag_corr) \
            and np.isfinite(corr) and abs(lag_corr) > abs(corr) + 0.05:
        notes.append(f"最佳相关出现在 {lag:+.2f} s 滞后处，优先怀疑时间戳/位姿滞后")
    if np.isfinite(lag) and abs(lag) >= settings.lag_max - 0.5 * settings.lag_step \
            and abs(lag_corr) >= 0.5:
        notes.append("最佳滞后落在扫描边界，实际可能更大（加大 --lag-max 再算）")
    if kind in ("follow_pos", "follow_neg", "follow_2x") and np.isfinite(corr) \
            and abs(corr) < 0.6:
        notes.append("相关性 <0.6，斜率只能当方向参考")

    return TopicResult(
        topic=topic, frames=frames, valid=len(valid),
        frame_id=recording.frame_ids[0] if frames else "",
        travel_deg=travel, gimbal_min=gimbal_min, gimbal_max=gimbal_max,
        pairs=pairs, n_pairs=n_pairs, slope=slope, corr=corr,
        slope_anchor=slope_anchor, corr_anchor=corr_anchor,
        lag=lag, lag_corr=lag_corr, lag_slope=lag_slope,
        median_inlier=median_inlier, median_shift=median_shift,
        median_points=median_points, angle_source=angle_source,
        kind=kind, notes=notes,
        orbit_radius=orbit_radius, orbit_residual=orbit_residual,
        base_orbit_radius=base_orbit_radius)


def fit_translation_orbit(angles_deg, shifts_x, shifts_y):
    """把锚点平移 (sx, sy) 对云台角做一次谐波拟合，估计“平移绕圈”半径。

    模型：sx = cx + A cosθ + B sinθ，sy = cy + C cosθ + D sinθ。
    半径取两轴振幅的平均；残差是两轴 RMS。半径远大于残差才说明是真绕圈，
    否则更可能是配准噪声/可见面变化带来的平移抖动。
    """
    angles = np.asarray(angles_deg, dtype=np.float64)
    xs = np.asarray(shifts_x, dtype=np.float64)
    ys = np.asarray(shifts_y, dtype=np.float64)
    mask = np.isfinite(angles) & np.isfinite(xs) & np.isfinite(ys)
    if mask.sum() < 8 or (np.max(angles[mask]) - np.min(angles[mask])) < 10.0:
        return float("nan"), float("nan")
    theta = np.radians(angles[mask])
    design = np.column_stack([np.ones_like(theta), np.cos(theta), np.sin(theta)])
    solx, *_ = np.linalg.lstsq(design, xs[mask], rcond=None)
    soly, *_ = np.linalg.lstsq(design, ys[mask], rcond=None)
    radius = 0.5 * (math.hypot(solx[1], solx[2]) + math.hypot(soly[1], soly[2]))
    pred_x = design @ solx
    pred_y = design @ soly
    residual = float(np.sqrt(np.mean((xs[mask] - pred_x) ** 2 +
                                     (ys[mask] - pred_y) ** 2)))
    return float(radius), residual


def _delta(a, b):
    if not (np.isfinite(a) and np.isfinite(b)):
        return float("nan")
    return wrap_deg(float(b) - float(a))


def estimate_lag(recording, angles, good, settings):
    if len(good) < 8 or settings.lag_max <= 0:
        return float("nan"), float("nan"), float("nan")
    times = np.asarray([pair.stamp for pair in good], dtype=np.float64)
    prev_times = np.asarray([pair.prev_stamp for pair in good], dtype=np.float64)
    y = np.asarray([pair.dpsi_cloud for pair in good], dtype=np.float64)
    valid = np.isfinite(angles)
    if valid.sum() < 3:
        return float("nan"), float("nan"), float("nan")
    finite_times = np.asarray(recording.stamps, dtype=np.float64)[valid]
    finite_angles = angles[valid]
    order = np.argsort(finite_times)
    finite_times, finite_angles = finite_times[order], finite_angles[order]
    best = (float("nan"), float("nan"), float("nan"))
    for lag in np.arange(-settings.lag_max, settings.lag_max + 1e-9, settings.lag_step):
        angle_now = np.interp(times + lag, finite_times, finite_angles,
                              left=np.nan, right=np.nan)
        angle_prev = np.interp(prev_times + lag, finite_times, finite_angles,
                               left=np.nan, right=np.nan)
        x = wrap_deg_array(angle_now - angle_prev)
        mask = np.isfinite(x) & np.isfinite(y)
        if mask.sum() < 8:
            continue
        slope, corr, _ = linear_fit(x[mask], y[mask])
        if not np.isfinite(corr):
            continue
        if not np.isfinite(best[1]) or abs(corr) > abs(best[1]):
            best = (float(lag), float(corr), float(slope))
    # 相关性本身就很低时，"最佳滞后"只是噪声里的最大值，报出来会误导。
    if not np.isfinite(best[1]) or abs(best[1]) < 0.5:
        return float("nan"), float("nan"), float("nan")
    return best


def classify(slope, corr, n_pairs, travel, min_travel):
    if n_pairs < 8 or travel < min_travel or not np.isfinite(slope):
        return "insufficient"
    magnitude = abs(slope)
    if magnitude <= 0.15:
        return "stable"
    # 相关性太低时，斜率本身不可信：不要因为“刚好落在 ±1 附近”就下结论。
    if not np.isfinite(corr) or abs(corr) < 0.5:
        return "unclear"
    if abs(slope - 1.0) <= 0.25:
        return "follow_pos"
    if abs(slope + 1.0) <= 0.25:
        return "follow_neg"
    if abs(magnitude - 2.0) <= 0.4:
        return "follow_2x"
    if magnitude <= 0.7:
        return "partial"
    return "other"


def _maybe_float(value):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return value if np.isfinite(value) else None


def detect_anomalies(samples, settings):
    """找坐标/位姿的单步跳变：map->odom、odom->base、TF 与 /lidar_odometry 的差。

    只看「首末变化」会被启动瞬态和单帧尖峰骗；这些跳变恰恰是「RViz 固定帧 map 时
    整片点云突然摆一下」的直接来源，必须把时间点、幅度、方向列出来。
    """
    anomalies = []
    times = np.asarray(samples.get("t_stamp", np.zeros(0)), dtype=np.float64)
    if len(times) < 3:
        return anomalies
    finite_times = times[np.isfinite(times)]
    if len(finite_times) < 3:
        return anomalies
    t0 = float(finite_times[0])
    threshold = max(float(settings.anomaly_deg), 0.01)

    def collect(name, label):
        values = np.asarray(samples.get(name, np.zeros(0)), dtype=np.float64)
        if len(values) != len(times):
            return
        deltas = wrap_deg_array(np.diff(values))
        for index in np.where(np.abs(deltas) > threshold)[0]:
            anomalies.append({
                "t": float(times[index + 1] - t0),
                "kind": name,
                "label": label,
                "delta_deg": float(deltas[index]),
                "before_deg": _maybe_float(values[index]),
                "after_deg": _maybe_float(values[index + 1]),
            })

    collect("yaw_map_odom_deg", "map→odom 台阶")
    collect("yaw_odom_base_deg", "odom→base 台阶")
    tf_yaw = np.asarray(samples.get("yaw_odom_lidar_tf_deg", np.zeros(0)), dtype=np.float64)
    odom_yaw = np.asarray(samples.get("yaw_odom_lidar_odom_deg", np.zeros(0)), dtype=np.float64)
    if len(tf_yaw) == len(times) and len(odom_yaw) == len(times):
        diff = wrap_deg_array(tf_yaw - odom_yaw)
        for index in np.where(np.abs(diff) > threshold)[0]:
            anomalies.append({
                "t": float(times[index] - t0),
                "kind": "tf_pose_diff",
                "label": "TF odom→lidar 与 /lidar_odometry 不一致",
                "delta_deg": float(diff[index]),
                "before_deg": None,
                "after_deg": None,
            })
    anomalies.sort(key=lambda item: item["t"])
    return anomalies[:60]


def analyze_bundle(bundle, settings):
    bundle = Path(bundle)
    bundle, manifest, samples, recordings = load_bundle(bundle)
    if not recordings:
        print(f"[错误] 数据包里没有点云: {bundle}", file=sys.stderr)
        return None
    print(f"[分析] 数据包 {bundle}")
    print(f"[分析] 时间 {manifest.get('created', '?')} · 时长 "
          f"{float(manifest.get('duration_s', 0.0)):.1f} s · 话题 {len(recordings)} 路")
    map_frame = manifest.get("frames", {}).get("map", "map")
    if settings.analysis_frame == map_frame and \
            not np.isfinite(samples.get("yaw_map_odom_deg", np.zeros(0))).any():
        print(f"[警告] 要按 {map_frame} 分析，但采集期间没收到 {map_frame}->odom；"
              "点云仍按 odom 分析。", file=sys.stderr)

    results = []
    events = []
    for topic, recording in recordings.items():
        print(f"[分析] {topic} ...", end="", flush=True)
        try:
            result = analyze_topic(topic, recording, samples, settings)
        except Exception as exc:  # noqa: BLE001
            print(f" 失败: {describe_exception(exc)}")
            events.append(f"{topic}: 分析失败 {describe_exception(exc)}")
            continue
        results.append(result)
        print(f" 斜率 {result.slope:+.2f} r={result.corr:+.2f} n={result.n_pairs}"
              f" -> {result.verdict}")

    analysis = Analysis(bundle=bundle, settings=settings, manifest=manifest,
                        samples=samples, recordings=recordings, results=results,
                        events=events, anomalies=detect_anomalies(samples, settings))
    write_pair_csv(bundle / "analysis" / "pairs.csv", results)
    write_analysis_json(bundle / "analysis" / "analysis.json", analysis)

    if not settings.no_plot:
        try:
            plot_analysis(bundle, analysis)
        except Exception as exc:  # noqa: BLE001
            print(f"[警告] 出图失败: {describe_exception(exc)}")
            events.append(f"出图失败: {describe_exception(exc)}")

    report = render_report(analysis)
    (bundle / "report.md").write_text(report, encoding="utf-8")
    print_summary(analysis)
    print(f"[输出] 报告: {bundle / 'report.md'}")
    if not settings.no_plot:
        print(f"[输出] 总图: {bundle / 'analysis' / 'plots.png'}")
    return analysis


def write_pair_csv(path, results):
    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(PAIR_FIELDS)
        for result in results:
            for pair in result.pairs:
                writer.writerow([
                    result.topic, pair.prev_idx, pair.idx,
                    f"{pair.prev_stamp:.6f}", f"{pair.stamp:.6f}", f"{pair.dt:.4f}",
                    f"{pair.dtheta_gimbal:.3f}", f"{pair.dtheta_base:.3f}",
                    f"{pair.dyaw_map_odom:.3f}", f"{pair.psi_align:.3f}",
                    f"{pair.dpsi_cloud:.3f}", f"{pair.shift:.4f}",
                    f"{pair.inlier:.3f}", f"{pair.rmse:.4f}",
                    f"{pair.anchor_yaw:.3f}", f"{pair.anchor_inlier:.3f}",
                    f"{pair.anchor_shift:.4f}",
                ])


def _json_float(value):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return value if np.isfinite(value) else None


def write_analysis_json(path, analysis):
    payload = {
        "bundle": str(analysis.bundle),
        "analysis_frame": analysis.settings.analysis_frame,
        "created": datetime.now().isoformat(timespec="seconds"),
        "topics": [],
        "events": analysis.events,
        "anomalies": analysis.anomalies,
    }
    for result in analysis.results:
        payload["topics"].append({
            "topic": result.topic,
            "frame_id": result.frame_id,
            "frames": result.frames,
            "valid": result.valid,
            "travel_deg": _json_float(result.travel_deg),
            "gimbal_range_deg": [_json_float(result.gimbal_min),
                                 _json_float(result.gimbal_max)],
            "pairs": result.n_pairs,
            "slope": _json_float(result.slope),
            "corr": _json_float(result.corr),
            "slope_anchor": _json_float(result.slope_anchor),
            "corr_anchor": _json_float(result.corr_anchor),
            "lag_s": _json_float(result.lag),
            "lag_corr": _json_float(result.lag_corr),
            "lag_slope": _json_float(result.lag_slope),
            "median_inlier": _json_float(result.median_inlier),
            "median_shift_m": _json_float(result.median_shift),
            "median_points": result.median_points,
            "angle_source": result.angle_source,
            "kind": result.kind,
            "orbit_radius_m": _json_float(result.orbit_radius),
            "orbit_residual_m": _json_float(result.orbit_residual),
            "base_orbit_radius_m": _json_float(result.base_orbit_radius),
            "notes": result.notes,
        })
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")


# =============================================================
# 出图
# =============================================================
def _setup_pyplot():
    sys.path.insert(0, str(DEBUG_DIR))
    from _common import prepare_matplotlib_env, setup_cjk_font

    prepare_matplotlib_env()
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    setup_cjk_font()
    return plt


def _outside_legend(ax, ncol=2, fontsize=7.0):
    """图例放在坐标区上方，并按图例行数把标题顶上去，避免压在一起。"""
    handles, labels = ax.get_legend_handles_labels()
    if not handles:
        return
    rows = max(1, math.ceil(len(labels) / max(ncol, 1)))
    # ⚠ 必须显式取 loc="left"：get_title() 默认取的是 center 那份，
    # 传给 set_title(loc="left") 等于把左标题清空（踩过一次）。
    ax.set_title(ax.get_title(loc="left"), loc="left", pad=16 + 11 * rows)
    ax.legend(loc="lower left", bbox_to_anchor=(0.0, 1.0), ncol=ncol,
              fontsize=fontsize, frameon=False, borderaxespad=0.2)


def plot_analysis(bundle, analysis):
    plt = _setup_pyplot()
    settings = analysis.settings
    samples = analysis.samples
    bundle = Path(bundle)
    times = samples.get("t_stamp", np.zeros(0))
    if len(times) < 2:
        return
    # 就绪门期间已经开始收点云，云的时间戳可能早于第一个标量采样；取两者最小值当零点，
    # 否则 ② 的横轴会从负数开始。
    candidates = [float(times[np.isfinite(times)][0])]
    for recording in analysis.recordings.values():
        if len(recording.stamps):
            candidates.append(float(np.nanmin(recording.stamps)))
    t0 = min(candidates)
    x = times - t0
    colors = plt.rcParams["axes.prop_cycle"].by_key()["color"]

    figure, axes = plt.subplots(3, 2, figsize=(15.0, 11.8))
    figure.subplots_adjust(left=0.07, right=0.97, top=0.92, bottom=0.13,
                           hspace=0.52, wspace=0.19)

    # ① 云台角
    ax = axes[0][0]
    joint = samples.get("joint_deg", np.full_like(times, np.nan))
    gimbal = samples.get("gimbal_phys_deg", np.full_like(times, np.nan))
    base_yaw = samples.get("yaw_odom_base_deg", np.full_like(times, np.nan))
    ax.plot(x, gimbal, lw=1.7, color="tab:blue",
            label="TF 物理角")
    ax.plot(x, -joint, lw=1.0, ls="--", color="tab:orange",
            label="-gimbal_yaw_joint")
    ax.plot(x, base_yaw, lw=1.1, color="tab:green", label="底盘 yaw")
    ax.set_title("① 云台角（判据用物理角）", loc="left")
    ax.set_ylabel("度")
    ax.margins(y=0.12)
    ax.grid(alpha=0.3)
    _outside_legend(ax, ncol=3)

    # ② 每路点云的世界系转角（锚点对齐）
    ax = axes[0][1]
    for color, result in zip(colors, analysis.results):
        points = [(pair.stamp - t0, pair.anchor_yaw) for pair in result.pairs
                  if np.isfinite(pair.anchor_yaw)]
        if not points:
            continue
        xs, ys = zip(*points)
        ax.plot(xs, ys, marker=".", ms=3, lw=1.2, color=color, label=result.topic)
    ax.set_title("② 点云在分析系里的转角（锚点对齐，0 = 稳定）", loc="left")
    ax.set_xlabel("时间（s）")
    ax.set_ylabel("度")
    ax.grid(alpha=0.3)
    _outside_legend(ax, ncol=2)

    # ③ Δψ vs Δθ
    ax = axes[1][0]
    for color, result in zip(colors, analysis.results):
        good = [pair for pair in result.pairs
                if np.isfinite(pair.dtheta_gimbal) and np.isfinite(pair.dpsi_cloud)
                and pair.inlier >= settings.min_inlier]
        if not good:
            continue
        xs = np.array([pair.dtheta_gimbal for pair in good])
        ys = np.array([pair.dpsi_cloud for pair in good])
        ax.scatter(xs, ys, s=7, alpha=0.55, color=color,
                   label=f"{result.topic}  s={result.slope:+.2f}")
        if np.isfinite(result.slope):
            span = np.array([xs.min(), xs.max()])
            ax.plot(span, result.slope * span, lw=1.0, color=color)
    ax.axhline(0.0, color="0.5", lw=0.8)
    ax.set_title("③ 相邻帧：点云世界系转角 vs 云台角增量", loc="left")
    ax.set_xlabel("Δ云台角（度）")
    ax.set_ylabel("Δ点云世界系转角（度）")
    ax.grid(alpha=0.3)
    _outside_legend(ax, ncol=2)

    # ④ 底盘运动
    ax = axes[1][1]
    speed = np.hypot(samples.get("base_vx", np.zeros_like(times)),
                     samples.get("base_vy", np.zeros_like(times)))
    ax.plot(x, speed, lw=1.2, color="tab:red", label="|v|（/odometry twist）")
    ax.plot(x, samples.get("base_wz", np.zeros_like(times)), lw=1.0,
            color="tab:purple", label="wz")
    ax.set_title("④ 底盘运动（应接近 0）", loc="left")
    ax.set_xlabel("时间（s）")
    ax.set_ylabel("m/s, rad/s")
    ax.set_ylim(min(-0.1, float(np.nanmin(speed)) - 0.05),
                max(0.1, float(np.nanmax(speed)) + 0.05))
    ax.axhline(0.0, color="0.6", lw=0.7)
    ax.grid(alpha=0.3)
    _outside_legend(ax, ncol=2)

    # ⑤ TF 链分解
    ax = axes[2][0]
    for name, color in (("yaw_map_odom_deg", "tab:brown"),
                        ("yaw_odom_base_deg", "tab:green"),
                        ("yaw_base_gimbal_deg", "tab:blue"),
                        ("yaw_gimbal_lidar_deg", "tab:gray")):
        values = samples.get(name, np.full_like(times, np.nan))
        if np.isfinite(values).sum() < 2:
            continue
        ax.plot(x, values, lw=1.1, color=color, label=name.replace("_deg", ""))
    ax.set_title("⑤ TF 各段 yaw（云台角落在哪一段）", loc="left")
    ax.set_xlabel("时间（s）")
    ax.set_ylabel("度")
    ax.margins(y=0.12)
    ax.grid(alpha=0.3)
    _outside_legend(ax, ncol=2)

    # ⑥ 每帧点数
    ax = axes[2][1]
    names, medians = [], []
    for result in analysis.results:
        names.append(textwrap.shorten(result.topic, 30, placeholder="…"))
        medians.append(result.median_points)
    if names:
        ax.barh(range(len(names)), medians, color="tab:cyan")
        ax.set_yticks(range(len(names)))
        ax.set_yticklabels(names, fontsize=7.5)
        ax.invert_yaxis()
    ax.set_title("⑥ 每帧点数（中位数）", loc="left")
    ax.set_xlabel("点/帧")
    ax.grid(alpha=0.3, axis="x")

    caption = (
        "判读：斜率 s = Δ(点云分析系转角) / Δ(云台角)。s≈0 点云在世界系稳定；"
        "s≈+1 云台角被同向加进点云；s≈-1 该减掉的云台角没减/符号反；s≈±2 双计。"
        "③ 的每条线是过原点增量回归，与 capture_gimbal_spin.py 同口径。"
        "① 里 TF 口径的物理角才是判据，/joint_states 的符号取决于 URDF 轴方向。"
    )
    figure.text(0.01, 0.012, textwrap.fill(caption, 150), fontsize=8.5, va="bottom")
    figure.suptitle("云台转动 × 点云世界系稳定性", x=0.01, ha="left", fontsize=13, y=0.985)
    figure.savefig(bundle / "analysis" / "plots.png", dpi=140)
    plt.close(figure)

    plot_overlay(bundle, analysis, plt)


def plot_overlay(bundle, analysis, plt):
    primary = _primary_result(analysis)
    if primary is None:
        return
    recording = analysis.recordings.get(primary.topic)
    if recording is None:
        return
    settings = analysis.settings
    valid = [index for index in range(len(recording))
             if recording.tf_ok[index] and np.isfinite(recording.gimbal_deg[index])
             and len(recording.points[index]) >= settings.min_points]
    if len(valid) < 4:
        return
    angles = np.array([recording.gimbal_deg[index] for index in valid])
    low = valid[int(np.argmin(angles))]
    high = valid[int(np.argmax(angles))]
    if abs(recording.gimbal_deg[high] - recording.gimbal_deg[low]) < 5.0:
        return

    def to_world(index):
        matrix = recording.T_odom_frame[index]
        if settings.analysis_frame == settings.map_frame:
            matrix = recording.T_map_odom[index] @ matrix
        return transform_points(np.asarray(recording.points[index], dtype=np.float64),
                                matrix)

    low_points = to_world(low)
    high_points = to_world(high)
    figure, ax = plt.subplots(figsize=(9.5, 7.0))
    figure.subplots_adjust(left=0.08, right=0.98, top=0.9, bottom=0.14)
    ax.scatter(low_points[:, 0], low_points[:, 1], s=1.2, color="tab:blue", alpha=0.65,
               label=f"云台 {recording.gimbal_deg[low]:+.1f}°（第 {low} 帧）")
    ax.scatter(high_points[:, 0], high_points[:, 1], s=1.2, color="tab:red", alpha=0.65,
               label=f"云台 {recording.gimbal_deg[high]:+.1f}°（第 {high} 帧）")
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_title(f"{primary.topic}：云台两端角度的点云在 {settings.analysis_frame} 里是否重合",
                 loc="left")
    ax.set_xlabel(f"{settings.analysis_frame} x（m）")
    ax.set_ylabel(f"{settings.analysis_frame} y（m）")
    ax.grid(alpha=0.25)
    ax.legend(loc="upper right", fontsize=8.5)
    ax.text(0.01, 0.01,
            "两层墙/两张图 = 点云跟着云台转；完全重合 = 点云在世界系稳定。",
            transform=ax.transAxes, fontsize=8.5, va="bottom")
    figure.savefig(bundle / "analysis" / "cloud_overlay.png", dpi=140)
    plt.close(figure)


def _primary_result(analysis):
    if not analysis.results:
        return None
    decisive = [item for item in analysis.results
                if item.kind not in ("insufficient", "unclear")]
    pool = decisive or analysis.results
    def priority(item):
        if item.kind in ("follow_pos", "follow_neg", "follow_2x"):
            return 3
        if item.kind == "translation_orbit":
            return 2
        if item.kind == "partial":
            return 1
        return 0

    return max(
        pool,
        key=lambda item: (priority(item),
                          abs(item.slope) if np.isfinite(item.slope) else 0.0,
                          item.n_pairs))


# =============================================================
# 报告
# =============================================================
def _fmt(value, digits=2, unit=""):
    if value is None or not np.isfinite(value):
        return "—"
    return f"{value:+.{digits}f}{unit}"


def _fmt_lag(value):
    if value is None or not np.isfinite(value):
        return "—"
    if abs(value) < 0.005:
        return "0.00 s"
    return f"{value:+.2f} s"


def render_report(analysis):
    manifest = analysis.manifest
    settings = analysis.settings
    results = analysis.results
    primary = _primary_result(analysis)
    lines = []
    lines.append("# 云台转动 × 点云世界系稳定性 —— 抓包报告")
    lines.append("")
    lines.append(f"- 数据包：`{analysis.bundle}`")
    lines.append(f"- 采集：{manifest.get('created', '?')}，时长 "
                 f"{float(manifest.get('duration_s', 0.0)):.1f} s，"
                 f"{manifest.get('host', '?')}，commit "
                 f"`{str(manifest.get('git', {}).get('commit', ''))[:10]}`")
    lines.append(f"- 分析系：`{settings.analysis_frame}`；判据：相邻帧二维配准的过原点增量回归")
    note = str(manifest.get("note") or "").strip()
    if note:
        lines.append(f"- 采集备注：{note}")
    lines.append("")

    lines.append("## 结论")
    lines.append("")
    if primary is not None and primary.kind != "insufficient":
        lines.append(f"最主要的一路是 **`{primary.topic}`**（frame=`{primary.frame_id}`）："
                     f"斜率 **{_fmt(primary.slope)}**，相关 r={_fmt(primary.corr)}，"
                     f"云台行程 {primary.travel_deg:.1f}°。")
        lines.append("")
        lines.append(f"> {primary.verdict}")
        lines.append("")
        for note in primary.notes:
            lines.append(f"- {note}")
        if primary.notes:
            lines.append("")
    elif not results:
        lines.append("没有可用话题。")
        lines.append("")
    else:
        lines.append("**本次数据不足以判定**：云台行程不足或有效配准对太少。"
                     "请重录一次，转云台 ±30~45°，转过去再转回来。")
        lines.append("")
    if analysis.anomalies:
        biggest = max(analysis.anomalies, key=lambda item: abs(item["delta_deg"]))
        lines.append(f"> ⚠ 采集窗口内有 **{len(analysis.anomalies)} 个坐标/位姿跳变**，最大 "
                     f"{biggest['delta_deg']:+.2f}°（t={biggest['t']:.2f} s）。在 RViz "
                     "固定帧 map 下，整片点云会按这些台阶整体摆动；明细见 §2.1。")
        lines.append("")
    conflicts = manifest.get("capture", {}).get("tf_static_dynamic_conflicts") or []
    if conflicts:
        pairs = "、".join(f"`{item['parent']}→{item['child']}`" for item in conflicts)
        lines.append(f"> ⚠ **TF 冲突**：{pairs} 同时有 static 和 dynamic 发布者。"
                     "这会让 map/odom 在两个值之间来回跳，点云整片摆动；"
                     "先按 §2 的发布者列表停掉多余的那一套进程。")
        lines.append("")

    lines.append("## 一、按话题的判读")
    lines.append("")
    lines.append("| 话题 | frame | 有效帧 | 云台行程 | 斜率 s | 相关 r | 锚点斜率 | 平移绕圈 | 最佳滞后 | 判读 |")
    lines.append("| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |")
    for result in results:
        lag_text = _fmt_lag(result.lag)
        if np.isfinite(result.orbit_radius):
            orbit_text = (f"R={result.orbit_radius * 100:.1f} cm"
                          f"（残差 {result.orbit_residual * 100:.1f}）")
        else:
            orbit_text = "—"
        lines.append(
            f"| `{result.topic}` | `{result.frame_id}` | {result.valid}/{result.frames} | "
            f"{result.travel_deg:.1f}° | {_fmt(result.slope)} | {_fmt(result.corr)} | "
            f"{_fmt(result.slope_anchor)} | {orbit_text} | {lag_text} | {result.verdict} |")
    lines.append("")

    lines.append("## 二、采集条件（判据成不成立）")
    lines.append("")
    samples = analysis.samples
    times_all = samples.get("t_stamp", np.zeros(0))
    finite_times = times_all[np.isfinite(times_all)]
    t0 = float(finite_times[0]) if len(finite_times) else 0.0
    gimbal = samples.get("gimbal_phys_deg", np.zeros(0))
    finite = gimbal[np.isfinite(gimbal)]
    if len(finite):
        lines.append(f"- 云台：{finite.min():+.1f}° ~ {finite.max():+.1f}°"
                     f"（行程 {finite.max() - finite.min():.1f}°）")
    if len(times_all) > 2:
        raw_steps = np.diff(times_all)
        deltas = np.abs(np.diff(gimbal))
        rate_mask = (np.isfinite(raw_steps) & (raw_steps > 0) & np.isfinite(deltas))
        rates = (deltas[rate_mask] / raw_steps[rate_mask]) if rate_mask.any() else np.zeros(0)
        if len(rates):
            moving = float(np.mean(rates > 5.0))
            lines.append(f"- 云台角速度：中位 {np.median(rates):.1f}°/s，p95 "
                         f"{np.percentile(rates, 95):.1f}°/s，峰值 {rates.max():.1f}°/s；"
                         f"运动时段占 {moving * 100:.0f}%")
            if rates.max() > 120.0:
                lines.append(f"- ⚠ 转得偏快（峰值 {rates.max():.0f}°/s）：建议 15~20°/s。"
                             "3 个来回就够，不用刻意加时长；转太快会让 TF/时间同步的尖峰变多")
    base_x = samples.get("base_x", np.zeros(0))
    base_y = samples.get("base_y", np.zeros(0))
    if len(base_x) and np.isfinite(base_x).sum() > 2:
        finite_base = np.isfinite(base_x) & np.isfinite(base_y)
        dx = base_x[finite_base] - base_x[finite_base][0]
        dy = base_y[finite_base] - base_y[finite_base][0]
        travel = float(np.max(np.hypot(dx, dy)))
        lines.append(f"- 底盘最大位移（odom->base 相对首帧）：{travel:.3f} m "
                     + ("（静止，判据成立）" if travel < 0.05
                        else "（**底盘在动**，斜率会被底盘运动污染）"))
    base_yaw = samples.get("yaw_odom_base_deg", np.zeros(0))
    finite_base_yaw = base_yaw[np.isfinite(base_yaw)]
    if len(finite_base_yaw):
        base_yaw_delta = float(finite_base_yaw.max() - finite_base_yaw.min())
        lines.append(f"- 底盘 yaw（odom->base）：{finite_base_yaw.min():+.2f}° ~ "
                     f"{finite_base_yaw.max():+.2f}°，变化 {base_yaw_delta:.2f}°"
                     + ("（>1° 说明云台转动时车体航向估计在动；"
                        "点云相对车体会跟着摆这个角度）" if base_yaw_delta > 1.0 else ""))
    map_odom = samples.get("yaw_map_odom_deg", np.zeros(0))
    finite_map = map_odom[np.isfinite(map_odom)]
    if len(finite_map):
        text = (f"- map->odom yaw：{finite_map.min():+.2f}° ~ {finite_map.max():+.2f}°，"
                f"变化 {finite_map.max() - finite_map.min():.2f}°")
        map_steps = [item for item in analysis.anomalies
                     if item["kind"] == "yaw_map_odom_deg"]
        if map_steps:
            biggest = max(map_steps, key=lambda item: abs(item["delta_deg"]))
            text += (f"；最大单步 {biggest['delta_deg']:+.2f}° @ t={biggest['t']:.2f} s")
        lines.append(text)
    tf_diff = samples.get("yaw_odom_lidar_tf_deg", np.zeros(0)) - \
        samples.get("yaw_odom_lidar_odom_deg", np.zeros(0))
    tf_diff = wrap_deg_array(tf_diff)
    finite_diff = np.abs(tf_diff[np.isfinite(tf_diff)])
    if len(finite_diff):
        index_max = int(np.nanargmax(np.abs(tf_diff)))
        lines.append(f"- TF `odom->lidar` 与 `/lidar_odometry`：中位 "
                     f"{np.median(finite_diff):.3f}°，p95 {np.percentile(finite_diff, 95):.3f}°，"
                     f"最大 {finite_diff.max():.2f}° @ t={times_all[index_max] - t0:.2f} s"
                     "（单帧尖峰多半是快速转动时 TF/关节不同步）")
    duplicates = manifest.get("capture", {}).get("tf_duplicate_parents") or {}
    if duplicates:
        lines.append("- ⚠ TF 里同一 child 有多个 parent：" +
                     "; ".join(f"`{child}` <- {parents}"
                               for child, parents in duplicates.items()))
    else:
        lines.append("- TF：本窗口内没有发现同一 child 有多个 parent")
    conflicts = manifest.get("capture", {}).get("tf_static_dynamic_conflicts") or []
    if conflicts:
        lines.append("- ⚠ **同一对 TF 同时有 static 和 dynamic 发布者**：" +
                     "; ".join(f"`{item['parent']}→{item['child']}`"
                               for item in conflicts) +
                     "（典型：上次启动残留的 `static_transform_publisher_map2odom`、"
                     " 没退干净的 `backend_node`）。RViz 会在两个值之间跳")
    static_map_odom = manifest.get("capture", {}).get("tf_static_map_odom")
    if static_map_odom:
        lines.append(f"- 静态 map->odom：x={static_map_odom['x']:+.3f} m, "
                     f"y={static_map_odom['y']:+.3f} m, "
                     f"yaw={static_map_odom['yaw_deg']:+.2f}°")
    publishers = manifest.get("capture", {}).get("tf_publishers") or {}
    if publishers:
        lines.append("- TF 发布者：/tf [" +
                     ", ".join(publishers.get("/tf", [])) + "]；/tf_static [" +
                     ", ".join(publishers.get("/tf_static", [])) + "]")
    bag = manifest.get("capture", {}).get("bag")
    if bag:
        lines.append(f"- 原始 rosbag：`bag/{bag}`（要看未下采样点云/复算配准时用它）")
    lines.append("")
    lines.append("### 2.1 异常事件（坐标/位姿跳变）")
    lines.append("")
    if analysis.anomalies:
        lines.append(f"阈值 `--anomaly-deg {settings.anomaly_deg:g}°`。这些台阶在 RViz "
                     "固定帧 map 下会表现为**整片点云整体摆动**，不要误判成点云内容在转。")
        lines.append("")
        lines.append("| 时间 (s) | 类型 | 幅度 | 说明 |")
        lines.append("| ---: | --- | ---: | --- |")
        for anomaly in analysis.anomalies[:15]:
            kind = anomaly["kind"]
            if kind == "yaw_map_odom_deg":
                detail = "map->odom 跳变（backend/TF 发布源）；map 系点云整体转一下"
            elif kind == "yaw_odom_base_deg":
                detail = "底盘 yaw 在 odom 里跳；点云相对车体摆一下"
            else:
                detail = "raw Livox 用 TF 链显示时会跳；registered_scan 不受影响"
            lines.append(f"| {anomaly['t']:.2f} | {anomaly['label']} | "
                         f"{anomaly['delta_deg']:+.2f}° | {detail} |")
        lines.append("")
    else:
        lines.append(f"无（阈值 {settings.anomaly_deg:g}°）。")
        lines.append("")

    lines.append("## 三、斜率怎么翻译成故障层")
    lines.append("")
    lines.append("| 斜率 | 含义 | 点云 frame 是 odom/世界系时，优先查 | 点云 frame 是车体系/云台系时 |")
    lines.append("| --- | --- | --- | --- |")
    lines.append("| ≈ 0 | 世界系稳定 | 无需修；你看的可能是另一路点云或 RViz Fixed Frame | TF 链在正常补偿云台角 |")
    lines.append("| ≈ +1 | 云台角被同向加了一次 | 点云变换的位姿里**多**了一份云台角 | 不该被 TF 再转一次，查静态外参/双 parent |")
    lines.append("| ≈ -1 | 该减的云台角没减/符号反 | 去旋转漏了、`gimbal_joint_sign` 反了、点云其实是车体系却标成 odom | 查 TF 符号与 joint 轴向 |")
    lines.append("| ≈ ±2 | 同一个角被算了两次 | 既在点云里去旋转、又在 TF 里补了一次 | 双计 |")
    lines.append("")
    lines.append("按本仓库的链路，重点位置：")
    lines.append("")
    lines.append("- `src/localization/backend/srm27_localization_adapter/scripts/zlio_bridge.py`："
                 "`_publish()` 用 `cloud_pose` 把前端点云左乘到 odom；`T_off` 是启动时抓的安装外参。")
    lines.append("- `src/localization/backend/sensor_scan_generation/src/sensor_scan_generation.cpp`："
                 "`sensor_scan` 的 odom 表达与 `odom->lidar` 的取法。")
    lines.append("- `src/localization/frontend/frontend/src/preparation.hpp`："
                 "前端输出点云到底在 body 还是 world 系（`FrontendFrame.cloud` 的契约）。")
    lines.append("- `src/planner/frontend/terrain_analysis*/src/*.cpp`："
                 "地形图在 odom 里是绝对坐标，若输入点云在转，这里会留下旋转拖影。")
    lines.append("")

    lines.append("## 四、每路点云的证据")
    lines.append("")
    for result in results:
        lines.append(f"### `{result.topic}`")
        lines.append("")
        lines.append(f"- frame=`{result.frame_id}`，有效帧 {result.valid}/{result.frames}，"
                     f"每帧中位 {result.median_points} 点")
        lines.append(f"- 云台角来源：{result.angle_source}")
        lines.append(f"- 相邻帧对齐：n={result.n_pairs}，斜率 {_fmt(result.slope)}，"
                     f"相关 {_fmt(result.corr)}，中位内点率 {_fmt(result.median_inlier)}，"
                     f"中位平移 {_fmt(result.median_shift, 3)} m")
        lines.append(f"- 锚点对齐：斜率 {_fmt(result.slope_anchor)}，"
                     f"相关 {_fmt(result.corr_anchor)}")
        if np.isfinite(result.orbit_radius):
            lines.append(f"- 锚点平移绕圈：R={result.orbit_radius * 100:.1f} cm"
                         f"（残差 {result.orbit_residual * 100:.1f} cm；车体位姿同法 "
                         f"R={result.base_orbit_radius * 100:.1f} cm）")
        if np.isfinite(result.lag):
            lines.append(f"- 滞后扫描：最佳 {_fmt_lag(result.lag)}"
                         f"（|r|={_fmt(abs(result.lag_corr))}，"
                         f"该滞后下斜率 {_fmt(result.lag_slope)}）")
        lines.append(f"- 判读：**{result.verdict}**")
        for note in result.notes:
            lines.append(f"  - {note}")
        lines.append("")

    lines.append("## 五、文件清单与数据字典")
    lines.append("")
    lines.append("| 文件 | 内容 |")
    lines.append("| --- | --- |")
    lines.append("| `manifest.json` | 话题、帧、git 版本、TF 父子关系、节点参数、采集设置 |")
    lines.append("| `samples.csv` | 20 Hz 标量：云台角三路、TF 各段 yaw、底盘位姿/速度 |")
    lines.append("| `raw/<话题>.npz` | 每帧点云（发布帧坐标）+ 逐帧 TF 矩阵 + 逐帧云台角 |")
    lines.append("| `clouds/*.pcd` | 关键帧，已变换到分析系，可直接 CloudCompare 打开 |")
    lines.append("| `clouds/index.csv` | 每个 PCD 对应第几帧、云台多少度 |")
    lines.append("| `analysis/pairs.csv` | 每对相邻帧的 Δψ/Δθ、内点率、平移、锚点结果 |")
    lines.append("| `analysis/analysis.json` | 上面所有关键数字的机器可读版本 |")
    lines.append("| `analysis/plots.png` | 六面板总图 |")
    lines.append("| `analysis/cloud_overlay.png` | 云台两端角度的点云叠加 |")
    lines.append("")
    lines.append("`samples.csv` 列含义：")
    lines.append("")
    lines.append("| 列 | 含义 |")
    lines.append("| --- | --- |")
    lines.append("| `t_stamp` / `t_wall` | ROS 时间戳（秒） |")
    lines.append("| `joint_deg` | `/joint_states` 的 `gimbal_yaw_joint`（URDF 关节值，轴向为 -z 时与物理角反号） |")
    lines.append("| `c2g_deg` | `/chassis_to_gimbal_yaw_deg`（底盘相对云台） |")
    lines.append("| `gimbal_phys_deg` | **判据用的物理云台角** = yaw(odom->gimbal) − yaw(odom->base) |")
    lines.append("| `yaw_map_odom_deg` | map->odom 的 yaw（backend/静态 TF 的纠正面） |")
    lines.append("| `yaw_odom_base_deg` | 底盘在 odom 下的 yaw |")
    lines.append("| `yaw_base_gimbal_deg` | 云台关节的 TF yaw |")
    lines.append("| `yaw_gimbal_lidar_deg` | 云台到雷达的安装 yaw |")
    lines.append("| `yaw_odom_lidar_tf_deg` | TF 直查 odom->lidar 的 yaw |")
    lines.append("| `yaw_odom_lidar_odom_deg` | `/lidar_odometry` 位姿的 yaw（应与上一列一致） |")
    lines.append("| `yaw_odom_nav_deg` | `/odometry`（导航用，child=gimbal_yaw）的 yaw |")
    lines.append("| `base_x` / `base_y` | 底盘在 odom 下的位置 |")
    lines.append("| `base_vx/vy/wz` | `/odometry.twist` |")
    lines.append("| `tf_ok` | 该采样时刻底盘 TF 是否查到 |")
    lines.append("")

    lines.append("## 六、复现与复算")
    lines.append("")
    lines.append("```bash")
    lines.append("# 1) 采集（车端，导航已在跑）")
    lines.append("./debug_tools/capture_gimbal_cloud_rotation.py --duration 60")
    lines.append("")
    lines.append("# 2) 换参数重算（不需要 ROS，只需要 numpy/scipy/matplotlib）")
    lines.append(f"./debug_tools/capture_gimbal_cloud_rotation.py --analyze {analysis.bundle} "
                 "--yaw-range 90 --no-plot")
    lines.append("")
    lines.append("# 3) 自检（造 4 路已知答案的点云，验证配准/斜率/判读整条链）")
    lines.append("./debug_tools/capture_gimbal_cloud_rotation.py --demo")
    lines.append("```")
    lines.append("")
    lines.append("配准参数（`--align-points`/`--inlier-dist`/`--yaw-range`/`--coarse-step`）"
                 "只在 `analysis/` 里生效，原始数据不变；稀疏场景可把 `--inlier-dist` 放宽到 0.2，"
                 "转动幅度大时把 `--yaw-range` 放到 90。")
    lines.append("")

    lines.append("## 七、限制（先看这里再下结论）")
    lines.append("")
    lines.append("- 配准是**二维**的（yaw + xy），roll/pitch 不进入斜率；"
                 "本问题（云台 yaw）正好是二维。")
    lines.append("- 配准依赖场景几何：正对单面墙/空旷场地时内点率会低，斜率要打折。"
                 "中位内点率低于 0.55 时会在上面标注。")
    lines.append("- 逐帧增量回归把每对相邻帧当独立样本；同时给了锚点斜率做交叉验证，两者应一致。")
    lines.append("- 工具启动前的那段历史补不回来；要覆盖完整动作请先开工具再转云台。")
    lines.append("- 点云按 `header.stamp` 查 TF；超过 `--tf-cache` 的帧会被跳过（manifest 里有计数）。")
    lines.append("")
    try:
        relative = analysis.bundle.relative_to(REPO_ROOT)
    except ValueError:
        relative = analysis.bundle
    lines.append(f"*报告由 `debug_tools/capture_gimbal_cloud_rotation.py` 生成；"
                 f"数据包：`{relative}`*")
    return "\n".join(lines) + "\n"


def print_summary(analysis):
    print("")
    print("=" * 68)
    print(" 判定汇总（斜率 s = Δ点云分析系转角 / Δ云台角）")
    print("=" * 68)
    for result in analysis.results:
        print(f"  {result.topic}")
        print(f"    frame={result.frame_id}  有效 {result.valid}/{result.frames}  "
              f"行程 {result.travel_deg:.1f}°  n={result.n_pairs}")
        print(f"    斜率 {_fmt(result.slope)}  r={_fmt(result.corr)}  "
              f"锚点斜率 {_fmt(result.slope_anchor)}  内点率 {_fmt(result.median_inlier)}")
        if np.isfinite(result.orbit_radius):
            print(f"    平移绕圈 R={result.orbit_radius * 100:.1f} cm"
                  f"（残差 {result.orbit_residual * 100:.1f} cm；车体同法 R="
                  f"{result.base_orbit_radius * 100:.1f} cm）")
        print(f"    -> {result.verdict}")
        for note in result.notes:
            print(f"       · {note}")
    primary = _primary_result(analysis)
    if primary is not None:
        print("-" * 68)
        if primary.kind == "insufficient":
            print("  [结论] 数据不足：云台行程不够或有效帧太少，重录一次。")
        else:
            print(f"  [结论] {primary.topic}: {primary.verdict}")
            print(f"         斜率 {_fmt(primary.slope)}，相关 {_fmt(primary.corr)}，"
                  f"云台行程 {primary.travel_deg:.1f}°")
    if analysis.anomalies:
        biggest = max(analysis.anomalies, key=lambda item: abs(item["delta_deg"]))
        print(f"  [异常] {len(analysis.anomalies)} 个坐标/位姿跳变，最大 "
              f"{biggest['delta_deg']:+.2f}° @ t={biggest['t']:.2f} s"
              "（map 固定帧下会整片点云摆一下；见 report §2.1）")
    conflicts = analysis.manifest.get("capture", {}).get(
        "tf_static_dynamic_conflicts") or []
    if conflicts:
        print("  [TF冲突] " + ", ".join(f"{item['parent']}->{item['child']}"
                                         for item in conflicts) +
              " 同时有 static/dynamic 发布者；先停掉多余的那套进程")
    print("=" * 68)
    print("")


# =============================================================
# 演示数据（不连 ROS）
# =============================================================
DEMO_LIDAR_XY = np.array([0.12, -0.057])
DEMO_LIDAR_Z = 0.346
DEMO_MOUNT_YAW_DEG = 63.0


def build_demo(settings):
    """造 4 路已知答案的点云：正确 / 车体系 / 漏补 / 双计。"""
    rng = np.random.default_rng(20261123)
    scene = _demo_scene(rng)
    mount_yaw = math.radians(DEMO_MOUNT_YAW_DEG)
    lidar_world = np.array([DEMO_LIDAR_XY[0], DEMO_LIDAR_XY[1], DEMO_LIDAR_Z])

    duration = 24.0
    times = np.arange(0.0, duration, 1.0 / 20.0)
    theta = 40.0 * np.sin(2.0 * math.pi * times / 8.0)

    def base_yaw(t):
        return 0.4 * math.sin(2.0 * math.pi * t / 37.0)

    rows = []
    for t, th in zip(times, theta):
        by = base_yaw(t)
        rows.append({
            "t_stamp": t + 1000.0, "t_wall": t,
            "joint_deg": -th, "c2g_deg": th, "gimbal_phys_deg": th,
            "yaw_map_odom_deg": -1.4, "yaw_odom_base_deg": by,
            "yaw_base_gimbal_deg": by - th, "yaw_gimbal_lidar_deg": 63.0,
            "yaw_odom_lidar_tf_deg": wrap_deg(by - th + 63.0),
            "yaw_odom_lidar_odom_deg": wrap_deg(by - th + 63.0),
            "yaw_odom_nav_deg": wrap_deg(by - th),
            "base_x": 0.01 * math.sin(t), "base_y": 0.01 * math.cos(t),
            "base_vx": 0.0, "base_vy": 0.0, "base_wz": 0.0, "tf_ok": 1.0,
        })
    samples = samples_to_arrays(rows)

    cloud_times = np.arange(0.0, duration, 1.0 / 5.0)
    topics = {
        "/demo/cloud_world_correct": {"frame": "odom", "kind": "correct"},
        "/demo/cloud_in_lidar_frame": {"frame": "livox_frame", "kind": "body"},
        "/demo/cloud_missing_derotate": {"frame": "odom", "kind": "missing"},
        "/demo/cloud_double_rotate": {"frame": "odom", "kind": "double"},
    }
    recordings = {}
    for topic, spec in topics.items():
        stamps, frame_ids, points_list, intensities = [], [], [], []
        T_odom_frame, T_odom_lidar, T_map_odom = [], [], []
        gimbal, base, tf_ok = [], [], []
        for t in cloud_times:
            th = 40.0 * math.sin(2.0 * math.pi * t / 8.0)
            by = base_yaw(t)
            lidar_yaw = math.radians(by - th + DEMO_MOUNT_YAW_DEG)
            T_gimbal = rigid_matrix(
                (0.0, 0.0, 0.268),
                (0.0, 0.0, math.sin(math.radians(by - th) / 2.0),
                 math.cos(math.radians(by - th) / 2.0)))
            T_mount = rigid_matrix(
                (DEMO_LIDAR_XY[0], DEMO_LIDAR_XY[1], 0.078),
                (0.0, 0.0, math.sin(mount_yaw / 2.0), math.cos(mount_yaw / 2.0)))
            T_lidar = T_gimbal @ T_mount
            T_map = rigid_matrix(
                (0.617, 0.202, 0.083),
                (0.0, 0.0, math.sin(math.radians(-1.4) / 2.0),
                 math.cos(math.radians(-1.4) / 2.0)))

            sample = scene[rng.choice(len(scene), size=min(3800, len(scene)),
                                      replace=False)]
            sample = sample + rng.normal(0.0, 0.004, sample.shape)
            # 车体系点云要相对**当前**雷达位置去局部化：T_mount 的平移会随云台转，
            # 用固定零位 lidar_world 会凭空造出一个杠杆臂绕圈（新判据会抓成
            # translation_orbit）。missing/double 两路仍绕零位点转，不影响它们的 yaw 判读。
            lidar_position = T_lidar[:3, 3]
            local = sample - lidar_position
            cos_a, sin_a = math.cos(-lidar_yaw), math.sin(-lidar_yaw)
            body = np.column_stack([
                cos_a * local[:, 0] - sin_a * local[:, 1],
                sin_a * local[:, 0] + cos_a * local[:, 1],
                local[:, 2],
            ])

            if spec["kind"] == "correct":
                points = sample
            elif spec["kind"] == "body":
                points = body
            else:
                angle = math.radians(-th if spec["kind"] == "missing" else th)
                cos_t, sin_t = math.cos(angle), math.sin(angle)
                delta = sample - lidar_world
                points = np.column_stack([
                    cos_t * delta[:, 0] - sin_t * delta[:, 1],
                    sin_t * delta[:, 0] + cos_t * delta[:, 1],
                    delta[:, 2],
                ]) + lidar_world

            stamps.append(t + 1000.0)
            frame_ids.append(spec["frame"])
            points_list.append(points.astype(np.float32))
            intensities.append(None)
            T_odom_frame.append(T_lidar if spec["kind"] == "body" else np.eye(4))
            T_odom_lidar.append(T_lidar)
            T_map_odom.append(T_map)
            gimbal.append(th)
            base.append(by)
            tf_ok.append(True)

        recordings[topic] = TopicRecording(
            topic=topic, stamps=np.asarray(stamps, dtype=np.float64),
            frame_ids=frame_ids, points=points_list, intensity=intensities,
            T_odom_frame=np.asarray(T_odom_frame),
            T_odom_lidar=np.asarray(T_odom_lidar),
            T_map_odom=np.asarray(T_map_odom),
            gimbal_deg=np.asarray(gimbal), base_deg=np.asarray(base),
            tf_ok=np.asarray(tf_ok, dtype=bool))

    manifest = {
        "tool": "capture_gimbal_cloud_rotation.py",
        "version": 1,
        "created": datetime.now().isoformat(timespec="seconds"),
        "duration_s": duration,
        "host": platform.node(),
        "git": git_info(),
        "note": settings.note,
        "demo": True,
        "expected": {
            "/demo/cloud_world_correct": 0.0,
            "/demo/cloud_in_lidar_frame": 0.0,
            "/demo/cloud_missing_derotate": -1.0,
            "/demo/cloud_double_rotate": 1.0,
        },
        "frames": {"map": "map", "odom": "odom", "base": "base_link",
                   "gimbal": "gimbal_yaw", "lidar": "livox_frame"},
        "topics": [{"name": topic, "frame": spec["frame"]}
                   for topic, spec in topics.items()],
        "capture": {"demo": True, "tf_duplicate_parents": {}, "missing_tf": {},
                    "cloud_counts": {topic: len(rec.stamps)
                                     for topic, rec in recordings.items()}},
        "settings": {key: str(value) for key, value in settings.__dict__.items()},
        "notes": ["演示数据：4 路点云分别对应正确/车体系/漏补/双计，斜率应为 0/0/-1/+1"],
    }
    return samples, recordings, manifest


def _demo_scene(rng):
    """一个 14x10 m、带斜墙和柱子的房间（保证各方向都有结构，配准不退化）。"""
    chunks = []
    count = 9000
    chunks.append(np.column_stack([
        rng.uniform(-7.0, 7.0, count), rng.uniform(-5.0, 5.0, count),
        rng.normal(0.0, 0.008, count)]))
    for side in range(4):
        count = 4000
        z = rng.uniform(0.0, 3.0, count)
        if side < 2:
            x = rng.uniform(-7.0, 7.0, count)
            y = np.full(count, 5.0 if side == 0 else -5.0) + rng.normal(0.0, 0.008, count)
        else:
            y = rng.uniform(-5.0, 5.0, count)
            x = np.full(count, 7.0 if side == 2 else -7.0) + rng.normal(0.0, 0.008, count)
        chunks.append(np.column_stack([x, y, z]))
    count = 3000
    t = rng.uniform(0.0, 1.0, count)
    chunks.append(np.column_stack([
        -4.0 + 3.0 * t, -2.0 + 3.5 * t, rng.uniform(0.0, 2.2, count)]))
    for cx, cy in ((3.0, 2.0), (-3.5, 3.0), (2.5, -3.0)):
        count = 1200
        angle = rng.uniform(0.0, 2.0 * math.pi, count)
        radius = 0.35 + rng.normal(0.0, 0.01, count)
        chunks.append(np.column_stack([
            cx + radius * np.cos(angle), cy + radius * np.sin(angle),
            rng.uniform(0.0, 2.5, count)]))
    scene = np.concatenate(chunks, axis=0)
    ranges = np.linalg.norm(scene[:, :2] - DEMO_LIDAR_XY, axis=1)
    return scene[(ranges > 0.6) & (ranges < 20.0)]


# =============================================================
# 采集主流程
# =============================================================
def run_capture(settings, ros_argv):
    ensure_ros_log_dir()
    try:
        import rclpy
    except ImportError as exc:
        print(f"[错误] 没有 rclpy：{describe_exception(exc)}", file=sys.stderr)
        print("       先 source /opt/ros/humble/setup.bash 再 source install/setup.bash",
              file=sys.stderr)
        return 2

    rclpy.init(args=[sys.argv[0]] + ros_argv)
    capture = None
    bag = None
    try:
        capture = RosCapture(settings)
        if not capture.wait_ready(settings.wait_s):
            print("[错误] 等不到可用的点云/TF。先用下面的命令确认话题与数据：\n"
                  "  ros2 topic list | grep -E 'cloud|scan|terrain'\n"
                  "  ros2 topic hz /registered_scan\n"
                  "  ros2 run tf2_ros tf2_echo odom base_link",
                  file=sys.stderr)
            return 2

        if settings.bag:
            bag = start_bag(settings, capture.cloud_topics, capture.node)

        capture.spin()
        # 数据已经在内存里，后面的收尾/分析不要再被第二个 Ctrl-C 打断；
        # 真要中断就 kill，原始数据包在 finalize 之后已经落盘。
        try:
            signal.signal(signal.SIGINT, signal.SIG_IGN)
        except (ValueError, OSError):
            pass
        recordings, samples, extra = capture.finalize()
        if not recordings:
            print("[错误] 没有收到任何点云帧，未生成数据包。", file=sys.stderr)
            return 1
        if bag is not None:
            stop_bag(bag[0])
            extra["bag"] = bag[1].name
        extra["parameters"] = dump_parameters(settings, extra.get("node_names", []))
        manifest = build_manifest(settings, capture, recordings, samples, extra)
        bundle = find_free_dir(
            settings.out_dir / f"{settings.tag}_{datetime.now():%Y%m%d_%H%M%S}")
        write_bundle(bundle, samples, recordings, manifest)
        export_clouds(bundle, recordings, settings)
        print(f"[输出] 数据包: {bundle}")
        if settings.no_analyze:
            return 0
        analysis = analyze_bundle(bundle, settings)
        if analysis is None:
            return 3
        if settings.make_tar:
            make_tarball(bundle)
        return 0 if analysis_is_decisive(analysis) else 1
    except KeyboardInterrupt:
        print("\n[中断] Ctrl-C。")
        return 1
    except Exception as exc:  # noqa: BLE001
        print(f"[错误] 采集失败: {describe_exception(exc)}", file=sys.stderr)
        traceback.print_exc()
        return 2
    finally:
        if bag is not None:
            stop_bag(bag[0])
        if capture is not None:
            try:
                capture.node.destroy_node()
            except Exception:  # noqa: BLE001
                pass
        try:
            rclpy.try_shutdown()
        except Exception:  # noqa: BLE001
            pass


def analysis_is_decisive(analysis):
    if analysis is None:
        return False
    return any(result.kind not in ("insufficient", "unclear")
               for result in analysis.results)


def build_manifest(settings, capture, recordings, samples, extra):
    settings_dict = {}
    for key, value in settings.__dict__.items():
        settings_dict[key] = str(value) if isinstance(value, Path) else value
    return {
        "tool": "capture_gimbal_cloud_rotation.py",
        "version": 1,
        "created": datetime.now().isoformat(timespec="seconds"),
        "duration_s": float(getattr(capture, "recorded_duration", 0.0)),
        "host": platform.node(),
        "user": os.environ.get("USER", ""),
        "python": sys.version.split()[0],
        "numpy": np.__version__,
        "git": git_info(),
        "ros": ros_environment(),
        "frames": {
            "map": capture.map_frame, "odom": capture.odom_frame,
            "base": capture.base_frame, "gimbal": capture.gimbal_frame,
            "lidar": capture.lidar_frame,
        },
        "topics": [
            {
                "name": topic,
                "frame": recording.frame_ids[0] if recording.frame_ids else "",
                "frames": len(recording),
                "valid_tf": int(np.asarray(recording.tf_ok, dtype=bool).sum()),
                "median_points": int(np.median([len(points) for points in recording.points]))
                if recording.points else 0,
            }
            for topic, recording in recordings.items()
        ],
        "settings": settings_dict,
        "capture": extra,
        "note": settings.note,
        "samples": int(len(samples.get("t_stamp", []))),
    }


def start_bag(settings, cloud_topics, node=None):
    robot_topics = []
    for name in ("joint_states", "chassis_to_gimbal_yaw_deg", "odometry",
                 "lidar_odometry", "gimbal_joint_angle"):
        # 有命名空间时解析成绝对名，否则 bag 里录的是 /joint_states 而不是 /ns/joint_states。
        robot_topics.append(node.resolve_topic_name(name) if node is not None
                            else "/" + name)
    topics = list(dict.fromkeys(list(cloud_topics) + ["/tf", "/tf_static"] + robot_topics))
    bag_dir = Path(settings.out_dir) / f"{settings.tag}_bag_{datetime.now():%Y%m%d_%H%M%S}"
    bag_dir.parent.mkdir(parents=True, exist_ok=True)
    print(f"[bag] 开始录制 {len(topics)} 路话题 -> {bag_dir}（点云很占空间，注意磁盘）")
    try:
        process = subprocess.Popen(
            ["ros2", "bag", "record", "-o", str(bag_dir)] + topics,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            env=os.environ.copy())
        return process, bag_dir
    except Exception as exc:  # noqa: BLE001
        print(f"[警告] 启动 rosbag 失败: {describe_exception(exc)}")
        return None


def stop_bag(process):
    try:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            process.wait(timeout=15)
    except Exception:  # noqa: BLE001
        try:
            process.kill()
        except Exception:  # noqa: BLE001
            pass


def make_tarball(bundle):
    bundle = Path(bundle)
    target = bundle.with_suffix(".tar.gz")
    print(f"[输出] 正在打包 {target.name} …")
    with tarfile.open(target, "w:gz") as archive:
        archive.add(bundle, arcname=bundle.name)
    print(f"[输出] 压缩包: {target}")


# =============================================================
# CLI
# =============================================================
def print_help():
    # 本工具的文件头是 docstring（不是 # 注释块），直接打 __doc__；
    # `_common.print_header_help()` 只认 # 开头的行，对这种文件会输出空。
    print(__doc__ or "")


def resolve_cloud_topics(values):
    resolved = []
    for item in values:
        resolved.extend(part for part in item.split(",") if part.strip())
    return tuple(resolved)


def build_settings(argv):
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("-h", "--help", action="store_true")
    parser.add_argument("--duration", type=float, default=0.0)
    parser.add_argument("--rate", type=float, default=5.0)
    parser.add_argument("--sample-hz", type=float, default=20.0)
    parser.add_argument("--wait", type=float, default=60.0)
    parser.add_argument("--cloud-topics", action="append", default=[])
    parser.add_argument("--max-topics", type=int, default=6)
    parser.add_argument("--max-points", type=int, default=4000)
    parser.add_argument("--voxel", type=float, default=0.10)
    parser.add_argument("--tf-cache", type=float, default=0.0)
    parser.add_argument("--sim-time", choices=("auto", "on", "off"), default="auto")
    parser.add_argument("--bag", action="store_true")
    parser.add_argument("--map-frame", default=DEFAULT_MAP_FRAME)
    parser.add_argument("--odom-frame", default="")
    parser.add_argument("--base-frame", default=DEFAULT_BASE_FRAME)
    parser.add_argument("--gimbal-frame", default=DEFAULT_GIMBAL_FRAME)
    parser.add_argument("--lidar-frame", default="")
    parser.add_argument("--analysis-frame", default=DEFAULT_ODOM_FRAME)
    parser.add_argument("--min-range", type=float, default=0.5)
    parser.add_argument("--max-range", type=float, default=20.0)
    parser.add_argument("--align-points", type=int, default=900)
    parser.add_argument("--inlier-dist", type=float, default=0.15)
    parser.add_argument("--max-shift", type=float, default=0.6)
    parser.add_argument("--yaw-range", type=float, default=60.0)
    parser.add_argument("--coarse-step", type=float, default=3.0)
    parser.add_argument("--fine-step", type=float, default=0.30)
    parser.add_argument("--pair-gap", type=float, default=0.6)
    parser.add_argument("--min-travel", type=float, default=10.0)
    parser.add_argument("--min-inlier", type=float, default=0.45)
    parser.add_argument("--min-dtheta", type=float, default=0.15)
    parser.add_argument("--orbit-min", type=float, default=0.03,
                        help="平移绕圈半径超过这个值（米）就单独判读")
    parser.add_argument("--anomaly-deg", type=float, default=1.0,
                        help="坐标/位姿单步跳变超过这个值就记一条异常（度）")
    parser.add_argument("--lag-max", type=float, default=0.4)
    parser.add_argument("--lag-step", type=float, default=0.05)
    parser.add_argument("--anchor-stride", type=int, default=2)
    parser.add_argument("--out-dir", default=str(DEFAULT_OUT_DIR))
    parser.add_argument("--tag", default="gimbal_cloud")
    parser.add_argument("--note", default="",
                        help="采集动作备注（写进 report/manifest，方便交接）")
    parser.add_argument("--no-pcd", action="store_true")
    parser.add_argument("--pcd-max", type=int, default=40)
    parser.add_argument("--no-plot", action="store_true")
    parser.add_argument("--no-analyze", action="store_true")
    parser.add_argument("--tar", action="store_true")
    parser.add_argument("--analyze", default="")
    parser.add_argument("--demo", action="store_true")
    parser.add_argument("--param-nodes", default=(
        "zlio_bridge,sensor_scan_generation,terrain_analysis,terrain_analysis_ext"))

    pair_markers = ("-r", "-p", "--remap", "--param", "--params-file")
    plain_argv, ros_argv = [], []
    index = 0
    while index < len(argv):
        token = argv[index]
        if token == "--ros-args":
            ros_argv.extend(argv[index:])
            break
        if token in pair_markers and index + 1 < len(argv):
            ros_argv.extend(argv[index:index + 2])
            index += 2
            continue
        plain_argv.append(token)
        index += 1
    if ros_argv and ros_argv[0] != "--ros-args":
        ros_argv.insert(0, "--ros-args")

    args = parser.parse_args(plain_argv)
    if args.help:
        print_help()
        raise SystemExit(0)

    settings = Settings(
        duration=max(args.duration, 0.0),
        rate_hz=max(args.rate, 1.0),
        sample_hz=max(args.sample_hz, 1.0),
        wait_s=max(args.wait, 0.0),
        max_points=max(args.max_points, 500),
        voxel=max(args.voxel, 0.0),
        max_topics=max(args.max_topics, 1),
        cloud_topics=resolve_cloud_topics(args.cloud_topics),
        tf_cache=max(args.tf_cache, 0.0),
        sim_time=args.sim_time,
        bag=args.bag,
        param_nodes=args.param_nodes,
        map_frame=args.map_frame,
        odom_frame=args.odom_frame,
        base_frame=args.base_frame,
        gimbal_frame=args.gimbal_frame,
        lidar_frame=args.lidar_frame,
        analysis_frame=args.analysis_frame,
        min_range=max(args.min_range, 0.0),
        max_range=max(args.max_range, 1.0),
        align_points=max(args.align_points, 100),
        inlier_dist=max(args.inlier_dist, 0.02),
        max_shift=max(args.max_shift, 0.0),
        yaw_range=min(max(args.yaw_range, 5.0), 180.0),
        coarse_step=max(args.coarse_step, 0.5),
        fine_step=max(args.fine_step, 0.05),
        pair_gap=max(args.pair_gap, 0.05),
        min_travel=max(args.min_travel, 0.0),
        min_inlier=min(max(args.min_inlier, 0.0), 1.0),
        min_dtheta=max(args.min_dtheta, 0.0),
        anomaly_deg=max(args.anomaly_deg, 0.1),
        orbit_min=max(args.orbit_min, 0.0),
        lag_max=max(args.lag_max, 0.0),
        lag_step=max(args.lag_step, 0.01),
        anchor_stride=max(args.anchor_stride, 1),
        out_dir=Path(args.out_dir).expanduser(),
        tag=args.tag,
        note=args.note.strip(),
        no_pcd=args.no_pcd,
        pcd_max=max(args.pcd_max, 2),
        no_plot=args.no_plot,
        no_analyze=args.no_analyze,
        make_tar=args.tar,
        analyze_dir=args.analyze,
        demo=args.demo,
    )
    return settings, ros_argv


def describe_settings(settings):
    lines = ["=" * 68,
             " 云台点云转动抓包 (debug_tools/capture_gimbal_cloud_rotation.py)",
             "=" * 68,
             f"  点云     : "
             f"{', '.join(settings.cloud_topics) if settings.cloud_topics else '自动识别'}"
             f"（最多 {settings.max_topics} 路，{settings.rate_hz:g} Hz，"
             f"体素 {settings.voxel:g} m，上限 {settings.max_points} 点/帧）",
             f"  标量采样 : {settings.sample_hz:g} Hz（云台角/TF/底盘速度）",
             f"  分析系   : {settings.analysis_frame}"
             f"    配准: {settings.align_points} 点/帧，搜索 ±{settings.yaw_range:g}°",
             f"  判据     : 相邻帧 2D 配准 + 锚点对齐；阈值 行程≥{settings.min_travel:g}°、"
             f"内点≥{settings.min_inlier:g}",
             f"  产物     : {settings.out_dir}",
             f"  时长     : " + (human_duration(settings.duration)
                                if settings.duration > 0 else "一直录到 Ctrl-C")]
    if settings.note:
        lines.append(f"  备注     : {settings.note}")
    if settings.bag:
        lines.append("  rosbag   : 开（点云全量，注意磁盘）")
    return "\n".join(lines)


def run_demo(settings):
    print("[demo] 生成 4 路已知答案的点云（正确 / 车体系 / 漏补 / 双计）…")
    samples, recordings, manifest = build_demo(settings)
    bundle = find_free_dir(
        settings.out_dir / f"{settings.tag}_demo_{datetime.now():%Y%m%d_%H%M%S}")
    write_bundle(bundle, samples, recordings, manifest)
    export_clouds(bundle, recordings, settings)
    print(f"[demo] 数据包: {bundle}")
    settings.analysis_frame = "odom"
    analysis = analyze_bundle(bundle, settings)
    if analysis is None:
        return 3
    if settings.make_tar:
        make_tarball(bundle)
    expected = manifest.get("expected", {})
    ok = True
    print("[demo] 自检（测量斜率 vs 期望）：")
    for result in analysis.results:
        want = expected.get(result.topic)
        if want is None:
            continue
        got = result.slope
        passed = np.isfinite(got) and abs(got - want) <= 0.25
        ok = ok and passed
        print(f"  {'PASS' if passed else 'FAIL'}  {result.topic:34s} "
              f"期望 {want:+.1f}  实测 {got:+.2f}  r={result.corr:+.2f}")
    print("[demo] " + ("全部通过：配准、符号、斜率、判读链路都正常。" if ok
                       else "有偏差，先看 analysis/pairs.csv。"))
    return 0 if ok else 1


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    settings, ros_argv = build_settings(argv)
    print(describe_settings(settings))

    if settings.analyze_dir:
        if settings.demo:
            print("[错误] --analyze 与 --demo 不能同时用。", file=sys.stderr)
            return 2
        try:
            analysis = analyze_bundle(Path(settings.analyze_dir).expanduser(), settings)
        except FileNotFoundError as exc:
            print(f"[错误] {exc}", file=sys.stderr)
            return 2
        except Exception as exc:  # noqa: BLE001
            print(f"[错误] 分析失败: {describe_exception(exc)}", file=sys.stderr)
            traceback.print_exc()
            return 3
        if analysis is None:
            return 3
        if settings.make_tar:
            make_tarball(analysis.bundle)
        return 0 if analysis_is_decisive(analysis) else 1

    if settings.demo:
        return run_demo(settings)

    return run_capture(settings, ros_argv)


if __name__ == "__main__":
    sys.exit(main())
