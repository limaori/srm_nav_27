#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# =============================================================
# 电脑占用监视 (debug_tools #2)
#
# 实时画本机性能占用：CPU（总 + 每个核心）、内存、交换、CPU 温度、
# 磁盘 I/O、网络 I/O，外加"最吃 CPU 的几个进程"。
#
# 为什么导航调试需要它：
#   * 导航栈卡顿/轨迹跳变，很多时候不是算法问题，是 CPU 打满、内存吃紧、
#     swap 被换出去（本机 swap 就常年 ~99%）、或者温度高到降频；
#   * Point-LIO + Nav2 + RViz + Gazebo 同时跑时，是哪个进程在吃核一清二楚；
#   * 网络收发异常高 = DDS 在狂发（多机/回环配置问题）。
#
# 分组统计（回答"跑导航一共吃了多少机器"）：
#   Nav2 一启动就是十几个节点，单看某个进程说明不了问题。工具会按命令行把进程
#   归到 6 个组（仿真 Gazebo / RViz / Nav2 导航栈 / 定位里程计 / 雷达驱动 / 底盘串口），
#   加总成"导航栈合计 x.xx 核"，并单独列出远程桌面、本工具这类干扰项 ——
#   它们不算在导航的账上，但会抢同一个 CPU。
#
# 用法
# -------------------------------------------------------------
#   ./debug_tools/system_monitor.py                    # 实时窗口（4 个子图）
#   ./debug_tools/system_monitor.py --text             # 纯终端（ssh/无显示器）
#   ./debug_tools/system_monitor.py --text --once      # 只打印一屏快照就退出
#   ./debug_tools/system_monitor.py --text --match nav2,point_lio
#                                                      # 只看匹配命令行/进程名的进程
#   ./debug_tools/system_monitor.py --duration 60 --save /tmp/sys.png
#   ./debug_tools/system_monitor.py --csv /tmp/sys.csv --duration 30
#   ./debug_tools/system_monitor.py --headless --duration 20 --save /tmp/sys.png
#
# 依赖：python3-psutil（`sudo apt install python3-psutil`），不需要 ROS。
#
# 参数
# -------------------------------------------------------------
#   --text             纯终端输出（不画图）；没有 DISPLAY 时不给 --save 会自动用它
#   --once             只采一屏就退出（配 --text 用，适合"现在到底谁在吃 CPU"）
#   --procs <N>        进程表行数，0 = 不采集进程（默认 6；画图时角标只显示前 3）
#                      "导航栈合计"要靠进程表算，所以设成 0 就没有分组统计了
#   --procs-csv <路径> 把每次进程扫描写进 CSV（t/分组/pid/cpu/rss/命令），
#                      用于事后回溯"某一刻是谁在吃 CPU"；默认不写
#   --match <关键字>   只看命令行/进程名里包含这些关键字的进程，逗号分隔或重复给
#   --interval <秒>    --text 模式的刷新周期，默认 1.0
#   --window <秒>      波形时间窗，默认 60
#   --sample-hz <Hz>   采样频率，默认 5（系统占用不需要 50 Hz，高了纯浪费）
#   --plot-hz <Hz>     窗口刷新频率，默认 2（看趋势够用；调高会让工具自己吃 CPU）
#   --duration <秒>    运行时长，0 = 一直跑到关窗口 / Ctrl-C
#   --csv <路径>       录 CSV（每行一个采样点）
#   --save <路径>      退出时保存一张 PNG
#   --headless         不弹窗（Agg 后端），配合 --duration/--save 用
#   -h/--help          显示本帮助
#
# 图上/屏上看什么
# -------------------------------------------------------------
#   * 最上面一行是"导航栈合计 x.xx 核（占整机 y%）"以及各组分解，
#     底部状态行也有一个"导航栈 x.xx 核"——这是回答"导航吃了多少机器"的地方；
#   * 终端模式（--text）会多一张按分组的表，含每组的内存和进程数；
#   * CPU 到 90% 以上：先在进程表里找吃核的那个，再谈调参；
#   * swap 长期高占用（本机历史上 ~99%）：说明物理内存不够，整机都会变慢，
#     这种"卡"跟控制器参数无关；
#   * 温度逼近 90~100°C：CPU 在降频，同样的代码会突然变慢；
#   * 磁盘写持续很高：录包/日志在拖慢系统（尤其是往 eMMC/SD 上写）；
#   * 网络收发很高但没在传文件：查 DDS 配置（多机/回环），别让它占满带宽。
#
# 注意：进程 CPU% 沿用 psutil 的口径 —— **单核百分比**，多线程进程会超过 100%
# （16 线程机器上满载可达 1600%）。这是相对"一个核"的占比，不是整机占比。
# =============================================================

from __future__ import annotations

import argparse
import math
import os
import shutil
import sys
import threading
import time
from collections import deque
from pathlib import Path

from _common import (
    CsvLog,
    fmt_bytes,
    prepare_matplotlib_env,
    print_header_help,
    setup_cjk_font,
)

CSV_FIELDS = (
    "t_elapsed", "cpu_total", "cpu_max", "mem_pct", "mem_used_gb", "swap_pct",
    "temp_c", "disk_read_mbps", "disk_write_mbps", "net_rx_mbps", "net_tx_mbps",
    "net_lo_rx_mbps", "net_lo_tx_mbps",
    "load1", "load5", "load15", "disk_used_pct", "mem_avail_gb",
    "nav_stack_cores", "nav_stack_share", "nav_stack_rss_gb",
)

PROC_CSV_FIELDS = ("t_elapsed", "group", "pid", "cpu_pct", "rss_mb", "cmd")

# 阈值：超过就画参考线 / 在终端里标黄（都是经验值，够用即可）
WARN_CPU = 90.0
WARN_SWAP = 90.0
WARN_MEM = 95.0
WARN_TEMP = 90.0

BAR_FULL = "█"
BAR_EMPTY = "░"


# 进程分组：回答"跑导航项目一共吃了多少机器"这个问题，靠单个进程是看不出来的 ——
# Nav2 一启动就是十几个节点，外加 gz、rviz、LIO、雷达驱动，必须按组加总。
# 归类只看命令行里的关键字（不区分大小写），先匹配先得，都不中就是"其他"。
PROCESS_GROUPS = (
    ("仿真 Gazebo", ("ign gazebo", "gz sim", "gz-sim", "gazebo", "gz_", "/gz ")),
    ("RViz", ("rviz",)),
    ("Nav2 导航栈", (
        "nav2", "controller_server", "planner_server", "bt_navigator", "behavior_server",
        "smoother_server", "velocity_smoother", "lifecycle_manager", "map_server",
        "waypoint_follower", "costmap", "nav2_container", "amcl", "slam_toolbox",
        "recoveries_server",
    )),
    ("定位/里程计", (
        "point_lio", "pointlio", "lio_sam", "loam", "sensor_scan", "small_gicp", "gicp",
        "odometry", "relocalization", "terrain_analysis",
    )),
    ("雷达驱动", ("livox", "mid360", "mid-360", "ouster", "velodyne")),
    ("底盘串口", ("srm27_nav_protocol", "nav_protocol")),
    ("远程桌面", ("todesk", "sunlogin", "anydesk", "teamviewer", "vnc")),
    ("本工具", ("system_monitor", "plot_vxvywz", "plot_lio_gt_yaw")),
)
# 这几组算"导航项目"的账：仿真 + 可视化 + 导航栈 + 定位 + 雷达 + 底盘串口
NAV_GROUPS = ("仿真 Gazebo", "RViz", "Nav2 导航栈", "定位/里程计", "雷达驱动", "底盘串口")
GROUP_OTHER = "其他"


def classify(cmd):
    """按命令行关键字把进程归档；顺序即优先级（比如 gz 的 rviz 插件先算 Gazebo）。"""
    lowered = cmd.lower()
    for name, keywords in PROCESS_GROUPS:
        if any(keyword in lowered for keyword in keywords):
            return name
    return GROUP_OTHER


def summarize_groups(rows, ncpu):
    """把进程行按组加总（CPU% 相对单核，除以 100 就是"核"）。"""
    summary = {}
    for row in rows:
        group = row.get("group") or GROUP_OTHER
        item = summary.setdefault(group, {"cpu": 0.0, "rss": 0.0, "count": 0})
        item["cpu"] += row["cpu"]
        item["rss"] += row["rss"]
        item["count"] += 1
    nav = {
        "cpu": sum(summary[group]["cpu"] for group in NAV_GROUPS if group in summary) / 100.0,
        "rss": sum(summary[group]["rss"] for group in NAV_GROUPS if group in summary),
        "count": sum(summary[group]["count"] for group in NAV_GROUPS if group in summary),
    }
    nav["share"] = nav["cpu"] / ncpu if ncpu else float("nan")   # 占整机比例
    return summary, nav


def cpu_model():
    """从 /proc/cpuinfo 取 CPU 型号，取不到就返回空串。"""
    try:
        with open("/proc/cpuinfo", encoding="utf-8", errors="replace") as stream:
            for line in stream:
                if line.startswith("model name"):
                    return line.split(":", 1)[1].strip()
    except OSError:
        pass
    return ""


# =============================================================
# 采样：psutil 读系统计数，速率类指标靠两次采样求差
# =============================================================
class Probe:
    """把一次系统采样收敛成一个 dict；进程表单独按低频采集。"""

    TEMP_PREFERENCE = ("coretemp", "k10temp", "zenpower", "cpu_thermal",
                       "soc_thermal", "acpitz")

    def __init__(self, settings):
        import psutil
        self.ps = psutil
        self.settings = settings
        # 第一次 cup_percent() 永远返回 0，先预热掉，免得图上开局一根假的下冲
        psutil.cpu_percent(interval=None)
        cores = psutil.cpu_percent(interval=None, percpu=True) or []
        self.ncpu = len(cores) or (os.cpu_count() or 1)
        # 速率类指标要有两次采样才能求差；第一次故意不给基准，
        # 否则"初始化 -> 第一个采样点"这段短窗口会算出一个假的大尖峰。
        self._last_disk = None
        self._last_nic = None
        self._last_stamp = time.monotonic()
        self._temp_source = self._pick_temp_source()
        self._temp_files = self._find_thermal_files()

    # ------------------------------------------------------------ 小工具
    @staticmethod
    def _safe_call(func, *args, **kwargs):
        try:
            return func(*args, **kwargs)
        except Exception:
            return None

    def _pick_temp_source(self):
        """挑一个"CPU 温度"的传感器名；挑不到返回空串。"""
        sensors = self._safe_call(self.ps.sensors_temperatures) or {}
        for name in self.TEMP_PREFERENCE:
            if sensors.get(name):
                return name
        for name in sensors:
            if "nvme" not in name and "iwlwifi" not in name and sensors[name]:
                return name
        return ""

    @staticmethod
    def _find_thermal_files():
        """/sys/class/thermal 兜底（在容器/无 sensors_temperatures 的环境里）。"""
        import glob
        return sorted(glob.glob("/sys/class/thermal/thermal_zone*/temp"))

    def _read_temperature(self):
        if self._temp_source:
            sensors = self._safe_call(self.ps.sensors_temperatures) or {}
            values = [
                entry.current for entry in sensors.get(self._temp_source, [])
                if entry.current is not None and math.isfinite(entry.current)
            ]
            if values:
                return max(values), self._temp_source
            return float("nan"), self._temp_source
        values = []
        for path in self._temp_files:
            try:
                with open(path, encoding="ascii") as stream:
                    raw = float(stream.read().strip())
            except (OSError, ValueError):
                continue
            if raw > 1000.0:            # 毫摄氏度
                raw /= 1000.0
            if 0.0 < raw < 150.0:
                values.append(raw)
        if values:
            return max(values), "thermal_zone"
        return float("nan"), ""

    # ------------------------------------------------------------ 采样
    def sample(self):
        psutil = self.ps
        now = time.monotonic()
        dt = max(now - self._last_stamp, 1e-9)

        cores = psutil.cpu_percent(interval=None, percpu=True) or []
        total = psutil.cpu_percent(interval=None)
        mem = psutil.virtual_memory()
        swap = psutil.swap_memory()
        disk = self._safe_call(psutil.disk_io_counters)
        per_nic = self._safe_call(psutil.net_io_counters, pernic=True) or {}

        # 累计计数求差 -> 瞬时速率；第一次采样没有基准，先给 nan（图上就是断线）
        disk_r = disk_w = float("nan")
        if disk is not None and self._last_disk is not None:
            disk_r = max(disk.read_bytes - self._last_disk.read_bytes, 0) / dt / 1e6
            disk_w = max(disk.write_bytes - self._last_disk.write_bytes, 0) / dt / 1e6

        # 网络按网卡分开算：总和里往往 lo（回环）占大头 —— 单机跑 ROS 2/gazebo 时，
        # DDS 的点云/时钟全走 localhost，而远程桌面（ToDesk 之类）走真实网卡。
        # 混在一起看会把两件完全不同的事读成一件事，所以 lo 单独留一份。
        net_rx = net_tx = net_lo_rx = net_lo_tx = float("nan")
        if self._last_nic and per_nic:
            rates = {}
            for name, counter in per_nic.items():
                prev = self._last_nic.get(name)
                if prev is None:
                    continue
                rates[name] = (
                    max(counter.bytes_recv - prev.bytes_recv, 0) / dt / 1e6,
                    max(counter.bytes_sent - prev.bytes_sent, 0) / dt / 1e6,
                )
            selected = self.settings.interfaces or list(rates)
            net_rx = sum(rates[name][0] for name in selected if name in rates)
            net_tx = sum(rates[name][1] for name in selected if name in rates)
            net_lo_rx = rates.get("lo", (0.0, 0.0))[0]
            net_lo_tx = rates.get("lo", (0.0, 0.0))[1]
        self._last_disk, self._last_nic, self._last_stamp = disk, per_nic, now

        temp, temp_src = self._read_temperature()

        try:
            load1, load5, load15 = psutil.getloadavg()
        except (AttributeError, OSError):
            load1 = load5 = load15 = float("nan")

        usage = self._safe_call(psutil.disk_usage, self.settings.disk_path)
        battery = self._safe_call(psutil.sensors_battery)
        if getattr(battery, "percent", None) is None:
            battery = None

        return {
            "t": 0.0,                     # 由 Sampler 填
            "cpu": float(total),
            "cores": tuple(float(value) for value in cores),
            "mem_pct": float(mem.percent),
            "mem_used": float(mem.used),
            "mem_total": float(mem.total),
            "mem_avail": float(mem.available),
            "swap_pct": float(swap.percent) if swap.total else 0.0,
            "swap_used": float(swap.used),
            "swap_total": float(swap.total),
            "temp": temp,
            "temp_src": temp_src,
            "disk_r": disk_r,
            "disk_w": disk_w,
            "net_rx": net_rx,
            "net_tx": net_tx,
            "net_lo_rx": net_lo_rx,
            "net_lo_tx": net_lo_tx,
            "load1": float(load1),
            "load5": float(load5),
            "load15": float(load15),
            "disk_used_pct": float(usage.percent) if usage else float("nan"),
            "disk_free": float(usage.free) if usage else float("nan"),
            "battery": ((float(battery.percent), bool(battery.power_plugged))
                        if battery is not None else None),
            "ncpu": self.ncpu,
        }

    def processes(self, limit, match):
        """扫一遍进程，按 CPU% 降序返回前 limit 个（limit<=0 = 全部，分组汇总要用全部）。

        psutil 的进程 CPU% 是"相对一个核"的占比，两次调用之间求差；第一次见到的
        进程给 0。`process_iter` 内部有进程对象缓存，所以循环调用是有效的。
        """
        rows = []
        try:
            iterator = self.ps.process_iter(
                ["pid", "name", "cpu_percent", "memory_info", "cmdline"])
        except Exception:
            return rows
        for proc in iterator:
            try:
                info = proc.info
                cmd = " ".join(info.get("cmdline") or []) or (info.get("name") or "?")
                if match and not any(token in cmd for token in match):
                    continue
                memory = info.get("memory_info")
                rows.append({
                    "pid": int(info.get("pid") or 0),
                    "cpu": float(info.get("cpu_percent") or 0.0),
                    "rss": float(getattr(memory, "rss", 0) or 0),
                    "cmd": cmd,
                    "group": classify(cmd),
                })
            except Exception:            # 进程在枚举过程中退出/N多，跳过即可
                continue
        rows.sort(key=lambda row: (row["cpu"], row["rss"]), reverse=True)
        return rows[:limit] if limit > 0 else rows


# =============================================================
# 记录：环形缓冲 + CSV + 进程表
# =============================================================
class Recorder:
    def __init__(self, settings):
        self.settings = settings
        self.lock = threading.Lock()
        self.samples = deque()
        self.procs = []
        self.groups = {}
        self.nav = {"cpu": 0.0, "rss": 0.0, "count": 0, "share": float("nan")}
        self.proc_scans = 0
        self.count = 0
        self.csv = CsvLog(settings.csv_path, CSV_FIELDS) if settings.csv_path else None
        self.proc_csv = (CsvLog(settings.procs_csv_path, PROC_CSV_FIELDS)
                         if settings.procs_csv_path else None)

    def push(self, sample):
        with self.lock:
            # 把当前的分组快照挂到样本上：CSV 里就能看到"那一刻导航栈吃了几核"
            sample["nav_cores"] = self.nav.get("cpu", float("nan"))
            sample["nav_share"] = self.nav.get("share", float("nan"))
            sample["nav_rss_gb"] = self.nav.get("rss", 0.0) / 2**30
            self.samples.append(sample)
            horizon = sample["t"] - self.settings.window_sec - 5.0
            while len(self.samples) > 2 and self.samples[0]["t"] < horizon:
                self.samples.popleft()
            self.count += 1
        if self.csv is not None:
            self.csv.write(self._csv_row(sample))

    @staticmethod
    def _csv_row(sample):
        def num(value, scale=1.0):
            if value is None or (isinstance(value, float) and not math.isfinite(value)):
                return ""
            return f"{value / scale:.6f}"

        return (
            f"{sample['t']:.3f}",
            f"{sample['cpu']:.3f}",
            f"{max(sample['cores']) if sample['cores'] else 0.0:.3f}",
            f"{sample['mem_pct']:.3f}",
            f"{sample['mem_used'] / 2**30:.3f}",
            f"{sample['swap_pct']:.3f}",
            num(sample["temp"]),
            num(sample["disk_r"]),
            num(sample["disk_w"]),
            num(sample["net_rx"]),
            num(sample["net_tx"]),
            num(sample["net_lo_rx"]),
            num(sample["net_lo_tx"]),
            num(sample["load1"]),
            num(sample["load5"]),
            num(sample["load15"]),
            num(sample["disk_used_pct"]),
            f"{sample['mem_avail'] / 2**30:.3f}",
            num(sample.get("nav_cores", float("nan"))),
            num(sample.get("nav_share", float("nan"))),
            num(sample.get("nav_rss_gb", float("nan"))),
        )

    def set_procs(self, rows, groups=None, nav=None, elapsed=None):
        with self.lock:
            self.procs = list(rows)
            if groups is not None:
                self.groups = groups
            if nav is not None:
                self.nav = nav
            self.proc_scans += 1
        # 进程级 CSV：想回溯"某一刻到底是谁在吃 CPU"只能靠它（主 CSV 只有汇总）
        if self.proc_csv is not None and rows:
            for row in rows:
                self.proc_csv.write((
                    f"{elapsed if elapsed is not None else 0.0:.3f}",
                    row.get("group", GROUP_OTHER),
                    str(row["pid"]),
                    f"{row['cpu']:.1f}",
                    f"{row['rss'] / 2**20:.1f}",
                    " ".join(row["cmd"].split()).replace(",", " "),
                ))

    def snapshot_groups(self):
        with self.lock:
            return dict(self.groups), dict(self.nav)

    def proc_scan_count(self):
        with self.lock:
            return self.proc_scans

    def sample_count(self):
        with self.lock:
            return self.count

    def snapshot(self):
        with self.lock:
            return list(self.samples)

    def latest(self):
        with self.lock:
            return self.samples[-1] if self.samples else None

    def snapshot_procs(self):
        with self.lock:
            return list(self.procs)

    def close(self):
        if self.csv is not None:
            self.csv.close()
        if self.proc_csv is not None:
            self.proc_csv.close()


class Sampler(threading.Thread):
    """后台线程：按 sample_hz 采样，按 proc_hz 扫一次进程表。"""

    def __init__(self, settings, recorder, probe):
        super().__init__(daemon=True)
        self.settings = settings
        self.recorder = recorder
        self.probe = probe
        self._stop = threading.Event()
        self._error = None

    def run(self):
        period = 1.0 / max(self.settings.sample_hz, 1.0)
        proc_period = 1.0 / max(self.settings.proc_hz, 0.1)
        start = time.monotonic()
        next_proc = start
        while not self._stop.is_set():
            tick = time.monotonic()
            try:
                sample = self.probe.sample()
            except Exception as exc:        # psutil 偶发读不到（进程/内核竞态）
                self._error = str(exc)
                time.sleep(period)
                continue
            sample["t"] = tick - start
            self.recorder.push(sample)

            if self.settings.procs > 0 and tick >= next_proc:
                # 先拿全量（分组要加总所有进程），再截出前 N 行给表格用
                rows = self.probe.processes(0, self.settings.match)
                groups, nav = summarize_groups(rows, self.probe.ncpu)
                self.recorder.set_procs(
                    rows[:self.settings.procs], groups, nav, elapsed=tick - start)
                next_proc = tick + proc_period
            time.sleep(max(period - (time.monotonic() - tick), 0.0))

    def stop(self):
        self._stop.set()


# =============================================================
# 终端模式（ssh / 无显示器 / --once 快照）
# =============================================================
def make_bar(percent, width, full=BAR_FULL, empty=BAR_EMPTY):
    percent = 0.0 if not math.isfinite(percent) else max(0.0, min(percent, 100.0))
    filled = int(round(percent / 100.0 * width))
    return full * filled + empty * (width - filled)


def warn_tag(condition, text=""):
    """超标时补一句提示：⚠ 固定在最前面，便于 grep。"""
    if not condition:
        return ""
    return f"   ⚠ {text}".rstrip()


class TextView:
    def __init__(self, settings, recorder):
        self.settings = settings
        self.recorder = recorder
        self.interactive = sys.stdout.isatty()
        # 终端编码不是 UTF-8 时用 ASCII 进度条，否则 Linux 控制台/串口终端会乱码
        encoding = (sys.stdout.encoding or "").lower()
        self.full = BAR_FULL if "utf" in encoding else "#"
        self.empty = BAR_EMPTY if "utf" in encoding else "-"
        self.width = max(shutil.get_terminal_size((100, 30)).columns, 60)

    def _bar(self, percent, width):
        return make_bar(percent, width, self.full, self.empty)

    # ------------------------------------------------------------ 渲染
    def render(self, sample, procs):
        out = []
        ncpu = sample.get("ncpu", 0)
        mem_gb = sample["mem_total"] / 2**30
        out.append("=" * self.width)
        out.append(
            f" 电脑占用   {cpu_model() or 'CPU'}   {ncpu} 线程   {mem_gb:.1f} GB"
            f"   运行 {sample['t']:.1f} s"
        )
        out.append("=" * self.width)

        total = sample["cpu"]
        out.append(
            f" CPU   [{self._bar(total, 24)}] {total:5.1f}%"
            f"   load {sample['load1']:.2f} / {sample['load5']:.2f} / {sample['load15']:.2f}"
            + warn_tag(total >= WARN_CPU)
        )
        # 每核占用按终端宽度换行，16 线程一行放不下
        per_line = max((self.width - 8) // 11, 4)
        core_texts = [f"c{index:02d}{value:4.0f}%"
                      for index, value in enumerate(sample["cores"])]
        for start in range(0, len(core_texts), per_line):
            prefix = "       " if start else "       每核 "
            out.append(prefix + " ".join(core_texts[start:start + per_line]))

        out.append(
            f" 内存  [{self._bar(sample['mem_pct'], 24)}] {sample['mem_pct']:5.1f}%"
            f"   {sample['mem_used'] / 2**30:.1f} / {mem_gb:.1f} GB"
            f"   可用 {sample['mem_avail'] / 2**30:.1f} GB"
            + warn_tag(100.0 - sample["mem_pct"] < 5.0, "物理内存见底")
        )
        if sample["swap_total"] > 0:
            out.append(
                f" 交换  [{self._bar(sample['swap_pct'], 24)}] {sample['swap_pct']:5.1f}%"
                f"   {sample['swap_used'] / 2**30:.1f} / {sample['swap_total'] / 2**30:.1f} GB"
                + warn_tag(sample["swap_pct"] >= WARN_SWAP, "swap 快满了，整机会变慢")
            )

        temp = sample["temp"]
        if math.isfinite(temp):
            temp_text = f"温度  {temp:.1f} °C ({sample['temp_src']})"
            if temp >= WARN_TEMP:
                temp_text += "   ⚠ 接近降频阈值"
        else:
            temp_text = "温度  不可用（没有可读的传感器）"
        out.append(" " + temp_text)

        disk_used = sample["disk_used_pct"]
        out.append(
            f" 磁盘  {self.settings.disk_path} {disk_used:5.1f}% 已用"
            f"   剩 {sample['disk_free'] / 2**30:.1f} GB"
            f"   读 {self._rate(sample['disk_r'])}  写 {self._rate(sample['disk_w'])}"
        )
        out.append(
            f" 网络  收 {self._rate(sample['net_rx'])}  发 {self._rate(sample['net_tx'])}"
            + self._loopback_note(sample)
        )
        battery = sample.get("battery")
        if battery is not None:
            percent, plugged = battery
            out.append(f" 电池  {percent:.0f}%{'  充电中' if plugged else '  未充电'}")

        groups, nav = self.recorder.snapshot_groups()
        if groups:
            out.append("-" * self.width)
            ncpu = sample.get("ncpu") or 0
            out.append(
                " 按分组（CPU% 相对单核，100% = 1 核）："
            )
            out.append(f"   {'分组':<12}{'CPU%':>9}{'核':>7}{'内存':>9}{'进程':>6}")
            ordered = sorted(groups.items(), key=lambda kv: -kv[1]["cpu"])
            for name, item in ordered:
                out.append(
                    f"   {name:<12}{item['cpu']:>9.1f}{item['cpu'] / 100:>7.2f}"
                    f"{fmt_bytes(item['rss']):>9}{item['count']:>6}"
                )
            share = nav.get("share", float("nan"))
            share_text = f"，占整机 {share * 100:.0f}%" if math.isfinite(share) else ""
            out.append(
                f"   {'导航栈合计':<12}{nav['cpu'] * 100:>9.1f}{nav['cpu']:>7.2f}"
                f"{fmt_bytes(nav['rss']):>9}{nav['count']:>6}   ← 仿真+RViz+Nav2+"
                f"定位+雷达+串口{share_text}"
            )

        if procs:
            out.append("-" * self.width)
            out.append(f" CPU 占用前 {len(procs)} 个进程"
                       "（% 相对单核，>100% 表示多核并行）：")
            out.append(f" {'PID':>7}  {'CPU%':>6}  {'内存':>8}  命令")
            for row in procs:
                out.append(
                    f" {row['pid']:>7}  {row['cpu']:>6.1f}  {fmt_bytes(row['rss']):>8}  "
                    f"{self._ellipsis(row['cmd'], self.width - 30)}"
                )
        out.append("=" * self.width)
        return "\n".join(out)

    @staticmethod
    def _loopback_note(sample):
        """lo 占大头时补一句 —— 单机跑 ROS 2 时 DDS 的流量全在回环上，别读成"网卡被打满"。"""
        total = sample.get("net_rx") or 0.0
        lo = sample.get("net_lo_rx") or 0.0
        if not math.isfinite(total) or not math.isfinite(lo) or total <= 0.01:
            return ""
        share = lo / total
        if share < 0.3:
            return ""
        return (f"   （lo 回环 {lo:.1f}/{sample.get('net_lo_tx', 0.0):.1f} MB/s，"
                f"占 {share * 100:.0f}%）")

    @staticmethod
    def _rate(value):
        if value is None or not math.isfinite(value):
            return "  —  MB/s"
        return f"{value:5.2f} MB/s"

    @staticmethod
    def _ellipsis(text, limit):
        limit = max(limit, 12)
        return text if len(text) <= limit else text[:limit - 1] + "…"

    # ------------------------------------------------------------ 主循环
    def _draw(self, text):
        if self.interactive:
            print("\033[H\033[J", end="")     # 原地刷新，不刷屏
        print(text, flush=True)

    def run(self, duration=0.0):
        deadline = time.monotonic() + duration if duration > 0 else None
        if self.settings.once:
            self._wait_ready()
            sample = self.recorder.latest()
            if sample is None:
                print("[错误] 没采到数据。", file=sys.stderr)
                return 1
            # --once 时进程表要现采一次：进程 CPU% 需要两次调用才有值
            procs = self.recorder.snapshot_procs()
            self._draw(self.render(sample, procs))
            return 0

        try:
            while True:
                sample = self.recorder.latest()
                if sample is not None:
                    self._draw(self.render(sample, self.recorder.snapshot_procs()))
                if deadline is not None and time.monotonic() >= deadline:
                    break
                time.sleep(max(self.settings.interval, 0.1))
        except KeyboardInterrupt:
            print()
        return 0

    def _wait_ready(self, timeout=4.0):
        """等到"能打得出有意义的快照"为止。

        两件事都必须等：
          * 至少两个采样点 —— 磁盘/网络是累计计数求差，只有第一个点时速率是未知
            （图上断线、屏上显示"—"）；
          * 进程表至少扫两次 —— psutil 的进程 CPU% 也是"两次调用之间"的占用，
            第一次扫到的进程永远是 0.0%。
        """
        deadline = time.monotonic() + timeout
        while self.recorder.sample_count() < 2 and time.monotonic() < deadline:
            time.sleep(0.05)
        if self.settings.procs > 0:
            while self.recorder.proc_scan_count() < 2 and time.monotonic() < deadline:
                time.sleep(0.05)


# =============================================================
# 画图模式
# =============================================================
class PlotView:
    """四个子图：CPU（含每核与温度）、内存、磁盘、网络。"""

    MAIN = dict(color="#1f77b4", linewidth=2.0)
    SECOND = dict(color="#d62728", linewidth=1.4, linestyle="--")
    THIRD = dict(color="#ff7f0e", linewidth=1.4, linestyle="--")
    TEMP = dict(color="#9467bd", linewidth=1.4, linestyle="-.", label="CPU 温度")

    def __init__(self, settings, recorder):
        import matplotlib.pyplot as plt

        self.plt = plt
        self.settings = settings
        self.recorder = recorder
        self._temp_in_legend = False

        self.fig, axes = plt.subplots(
            4, 1, figsize=(11.0, 9.0), sharex=True,
            gridspec_kw={"hspace": 0.30, "height_ratios": [1.5, 1.0, 1.0, 1.0]},
        )
        self.axes = list(axes)

        # ---- CPU：总量（粗线）+ 每核（细灰线）+ 温度（右轴）
        cpu_axis = self.axes[0]
        self.line_cpu, = cpu_axis.plot([], [], **self.MAIN, label="CPU 总占用")
        self.line_cores = []
        for index in range(64):              # 先建满，按实际核数决定显隐
            line, = cpu_axis.plot(
                [], [], color="#999999", linewidth=0.6, alpha=0.35,
                label="单核占用" if index == 0 else "_nolegend_",
            )
            self.line_cores.append(line)
        cpu_axis.set_ylabel("CPU [%]")
        cpu_axis.set_ylim(0, 105)
        cpu_axis.grid(True, alpha=0.3)
        cpu_axis.axhline(WARN_CPU, color="#d62728", linewidth=0.7,
                         linestyle=":", alpha=0.7)
        cpu_axis.legend(loc="upper left", fontsize=8, framealpha=0.9)
        cpu_axis.text(
            0.004, WARN_CPU + 3, f"{WARN_CPU:.0f}%", transform=cpu_axis.get_yaxis_transform(),
            fontsize=7, color="#d62728", va="bottom", ha="left",
        )

        # 温度走右轴；传感器不存在时把整条轴隐藏（不占位、不出假线）
        self.temp_axis = cpu_axis.twinx()
        self.line_temp, = self.temp_axis.plot([], [], **self.TEMP)
        # 纵轴标题只留单位：右边缘空间有限，"温度"二字交给图例说
        self.temp_axis.set_ylabel("°C", color="#9467bd", labelpad=2)
        self.temp_axis.tick_params(axis="y", colors="#9467bd", labelsize=8)
        self.temp_axis.set_ylim(20, 110)

        # 进程摘要放在标题下方的图外区域：原来是画在 CPU 轴右上角的浮动框，
        # 结果是它正好把"CPU 冲高那一段"盖住，看图的人会以为曲线在那里断了。
        self.proc_text = self.fig.text(
            0.995, 0.948, "", ha="right", va="top", fontsize=7.5, color="#333333",
        )

        # ---- 内存：物理 + swap
        mem_axis = self.axes[1]
        self.line_mem, = mem_axis.plot([], [], **self.MAIN, label="物理内存")
        self.line_swap, = mem_axis.plot([], [], **self.SECOND, label="swap")
        mem_axis.set_ylabel("内存 [%]")
        mem_axis.set_ylim(0, 105)
        mem_axis.grid(True, alpha=0.3)
        mem_axis.axhline(WARN_MEM, color="#d62728", linewidth=0.7,
                         linestyle=":", alpha=0.7)
        mem_axis.legend(loc="lower left", fontsize=8, ncol=2, framealpha=0.9)

        # ---- 磁盘 / 网络：MB/s
        disk_axis = self.axes[2]
        self.line_disk_r, = disk_axis.plot([], [], **self.MAIN, label="读")
        self.line_disk_w, = disk_axis.plot([], [], **self.THIRD, label="写")
        disk_axis.set_ylabel("磁盘 [MB/s]")
        disk_axis.grid(True, alpha=0.3)
        disk_axis.legend(loc="upper left", fontsize=8, ncol=2, framealpha=0.9)

        net_axis = self.axes[3]
        self.line_net_rx, = net_axis.plot([], [], **self.MAIN, label="收")
        self.line_net_tx, = net_axis.plot([], [], **self.THIRD, label="发")
        net_axis.set_ylabel("网络 [MB/s]")
        net_axis.set_xlabel("时间 [s]")
        net_axis.grid(True, alpha=0.3)
        net_axis.legend(loc="upper left", fontsize=8, ncol=2, framealpha=0.9)

        self.title = self.fig.suptitle("", fontsize=12, y=0.975)
        self.status = self.fig.text(0.5, 0.008, "", ha="center", va="bottom",
                                    fontsize=9, color="#333333")
        # 右侧要留出温度轴的刻度 + 纵轴标题，否则标题会被画到图外裁掉
        # 顶部留出两行给进程摘要，右下留出温度轴
        self.fig.subplots_adjust(left=0.085, right=0.895, top=0.895, bottom=0.075)

    # ------------------------------------------------------------ 绘制
    def redraw(self, _frame=None):
        import numpy as np

        samples = self.recorder.snapshot()
        settings = self.settings
        if samples:
            window_start = max(samples[-1]["t"] - settings.window_sec, 0.0)
            samples = [sample for sample in samples if sample["t"] >= window_start]
        t = np.array([sample["t"] for sample in samples], dtype=float)

        def series(key):
            return np.array(
                [sample[key] if math.isfinite(sample[key]) else np.nan
                 for sample in samples], dtype=float)

        if t.size:
            self.line_cpu.set_data(t, series("cpu"))
            self.line_mem.set_data(t, series("mem_pct"))
            self.line_swap.set_data(t, series("swap_pct"))
            self.line_disk_r.set_data(t, series("disk_r"))
            self.line_disk_w.set_data(t, series("disk_w"))
            self.line_net_rx.set_data(t, series("net_rx"))
            self.line_net_tx.set_data(t, series("net_tx"))

        # 每核曲线：核数可能变（离线核/虚拟化），多余的隐藏
        ncpu = samples[-1]["ncpu"] if samples else 0
        for index, line in enumerate(self.line_cores):
            if samples and index < ncpu:
                values = np.array(
                    [sample["cores"][index] if index < len(sample["cores"]) else np.nan
                     for sample in samples], dtype=float)
                line.set_data(t, values)
                line.set_visible(True)
            else:
                line.set_data([], [])
                line.set_visible(False)

        # 温度：没有可读传感器时整条右轴隐藏
        temps = series("temp")
        has_temp = bool(temps.size) and bool(np.isfinite(temps).any())
        self.temp_axis.set_visible(has_temp)
        if has_temp:
            self.line_temp.set_data(t, temps)
            top = float(np.nanmax(temps))
            self.temp_axis.set_ylim(min(20.0, top - 20.0), max(top + 10.0, 100.0))
            if not self._temp_in_legend:
                self._temp_in_legend = True
                handles, labels = self.axes[0].get_legend_handles_labels()
                handles.append(self.line_temp)
                labels.append("CPU 温度")
                self.axes[0].legend(handles, labels, loc="upper left",
                                    fontsize=8, framealpha=0.9)

        # 磁盘/网络纵轴：按窗口内峰值自适应（不压平数据，尖峰该看见就看见）
        for axis, keys, floor in (
            (self.axes[2], ("disk_r", "disk_w"), 1.0),
            (self.axes[3], ("net_rx", "net_tx"), 0.1),
        ):
            values = [value for key in keys for value in series(key)
                      if math.isfinite(value)]
            top = max(values) if values else 0.0
            axis.set_ylim(0, max(top * 1.25, floor))

        if t.size:
            self.axes[3].set_xlim(max(window_start - 0.5, 0.0), t[-1] + 0.5)
        else:
            self.axes[3].set_xlim(0.0, settings.window_sec)

        latest = samples[-1] if samples else None
        procs = self.recorder.snapshot_procs()
        self.title.set_text(
            f"本机占用 —— {cpu_model() or 'CPU'}   采样 {settings.sample_hz:g} Hz"
            f"   窗口 {settings.window_sec:g} s"
        )
        groups, nav = self.recorder.snapshot_groups()
        if groups:
            share = nav.get("share", float("nan"))
            share_text = f"{share * 100:.0f}% 整机" if math.isfinite(share) else "—"
            nav_line = (f"导航栈合计 {nav['cpu']:.2f} 核（{share_text}，"
                        f"{fmt_bytes(nav['rss'])}）"
                        + "   " + self._group_brief(groups))
            if procs:
                top = "   |   ".join(
                    f"{row['cpu']:5.1f}% {self._short_cmd(row['cmd'], 22)}"
                    for row in procs[:3])
                self.proc_text.set_text(nav_line + "\n" + "吃 CPU 前 3：" + top)
            else:
                self.proc_text.set_text(nav_line)
        else:
            self.proc_text.set_text(
                "进程表未采集（--procs 0）" if settings.procs == 0 else "")

        if latest is not None:
            temp_text = (f"{latest['temp']:.0f}°C" if math.isfinite(latest["temp"])
                         else "—")
            swap_text = (f"swap {latest['swap_pct']:.0f}%"
                         f" {latest['swap_used'] / 2**30:.1f}"
                         f"/{latest['swap_total'] / 2**30:.1f}G"
                         if latest["swap_total"] > 0 else "swap 无")
            lo_rx = latest.get("net_lo_rx")
            lo_note = ""
            if (math.isfinite(latest["net_rx"]) and math.isfinite(lo_rx)
                    and latest["net_rx"] > 1.0 and lo_rx / latest["net_rx"] > 0.3):
                lo_note = f"（其中 lo 回环 {lo_rx:.0f}/{latest['net_lo_tx']:.0f}）"
            nav_cores = latest.get("nav_cores", float("nan"))
            nav_text = (f"导航栈 {nav_cores:.2f} 核   "
                        if math.isfinite(nav_cores) else "")
            self.status.set_text(
                f"{nav_text}CPU {latest['cpu']:.0f}%   内存 {latest['mem_pct']:.0f}%"
                f" {latest['mem_used'] / 2**30:.1f}/{latest['mem_total'] / 2**30:.1f}G"
                f"   {swap_text}   温度 {temp_text}"
                f"   load {latest['load1']:.2f} {latest['load5']:.2f} {latest['load15']:.2f}"
                f"   网络 {latest['net_rx']:.0f}/{latest['net_tx']:.0f} MB/s{lo_note}"
                f"   {settings.disk_path} 剩 {latest['disk_free'] / 2**30:.0f}G"
                f"   进程 {len(procs) if procs else 0} 行"
            )
        # 不在这里调 canvas.draw_idle()：GUI 下 FuncAnimation 自己会重绘，
        # 存图时 savefig 也会重新渲染；加了等于每帧画两遍，白吃一到两成 CPU。
        return ()

    @staticmethod
    def _group_brief(groups, limit=4):
        """一行里放几个主要分组：gazebo 1.19 · nav2 0.92 · rviz 0.60 …（单位：核）"""
        ordered = sorted(
            ((name, item["cpu"] / 100.0) for name, item in groups.items()
             if name in NAV_GROUPS and item["cpu"] >= 1.0),
            key=lambda kv: -kv[1])
        if not ordered:
            return ""
        alias = {"仿真 Gazebo": "gazebo", "Nav2 导航栈": "nav2", "定位/里程计": "lio",
                 "雷达驱动": "lidar", "底盘串口": "serial", "RViz": "rviz"}
        parts = [f"{alias.get(name, name)} {cores:.2f}" for name, cores in ordered[:limit]]
        if len(ordered) > limit:
            parts.append("…")
        return " · ".join(parts)

    @staticmethod
    def _short_cmd(cmd, limit=28):
        """截断命令行：保留**开头**（能看出是哪个程序），别保留结尾的下标/参数。"""
        collapsed = " ".join(cmd.split())
        if len(collapsed) <= limit:
            return collapsed
        return collapsed[:limit - 1] + "…"

    def save(self, path):
        path = Path(path)
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            self.redraw()
            self.fig.savefig(path, dpi=130, bbox_inches="tight")
            print(f"[输出] 图已保存: {path}")
        except (OSError, ValueError) as exc:
            print(f"[警告] 保存图片失败 {path}: {exc}", file=sys.stderr)

    def close(self):
        self.plt.close(self.fig)


class Settings:
    def __init__(self, **kwargs):
        for key, value in kwargs.items():
            setattr(self, key, value)

    def __repr__(self):
        return f"Settings({self.__dict__})"


def build_settings(argv):
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("-h", "--help", action="store_true")
    parser.add_argument("--text", action="store_true")
    parser.add_argument("--once", action="store_true")
    parser.add_argument("--procs", type=int, default=6)
    parser.add_argument("--match", action="append", default=[])
    parser.add_argument("--interval", type=float, default=1.0)
    parser.add_argument("--window", type=float, default=60.0)
    parser.add_argument("--sample-hz", type=float, default=5.0)
    parser.add_argument("--plot-hz", type=float, default=1.0)
    parser.add_argument("--duration", type=float, default=0.0)
    parser.add_argument("--csv", default="")
    parser.add_argument("--procs-csv", default="",
                        help="把每次进程扫描写成 CSV，用于回溯某一刻谁在吃 CPU")
    parser.add_argument("--save", default="")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--disk-path", default="/")
    parser.add_argument("--iface", action="append", default=[],
                        help="只统计这些网卡（默认全部，含 lo；逗号分隔或重复给）")
    args = parser.parse_args(argv)

    if args.help:
        print_header_help(__file__)
        raise SystemExit(0)

    match = []
    for chunk in args.match:
        match.extend(token.strip() for token in chunk.split(",") if token.strip())

    interfaces = []
    for chunk in args.iface:
        interfaces.extend(token.strip() for token in chunk.split(",") if token.strip())

    return Settings(
        text=args.text,
        once=args.once,
        procs=max(args.procs, 0),
        proc_hz=1.0,
        match=match,
        interfaces=interfaces,
        interval=max(args.interval, 0.1),
        window_sec=max(args.window, 5.0),
        sample_hz=max(args.sample_hz, 0.5),
        plot_hz=max(args.plot_hz, 0.5),
        duration=max(args.duration, 0.0),
        csv_path=args.csv,
        procs_csv_path=args.procs_csv,
        save_path=args.save,
        headless=args.headless,
        disk_path=args.disk_path,
    )


def describe(settings, mode):
    lines = [
        "=" * 66,
        " 电脑占用监视 (debug_tools/system_monitor.py)",
        "=" * 66,
        f"  模式       : {mode}",
        f"  采样       : {settings.sample_hz:g} Hz（进程表 {settings.proc_hz:g} Hz，"
        f"{settings.procs} 行）",
    ]
    if settings.text:
        lines.append(f"  刷新周期   : {settings.interval:g} s"
                     + ("（--once 只打一屏）" if settings.once else ""))
    else:
        lines.append(f"  波形窗口   : {settings.window_sec:g} s，刷新 {settings.plot_hz:g} Hz")
    if settings.match:
        lines.append(f"  进程过滤   : {', '.join(settings.match)}")
    lines.append(f"  网络统计   : {', '.join(settings.interfaces) if settings.interfaces else '全部网卡（含 lo）'}")
    if settings.duration > 0:
        lines.append(f"  运行时长   : {settings.duration:g} s")
    lines.append(f"  磁盘统计   : {settings.disk_path}")
    if settings.csv_path:
        lines.append(f"  CSV        : {settings.csv_path}")
    if settings.procs_csv_path:
        lines.append(f"  进程 CSV   : {settings.procs_csv_path}")
    if settings.save_path:
        lines.append(f"  退出存图   : {settings.save_path}")
    lines.append("=" * 66)
    return "\n".join(lines)


# =============================================================
# 入口
# =============================================================
def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    settings = build_settings(argv)

    try:
        import psutil  # noqa: F401  只是探一下依赖
    except ImportError:
        print("[错误] 缺少 psutil。装一下：sudo apt install python3-psutil "
              "或 pip3 install psutil", file=sys.stderr)
        return 2

    gui_ok = bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
    # 没有窗口又没说 --headless/--save 时，直接退化成终端模式：ssh 上去也看得见
    text_mode = settings.text or (not gui_ok and not settings.headless and not settings.save_path)
    settings.text = text_mode
    if settings.text and settings.save_path:
        print("[提示] 终端模式下不画图，--save 被忽略。", file=sys.stderr)

    print(describe(settings, "终端" if text_mode else "波形窗口"))

    probe = Probe(settings)
    recorder = Recorder(settings)
    sampler = Sampler(settings, recorder, probe)

    if text_mode:
        sampler.start()
        try:
            return TextView(settings, recorder).run(settings.duration)
        finally:
            sampler.stop()
            recorder.close()
            print(f"[输出] 共 {recorder.count} 个采样点"
                  + (f"，CSV: {settings.csv_path}" if settings.csv_path else ""))

    # ---- 画图模式
    prepare_matplotlib_env()
    headless = settings.headless or not gui_ok
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

    if not setup_cjk_font():
        print("[提示] 没找到中文字体，图上中文可能显示成方块（不影响数据）。")

    view = PlotView(settings, recorder)
    sampler.start()
    deadline = time.monotonic() + settings.duration if settings.duration > 0 else None
    animation = None
    try:
        if headless:
            period = 1.0 / settings.plot_hz
            while True:
                view.redraw()
                if deadline is not None and time.monotonic() >= deadline:
                    break
                time.sleep(period)
        else:
            from matplotlib.animation import FuncAnimation

            def _update(_frame):
                if deadline is not None and time.monotonic() >= deadline:
                    plt.close(view.fig)
                    return ()
                return view.redraw(_frame)

            animation = FuncAnimation(
                view.fig, _update, interval=int(1000.0 / settings.plot_hz),
                blit=False, cache_frame_data=False,
            )
            plt.show(block=True)
        if settings.save_path:
            view.save(settings.save_path)
    except KeyboardInterrupt:
        print("\n[中断] Ctrl-C，保存结果后退出。")
        if settings.save_path:
            view.save(settings.save_path)
    finally:
        sampler.stop()
        if animation is not None:
            try:
                animation.event_source.stop()
            except Exception:
                pass
        recorder.close()
        print(f"[输出] 共 {recorder.count} 个采样点"
              + (f"，CSV: {settings.csv_path}" if settings.csv_path else ""))
        try:
            view.close()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
