#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# =============================================================
# backend 纠正贡献分解：先验 / UWB / 回环 / 恢复 / 总效果
#
# 回答的问题
# ----------
#   · 先验配准一共把 map→odom 推了多少？净效果是多少？
#   · UWB 贡献了多少？（本车未接时显示 0/未接入）
#   · 回环贡献了多少？（由 --zloc-loop 独立开启，可与先验/UWB 同开）
#   · 恢复/其它（Scan Context + scan-to-map recovery）贡献了多少？
#   · 所有来源加起来，map→odom 的总效果是多少？
#
# 数据来源
# --------
#   直接读 plot_backend_correction.py 落盘的：
#     <name>_samples.csv   map→odom / 里程计 / 状态逐行
#     <name>_events.csv    每次修正一行（source / delta_t / detail）
#   不需要 ROS，不重新跑车；适合跑完后复盘。
#
# 口径（重要）
# ----------
#   1) 总效果 = 最后一个 map→odom ∘ 第一个 map→odom⁻¹，是**变换**的真实净变化；
#   2) 每个来源的 net Δx/Δy/Δyaw = 该来源认领的 TF 台阶增量（map→odom 的
#      T_after·T_before⁻¹）的向量和；小角度下与总效果相加闭合，脚本会打印
#      “残差”检查闭合程度；
#   3) Σ|Δp| 是**累计绝对修正量**，不是净效果；它可以很大，但净效果可能接近 0
#      （来回修正互相抵消）。这两个数要同时看；
#   4) 启动重定位发生在第一条 map→odom 之前，没有 before 基線，不计入增量；
#      脚本会单独提示。
#
# 典型输出（当前 227 场地那次自转测试）：
#   先验 113 步，Σ|Δp|≈1.40 m，净 x≈+0.285 m、y≈-0.015 m、yaw≈+0.36°
#   UWB  0 次（未接入）
#   回环 0 次（未加 --zloc-loop 时关闭；加了开关但无重访也可能为 0）
#   恢复 2 步，Σ|Δp|≈0.78 m，净 x≈-0.261 m、y≈+0.041 m、yaw≈-0.25°
#   总计 121 步，Σ|Δp|≈2.19 m，净 x≈+0.022 m、y≈+0.022 m、yaw≈+0.08°
#   先验的 +0.285 m 与恢复的 -0.261 m 几乎抵消 —— 总效果看起来“没动”，
#   但中间已经来回修正了两米多的绝对量。
#
# 用法
# ----
#   # 默认找 user_log/ 下最新的 *_samples.csv
#   ./debug_tools/backend_contribution_report.py
#
#   # 指定某次录制的 CSV
#   ./debug_tools/backend_contribution_report.py \
#     --samples user_log/pic_samples.csv \
#     --events  user_log/pic_events.csv
#
#   # 指定输出 / 只看终端表
#   ./debug_tools/backend_contribution_report.py --save /tmp/contrib.png
#   ./debug_tools/backend_contribution_report.py --stats-only
#
#   # 如果 backend 日志还在，可以让它读取 UWB/loop 状态（可选）
#   ./debug_tools/backend_contribution_report.py \
#     --backend-log ~/.ros/log/srm27_zlocalization_backend_node_*.log
# =============================================================

from __future__ import annotations

import argparse
import csv
import sys
from collections import defaultdict
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from _common import prepare_matplotlib_env, print_header_help, setup_cjk_font

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
DEFAULT_OUT_DIR = REPO_ROOT / "user_log"

# 复用主工具的数据结构、事件匹配与颜色约定。
sys.path.insert(0, str(SCRIPT_DIR))
import plot_backend_correction as core  # noqa: E402

SOURCE_ORDER = ("prior", "uwb", "loop", "prior_uwb", "recovery", "other")
SOURCE_LABELS = {
    "prior": "先验配准",
    "uwb": "UWB",
    "loop": "回环检测",
    "prior_uwb": "先验+UWB 联合",
    "recovery": "恢复/其它",
    "other": "其它",
    "unassigned": "未归属台阶",
}


# ---------------------------------------------------------------------------
# 读 CSV
# ---------------------------------------------------------------------------
def read_csv_rows(path: Path) -> list[dict]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def latest_samples_path() -> Path:
    candidates = sorted(
        DEFAULT_OUT_DIR.glob("*_samples.csv"),
        key=lambda path: path.stat().st_mtime,
        reverse=True,
    ) if DEFAULT_OUT_DIR.is_dir() else []
    if not candidates:
        raise FileNotFoundError(
            "没有找到 *_samples.csv；请先跑 plot_backend_correction.py，"
            "或用 --samples 指定")
    return candidates[0]


def pose_matrix_from_row(row: dict) -> np.ndarray:
    yaw = np.deg2rad(float(row["yaw_deg"]))
    c, s = np.cos(yaw), np.sin(yaw)
    matrix = np.eye(4)
    matrix[:3, :3] = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    matrix[:3, 3] = (float(row["x"]), float(row["y"]), float(row["z"]))
    return matrix


def load_recording(samples_path: Path, events_path: Path) -> core.Recording:
    args = SimpleNamespace(
        odom_hz=0.0,
        match_before=0.40,
        match_after=0.40,
    )
    recording = core.Recording(args)
    rows = read_csv_rows(samples_path)
    # 先 map→odom / 里程计，再 state；按 CSV 中 t 的写入顺序即可。
    for row in rows:
        kind = row.get("kind", "")
        if kind == "map_odom":
            recording.add_map_odom(
                float(row["t"]), int(row["stamp_ns"]), pose_matrix_from_row(row))
        elif kind == "localization_odometry":
            recording.add_pose(
                float(row["t"]), int(row["stamp_ns"]), pose_matrix_from_row(row))
    for row in rows:
        if row.get("kind") != "state":
            continue
        status = row.get("status", "")
        if not status:
            continue
        recording.add_state(
            float(row["t"]),
            core.StateSample(t=float(row["t"]), status=status,
                             epoch=int(row.get("epoch") or 0),
                             reason="", text=""))
    if events_path.is_file():
        for row in read_csv_rows(events_path):
            if not row.get("source") or not row.get("t"):
                continue
            recording.events.append(
                core.CorrectionEvent(
                    t=float(row["t"]),
                    source=row["source"],
                    delta_t=float(row["delta_t_m"] or 0.0),
                    delta_r_deg=float(row["delta_r_deg"] or 0.0),
                    detail=row.get("detail", ""),
                    raw="",
                )
            )
    recording.events.sort(key=lambda event: event.t)
    if not recording.samples:
        raise RuntimeError("samples CSV 里没有 map_odom 行")
    core.analyze_effects(recording)
    return recording


# ---------------------------------------------------------------------------
# 贡献分解
# ---------------------------------------------------------------------------
def build_contributions(recording: core.Recording) -> dict:
    """按来源把每个 map→odom 台阶的增量归并。

    返回：
      rows: {source: {count, sum_abs, dx, dy, dyaw, max}}
      total: {dx, dy, dyaw, sum_abs}
      residual: total - sum(rows)
      no_effect_events: 没有 TF 台阶可归属的事件（启动恢复等）
    """
    steps = core.build_tf_steps(recording)
    step_source = {}
    for event in recording.events:
        if event.effect is not None and event.effect.step_index >= 0:
            step_source[event.effect.step_index] = event.source

    rows = defaultdict(
        lambda: {"count": 0, "sum_abs": 0.0, "dx": 0.0, "dy": 0.0,
                 "dyaw": 0.0, "max": 0.0})
    for step in steps:
        first = recording.samples[step.index]
        second = recording.samples[step.index + 1]
        delta = second.T @ np.linalg.inv(first.T)
        dx, dy = float(delta[0, 3]), float(delta[1, 3])
        dyaw = np.rad2deg(np.arctan2(delta[1, 0], delta[0, 0]))
        norm = float(np.hypot(dx, dy))
        source = step_source.get(step.index, "unassigned")
        row = rows[source]
        row["count"] += 1
        row["sum_abs"] += norm
        row["dx"] += dx
        row["dy"] += dy
        row["dyaw"] += dyaw
        row["max"] = max(row["max"], norm)

    total_matrix = recording.samples[-1].T @ np.linalg.inv(recording.samples[0].T)
    total = {
        "count": len(steps),
        "sum_abs": sum(row["sum_abs"] for row in rows.values()),
        "dx": float(total_matrix[0, 3]),
        "dy": float(total_matrix[1, 3]),
        "dyaw": float(np.rad2deg(np.arctan2(total_matrix[1, 0], total_matrix[0, 0]))),
    }
    summed = {
        "dx": sum(row["dx"] for row in rows.values()),
        "dy": sum(row["dy"] for row in rows.values()),
        "dyaw": sum(row["dyaw"] for row in rows.values()),
    }
    residual = {key: total[key] - summed[key] for key in ("dx", "dy", "dyaw")}
    no_effect = [event for event in recording.events if event.effect is None]
    return {"rows": rows, "total": total, "residual": residual,
            "no_effect_events": no_effect}


def status_for(contributions: dict, *, uwb_mode: str, loop_mode: str,
               backend_log: Path | None) -> tuple[str, str]:
    """返回 (UWB 状态, 回环状态)；优先用 backend 日志，其次按事件推断。"""
    uwb_steps = (contributions["rows"].get("uwb", {}).get("count", 0)
                 + contributions["rows"].get("prior_uwb", {}).get("count", 0))
    loop_steps = contributions["rows"].get("loop", {}).get("count", 0)

    backend_uwb = None
    backend_loop = None
    if backend_log and backend_log.is_file():
        try:
            text = backend_log.read_text(encoding="utf-8", errors="ignore")
            for line in text.splitlines():
                if "unified backend ready:" not in line:
                    continue
                if "uwb=disabled" in line:
                    backend_uwb = "disabled"
                elif "uwb=enabled" in line:
                    backend_uwb = "enabled"
                if "loop=disabled" in line:
                    backend_loop = "disabled"
                elif "loop=enabled" in line:
                    backend_loop = "enabled"
                break
        except OSError:
            pass

    if uwb_mode == "off":
        uwb_text = "关闭（--uwb off）"
    elif uwb_mode == "on":
        uwb_text = ("启用，%d 个台阶" % uwb_steps) if uwb_steps else "启用，0 次"
    elif backend_uwb == "disabled":
        uwb_text = "未接入（backend: uwb=disabled）"
    elif backend_uwb == "enabled":
        uwb_text = ("启用，%d 个台阶" % uwb_steps) if uwb_steps else \
            "启用，0 次（未收到有效 UWB 观测）"
    elif uwb_steps:
        uwb_text = "%d 个台阶" % uwb_steps
    else:
        uwb_text = "未检出（可能未接入/无观测；加 --backend-log 可确认开关）"

    if loop_mode == "off":
        loop_text = "关闭（--loop off）"
    elif loop_mode == "on":
        loop_text = ("启用，%d 个台阶" % loop_steps) if loop_steps else "启用，0 次"
    elif loop_steps:
        loop_text = "%d 个台阶" % loop_steps
    elif backend_loop == "disabled":
        loop_text = "0 次（backend 明确 loop=disabled）"
    elif backend_loop == "enabled":
        loop_text = "0 次（backend loop=enabled，但未产生闭合）"
    else:
        loop_text = "0 次（未加 --zloc-loop 时关闭；无 backend 日志无法确认）"
    return uwb_text, loop_text


def print_table(contributions: dict, uwb_text: str, loop_text: str) -> None:
    rows = contributions["rows"]
    total = contributions["total"]
    residual = contributions["residual"]
    print("")
    print("=" * 92)
    print("backend 纠正贡献分解（baseline = 第一条 map→odom；Σ|Δp| 与净效果分开看）")
    print("=" * 92)
    print("%-16s %6s %10s %10s %10s %11s %9s  %s"
          % ("来源", "台阶数", "Σ|Δp|(m)", "net Δx(m)", "net Δy(m)",
             "net Δyaw(°)", "max(m)", "状态"))
    print("-" * 92)
    for source in SOURCE_ORDER:
        row = rows.get(source)
        if row is None:
            if source == "uwb":
                print("%-16s %6s %10s %10s %10s %11s %9s  %s"
                      % (SOURCE_LABELS[source], "0", "--", "--", "--", "--", "--",
                         uwb_text))
            elif source == "loop":
                print("%-16s %6s %10s %10s %10s %11s %9s  %s"
                      % (SOURCE_LABELS[source], "0", "--", "--", "--", "--", "--",
                         loop_text))
            continue
        print("%-16s %6d %10.4f %+10.4f %+10.4f %+11.3f %9.4f  %s"
              % (SOURCE_LABELS[source], row["count"], row["sum_abs"],
                 row["dx"], row["dy"], row["dyaw"], row["max"], ""))
    unassigned = rows.get("unassigned")
    if unassigned:
        print("%-16s %6d %10.4f %+10.4f %+10.4f %+11.3f %9.4f  %s"
              % (SOURCE_LABELS["unassigned"], unassigned["count"],
                 unassigned["sum_abs"], unassigned["dx"], unassigned["dy"],
                 unassigned["dyaw"], unassigned["max"], "没有事件认领的台阶"))
    print("-" * 92)
    print("%-16s %6d %10.4f %+10.4f %+10.4f %+11.3f %9s  %s"
          % ("总计", total["count"], total["sum_abs"], total["dx"], total["dy"],
             total["dyaw"], "--", "map→odom 首末的精确变化"))
    print("%-16s %6s %10s %+10.4f %+10.4f %+11.4f %9s  %s"
          % ("残差", "--", "--", residual["dx"], residual["dy"], residual["dyaw"],
             "--", "总效果 − 各来源向量和（应接近 0）"))
    print("")
    print("UWB：%s" % uwb_text)
    print("回环：%s" % loop_text)
    print("口径：Σ|Δp| 是累计绝对修正量；net 是各来源认领的 map→odom 台阶向量和。")
    print("      总效果是 T_last·T_first⁻¹ 的精确变化；残差用于检查分解闭合。")
    if contributions["no_effect_events"]:
        print("未计入增量的事件：%d 个（没有可归属的 before 台阶；可能是启动重定位、"
              "与其它来源同刻、或 TF 空档）：" % len(contributions["no_effect_events"]))
        for event in contributions["no_effect_events"][:5]:
            print("  t=%.2fs source=%s Δp=%.3fm Δyaw=%.3f°"
                  % (event.t, event.source, event.delta_t, event.delta_r_deg))
    else:
        print("未计入增量的事件：0 个")
    if total["sum_abs"] > 0.0 and abs(total["dx"]) + abs(total["dy"]) < total["sum_abs"] * 0.5:
        print("提示：净效果远小于累计绝对修正，说明各来源在来回抵消；看 net 与 Σ|Δp| 两列。")


# ---------------------------------------------------------------------------
# 画图
# ---------------------------------------------------------------------------
def plot_contributions(recording: core.Recording, contributions: dict,
                       png_path: Path, *, caption: bool = True,
                       title: str = "", uwb_text: str = "", loop_text: str = "") -> bool:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    rows = contributions["rows"]
    total = contributions["total"]
    steps = core.build_tf_steps(recording)
    step_source = {}
    for event in recording.events:
        if event.effect is not None and event.effect.step_index >= 0:
            step_source[event.effect.step_index] = event.source

    # 每个 source 的累计曲线
    cum = defaultdict(lambda: {"t": [0.0], "dx": [0.0], "dy": [0.0], "dyaw": [0.0]})
    t0 = recording.samples[0].t
    total_t, total_dx, total_dy, total_dyaw = [0.0], [0.0], [0.0], [0.0]
    for step in steps:
        first = recording.samples[step.index]
        second = recording.samples[step.index + 1]
        delta = second.T @ np.linalg.inv(first.T)
        dx, dy = float(delta[0, 3]), float(delta[1, 3])
        dyaw = float(np.rad2deg(np.arctan2(delta[1, 0], delta[0, 0])))
        source = step_source.get(step.index, "unassigned")
        bucket = cum[source]
        bucket["t"].append(step.end_t - t0)
        bucket["dx"].append(bucket["dx"][-1] + dx)
        bucket["dy"].append(bucket["dy"][-1] + dy)
        bucket["dyaw"].append(bucket["dyaw"][-1] + dyaw)
        total_t.append(step.end_t - t0)
        total_dx.append(total_dx[-1] + dx)
        total_dy.append(total_dy[-1] + dy)
        total_dyaw.append(total_dyaw[-1] + dyaw)

    fig, axes = plt.subplots(2, 2, figsize=(13.6, 10.8))
    ax_x, ax_y, ax_yaw, ax_bar = axes[0, 0], axes[0, 1], axes[1, 0], axes[1, 1]
    for axis, key, label in ((ax_x, "dx", "累计 Δx [m]"),
                             (ax_y, "dy", "累计 Δy [m]"),
                             (ax_yaw, "dyaw", "累计 Δyaw [°]")):
        for source in SOURCE_ORDER + ("unassigned",):
            if source not in cum:
                continue
            color = core.SOURCE_COLORS.get(source, "#7f7f7f")
            axis.step(cum[source]["t"], cum[source][key], where="post", lw=1.3,
                      color=color, label=SOURCE_LABELS.get(source, source))
        axis.step(total_t, {"dx": total_dx, "dy": total_dy, "dyaw": total_dyaw}[key],
                  where="post", lw=1.6, color="#000000", ls="--", label="全部台阶（总）")
        axis.set_xlabel("时间 [s]")
        axis.set_ylabel(label)
        axis.grid(True, alpha=0.28)
        axis.legend(fontsize=7.5, loc="best")
        axis.set_title(label + "（按来源累计）", fontsize=10, loc="left")

    # 柱状图：Σ|Δp| vs |net|
    categories, abs_values, net_values = [], [], []
    for source in ("prior", "uwb", "loop") + SOURCE_ORDER[3:] + ("unassigned",):
        row = rows.get(source)
        if (not row or row["count"] == 0) and source not in ("uwb", "loop"):
            continue
        categories.append(SOURCE_LABELS.get(source, source))
        abs_values.append(0.0 if not row else row["sum_abs"])
        net_values.append(0.0 if not row else float(np.hypot(row["dx"], row["dy"])))
    categories.append("总计")
    abs_values.append(total["sum_abs"])
    net_values.append(float(np.hypot(total["dx"], total["dy"])))
    x = np.arange(len(categories))
    width = 0.36
    ax_bar.bar(x - width / 2, abs_values, width, color="#4C78A8", label="Σ|Δp| 累计绝对")
    ax_bar.bar(x + width / 2, net_values, width, color="#F58518", label="|net| 净效果")
    ax_bar.set_xticks(x)
    ax_bar.set_xticklabels(categories, rotation=18, ha="right", fontsize=8)
    ax_bar.set_ylabel("位移 [m]")
    ax_bar.grid(True, axis="y", alpha=0.28)
    ax_bar.legend(fontsize=8)
    ax_bar.set_title("累计绝对修正 vs 净效果", fontsize=10, loc="left")
    for index, (absolute, net) in enumerate(zip(abs_values, net_values)):
        ax_bar.text(index - width / 2, absolute, "%.2f" % absolute,
                    ha="center", va="bottom", fontsize=7)
        ax_bar.text(index + width / 2, net, "%.2f" % net,
                    ha="center", va="bottom", fontsize=7)

    fig.suptitle(title or
                 "backend 纠正贡献分解 | 总效果 %.3f m / %.3f° | Σ|Δp| %.2f m"
                 % (np.hypot(total["dx"], total["dy"]), total["dyaw"],
                    total["sum_abs"]), fontsize=13, y=0.985)
    if caption:
        caption_text = (
            "口径：总效果=末帧 map→odom × inv(首帧 map→odom)；各来源 net=认领的 TF 台阶增量向量和，"
            "残差应接近 0。Σ|Δp| 是累计绝对修正，可能与净效果差很多（来回抵消）。"
            "启动重定位在首条 map→odom 之前，无 before 基线，不计入增量。"
            "UWB: %s；回环: %s。对应来源为 0 时不是漏统计。"
            % (uwb_text or "见终端表", loop_text or "见终端表")
        )
        fig.text(0.012, 0.008, caption_text, fontsize=8.2, va="bottom", ha="left",
                 color="#333333")
        fig.subplots_adjust(left=0.07, right=0.97, top=0.94, bottom=0.08,
                            hspace=0.34, wspace=0.22)
    else:
        fig.subplots_adjust(left=0.07, right=0.97, top=0.95, bottom=0.04,
                            hspace=0.34, wspace=0.22)
    try:
        png_path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(str(png_path), dpi=140, bbox_inches="tight")
        return True
    except OSError as error:
        print("[错误] 图保存失败 %s: %s" % (png_path, error), file=sys.stderr)
        return False
    finally:
        plt.close(fig)


# ---------------------------------------------------------------------------
# CSV / 主入口
# ---------------------------------------------------------------------------
def save_contributions_csv(contributions: dict, path: Path,
                           uwb_text: str, loop_text: str) -> bool:
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(["source", "count", "sum_abs_m", "net_dx_m",
                             "net_dy_m", "net_dyaw_deg", "max_step_m", "status"])
            for source in SOURCE_ORDER + ("unassigned",):
                row = contributions["rows"].get(source)
                if not row:
                    if source == "uwb":
                        writer.writerow([source, 0, "", "", "", "", "", uwb_text])
                    elif source == "loop":
                        writer.writerow([source, 0, "", "", "", "", "", loop_text])
                    continue
                writer.writerow([source, row["count"], "%.9f" % row["sum_abs"],
                                 "%.9f" % row["dx"], "%.9f" % row["dy"],
                                 "%.9f" % row["dyaw"], "%.9f" % row["max"], ""])
            total = contributions["total"]
            writer.writerow(["total", total["count"], "%.9f" % total["sum_abs"],
                             "%.9f" % total["dx"], "%.9f" % total["dy"],
                             "%.9f" % total["dyaw"], "", "map->odom first->last"])
            residual = contributions["residual"]
            writer.writerow(["residual", "", "", "%.9f" % residual["dx"],
                             "%.9f" % residual["dy"], "%.9f" % residual["dyaw"], "",
                             "total - sum(sources)"])
        return True
    except OSError as error:
        print("[错误] CSV 保存失败 %s: %s" % (path, error), file=sys.stderr)
        return False


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="backend 纠正贡献分解：先验 / UWB / 回环 / 恢复 / 总效果",
        add_help=False)
    parser.add_argument("-h", "--help", action="store_true")
    parser.add_argument("--samples", default="",
                        help="plot_backend_correction.py 的 *_samples.csv；"
                             "不写则找 user_log/ 下最新一份")
    parser.add_argument("--events", default="",
                        help="对应的 *_events.csv；不写则按 _samples.csv 推导")
    parser.add_argument("--backend-log", default="",
                        help="可选：backend 节点日志；用来确认 UWB/loop 是 disabled 还是 enabled")
    parser.add_argument("--uwb", choices=("auto", "on", "off"), default="auto",
                        help="UWB 状态显示（auto: 按事件/backend 日志推断）")
    parser.add_argument("--loop", choices=("auto", "on", "off"), default="auto",
                        help="回环状态显示（auto: 按事件/backend 日志推断）")
    parser.add_argument("--save", default="",
                        help="贡献分解 PNG；不写则与 samples 同目录 <stem>_contributions.png")
    parser.add_argument("--csv", default="",
                        help="贡献分解 CSV；不写则与 PNG 同名 .csv")
    parser.add_argument("--title", default="", help="图标题（可选）")
    parser.add_argument("--no-caption", action="store_true",
                        help="不把口径说明画进图里")
    parser.add_argument("--stats-only", action="store_true",
                        help="只打印表，不出图")
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.help:
        print_header_help(__file__)
        return 0

    samples_path = Path(args.samples).expanduser() if args.samples else latest_samples_path()
    if not samples_path.is_file():
        print("[错误] samples CSV 不存在: %s" % samples_path, file=sys.stderr)
        return 2
    if args.events:
        events_path = Path(args.events).expanduser()
    elif samples_path.name.endswith("_samples.csv"):
        events_path = samples_path.with_name(
            samples_path.name[:-len("_samples.csv")] + "_events.csv")
    else:
        events_path = samples_path.with_name(samples_path.stem + "_events.csv")
    if not events_path.is_file():
        print("[警告] events CSV 不存在，所有台阶都会归到“未归属”: %s" % events_path,
              file=sys.stderr)

    try:
        recording = load_recording(samples_path, events_path)
    except Exception as error:  # noqa: BLE001 - 现场工具给出可读错误
        print("[错误] 读取 CSV 失败: %s" % error, file=sys.stderr)
        return 2
    contributions = build_contributions(recording)
    backend_log = Path(args.backend_log).expanduser() if args.backend_log else None
    uwb_text, loop_text = status_for(
        contributions, uwb_mode=args.uwb, loop_mode=args.loop, backend_log=backend_log)

    print_table(contributions, uwb_text, loop_text)

    if samples_path.name.endswith("_samples.csv"):
        base = samples_path.with_name(samples_path.name[:-len("_samples.csv")])
    else:
        base = samples_path.with_suffix("")
    png_path = Path(args.save).expanduser() if args.save else \
        base.with_name(base.name + "_contributions.png")
    csv_path = Path(args.csv).expanduser() if args.csv else png_path.with_suffix(".csv")
    if save_contributions_csv(contributions, csv_path, uwb_text, loop_text):
        print("\n贡献表 CSV: %s" % csv_path)
    if not args.stats_only:
        prepare_matplotlib_env()
        cjk = setup_cjk_font()
        if not cjk:
            print("[提示] 没找到中文字体，图上中文可能显示为方块。", file=sys.stderr)
        if plot_contributions(recording, contributions, png_path,
                              caption=not args.no_caption, title=args.title,
                              uwb_text=uwb_text, loop_text=loop_text):
            print("贡献分解图: %s" % png_path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
