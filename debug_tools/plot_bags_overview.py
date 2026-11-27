#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把多个 rosbag 的 map 系轨迹画到同一张先验地图上，便于横向对比。

单包细节（速度/角速度/指令）仍看 `plot_bag_trajectory.py`；这里只回答
"今天这段时间都跑到哪去了、几条轨迹是不是同一个圈"。

用法::

    ./debug_tools/plot_bags_overview.py --bags ~/srm_nav_27_data/bags/real_zloc_1127_16* ...
"""

import argparse
import os
import re
import sys
from datetime import datetime

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from plot_bag_trajectory import read_bag, read_pcd_xy  # noqa: E402


def bag_start(name):
    """从 real_zloc_1127_160324 这种目录名里取 16:03:24。"""
    m = re.search(r"_(\d{2})(\d{2})(\d{2})$", name)
    return "%s:%s:%s" % m.groups() if m else "??:??:??"


def normalize(traj):
    """平移+旋转到以起点为原点、起始朝向为 0，比较轨迹形状。"""
    yaw = np.radians(traj[:, 3])
    dx = traj[:, 1] - traj[0, 1]
    dy = traj[:, 2] - traj[0, 2]
    c, s = np.cos(-yaw[0]), np.sin(-yaw[0])
    return np.column_stack([traj[:, 0], c * dx - s * dy, s * dx + c * dy])


def main():
    ap = argparse.ArgumentParser(description="多个 rosbag 的 map 系轨迹叠加总览")
    ap.add_argument("--bags", nargs="+", required=True, help="rosbag 目录列表")
    ap.add_argument("--body-frame", default="gimbal_yaw")
    ap.add_argument("--map-pcd", default="maps/pcd/map_bundles/227to219_1125/map.pcd")
    ap.add_argument("--no-map", action="store_true")
    ap.add_argument("--out-dir", default="user_test_log")
    ap.add_argument("--title", default=None)
    args = ap.parse_args()

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    from matplotlib.lines import Line2D

    # 图里要写中文标题；找不到 CJK 字体就退回英文，避免出现一排方块。
    have_cjk = False
    names = {f.name for f in font_manager.fontManager.ttflist}
    for fam in ("Noto Sans CJK SC", "Noto Sans CJK JP", "Droid Sans Fallback"):
        if fam in names:
            plt.rcParams["font.sans-serif"] = [fam, "DejaVu Sans"]
            have_cjk = True
            break

    runs = []
    for bag in args.bags:
        name = os.path.basename(bag.rstrip("/"))
        try:
            (map_body, odom_body, speeds, n_mo,
             cmd_nav, cmd_chassis, plan_xy, plan_t) = read_bag(bag, args.body_frame)
        except SystemExit as e:
            print("[跳过] %s: %s" % (name, e))
            continue
        except Exception as e:  # 坏包不该拖垮整张图
            print("[跳过] %s: %s: %s" % (name, type(e).__name__, e))
            continue
        if len(map_body) < 2:
            print("[跳过] %s: map 系轨迹点不足" % name)
            continue
        path_len = float(np.hypot(np.diff(map_body[:, 1]), np.diff(map_body[:, 2])).sum())
        runs.append(dict(name=name, map_body=map_body, dur=float(map_body[-1, 0] - map_body[0, 0]),
                         path_len=path_len, start=bag_start(name)))
        print("%-24s %5.0f s  %6.1f m  start(map) (%.2f, %.2f)  end (%.2f, %.2f)"
              % (name, runs[-1]["dur"], path_len, map_body[0, 1], map_body[0, 2],
                 map_body[-1, 1], map_body[-1, 2]))
    if not runs:
        raise SystemExit("[错误] 没有可用的包")

    # tab10 的第 8 个颜色是灰色，会跟先验地图糊在一起；换成一组不带灰的定性色。
    palette = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd",
               "#8c564b", "#e377c2", "#17becf", "#bcbd22", "#393b79"]
    colors = [palette[i % len(palette)] for i in range(len(runs))]
    fig = plt.figure(figsize=(18, 10))
    gs = fig.add_gridspec(1, 2, width_ratios=[1.55, 1.0], wspace=0.16)

    # 左：map 系叠加，底图按所有轨迹的范围裁
    a = fig.add_subplot(gs[0, 0])
    pad = 6.0
    xs = np.concatenate([r["map_body"][:, 1] for r in runs])
    ys = np.concatenate([r["map_body"][:, 2] for r in runs])
    x0, x1 = xs.min() - pad, xs.max() + pad
    y0, y1 = ys.min() - pad, ys.max() + pad
    if not args.no_map and os.path.isfile(args.map_pcd):
        mx, my = read_pcd_xy(args.map_pcd)
        keep = (mx > x0) & (mx < x1) & (my > y0) & (my < y1)
        a.scatter(mx[keep], my[keep], c="0.72", s=2.0, zorder=1,
                  label="prior map (z 0.3~2.5 m)")
    handles = []
    for r, c in zip(runs, colors):
        t = r["map_body"]
        a.plot(t[:, 1], t[:, 2], color=c, lw=1.9, zorder=3)
        a.plot(t[0, 1], t[0, 2], "o", mfc="white", mec=c, mew=2.0, ms=10, zorder=4)
        a.plot(t[-1, 1], t[-1, 2], "s", color=c, mec="black", mew=0.6, ms=9, zorder=5)
        handles.append(Line2D([], [], color=c, lw=2.6,
                              label="%s  %-3.0fs %5.1fm" % (r["start"], r["dur"], r["path_len"])))
    a.set_xlim(x0, x1); a.set_ylim(y0, y1)
    a.set_aspect("equal"); a.grid(alpha=.3)
    a.set_xlabel("map x [m]"); a.set_ylabel("map y [m]")
    a.set_title("A. map-frame trajectories of all bags (○ start, □ end)")
    a.legend(handles=[Line2D([], [], color="0.72", marker=".", ls="", ms=6, label="prior map"),
                      handles[0]] + handles[1:],
             loc="upper right", fontsize=9, framealpha=.92, title="rec start / duration / path")

    # 右：对齐到起点，比形状（排除整体平移/错角）
    b = fig.add_subplot(gs[0, 1])
    for r, c in zip(runs, colors):
        n = normalize(r["map_body"])
        b.plot(n[:, 1], n[:, 2], color=c, lw=1.6)
        b.plot(n[-1, 1], n[-1, 2], "s", color=c, mec="black", mew=0.6, ms=8)
    b.plot(0, 0, "o", mfc="white", mec="black", ms=10, zorder=5)
    lim = max(np.abs(np.concatenate([normalize(r["map_body"])[:, 1:3].ravel() for r in runs]))) + 3
    b.set_xlim(-lim, lim); b.set_ylim(-lim, lim)
    b.set_aspect("equal"); b.grid(alpha=.3)
    b.set_xlabel("x [m] (start-aligned)"); b.set_ylabel("y [m]")
    b.set_title("B. start-aligned shapes (start at origin, heading -> 0°)")

    title = args.title or ("%s 之后录制的 %d 个包 · map 系轨迹" % (runs[0]["start"][:5], len(runs))
                           if have_cjk else
                           "trajectories recorded after %s (%d bags)" % (runs[0]["start"][:5], len(runs)))
    fig.suptitle(title, fontsize=14)
    fig.text(0.012, 0.012,
             "note: 160324 had no metadata.yaml and 172451 had a truncated last db3 page; "
             "both were rebuilt (172451 lost its last 3 messages).",
             fontsize=8, color="0.35")
    fig.subplots_adjust(bottom=0.09, top=0.9)
    os.makedirs(args.out_dir, exist_ok=True)
    stem = "overview_traj_%s" % datetime.now().strftime("%Y%m%d_%H%M%S")
    png = os.path.join(args.out_dir, stem + ".png")
    plt.savefig(png, dpi=120, bbox_inches="tight")
    print("\n输出:\n  %s" % png)
    return 0


if __name__ == "__main__":
    sys.exit(main())
