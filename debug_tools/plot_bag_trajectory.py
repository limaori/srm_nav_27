#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从一个 rosbag 还原轨迹，并叠到先验点云上看它跑到哪去了。

为什么要单独做这件事
====================
"定位飘了"这种判断光看 TF 数字不够直观，得把**轨迹画在地图上**：

- **odom 系轨迹**：前端自己认为走了哪条路。它不受 backend 影响，可以拿来判断
  "车实际怎么走的"（前提是 LIO 自己没崩）。
- **map 系轨迹** = `map→odom × odom→body`：backend 输出给规划器/导航的那条。
  两条轨迹**形状一致但整体错位/错角**时，说明错的是 `map→odom`（全局修正或锁存初值）；
  形状本身变形时，说明错的是前端。
- 锁存模式（`--zloc-prior-once`）下 `map→odom` 恒定，map 系轨迹就是 odom 轨迹的一次刚体
  变换，所以它相对先验图的位置**完全取决于 5.8 s 那次匹配**，之后不再纠偏。

输入：带 metadata 的 rosbag（`/tf` 必须有；有 `/odometry` 时用它算速度）。
输出：`user_test_log/traj_<bag名>_<时间>.png` 与同名 `.csv`（逐点 map/odom 位姿）。

用法::

    ./debug_tools/plot_bag_trajectory.py --bag ~/srm_nav_27_data/bags/real_zloc_1127_012727
    ./debug_tools/plot_bag_trajectory.py --bag ... --map-pcd maps/pcd/map_bundles/227to219_1125/map.pcd
    ./debug_tools/plot_bag_trajectory.py --bag ... --body-frame gimbal_yaw --no-map
"""

import argparse
import math
import os
import sys
from datetime import datetime

import numpy as np


def yaw_of(q):
    """四元数 (x,y,z,w) → 偏航角（度）。"""
    x, y, z, w = [float(v) for v in q]
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return math.degrees(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def compose_2d(a, b):
    """把 (yaw, x, y) 的两段变换合成（map←odom 与 odom←body）。"""
    ya, xa, ya_ = math.radians(a[0]), a[1], a[2]
    yb, xb, yb_ = math.radians(b[0]), b[1], b[2]
    c, s = math.cos(ya), math.sin(ya)
    return ((a[0] + b[0] + 180.0) % 360.0 - 180.0, xa + c * xb - s * yb_, ya_ + s * xb + c * yb_)


def read_bag(path, body_frame):
    """读 bag，返回轨迹、速度、cmd_vel 与最后一帧 /plan。

    返回 `(map_body, odom_body, speeds, n_mo, cmd_nav, cmd_chassis, plan_xy, plan_t)`：
    - `map_body/odom_body` 每行统一为 **(t, x, y, yaw_deg)**；
    - `speeds` 为 `(t, |v|)`；
    - `cmd_nav/cmd_chassis` 为 `(t, vx, vy, wz)`，对应 `/cmd_vel_nav_body`、
      `/cmd_vel_chassis`（Twist，无 header，用 bag 接收时刻）；
    - `plan_xy` 为最后一帧 `/plan` 的 `(N,2)` map 系路径，没有则为 None。
    列序写错一次就够把 300 m 算成 30 km，所以这里在函数头把约定钉死。
    """
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if "/tf" not in types:
        raise SystemExit("[错误] 这个包里没有 /tf，无法还原轨迹")

    map_odom, odom_body, speeds = [], [], []
    cmd_nav, cmd_chassis = [], []
    plan_xy, plan_t = None, None
    first = None
    while reader.has_next():
        topic, raw, stamp = reader.read_next()
        if first is None:
            first = stamp
        t = (stamp - first) * 1e-9
        if topic == "/tf":
            msg = deserialize_message(raw, get_message(types[topic]))
            for tr in msg.transforms:
                r = tr.transform.rotation
                v = (tr.transform.translation.x, tr.transform.translation.y,
                     yaw_of((r.x, r.y, r.z, r.w)))
                if tr.header.frame_id == "map" and tr.child_frame_id == "odom":
                    map_odom.append((t,) + v)
                elif tr.header.frame_id == "odom" and tr.child_frame_id == body_frame:
                    odom_body.append((t,) + v)
        elif topic == "/odometry":
            msg = deserialize_message(raw, get_message(types[topic]))
            tw = msg.twist.twist.linear
            speeds.append((t, math.hypot(tw.x, tw.y)))
        elif topic in ("/cmd_vel_nav_body", "/cmd_vel_chassis"):
            msg = deserialize_message(raw, get_message(types[topic]))
            row = (t, msg.linear.x, msg.linear.y, msg.angular.z)
            (cmd_nav if topic == "/cmd_vel_nav_body" else cmd_chassis).append(row)
        elif topic == "/plan":
            msg = deserialize_message(raw, get_message(types[topic]))
            if msg.poses:
                plan_xy = np.asarray(
                    [(p.pose.position.x, p.pose.position.y) for p in msg.poses], dtype=float)
                plan_t = t
    if not odom_body:
        raise SystemExit("[错误] 包里没有 odom→%s，无法还原轨迹" % body_frame)

    map_odom.sort(key=lambda r: r[0])
    odom_body.sort(key=lambda r: r[0])
    # map→odom 是低频/阶梯信号：对每个 body 时刻取"最近一次已知值"，不插值，
    # 避免把两次跳变之间伪造出斜线。
    mo = np.asarray(map_odom, dtype=float) if map_odom else np.zeros((0, 4))
    map_body = []
    for t, x, y, yaw in odom_body:
        # map→odom 还没有的时间段（初始化之前）不能当成单位阵去画，否则 map 系轨迹的
        # 开头会变成 odom 系轨迹，起点坐标/yaw 全是错的。直接丢掉这些点。
        if len(mo) and t < mo[0, 0]:
            continue
        base = (0.0, 0.0, 0.0)                      # (yaw, x, y)
        if len(mo):
            i = int(np.searchsorted(mo[:, 0], t, side="right")) - 1
            if i >= 0:
                base = (mo[i, 3], mo[i, 1], mo[i, 2])
        out_yaw, out_x, out_y = compose_2d(base, (yaw, x, y))
        map_body.append((t, out_x, out_y, out_yaw))
    speeds.sort(key=lambda r: r[0])
    cmd_nav.sort(key=lambda r: r[0])
    cmd_chassis.sort(key=lambda r: r[0])
    return (np.asarray(map_body, dtype=float), np.asarray(odom_body, dtype=float),
            np.asarray(speeds, dtype=float), len(mo),
            np.asarray(cmd_nav, dtype=float).reshape(-1, 4),
            np.asarray(cmd_chassis, dtype=float).reshape(-1, 4),
            plan_xy, plan_t)


def read_pcd_xy(path):
    """读 pcd 的 xy 与高度，只留 0.3~2.5 m 的墙/实体点。"""
    with open(path, "rb") as f:
        hdr = {}
        while True:
            line = f.readline().decode("ascii", "replace").strip()
            if not line or line.startswith("#"):
                continue
            k, _, v = line.partition(" ")
            hdr[k.upper()] = v.strip()
            if k.upper() == "DATA":
                break
        payload = f.read()
    fields = hdr["FIELDS"].split()
    sizes = [int(x) for x in hdr["SIZE"].split()]
    types = hdr["TYPE"].split()
    dt = {"F": {4: "f4", 8: "f8"}, "U": {1: "u1", 2: "u2", 4: "u4", 8: "u8"},
          "I": {1: "i1", 2: "i2", 4: "i4", 8: "i8"}}
    dtype = np.dtype([(f, "<" + dt[t][s]) for f, t, s in zip(fields, types, sizes)])
    a = np.frombuffer(payload, dtype=dtype, count=int(hdr["POINTS"]))
    x, y, z = a["x"].astype(float), a["y"].astype(float), a["z"].astype(float)
    m = (z > 0.3) & (z < 2.5)
    return x[m], y[m]


def main():
    ap = argparse.ArgumentParser(description="从 rosbag 还原轨迹并叠到先验点云")
    ap.add_argument("--bag", required=True, help="rosbag 目录（需要 metadata.yaml）")
    ap.add_argument("--body-frame", default="gimbal_yaw", help="odom 下的车体系（默认 gimbal_yaw）")
    ap.add_argument("--map-pcd", default="maps/pcd/map_bundles/227to219_1125/map.pcd",
                    help="先验点云（默认 227to219_1125 的显示云）")
    ap.add_argument("--no-map", action="store_true", help="不叠先验点云")
    ap.add_argument("--out-dir", default="user_test_log")
    args = ap.parse_args()

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    (map_body, odom_body, speeds, n_mo,
     cmd_nav, cmd_chassis, plan_xy, plan_t) = read_bag(args.bag, args.body_frame)
    t = map_body[:, 0]
    path_len = float(np.hypot(np.diff(map_body[:, 1]), np.diff(map_body[:, 2])).sum())
    print("包      : %s" % args.bag)
    print("时长    : %.1f s   轨迹点 %d   map→odom 帧 %d" % (t[-1] - t[0], len(t), n_mo))
    print("行程    : %.2f m" % path_len)
    print("起点(map): (%.2f, %.2f) %.1f°   终点: (%.2f, %.2f) %.1f°"
          % (map_body[0, 1], map_body[0, 2], map_body[0, 3],
             map_body[-1, 1], map_body[-1, 2], map_body[-1, 3]))
    print("odom 系行程: %.2f m" % float(np.hypot(np.diff(odom_body[:, 1]),
                                                np.diff(odom_body[:, 2])).sum()))
    print("指令      : /cmd_vel_nav_body %d 帧, /cmd_vel_chassis %d 帧"
          % (len(cmd_nav), len(cmd_chassis)))
    print("全局路径  : %s" % ("最后一帧 /plan %d 点 @ %.1fs" % (len(plan_xy), plan_t)
                                if plan_xy is not None else "包里没有 /plan"))

    # 统一到 10 Hz 再算速率：TF 的到达间隔在 0.1~0.4 s 之间跳，直接 diff 会把
    # "0.4 s 走 60°" 算成 3400°/s 的假尖峰（本工具的 C/D 面板第一版就栽在这）。
    tu = np.arange(odom_body[0, 0], odom_body[-1, 0], 0.1)
    oyaw = np.degrees(np.unwrap(np.radians(odom_body[:, 3])))
    oy = np.interp(tu, odom_body[:, 0], oyaw)
    ox = np.interp(tu, odom_body[:, 0], odom_body[:, 1])
    oy2 = np.interp(tu, odom_body[:, 0], odom_body[:, 2])
    rate_u = np.diff(oy) / 0.1
    step_u = np.hypot(np.diff(ox), np.diff(oy2)) / 0.1

    fig = plt.figure(figsize=(17, 17))
    gs = fig.add_gridspec(3, 2, height_ratios=[1.35, 1.0, 0.9],
                          hspace=0.30, wspace=0.22)
    # A: map 系轨迹 + 先验图 + 最后一帧全局路径
    a = fig.add_subplot(gs[0, 0])
    # 先按轨迹算窗口：先验图有 51×82 m，全画上去会把 10 m 的轨迹压成一个点。
    pad = 6.0
    x0, x1 = map_body[:, 1].min() - pad, map_body[:, 1].max() + pad
    y0, y1 = map_body[:, 2].min() - pad, map_body[:, 2].max() + pad
    if not args.no_map and os.path.isfile(args.map_pcd):
        mx, my = read_pcd_xy(args.map_pcd)
        keep = (mx > x0) & (mx < x1) & (my > y0) & (my < y1)
        a.scatter(mx[keep], my[keep], c="0.72", s=2.0, label="prior map (z 0.3~2.5 m)")
    sc = a.scatter(map_body[:, 1], map_body[:, 2], c=t, cmap="viridis", s=6, label="map→body")
    if plan_xy is not None and len(plan_xy) > 1:
        a.plot(plan_xy[:, 0], plan_xy[:, 1], color="tab:red", lw=1.2, ls="--",
               label="global plan (/plan, last @ %.1fs)" % plan_t)
    a.plot(map_body[0, 1], map_body[0, 2], "g^", ms=13, label="start")
    a.plot(map_body[-1, 1], map_body[-1, 2], "rs", ms=11, label="end")
    a.set_xlim(x0, x1); a.set_ylim(y0, y1)
    a.set_title("A. map frame trajectory (%.1f m, start->end %.1f m apart)"
                % (path_len, float(np.hypot(*(map_body[-1, 1:3] - map_body[0, 1:3])))))
    a.set_xlabel("x [m]"); a.set_ylabel("y [m]"); a.set_aspect("equal"); a.grid(alpha=.3); a.legend(loc="best")
    plt.colorbar(sc, ax=a, shrink=.8, label="t [s]")

    # B: odom 系轨迹（不受 backend 影响）
    b = fig.add_subplot(gs[0, 1])
    sc2 = b.scatter(odom_body[:, 1], odom_body[:, 2], c=odom_body[:, 0], cmap="viridis", s=6)
    b.plot(odom_body[0, 1], odom_body[0, 2], "g^", ms=13)
    b.plot(odom_body[-1, 1], odom_body[-1, 2], "rs", ms=11)
    b.set_title("B. odom frame trajectory (frontend only, %.1f m)" % path_len)
    b.set_xlabel("x [m]"); b.set_ylabel("y [m]"); b.set_aspect("equal"); b.grid(alpha=.3)
    plt.colorbar(sc2, ax=b, shrink=.8, label="t [s]")

    # C: yaw 走势
    c = fig.add_subplot(gs[1, 0])
    c.plot(tu[1:], rate_u, lw=.8, label="yaw rate (10 Hz resampled)")
    c.axhline(0, color="0.6", lw=.6)
    c.set_title("C. yaw rate: peak %.0f deg/s, median %.0f deg/s, total %.0f deg (%.1f rev)"
                % (np.abs(rate_u).max(), np.median(np.abs(rate_u)),
                   np.abs(np.diff(oy)).sum(), np.abs(np.diff(oy)).sum() / 360.0))
    c.set_xlabel("t [s]"); c.set_ylabel("deg/s"); c.grid(alpha=.3); c.legend(loc="best")

    # D: 速度
    d = fig.add_subplot(gs[1, 1])
    d.plot(tu[1:], step_u, lw=.8, label="path speed (10 Hz resampled)")
    if len(speeds):
        d.plot(speeds[:, 0], speeds[:, 1], lw=.6, alpha=.6, label="/odometry twist |v|")
    d.set_title("D. speed: max %.1f m/s, median %.2f m/s" % (step_u.max(), np.median(step_u)))
    d.set_xlabel("t [s]"); d.set_ylabel("m/s"); d.grid(alpha=.3); d.legend(loc="best")

    # E: 指令 vs 实际。拐角那个"反向折弯"看 vx 符号就能分清：
    #    控制器主动倒车 → /cmd_vel_nav_body vx 变负；底盘没跟上 → 指令一直为正。
    e = fig.add_subplot(gs[2, :])
    if len(cmd_nav):
        e.plot(cmd_nav[:, 0], cmd_nav[:, 1], lw=1.0, label="/cmd_vel_nav_body vx")
    if len(cmd_chassis):
        e.plot(cmd_chassis[:, 0], cmd_chassis[:, 1], lw=1.0, label="/cmd_vel_chassis vx")
        e.plot(cmd_chassis[:, 0], cmd_chassis[:, 3], lw=0.8, alpha=0.65,
               label="/cmd_vel_chassis wz [rad/s]")
    e.plot(tu[1:], step_u, lw=0.8, alpha=0.7, label="actual |v| [m/s]")
    e.axhline(0, color="0.6", lw=0.6)
    e.set_xlabel("t [s]"); e.set_ylabel("m/s, rad/s")
    e.set_title("E. command vs actual: negative vx = controller commanded reverse"
                if (len(cmd_nav) or len(cmd_chassis)) else
                "E. cmd_vel topics not in bag (record /cmd_vel_nav_body, /cmd_vel_chassis)")
    e.grid(alpha=0.3); e.legend(loc="best", ncol=2)

    fig.suptitle("trajectory from %s" % os.path.basename(args.bag.rstrip("/")), fontsize=13)
    plt.tight_layout()
    os.makedirs(args.out_dir, exist_ok=True)
    stem = "traj_%s_%s" % (os.path.basename(args.bag.rstrip("/")), datetime.now().strftime("%Y%m%d_%H%M%S"))
    png = os.path.join(args.out_dir, stem + ".png")
    csv = os.path.join(args.out_dir, stem + ".csv")
    plt.savefig(png, dpi=110)
    with open(csv, "w", encoding="utf-8") as f:
        f.write("t,map_x,map_y,map_yaw_deg,odom_x,odom_y,odom_yaw_deg\n")
        n = min(len(map_body), len(odom_body))
        for i in range(n):
            f.write("%.4f,%.4f,%.4f,%.3f,%.4f,%.4f,%.3f\n"
                    % (map_body[i, 0], map_body[i, 1], map_body[i, 2], map_body[i, 3],
                       odom_body[i, 1], odom_body[i, 2], odom_body[i, 3]))
    print("\n输出:\n  %s\n  %s" % (png, csv))
    return 0


if __name__ == "__main__":
    sys.exit(main())
