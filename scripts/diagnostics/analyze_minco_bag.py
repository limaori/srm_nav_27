#!/usr/bin/env python3
# Copyright 2026 SRM
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""从 rosbag(含未写完 metadata 的 db3) 逐拍复盘 MINCO：

每个规划周期（/FollowPath/minco_trajectory，约 10 Hz）输出一行：

  机器人位置/速度、命令速度、轨迹段数/总时长、轨迹自身的最大速度与最大加速度、
  轨迹相对参考路径(planning_input_path)的横向偏离、轨迹到局部代价地图 LETHAL 格的最近距离、
  diagnostics 里的 planning_result / minimum_clearance / stop_reason / speed_repair_count 等，
  以及两个"跳变"指标：
    start_gap  = 轨迹起点与机器人当前位置的距离（大 = RViz 里红线不接车）
    lat1s      = 轨迹在 1 m 弧长处相对车体纵轴的横向偏移，逐拍比较 → 规划形状的抖动

用法:
  python3 analyze_minco_bag.py <bag目录> [--window 5 30] [--csv out.csv]

不依赖 metadata.yaml：直接读 sqlite（录包还在进行时也能读）。
"""

import argparse
import math
import sqlite3
import sys
from pathlib import Path

from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path as RosPath
from rclpy.serialization import deserialize_message

TYPES = {
    "/FollowPath/minco_trajectory": RosPath,
    "/FollowPath/planning_input_path": RosPath,
    "/odometry": Odometry,
    "/cmd_vel_controller": Twist,
    "/cmd_vel_chassis": Twist,
    "/FollowPath/diagnostics": DiagnosticArray,
    "/local_costmap/costmap": OccupancyGrid,
}


def load_messages(bag: Path):
    rows = []
    for db in sorted(bag.glob("*.db3")):
        con = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
        names = {tid: name for tid, name in con.execute("select id, name from topics")}
        for topic_id, stamp, data in con.execute(
                "select topic_id, timestamp, data from messages order by timestamp"):
            name = names.get(topic_id)
            if name in TYPES:
                rows.append((stamp, name, data))
        con.close()
    rows.sort(key=lambda r: r[0])
    return rows


def path_points(msg):
    return [(p.pose.position.x, p.pose.position.y,
             p.header.stamp.sec + p.header.stamp.nanosec * 1e-9) for p in msg.poses]


def yaw_of(msg):
    q = msg.pose.pose.orientation
    return 2.0 * math.atan2(q.z, q.w)


def dist_to_polyline(p, pts):
    best = float("inf")
    for a, b in zip(pts, pts[1:]):
        vx, vy = b[0] - a[0], b[1] - a[1]
        l2 = vx * vx + vy * vy
        t = 0.0 if l2 <= 1e-12 else max(0.0, min(1.0, ((p[0] - a[0]) * vx + (p[1] - a[1]) * vy) / l2))
        best = min(best, math.hypot(p[0] - (a[0] + t * vx), p[1] - (a[1] + t * vy)))
    return best


def lethal_points(grid: OccupancyGrid, stride=2):
    info = grid.info
    res, w, h = info.resolution, info.width, info.height
    ox, oy = info.origin.position.x, info.origin.position.y
    data = grid.data
    pts = []
    for j in range(0, h, stride):
        base = j * w
        for i in range(0, w, stride):
            if data[base + i] >= 100:
                pts.append((ox + i * res, oy + j * res))
    return pts


def profile(points):
    """轨迹的时长、最大速度、最大加速度（用 pose 自带时间戳做数值微分）。"""
    if len(points) < 3:
        return float("nan"), float("nan"), float("nan")
    dur = points[-1][2] - points[0][2]
    speeds = []
    for i in range(1, len(points)):
        dt = points[i][2] - points[i - 1][2]
        if dt <= 1e-6:
            continue
        speeds.append((math.hypot(points[i][0] - points[i - 1][0],
                                  points[i][1] - points[i - 1][1]) / dt, points[i][2]))
    if len(speeds) < 2:
        return dur, (max(s[0] for s in speeds) if speeds else float("nan")), float("nan")
    accels = []
    for i in range(1, len(speeds)):
        dt = speeds[i][1] - speeds[i - 1][1]
        if dt > 1e-6:
            accels.append(abs(speeds[i][0] - speeds[i - 1][0]) / dt)
    return dur, max(s[0] for s in speeds), (max(accels) if accels else float("nan"))


def lat_offset_at_arc(points, yaw, arc):
    """轨迹在弧长 arc 处、车体系下的横向偏移。"""
    if len(points) < 2:
        return None
    acc = 0.0
    for a, b in zip(points, points[1:]):
        seg = math.hypot(b[0] - a[0], b[1] - a[1])
        if acc + seg >= arc and seg > 1e-9:
            t = (arc - acc) / seg
            px, py = a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])
            dx, dy = px - points[0][0], py - points[0][1]
            return -math.sin(yaw) * dx + math.cos(yaw) * dy
        acc += seg
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--window", nargs=2, type=float, default=None, help="只打印这个时间段(秒, 相对第一帧)")
    ap.add_argument("--csv", default="")
    args = ap.parse_args()

    bag = Path(args.bag)
    rows = load_messages(bag)
    if not rows:
        print(f"没有可读消息: {bag}", file=sys.stderr)
        return 1
    t0 = rows[0][0] * 1e-9

    odom = cmd_ctrl = cmd_chassis = diag = input_path = grid = None
    lethal = []
    records = []
    last_arc = {}
    for stamp, name, data in rows:
        t = stamp * 1e-9 - t0
        if name == "/odometry":
            odom = deserialize_message(data, Odometry)
        elif name == "/cmd_vel_controller":
            cmd_ctrl = deserialize_message(data, Twist)
        elif name == "/cmd_vel_chassis":
            cmd_chassis = deserialize_message(data, Twist)
        elif name == "/FollowPath/planning_input_path":
            input_path = deserialize_message(data, RosPath)
        elif name == "/local_costmap/costmap":
            grid = deserialize_message(data, OccupancyGrid)
            lethal = lethal_points(grid)
        elif name == "/FollowPath/diagnostics":
            diag = {kv.key: kv.value for s in deserialize_message(data, DiagnosticArray).status
                    for kv in s.values}
        elif name == "/FollowPath/minco_trajectory":
            traj = path_points(deserialize_message(data, RosPath))
            if not traj or odom is None:
                continue
            yaw = yaw_of(odom)
            x, y = odom.pose.pose.position.x, odom.pose.pose.position.y
            v_odom = math.hypot(odom.twist.twist.linear.x, odom.twist.twist.linear.y)
            ref = [(p[0], p[1]) for p in path_points(input_path)] if input_path else []
            devs = [dist_to_polyline((p[0], p[1]), ref) for p in traj] if len(ref) > 1 else [float("nan")]
            clear = min((min(math.hypot(p[0] - q[0], p[1] - q[1]) for q in lethal)
                         for p in traj[::2]), default=float("nan")) if lethal else float("nan")
            dur, vmax, amax = profile(traj)
            start_gap = math.hypot(traj[0][0] - x, traj[0][1] - y)
            lat = lat_offset_at_arc(traj, yaw, 1.0)
            rec = dict(
                t=round(t, 2), x=round(x, 3), y=round(y, 3),
                v_odom=round(v_odom, 3),
                v_ctrl=round(math.hypot(cmd_ctrl.linear.x, cmd_ctrl.linear.y), 3) if cmd_ctrl else None,
                v_chassis=round(math.hypot(cmd_chassis.linear.x, cmd_chassis.linear.y), 3) if cmd_chassis else None,
                pts=len(traj), dur=round(dur, 3), vmax=round(vmax, 3), amax=round(amax, 3),
                dev_max=round(max(devs), 3), dev_mean=round(sum(devs) / len(devs), 3),
                clearance=round(clear, 3) if clear == clear else None,
                start_gap=round(start_gap, 3), lat1m=round(lat, 3) if lat is not None else None,
                result=(diag or {}).get("planning_result"),
                min_clear=(diag or {}).get("minimum_clearance"),
                stop_reason=(diag or {}).get("stop_reason"),
                terminal_active=(diag or {}).get("terminal_active"),
                repair=(diag or {}).get("speed_repair_count"),
                time_scale=(diag or {}).get("time_scale"),
            )
            records.append(rec)

    print(f"# {bag}\n# 规划周期 {len(records)} 拍, 时长 {records[-1]['t'] - records[0]['t']:.1f} s\n")
    head = (f"{'t':>6} {'v_odom':>6} {'v_ctrl':>6} {'v_chas':>6} {'段':>3} {'时长':>5} {'vmax':>5} {'amax':>5} "
            f"{'切弯':>5} {'净空':>6} {'startGap':>8} {'lat1m':>6} {'结果':>9} {'停因':>8} {'修复':>4}")
    print(head)
    print("-" * len(head))
    show = records
    if args.window:
        lo, hi = args.window
        show = [r for r in records if lo <= r["t"] <= hi]
    prev_lat = None
    for r in show:
        jump = "" if prev_lat is None or r["lat1m"] is None else f"{abs(r['lat1m'] - prev_lat):.3f}"
        prev_lat = r["lat1m"] if r["lat1m"] is not None else prev_lat
        print(f"{r['t']:6.1f} {r['v_odom']:6.2f} {str(r['v_ctrl']):>6} {str(r['v_chassis']):>6} "
              f"{r['pts']:3d} {r['dur']:5.2f} {r['vmax']:5.2f} {r['amax']:5.2f} {r['dev_max']:5.2f} "
              f"{str(r['clearance']):>6} {r['start_gap']:8.3f} {str(r['lat1m']):>6} "
              f"{str(r['result']):>9} {str(r['stop_reason'])[:8]:>8} {str(r['repair']):>4}")

    # ---- 汇总 ----
    print("\n## 汇总")
    fails = [r for r in records if r["result"] not in (None, "success", "idle")]
    print(f"规划失败拍数: {len(fails)} / {len(records)}")
    for r in fails:
        print(f"  t={r['t']:6.1f}s  {r['result']:16s} 净空(diag)={r['min_clear']} "
              f"轨迹净空(实测)={r['clearance']} 切弯={r['dev_max']} vmax={r['vmax']} amax={r['amax']}")
    if fails:
        print(f"  失败时机器人速度: {min(r['v_odom'] for r in fails):.2f} ~ {max(r['v_odom'] for r in fails):.2f} m/s")
    gaps = [r["start_gap"] for r in records if r["result"] == "success"]
    if gaps:
        print(f"\n轨迹起点与车位置距离(start_gap): 均值 {sum(gaps)/len(gaps):.3f} 最大 {max(gaps):.3f} m")
    lats = [r["lat1m"] for r in records if r["lat1m"] is not None]
    jumps = [abs(lats[i] - lats[i - 1]) for i in range(1, len(lats))]
    if jumps:
        jumps_sorted = sorted(jumps)
        print(f"1 m 弧长处横向偏移的逐拍变化: 中位 {jumps_sorted[len(jumps)//2]:.4f} "
              f"p95 {jumps_sorted[int(0.95*len(jumps_sorted))]:.4f} 最大 {max(jumps):.4f} m")
    devs = [r["dev_max"] for r in records if r["dev_max"] == r["dev_max"]]
    if devs:
        print(f"轨迹相对参考路径的最大偏离: 中位 {sorted(devs)[len(devs)//2]:.3f} 最大 {max(devs):.3f} m")
    accs = [r["amax"] for r in records if r["amax"] == r["amax"]]
    if accs:
        accs_sorted = sorted(accs)
        print(f"规划轨迹自身最大加速度: 中位 {accs_sorted[len(accs)//2]:.2f} p95 "
              f"{accs_sorted[int(0.95*len(accs_sorted))]:.2f} 最大 {max(accs):.2f} m/s²")
    durs = sorted(r["dur"] for r in records if r["dur"] == r["dur"])
    if durs:
        print(f"规划轨迹总时长: 最短 {durs[0]:.2f} 中位 {durs[len(durs)//2]:.2f} 最长 {durs[-1]:.2f} s")

    if args.csv:
        import csv
        with open(args.csv, "w", newline="", encoding="utf-8") as fh:
            writer = csv.DictWriter(fh, fieldnames=list(records[0].keys()))
            writer.writeheader()
            writer.writerows(records)
        print(f"\n逐拍明细已写: {args.csv}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
