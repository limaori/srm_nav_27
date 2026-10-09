#!/usr/bin/env python3
"""量化"车偏离规划线"到底有多大, 并分清原因。

同时测四件事:
  1) 横偏 cross-track: 车当前位姿 vs "0.5 s 之前那条 /plan"(去掉起点 0.25 m 段), 带左/右符号
  2) 指令 vs 实速: cmd_vel_nav / cmd_vel_sim 与 /odometry 的 twist 比 方向差(度) 与 模长比
  3) MPC 预测 vs 实际: 0.4 s 前那帧 /FollowPath/mpc_prediction 预测的位移 vs 真实位移
  4) 历史轨迹与"当时那条路径"的偏离(车碾过的轨迹画出来看)
"""

import math
import time
from collections import deque

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node

NS = "/red_standard_robot1"
LAG = 0.3          # 横偏用多久之前的路径(短一点, 让老路径的"内部"正好在车附近)
MIN_SPEED = 0.15   # 只在车真的在动时才统计(停着的时候比没意义)
PRED_LAG = 0.4     # 用多久之前的 MPC 预测
PRED_STEP = 0.02   # prediction_dt


class Probe(Node):
    def __init__(self):
        super().__init__("track_probe")
        self.odom = None
        self.cmd_nav = None
        self.cmd_sim = None
        self.plans = deque(maxlen=40)      # (t, Path)
        self.preds = deque(maxlen=40)      # (t, Path)
        self.odom_hist = deque(maxlen=400)  # (t, x, y)
        self.create_subscription(Odometry, f"{NS}/odometry", self.on_odom, 20)
        self.create_subscription(Twist, f"{NS}/cmd_vel_nav", lambda m: setattr(self, "cmd_nav", m), 20)
        self.create_subscription(Twist, f"{NS}/cmd_vel_sim", lambda m: setattr(self, "cmd_sim", m), 20)
        self.create_subscription(Path, f"{NS}/plan",
                                 lambda m: self.plans.append((time.time(), m)), 10)
        self.create_subscription(Path, f"{NS}/FollowPath/mpc_prediction",
                                 lambda m: self.preds.append((time.time(), m)), 10)

    def on_odom(self, m):
        self.odom = m
        p = m.pose.pose.position
        self.odom_hist.append((time.time(), p.x, p.y))


def cross_track(poses, x, y):
    """车到折线的最近距离 + 符号(正=在前进方向左侧) + 投影弧长"""
    best_d, best_s, best_arc = 1e9, 0.0, 0.0
    arc = 0.0
    for a, b in zip(poses, poses[1:]):
        ax, ay = a.pose.position.x, a.pose.position.y
        bx, by = b.pose.position.x, b.pose.position.y
        dx, dy = bx - ax, by - ay
        seg = math.hypot(dx, dy)
        if seg < 1e-6:
            continue
        t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / (seg * seg)))
        px, py = ax + t * dx, ay + t * dy
        d = math.hypot(x - px, y - py)
        if d < best_d:
            best_d = d
            best_s = math.copysign(1.0, dx * (y - ay) - dy * (x - ax))
            best_arc = arc + t * seg
        arc += seg
    return best_d, best_s, best_arc


def trim_start(poses, skip_m):
    """去掉路径开头 skip_m 米(那段永远在车当前位置附近, 比了没意义)"""
    arc, out = 0.0, []
    started = False
    for a, b in zip(poses, poses[1:]):
        seg = math.hypot(b.pose.position.x - a.pose.position.x,
                         b.pose.position.y - a.pose.position.y)
        if not started:
            if arc + seg < skip_m:
                arc += seg
                continue
            started = True
        out.append(b)
    return out or poses


def main():
    rclpy.init()
    node = Probe()
    t0 = time.time()
    rows = []
    last_sample = [0.0]
    print(f"{'t':>5} | {'车位置':>15} | 横偏 | 指令nav(vx,vy) | 实速(vx,vy) | 方向差 | 模长比")
    while time.time() - t0 < 35.0:
        rclpy.spin_once(node, timeout_sec=0.02)
        now = time.time()
        if now - t0 < 1.0 or node.odom is None or not node.plans:
            continue
        if now - last_sample[0] < 0.4:
            continue
        last_sample[0] = now
        p = node.odom.pose.pose.position
        # 1) 横偏
        past = [pl for (t, pl) in node.plans if t <= now - LAG]
        xt = None
        speed = math.hypot(node.odom.twist.twist.linear.x, node.odom.twist.twist.linear.y)
        if past and speed > MIN_SPEED:
            pl = past[-1]
            if len(pl.poses) > 2:
                xt = cross_track(pl.poses, p.x, p.y)
        # 2) 指令 vs 实速 (车体系, 都在 base_link)
        tw = node.odom.twist.twist
        def fmt(t):
            return f"({t.linear.x:5.2f},{t.linear.y:5.2f})" if t else "     -      "
        ang = ratio = float("nan")
        if node.cmd_sim is not None:
            cv = math.hypot(node.cmd_sim.linear.x, node.cmd_sim.linear.y)
            av = math.hypot(tw.linear.x, tw.linear.y)
            if cv > 0.05 and av > 0.02:
                dot = node.cmd_sim.linear.x * tw.linear.x + node.cmd_sim.linear.y * tw.linear.y
                ang = math.degrees(math.acos(max(-1.0, min(1.0, dot / (cv * av)))))
                ratio = av / cv
        # 3) MPC 预测 vs 实际
        pred_err = float("nan")
        pd = [pp for (t, pp) in node.preds if t <= now - PRED_LAG]
        if pd and len(pd[-1].poses) > 21:
            pl = pd[-1]
            tp = [t for (t, _) in node.preds if _ is pl]
            base = pl.poses[0].pose.position
            aim = pl.poses[int(PRED_LAG / PRED_STEP)].pose.position
            px, py = aim.x - base.x, aim.y - base.y
            old = min(node.odom_hist, key=lambda r: abs(r[0] - (now - PRED_LAG))) if node.odom_hist else None
            if old:
                ax, ay = p.x - old[1], p.y - old[2]
                pred_err = math.hypot(ax - px, ay - py)
        if xt:
            rows.append((now - t0, xt[0], xt[1], pred_err, ang, ratio, speed, (p.x, p.y)))
            print(f"{now-t0:5.1f} | ({p.x:6.2f},{p.y:6.2f}) v={speed:4.2f} | 横偏{xt[1]*xt[0]:+5.2f} m | "
                  f"指令{fmt(node.cmd_nav)} 实速{fmt(tw)} | {ang:5.1f}° {ratio:4.2f} | 预测差 {pred_err:4.2f}",
                  flush=True)
        time.sleep(0.2)

    if rows:
        xs = [abs(r[1]) for r in rows]
        print("\n===== 1) 横偏 (车 vs 0.5 s 前的规划线) =====")
        print(f"  样本 {len(rows)} 个(仅车在动的时刻), 平均 |横偏| {sum(xs)/len(xs):.2f} m, 最大 {max(xs):.2f} m")
        print(f"  左偏 {sum(1 for r in rows if r[2] > 0)} 次 / 右偏 {sum(1 for r in rows if r[2] < 0)} 次"
              f"; 符号翻转 {sum(1 for a, b in zip(rows, rows[1:]) if a[2] * b[2] < 0)} 次(画龙指标)")
        print(f"  行驶速度 平均 {sum(r[6] for r in rows)/len(rows):.2f} m/s")
        print("\n  偏离最大的 5 个时刻:")
        for r in sorted(rows, key=lambda r: -abs(r[1]))[:5]:
            print(f"    t={r[0]:5.1f}s 车({r[7][0]:6.2f},{r[7][1]:6.2f}) v={r[6]:4.2f} "
                  f"横偏 {r[2]*r[1]:+5.2f} m")
        pe = [r[3] for r in rows if not math.isnan(r[3])]
        if pe:
            print(f"\n===== 3) MPC 预测 vs 实际 (0.4 s 前那帧预测的位移 vs 真实位移) =====")
            print(f"  平均差 {sum(pe)/len(pe):.2f} m, 最大 {max(pe):.2f} m")
        angs = [r[4] for r in rows if not math.isnan(r[4])]
        rats = [r[5] for r in rows if not math.isnan(r[5])]
        if angs:
            print(f"\n===== 2) 指令(cmd_vel_sim) vs 实速 =====")
            print(f"  方向差 平均 {sum(angs)/len(angs):.1f}° 最大 {max(angs):.1f}°")
            print(f"  模长比 平均 {sum(rats)/len(rats):.2f} 最大 {max(rats):.2f} 最小 {min(rats):.2f}"
                  "   (1.0 = 完全一致)")
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
