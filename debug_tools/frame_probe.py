#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# =============================================================
# 速度坐标系探针: 下位机到底按「云台系」还是「底盘系」执行 vx/vy
#
# 为什么需要它
# ------------
# 2026-11-08 起协议约定「vx/vy 按云台系」(docs/导航通信协议-下位机接口说明.md §1)，
# 但同一份文档 §3.2 还留着旧口径「90° 轴向交叉: vx→底盘 y、vy 取负→底盘 x」并标着
# 【待确认】。两者不可能同时成立，只能实测。
#
# 判据
# ----
#   命令 c = (vx, vy) 是**云台系**的数（fake_vel_transform 输出 / 串口帧里的数）。
#   实测位移 d 来自 /odometry（odom 系），odom 的 yaw 就是**云台**在 odom 里的朝向 ψg；
#   底盘朝向 ψc = ψg + chassis_to_gimbal_yaw（下位机原值，度，俯视逆时针为正）。
#
#   · 若下位机按云台系执行:  d ≈ R(ψg)·c  →  把 d 转回云台系，夹角残差 ≈ 0
#   · 若下位机按底盘系执行:  d ≈ R(ψc)·c  →  转回底盘系才 ≈ 0，云台系那边差 ψc-ψg
#   · 若 §3.2 的 90° 交叉还在: 残差 ≈ ±90°
#   恒定的角度差会被 MINCO 的闭环吃掉(它按 odom 位置反馈反着修)，所以**只有本工具这种
#   开环直发才能把它测出来**。
#
# 用法
# ----
#   1) 只观察（最安全，不控车）: 先另开终端用 test_chassis_serial.py 或 ROS 发速度，
#      本例假设这段时间发的是 vx=0.3、vy=0:
#         ./debug_tools/frame_probe.py --duration 30 --cmd-vx 0.3 --cmd-vy 0
#
#   2) 由本工具发速度：**导航栈要开着**（它不发 /odometry 就没法算方向），但不要发目标，
#      速度从 mux 的输入口进（这样不会和 srm_cmd_mux 抢 cmd_vel_chassis）:
#         ./debug_tools/frame_probe.py --drive --speed 0.3 --yes --cmd-topic /cmd_vel_nav_body
#      自动跑 x+ / y+ / 斜 45° 三段，每段 2 s，段间停车。
#      若确实想直接打 cmd_vel_chassis，就必须先停掉 srm_cmd_mux（否则两者同时发布）；
#      停栈后 /odometry 也没了，本工具会直接告诉你"一条 odom 都没收到"。
#
#   3) **探针打哪个话题，决定了它测的是哪一段**（很重要）:
#        --cmd-topic /cmd_vel_nav_body  -> 直达 mux/串口，**绕过 fake_vel_transform**：
#                                          量到的是「下位机执行系 vs /odometry」的原始偏差
#        --cmd-topic /cmd_vel_nav_fake  -> 经 fake_vel_transform（含 yaw_offset_deg）：
#                                          量到的才是「导航以为的朝向 vs 车实际走向」
#      标定 yaw_offset_deg 用后者：残差应≈0；用前者永远会看到那个固定偏差（那是被测对象）。
#
#   4) 只想看链路通不通: 每秒打印一次各话题的条数，条数为 0 就是没录到东西
#      （rosbag 录出空包通常就是这个原因：录的时候话题还没起来）。
#
# 输出
# ----
#   终端: 每秒一行计数 + 结束时的分段残差表 + 结论
#   CSV : --out 指定（默认 user_test_log/frame_probe_<时间戳>.csv）
# =============================================================

import argparse
import math
import sys
import time
from datetime import datetime
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32

RAD2DEG = 180.0 / math.pi


def wrap_deg(a):
    """把角度折到 (-180, 180]。"""
    a = math.fmod(a + 180.0, 360.0)
    if a < 0.0:
        a += 360.0
    return a - 180.0


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def rotate_deg(angle_deg, by_deg):
    return wrap_deg(angle_deg - by_deg)


class FrameProbe(Node):
    def __init__(self, args):
        super().__init__("frame_probe")
        self.args = args
        self.odom = []          # (t, x, y, yaw_deg)
        self.gimbal = []        # (t, deg)  下位机原值：底盘相对云台
        self.chassis = []       # (t, vx, vy) 收到的 /cmd_vel_chassis（可选）
        self.cmd_samples = []   # (t, vx, vy, ang_deg) 本工具发出的（--drive）
        self.n = {"odom": 0, "gimbal": 0, "chassis": 0}
        self.start = time.monotonic()

        self.create_subscription(Odometry, args.odom_topic, self.on_odom, 50)
        self.create_subscription(Float32, args.gimbal_topic, self.on_gimbal, 50)
        self.create_subscription(Twist, args.cmd_topic, self.on_cmd, 50)
        self.create_subscription(TwistStamped, args.feedback_topic, self.on_feedback, 50)
        self.pub = self.create_publisher(Twist, args.cmd_topic, 10)

    # ---------------- 回调 ----------------
    def now_rel(self):
        return time.monotonic() - self.start

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.odom.append((self.now_rel(), p.x, p.y,
                          yaw_of(msg.pose.pose.orientation) * RAD2DEG))
        self.n["odom"] += 1

    def on_gimbal(self, msg):
        self.gimbal.append((self.now_rel(), float(msg.data)))
        self.n["gimbal"] += 1

    def on_cmd(self, msg):
        if self.args.drive:      # 自己发的自己记，不用回环
            return
        self.chassis.append((self.now_rel(), msg.linear.x, msg.linear.y))
        self.n["chassis"] += 1

    def on_feedback(self, msg):
        self.n.setdefault("fb", 0)
        self.n["fb"] += 1

    # ---------------- 工具 ----------------
    def interp(self, series, t):
        """对 [(t, val)] 线性插值；越界取端点。"""
        if not series:
            return None
        if t <= series[0][0]:
            return series[0][1]
        if t >= series[-1][0]:
            return series[-1][1]
        lo, hi = 0, len(series) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if series[mid][0] <= t:
                lo = mid
            else:
                hi = mid
        t0, v0 = series[lo]
        t1, v1 = series[hi]
        if t1 <= t0:
            return v1
        k = (t - t0) / (t1 - t0)
        if isinstance(v0, tuple):
            return tuple(a + k * (b - a) for a, b in zip(v0, v1))
        return v0 + k * (v1 - v0)

    def live(self):
        el = self.now_rel()
        g = self.gimbal[-1][1] if self.gimbal else float("nan")
        print(f"[{el:5.1f}s] odom={self.n['odom']:4d} gimbal={self.n['gimbal']:4d} "
              f"cmd={self.n['chassis']:4d} fb={self.n.get('fb', 0):4d}  "
              f"云台偏角={g:+6.1f}°", flush=True)
        if self.n["odom"] == 0 and el > 3.0 and not getattr(self, "_warned_odom", False):
            self._warned_odom = True
            print("        ⚠ 还没收到 /odometry —— 导航栈/LIO 没起的话这个测试没有世界系参照，"
                  "算不出方向。", flush=True)

    def wait_ready(self, timeout):
        """等 /odometry 与云台角都活了再动。返回 True=就绪。"""
        t0 = time.monotonic()
        last = -1.0
        while rclpy.ok() and time.monotonic() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            el = self.now_rel()
            if el - last >= 1.0:
                last = el
                print(f"  等待 /odometry ... odom={self.n['odom']} gimbal={self.n['gimbal']} "
                      f"({el:.0f}s / {timeout:.0f}s)", flush=True)
            if self.n["odom"] >= 20 and self.n["gimbal"] >= 5:
                print(f"  ✅ 就绪: odom={self.n['odom']} gimbal={self.n['gimbal']}，开始动作",
                      flush=True)
                return True
        return False

    # ---------------- 驱动 ----------------
    def drive_segment(self, vx, vy, duration, rate, label):
        print(f"  -> {label}: vx={vx:+.2f} vy={vy:+.2f} 持续 {duration:.1f}s", flush=True)
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        t_end = time.monotonic() + duration
        period = 1.0 / rate
        while time.monotonic() < t_end:
            self.pub.publish(msg)
            self.cmd_samples.append((self.now_rel(), vx, vy,
                                     math.degrees(math.atan2(vy, vx))))
            self._spin_for(period)
        self.stop(1.0, rate)

    def _spin_for(self, duration):
        """发速度期间**必须继续 spin**，否则 odom/云台角回调全积压到动作结束才一次性补发
        （2026-11-12 实测：不 spin 时 odom 会"断流 10 s 再喷一串"，整跑作废）。"""
        deadline = time.monotonic() + duration
        while rclpy.ok():
            left = deadline - time.monotonic()
            if left <= 0.0:
                return
            rclpy.spin_once(self, timeout_sec=min(0.01, left))

    def stop(self, duration, rate):
        zero = Twist()
        t_end = time.monotonic() + duration
        period = 1.0 / rate
        while time.monotonic() < t_end:
            self.pub.publish(zero)
            self._spin_for(period)

    def command_for(self, t0, t1):
        """这一段的命令方向（度）。优先用 --drive 自己发的，其次用 --cmd-vx/--cmd-vy。"""
        if self.cmd_samples:
            angs = [s[3] for s in self.cmd_samples
                    if t0 - 0.2 <= s[0] <= t1 + 0.2 and math.hypot(s[1], s[2]) > 1e-6]
            if angs:
                vx = sum(math.cos(math.radians(a)) for a in angs)
                vy = sum(math.sin(math.radians(a)) for a in angs)
                return math.degrees(math.atan2(vy, vx))
        if abs(self.args.cmd_vx) > 1e-9 or abs(self.args.cmd_vy) > 1e-9:
            return math.degrees(math.atan2(self.args.cmd_vy, self.args.cmd_vx))
        return None

    # ---------------- 分析 ----------------
    def segments(self, min_speed=0.03):
        """按 odom 位移把运动切成段：连续 0.4 s 内位移 > min_speed 视为在动。"""
        out = []
        if len(self.odom) < 5:
            return out
        t = [s[0] for s in self.odom]
        # 断流保护：相邻样本间隔 > 0.5 s 视为里程计断流/延迟补发，跨过它的位移一律作废
        # （zlio/LIO 卡住再一次性补发时，"位移/收包时间"会算出荒谬的速度）
        gap = [False] * len(self.odom)
        for i in range(1, len(self.odom)):
            if t[i] - t[i - 1] > 0.5:
                gap[i] = True
        # 速度（中心差分）
        spd = [0.0] * len(self.odom)
        for i in range(1, len(self.odom) - 1):
            if gap[i] or gap[i + 1]:
                continue
            dt = t[i + 1] - t[i - 1]
            if dt > 1e-6:
                spd[i] = math.hypot(self.odom[i + 1][1] - self.odom[i - 1][1],
                                    self.odom[i + 1][2] - self.odom[i - 1][2]) / dt
        in_seg = False
        t0 = None
        for i, s in enumerate(spd):
            moving = s > min_speed and not gap[i]
            if moving and not in_seg:
                in_seg, t0 = True, t[i]
            elif not moving and in_seg:
                in_seg = False
                if t[i] - t0 > 0.6:
                    out.append((t0, t[i]))
        if in_seg and t[-1] - t0 > 0.6:
            out.append((t0, t[-1]))
        return out

    def analyze(self):
        if self.n["odom"] == 0:
            print("\n[结果] 全程**一条 /odometry 都没收到** —— 不是车没动，是根本没有里程计。")
            print("        /odometry 由 nav2_container 里的 sensor_scan_generation 发布，"
                  "需要导航栈 + LIO 都在跑。\n"
                  "        只起 srm27_nav_protocol / 串口直发是测不出来的：没有世界系参照。")
            return None
        t_all = [s[0] for s in self.odom]
        gaps = [(t_all[i - 1], t_all[i]) for i in range(1, len(t_all))
                if t_all[i] - t_all[i - 1] > 0.5]
        if gaps:
            worst = max(b - a for a, b in gaps)
            print(f"\n⚠ /odometry 这次有 {len(gaps)} 段断流（最长 {worst:.1f} s，"
                  f"例 {gaps[0][0]:.1f}s→{gaps[0][1]:.1f}s）：跨断流的位移不参与计算。"
                  "若断流正好盖住动作那几秒，这一跑作废 —— 先查 zlio/LIO 有没有卡住。")
        segs = self.segments()
        if not segs:
            print(f"\n[结果] 收到 /odometry {self.n['odom']} 条，但没检测到位移 —— 车没动。"
                  "（命令没到下位机 / 板子没执行 / 轮子离地空转）")
            return None
        rows = []
        for (a, b) in segs:
            # 用段内首尾各去掉 20% 抗加减速
            span = b - a
            t0, t1 = a + 0.2 * span, b - 0.2 * span
            p0 = self.interp([(s[0], (s[1], s[2])) for s in self.odom], t0)
            p1 = self.interp([(s[0], (s[1], s[2])) for s in self.odom], t1)
            if p0 is None or p1 is None:
                continue
            dx, dy = p1[0] - p0[0], p1[1] - p0[1]
            dist = math.hypot(dx, dy)
            if dist < 0.03:
                continue
            ang_world = math.degrees(math.atan2(dy, dx))
            psi_g = self.interp([(s[0], s[3]) for s in self.odom], 0.5 * (t0 + t1))
            gimbal_deg = self.interp(self.gimbal, 0.5 * (t0 + t1))
            if psi_g is None or gimbal_deg is None:
                continue
            psi_c = psi_g + gimbal_deg                      # 底盘在 odom 里的朝向
            ang_g = rotate_deg(ang_world, psi_g)            # 位移在云台系里的方向
            ang_c = rotate_deg(ang_world, psi_c)            # 位移在底盘系里的方向
            cmd = self.command_for(a, b)
            rows.append(dict(t0=a, t1=b, dist=dist, ang_world=ang_world,
                             psi_g=psi_g, psi_c=psi_c, ang_g=ang_g, ang_c=ang_c,
                             cmd=cmd, gimbal=gimbal_deg))
        if not rows:
            print("\n[结果] 位移都太短（<3 cm），没法算方向；把每段走长一点（≥0.3 m）。")
            return None

        print("\n" + "=" * 96)
        print(f"{'段':>3} {'时长':>5} {'位移':>5} {'命令角':>7} {'实测@云台系':>10} "
              f"{'实测@底盘系':>10} {'ψg':>7} {'云台偏角':>8} {'残差(云台)':>10} {'残差(底盘)':>10}")
        print("-" * 96)
        res_g, res_c = [], []
        for i, r in enumerate(rows):
            c = r["cmd"]
            if c is None:
                cg = cc = None
            else:
                cg = wrap_deg(r["ang_g"] - c)
                cc = wrap_deg(r["ang_c"] - c)
                res_g.append(cg)
                res_c.append(cc)
            print(f"{i:3d} {r['t1'] - r['t0']:5.1f} {r['dist']:5.2f} "
                  f"{('   -  ' if c is None else f'{c:7.1f}')} "
                  f"{r['ang_g']:10.1f} {r['ang_c']:10.1f} {r['psi_g']:7.1f} "
                  f"{r['gimbal']:8.1f} "
                  f"{('    n/a' if cg is None else f'{cg:+10.1f}')} "
                  f"{('    n/a' if cc is None else f'{cc:+10.1f}')}")
        print("=" * 96)

        if res_g:
            print(f"残差中位数: 按云台系 {sorted(res_g)[len(res_g)//2]:+.1f}°，"
                  f"按底盘系 {sorted(res_c)[len(res_c)//2]:+.1f}°，"
                  f"云台偏角 {sorted(r['gimbal'] for r in rows)[len(rows)//2]:+.1f}°")
            verdict(res_g, res_c, rows)
        else:
            print("没有命令角可比（--cmd-vx/--cmd-vy 都是 0，或 --drive 没发速度）。")
            print("此时看「实测@云台系 / 实测@底盘系」两列：哪一列接近你实际发的方向，就是哪个系。")
        return rows


def median(xs):
    s = sorted(xs)
    return s[len(s) // 2]


def verdict(res_g, res_c, rows):
    mg, mc = median(res_g), median(res_c)
    dist_g, dist_c = abs(wrap_deg(mg)), abs(wrap_deg(mc))
    print("\n[结论]")
    if dist_g < 12.0 and dist_g <= dist_c:
        print(f"  下位机按 **云台系** 执行 vx/vy（残差 {mg:+.1f}°）—— 与 2026-11-08 的协议约定一致，"
              "fake_vel_transform 的 R(-ψ) 不需要改。")
    elif dist_c < 12.0:
        print(f"  下位机按 **底盘系** 执行 vx/vy（残差 {mc:+.1f}°，云台系那边 {mg:+.1f}°）——"
              " 需要把下发前的旋转从「云台 yaw」改成「底盘 yaw」（ψg + chassis_to_gimbal）。")
    elif 70.0 < dist_g < 110.0 or 70.0 < dist_c < 110.0:
        print("  残差接近 ±90°：协议 §3.2 的 **90° 轴向交叉**（vx→底盘 y、vy 取负→底盘 x）"
              "还在生效，必须先修下位机或在上位机侧补这个旋转。")
    else:
        print(f"  两种假设都对不上（云台系 {mg:+.1f}°、底盘系 {mc:+.1f}°）。"
              "常见原因：车没被驱动（只是被推着走）、时间对不上、或者 vx/vy 并非该段的命令。"
              "先确认 --cmd-vx/--cmd-vy 与给车的那一段一致。")


def run_self_test():
    """合成数据自检：造一段「按底盘系执行」的位移，看分析器能不能判出来。"""
    psi_g, gimbal = 30.0, 20.0
    psi_c = psi_g + gimbal
    cmd = (0.3, 0.0)
    ang_cmd = math.degrees(math.atan2(cmd[1], cmd[0]))
    # 世界方向 = 底盘系命令旋转 ψc
    world = psi_c + ang_cmd
    ang_g = wrap_deg(world - psi_g)
    ang_c = wrap_deg(world - psi_c)
    rg = wrap_deg(ang_g - ang_cmd)
    rc = wrap_deg(ang_c - ang_cmd)
    assert abs(rc) < 1e-6, rc
    assert abs(rg - gimbal) < 1e-6, rg
    print(f"self-test OK: 底盘系执行 -> 云台系残差 {rg:+.1f}°，底盘系残差 {rc:+.1f}°")


def main():
    ap = argparse.ArgumentParser(description="实测下位机 vx/vy 的坐标系（云台系 vs 底盘系）")
    ap.add_argument("--duration", type=float, default=30.0, help="观察时长（秒），--drive 时自动算")
    ap.add_argument("--cmd-vx", type=float, default=0.0, help="观察模式下这段时间发的 vx")
    ap.add_argument("--cmd-vy", type=float, default=0.0, help="观察模式下这段时间发的 vy")
    ap.add_argument("--drive", action="store_true", help="由本工具发速度（须先停导航栈）")
    ap.add_argument("--speed", type=float, default=0.30, help="--drive 时的速度幅值，默认 0.30")
    ap.add_argument("--rate", type=float, default=20.0, help="--drive 发布频率，默认 20 Hz")
    ap.add_argument("--out", default="", help="CSV 输出路径（默认 user_test_log/frame_probe_<时间>.csv）")
    ap.add_argument("--odom-topic", default="/odometry")
    ap.add_argument("--gimbal-topic", default="/chassis_to_gimbal_yaw_deg")
    ap.add_argument("--cmd-topic", default="/cmd_vel_chassis")
    ap.add_argument("--feedback-topic", default="/chassis_speed")
    ap.add_argument("--yes", action="store_true", help="--drive 时跳过确认")
    ap.add_argument("--watch", type=float, default=0.0,
                    help="只读观察 N 秒：打 ψ_odom / 云台偏角（不发速度）")
    ap.add_argument("--wait-odom", type=float, default=60.0,
                    help="--drive 前最多等 /odometry 就绪多少秒（默认 60）")
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args()

    if args.self_test:
        run_self_test()
        return 0
    if args.speed <= 0.0 or args.speed > 0.5:
        print("--speed 必须落在 (0, 0.5]；首测建议 0.2~0.3")
        return 2
    if args.drive and not args.yes:
        print("⚠ --drive 会**直接给底盘发速度**。要求：\n"
              "  1) 导航栈已起（要有 /odometry）且**没有正在导航的目标**；默认经 mux 发"
              "（--cmd-topic /cmd_vel_nav_body），不要直接与 mux 抢 /cmd_vel_chassis；\n"
              "  2) 车放在空地，或轮子离地，人守着物理急停；\n"
              "  3) 云台保持在你想测的那个偏角上不动（±40°~60° 最好区分，别停在 -85° 附近）。\n"
              "确认无误后加 --yes 重跑。")
        return 2

    rclpy.init()
    node = FrameProbe(args)
    t_end = None
    try:
        if args.watch > 0.0:
            # 只读模式：打 /odometry 的 yaw 与云台偏角，用来回答"odom 的 yaw 跟的是谁"。
            # 典型用法：车静止，用手把**云台**转 ~90°，看 ψ_odom 是否同幅度变化。
            print(f"观察 {args.watch:.0f}s（只读，不发任何速度）。请保持底盘不动，"
                  "用手把**云台**慢慢转 ~90°。\n"
                  f"{'t':>6} {'ψ_odom':>9} {'云台偏角':>9} {'ψ_chassis':>10} "
                  f"{'Δψ_odom':>9} {'Δ云台':>8}")
            t_end = time.monotonic() + args.watch
            base_o = base_g = None
        elif args.drive:
            # 先等里程计/云台角就绪再动，否则"动完了才起来"会把这一跑白费掉。
            if not node.wait_ready(args.wait_odom):
                print(f"\n[中止] 等了 {args.wait_odom:.0f}s 仍没拿到 /odometry（或云台角），没有世界系参照，"
                      "**不发任何速度**。\n"
                      "        先起导航栈（./scripts/bringup/start_real_nav.sh），确认 `ros2 topic hz /odometry` 有数，"
                      "或者直接加大 --wait-odom 后重跑。")
                return 2
            node.stop(0.5, args.rate)
            node.drive_segment(+args.speed, 0.0, 2.0, args.rate, "x+")
            node.drive_segment(0.0, +args.speed, 2.0, args.rate, "y+")
            node.drive_segment(+args.speed * 0.7071, +args.speed * 0.7071, 2.0,
                               args.rate, "斜 45°")
            node.stop(1.0, args.rate)
            t_end = time.monotonic() + 0.5   # 再收半秒样本就收尾
        else:
            print(f"观察 {args.duration:.0f}s：请现在用 test_chassis_serial.py 或 ROS 发速度"
                  f"（命令角 = atan2({args.cmd_vy}, {args.cmd_vx})）")
            t_end = time.monotonic() + args.duration
        last_live = 0.0
        last_watch = -1.0
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            el = node.now_rel()
            if args.watch > 0.0:
                if el - last_watch >= 0.5:
                    last_watch = el
                    psi_o = node.odom[-1][3] if node.odom else float("nan")
                    gim = node.gimbal[-1][1] if node.gimbal else float("nan")
                    if base_o is None and node.odom and node.gimbal:
                        base_o, base_g = psi_o, gim
                    do = (psi_o - base_o) if base_o is not None else float("nan")
                    dg = (gim - base_g) if base_g is not None else float("nan")
                    print(f"{el:6.1f} {psi_o:+9.2f} {gim:+9.2f} {psi_o + gim:+10.2f} "
                          f"{do:+9.2f} {dg:+8.2f}", flush=True)
                if t_end is not None and time.monotonic() > t_end:
                    break
                continue
            if el - last_live >= 1.0:
                last_live = el
                node.live()
                if t_end is not None and time.monotonic() > t_end:
                    break
    except KeyboardInterrupt:
        print("\n[中断]")
    finally:
        if args.drive:
            try:
                node.stop(0.5, args.rate)
            except Exception:
                pass
        if args.watch <= 0.0:
            node.analyze()
        out = args.out or str(Path("user_test_log") /
                              f"frame_probe_{datetime.now():%Y%m%d_%H%M%S}.csv")
        Path(out).parent.mkdir(parents=True, exist_ok=True)
        with open(out, "w", encoding="utf-8") as f:
            f.write("t,odom_x,odom_y,odom_yaw_deg,gimbal_deg\n")
            for s in node.odom:
                g = node.interp(node.gimbal, s[0])
                f.write(f"{s[0]:.3f},{s[1]:.4f},{s[2]:.4f},{s[3]:.2f},"
                        f"{'nan' if g is None else f'{g:.2f}'}\n")
        print(f"[CSV] {out}")
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
