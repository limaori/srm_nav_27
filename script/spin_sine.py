#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# =============================================================
# 正弦自转：向 <ns>/rotation_cmd 连续发 wz = mid + amp·sin(2πt/T)
#
#   mid = (max + min) / 2 ,  amp = (max - min) / 2
#   t=0 时 wz = mid（正弦从 0 起步），前 --ramp 秒整体按比例淡入，避免阶跃。
#
# 话题契约与 spin_control.sh on/off 完全一致（同一话题、同一 PID 文件由调用方管理）：
#   rotation_cmd --50 Hz--> srm_cmd_mux（只取 angular.z）--> cmd_vel_chassis --> 串口
# 停发超过 mux 的 spin_timeout(0.25 s) 后底盘自动停转；本脚本退出前还会补一条零速。
#
# 直接用法（一般由 spin_control.sh sine / start_real_nav.sh --chassis-spin-sine 调起）:
#   ./script/spin_sine.py --max 0.5 --min -0.3 --period 4
#   ./script/spin_sine.py --max 0.5 --min -0.5 --period 4 --duration 20
# =============================================================

import argparse
import math
import signal
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class SineSpin(Node):
    def __init__(self, args):
        super().__init__("spin_sine")
        self.args = args
        self.mid = 0.5 * (args.max + args.min)
        self.amp = 0.5 * (args.max - args.min)
        self.t0 = time.monotonic()
        self.pub = self.create_publisher(Twist, args.topic, 10)
        self.done = False
        self.last_log = -1.0

    def wz_at(self, t):
        w = self.mid + self.amp * math.sin(2.0 * math.pi * t / self.args.period)
        if self.args.ramp > 0.0 and t < self.args.ramp:
            w *= t / self.args.ramp
        return w

    def run(self):
        period = 1.0 / max(1.0, self.args.rate)
        next_tick = time.monotonic()
        while rclpy.ok() and not self.done:
            t = time.monotonic() - self.t0
            if self.args.duration > 0.0 and t >= self.args.duration:
                break
            msg = Twist()
            msg.angular.z = self.wz_at(t)
            self.pub.publish(msg)
            if t - self.last_log >= 1.0:
                self.last_log = t
                print(f"[正弦自转] t={t:6.1f}s  wz={msg.angular.z:+.3f} rad/s "
                      f"(max {self.args.max:+.3f} / min {self.args.min:+.3f} / T {self.args.period:.2f}s)",
                      flush=True)
            next_tick += period
            sleep = next_tick - time.monotonic()
            if sleep > 0.0:
                time.sleep(sleep)
            else:                      # 落后了就跳过补偿，别攒抖动
                next_tick = time.monotonic()
            rclpy.spin_once(self, timeout_sec=0.0)

    def send_zero(self):
        try:
            self.pub.publish(Twist())
        except Exception:
            pass


def main():
    ap = argparse.ArgumentParser(description="正弦自转（rotation_cmd）")
    ap.add_argument("--ns", default="", help="命名空间（空串 = 根命名空间，实车默认）")
    ap.add_argument("--topic", default="", help="话题名（默认 <ns>/rotation_cmd）")
    ap.add_argument("--max", type=float, required=True, help="正弦正峰值 rad/s")
    ap.add_argument("--min", type=float, required=True, help="正弦负峰值 rad/s")
    ap.add_argument("--period", type=float, required=True, help="周期 s")
    ap.add_argument("--rate", type=float, default=50.0, help="发布频率 Hz（默认 50）")
    ap.add_argument("--ramp", type=float, default=0.5, help="起步淡入时间 s（默认 0.5，0=不淡入）")
    ap.add_argument("--duration", type=float, default=0.0, help="运行时长 s（0=一直跑）")
    args = ap.parse_args()

    if not (args.period > 0.0):
        print(f"[错误] --period 必须 > 0，收到 {args.period}", file=sys.stderr)
        return 2
    if args.max < args.min:
        print(f"[错误] --max({args.max}) 必须 >= --min({args.min})", file=sys.stderr)
        return 2
    if args.topic:
        topic = args.topic
    else:
        topic = (f"/{args.ns.strip('/')}/rotation_cmd" if args.ns.strip("/") else "/rotation_cmd")
    args.topic = topic

    rclpy.init()
    node = SineSpin(args)
    node.get_logger().info(
        f"正弦自转: {topic}  mid={0.5*(args.max+args.min):+.3f} "
        f"amp={0.5*(args.max-args.min):.3f} T={args.period:.2f}s rate={args.rate:.0f}Hz")

    def _stop(_sig, _frame):
        node.done = True

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)
    try:
        node.run()
    finally:
        node.send_zero()
        print("[正弦自转] 已停止（补发零速）。", flush=True)
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
