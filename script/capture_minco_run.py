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
"""抓一次 MINCO 实车运行的数据（不控制车，只订阅）。

为什么需要它：`~/.ros/log` 里只有**文本**（谁失败、失败在哪），
没有**几何**（MINCO 轨迹长什么样、和全局红线差多少、在哪一刻切了轨迹）。
本脚本把两者一起落到一个目录里，事后可以直接复盘"规划为什么看着奇怪"。

输出（默认 `log_diag/real_capture_<时间戳>/`）：

  samples.jsonl        10 Hz 一行：odom / 命令 / diagnostics 全量键值 / 当前情况
  frames/frame_%04d.json.gz   1 Hz 快照：全局路径、MINCO 轨迹、planning_input_path、
                              mpc_prediction、局部/全局代价地图（含元数据与栅格）
  run.log              5 s 一条的进度摘要（终端也能看）
  capture_meta.json    话题清单与开始时间

用法：

  # 先在另一个终端起导航栈: ./script/start_real_nav.sh --controller minco
  # 再抓（Ctrl-C 结束；车归你开/归航点脚本开）
  python3 script/capture_minco_run.py                       # 无限时，Ctrl-C 停
  python3 script/capture_minco_run.py --duration 180        # 只抓 3 分钟
  python3 script/capture_minco_run.py --out log_diag/xxx    # 指定目录
  python3 script/capture_minco_run.py --ns red_standard_robot1   # 仿真命名空间

注意：本脚本只订阅，绝不发速度、也不下发目标 —— 不会和 waypoint 脚本抢控制权。
"""

import argparse
import gzip
import json
import math
import signal
import sys
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.msg import OccupancyGrid, Odometry, Path as RosPath

# (键, 类型, 相对话题名)  —— 全部按 qos_profile_sensor_data 订阅（与控制器一致）
TOPICS = [
    ("odom", Odometry, "odometry"),
    ("cmd_nav", Twist, "cmd_vel_nav"),
    ("cmd_ctrl", Twist, "cmd_vel_controller"),
    ("cmd_chassis", Twist, "cmd_vel_chassis"),
    ("diag", DiagnosticArray, "FollowPath/diagnostics"),
    ("minco", RosPath, "FollowPath/minco_trajectory"),
    ("input_path", RosPath, "FollowPath/planning_input_path"),
    ("prediction", RosPath, "FollowPath/mpc_prediction"),
    ("plan", RosPath, "plan"),
    ("local_plan", RosPath, "local_plan"),
    ("lookahead", PointStamped, "lookahead_point"),
    ("goal", PoseStamped, "goal_pose"),
]

MAP_TOPICS = [("local_costmap", "local_costmap/costmap"), ("global_costmap", "global_costmap/costmap")]

# diagnostics 里最值得每行都留的键（其余也全量存，只是这几个提到顶层方便 grep）
DIAG_KEYS = (
    "planning_result", "stop_reason", "terminal_reason", "terminal_speed", "terminal_distance",
    "terminal_active", "terminal_tolerance", "command_owner", "minimum_clearance", "map_age",
    "trajectory_age", "speed_repair_count", "time_scale", "rejected_stale_result_count",
    "switched_trajectory_count", "missed_deadline_count", "replan_type", "planning_time_ms",
)


def path_summary(msg):
    pts = [[round(p.pose.position.x, 3), round(p.pose.position.y, 3)] for p in msg.poses]
    length = 0.0
    for a, b in zip(pts, pts[1:]):
        length += math.hypot(b[0] - a[0], b[1] - a[1])
    return {
        "frame": msg.header.frame_id,
        "stamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
        "count": len(pts),
        "length": round(length, 3),
        "start": pts[0] if pts else None,
        "end": pts[-1] if pts else None,
        "points": pts,
    }


class MincoCapture(Node):
    def __init__(self, ns, out: Path, frame_period: float, sample_period: float):
        super().__init__("minco_capture")
        self.ns = ns
        self.out = out
        self.frame_period = frame_period
        self.sample_period = sample_period
        self.latest = {}
        self.maps = {}
        self.map_meta = {}
        self.frames = 0
        self.samples = 0
        self.t0 = time.monotonic()
        self.next_sample = 0.0
        self.next_frame = 0.0
        self.next_print = 0.0
        self.samples_file = (out / "samples.jsonl").open("w", buffering=1)
        (out / "frames").mkdir(exist_ok=True)
        self.log = (out / "run.log").open("w", buffering=1)

        for key, typ, topic in TOPICS:
            self.create_subscription(typ, f"{ns}/{topic}", self._make_cb(key), qos_profile_sensor_data)
        # 代价地图用 transient_local 拿全量，再用 updates 增量维护（与 costmap 发布端一致）
        qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for key, topic in MAP_TOPICS:
            self.create_subscription(
                OccupancyGrid, f"{ns}/{topic}", self._make_map_cb(key), qos)
            self.create_subscription(
                OccupancyGridUpdate, f"{ns}/{topic}_updates", self._make_update_cb(key), 10)

    # ---- 回调 ----
    def _make_cb(self, key):
        def cb(msg):
            if key == "odom":
                p = msg.pose.pose.position
                v = msg.twist.twist.linear
                self.latest[key] = {
                    "x": round(p.x, 4), "y": round(p.y, 4),
                    "yaw": round(2.0 * math.atan2(msg.pose.pose.orientation.z,
                                                 msg.pose.pose.orientation.w), 4),
                    "speed": round(math.hypot(v.x, v.y), 4),
                    "wz": round(msg.twist.twist.angular.z, 4),
                    "stamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                }
            elif key in ("cmd_nav", "cmd_ctrl", "cmd_chassis"):
                self.latest[key] = {
                    "vx": round(msg.linear.x, 4), "vy": round(msg.linear.y, 4),
                    "wz": round(msg.angular.z, 4),
                    "speed": round(math.hypot(msg.linear.x, msg.linear.y), 4),
                }
            elif key == "diag":
                values = {kv.key: kv.value for s in msg.status for kv in s.values}
                self.latest[key] = values
            elif key == "lookahead":
                self.latest[key] = {"x": round(msg.point.x, 3), "y": round(msg.point.y, 3)}
            elif key == "goal":
                self.latest[key] = {
                    "frame": msg.header.frame_id,
                    "x": round(msg.pose.position.x, 3), "y": round(msg.pose.position.y, 3)}
            else:
                self.latest[key] = path_summary(msg)
        return cb

    def _make_map_cb(self, key):
        def cb(msg):
            self.map_meta[key] = {
                "frame": msg.header.frame_id, "resolution": msg.info.resolution,
                "width": msg.info.width, "height": msg.info.height,
                "origin": [round(msg.info.origin.position.x, 3), round(msg.info.origin.position.y, 3)],
                "stamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            }
            self.maps[key] = list(msg.data)
        return cb

    def _make_update_cb(self, key):
        def cb(msg):
            if key not in self.maps:
                return
            meta = self.map_meta[key]
            width = meta["width"]
            data = self.maps[key]
            for row in range(msg.height):
                dst_y = msg.y + row
                if dst_y >= meta["height"]:
                    continue
                start = dst_y * width + msg.x
                data[start:start + msg.width] = list(msg.data[row * msg.width:(row + 1) * msg.width])
        return cb

    # ---- 落盘 ----
    def sample_row(self, elapsed):
        row = {"elapsed": round(elapsed, 3), "wall": time.time()}
        for key, value in self.latest.items():
            row[key] = value
        diag = self.latest.get("diag", {})
        for key in DIAG_KEYS:
            if key in diag:
                try:
                    row[key] = float(diag[key])
                except (TypeError, ValueError):
                    row[key] = diag[key]
        return row

    def write_frame(self, elapsed):
        payload = {"elapsed": round(elapsed, 3), "odom": self.latest.get("odom"),
                   "diag": self.latest.get("diag", {}), "maps": {}}
        for key, points in self.maps.items():
            payload["maps"][key] = {**self.map_meta[key], "data": points}
        for key in ("plan", "minco", "input_path", "prediction", "local_plan"):
            if key in self.latest:
                payload[key] = self.latest[key]
        name = self.out / "frames" / f"frame_{self.frames:04d}.json.gz"
        with gzip.open(name, "wt") as stream:
            json.dump(payload, stream)
        self.frames += 1

    def spin(self, duration):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
            elapsed = time.monotonic() - self.t0
            if duration > 0.0 and elapsed >= duration:
                return
            if elapsed >= self.next_sample:
                self.samples_file.write(json.dumps(self.sample_row(elapsed)) + "\n")
                self.samples += 1
                self.next_sample = elapsed + self.sample_period
            if elapsed >= self.next_frame:
                self.write_frame(elapsed)
                self.next_frame = elapsed + self.frame_period
            if elapsed >= self.next_print:
                self.print_summary(elapsed)
                self.next_print = elapsed + 5.0

    def print_summary(self, elapsed):
        diag = self.latest.get("diag", {})
        odom = self.latest.get("odom") or {}
        cmd = self.latest.get("cmd_chassis") or {}
        minco = self.latest.get("minco") or {}
        line = (f"[{elapsed:7.1f}s] pos=({odom.get('x')}, {odom.get('y')}) v={odom.get('speed')} "
                f"cmd={cmd.get('vx')},{cmd.get('vy')} minco段={minco.get('count')} "
                f"结果={diag.get('planning_result')} 净空={diag.get('minimum_clearance')} "
                f"停因={diag.get('stop_reason')} 会话={diag.get('rejected_stale_result_count')}")
        print(line, flush=True)
        self.log.write(line + "\n")

    def close(self):
        self.samples_file.close()
        self.log.close()


def main():
    parser = argparse.ArgumentParser(description="抓一次 MINCO 实车运行数据（只订阅）")
    parser.add_argument("--ns", default="", help="命名空间（实车留空；仿真如 red_standard_robot1）")
    parser.add_argument("--out", default="", help="输出目录（默认 log_diag/real_capture_<时间戳>）")
    parser.add_argument("--duration", type=float, default=0.0, help="抓取时长秒（0=直到 Ctrl-C）")
    parser.add_argument("--sample-period", type=float, default=0.1, help="samples.jsonl 采样周期（秒）")
    parser.add_argument("--frame-period", type=float, default=1.0, help="几何/代价地图快照周期（秒）")
    args = parser.parse_args()

    ws = Path(__file__).resolve().parent.parent
    stamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    out = Path(args.out) if args.out else ws / "log_diag" / f"real_capture_{stamp}"
    if not out.is_absolute():
        out = ws / out
    out.mkdir(parents=True, exist_ok=True)

    ns = args.ns.rstrip("/")
    rclpy.init()
    node = MincoCapture(ns, out, args.frame_period, args.sample_period)
    (out / "capture_meta.json").write_text(json.dumps({
        "started": datetime.now().isoformat(), "namespace": ns,
        "topics": [t for _, _, t in TOPICS] + [t for _, t in MAP_TOPICS],
        "sample_period": args.sample_period, "frame_period": args.frame_period,
    }, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"[抓取] 输出目录: {out}", flush=True)
    print("[抓取] 只订阅、不发命令；Ctrl-C 结束。现在可以跑航点/手动给目标。", flush=True)

    def stop(_signum, _frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop)
    try:
        node.spin(args.duration)
    except KeyboardInterrupt:
        print("[抓取] 收到中断，收尾中…", flush=True)
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print(f"[抓取] 完成: {node.samples} 条采样, {node.frames} 帧几何 → {out}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
