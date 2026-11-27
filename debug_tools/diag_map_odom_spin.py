#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""判 `map→odom` 转圈的**归属**：是"校正被吃进地图系"，还是"冻结态反算"。

为什么要这个工具
================
现场看到 RViz 里整张点云地图跟着云台转，第一反应是"backend 又在把 map->odom 转过去"。
但本实现里 `map→odom` 有**两条完全不同的来源**，处置办法相反：

1. **校正被吃进去**（要修算法/图）：
   跟踪或恢复把 `map→body` 改掉了，`map→odom` 跟着变。
2. **冻结态反算**（是设计行为，要修的是"为什么进冻结"）：

   ```cpp
   // pose_output.cpp::TrustedPoseProjector::project
   if (!pose_trusted && has_pose_) {
       out.T_map_sensor = map_from_sensor_;                        // 位姿冻住
       out.T_map_odom   = map_from_sensor_ * T_odom_sensor.inverse(); // 用实时 odom 反算
   }
   ```

   "云台就是车"时 odom 的 yaw 跟着云台转，于是 `map→odom` 以 **-云台角速度**匀速转。
   这时 `map→body`（真正的位姿）是**不动**的 —— 车没被传送，只是 TF 在转。

判据（三列对照，一眼分开）
==========================
每个采样点同时记三个 yaw：

    甲  yaw(map→odom)     —— RViz 里"地图转"的那个量
    乙  yaw(odom→body)    —— 云台/底盘的角速度
    丙  yaw(map→body)     —— 真正的定位结果 = 甲 × 乙

- **冻结反算**：甲 大幅摆、乙 大幅摆、**丙 几乎不动**，且 甲 对 乙 的回归斜率 ≈ -1；
  状态串里 `pose_trusted=false`、`tf_frozen=true`。
- **校正被吃**：甲 大幅摆，同时 **丙 也大幅摆**（车被真的转走了）；
  状态串里多半是 `NORMAL` + `recovered via ...`。
- **稳定**：三列都 < 2°。

用法::

    ./debug_tools/diag_map_odom_spin.py                       # 跑 30 s，边看边判
    ./debug_tools/diag_map_odom_spin.py --duration 60
    ./debug_tools/diag_map_odom_spin.py --body-frame gimbal_yaw

退出码：0 = 稳定；1 = 校正被吃进地图系；2 = 冻结态反算；3 = 数据不足/无法判定。
"""

import argparse
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

MAP_FRAME = "map"
ODOM_FRAME = "odom"


def _yaw(q):
    """四元数 (x,y,z,w) → 偏航角（度）。"""
    x, y, z, w = q
    return math.degrees(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def _wrap(deg):
    """把角度折到 (-180, 180]。"""
    return (deg + 180.0) % 360.0 - 180.0


def _spread(values):
    """返回一组角度的最大-最小跨度（度），先解缠绕。"""
    if len(values) < 2:
        return 0.0
    arr = np.radians(np.asarray(values, dtype=float))
    unwrapped = np.degrees(np.unwrap(arr))
    return float(unwrapped.max() - unwrapped.min())


def _slope(xs, ys):
    """最小二乘斜率；样本太少或方差为 0 时返回 None。"""
    if len(xs) < 8:
        return None
    x = np.degrees(np.unwrap(np.radians(np.asarray(xs, dtype=float))))
    y = np.degrees(np.unwrap(np.radians(np.asarray(ys, dtype=float))))
    if float(np.ptp(x)) < 1e-6:
        return None
    return float(np.polyfit(x, y, 1)[0])


def _parse_state(text):
    """把 `status=...;tf_frozen=...;reason=...` 拆成字典。"""
    out = {}
    for part in text.split(";"):
        key, _, value = part.partition("=")
        out[key.strip()] = value.strip()
    return out


class MapOdomSpinProbe(Node):
    """同时采集 map→odom、odom→body 与定位状态，按三列判据给出归属。"""

    def __init__(self, body_frame):
        super().__init__("diag_map_odom_spin")
        self.body_frame = body_frame
        qos = QoSProfile(depth=2000, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        self.state_qos = QoSProfile(depth=50, history=HistoryPolicy.KEEP_LAST,
                                    reliability=ReliabilityPolicy.RELIABLE,
                                    durability=DurabilityPolicy.VOLATILE)
        self.t_map_odom = None
        self.t_odom_body = None
        self.map_odom_yaw = []      # 甲
        self.odom_body_yaw = []     # 乙
        self.map_body_yaw = []      # 丙
        self.times = []
        self.frozen_flags = []
        self.trusted_flags = []
        self.statuses = []
        self.reasons = []
        self.branch_hits = {}
        self.started = time.time()
        self.create_subscription(TFMessage, "/tf", self._on_tf, qos)
        self.create_subscription(String, "/backend_node/localization_state",
                                 self._on_state, self.state_qos)
        # 按固定频率采样：稳定时两路 TF 都不变，靠"变化才记"会采不到样本，
        # 于是把"没动"误判成"数据不足"。定时采样才能同时覆盖动与不动。
        self.create_timer(0.1, self._sample)
        self.get_logger().info(
            "等 TF: %s→%s 与 %s→%s；转云台/开车，%s"
            % (MAP_FRAME, ODOM_FRAME, ODOM_FRAME, body_frame, "开始采集"))

    def _on_tf(self, msg):
        for tr in msg.transforms:
            parent, child = tr.header.frame_id, tr.child_frame_id
            q = (tr.transform.rotation.x, tr.transform.rotation.y,
                 tr.transform.rotation.z, tr.transform.rotation.w)
            if parent == MAP_FRAME and child == ODOM_FRAME:
                self.t_map_odom = (q, tr.transform.translation)
            elif parent == ODOM_FRAME and child == self.body_frame:
                self.t_odom_body = (q, tr.transform.translation)

    def _on_state(self, msg):
        self._handle_state_text(msg.data)

    def _handle_state_text(self, text):
        """解析一条状态串：记状态/冻结标志/reason，并统计关键分支命中。"""
        fields = _parse_state(text)
        self.statuses.append(fields.get("status", "?"))
        self.frozen_flags.append(fields.get("tf_frozen") == "true")
        self.trusted_flags.append(fields.get("pose_trusted") == "true")
        reason = fields.get("reason", "")
        # 只保留"变化过"的 reason，避免刷屏；关键分支本身就够判。
        changed = not self.reasons or self.reasons[-1] != reason
        if changed:
            self.reasons.append(reason)
        # 关键分支计数只在"reason 变了"时累加：状态串是持续重发的，
        # 按消息计数会把"一直停在 initial match"数成几百次"恢复提交"。
        for key, needle in (("累计锚点拦下", "cumulative correction anchor"),
                            ("单次幅度拦下", "correction budget"),
                            ("观测模型失效", "observation model invalid"),
                            ("恢复提交", "recovered via"),
                            ("NIS/冲突", "conflicted")):
            if changed and needle in reason:
                self.branch_hits[key] = self.branch_hits.get(key, 0) + 1

    def _sample(self):
        """两路 TF 都到齐才记一个样本，保证三列同一时刻。"""
        if self.t_map_odom is None or self.t_odom_body is None:
            return
        a = _yaw(self.t_map_odom[0])
        b = _yaw(self.t_odom_body[0])
        self.map_odom_yaw.append(a)
        self.odom_body_yaw.append(b)
        self.map_body_yaw.append(_wrap(a + b))
        self.times.append(time.time() - self.started)

    def report(self):
        n = len(self.map_odom_yaw)
        print("\n================ map→odom 转速归属判定 ================")
        if n < 10:
            print("样本只有 %d 个：TF 没起来，或采集期间没有运动。" % n)
            return 3
        a, b, c = self.map_odom_yaw, self.odom_body_yaw, self.map_body_yaw
        span_a, span_b, span_c = _spread(a), _spread(b), _spread(c)
        duration = max(self.times[-1] - self.times[0], 1e-6)
        rate_a = span_a / duration
        frozen = (sum(self.frozen_flags) / len(self.frozen_flags)) if self.frozen_flags else 0.0
        untrusted = (1.0 - sum(self.trusted_flags) / len(self.trusted_flags)) \
            if self.trusted_flags else 0.0
        slope = _slope(b, a)
        print("样本 %d 个 / %.1f s" % (n, duration))
        print("  甲 yaw(map→odom) 跨度 %7.1f°   （平均 %.1f°/s）" % (span_a, rate_a))
        print("  乙 yaw(odom→body) 跨度 %7.1f°" % span_b)
        print("  丙 yaw(map→body)  跨度 %7.1f°   ← 真正的定位结果" % span_c)
        print("  甲 对 乙 的回归斜率 : %s   （冻结反算应 ≈ -1）"
              % ("n/a" if slope is None else "%+.2f" % slope))
        print("  tf_frozen=true 占比 : %.0f%%    pose_trusted=false 占比: %.0f%%"
              % (100 * frozen, 100 * untrusted))
        if self.statuses:
            uniq = []
            for s in self.statuses:
                if not uniq or uniq[-1] != s:
                    uniq.append(s)
            print("  状态序列            : %s" % " → ".join(uniq[:8]))
        if self.branch_hits:
            print("  reason 命中统计     : %s"
                  % ", ".join("%s×%d" % kv for kv in sorted(self.branch_hits.items())))
        if self.reasons:
            print("  最近的 reason       : %s" % self.reasons[-1][:110])

        # 判据 0：甲纹丝不动而乙在动 → 根本没有校正，先验观测被关掉了（锁存模式）。
        # 这时"地图跟着雷达转"不可能出现，分析前先确认是不是录错了模式。
        if span_a < 0.5 and span_b > 30.0:
            print("\n[结论] map→odom 锁存：本次运行**没有任何全局校正**（唯一值 1 个）。")
            print("       典型成因：入口带 --zloc-prior-once（prior_map.initialization_only=true），")
            print("       或先验观测被别的开关关掉。此时定位是纯 LIO 递推，")
            print("       `map→odom` 没有机制能转 —— 这份包不能用来复现『地图跟着雷达转』。")
            print("       要看现象请去掉 --zloc-prior-once 重录。")
            return 4
        # 判据：甲大幅摆而丙几乎不动 → 冻结反算，不是校正。
        if span_a > 15.0 and span_c < max(5.0, 0.15 * span_a) and (frozen > 0.2 or untrusted > 0.2):
            print("\n[结论] 冻结态反算（设计行为，不是校正被吃进地图系）")
            print("       `map→body` 只摆了 %.1f°，车没有被传送；转的是 TF。" % span_c)
            print("       要修的是\"为什么进冻结\"：看 reason 里的 observation model invalid /")
            print("       conflicts / LOST；先验图能不能用见 debug_tools/check_sc_separability.py。")
            return 2
        if span_a > 15.0 and span_c > 15.0:
            print("\n[结论] 校正被吃进地图系（`map→body` 也摆了 %.1f°，车真的被转走了）" % span_c)
            print("       若已开 correction_anchor 仍如此，检查：① 启动日志里 enabled 是否为 true；")
            print("       ② 是不是恢复/人工重定位在反复重落锚；③ 是否走了图优化而非观测提交。")
            return 1
        print("\n[结论] 稳定：`map→odom` 与 `map→body` 都没超过判据。")
        return 0


def analyse_bag(path, body_frame):
    """离线分析 rosbag：抽 /tf 两段与状态串，喂同一套判据。"""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    probe = object.__new__(MapOdomSpinProbe)
    probe.body_frame = body_frame
    probe.map_odom_yaw, probe.odom_body_yaw, probe.map_body_yaw = [], [], []
    probe.times, probe.frozen_flags, probe.trusted_flags = [], [], []
    probe.statuses, probe.reasons, probe.branch_hits = [], [], {}
    t_map_odom, t_odom_body, first = None, None, None
    while reader.has_next():
        topic, raw, stamp = reader.read_next()
        if first is None:
            first = stamp
        if topic == "/tf" and "/tf" in types:
            msg = deserialize_message(raw, get_message(types[topic]))
            for tr in msg.transforms:
                q = (tr.transform.rotation.x, tr.transform.rotation.y,
                     tr.transform.rotation.z, tr.transform.rotation.w)
                if tr.header.frame_id == MAP_FRAME and tr.child_frame_id == ODOM_FRAME:
                    t_map_odom = q
                elif tr.header.frame_id == ODOM_FRAME and tr.child_frame_id == body_frame:
                    t_odom_body = q
            if t_map_odom is not None and t_odom_body is not None:
                a, b = _yaw(t_map_odom), _yaw(t_odom_body)
                probe.map_odom_yaw.append(a)
                probe.odom_body_yaw.append(b)
                probe.map_body_yaw.append(_wrap(a + b))
                probe.times.append((stamp - first) * 1e-9)
        elif topic == "/backend_node/localization_state":
            msg = deserialize_message(raw, get_message(types[topic]))
            probe._handle_state_text(msg.data)
    return probe


def main():
    ap = argparse.ArgumentParser(description="判 map→odom 转圈是校正还是冻结反算")
    ap.add_argument("--duration", type=float, default=30.0, help="采集秒数（默认 30）")
    ap.add_argument("--bag", help="离线分析一个 rosbag 目录（不开 ROS）")
    ap.add_argument("--body-frame", default="gimbal_yaw",
                    help="odom 下的车体系名（默认 gimbal_yaw；本车云台就是车）")
    args = ap.parse_args()

    if args.bag:
        # 离线分支要先有 rclpy 的 msg 定义可用，所以仍需要 source 工作区的环境。
        probe = analyse_bag(args.bag, args.body_frame)
        return probe.report()

    rclpy.init()
    node = MapOdomSpinProbe(args.body_frame)
    deadline = time.time() + args.duration
    try:
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    code = node.report()
    node.destroy_node()
    rclpy.try_shutdown()
    return code


if __name__ == "__main__":
    sys.exit(main())
