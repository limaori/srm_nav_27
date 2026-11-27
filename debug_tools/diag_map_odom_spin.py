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

判据（四列对照，一眼分开）
==========================
每个采样点同时记四个 yaw：

    甲  yaw(map→odom)       —— RViz 里"地图转"的那个量
    乙  yaw(odom→sensor)    —— 云台/底盘相对 odom 的物理角速度（默认 gimbal_yaw）
    丙  yaw(map→sensor)     —— 传感器在 map 里的位姿 = 甲 × 乙（物理上会跟着自转）
    丁  yaw(map→nav)        —— 导航参考系在 map 里的朝向（默认 gimbal_yaw_fake）
                               **代价地图 / 全局路径 / RViz 固定帧真正看的量**

> 2026-11-27 之后：`pose_output.cpp` 在不可信时改为冻结 `map→odom`。
> 此时甲平直、乙在转、丁也平直 —— 这是**正确**的死推算，不是锁存，也不是 bug。
> 旧版"冻结反算"是甲反向转、丁跟着转，两者必须分开。

- **冻结 map→odom（新，正确）**：甲 几乎不动、乙 大幅摆、**丁 几乎不动**，
  且状态串里出现 `tf_frozen=true` / `pose_trusted=false`。
- **旧冻结反算（bug）**：甲 大幅摆、**丁 也大幅摆**、状态串不可信，
  且甲对乙斜率 ≈ -1。修 `pose_output.cpp` 的冻结投影。
- **校正被吃**：甲 大幅摆、**丁 也大幅摆**，状态串多为 `NORMAL` + `recovered via ...`。
- **锁存/无观测**：甲、丁 都不动，但状态一直 `NORMAL`（先验观测被关掉）。
- **稳定**：甲、丁 都 < 2°。

用法::

    ./debug_tools/diag_map_odom_spin.py                       # 跑 30 s，边看边判
    ./debug_tools/diag_map_odom_spin.py --duration 60
    ./debug_tools/diag_map_odom_spin.py --body-frame gimbal_yaw

退出码：0 = 稳定；1 = 校正被吃进地图系；2 = 旧冻结态反算（bug）；
        3 = 数据不足/无法判定；4 = 锁存/无观测；5 = 冻结 map→odom（2026-11-27 后正确行为）；
        6 = map→odom/导航系平直但大面积不可信/冻结（死推算，状态机在循环，不能算稳定）。
"""

import argparse
import bisect
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


def _interp_yaw(series, t, tol=0.30):
    """在一条 (t, yaw_deg) 边沿序列上按时间插值，走最短角差。

    不能直接用“最后收到的值”：`odom→gimbal_yaw` 是 10 Hz 前端，而
    `gimbal_yaw→gimbal_yaw_fake` 是 100 Hz 的 fake_vel_transform；自转 150°/s 时
    两条边差 0.3 s 就能拼出 45° 的假旋转（2026-11-27 18:17 包就踩到了）。
    """
    if not series:
        return None
    i = bisect.bisect_left(series, (t, -math.inf))
    if i == 0:
        return series[0][1] if series[0][0] - t <= tol else None
    if i == len(series):
        return series[-1][1] if t - series[-1][0] <= tol else None
    t0, y0 = series[i - 1]
    t1, y1 = series[i]
    if t - t0 > tol and t1 - t > tol:
        return None
    if t1 - t0 <= 1e-9:
        return y1
    dy = (y1 - y0 + 180.0) % 360.0 - 180.0
    return y0 + dy * (t - t0) / (t1 - t0)


class MapOdomSpinProbe(Node):
    """同时采集 map→odom、odom→body 与定位状态，按三列判据给出归属。"""

    def __init__(self, body_frame, nav_frame="gimbal_yaw_fake"):
        super().__init__("diag_map_odom_spin")
        self.body_frame = body_frame
        self.nav_frame = nav_frame
        qos = QoSProfile(depth=2000, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        self.state_qos = QoSProfile(depth=50, history=HistoryPolicy.KEEP_LAST,
                                    reliability=ReliabilityPolicy.RELIABLE,
                                    durability=DurabilityPolicy.VOLATILE)
        # 每条 TF 边独立存 (t, yaw_deg) 序列，报告时按同一时刻插值再组合。
        self.edges = {}
        self.map_odom_yaw = []      # 甲
        self.odom_body_yaw = []     # 乙
        self.map_body_yaw = []      # 丙
        self.map_nav_yaw = []       # 丁
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
            "等 TF: %s→%s、%s→%s、%s→%s；转云台/开车，%s"
            % (MAP_FRAME, ODOM_FRAME, ODOM_FRAME, body_frame,
               ODOM_FRAME, nav_frame, "开始采集"))

    def _on_tf(self, msg):
        t = time.time() - self.started
        for tr in msg.transforms:
            parent, child = tr.header.frame_id, tr.child_frame_id
            q = (tr.transform.rotation.x, tr.transform.rotation.y,
                 tr.transform.rotation.z, tr.transform.rotation.w)
            self.edges.setdefault((parent, child), []).append((t, _yaw(q)))

    def _compose_at(self, t):
        """在时刻 t 插值四条边，返回 (甲, 乙, 丙, 丁) 的 yaw（度）。"""
        a = _interp_yaw(self.edges.get((MAP_FRAME, ODOM_FRAME)), t)
        b = _interp_yaw(self.edges.get((ODOM_FRAME, self.body_frame)), t)
        if a is None or b is None:
            return None
        c = _wrap(a + b)
        d = None
        if self.nav_frame:
            d_nav = _interp_yaw(self.edges.get((ODOM_FRAME, self.nav_frame)), t)
            e_body = _interp_yaw(self.edges.get((self.body_frame, self.nav_frame)), t)
            if d_nav is not None:
                d = _wrap(a + d_nav)
            elif e_body is not None:
                d = _wrap(a + b + e_body)
        return a, b, c, (d if d is not None else c)

    def _append_sample(self, t):
        res = self._compose_at(t)
        if res is None:
            return False
        a, b, c, d = res
        self.map_odom_yaw.append(a)
        self.odom_body_yaw.append(b)
        self.map_body_yaw.append(c)
        self.map_nav_yaw.append(d)
        self.times.append(t)
        return True

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
        """按当前时刻插值四条边，保证四列同一时刻。"""
        self._append_sample(time.time() - self.started)

    def report(self):
        n = len(self.map_odom_yaw)
        print("\n================ map→odom 转速归属判定 ================")
        if n < 10:
            print("样本只有 %d 个：TF 没起来，或采集期间没有运动。" % n)
            return 3
        a, b, c = self.map_odom_yaw, self.odom_body_yaw, self.map_body_yaw
        d = getattr(self, "map_nav_yaw", None) or c
        span_a, span_b, span_c, span_d = _spread(a), _spread(b), _spread(c), _spread(d)
        duration = max(self.times[-1] - self.times[0], 1e-6)
        rate_a = span_a / duration
        frozen = (sum(self.frozen_flags) / len(self.frozen_flags)) if self.frozen_flags else 0.0
        untrusted = (1.0 - sum(self.trusted_flags) / len(self.trusted_flags)) \
            if self.trusted_flags else 0.0
        slope = _slope(b, a)
        nav_name = getattr(self, "nav_frame", "gimbal_yaw_fake")
        print("样本 %d 个 / %.1f s" % (n, duration))
        print("  甲 yaw(map→odom) 跨度 %7.1f°   （平均 %.1f°/s）" % (span_a, rate_a))
        print("  乙 yaw(odom→%s) 跨度 %7.1f°" % (self.body_frame, span_b))
        print("  丙 yaw(map→%s) 跨度 %7.1f°   ← 传感器物理位姿（会跟着自转）"
              % (self.body_frame, span_c))
        print("  丁 yaw(map→%s) 跨度 %7.1f°   ← 导航参考系（代价图/全局路径看这个）"
              % (nav_name, span_d))
        print("  甲 对 乙 的回归斜率 : %s   （旧冻结反算应 ≈ -1）"
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

        # 判据 1：甲不动而乙在动 → 没有施加校正。2026-11-27 之后这有两种含义：
        #   * 不可信：冻结 map→odom 的死推算（正确），看丁是否稳定；
        #   * 全程可信：先验观测被关掉（锁存模式），这份包不能用来复现"地图转"。
        if span_a < 0.5 and span_b > 30.0:
            if frozen > 0.2 or untrusted > 0.2:
                print("\n[结论] 冻结 map→odom（2026-11-27 后的正确行为）")
                print("       甲平直、丁 %.1f°：导航参考系没有跟着传感器转。" % span_d)
                print("       定位在不可信状态按前端里程计递推；要查的是为什么进 SUSPECT/LOST，")
                print("       而不是 map→odom 本身。")
                return 5
            print("\n[结论] map→odom 锁存：本次运行**没有任何全局校正**（唯一值 1 个）。")
            print("       典型成因：入口带 --zloc-prior-once（prior_map.initialization_only=true），")
            print("       或先验观测被别的开关关掉。此时定位是纯 LIO 递推，")
            print("       `map→odom` 没有机制能转 —— 这份包不能用来复现『地图跟着雷达转』。")
            print("       要看现象请去掉 --zloc-prior-once 重录。")
            return 4
        # 判据 2：甲和丁一起大幅摆 → 导航参考系真的在 map 里转。
        if span_a > 15.0 and span_d > 15.0:
            if frozen > 0.2 or untrusted > 0.2:
                print("\n[结论] 旧冻结反算（bug）：map→odom 与导航参考系一起摆了 %.1f°。" % span_d)
                print("       pose_output.cpp 之前冻的是 T_map_sensor，传感器/云台一转就反算成")
                print("       map→odom 反向转；改成冻结 T_map_odom 即可。")
                return 2
            print("\n[结论] 校正被吃进地图系（导航参考系也摆了 %.1f°，车真的被转走了）" % span_d)
            print("       若已开 correction_anchor 仍如此，检查：① 启动日志里 enabled 是否为 true；")
            print("       ② 是不是恢复/人工重定位在反复重落锚；③ 是否走了图优化而非观测提交。")
            return 1
        # 判据 3：甲在转但丁不动 → 导航参考系被 fake 系抵消，代价图/路径不转。
        if span_a > 15.0 and span_d < 5.0:
            print("\n[结论] map→odom 在转，但导航参考系（%s）没转；" % nav_name)
            print("       至少代价地图/全局路径不会跟着传感器转。若 RViz 仍看到地图转，")
            print("       先确认 Fixed Frame 不是 base_link/gimbal_yaw。")
            return 0
        # 判据 4：甲/丁平直，但大面积不可信/冻结 → 不是稳定，是死推算/状态机在循环。
        if span_a < 2.0 and span_d < 2.0 and (untrusted > 0.2 or frozen > 0.2):
            print("\n[结论] map→odom / 导航系平直，但 %.0f%% 时间不可信（tf_frozen=%.0f%%）。"
                  % (100 * untrusted, 100 * frozen))
            print("       这不是稳定：backend 在 SUSPECT/LOST/恢复提交循环里，")
            print("       导航位姿是冻结 map→odom + 前端里程计递推。看 reason 里的")
            print("       NIS/conflict、observation model invalid、empty cloud，先修这些；")
            print("       不要因为甲/丁平直就当成本次定位正常。")
            return 6
        print("\n[结论] 稳定：`map→odom` 与 `map→%s` 都没超过判据。" % nav_name)
        return 0


def analyse_bag(path, body_frame, nav_frame="gimbal_yaw_fake"):
    """离线分析 rosbag：抽 /tf 各段与状态串，喂同一套判据。"""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    probe = object.__new__(MapOdomSpinProbe)
    probe.body_frame = body_frame
    probe.nav_frame = nav_frame
    probe.edges = {}
    probe.map_odom_yaw, probe.odom_body_yaw, probe.map_body_yaw = [], [], []
    probe.map_nav_yaw = []
    probe.times, probe.frozen_flags, probe.trusted_flags = [], [], []
    probe.statuses, probe.reasons, probe.branch_hits = [], [], {}
    first = None
    while reader.has_next():
        topic, raw, stamp = reader.read_next()
        if first is None:
            first = stamp
        if topic == "/tf" and "/tf" in types:
            msg = deserialize_message(raw, get_message(types[topic]))
            t = (stamp - first) * 1e-9
            for tr in msg.transforms:
                q = (tr.transform.rotation.x, tr.transform.rotation.y,
                     tr.transform.rotation.z, tr.transform.rotation.w)
                key = (tr.header.frame_id, tr.child_frame_id)
                probe.edges.setdefault(key, []).append((t, _yaw(q)))
        elif topic == "/backend_node/localization_state":
            msg = deserialize_message(raw, get_message(types[topic]))
            probe._handle_state_text(msg.data)
    # 各条 TF 边按时间排序后，在公共时基上插值再组合；
    # 不能拿"最后收到的各边"直接拼（自转时不同频率的边会差几十度）。
    for series in probe.edges.values():
        series.sort(key=lambda x: x[0])
    all_t = [x[0] for s in probe.edges.values() for x in s]
    if all_t:
        for t in np.arange(min(all_t), max(all_t), 0.05):
            probe._append_sample(float(t))
    return probe


def main():
    ap = argparse.ArgumentParser(description="判 map→odom 转圈是校正还是冻结反算")
    ap.add_argument("--duration", type=float, default=30.0, help="采集秒数（默认 30）")
    ap.add_argument("--bag", help="离线分析一个 rosbag 目录（不开 ROS）")
    ap.add_argument("--body-frame", default="gimbal_yaw",
                    help="odom 下的传感器车体系名（默认 gimbal_yaw；本车云台就是车）")
    ap.add_argument("--nav-frame", default="gimbal_yaw_fake",
                    help="odom 下的导航参考系名（默认 gimbal_yaw_fake；"
                         "代价地图/全局路径看这个）")
    args = ap.parse_args()

    if args.bag:
        # 离线分支要先有 rclpy 的 msg 定义可用，所以仍需要 source 工作区的环境。
        probe = analyse_bag(args.bag, args.body_frame, args.nav_frame)
        return probe.report()

    rclpy.init()
    node = MapOdomSpinProbe(args.body_frame, args.nav_frame)
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
