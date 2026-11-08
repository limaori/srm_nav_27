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
"""IMU 加速度饱和保护：给 ZLIO 前端补上 Point-LIO 早就有的那一层。

为什么需要它
============
Point-LIO 的配置里有 `check_satu: True` + `satu_acc: 30.0`（nav2_params_srm_fakeyaw.yaml:90/141），
也就是"加速度模长超过 30 m/s² 就不信这一帧"。**ZLIO 前端运行时没有任何等价的检查**
（`src/frontend` 全文搜 `satur`/`clip` 为空），它唯一的门是"一个面都没匹配到"
（`filter.cpp:112`）。

后果在撞墙这种场景上是实打实的：Gazebo 里 0.2 m/s 撞停在 1~2 个物理步内完成，
等效加速度比 3 g 高一个量级，这一脚冲量被原样积分成几十 m/s 的速度，而激光匹配在
"正对一面大墙"时是退化的（法向可观、切向不可观），救不回来 —— 表现就是
**航向还贴着真值、位置飞出去几千米**（2026-11-05 实跑，见
docs/仿真LIO前端替换Z-LIO实施记录(ai).md §5.4）。

本节点做的事只有一件：`|linear_acceleration|` 超过 `max_acceleration`（默认 30 m/s²，
与 Point-LIO 的 `satu_acc` 对齐）时，**这一帧的加速度直接换成"上一次没饱和时的值"**
（保持时间连续性），并把饱和次数打出来。

为什么是"替换成上一帧"而不是"按比例缩小"
========================================
按比例缩到 30 m/s² 看起来更"温和"，其实更糟：它把真实碰撞的冲量**摊平成一个持续的高加速度**，
滤波器照样把这段积分进速度里（只是慢一点），而且缩过的值在物理上根本不存在。
替换成上一帧的意图是"**这一帧的加速度计不可信，当它没发生**"：滤波器按"没有冲击"继续外推，
注入的误差就只剩**真实碰撞的速度变化**（0.2 m/s 撞停 ≈ 0.2 m/s 量级，实车 1~2 m/s），
这个量级激光匹配完全能拉回来。

这也是 Point-LIO `check_satu` 的思路：饱和的那一帧**不信任**它，而不是给它编一个折中值。

不是它在管的事
=============
* 不做低通/去噪（ZLIO 的噪声模型假定输入就是原始 IMU）；
* 不动角速度（Point-LIO 的饱和参数也只管加速度）；
* 不做"撞击检测后重置定位"——那属于上层决策，本节点只保证"滤波器看不到物理上不可能、也不可信的加速度"。

边界情况
========
* **还没出现过未饱和值**（第一帧就饱和）：没有可替换的参考值 → 原样透传并计数，
  不做猜测；
* **持续饱和超过 `max_hold_sec`**（默认 0.5 s，远超任何碰撞时长）：说明不是"撞击"而是
  传感器/链路异常，继续假装"没加速度"反而会掩盖故障 → 恢复原样透传并告警。

不是它在管的事
==============
* 不做低通/去噪（ZLIO 的噪声模型假定输入就是原始 IMU）；
* 不动角速度（Point-LIO 的饱和参数也只管加速度）；
* 不做"撞击检测后重置定位"——那属于上层决策，本节点只保证"滤波器看不到物理上不可能的加速度"。

参数
====
| 参数 | 默认 | 说明 |
|---|---|---|
| `input_topic` | `livox/imu` | 原始 IMU（相对名，跟随命名空间） |
| `output_topic` | `livox/imu_saturated` | 处理后的 IMU（饱和帧已替换），ZLIO 订这一个 |
| `max_acceleration` | `30.0` | 饱和判据（m/s²）；设成很大（如 1e6）等于关掉整个保护 |
| `max_hold_sec` | `0.5` | 持续饱和多久后放弃"替换"、恢复原样透传（防传感器故障被掩盖） |
| `warn_throttle_sec` | `2.0` | 告警节流（秒） |
| `report_interval_sec` | `10.0` | 周期性汇报：饱和帧数、替换帧数、无参考帧数、最大 \|a\| |

输出与输入**同 stamp、同 header**，其它字段（角速度、协方差、朝向）原样透传。
"""

import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu


class ImuSaturationGuard(Node):
    """饱和帧的加速度替换成上一次未饱和值，并把饱和事件显式报出来。"""

    def __init__(self):
        super().__init__("imu_saturation_guard")

        self.declare_parameter("input_topic", "livox/imu")
        self.declare_parameter("output_topic", "livox/imu_saturated")
        self.declare_parameter("max_acceleration", 30.0)
        # 持续饱和超过这个时长就放弃"替换"（碰撞不会这么久；更可能是传感器/链路异常）
        self.declare_parameter("max_hold_sec", 0.5)
        self.declare_parameter("warn_throttle_sec", 2.0)
        self.declare_parameter("report_interval_sec", 10.0)

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self.max_acceleration = float(self.get_parameter("max_acceleration").value)
        self.max_hold_sec = float(self.get_parameter("max_hold_sec").value)
        self.warn_throttle_sec = float(self.get_parameter("warn_throttle_sec").value)
        report_interval = float(self.get_parameter("report_interval_sec").value)

        if not (self.max_acceleration > 0.0) or not math.isfinite(self.max_acceleration):
            raise ValueError("max_acceleration must be finite and positive")
        if not (self.max_hold_sec > 0.0) or not math.isfinite(self.max_hold_sec):
            raise ValueError("max_hold_sec must be finite and positive")
        if not input_topic or not output_topic:
            raise ValueError("input_topic / output_topic must not be empty")

        self.samples = 0
        self.saturated = 0        # |a| 超阈值的帧数
        self.replaced = 0         # 其中真正被"上一帧值"替换的帧数
        self.no_reference = 0     # 饱和但还没有未饱和参考值（原样透传）
        self.hold_expired = 0     # 饱和持续超过 max_hold_sec（放弃替换，原样透传）
        self.worst = 0.0
        self._last_warn = {}
        self._report_samples = 0
        self._report_time = None
        self.last_good = None     # 最近一次**未饱和**的加速度 (x, y, z)
        self.last_good_time = None
        self._hold_expired_reported = False

        # 订阅：源头（ros_gz_bridge）可能是 best-effort，这里按传感器 QoS 收；
        # 发布：只有 zlio 一个订阅者（best-effort keep_last(4000)），所以也用 best-effort，
        # 但把深度放大到 1000 —— 200 Hz 的 IMU 用深度 5 会丢帧，而 ZLIO 的预积分
        # 要求时间连续。
        sensor_qos = QoSProfile(depth=1000, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.publisher = self.create_publisher(Imu, output_topic, sensor_qos)
        # 深度放大到 1000：200 Hz 的 IMU 用默认 depth=5 只剩 25 ms 缓冲，
        # 而 ZLIO 的预积分对"样本间隔"敏感（`maximum_imu_interval`: 0.1 s）。
        self.subscription = self.create_subscription(
            Imu, input_topic, self._on_imu, sensor_qos
        )
        self.report_timer = self.create_timer(max(1.0, report_interval), self._report)

        self.get_logger().info(
            f"IMU 饱和保护: {input_topic} -> {output_topic}, "
            f"|a| 上限 {self.max_acceleration:.1f} m/s²"
        )

    def _on_imu(self, msg: Imu):
        self.samples += 1
        acceleration = msg.linear_acceleration
        norm = math.sqrt(
            acceleration.x * acceleration.x
            + acceleration.y * acceleration.y
            + acceleration.z * acceleration.z
        )
        now = self.get_clock().now().nanoseconds * 1e-9

        if norm > self.max_acceleration:
            self.saturated += 1
            self.worst = max(self.worst, norm)

            if self.last_good is None:
                # 还没有任何"未饱和"参考值：不猜，原样透传（并计数，让上层看得见）
                self.no_reference += 1
                self._warn_throttle(
                    now,
                    "IMU 加速度饱和但还没有可替换的参考值：|a|=%.1f m/s² 原样透传"
                    "（累计 %d 帧）" % (norm, self.no_reference),
                    key="no_reference",
                )
            elif (now - self.last_good_time) > self.max_hold_sec:
                # 持续饱和远超过碰撞时长：更可能是传感器/链路异常，继续"假装没加速度"
                # 会掩盖故障 —— 恢复原样透传。
                self.hold_expired += 1
                # 每次"连续饱和"只在进入这个状态时报一次：这是要人看见的故障转移点，
                # 不能被节流吞掉（实测踩到："替换"告警占满了节流窗口）。
                if not self._hold_expired_reported:
                    self._hold_expired_reported = True
                    self.get_logger().warn(
                        "IMU 加速度持续饱和 %.2f s 超过 max_hold_sec=%.2f："
                        "放弃替换、恢复原样透传（|a|=%.1f m/s²）；"
                        "碰撞不会持续这么久，请检查 IMU/链路"
                        % (now - self.last_good_time, self.max_hold_sec, norm)
                    )
            else:
                # ★ 核心：这一帧加速度不可信 → 用**上一次未饱和**的值替换。
                acceleration.x = self.last_good[0]
                acceleration.y = self.last_good[1]
                acceleration.z = self.last_good[2]
                self.replaced += 1
                self._warn_throttle(
                    now,
                    "IMU 加速度饱和: |a|=%.1f m/s² > %.1f，本帧加速度已替换为上一帧未饱和值 "
                    "(%.2f, %.2f, %.2f)（累计 %d/%d 帧，最大 %.1f）"
                    % (norm, self.max_acceleration, self.last_good[0], self.last_good[1],
                       self.last_good[2], self.saturated, self.samples, self.worst),
                )
        else:
            # 未饱和 → 更新参考值（被替换过的帧不会污染这个参考），并清掉"超时"状态
            self.last_good = (acceleration.x, acceleration.y, acceleration.z)
            self.last_good_time = now
            self._hold_expired_reported = False

        self.publisher.publish(msg)

    def _warn_throttle(self, now, message, key="replaced"):
        """按事件类型分别节流：否则"替换"的告警会把"超时放弃替换"的告警挤掉（实测踩到）。"""
        last = self._last_warn.get(key)
        if last is None or now - last >= self.warn_throttle_sec:
            self._last_warn[key] = now
            self.get_logger().warn(message)

    def _report(self):
        """周期性汇报：IMU 实际频率 + 饱和/替换/无参考/超时 的计数。

        频率这一项很值得看：仿真 IMU 应该是 200 Hz，如果这里只有 60~100 Hz，
        说明链路上有丢帧（ros_gz_bridge 的 QoS/负载），而 ZLIO 的预积分会因此变差 ——
        在排查"里程计抖"的时候，这是第一个该看的数。
        """
        now = time.monotonic()
        elapsed = 0.0 if self._report_time is None else now - self._report_time
        delta = self.samples - self._report_samples
        rate = (delta / elapsed) if elapsed > 0.0 else float("nan")
        self._report_time = now
        self._report_samples = self.samples
        if self.samples == 0:
            self.get_logger().warn("IMU 饱和保护: 一条 IMU 都没收到（检查 input_topic / 命名空间）")
            return
        ratio = 100.0 * self.saturated / max(1, self.samples)
        message = ("IMU 饱和保护统计: %d 帧, 实测 %.0f Hz, 饱和 %d 帧 (%.2f%%), "
                   "替换 %d, 无参考 %d, 超时透传 %d"
                   % (self.samples, rate, self.saturated, ratio,
                      self.replaced, self.no_reference, self.hold_expired))
        if self.saturated:
            message += ", 最大 |a|=%.1f m/s²" % self.worst
        self.get_logger().info(message)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = ImuSaturationGuard()
    except Exception as error:  # noqa: BLE001 - 参数错误要让启动直接失败
        print(f"[imu_saturation_guard] 初始化失败: {error}", file=sys.stderr)
        rclpy.shutdown()
        return 1
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
