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
"""雷达安装角三方对账：URDF/TF ↔ IMU 实测重力 ↔ LIO 发出来的 lidar_odometry。

只读，不发任何指令，起完导航、**车静止**时随时可跑（约 3 s）：

    source install/setup.bash
    python3 script/check_lio_mount_consistency.py

为什么要这个脚本（2026-11-13 实车踩到）
=====================================
模型的雷达安装角和实车**差 60°（roll 符号写反）**时，两套前端各坏一半、都不报错：

* **Point-LIO**：`rot_init` 用 `mapping.gravity` 把世界系扶正，模型误差被它吸收进
  自己的世界系 → `cloud_registered` 看着是好的，但 `/odometry` 的航向带一个 ~17.8°
  的常数偏置（`map->gimbal_yaw` 在出生点就不为 0，实测 17.82°，而云台关节角只有 -2.2°）。
* **ZLIO**：`zlio_bridge` 用 `T_off = TF(gimbal_yaw←livox_imu)·R_0⁻¹` 显式抵消首帧
  位姿，模型误差**原样保留** → 整片 `registered_scan` / `terrain_map_ext` 歪 60°。

肉眼只能看出"哪张图怪"，看不出是谁错了。本脚本拿**静止时 IMU 的比力**当基准
（比力方向 = "世界上方向在雷达系里的表达"，与任何模型无关），三边对账：

| 项 | 来源 | 差多少度说明 |
|---|---|---|
| TF | `gimbal_yaw ← livox_imu` | URDF 的 `lidar_roll/pitch` 或 `--lidar-rpy` 与真实安装不符 |
| LIO | `/lidar_odometry` 姿态 | 前端/桥接实际发出去的姿态（点云/避障吃的是它） |
| CLOUD | `/sensor_scan` 地面法向 | 点云本身歪没歪（最直观） |

三行都换算成"**重力在雷达系里的方向**"再比，单位向量夹角，所以跟相机视角、
车的朝向都无关。判据：与 IMU 实测夹角 ≤ `--tolerance-deg`（默认 2°）。

退出码：0 = 通过；1 = 有超差；2 = 拿不到数据。
"""

import argparse
import math
import sys

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener


def quat_to_rpy(q):
    """(x, y, z, w) -> (roll, pitch, yaw)，rad。"""
    x, y, z, w = q
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def gravity_from_rpy(roll, pitch):
    """重力在"相对父系转过 (roll,pitch) 的那个系"里的表达。

    与 `srm_robot_description.gravity_from_mounting()` /
    `real_mapping_launch.py` / point_lio 的 `mapping.gravity` 同一个公式。
    """
    return np.array([
        math.sin(pitch),
        -math.cos(pitch) * math.sin(roll),
        -math.cos(pitch) * math.cos(roll),
    ])


def rpy_from_gravity(g):
    g = np.asarray(g, float)
    norm = np.linalg.norm(g)
    if norm <= 1e-9:
        return 0.0, 0.0
    g = g / norm
    return math.atan2(-g[1], -g[2]), math.asin(max(-1.0, min(1.0, g[0])))


def angle_deg(a, b):
    a = np.asarray(a, float)
    b = np.asarray(b, float)
    na, nb = np.linalg.norm(a), np.linalg.norm(b)
    if na <= 1e-9 or nb <= 1e-9:
        return float("nan")
    return math.degrees(math.acos(max(-1.0, min(1.0, float(a @ b) / (na * nb)))))


def deg3(rpy):
    return "(%.2f, %.2f, %.2f)°" % tuple(math.degrees(v) for v in rpy)


class Probe(Node):
    def __init__(self, imu_topic, odom_topic, cloud_topic):
        super().__init__("check_lio_mount_consistency")
        self.acc = []
        self.odom = []
        self.clouds = []
        self.create_subscription(Imu, imu_topic, self.on_imu, 50)
        self.create_subscription(Odometry, odom_topic, self.on_odom, 20)
        self.create_subscription(PointCloud2, cloud_topic, self.on_cloud, 5)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def on_imu(self, msg):
        a = msg.linear_acceleration
        self.acc.append((a.x, a.y, a.z))

    def on_odom(self, msg):
        p = msg.pose.pose
        self.odom.append(((p.orientation.x, p.orientation.y, p.orientation.z,
                           p.orientation.w), msg.header.frame_id, msg.child_frame_id))

    def on_cloud(self, msg):
        if len(self.clouds) >= 3:
            return
        points = [(p[0], p[1], p[2]) for p in point_cloud2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True)]
        self.clouds.append(np.asarray(points, float))


def main(argv=None):
    parser = argparse.ArgumentParser(description="雷达安装角三方对账 (TF / IMU / LIO / 点云)")
    parser.add_argument("--imu-topic", default="/livox/imu")
    parser.add_argument("--odom-topic", default="/lidar_odometry")
    parser.add_argument("--cloud-topic", default="/sensor_scan")
    parser.add_argument("--base-frame", default="gimbal_yaw")
    parser.add_argument("--lidar-frame", default="livox_imu")
    parser.add_argument("--duration", type=float, default=3.0, help="采样时长 (秒)，车必须静止")
    parser.add_argument("--tolerance-deg", type=float, default=2.0)
    args = parser.parse_args(argv)

    rclpy.init()
    node = Probe(args.imu_topic, args.odom_topic, args.cloud_topic)
    deadline = node.get_clock().now().nanoseconds + int(args.duration * 1e9)
    while node.get_clock().now().nanoseconds < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    print("=" * 76)
    print("雷达安装角三方对账 (tolerance %.1f°, 采样 %.1f s, 车必须静止)"
          % (args.tolerance_deg, args.duration))
    failures = []

    # ---- 1) IMU：静止比力就是"世界上方向"在本体系里的表达 --------------------
    if len(node.acc) < 50:
        print("[错误] %s 只收到 %d 帧：车在动 / 话题名不对 / 驱动没起"
              % (args.imu_topic, len(node.acc)))
        node.destroy_node()
        rclpy.shutdown()
        return 2
    acc = np.mean(np.asarray(node.acc, float), axis=0)
    norm = float(np.linalg.norm(acc))
    if norm <= 1e-6:
        print("[错误] IMU 加速度均值为零，数据不合法")
        node.destroy_node()
        rclpy.shutdown()
        return 2
    up_body = acc / norm                 # 比力方向（= 世界上方向）
    g_meas = -up_body                    # 重力方向（与 point_lio 的 tmp_gravity 同号）
    meas_roll, meas_pitch = rpy_from_gravity(g_meas)
    print("[IMU  ] %s: %d 帧, |a|=%.4f, 比力=(%+.4f, %+.4f, %+.4f)"
          % (args.imu_topic, len(node.acc), norm, up_body[0], up_body[1], up_body[2]))
    print("        实测安装姿态 roll=%+.2f° pitch=%+.2f°  (单位: |a|≈1 => g, ≈9.81 => m/s²)"
          % (math.degrees(meas_roll), math.degrees(meas_pitch)))
    if not (abs(norm - 1.0) < 0.2 or abs(norm - 9.81) < 0.5):
        print("[警告] |a| 既不像 1 g 也不像 9.81 m/s² —— 车可能在动，这次结果不可信")

    # ---- 2) TF：模型（URDF / --lidar-rpy）说的安装姿态 ----------------------
    tf_rpy = None
    try:
        tf = node.tf_buffer.lookup_transform(args.base_frame, args.lidar_frame, Time(),
                                             Duration(seconds=2.0))
        t = tf.transform.translation
        q = tf.transform.rotation
        tf_rpy = quat_to_rpy((q.x, q.y, q.z, q.w))
        print("[TF   ] %s ← %s: xyz=(%+.3f, %+.3f, %+.3f) rpy=%s"
              % (args.base_frame, args.lidar_frame, t.x, t.y, t.z, deg3(tf_rpy)))
    except Exception as error:  # noqa: BLE001
        print("[错误] TF %s ← %s 查不到: %s" % (args.base_frame, args.lidar_frame, error))
        failures.append("tf_missing")

    # ---- 3) LIO：前端 + 桥接实际发出来的雷达姿态 -----------------------------
    lio_rpy = None
    if not node.odom:
        print("[警告] %s 没数据，跳过 LIO 项" % args.odom_topic)
    else:
        q, frame, child = node.odom[-1]
        lio_rpy = quat_to_rpy(q)
        print("[LIO  ] %s: frame=%s child=%s rpy=%s"
              % (args.odom_topic, frame, child, deg3(lio_rpy)))

    # ---- 4) 点云：地面法向 ---------------------------------------------------
    floor_deg = None
    for cloud in node.clouds:
        if len(cloud) < 200:
            continue
        z = cloud[:, 2]
        low = cloud[z <= np.percentile(z, 25)]
        if len(low) < 50:
            continue
        normal = np.linalg.svd(low - low.mean(axis=0), full_matrices=False)[2][2]
        if normal[2] < 0:
            normal = -normal
        floor_deg = math.degrees(math.acos(max(-1.0, min(1.0, float(normal[2])))))
        print("[CLOUD] %s: %d 点, 地面法向=(%+.3f, %+.3f, %+.3f), 与 +z 夹角 %.2f°"
              % (args.cloud_topic, len(cloud), normal[0], normal[1], normal[2], floor_deg))
        break

    # ---- 判据：全部换算成"重力在本体系里的方向"再比 -------------------------
    print("-" * 76)
    if tf_rpy is not None:
        delta = angle_deg(gravity_from_rpy(tf_rpy[0], tf_rpy[1]), g_meas)
        ok = delta <= args.tolerance_deg
        print("TF   ↔ IMU : %6.2f°  %s" % (delta, "OK" if ok else "**不一致**"))
        if not ok:
            failures.append("tf_vs_imu")
            print("   URDF 的 lidar_roll/pitch（或 --lidar-rpy）与真实安装角差了 %.1f°。" % delta)
            print("   改 URDF 默认值，并同步改 nav2 参数 point_lio.mapping.gravity/gravity_init")
            print("   （两边必须同号同值；公式 9.81*[sin(p), -cos(p)sin(r), -cos(p)cos(r)]）。")
    if lio_rpy is not None:
        delta = angle_deg(gravity_from_rpy(lio_rpy[0], lio_rpy[1]), g_meas)
        ok = delta <= args.tolerance_deg
        print("LIO  ↔ IMU : %6.2f°  %s" % (delta, "OK" if ok else "**不一致**"))
        if not ok:
            failures.append("lio_vs_imu")
            print("   %s 的雷达姿态与 IMU 实测差 %.1f°（点云/避障就是用这个姿态放出去的）："
                  % (args.odom_topic, delta))
            print("   * point_lio 前端：误差被 rot_init 吸收进世界系，检查 mapping.gravity；")
            print("   * zlio 前端：zlio_bridge 的 T_off = TF·R_0⁻¹ 不修正模型误差，")
            print("     点云会原样歪这么多 —— 先修 TF/URDF 那一项。")
    if floor_deg is not None:
        ok = floor_deg <= args.tolerance_deg
        print("地面 ↔ +z  : %6.2f°  %s" % (floor_deg, "OK" if ok else "**歪了**"))
        if not ok:
            failures.append("cloud_tilt")
    print("=" * 76)
    if failures:
        print("结论: 不通过 (%s)" % ", ".join(failures))
    else:
        print("结论: 通过 —— 模型 / IMU / 前端三边一致，点云不会因为外参被转歪")
    node.destroy_node()
    rclpy.shutdown()
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
