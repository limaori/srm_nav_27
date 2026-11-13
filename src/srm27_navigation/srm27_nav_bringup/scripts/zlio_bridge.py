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
"""ZLIO 前端 → 本仓库定位契约的桥接节点（`loam_interface` 的 ZLIO 版本）。

它是 `docs/仿真LIO前端替换Z-LIO实施方案(ai).md` §4 里"唯一的新代码"：方案把定位链拆成
四层，ZLIO 只能替第 ① 层（激光惯性里程计），本节点负责第 ② 层（坐标系改写）。

为什么要单独一个节点，而不是 remap 一下 `loam_interface`
=========================================================
`loam_interface` 的点云通路假设"点云已经在 LIO 世界系里"：

    tf2::Transform tf_odom_to_camera_init = tf_odom_to_lidar_odom_;   // 静态 T_base←lidar
    pcl_ros::transformPointCloud(odom_frame_, tf_odom_to_camera_init, *msg, *out);

它对点云只左乘**一个静态安装变换**（语义：点云本来在世界系，我只把世界系的朝向改成底盘
朝向）。而 ZLIO 的 `FrontendFrame.cloud` 是**局部 IMU 系**（去畸变到后验时刻的
`odometry.child_frame_id`），必须左乘**当时的位姿**。硬接的后果是点云只被旋转了安装角、
平移完全丢失 —— 地形图与避障全错，而且没有任何节点会报错。所以只能新写。

本节点做的事（数学定义，与方案 §4.2 一致）
==========================================
设 ZLIO 给出的原始位姿 `R_k = T_zlodom←lidar`（`frame.odometry.pose`），**首帧**的
安装外参 `M = TF(base_frame ← lidar_frame)`（此时云台被 gimbal_spin_gate 锁在零位），
以及首帧的 ZLIO 位姿 `R_0`：

    T_off           = M · R_0⁻¹           # ★ 必须抵消 R_0，见下
    T_odom←lidar(k) = T_off · R_k
    cloud_odom      = do_transform_cloud(frame.cloud, T_odom←lidar(k))
    lidar_odometry  = { frame_id: odom, child_frame_id: lidar_frame, pose: T_odom←lidar(k) }

**为什么必须乘 `R_0⁻¹`（2026-11-05 修正：迁移引入的 bug）**：契约要求"首帧的 odom 位姿 ==
安装外参"（odom 的原点/轴向 = 出生时的 `base_link`），即 `T_off · R_0 = M`，故
`T_off = M · R_0⁻¹`。少乘这个逆，等价于假设 `R_0 ≈ 单位阵`：

* Point-LIO 的世界系按定义就是首帧机体系，`R_0 ≈ I`，所以 `loam_interface` 那样写没问题；
* **ZLIO 不是**：`src/frontend/src/initialization.cpp` 的 `alignGravity()` 把世界系旋转到重力
  朝 -Z（`state.rotation = rotation * state.rotation`），于是 `R_0` 就是那次"扶正"旋转。
  仿真雷达装了 30° roll，`R_0` 就带 30° —— 少乘逆等于把安装 roll 算两次，**odom 系整体被
  绕一根水平轴转了约 30°**：`registered_scan` 与地图平面歪 30°，`terrain_analysis` 把地面
  当斜面；同时 odom 的 xy 被投影压缩（沿某方向最多压到 cos30°），表现为方向相关的尺度误差。

`T_off` 只在收到第一帧时查一次，之后绝不每帧查：每帧查等于把云台角重新引进 odom 帧，
`sensor_scan_generation` 会再减一次，姿态直接翻倍（方案 §4.2 的 ⚠）。

输出沿用 `registered_scan` / `lidar_odometry` 两个**原话题名**，因此下游
（`terrain_analysis(_ext)` / `sensor_scan_generation` / `gimbal_spin_gate`）一个 remap 都
不用加；改动只有"关掉 `loam_interface`"这一条。

换代（epoch）锚定 —— 方案 §4.3，必做项
=====================================
ZLIO 在输入中断后会自己换代冷启动（`coordinated_reset: false`），新代次的 odom 原点重置
为当时的位姿。原样转发会让 `odom` 帧发生一次等于"已走距离"的跳变：
`sensor_scan_generation` 有跳变守卫只清速度，但 `terrain_analysis` 的 odom 系地形图会错位
（避障静默失效）。所以在 bridge 里做一次坐标锚定：

    epoch 变化时： T_off ← T_off · R_last_old · R_first_new⁻¹

即"让新代次第一帧的 odom 位姿 == 老代次最后一帧的 odom 位姿"。这个式子是**相对**的、
与 `R_0` 无关，所以它本来就是对的；首帧那次抵消用的也是同一个形式。

> ⚠ 不要沿用"冷启动后 `R_first_new ≈ 单位阵`"这个说法（原方案 §4.3 的说法）：那是 Point-LIO
> 的性质。ZLIO 的新代次同样会 `alignGravity()` 扶正，`R_first_new` 带的是**当时**的安装倾角
> （仿真约 30°）。锚定只关心"新代次首帧 vs 老代次末帧"的相对关系，所以不受影响；
> 但任何"假设 R ≈ 单位阵"的推导在 ZLIO 上都是错的。

坐标系基准（本仓库的硬约定，方案 §1.1 / §1.3 C3）
================================================
仿真的 `odom` 与 `map` **同源**：原点 = 出生点、轴向 = 出生时的 `base_link`。
`T_off` 把雷达系位姿改写成底盘轴向，正是这条约定的实现方式；不做这个修正，机器人出生就会
偏 63°（那正是 nav2_params_srm_fakeyaw.yaml:190 附近那段标定的结论）。

参数
====
`odom_frame` / `base_frame` / `lidar_frame` / `frame_topic` / `session_topic` /
`planarize_pose`（默认关，平面化只在 `sensor_scan_generation` 做一次）/ `tf_timeout` /
`anchor_on_epoch_change`（A/B 复现用开关）/ `epoch_topic`（诊断）/
`pose_body_offset_xyz`（前端随体偏置补偿，默认 `[0,0,0]` = 关，见下）。

`pose_body_offset_xyz`：ZLIO 在"云台绕偏轴自转"这种高角速度场景下，估出来的雷达平移带
一个**随体**常量偏置 —— 云台转一圈，`odom→gimbal_yaw` 就画一个半径等于该偏置的圆
（2026-11-13 实测 0.145~0.150 m）。该值把偏置从**对外发布的位姿**上右乘减掉：

    pose_pub = pose · T(offset)

作用范围由 `pose_body_offset_apply_to_cloud` 决定（默认 true = 位姿和点云一起改）：
整条链变成正确的，且点云与车位姿的相对几何仍是外参 A（避障/代价地图不受影响）；
代价是点云相对**旧地图**的位置变了（旧图按带误差的链条建的）⇒ 要重跑一次建图，或用
`--reloc` 让 GICP 重新对齐。设成 false 只改位姿，旧图配准不变，但障碍物会相对车位姿
永久偏 |offset|（≈0.26 m 且随车头转）⇒ 会破坏避障，**只适合做 A/B 诊断**。
取值不用手算：`debug_tools/capture_gimbal_spin.py` 会把建议值直接打印成
`pose_body_offset_xyz: [x, y, z]`。默认全零时本节点行为与改造前逐位一致。

输出字段
========
ZLIO 的点云只有 `x/y/z`，而本仓库下游（`terrain_analysis` / `terrain_analysis_ext`）用
`pcl::PointXYZI` 接，缺 `intensity` 会**每条消息**刷一行
`Failed to find match for field 'intensity'`（20 Hz × 2 节点 = 40 行/秒，把真告警淹掉）。
所以这里统一补一个常量 `intensity`（`_with_intensity()`），输出布局与 Point-LIO 时代的
`registered_scan` 一致：`x, y, z, intensity`（全 float32）。

输出 QoS
========
`registered_scan` 与 `lidar_odometry` 都用 reliable + keep_last(5)（与 `loam_interface` 一致）。
注意 `lidar_odometry` **必须** reliable：`terrain_analysis` 用默认 QoS（reliable, depth 5）
订阅它，best-effort 发布会直接建不上连接；`sensor_scan_generation` 的 message_filters 用
best-effort 也能收到 reliable 发布者的数据，所以 reliable 是两边都兼容的那一侧。
"""

import math
import os
import struct
import sys

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import UInt32, UInt64
from tf2_ros import Buffer, TransformException, TransformListener
from tf2_sensor_msgs import do_transform_cloud

from localization_interfaces.msg import FrontendFrame, FrontendSession


# ---------------------------------------------------------------------------
# 只在本节点内使用的刚体变换小工具。
#
# 不依赖 tf_transformations / transforms3d（本工作空间没装，也不值得为一个 7 维复合引入
# 新依赖），四元数按 (w, x, y, z) 存放，与 geometry_msgs 的字段顺序一致但更紧凑。
# ---------------------------------------------------------------------------
def _quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    q = (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )
    norm = math.sqrt(sum(v * v for v in q))
    if norm <= 0.0:
        raise ValueError("degenerate quaternion product")
    return tuple(v / norm for v in q)


def _quat_inverse(q):
    w, x, y, z = q
    norm_sq = w * w + x * x + y * y + z * z
    if norm_sq <= 0.0:
        raise ValueError("degenerate quaternion")
    return (w / norm_sq, -x / norm_sq, -y / norm_sq, -z / norm_sq)


def _quat_rotate(q, v):
    """用单位四元数旋转向量（Rodrigues 形式，无矩阵构造开销）。"""
    w, x, y, z = q
    vx, vy, vz = v
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def _rotation_angle(a, b):
    """两个姿态之间的旋转角（rad），用 |q_a·q_b| 的 acos，与 roll/pitch 无关。"""
    dot = abs(sum(x * y for x, y in zip(a, b)))
    dot = min(1.0, max(-1.0, dot))
    return 2.0 * math.acos(dot)


def _quat_normalize(q):
    w, x, y, z = q
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm <= 0.0:
        return (1.0, 0.0, 0.0, 0.0)
    return (w / norm, x / norm, y / norm, z / norm)


def _with_intensity(cloud, value=0.0):
    """给"只有 x,y,z"的点云补一个 `intensity` 字段。

    为什么必须补（2026-11-05 实跑踩到）：ZLIO 的 `FrontendFrame.cloud` 由
    `sensor_msgs::PointCloud2Modifier::setPointCloud2FieldsByString(1, "xyz")` 生成，
    只有 x/y/z 三个字段。而 `terrain_analysis` / `terrain_analysis_ext` 用
    `pcl::PointXYZI` 接：

        pcl::fromROSMsg(*msg, *laserCloud);   // laserCloud 是 PointCloudXYZI

    PCL 找不到 `intensity` 映射时会**每条消息**刷一行

        Failed to find match for field 'intensity'.

    20 Hz × 2 个节点 = 40 行/秒，把真正的告警全淹掉。逻辑上它随后会把
    `point.intensity` 覆盖成扫描时间戳（terrainAnalysis.cpp:170），所以这个值只要**存在**
    就行，取 0.0 即可 —— 但"存在"这件事只能在这里给，改 ZLIO 上游反而更贵。

    另外顺手保证字段布局与 Point-LIO 时代的 `registered_scan` 一致（x,y,z,intensity），
    下游任何按名字取字段的节点都不用改。
    """
    if any(field.name == "intensity" for field in cloud.fields):
        return cloud
    points = point_cloud2.read_points_numpy(
        cloud, field_names=("x", "y", "z"), skip_nans=False
    )
    if points.size == 0:
        return cloud
    points = points.astype(np.float32, copy=False).reshape(-1, 3)
    packed = np.empty((points.shape[0], 4), dtype=np.float32)
    packed[:, :3] = points
    packed[:, 3] = value
    return point_cloud2.create_cloud(
        cloud.header,
        [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ],
        packed,
    )


class _Rigid:
    """刚体变换：`q` 为 (w, x, y, z)，`t` 为 (x, y, z)。"""

    __slots__ = ("q", "t")

    def __init__(self, q=(1.0, 0.0, 0.0, 0.0), t=(0.0, 0.0, 0.0)):
        self.q = _quat_normalize(q)
        self.t = (float(t[0]), float(t[1]), float(t[2]))

    @classmethod
    def from_transform_stamped(cls, stamped):
        rotation = stamped.transform.rotation
        translation = stamped.transform.translation
        return cls(
            (rotation.w, rotation.x, rotation.y, rotation.z),
            (translation.x, translation.y, translation.z),
        )

    @classmethod
    def from_pose(cls, pose):
        return cls(
            (pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z),
            (pose.position.x, pose.position.y, pose.position.z),
        )

    def __mul__(self, other):
        """复合：`self · other`（先 other 再 self）。"""
        rotated = _quat_rotate(self.q, other.t)
        return _Rigid(
            _quat_mul(self.q, other.q),
            (self.t[0] + rotated[0], self.t[1] + rotated[1], self.t[2] + rotated[2]),
        )

    def inverse(self):
        inverse_rotation = _quat_inverse(self.q)
        rotated = _quat_rotate(inverse_rotation, self.t)
        return _Rigid(inverse_rotation, (-rotated[0], -rotated[1], -rotated[2]))

    def planar(self):
        """压平到平面：保 z 不变、只留 yaw。

        语义与 `loam_interface.cpp` 的 `planarizeBasePose()` 完全一致（z 取 0、roll/pitch
        归零、yaw 保留）。注意本节点默认**不用**它 —— 仿真里这会把倾斜的雷达系当底盘压平，
        yaw 被拉偏几十度（方案 §4.4），平面化只在 `sensor_scan_generation` 做一次。
        """
        w, x, y, z = self.q
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return _Rigid((math.cos(0.5 * yaw), 0.0, 0.0, math.sin(0.5 * yaw)), (self.t[0], self.t[1], 0.0))


class ZlioBridge(Node):
    """把 ZLIO 的 `FrontendFrame` 改写成 `registered_scan` + `lidar_odometry`。"""

    def __init__(self):
        super().__init__("zlio_bridge")

        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        # 原点在地面的参考系（实车 = base_link）。base_frame 不在车底平面时（2026-11-08
        # 云台系改造后是 gimbal_yaw），odom 的 z=0 会跟着抬到云台平面，而代价地图/体素层的
        # min/max_obstacle_height 与 origin_z 都是**绝对 z** —— 抬高之后轮子、低矮台阶会被
        # 当成"太低"丢掉。这里在拍快照时把这段高度反向补偿掉，odom z=0 仍在地面。
        # 留空 = 不补偿（仿真/旧链行为完全不变）。
        self.declare_parameter("ground_frame", "")
        self.declare_parameter("lidar_frame", "front_mid360")
        self.declare_parameter("frame_topic", "zlio/frame")
        self.declare_parameter("session_topic", "zlio/session")
        self.declare_parameter("planarize_pose", False)
        # 前端随体偏置补偿（米，雷达/IMU 体坐标系，2026-11-13）。
        # 语义：ZLIO 在"云台绕偏轴自转"这种高角速度场景下，估出来的雷达平移带一个
        # **随体**常量偏置 —— 云台转一圈，`odom→gimbal_yaw` 就画一个半径等于该偏置的圆
        # （2026-11-13 实测 0.15~0.26 m，而外参本身已被"点云 vs 实物"对照证实是对的）。
        # 补偿方式：把该偏置从位姿上右乘减掉
        #     pose_corrected = pose · T(offset)
        # 两种作用范围（`pose_body_offset_apply_to_cloud`）：
        #   true（默认）= **位姿和点云一起改**。整条链变成"正确"的：云台自转时车不再画圈，
        #       且点云与车位姿仍然自洽（相对几何 = 外参 A，完全不变）⇒ 避障/代价地图不受影响。
        #       代价：点云相对**旧地图**的位置会变（旧图是用带误差的链条建的）⇒ 需要重跑一次
        #       建图，或者用 `--reloc` 让 GICP 重新对齐。
        #   false（**只做诊断**）= 只改位姿，点云仍按未补偿的位姿变换。旧图配准不变，但
        #       障碍物相对车位姿会永久偏 |offset|（≈0.26 m、随车头转）⇒ 会破坏避障，
        #       只用来做"症结是不是它"的 A/B。
        # 取值：跑一次 `debug_tools/capture_gimbal_spin.py`，它会把建议值直接打印成
        #   `pose_body_offset_xyz: [x, y, z]`，粘过来即可；默认 [0,0,0] = 不补偿。
        self.declare_parameter("pose_body_offset_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("pose_body_offset_apply_to_cloud", True)
        self.declare_parameter("tf_timeout", 0.5)
        self.declare_parameter("anchor_on_epoch_change", True)
        self.declare_parameter("epoch_topic", "zlio_bridge/epoch")
        self.declare_parameter("warn_throttle_sec", 5.0)
        # 发散观测（2026-11-05 撞墙飞掉之后加的）：ZLIO 运行时**没有**加速度饱和检查、
        # 没有卡方/新息门限、没有离群点剔除，唯一的门是"一个面都没匹配到"
        # （filter.cpp:112 `if (observation.count == 0) return {}`）。
        # 所以"发散"在它的输出里和"正常"长得一模一样 —— 只能在 bridge 这一层把它显式暴露出来。
        # ---- 累积点云落盘（等价于 Point-LIO 的 pcd_save，见模块 docstring 的"PCD 落盘"）----
        # ZLIO 自己不落盘；这里把它已经转到 odom 系的 registered_scan 累积起来，
        # 退出时按体素降采样写成 PCD（默认写到 point_lio/PCD/scans.pcd，与旧链路同路径、
        # 同坐标系口径：都在建图起点系，转 map 系仍然用 script/pcd_to_mapframe.py）。
        self.declare_parameter("pcd_save_en", False)
        self.declare_parameter("pcd_save_path", "scans.pcd")
        self.declare_parameter("pcd_voxel_leaf", 0.10)      # 米；0 = 不降采样
        self.declare_parameter("pcd_merge_frames", 200)     # 每 N 帧合并一次体素，限制内存
        self.declare_parameter("pcd_save_min_points", 1000)  # 少于这个数不落盘（八成是没数据）
        self.declare_parameter("matches_topic", "zlio_bridge/matches")
        self.declare_parameter("min_matches_warn", 30)
        self.declare_parameter("max_frame_jump_pos", 1.0)      # m/帧：20 Hz 下 = 20 m/s
        self.declare_parameter("max_frame_jump_yaw", 0.5)      # rad/帧：20 Hz 下 = 10 rad/s
        self.declare_parameter("reject_implausible_jump", False)
        # 事件流：把 bridge 认为"不对劲"的时刻也发成话题，示波工具会画成竖线。
        # 为什么需要它：这次实跑里 22 s 姿态崩掉，但日志里的 WARN/ERROR 是节流的，
        # 事后只能从图上猜发生在哪一秒 —— 有了事件流，时刻是确定的。
        self.declare_parameter("events_topic", "zlio_bridge/events")

        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.ground_frame = str(self.get_parameter("ground_frame").value)
        self.lidar_frame = str(self.get_parameter("lidar_frame").value)
        frame_topic = str(self.get_parameter("frame_topic").value)
        session_topic = str(self.get_parameter("session_topic").value)
        self.planarize_pose = bool(self.get_parameter("planarize_pose").value)
        raw_offset = self.get_parameter("pose_body_offset_xyz").value
        try:
            values = [float(v) for v in raw_offset]
        except (TypeError, ValueError) as error:
            raise ValueError(f"pose_body_offset_xyz 需要 3 个数, 收到 {raw_offset!r}") from error
        if len(values) != 3 or not all(math.isfinite(v) for v in values):
            raise ValueError(f"pose_body_offset_xyz 需要 3 个有限数, 收到 {raw_offset!r}")
        # 全零 = 关闭（保持与改造前逐位一致，不做多余乘算）
        self.pose_body_offset = None if not any(values) else _Rigid(t=values)
        self.pose_body_offset_apply_to_cloud = bool(
            self.get_parameter("pose_body_offset_apply_to_cloud").value)
        self.tf_timeout = float(self.get_parameter("tf_timeout").value)
        self.anchor_on_epoch_change = bool(
            self.get_parameter("anchor_on_epoch_change").value
        )
        epoch_topic = str(self.get_parameter("epoch_topic").value)
        self.warn_throttle_sec = float(self.get_parameter("warn_throttle_sec").value)
        matches_topic = str(self.get_parameter("matches_topic").value)
        self.min_matches_warn = int(self.get_parameter("min_matches_warn").value)
        self.max_frame_jump_pos = float(self.get_parameter("max_frame_jump_pos").value)
        self.max_frame_jump_yaw = float(self.get_parameter("max_frame_jump_yaw").value)
        self.reject_implausible_jump = bool(
            self.get_parameter("reject_implausible_jump").value
        )
        events_topic = str(self.get_parameter("events_topic").value)

        # ---- PCD 累积状态 ----
        self.pcd_save_en = bool(self.get_parameter("pcd_save_en").value)
        self.pcd_save_path = str(self.get_parameter("pcd_save_path").value)
        self.pcd_voxel_leaf = float(self.get_parameter("pcd_voxel_leaf").value)
        self.pcd_merge_frames = max(1, int(self.get_parameter("pcd_merge_frames").value))
        self.pcd_save_min_points = int(self.get_parameter("pcd_save_min_points").value)
        self._pcd_chunks = []
        self._pcd_points = None
        self._pcd_frames = 0
        if self.pcd_save_en and np is None:
            self.pcd_save_en = False
            print("[zlio_bridge] 没有 numpy, pcd_save 关闭", file=sys.stderr)

        for name, value in (
            ("odom_frame", self.odom_frame),
            ("base_frame", self.base_frame),
            ("lidar_frame", self.lidar_frame),
        ):
            if not value:
                raise ValueError(f"{name} must not be empty")

        # 缓存状态：T_off、当前代次、上一帧的**原始**（未修正）位姿 —— 换代锚定需要它。
        self.t_off = None            # T_off = T_base←lidar(t0) · R_0⁻¹（含 ZLIO 世界系抵消）
        self.mount = None            # T_base←lidar(t0)，纯安装外参（planarize 用）
        self.first_raw = None        # 首帧 ZLIO 原始位姿 R_0（诊断用）
        self.epoch = None            # 以 frame.epoch 为准（每帧都带）
        self.last_raw = None         # R_last_old，用于换代锚定
        self._last_warn = {}
        self._offset_used_latest = False
        self._session_epoch = None
        self.frames = 0
        self.dropped = 0
        self.jump_events = 0        # 单帧跳变超阈值的次数（发散的直接证据）
        self.rejected_jumps = 0     # 其中被 reject_implausible_jump 拦下的次数
        self.low_match_frames = 0   # 匹配点过少/更新被拒的帧数

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 输出话题名与 loam_interface 完全一致，下游无需 remap。
        # QoS: reliable + keep_last(5)，理由见模块 docstring 的"输出 QoS"。
        output_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        self.cloud_pub = self.create_publisher(PointCloud2, "registered_scan", output_qos)
        self.odom_pub = self.create_publisher(Odometry, "lidar_odometry", output_qos)
        self.epoch_pub = self.create_publisher(UInt64, epoch_topic, 10)
        # 每帧匹配上的面观测点数：掉到个位数 = 几何退化（对着墙/空旷），
        # 这是"位姿开始不可观"最直接的信号，比事后看图早得多。
        self.matches_pub = self.create_publisher(UInt32, matches_topic, 20)
        self.events_pub = self.create_publisher(DiagnosticArray, events_topic, 50)

        # 输入：zlio 的 frame 发布端是 rclcpp::QoS(64)（reliable, volatile）；
        # session 是 QoS(1).transient_local()，订阅端也必须 transient_local 才能拿到
        # 启动时那一帧（否则重启 bridge 会一直等下一代次才知道 instance/epoch）。
        self.create_subscription(
            FrontendFrame, frame_topic, self._on_frame, QoSProfile(depth=64)
        )
        session_qos = QoSProfile(depth=1)
        session_qos.reliability = ReliabilityPolicy.RELIABLE
        session_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(FrontendSession, session_topic, self._on_session, session_qos)

        self.get_logger().info(
            f"ZLIO 桥接已启动: 订阅 {frame_topic} / {session_topic}, 用 "
            f"T({self.base_frame}←{self.lidar_frame}) 把位姿改写到 {self.odom_frame} 系; "
            f"发布 registered_scan + lidar_odometry (child={self.lidar_frame}); "
            f"planarize_pose={self.planarize_pose}, "
            f"anchor_on_epoch_change={self.anchor_on_epoch_change}"
            + (
                f", pose_body_offset_xyz=({self.pose_body_offset.t[0]:+.4f}, "
                f"{self.pose_body_offset.t[1]:+.4f}, {self.pose_body_offset.t[2]:+.4f})"
                "（位姿+点云一起改：链条自洽，但旧地图需重建或用 --reloc）"
                if self.pose_body_offset is not None
                and self.pose_body_offset_apply_to_cloud
                else (
                    "（⚠ 只改位姿：障碍物会相对车位姿偏 |offset|，仅供 A/B 诊断）"
                    if self.pose_body_offset is not None
                    else ""
                )
            )
        )

    # -- 日志节流（sim time 下也能用；rclpy 的 throttle 依赖 wall clock 与版本）------
    def _warn_throttle(self, key, message):
        now = self.get_clock().now().nanoseconds * 1e-9
        last = self._last_warn.get(key)
        if last is None or now - last >= self.warn_throttle_sec:
            self._last_warn[key] = now
            self.get_logger().warn(message)

    def _emit(self, level, name, message):
        """发一条诊断事件（示波工具会把它画成竖线）。level 用 DiagnosticStatus 常量。"""
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        # ⚠ DiagnosticStatus.level 是 `byte`，Python 侧是**单字节 bytes**；传 int 会抛
        # AssertionError。这里兼容两种写法，避免调用处踩坑。
        if isinstance(level, int):
            level = bytes([level])
        status.level = level
        status.name = name
        status.message = message
        status.hardware_id = "zlio"
        array.status = [status]
        try:
            self.events_pub.publish(array)
        except Exception:  # noqa: BLE001 - 诊断不该影响主链路
            pass

    def _error_throttle(self, key, message):
        now = self.get_clock().now().nanoseconds * 1e-9
        last = self._last_warn.get("E:" + key)
        if last is None or now - last >= self.warn_throttle_sec:
            self._last_warn["E:" + key] = now
            self.get_logger().error(message)

    # -- 输入 ------------------------------------------------------------------
    def _on_session(self, msg: FrontendSession):
        self._session_epoch = msg.epoch
        if msg.failed:
            self.get_logger().error(
                f"ZLIO 会话失败 (instance={msg.instance_id}, epoch={msg.epoch}): "
                "前端已停止且禁止自动重试, 需要重启 zlio 节点"
            )
            return
        if msg.reset_required:
            # coordinated_reset=true 时前端不会自己换代，只发这个标志等后端协调。
            # 本链路没有后端，方案要求 coordinated_reset:=false；这里把它显式暴露出来。
            self.get_logger().warn(
                f"ZLIO 报告 reset_required (instance={msg.instance_id}, epoch={msg.epoch}): "
                "coordinated_reset 必须为 false, 否则前端不会自己换代"
            )
            return
        if self.epoch is not None and msg.epoch != self.epoch:
            # 新代次已经宣告但还没有该代次的 frame；下一帧会自动锚定。
            self.get_logger().info(
                f"ZLIO 会话换代: epoch {self.epoch} -> {msg.epoch} (等待该代次的首帧)"
            )

    def _on_frame(self, msg: FrontendFrame):
        stamp = Time.from_msg(msg.odometry.header.stamp)

        # 契约自检（方案 §4.5）：点云坐标系必须等于位姿的子帧，否则整帧丢弃。
        cloud_frame = msg.cloud.header.frame_id
        body_frame = msg.odometry.child_frame_id
        if cloud_frame != body_frame:
            self._warn_throttle(
                "frame_mismatch",
                f"丢弃该帧: 点云坐标系 '{cloud_frame}' != odometry.child_frame_id "
                f"'{body_frame}'",
            )
            self._emit(DiagnosticStatus.WARN, "zlio_bridge:drop",
                       f"cloud/body frame mismatch ({cloud_frame})")
            self.dropped += 1
            return
        if body_frame != self.lidar_frame:
            self._warn_throttle(
                "body_frame_mismatch",
                f"丢弃该帧: ZLIO 的 body_frame '{body_frame}' != lidar_frame "
                f"'{self.lidar_frame}' (检查 zlio 的 body_frame 参数)",
            )
            self.dropped += 1
            return
        if msg.cloud.width * msg.cloud.height == 0:
            # 点云与位姿必须成对发出（契约 C1：同一时刻），空帧整帧丢弃。
            self._warn_throttle("empty_cloud", "丢弃该帧: 点云为 0 点")
            self._emit(DiagnosticStatus.WARN, "zlio_bridge:drop", "empty cloud")
            self.dropped += 1
            return

        try:
            raw = _Rigid.from_pose(msg.odometry.pose.pose)
        except ValueError:
            self._warn_throttle("bad_quat", "丢弃该帧: 位姿四元数退化")
            self.dropped += 1
            return

        # 匹配面观测点数：ZLIO 只会在"一个面都没匹配到"时才拒绝更新（filter.cpp:112），
        # 所以 matches 掉到很低时位姿已经在靠纯 IMU 外推 —— 这时它看起来仍然"正常"。
        self.matches_pub.publish(UInt32(data=int(msg.matches)))
        if not msg.lidar_updated:
            self.low_match_frames += 1
            self._warn_throttle(
                "update_rejected",
                f"ZLIO 本帧没有接受点云更新 (matches={msg.matches}, epoch={msg.epoch}): "
                "位姿在纯 IMU 外推，长时间如此会漂",
            )
            self._emit(DiagnosticStatus.WARN, "zlio_bridge:update_rejected",
                       f"matches={msg.matches}")
        elif msg.matches < self.min_matches_warn:
            self.low_match_frames += 1
            self._warn_throttle(
                "few_matches",
                f"匹配面观测点偏少 (matches={msg.matches} < {self.min_matches_warn}): "
                "几何可能退化（正对单面墙/空旷），平移方向会变得不可观",
            )
            self._emit(DiagnosticStatus.WARN, "zlio_bridge:few_matches",
                       f"matches={msg.matches}")

        # 单帧跳变检查：换代的"重置"由锚定负责，这里只看同一代内的突跳。
        epoch_changed = self.epoch is not None and msg.epoch != self.epoch
        if not epoch_changed and self.last_raw is not None:
            jump_pos = math.sqrt(sum((a - b) ** 2 for a, b in zip(raw.t, self.last_raw.t)))
            jump_yaw = _rotation_angle(self.last_raw.q, raw.q)
            if jump_pos > self.max_frame_jump_pos or jump_yaw > self.max_frame_jump_yaw:
                self.jump_events += 1
                self._error_throttle(
                    "raw_jump",
                    "ZLIO 原始位姿单帧跳变 %.3f m / %.3f rad (阈值 %.2f m / %.2f rad, "
                    "epoch=%d): 这是里程计发散的典型特征（撞击/几何退化/去畸变失效）。"
                    "下游会被原样继承 —— 检查 terrain_map 是否已错位"
                    % (jump_pos, jump_yaw, self.max_frame_jump_pos,
                       self.max_frame_jump_yaw, msg.epoch),
                )
                self._emit(DiagnosticStatus.ERROR, "zlio_bridge:frame_jump",
                           "%.3f m / %.3f rad" % (jump_pos, jump_yaw))
                if self.reject_implausible_jump:
                    self.rejected_jumps += 1
                    self.dropped += 1
                    return

        if self.t_off is None:
            mount = self._capture_offset(stamp)
            if mount is None:
                # 还没拿到 T_off 就丢帧。绝不发单位变换 —— 那是"看起来在跑"的静默错误
                # （同 sensor_scan_generation.cpp 里 getTransform() 的注释）。
                self.dropped += 1
                return
            # ⚠ 2026-11-05 修正（迁移引入的 bug，见实施记录 §5.5）：
            #   T_odom←lidar(k) = T_off · R_k 要成立，等价于要求"首帧的 odom 位姿 == 安装外参"，
            #   即 T_off · R_0 = T_base←lidar(t0)  ⇒  T_off = T_base←lidar(t0) · R_0⁻¹。
            #   原实现少乘了 R_0⁻¹，隐含假设"ZLIO 首帧位姿 ≈ 单位阵"。
            #   Point-LIO 成立（它的世界系就是首帧机体系），**ZLIO 不成立**：
            #   initialization.cpp 的 alignGravity() 会把世界系转成重力朝 -Z
            #   （`state.rotation = rotation * state.rotation`），所以 R_0 就是那次扶正旋转 ——
            #   仿真雷达装了 30° roll，R_0 就带 30°，少乘逆等于把安装 roll 算两次：
            #   odom 系整体被绕一根水平轴转了约 30°，点云与地图平面歪 30°。
            #   形式与 _anchor() 的换代锚定完全一致（那里本来就是对的）。
            ground_offset = self._capture_ground_offset(stamp)
            if ground_offset is None:
                # 与 mount 一样：拿不到就不猜，宁可丢帧。
                self.dropped += 1
                return
            self.mount = mount
            # ⚠ 补偿作用在 t_off（= odom 对齐量）上：点云与位姿走同一个 t_off，
            # 两者一起平移，几何关系不变，但 odom 的 z 原点回到地面。
            self.t_off = ground_offset * mount * raw.inverse()
            self.first_raw = raw
            self.epoch = msg.epoch
            self.get_logger().info(
                f"已捕获 安装外参 T({self.base_frame}←{self.lidar_frame}): "
                f"t=({mount.t[0]:.4f}, {mount.t[1]:.4f}, {mount.t[2]:.4f}) "
                f"q=({mount.q[0]:.4f}, {mount.q[1]:.4f}, {mount.q[2]:.4f}, {mount.q[3]:.4f})"
                f" @ stamp={msg.odometry.header.stamp.sec}."
                f"{msg.odometry.header.stamp.nanosec:09d}"
                + ("（退化: 用最新可用变换，云台此时应为零位）" if self._offset_used_latest else "")
            )
            self.get_logger().info(
                "  首帧 ZLIO 位姿 R_0: 位置 (%.4f, %.4f, %.4f), 相对重力的倾角 %.2f° "
                "(重力对齐后的世界系，非单位阵 → 已按 T_off = T_base←lidar · R_0⁻¹ 抵消)"
                % (raw.t[0], raw.t[1], raw.t[2],
                   math.degrees(math.acos(min(1.0, max(-1.0, abs(raw.q[0])))) * 2.0))
            )
        elif msg.epoch != self.epoch:
            self._anchor(msg.epoch, raw)

        self.last_raw = raw

        pose = self.t_off * raw
        if self.planarize_pose and self.mount is not None:
            # 与 loam_interface 的 planarize_pose 语义一致：只压平底盘位姿，再乘回安装外参。
            # ⚠ 这里必须用 mount（T_base←lidar），不能用 t_off —— 后者含 R_0⁻¹，
            # 已经不是安装外参了。
            pose = (pose * self.mount.inverse()).planar() * self.mount

        # 前端随体偏置补偿：把 `odom→gimbal_yaw` 上那个半径 0.15~0.26 m 的圆减掉。
        # 默认位姿+点云一起改（链条自洽、避障不受影响）；`apply_to_cloud=false` 只改位姿，
        # 仅供 A/B 诊断 —— 见参数注释里的取舍。
        pose_pub = pose if self.pose_body_offset is None else pose * self.pose_body_offset
        cloud_pose = pose_pub if self.pose_body_offset_apply_to_cloud else pose
        self._publish(msg, pose_pub, stamp, cloud_pose=cloud_pose)

    # -- 关键数学 --------------------------------------------------------------
    def _capture_offset(self, stamp):
        """查一次 T(base_frame ← lidar_frame) 并缓存。返回 None 表示还没查到。"""
        timeout = Duration(seconds=self.tf_timeout)
        try:
            stamped = self.tf_buffer.lookup_transform(
                self.base_frame, self.lidar_frame, stamp, timeout
            )
            return _Rigid.from_transform_stamped(stamped)
        except TransformException as ex:
            exact_error = ex

        # 精确时刻查不到时退化为"最新可用变换"是**显式**降级，不是静默回退：
        # 启动阶段云台由 gimbal_spin_gate 锁定在零位，此刻的安装外参与零位等价；
        # 并且只记一次日志 + 在 info 里标注，避免"抓到的 T_off 已经带了自转角"被忽略。
        try:
            stamped = self.tf_buffer.lookup_transform(
                self.base_frame, self.lidar_frame, Time(), timeout
            )
        except TransformException:
            self._warn_throttle(
                "tf_offset",
                f"等待 T({self.base_frame}←{self.lidar_frame}) 以捕获 T_off "
                f"(精确时刻查询失败: {exact_error}); 在此之前丢帧",
            )
            return None
        self._offset_used_latest = True
        return _Rigid.from_transform_stamped(stamped)

    def _capture_ground_offset(self, stamp):
        """把 odom 的 z 原点拉回地面：返回 T(地面补偿) 或 None（还没查到）。

        `ground_frame` 为空时返回单位变换（不做补偿）。否则查
        `T(base_frame <- ground_frame)`，它的 z 是"地面在 base_frame 下方多少"，
        取负得到"base_frame 离地多少"，作为纯 z 平移前乘到 t_off 上。
        """
        if not self.ground_frame:
            return _Rigid()
        timeout = Duration(seconds=self.tf_timeout)
        try:
            stamped = self.tf_buffer.lookup_transform(
                self.base_frame, self.ground_frame, Time(), timeout
            )
        except TransformException:
            self._warn_throttle(
                "tf_ground",
                f"等待 T({self.base_frame}←{self.ground_frame}) 以补偿 odom 的 z，在此之前丢帧",
            )
            return None
        dz = -stamped.transform.translation.z
        self.get_logger().info(
            f"odom z 按 ground_frame={self.ground_frame} 拉回地面: "
            f"base_frame={self.base_frame} 离地 {dz:.4f} m（补偿 {dz:+.4f} m）"
        )
        return _Rigid(t=(0.0, 0.0, dz))

    def _anchor(self, new_epoch, first_raw):
        """换代锚定（方案 §4.3）：T_off ← T_off · R_last_old · R_first_new⁻¹。"""
        previous_epoch = self.epoch
        previous_raw = self.last_raw
        if self.anchor_on_epoch_change and previous_raw is not None:
            try:
                self.t_off = self.t_off * previous_raw * first_raw.inverse()
            except ValueError:
                self.get_logger().error(
                    "换代锚定失败（位姿退化），保留原 T_off：odom 帧会跳变一次"
                )
            else:
                self.get_logger().warn(
                    f"ZLIO 换代 {previous_epoch} -> {new_epoch}: 已锚定 T_off, "
                    "odom 帧保持连续（下游最多看到厘米级小跳）"
                )
                self._emit(DiagnosticStatus.OK, "zlio_bridge:epoch",
                           f"{previous_epoch} -> {new_epoch} (anchored)")
        else:
            self.get_logger().warn(
                f"ZLIO 换代 {previous_epoch} -> {new_epoch}: "
                "anchor_on_epoch_change=false, 未锚定 —— odom 帧会发生一次等于已走距离的跳变, "
                "terrain_analysis 的地形图会整体错位"
            )
        self.epoch = new_epoch
        epoch_msg = UInt64()
        epoch_msg.data = int(new_epoch)
        self.epoch_pub.publish(epoch_msg)

    # -- 输出 ------------------------------------------------------------------
    def _publish(self, msg: FrontendFrame, pose: _Rigid, stamp: Time, cloud_pose=None):
        odometry = Odometry()
        odometry.header.stamp = msg.odometry.header.stamp
        odometry.header.frame_id = self.odom_frame
        odometry.child_frame_id = self.lidar_frame
        odometry.pose.pose.position.x = pose.t[0]
        odometry.pose.pose.position.y = pose.t[1]
        odometry.pose.pose.position.z = pose.t[2]
        odometry.pose.pose.orientation.w = pose.q[0]
        odometry.pose.pose.orientation.x = pose.q[1]
        odometry.pose.pose.orientation.y = pose.q[2]
        odometry.pose.pose.orientation.z = pose.q[3]
        # twist / covariance 留零：与 loam_interface 的输出形态一致。
        # 速度由 sensor_scan_generation 对 pose 做差分得到（它的 max_sample_gap /
        # max_position_jump / max_yaw_jump 三个守卫都在那里），这里给一份"二手速度"
        # 只会多一个可能打架的来源。

        stamped = TransformStamped()
        stamped.header.stamp = msg.odometry.header.stamp
        stamped.header.frame_id = self.odom_frame
        stamped.child_frame_id = self.lidar_frame
        # 点云用 cloud_pose（= 未做 pose_body_offset 补偿的原始 pose）：
        # 补偿只修"发布出去的位姿"，不能把点云一起挪，否则旧地图/先验 PCD 的配准会变。
        cloud_pose = pose if cloud_pose is None else cloud_pose
        stamped.transform.translation.x = cloud_pose.t[0]
        stamped.transform.translation.y = cloud_pose.t[1]
        stamped.transform.translation.z = cloud_pose.t[2]
        stamped.transform.rotation.w = cloud_pose.q[0]
        stamped.transform.rotation.x = cloud_pose.q[1]
        stamped.transform.rotation.y = cloud_pose.q[2]
        stamped.transform.rotation.z = cloud_pose.q[3]

        try:
            cloud = do_transform_cloud(msg.cloud, stamped)
        except Exception as ex:  # noqa: BLE001 - 库异常类型不稳定, 一律按"整帧丢弃"处理
            self._warn_throttle("cloud_transform", f"点云变换失败, 丢弃该帧: {ex}")
            self.dropped += 1
            return

        # 方案 §4.5：这两项必须显式设置，不依赖库对 header 的处理
        # （实测 do_transform_cloud 会把 stamp 清成 0，而 sensor_scan_generation 的
        #  message_filters 是按 stamp 配对 lidar_odometry 与 registered_scan 的）。
        cloud.header.frame_id = self.odom_frame
        cloud.header.stamp = odometry.header.stamp

        # ZLIO 的点云只有 x/y/z；补一个 intensity 字段，否则 terrain_analysis 会用 PCL 的
        # "Failed to find match for field 'intensity'" 把终端刷满（见该函数注释）。
        cloud = _with_intensity(cloud)

        if self.pcd_save_en:
            self._accumulate(cloud)

        self.odom_pub.publish(odometry)
        self.cloud_pub.publish(cloud)
        self.frames += 1
        if self.frames == 1 or self.frames % 200 == 0:
            self.get_logger().info(
                f"已转发 {self.frames} 帧 (丢弃 {self.dropped}), epoch={self.epoch}, "
                f"匹配点过少 {self.low_match_frames} 帧, 单帧跳变 {self.jump_events} 次"
                + (f" (拦下 {self.rejected_jumps})" if self.rejected_jumps else "")
                + f", odom←lidar t=({pose.t[0]:.3f}, {pose.t[1]:.3f}, {pose.t[2]:.3f})"
            )


    # ------------------------------------------------------------------
    # PCD 落盘（等价 Point-LIO 的 pcd_save）
    # ------------------------------------------------------------------
    def _accumulate(self, cloud):
        xyz = _cloud_xyz(cloud)
        if xyz is None or len(xyz) == 0:
            return
        self._pcd_chunks.append(xyz)
        self._pcd_frames += 1
        if len(self._pcd_chunks) >= self.pcd_merge_frames:
            self._merge_chunks()

    def _merge_chunks(self):
        """把新到的帧并进累积体素，控制内存。"""
        if not self._pcd_chunks:
            return
        arr = np.concatenate(self._pcd_chunks) if len(self._pcd_chunks) > 1 else self._pcd_chunks[0]
        self._pcd_chunks = []
        if self._pcd_points is not None and len(self._pcd_points):
            arr = np.concatenate([self._pcd_points, arr])
        if self.pcd_voxel_leaf > 0.0 and len(arr):
            keys = np.round(arr / self.pcd_voxel_leaf).astype(np.int64)
            _, idx = np.unique(keys, axis=0, return_index=True)
            arr = arr[np.sort(idx)]
        self._pcd_points = arr

    def save_pcd_if_enabled(self):
        if not self.pcd_save_en:
            return
        try:
            self._merge_chunks()
            pts = self._pcd_points
            if pts is None or len(pts) < self.pcd_save_min_points:
                self.get_logger().warn(
                    "累积点云只有 %d 点（阈值 %d），不落盘"
                    % (0 if pts is None else len(pts), self.pcd_save_min_points)
                )
                return
            path = os.path.abspath(os.path.expanduser(self.pcd_save_path))
            os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
            with open(path, "wb") as stream:
                header = (
                    "# .PCD v0.7 - Point Cloud Data file format\n"
                    "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
                    "WIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n"
                    % (len(pts), len(pts))
                )
                stream.write(header.encode("ascii"))
                stream.write(pts.astype(np.float32).tobytes())
            self.get_logger().info(
                "累积点云已落盘: %s (%d 点, %d 帧, 体素 %.2f m)"
                % (path, len(pts), self._pcd_frames, self.pcd_voxel_leaf)
            )
        except Exception as ex:  # noqa: BLE001 - 落盘失败不能掩盖真正的退出原因
            self.get_logger().error("累积点云落盘失败: %s" % ex)


def _cloud_xyz(cloud):
    """从 PointCloud2 里取出 (N,3) float32 的 xyz；字段缺失返回 None。"""
    if np is None:
        return None
    offsets = {f.name: f.offset for f in cloud.fields}
    if not {"x", "y", "z"} <= set(offsets):
        return None
    step = cloud.point_step
    count = cloud.width * cloud.height
    if step <= 0 or count <= 0:
        return None
    raw = np.frombuffer(cloud.data, dtype=np.uint8)
    if raw.size < count * step:
        count = raw.size // step
        if count <= 0:
            return None
    arr = raw[: count * step].reshape(count, step)
    out = np.empty((count, 3), dtype=np.float32)
    for i, key in enumerate(("x", "y", "z")):
        off = offsets[key]
        out[:, i] = arr[:, off:off + 4].copy().view(np.float32).ravel()
    finite = np.isfinite(out).all(axis=1)
    return out[finite]


def main(args=None):
    rclpy.init(args=args)
    try:
        node = ZlioBridge()
    except Exception as ex:  # noqa: BLE001 - 参数错误要让启动直接失败, 不要静默跑空
        print(f"[zlio_bridge] 初始化失败: {ex}", file=sys.stderr)
        rclpy.shutdown()
        return 1
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ctrl-C / SIGINT 走后正常落盘（与 Point-LIO 的 pcd_save 同一语义：
        # kill -9 不会执行到这里，这一趟的累积点云就丢了）。
        node.save_pcd_if_enabled()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
