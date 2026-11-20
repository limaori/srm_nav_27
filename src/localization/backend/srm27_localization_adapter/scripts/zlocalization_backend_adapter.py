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
"""ZLIO 前端观测 → `srm27_zlocalization_backend` 输入契约的适配层。

它是 `docs/design/Z-Localization整体迁移实施方案.md` §3.2 第 4、5 条要求的
"backend 专用输入适配"，由 `zlio_bridge` 持有并调用（**同一个节点**拥有 `T_off`
与换代锚定，见下）。不做这一层、只 remap 话题名，会在**没有任何报错**的情况下
把车定位到一个歪掉的世界里（问题分析 §B2）。

为什么必须长在 bridge 里，而不是另起一个节点
==========================================
backend 需要的位姿是**本仓库的 odom 系**：`T_odom←lidar(k) = T_off · R_k`，其中

    T_off = T(地面补偿) · T(base_frame←lidar_frame) · R_0⁻¹

`T_off` 里的 `R_0` 是"抓到第一帧时"的前端原始位姿。如果适配层自己再算一份
`T_off`，它抓到的"第一帧"可能和 bridge 不是同一帧（订阅建立时刻差几帧，或 TF
精确时刻查询失败退化成最新值），两个 odom 系就差一个常量刚体变换 D：

    backend 认为 map→odom = T_map_sensor · (D · T_odom←lidar)⁻¹
    导航实际需要 map→odom = T_map_sensor · (T_odom←lidar)⁻¹

于是整条 TF 链会稳定地错 D —— 而且 `localization_state` 仍然是 NORMAL。
所以本模块**不自己算 T_off**，由 `zlio_bridge` 把当帧的 `T_off` 与适配位姿传进来。

本模块负责的三件事
==================
1. 位姿/协方差改写。`FrontendFrame.odometry` 与 `FrontendOdometry.odometry` 的
   位姿左乘 `T_off`（与 bridge 发布的 `lidar_odometry` 逐位一致），协方差按
   点约定（point convention）旋转：

       C' = blkdiag(R_off, R_off) · C · blkdiag(R_off, R_off)ᵀ

   推导：前端协方差由 `zlio::poseCovariance()` 给出，是
   `[δp − [p]×δθ; δθ]` 的协方差（世界系左扰动）。左乘常量 `T_off` 后

       δp' = R_off·δp,  p' = t_off + R_off·p
       ρ' − [p']×θ' = R_off·(ρ − [p]×θ)

   即位置块、旋转块、交叉块一起左乘 `R_off`，与平移 `t_off` 无关。
2. 点云/时间/坐标系契约。backend 逐帧校验
   `odometry.header.frame_id == common.odom_frame`、
   `odometry.child_frame_id == common.sensor_frame`、
   `cloud.header.frame_id == child_frame_id`、两者时间戳相等。
   点云**保持在前端局部（IMU/雷达）系**（`input_cloud_in_local_frame: true`），
   不在这里做任何变换 —— backend 会自己用位姿把它搬到地图系。这也避免了
   20 Hz 下多一次整帧点云变换。
3. reset / epoch 唯一所有权（方案 §3.2 第 5 条）。见下。

reset 与 epoch 的所有权
=======================
本仓库的连续性补偿只有一个所有者：**bridge 的 `T_off` 锚定**。
换代时 `T_off ← T_off · R_last_old · R_first_new⁻¹`，于是适配后的 odom 坐标
在换代前后连续（旧代次末帧位姿 == 新代次首帧位姿）。backend 自己的
`PoseOutput::reframeOdometry()` 会再按 reset 回复里的坐标边界乘一次 ——
如果两个都做，全局位姿就会凭空跳一个锚定增量（问题分析 §B4）。

所以本模块做 reset 服务代理：backend 调 `zlocalization/reset_frontend`
（`reset.service`），本模块转发给前端 `/zlio/reset_internal`，并把回复里的
坐标边界改写成**恒等边界**。这在本链路上是精确的，不是近似：

* 前端的 `old_from_new` 描述"同一物理状态下新代次→老代次"的坐标关系；
* bridge 已经让适配后的 odom 坐标连续，所以适配后新旧坐标关系**就是**恒等；
* backend 收到恒等边界 ⇒ `map_from_odom` 不变，且按"边界有效"处理
  —— 比"假设停车"更强：移动中 reset 也不会累积误差。

同时前端必须 `coordinated_reset:=true`：它只置 `reset_required` 等后端来协调，
不再自己换代；否则一次断流会触发两次换代（问题分析 §B5）。

前端返回 `INDETERMINATE` 或没有边界时的兜底：只要能确定源代次与时间戳，
仍按恒等上报，因为 bridge 的锚定与前端是 warm 还是 cold 无关 —— 两种情况下
适配后的 odom 都连续。
"""

import math
import threading

from geometry_msgs.msg import Transform
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from localization_interfaces.msg import FrontendFrame, FrontendOdometry, FrontendSession
from localization_interfaces.srv import ResetFrontend


# ---------------------------------------------------------------------------
# 纯数学：位姿/协方差的坐标改写。不依赖 ROS 节点，便于单独做数值测试。
# ---------------------------------------------------------------------------
def quaternion_to_matrix(quaternion):
    """把 `(w, x, y, z)` 单位四元数转成 3×3 旋转矩阵（行主序 list）。"""
    w, x, y, z = quaternion
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm <= 0.0:
        raise ValueError("degenerate quaternion")
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    return [
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)],
        [2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)],
        [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)],
    ]


def matrix_vector(matrix, vector):
    """3×3 矩阵（行主序 list）乘三维向量。"""
    return [sum(matrix[row][column] * vector[column] for column in range(3))
            for row in range(3)]


def _skew(vector):
    x, y, z = vector
    return [[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]]


def rotation_covariance(covariance, rotation):
    """按点约定旋转 6×6 位姿协方差（`covariance` 为 36 元素行主序 list）。

    `C' = blkdiag(R, R) · C · blkdiag(R, R)ᵀ`：位置块与旋转块一起左乘 `R`，
    交叉块同理。
    """
    if len(covariance) != 36:
        raise ValueError("pose covariance must have 36 entries")
    rotated = [0.0] * 36
    for block_row in range(2):
        for block_column in range(2):
            for row in range(3):
                for column in range(3):
                    value = 0.0
                    for left in range(3):
                        for right in range(3):
                            value += (rotation[row][left]
                                      * covariance[(block_row * 3 + left) * 6
                                                   + (block_column * 3 + right)]
                                      * rotation[column][right])
                    rotated[(block_row * 3 + row) * 6 + (block_column * 3 + column)] = value
    return rotated


def shift_covariance_reference_point(covariance, displacement):
    """把点约定协方差的参考点平移 `displacement`（odom 系向量）。

    位姿的参考点平移 `d` 之后，点误差变成 `ε_p − [d]×θ`，即

        C'' = A · C · Aᵀ,  A = [[I, −[d]×], [0, I]]

    `zlio_bridge` 的 `pose_body_offset_xyz` 是**右乘**的随体平移，等价于参考点
    在 odom 系里移动 `R_pose · offset`；本函数负责这一项耦合。位移为零时原样返回。
    """
    if all(value == 0.0 for value in displacement):
        return list(covariance)
    skew = _skew(displacement)
    transform = [[0.0] * 6 for _ in range(6)]
    for row in range(3):
        transform[row][row] = 1.0
        transform[row + 3][row + 3] = 1.0
        for column in range(3):
            transform[row][column + 3] = -skew[row][column]
    shifted = [0.0] * 36
    for row in range(6):
        for column in range(6):
            value = 0.0
            for left in range(6):
                left_value = transform[row][left]
                if left_value == 0.0:
                    continue
                for right in range(6):
                    right_value = transform[column][right]
                    if right_value == 0.0:
                        continue
                    value += left_value * covariance[left * 6 + right] * right_value
            shifted[row * 6 + column] = value
    return shifted


def identity_transform_message():
    """构造 `geometry_msgs/Transform` 的恒等变换。"""
    transform = Transform()
    transform.rotation.w = 1.0
    return transform


# ---------------------------------------------------------------------------
# ROS 侧适配层。
# ---------------------------------------------------------------------------
class BackendInputAdapter:
    """把 ZLIO 观测改写成 backend 的输入契约，并代理 reset 事务。"""

    def __init__(self, node, *, sensor_frame, odom_frame, frame_topic, prediction_topic,
                 session_topic, prediction_input_topic, reset_service,
                 frontend_reset_service, pose_provider, frame_depth=50,
                 prediction_depth=200, reset_timeout_seconds=15.0,
                 service_callback_group=None, client_callback_group=None):
        """创建发布/订阅/服务；`pose_provider` 由 `zlio_bridge` 提供。

        `pose_provider(raw_pose)` 返回 `(t_off, adapted_pose, body_offset)`：
        `t_off` 为 None 表示还没抓到安装外参；`adapted_pose` 是含
        `pose_body_offset` 的 `T_odom←lidar`；`body_offset` 为随体平移或 None。
        """
        for name, value in (
            ("backend_sensor_frame", sensor_frame),
            ("backend_odom_frame", odom_frame),
            ("backend_frame_topic", frame_topic),
            ("backend_prediction_topic", prediction_topic),
            ("backend_session_topic", session_topic),
            ("backend_reset_service", reset_service),
            ("backend_frontend_reset_service", frontend_reset_service),
        ):
            if not value:
                raise ValueError(f"{name} must not be empty")
        if pose_provider is None:
            raise ValueError("pose_provider must not be None")
        self._node = node
        self._sensor_frame = str(sensor_frame)
        self._odom_frame = str(odom_frame)
        self._reset_timeout = max(0.1, float(reset_timeout_seconds))
        self._pose_provider = pose_provider

        frame_qos = QoSProfile(depth=max(1, int(frame_depth)),
                               reliability=ReliabilityPolicy.RELIABLE)
        prediction_qos = QoSProfile(depth=max(1, int(prediction_depth)),
                                    reliability=ReliabilityPolicy.RELIABLE)
        session_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        # 前端 session 是 transient local：backend 晚启动也必须能拿到 instance/epoch。
        session_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.frame_publisher = node.create_publisher(FrontendFrame, frame_topic, frame_qos)
        self.prediction_publisher = node.create_publisher(
            FrontendOdometry, prediction_topic, prediction_qos)
        self.session_publisher = node.create_publisher(
            FrontendSession, session_topic, session_qos)
        self.prediction_subscription = node.create_subscription(
            FrontendOdometry, prediction_input_topic, self._on_prediction, prediction_qos)
        self.reset_service = node.create_service(
            ResetFrontend, reset_service, self._on_reset,
            callback_group=service_callback_group)
        self.reset_client = node.create_client(
            ResetFrontend, frontend_reset_service, callback_group=client_callback_group)

        # 最近一次已发布的适配帧身份；reset 代理在缺少前端边界时用它构造恒等边界。
        # 只在帧回调里写、在服务回调里读，单个属性赋值在 CPython 下是原子的。
        self._last_epoch = None
        self._last_stamp = None
        self._warned_cloud_frame = False
        self._warned_cloud_stamp = False
        self._frames = 0
        self._predictions = 0
        self._resets = 0
        node.get_logger().info(
            f"backend 输入适配已启用: frame={frame_topic} prediction={prediction_topic} "
            f"session={session_topic} sensor_frame={self._sensor_frame} "
            f"reset={reset_service} -> {frontend_reset_service}"
        )

    # -- 帧 ------------------------------------------------------------------
    def on_frame(self, message, t_off, pose, body_offset=None):
        """用当帧 `T_off` 改写 `FrontendFrame` 并发布给 backend。

        `pose` 是 `zlio_bridge` 已经算好的 `T_odom←lidar`（含 `pose_body_offset`），
        与它发布的 `lidar_odometry` 逐位一致 —— 这是"TF owner 唯一"的前提。
        """
        adapted = FrontendFrame()
        adapted.instance_id = message.instance_id
        adapted.epoch = message.epoch
        adapted.sequence = message.sequence
        adapted.begin = message.begin
        adapted.matches = message.matches
        adapted.lidar_updated = message.lidar_updated
        adapted.odometry = self._adapt_odometry(message.odometry, pose, body_offset)
        adapted.cloud = message.cloud
        odometry_stamp = message.odometry.header.stamp
        if adapted.cloud.header.frame_id != self._sensor_frame:
            if not self._warned_cloud_frame:
                self._warned_cloud_frame = True
                self._node.get_logger().warn(
                    "前端点云 frame_id='%s' 与 backend sensor_frame='%s' 不一致，"
                    "已按 sensor_frame 改写（backend 会逐帧校验该字段）"
                    % (adapted.cloud.header.frame_id, self._sensor_frame)
                )
            adapted.cloud.header.frame_id = self._sensor_frame
        if (adapted.cloud.header.stamp.sec != odometry_stamp.sec
                or adapted.cloud.header.stamp.nanosec != odometry_stamp.nanosec):
            if not self._warned_cloud_stamp:
                self._warned_cloud_stamp = True
                self._node.get_logger().warn(
                    "前端点云与位姿时间戳不一致，已对齐到 odometry.header.stamp"
                )
            adapted.cloud.header.stamp = odometry_stamp
        self.frame_publisher.publish(adapted)
        self._last_epoch = message.epoch
        self._last_stamp = odometry_stamp
        self._frames += 1

    # -- 高频预测 -------------------------------------------------------------
    def _on_prediction(self, message):
        """预测只用于输出投影、不参与因子图，但位姿同样要左乘 `T_off`。"""
        handled = self._pose_provider(message.odometry.pose.pose)
        if handled is None:
            return
        _, pose, body_offset = handled
        adapted = FrontendOdometry()
        adapted.instance_id = message.instance_id
        adapted.epoch = message.epoch
        adapted.odometry = self._adapt_odometry(message.odometry, pose, body_offset)
        self.prediction_publisher.publish(adapted)
        self._predictions += 1

    # -- 会话 -----------------------------------------------------------------
    def on_session(self, message):
        """原样转发前端会话身份（instance/epoch/reset_required/failed）。"""
        self.session_publisher.publish(message)

    # -- reset 代理 -----------------------------------------------------------
    def _on_reset(self, request, response):
        """把 backend 的重置事务转发给前端，并把坐标边界改写为恒等边界。"""
        if not self.reset_client.service_is_ready():
            response.outcome = ResetFrontend.Response.REJECTED
            response.message = "front-end reset service unavailable"
            self._copy_request_identity(request, response)
            return response
        forwarded = ResetFrontend.Request()
        forwarded.requester_instance_id = request.requester_instance_id
        forwarded.transaction_id = request.transaction_id
        forwarded.target_epoch = request.target_epoch
        done = threading.Event()
        future = self.reset_client.call_async(forwarded)
        future.add_done_callback(lambda _future: done.set())
        if not done.wait(self._reset_timeout):
            # 非终态：backend 会沿用同一事务 ID 重试，前端按事务去重不会重复重置。
            response.outcome = ResetFrontend.Response.IN_PROGRESS
            response.message = "front-end reset still in progress"
            self._copy_request_identity(request, response)
            return response
        try:
            reply = future.result()
        except Exception as error:  # noqa: BLE001 - 传输异常按不确定终态交回 backend
            response.outcome = ResetFrontend.Response.INDETERMINATE
            response.message = f"front-end reset transport failed: {error}"
            self._copy_request_identity(request, response)
            return response
        response.outcome = reply.outcome
        response.message = reply.message
        response.requester_instance_id = reply.requester_instance_id
        response.transaction_id = reply.transaction_id
        response.target_epoch = reply.target_epoch
        response.frontend_instance_id = reply.frontend_instance_id
        response.reset_generation = reply.reset_generation
        response.has_boundary = False
        if reply.outcome == ResetFrontend.Response.COMPLETED:
            boundary = self._adapted_identity_boundary(reply)
            if boundary is not None:
                source_epoch, stamp = boundary
                response.has_boundary = True
                response.source_epoch = source_epoch
                response.boundary_stamp = stamp
                response.old_from_new = identity_transform_message()
        self._resets += 1
        self._node.get_logger().warn(
            f"backend reset 事务 {request.transaction_id}: outcome={reply.outcome} "
            f"frontend_boundary={reply.has_boundary} "
            f"adapted_identity_boundary={response.has_boundary}"
            "（适配后 odom 连续，边界按恒等上报）"
        )
        return response

    @staticmethod
    def _copy_request_identity(request, response):
        response.requester_instance_id = request.requester_instance_id
        response.transaction_id = request.transaction_id
        response.target_epoch = request.target_epoch

    def _adapted_identity_boundary(self, reply):
        """返回适配后 odom 的恒等边界 `(source_epoch, stamp)`；无法确定时返回 None。"""
        source_epoch = int(reply.source_epoch) if reply.has_boundary else self._last_epoch
        if source_epoch is None:
            return None
        target_epoch = int(reply.target_epoch)
        if not 0 <= source_epoch < target_epoch:
            return None
        stamp = reply.boundary_stamp if reply.has_boundary else self._last_stamp
        if stamp is None or stamp.sec < 0:
            return None
        return source_epoch, stamp

    # -- 内部 -----------------------------------------------------------------
    def _adapt_odometry(self, odometry, pose, body_offset=None):
        """就地把 `nav_msgs/Odometry` 改成 odom 系的适配位姿、协方差与帧名。"""
        adapted = odometry
        adapted.header.frame_id = self._odom_frame
        adapted.child_frame_id = self._sensor_frame
        adapted.pose.pose.position.x = pose.t[0]
        adapted.pose.pose.position.y = pose.t[1]
        adapted.pose.pose.position.z = pose.t[2]
        adapted.pose.pose.orientation.w = pose.q[0]
        adapted.pose.pose.orientation.x = pose.q[1]
        adapted.pose.pose.orientation.y = pose.q[2]
        adapted.pose.pose.orientation.z = pose.q[3]
        rotation = quaternion_to_matrix(pose.q)
        covariance = rotation_covariance(odometry.pose.covariance, rotation)
        if body_offset is not None:
            covariance = shift_covariance_reference_point(
                covariance, matrix_vector(rotation, body_offset))
        adapted.pose.covariance = covariance
        return adapted

    def summary(self):
        """返回 `(帧, 预测, reset)` 转发计数，便于退出或诊断时打印。"""
        return self._frames, self._predictions, self._resets
