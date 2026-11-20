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

"""`zlocalization_backend_adapter` 的输入契约与 reset 边界回归。

钉住四件事（方案 §3.2 第 4、5 条与验收 C、D）：

1. **适配数学**：`T_odom←lidar = T_off · R_k`，协方差按
   `blkdiag(R_off, R_off) C blkdiag(R_off, R_off)ᵀ` 旋转；随体偏置只改参考点。
2. **frame / time / QoS 契约**：`odom.header.frame_id == odom_frame`、
   `odom.child_frame_id == sensor_frame`、`cloud.header.frame_id == child`、
   点云与位姿时间戳相同 —— backend 逐帧校验这些字段，不符就静默丢帧。
3. **reset 边界所有权**：前端的 `old_from_new` 必须被改写成**恒等边界**
   （本节点已让适配后的 odom 连续），否则 backend 会二次补偿。
4. **配置契约**：legacy 输出、world_cloud、UWB 默认关闭，reset 服务指向适配层。

不启动任何 ROS 节点：用一个只记录 publisher 的假节点驱动适配层。
"""

import math
import sys
from pathlib import Path
from types import SimpleNamespace

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT / "scripts"))

from localization_interfaces.msg import (  # noqa: E402
    FrontendFrame,
    FrontendOdometry,
    FrontendSession,
)
from localization_interfaces.srv import ResetFrontend  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.serialization import serialize_message  # noqa: E402
from sensor_msgs.msg import PointCloud2  # noqa: E402

from zlocalization_backend_adapter import (  # noqa: E402
    BackendInputAdapter,
    identity_transform_message,
    quaternion_to_matrix,
    rotation_covariance,
    shift_covariance_reference_point,
)
from zlio_bridge import _Rigid, _quat_rotate  # noqa: E402

BACKEND_CONFIG = PACKAGE_ROOT.parent / "srm27_zlocalization_backend" / "config"


def _assert(condition, message):
    if not condition:
        raise AssertionError(message)


def _approx(a, b, tolerance=1e-9):
    return abs(a - b) <= tolerance


def _yaw_quaternion(yaw):
    return (math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0))


class _Recorder:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _Logger:
    def __init__(self):
        self.records = []

    def info(self, message):
        self.records.append(("info", message))

    def warn(self, message):
        self.records.append(("warn", message))

    def error(self, message):
        self.records.append(("error", message))


class _Future:
    def __init__(self, reply):
        self._reply = reply

    def add_done_callback(self, callback):
        callback(self)

    def result(self):
        return self._reply


class _FakeClient:
    def __init__(self, reply):
        self.reply = reply
        self.requests = []

    def service_is_ready(self):
        return True

    def call_async(self, request):
        self.requests.append(request)
        return _Future(self.reply)


class _FakeNode:
    """只实现适配层用到的四个工厂方法。"""

    def __init__(self, reply=None):
        self.publishers = {}
        self.subscriptions = {}
        self.services = {}
        self.clients = {}
        self.logger = _Logger()
        self._reply = reply or ResetFrontend.Response()

    def create_publisher(self, message_type, topic, qos):
        recorder = _Recorder()
        self.publishers[topic] = recorder
        return recorder

    def create_subscription(self, message_type, topic, callback, qos):
        self.subscriptions[topic] = callback
        return SimpleNamespace(topic=topic)

    def create_service(self, service_type, name, callback, callback_group=None):
        self.services[name] = callback
        return SimpleNamespace(name=name)

    def create_client(self, service_type, name, callback_group=None):
        client = _FakeClient(self._reply)
        self.clients[name] = client
        return client

    def get_logger(self):
        return self.logger


ARGS = dict(
    sensor_frame="livox_imu",
    odom_frame="odom",
    frame_topic="frontend/frame",
    prediction_topic="frontend/odometry",
    session_topic="frontend/session",
    prediction_input_topic="zlio/frontend_odometry",
    reset_service="zlocalization/reset_frontend",
    frontend_reset_service="/zlio/reset_internal",
    pose_provider=lambda pose: None,
)


def _adapter(reply=None):
    node = _FakeNode(reply)
    return node, BackendInputAdapter(node, **ARGS)


def _odometry(stamp_sec=10, stamp_nano=5, covariance=None):
    odometry = Odometry()
    odometry.header.stamp.sec = stamp_sec
    odometry.header.stamp.nanosec = stamp_nano
    odometry.header.frame_id = "zlodom"
    odometry.child_frame_id = "livox_imu"
    odometry.pose.pose.position.x = 1.5
    odometry.pose.pose.orientation.w = 1.0
    if covariance is not None:
        odometry.pose.covariance = covariance
    return odometry


def _frame(odometry=None):
    frame = FrontendFrame()
    frame.instance_id = 7
    frame.epoch = 3
    frame.sequence = 11
    frame.odometry = odometry if odometry is not None else _odometry()
    frame.begin.sec = 9
    frame.cloud = PointCloud2()
    frame.cloud.header.stamp = frame.odometry.header.stamp
    frame.cloud.header.frame_id = "livox_imu"
    frame.cloud.height = 1
    frame.cloud.width = 3
    frame.matches = 42
    frame.lidar_updated = True
    return frame


# ---------------------------------------------------------------------------
# 1. 适配数学
# ---------------------------------------------------------------------------
def test_quaternion_to_matrix_matches_zlib_rotation():
    """四元数→矩阵与 `_quat_rotate` 对同一向量的结果一致。"""
    quaternion = (0.5, 0.5, 0.5, 0.5)  # 绕 (1,1,1) 转 120°
    rotation = quaternion_to_matrix(quaternion)
    for vector in ((1.0, 0.0, 0.0), (0.3, -0.4, 0.5)):
        expected = [0.0, 0.0, 0.0]
        for index in range(3):
            basis = [0.0, 0.0, 0.0]
            basis[index] = vector[index]
            rotated = _quat_rotate(quaternion, tuple(basis))
            for axis in range(3):
                expected[axis] += rotated[axis]
        for axis in range(3):
            got = sum(rotation[axis][column] * vector[column] for column in range(3))
            _assert(_approx(got, expected[axis], 1e-9),
                    f"旋转矩阵与 _quat_rotate 不一致: {got} vs {expected[axis]}")


def test_rotation_covariance_rotates_every_block():
    """90° yaw：位置块 x/y 互换，旋转块 0.1/0.2 互换，交叉块同步转。"""
    covariance = [0.0] * 36
    covariance[0] = 1.0   # x
    covariance[7] = 2.0   # y
    covariance[14] = 3.0  # z
    covariance[21] = 0.1  # rx
    covariance[28] = 0.2  # ry
    covariance[35] = 0.3  # rz
    rotated = rotation_covariance(covariance, quaternion_to_matrix(_yaw_quaternion(math.pi / 2)))
    _assert(_approx(rotated[0], 2.0), f"x 方差应变成 y 的 2.0，实际 {rotated[0]}")
    _assert(_approx(rotated[7], 1.0), f"y 方差应变成 x 的 1.0，实际 {rotated[7]}")
    _assert(_approx(rotated[14], 3.0), "z 方差不受 yaw 影响")
    _assert(_approx(rotated[21], 0.2), f"rx 方差应变成 ry 的 0.2，实际 {rotated[21]}")
    _assert(_approx(rotated[28], 0.1), f"ry 方差应变成 rx 的 0.1，实际 {rotated[28]}")
    _assert(_approx(rotated[35], 0.3), "rz 方差不受 yaw 影响")


def test_rotation_covariance_preserves_trace_and_symmetry():
    """旋转不改变迹（正交变换），且结果仍对称。"""
    covariance = [0.0] * 36
    for index in range(6):
        covariance[index * 6 + index] = 0.1 * (index + 1)
    covariance[0 * 6 + 1] = covariance[1 * 6 + 0] = 0.05
    rotation = quaternion_to_matrix((0.9, 0.1, -0.2, 0.3))
    rotated = rotation_covariance(covariance, rotation)
    _assert(_approx(sum(rotated[i * 6 + i] for i in range(6)),
                    sum(covariance[i * 6 + i] for i in range(6)), 1e-9), "迹应保持不变")
    for row in range(6):
        for column in range(6):
            _assert(_approx(rotated[row * 6 + column], rotated[column * 6 + row], 1e-9),
                    f"协方差不对称: ({row},{column})")


def test_offset_shifts_covariance_reference_point():
    """参考点右移 1 m（x）后：rz 的不确定度变成 y 方向的位移不确定度。"""
    covariance = [0.0] * 36
    covariance[35] = 0.04   # rz 方差 (0.2 rad)^2
    _assert(shift_covariance_reference_point(covariance, (0.0, 0.0, 0.0)) == covariance,
            "零位移必须原样返回")
    shifted = shift_covariance_reference_point(covariance, (1.0, 0.0, 0.0))
    _assert(_approx(shifted[7], 0.04, 1e-12), f"y 方差应为 0.04，实际 {shifted[7]}")
    _assert(_approx(shifted[0], 0.0), "x 方差不应被旋转不确定度影响")
    _assert(_approx(shifted[8], 0.0), "y-z 交叉项应为 0")


# ---------------------------------------------------------------------------
# 2. frame / time 契约
# ---------------------------------------------------------------------------
def test_frame_adaptation_sets_backend_contract_fields():
    """适配帧的 frame 名、时间戳、位姿与协方差必须满足 backend 的逐帧校验。"""
    node, adapter = _adapter()
    covariance = [0.0] * 36
    covariance[0] = 1.0
    covariance[7] = 2.0
    odometry = _odometry(covariance=covariance)
    frame = _frame(odometry)
    t_off = _Rigid(_yaw_quaternion(math.pi / 2), (5.0, -2.0, 0.3))
    raw = _Rigid.from_pose(frame.odometry.pose.pose)
    pose = t_off * raw
    adapter.on_frame(frame, t_off, pose)

    published = node.publishers["frontend/frame"].messages
    _assert(len(published) == 1, "每帧应发布一次")
    adapted = published[0]
    _assert(adapted.odometry.header.frame_id == "odom",
            f"frame_id 应为 odom，实际 {adapted.odometry.header.frame_id}")
    _assert(adapted.odometry.child_frame_id == "livox_imu", "child_frame_id 必须是 sensor_frame")
    _assert(adapted.cloud.header.frame_id == adapted.odometry.child_frame_id,
            "点云坐标系必须等于 odometry.child_frame_id")
    _assert((adapted.cloud.header.stamp.sec == adapted.odometry.header.stamp.sec
             and adapted.cloud.header.stamp.nanosec == adapted.odometry.header.stamp.nanosec),
            "点云与位姿时间戳必须相等")
    _assert((adapted.odometry.header.stamp.sec, adapted.odometry.header.stamp.nanosec) == (10, 5),
            "时间戳不得被改写")
    _assert(adapted.instance_id == 7 and adapted.epoch == 3 and adapted.sequence == 11,
            "会话身份必须原样透传")
    _assert(adapted.matches == 42 and adapted.lidar_updated, "匹配字段必须原样透传")
    # 位姿：T_off · R，且点上 90° yaw（1.5 m 沿 x 变成沿 y）。
    _assert(_approx(adapted.odometry.pose.pose.position.x, 5.0, 1e-6),
            f"x 应为 5.0，实际 {adapted.odometry.pose.pose.position.x}")
    _assert(_approx(adapted.odometry.pose.pose.position.y, -0.5, 1e-6),
            f"y 应为 -0.5，实际 {adapted.odometry.pose.pose.position.y}")
    _assert(_approx(adapted.odometry.pose.pose.position.z, 0.3, 1e-6), "z 应原样保留")
    # 协方差：90° yaw 后 x/y 方差互换。
    _assert(_approx(adapted.odometry.pose.covariance[0], 2.0, 1e-6),
            "位置 x 方差应变成 2.0")
    _assert(_approx(adapted.odometry.pose.covariance[7], 1.0, 1e-6),
            "位置 y 方差应变成 1.0")


def test_frame_adaptation_keeps_raw_payload_and_warns_once():
    """点云 frame 不符时只改写一次并告警一次，不复制点云数据。"""
    node, adapter = _adapter()
    frame = _frame()
    frame.cloud.header.frame_id = "front_mid360"
    t_off = _Rigid()
    pose = t_off * _Rigid.from_pose(frame.odometry.pose.pose)
    adapter.on_frame(frame, t_off, pose)
    adapter.on_frame(frame, t_off, pose)
    warnings = [record for record in node.logger.records if record[0] == "warn"]
    _assert(len(warnings) == 1, f"frame 不符只应告警一次，实际 {len(warnings)}")


def test_session_forwarded_unchanged():
    """会话身份必须逐字段透传（backend 用它仲裁 instance/epoch）。"""
    node, adapter = _adapter()
    session = FrontendSession()
    session.instance_id = 5
    session.epoch = 2
    session.reset_required = True
    session.failed = False
    adapter.on_session(session)
    published = node.publishers["frontend/session"].messages[0]
    _assert(published.instance_id == 5 and published.epoch == 2, "身份必须原样")
    _assert(published.reset_required and not published.failed, "故障标志必须原样")


def test_prediction_is_dropped_until_offset_is_available():
    """还没抓到 T_off 时绝不发单位变换（那是"看起来在跑"的静默错误）。"""
    node = _FakeNode()
    adapter = BackendInputAdapter(node, **dict(ARGS, pose_provider=lambda pose: None))
    prediction_callback = node.subscriptions["zlio/frontend_odometry"]
    message = FrontendOdometry()
    message.odometry = _odometry()
    prediction_callback(message)
    _assert(not node.publishers["frontend/odometry"].messages, "没有 T_off 时不应发布预测")


# ---------------------------------------------------------------------------
# 3. reset 边界所有权
# ---------------------------------------------------------------------------
def _reply(outcome=ResetFrontend.Response.COMPLETED, has_boundary=True, source_epoch=3,
           target_epoch=4, boundary_sec=12):
    reply = ResetFrontend.Response()
    reply.outcome = outcome
    reply.message = "ZLIO warm reset"
    reply.requester_instance_id = 99
    reply.transaction_id = 7
    reply.target_epoch = target_epoch
    reply.frontend_instance_id = 11
    reply.reset_generation = 2
    reply.has_boundary = has_boundary
    reply.source_epoch = source_epoch
    reply.boundary_stamp.sec = boundary_sec
    reply.old_from_new.rotation.w = 0.9238795
    reply.old_from_new.rotation.z = 0.3826834
    reply.old_from_new.translation.x = 4.0
    reply.old_from_new.translation.y = -1.0
    return reply


def _call_reset(node, request, response):
    returned = node.services["zlocalization/reset_frontend"](request, response)
    _assert(isinstance(returned, ResetFrontend.Response),
            "rclpy 服务回调必须返回 Response，否则发送响应时 TypeError 并退出")
    _assert(returned is response, "必须返回已填充的响应")
    serialize_message(returned)
    return returned


def test_reset_early_exits_return_response():
    """不可用、超时和传输错误也必须返回可发送的响应，保留事务身份。"""
    class PendingFuture:
        def add_done_callback(self, callback):
            pass

    class FailedFuture:
        def add_done_callback(self, callback):
            callback(self)

        def result(self):
            raise RuntimeError("test transport failure")

    for case, expected in (
        ("unavailable", ResetFrontend.Response.REJECTED),
        ("timeout", ResetFrontend.Response.IN_PROGRESS),
        ("failure", ResetFrontend.Response.INDETERMINATE),
    ):
        node, adapter = _adapter()
        if case == "unavailable":
            adapter.reset_client.service_is_ready = lambda: False
        elif case == "timeout":
            adapter._reset_timeout = 0.0
            adapter.reset_client.call_async = lambda request: PendingFuture()
        else:
            adapter.reset_client.call_async = lambda request: FailedFuture()
        request = ResetFrontend.Request()
        request.requester_instance_id = 99
        request.transaction_id = 7
        request.target_epoch = 4
        response = _call_reset(node, request, ResetFrontend.Response())
        _assert(response.outcome == expected, f"{case}: outcome 错误")
        _assert((response.requester_instance_id, response.transaction_id,
                 response.target_epoch) == (99, 7, 4), f"{case}: 事务身份丢失")


def test_reset_boundary_becomes_identity():
    """前端边界（45° + 4 m 平移）必须被改写为恒等，否则 backend 二次补偿。"""
    reply = _reply()
    node, adapter = _adapter(reply)
    adapter._last_epoch = 3
    request = ResetFrontend.Request()
    request.requester_instance_id = 99
    request.transaction_id = 7
    request.target_epoch = 4
    response = ResetFrontend.Response()
    _call_reset(node, request, response)
    _assert(response.outcome == ResetFrontend.Response.COMPLETED, "终态必须透传")
    _assert(response.has_boundary, "暖重置必须给出边界")
    _assert(response.source_epoch == 3, f"源代次应透传，实际 {response.source_epoch}")
    _assert(response.boundary_stamp.sec == 12, "边界时刻应透传")
    transform = response.old_from_new
    _assert(_approx(transform.rotation.w, 1.0) and _approx(transform.rotation.z, 0.0),
            "边界旋转必须是恒等")
    _assert(_approx(transform.translation.x, 0.0) and _approx(transform.translation.y, 0.0),
            "边界平移必须是零")


def test_cold_reset_synthesises_identity_boundary_from_last_frame():
    """前端冷启动没有边界时，用最后一帧的代次/时刻补一条恒等边界。"""
    reply = _reply(has_boundary=False, source_epoch=0)
    node, adapter = _adapter(reply)
    adapter._last_epoch = 3
    adapter._last_stamp = _odometry(stamp_sec=9, stamp_nano=0).header.stamp
    request = ResetFrontend.Request()
    request.transaction_id = 7
    request.target_epoch = 4
    response = ResetFrontend.Response()
    _call_reset(node, request, response)
    _assert(response.has_boundary, "适配后 odom 连续，冷启动也应给出恒等边界")
    _assert(response.source_epoch == 3, "源代次应取适配层最后一帧的代次")
    _assert(response.boundary_stamp.sec == 9, "边界时刻应取适配层最后一帧")


def test_reset_without_any_frame_does_not_claim_boundary():
    """一帧都没发过时不冒充边界（backend 会退回"假设停车"契约）。"""
    reply = _reply(has_boundary=False, source_epoch=0)
    node, adapter = _adapter(reply)
    request = ResetFrontend.Request()
    request.transaction_id = 7
    request.target_epoch = 4
    response = ResetFrontend.Response()
    _call_reset(node, request, response)
    _assert(not response.has_boundary, "没有帧身份时不应声明边界")


def test_identity_transform_message_is_unit():
    """恒等边界的四元数必须满足 backend 的单位性检查。"""
    transform = identity_transform_message()
    _assert(transform.rotation.w == 1.0 and transform.rotation.x == 0.0, "必须是单位四元数")
    _assert(transform.translation.x == 0.0 and transform.translation.z == 0.0, "必须是零平移")


# ---------------------------------------------------------------------------
# 4. 配置契约
# ---------------------------------------------------------------------------
def _config_text(name):
    return (BACKEND_CONFIG / name).read_text(encoding="utf-8")


def test_backend_config_disables_unused_outputs_and_uwb():
    """方案 §3.2 第 8 条：无消费者的输出与未接入的输入必须默认关闭。"""
    import yaml

    params = yaml.safe_load(_config_text("backend.yaml"))
    _assert(params["output"]["legacy"]["enabled"] is False, "legacy 输出默认关闭")
    _assert(params["output"]["legacy"]["publish_tf"] is False, "不得额外发布 map→body")
    _assert(params["output"]["world_cloud"]["enabled"] is False, "world_cloud 默认关闭")
    _assert(params["uwb"]["enabled"] is False, "本车无 UWB，默认关闭")
    _assert(params["common"]["frame_topic"] == "frontend/frame", "话题必须是相对名")
    _assert(params["common"]["odom_frame"] == "odom", "backend 的 odom 必须是本仓库 odom")
    _assert(params["reset"]["service"] == "zlocalization/reset_frontend",
            "reset 必须走适配层代理")
    _assert(params["prior_map"]["directory"] == "", "先验地图目录必须由入口给绝对路径")
    _assert(params["trajectory"]["save_directory"] == "",
            "轨迹目录必须由入口给绝对路径（不依赖进程 CWD）")


def test_backend_profiles_only_override_sensor_frame():
    """两个平台 profile 只允许覆盖 sensor_frame，算法参数不许分叉。"""
    import yaml

    for name, expected in (("srm27_localization_real.yaml", "livox_imu"),
                           ("srm27_localization_sim.yaml", "front_mid360")):
        params = yaml.safe_load(_config_text(name))
        _assert(params.get("extends") == "backend.yaml", f"{name} 必须继承 backend.yaml")
        overrides = {key for key in params if key != "extends"}
        _assert(overrides == {"common"}, f"{name} 只允许覆盖 common，实际 {overrides}")
        _assert(params["common"] == {"sensor_frame": expected},
                f"{name} 只允许覆盖 sensor_frame，实际 {params['common']}")


def main():
    tests = [
        test_quaternion_to_matrix_matches_zlib_rotation,
        test_rotation_covariance_rotates_every_block,
        test_rotation_covariance_preserves_trace_and_symmetry,
        test_offset_shifts_covariance_reference_point,
        test_frame_adaptation_sets_backend_contract_fields,
        test_frame_adaptation_keeps_raw_payload_and_warns_once,
        test_session_forwarded_unchanged,
        test_prediction_is_dropped_until_offset_is_available,
        test_reset_early_exits_return_response,
        test_reset_boundary_becomes_identity,
        test_cold_reset_synthesises_identity_boundary_from_last_frame,
        test_reset_without_any_frame_does_not_claim_boundary,
        test_identity_transform_message_is_unit,
        test_backend_config_disables_unused_outputs_and_uwb,
        test_backend_profiles_only_override_sensor_frame,
    ]
    for test in tests:
        test()
        print(f"[OK] {test.__name__}")
    print(f"\n全部通过: {len(tests)} 项")
    return 0


if __name__ == "__main__":
    sys.exit(main())
