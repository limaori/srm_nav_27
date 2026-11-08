"""在生成类型的真实序列化边界保留身份、纳秒和复合消息。"""

import pytest
from localization_interfaces.msg import FrontendFrame, FrontendOdometry, FrontendSession
from localization_interfaces.srv import ResetFrontend
from rclpy.serialization import deserialize_message, serialize_message


@pytest.mark.parametrize('message', [
    FrontendSession(instance_id=2**64-1, epoch=2**63+1),
    FrontendSession(instance_id=17, epoch=4, reset_required=True),
    FrontendSession(instance_id=17, epoch=4, failed=True),
    FrontendOdometry(instance_id=19, epoch=7),
    ResetFrontend.Request(requester_instance_id=2**64-1, transaction_id=17, target_epoch=42),
    ResetFrontend.Response(outcome=ResetFrontend.Response.INDETERMINATE,
                           message='failure; retry\n原因', requester_instance_id=3,
                           transaction_id=17, target_epoch=42,
                           frontend_instance_id=29, reset_generation=8),
])
def test_wire_roundtrip(message):
    assert deserialize_message(serialize_message(message), type(message)) == message


def test_atomic_frame_keeps_measurement_time_and_identity():
    message = FrontendFrame(instance_id=31, epoch=4, sequence=2**63, matches=93, lidar_updated=True)
    message.begin.sec, message.begin.nanosec = 8, 999999999
    message.odometry.header.stamp.sec = 9
    message.odometry.header.frame_id = 'odom'
    message.odometry.child_frame_id = 'imu'
    message.cloud.header = message.odometry.header
    message.cloud.data = [0, 17, 255]
    assert deserialize_message(serialize_message(message), FrontendFrame) == message


def test_warm_reset_keeps_boundary_transform_and_source_epoch():
    """完整保留旧代次、纳秒后验时刻以及新系到旧系的旋转和平移。"""
    message = ResetFrontend.Response(
        outcome=ResetFrontend.Response.COMPLETED, message='warm boundary',
        requester_instance_id=31, transaction_id=17, target_epoch=2**63+1,
        frontend_instance_id=19, reset_generation=8,
        has_boundary=True, source_epoch=2**63)
    message.boundary_stamp.sec = 29
    message.boundary_stamp.nanosec = 999999999
    message.old_from_new.translation.x = 3.25
    message.old_from_new.translation.y = -7.5
    message.old_from_new.translation.z = .125
    message.old_from_new.rotation.x = .5
    message.old_from_new.rotation.y = -.5
    message.old_from_new.rotation.z = .5
    message.old_from_new.rotation.w = .5
    assert deserialize_message(serialize_message(message), ResetFrontend.Response) == message
