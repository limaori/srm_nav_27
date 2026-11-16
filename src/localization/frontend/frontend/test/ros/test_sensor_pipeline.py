"""真实传感器链路的原子发布、运行暖重置和输入中断后的冷启动恢复。"""

import atexit
from contextlib import ExitStack
import math
import os
import struct
import tempfile
import time
import unittest

from domain_coordinator import domain_id
import launch
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
import launch_testing
import launch_testing.actions
import launch_testing.asserts
from localization_interfaces.msg import FrontendFrame, FrontendOdometry, FrontendSession
from localization_interfaces.srv import ResetFrontend
from livox_ros_driver2.msg import CustomMsg, CustomPoint
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Imu, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_srvs.srv import SetBool, Trigger

_resources = ExitStack()
atexit.register(_resources.close)


@launch_testing.parametrize('scenario', ['pointcloud2', 'livox', 'standalone', 'startup'])
def generate_test_description(scenario):
    """分别启动协调输入、独立运行和启动中断场景。"""
    os.environ['ROS_DOMAIN_ID'] = str(_resources.enter_context(domain_id()))
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ['ROS_LOG_DIR'] = _resources.enter_context(tempfile.TemporaryDirectory())
    input_type = 'livox' if scenario == 'livox' else 'pointcloud2'
    process = ExecuteProcess(cmd=[
        LaunchConfiguration('node'), '--ros-args',
        '-p', ['config_file:=', LaunchConfiguration('config')],
        '-p', 'input_type:=' + input_type, '-p', 'lidar_topic:=/test/lidar',
        '-p', 'imu_topic:=/test/imu', '-p', 'imu_acceleration_scale:=1.0',
        '-p', 'reset_mode:=warm', '-p', 'publish_tf:=false',
        '-p', 'coordinated_reset:=' + ('true' if scenario in ('pointcloud2', 'livox') else 'false'),
    ], output='screen')
    return launch.LaunchDescription([process, launch_testing.actions.ReadyToTest()]), {
        'process': process}


def set_stamp(stamp, nanoseconds):
    stamp.sec, stamp.nanosec = divmod(nanoseconds, 1_000_000_000)


def stamp_ns(stamp):
    """将 ROS 时间还原为整数纳秒，精确核对预测所消费的输入。"""
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def room_message(input_type):
    points = []
    for i in range(-20, 21):
        for j in range(-20, 21):
            u, v = .1*i, .1*j
            points.extend(((4, u, v), (u, 4, v), (u, v, -2)))
    if input_type == 'livox':
        cloud = CustomMsg()
        cloud.points = [CustomPoint(
            x=float(x), y=float(y), z=float(z),
            offset_time=(index+1)*100_000_000//len(points), tag=0x10 if index % 3 == 0 else 0)
            for index, (x, y, z) in enumerate(points)]
        # x=4 墙面使用有效 tag；额外量程内点验证无效 tag 丢弃，保留共同几何。
        cloud.points.append(CustomPoint(x=7.0, y=0.0, z=0.0, offset_time=100_000_000, tag=0x20))
        cloud.point_num = len(cloud.points)
        return cloud
    cloud = PointCloud2(height=1, width=len(points), point_step=16,
                        row_step=16*len(points), is_dense=True)
    cloud.fields = [PointField(name=name, offset=4*index, count=1,
                               datatype=PointField.FLOAT32 if index < 3 else PointField.UINT32)
                    for index, name in enumerate(('x', 'y', 'z', 'offset_time'))]
    cloud.data = b''.join(struct.pack('<fffI', *point, (index+1)*100_000_000//len(points))
                          for index, point in enumerate(points))
    return cloud


class TestSensorPipeline(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.addCleanup(rclpy.shutdown)
        self.node = rclpy.create_node('sensor_pipeline_observer')
        self.addCleanup(self.node.destroy_node)
        self.imus = self.node.create_publisher(Imu, '/test/imu', 1000)
        self.frames, self.predictions, self.sessions = [], [], []
        self.node.create_subscription(FrontendFrame, '/frontend/frame', self.frames.append, 100)
        self.node.create_subscription(
            FrontendOdometry, '/frontend/odometry', self.predictions.append, 1000)
        self.node.create_subscription(
            FrontendSession, '/frontend/session', self.sessions.append,
            QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.step = 0

    def wait_until(self, predicate, timeout=45):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=.02)
        self.assertTrue(predicate(), 'sensor pipeline did not reach expected state')

    def feed(self, samples, lidar=True):
        """发送连续的传感器时间；纯 IMU 场景加速覆盖实际历史长度。"""
        end = self.step + samples

        def publish_sample():
            """发布惯性采样，并按当前场景选择是否同步发送扫描。"""
            self.step += 1
            timestamp = 1_000_000_000 + self.step * 10_000_000
            imu = Imu()
            set_stamp(imu.header.stamp, timestamp)
            imu.linear_acceleration.z = 9.81
            self.imus.publish(imu)
            if lidar and self.step % 10 == 0:
                set_stamp(self.cloud.header.stamp, timestamp - 100_000_000)
                if self.input_type == 'livox':
                    self.cloud.timebase = timestamp - 100_000_000
                self.clouds.publish(self.cloud)
            if self.step == end:
                timer.cancel()

        timer = self.node.create_timer(.005 if lidar else .001, publish_sample)
        try:
            self.wait_until(lambda: self.step >= end, 15)
        finally:
            self.node.destroy_timer(timer)

    def call(self, service_type, name, request):
        client = self.node.create_client(service_type, name)
        try:
            self.assertTrue(client.wait_for_service(timeout_sec=10))
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=45)
            self.assertTrue(future.done(), name)
            return future.result()
        finally:
            self.node.destroy_client(client)

    def check_boundary(self, response, source_epoch):
        """以同一旧后验验证边界时间、源代次和新系到旧系的刚性变换。"""
        self.assertEqual(response.source_epoch, source_epoch)
        self.wait_until(lambda: any(
            frame.epoch == source_epoch
            and frame.odometry.header.stamp == response.boundary_stamp
            for frame in self.frames), 10)
        previous = next(frame.odometry.pose.pose for frame in reversed(self.frames)
                        if frame.epoch == source_epoch
                        and frame.odometry.header.stamp == response.boundary_stamp)
        # 暖重置把新系原点放在当前机体位置，保留倾角并移除旧系航向。
        translation = response.old_from_new.translation
        for axis in ('x', 'y', 'z'):
            self.assertAlmostEqual(getattr(translation, axis),
                                   getattr(previous.position, axis), delta=1e-9)
        q = previous.orientation
        heading_x = (1-2*(q.y*q.y+q.z*q.z), 2*(q.x*q.y+q.w*q.z))
        if heading_x[0]**2 + heading_x[1]**2 > 1e-12:
            yaw = math.atan2(heading_x[1], heading_x[0])
        else:
            heading_y = (2*(q.x*q.y-q.w*q.z), 1-2*(q.x*q.x+q.z*q.z))
            yaw = math.atan2(-heading_y[0], heading_y[1])
        rotation = response.old_from_new.rotation
        self.assertAlmostEqual(rotation.x, 0, delta=1e-12)
        self.assertAlmostEqual(rotation.y, 0, delta=1e-12)
        self.assertAlmostEqual(
            abs(rotation.z*math.sin(yaw/2) + rotation.w*math.cos(yaw/2)), 1, delta=1e-12)

    def reset_to(self, epoch, has_boundary, instance):
        """核对显式重置边界、进程身份和重复事务的完整幂等结果。"""
        source_epoch = self.frames[-1].epoch
        request = ResetFrontend.Request(
            requester_instance_id=601, transaction_id=epoch, target_epoch=epoch)
        response = self.call(ResetFrontend, '/zlio/reset_internal', request)
        self.assertEqual(response.outcome, ResetFrontend.Response.COMPLETED, response.message)
        self.assertEqual(response.frontend_instance_id, instance)
        self.assertEqual(response.target_epoch, epoch)
        self.assertEqual(response.has_boundary, has_boundary)
        if has_boundary:
            self.check_boundary(response, source_epoch)
        else:
            empty = ResetFrontend.Response()
            self.assertEqual(response.source_epoch, empty.source_epoch)
            self.assertEqual(response.boundary_stamp, empty.boundary_stamp)
            self.assertEqual(response.old_from_new, empty.old_from_new)
        self.assertEqual(self.call(ResetFrontend, '/zlio/reset_internal', request), response)
        self.wait_until(lambda: self.imus.get_subscription_count() > 0
                        and self.clouds.get_subscription_count() > 0, 10)

    def wait_for_input_failure(self):
        """以启动服务拒绝为失败屏障，不额外停止会话或依赖日志文本。"""
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            # 对已启用的运行会话重复启用没有副作用；输入失败后必须要求重置。
            response = self.call(
                SetBool, '/localization_start', SetBool.Request(data=True))
            if not response.success:
                return
            rclpy.spin_once(self.node, timeout_sec=.02)
        self.fail('interrupted input did not close the previous session')

    def check_frames(self, frames, epoch):
        self.assertGreaterEqual(len(frames), 3)
        self.assertTrue(any(frame.lidar_updated and frame.matches > 0 for frame in frames))
        for message in frames:
            self.assertEqual(message.epoch, epoch)
            self.assertEqual(message.cloud.header.stamp, message.odometry.header.stamp)
            self.assertEqual(message.cloud.header.frame_id, message.odometry.child_frame_id)
            self.assertGreater(message.cloud.width, 0)
            self.assertTrue(all(math.isfinite(value) for value in message.odometry.pose.covariance))
            velocity = message.odometry.twist.twist.linear
            self.assertLess(math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2), .02)
        self.assertTrue(all(a.sequence < b.sequence for a, b in zip(frames, frames[1:])))
        if self.input_type == 'livox':
            coordinates = [point[0] for frame in frames for point in point_cloud2.read_points(
                frame.cloud, field_names=('x', 'y', 'z'), skip_nans=True)]
            self.assertLess(max(coordinates), 5.0)
            self.assertTrue(any(x > 3.5 for x in coordinates), 'valid Livox tag was discarded')

    def check_epoch(self, epoch, instance):
        """等待新代次真实出帧，验证序号、进程身份与传感器输出合同。"""
        self.wait_until(
            lambda: len([message for message in self.frames if message.epoch == epoch]) >= 3)
        frames = [message for message in self.frames if message.epoch == epoch]
        self.check_frames(frames, epoch)
        self.assertTrue(all(message.instance_id == instance for message in frames))
        self.assertLessEqual(frames[0].sequence, 2)
        # 初始化可能晚于输入发布完成；用新 IMU 预测确认当前会话已消费输入。
        self.feed(1, lidar=False)
        self.wait_for_prediction(epoch)

    def wait_for_prediction(self, epoch):
        """等待最后一条已发送惯性采样的同代次预测。"""
        timestamp = 1_000_000_000 + self.step * 10_000_000
        self.wait_until(
            lambda: self.predictions and self.predictions[-1].epoch == epoch
            and stamp_ns(self.predictions[-1].odometry.header.stamp) == timestamp, 10)

    def prepare_input(self, input_type):
        """建立指定点云格式的输入，并等待实际传感器订阅就绪。"""
        self.input_type = input_type
        message_type = CustomMsg if input_type == 'livox' else PointCloud2
        self.clouds = self.node.create_publisher(message_type, '/test/lidar', 100)
        self.cloud = room_message(input_type)
        self.wait_until(lambda: self.imus.get_subscription_count() > 0
                        and self.clouds.get_subscription_count() > 0, 10)

    def check_coordinated_recovery(self):
        """依次验证运行暖重置、停车长间隔、历史过期和惯性断流的显式恢复。"""
        self.feed(350)
        self.wait_until(lambda: len(self.frames) >= 3)
        instance = self.frames[-1].instance_id
        self.check_epoch(0, instance)

        # 持续运行且历史完整时保持暖重置，少量新扫描即可继续输出。
        self.reset_to(5, has_boundary=True, instance=instance)
        self.feed(60)
        self.check_epoch(5, instance)

        # 停止后先重置，再让首个新样本跨过 31 秒，禁止沿用旧后验时间。
        self.assertTrue(self.call(Trigger, '/stop', Trigger.Request()).success)
        self.reset_to(6, has_boundary=False, instance=instance)
        self.step += 3100
        self.feed(350)
        self.check_epoch(6, instance)

        # LiDAR 停止而 IMU 持续输入，旧后验退出 30 秒历史后直接重置并恢复。
        self.feed(3100, lidar=False)
        self.wait_for_input_failure()
        self.reset_to(7, has_boundary=False, instance=instance)
        self.feed(350)
        self.check_epoch(7, instance)

        # 一秒真实 IMU 断流小于历史容量，但已超过默认允许的积分间隔。
        self.step += 100
        self.feed(1, lidar=False)
        self.wait_for_input_failure()
        self.reset_to(8, has_boundary=False, instance=instance)
        self.feed(350)
        self.check_epoch(8, instance)
        self.assertTrue(self.call(Trigger, '/stop', Trigger.Request()).success)

    def check_startup_recovery(self):
        """在截止时间内持续制造采样缺口，以同代次会话重建验证启动重试。"""
        self.wait_until(lambda: self.sessions, 10)
        instance = self.sessions[-1].instance_id
        self.assertEqual(self.sessions[-1].epoch, 0)
        notifications = len(self.sessions)

        def publish_gap_sample():
            """任意两个已接收样本均间隔至少 0.2 秒，容忍中间样本丢失。"""
            self.step += 20
            imu = Imu()
            set_stamp(imu.header.stamp, 1_000_000_000 + self.step*10_000_000)
            imu.linear_acceleration.z = 9.81
            self.imus.publish(imu)

        timer = self.node.create_timer(.02, publish_gap_sample)
        try:
            self.wait_until(lambda: len(self.sessions) > notifications, 10)
        finally:
            self.node.destroy_timer(timer)
        self.assertEqual(self.sessions[-1].epoch, 0)
        self.assertEqual(self.sessions[-1].instance_id, instance)
        self.assertFalse(self.sessions[-1].reset_required)
        self.assertFalse(self.sessions[-1].failed)
        self.assertFalse(self.frames)
        self.feed(350)
        self.check_epoch(0, instance)
        self.assertFalse(any(session.reset_required or session.failed for session in self.sessions))

    def check_standalone_recovery(self):
        """缓冲积压重新收集输入，运行中断自动换代，主动停止和非法输入保持关闭。"""
        self.wait_until(lambda: self.sessions, 10)
        instance = self.sessions[-1].instance_id
        self.assertEqual(self.sessions[-1].epoch, 0)
        # IMU 尚未覆盖扫描时，点数容量先耗尽也应重新收集启动输入。
        notifications = len(self.sessions)

        def publish_waiting_scan():
            """只推进雷达时间，在历史时限之前填满有界点云缓冲。"""
            self.step += 10
            set_stamp(self.cloud.header.stamp, 1_000_000_000 + (self.step-10)*10_000_000)
            self.clouds.publish(self.cloud)

        timer = self.node.create_timer(.005, publish_waiting_scan)
        try:
            self.wait_until(lambda: len(self.sessions) > notifications, 10)
        finally:
            self.node.destroy_timer(timer)
        self.assertEqual(self.sessions[-1].epoch, 0)
        self.assertFalse(self.sessions[-1].failed)
        self.assertFalse(self.sessions[-1].reset_required)
        self.feed(350)
        self.check_epoch(0, instance)

        # 同一传感器包重复送达不产生重复预测，也不关闭当前会话。
        duplicate_stamp = 1_000_000_000 + self.step * 10_000_000
        duplicate = Imu()
        set_stamp(duplicate.header.stamp, duplicate_stamp)
        duplicate.linear_acceleration.z = 9.81
        self.imus.publish(duplicate)
        self.feed(1, lidar=False)
        self.wait_for_prediction(0)
        self.assertEqual(sum(stamp_ns(value.odometry.header.stamp) == duplicate_stamp
                             for value in self.predictions), 1)

        # 五个采样周期仍在允许间隔内，保持原代次及预测连续性。
        self.step += 4
        self.feed(1, lidar=False)
        self.wait_for_prediction(0)
        self.assertEqual(self.sessions[-1].epoch, 0)

        # 时间轴回退按新会话恢复，不能沿用旧积分区间或等待人工重置。
        self.step -= 100
        self.feed(1, lidar=False)
        self.wait_until(lambda: self.sessions[-1].epoch == 1
                        and not self.sessions[-1].reset_required, 10)
        self.assertEqual(self.sessions[-1].instance_id, instance)
        self.feed(350)
        self.check_epoch(1, instance)
        self.assertFalse(any(session.reset_required for session in self.sessions))

        # 主动停止后继续发送数据，既不能自动启动，也不能产生新的会话或输出。
        self.assertTrue(self.call(Trigger, '/stop', Trigger.Request()).success)
        stopped_at = 1_000_000_000 + self.step * 10_000_000
        self.step += 19
        self.feed(100)
        self.assertFalse(self.call(
            SetBool, '/localization_start', SetBool.Request(data=True)).success)
        self.assertEqual(self.sessions[-1].epoch, 1)
        self.assertFalse(any(stamp_ns(frame.odometry.header.stamp) > stopped_at
                             for frame in self.frames))
        self.assertFalse(any(stamp_ns(value.odometry.header.stamp) > stopped_at
                             for value in self.predictions))

        # 显式开启独立的新冷会话，再验证非有限测量不属于可自动恢复的采样中断。
        self.reset_to(2, has_boundary=False, instance=instance)
        self.step += 1
        invalid = Imu()
        set_stamp(invalid.header.stamp, 1_000_000_000 + self.step * 10_000_000)
        invalid.linear_acceleration.z = float('nan')
        self.imus.publish(invalid)
        self.wait_for_input_failure()
        self.wait_until(lambda: self.sessions[-1].failed, 10)
        self.feed(100)
        self.assertFalse(self.call(
            SetBool, '/localization_start', SetBool.Request(data=True)).success)
        self.assertEqual(self.sessions[-1].epoch, 2)
        self.assertFalse(self.sessions[-1].reset_required)
        self.assertFalse(any(frame.epoch == 2 for frame in self.frames))

    def test_input_session_contract(self, scenario):
        """按独立进程验证协调恢复与单机自动恢复的输入合同。"""
        self.prepare_input('livox' if scenario == 'livox' else 'pointcloud2')
        if scenario == 'startup':
            self.check_startup_recovery()
        elif scenario == 'standalone':
            self.check_standalone_recovery()
        else:
            self.check_coordinated_recovery()


@launch_testing.post_shutdown_test()
class TestSensorPipelineExit(unittest.TestCase):
    def test_clean_exit(self, proc_info, process):
        launch_testing.asserts.assertExitCodes(proc_info, process=process)
