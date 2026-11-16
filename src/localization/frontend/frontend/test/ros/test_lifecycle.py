"""通过真实服务验证启动门、停止、幂等重置及会话通知。"""

import atexit
from contextlib import ExitStack
import os
from pathlib import Path
import tempfile
import time
import unittest

from domain_coordinator import domain_id
import launch
import launch_testing
import launch_testing.actions
import launch_testing.asserts
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from localization_interfaces.msg import FrontendSession
from localization_interfaces.srv import ResetFrontend
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_srvs.srv import SetBool, Trigger

_resources = ExitStack()
atexit.register(_resources.close)


def generate_test_description():
    os.environ['ROS_DOMAIN_ID'] = str(_resources.enter_context(domain_id()))
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ['ROS_LOG_DIR'] = _resources.enter_context(tempfile.TemporaryDirectory())
    entry = Path(__file__).resolve().parents[2] / 'launch/frontend.launch.py'
    frontend = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(entry)), launch_arguments={
            'rviz': 'false', 'config': LaunchConfiguration('config'),
            'wait_for_start': 'true', 'reset_mode': 'cold', 'input_type': 'pointcloud2',
        }.items())
    return launch.LaunchDescription([frontend, launch_testing.actions.ReadyToTest()])


class TestFrontendLifecycle(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('frontend_lifecycle_observer')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def call(self, service_type, name, request):
        client = self.node.create_client(service_type, name)
        try:
            self.assertTrue(client.wait_for_service(timeout_sec=10), name)
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=10)
            self.assertTrue(future.done(), name)
            return future.result()
        finally:
            self.node.destroy_client(client)

    def test_session_lifecycle(self):
        sessions = []
        subscription = self.node.create_subscription(
            FrontendSession, '/frontend/session', sessions.append,
            QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        try:
            deadline = time.monotonic() + 10
            while not sessions and time.monotonic() < deadline:
                rclpy.spin_once(self.node, timeout_sec=.1)
            self.assertTrue(sessions, 'startup must publish a retained session')
            initial = sessions[-1]
            self.assertNotEqual(initial.instance_id, 0)
            self.assertFalse(self.call(
                SetBool, '/localization_start', SetBool.Request(data=False)).success)
            self.assertTrue(self.call(
                SetBool, '/localization_start', SetBool.Request(data=True)).success)
            self.assertTrue(self.call(
                SetBool, '/localization_start', SetBool.Request(data=True)).success)
            self.assertTrue(self.call(Trigger, '/stop', Trigger.Request()).success)
            self.assertFalse(self.call(
                SetBool, '/localization_start', SetBool.Request(data=True)).success)
            request = ResetFrontend.Request(
                requester_instance_id=701, transaction_id=1, target_epoch=7)
            reply = self.call(ResetFrontend, '/zlio/reset_internal', request)
            self.assertEqual(reply.outcome, ResetFrontend.Response.COMPLETED)
            self.assertEqual(reply.frontend_instance_id, initial.instance_id)
            duplicate = self.call(ResetFrontend, '/zlio/reset_internal', request)
            self.assertEqual(duplicate, reply)
            deadline = time.monotonic() + 10
            while not any(s.epoch == 7 for s in sessions) and time.monotonic() < deadline:
                rclpy.spin_once(self.node, timeout_sec=.1)
            self.assertTrue(any(
                s.epoch == 7 and s.instance_id == initial.instance_id for s in sessions))
            self.assertTrue(self.call(
                SetBool, '/localization_start', SetBool.Request(data=True)).success)
            self.assertTrue(self.call(Trigger, '/stop', Trigger.Request()).success)
        finally:
            self.node.destroy_subscription(subscription)


@launch_testing.post_shutdown_test()
class TestFrontendExit(unittest.TestCase):
    def test_clean_exit(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
