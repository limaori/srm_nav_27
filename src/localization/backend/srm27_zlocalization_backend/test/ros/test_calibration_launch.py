"""使用真实标定 launch 验证参数类型、覆盖传递与节点生命周期。"""

import atexit
from contextlib import ExitStack
import os
from pathlib import Path
import tempfile
import unittest

from domain_coordinator import domain_id
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
import launch_testing.asserts
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
import rclpy

_resources = ExitStack()
atexit.register(_resources.close)


def generate_test_description():
    """在独立 ROS 域中通过生产 launch 启动被测节点。"""
    os.environ['ROS_DOMAIN_ID'] = str(_resources.enter_context(domain_id()))
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ['ROS_LOG_DIR'] = _resources.enter_context(tempfile.TemporaryDirectory())
    path = Path(__file__).resolve().parents[2] / 'launch/body_calibration.launch.py'
    entry = IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)), launch_arguments={
        'map_frame': 'test_map', 'odom_topic': '/calibration/input',
        'angle_min_samples': '12', 'offset_min_yaw_span_deg': '210.0',
        'target_yaw_deg': '17.5', 'use_sim_time': 'false',
    }.items())
    return launch.LaunchDescription([entry, launch_testing.actions.ReadyToTest()])


class TestCalibrationParameters(unittest.TestCase):
    def test_parameter_values_and_types(self):
        """读取实际节点参数，核对 launch 覆盖后的值与 ROS 参数类型。"""
        rclpy.init()
        node = rclpy.create_node('calibration_parameter_observer')
        try:
            client = node.create_client(GetParameters, '/body_calibration_node/get_parameters')
            self.assertTrue(client.wait_for_service(timeout_sec=10))
            future = client.call_async(GetParameters.Request(names=[
                'map_frame', 'odom_topic', 'angle_min_samples',
                'offset_min_yaw_span_deg', 'target_yaw_deg', 'use_sim_time',
            ]))
            rclpy.spin_until_future_complete(node, future, timeout_sec=10)
            self.assertTrue(future.done())
            values = future.result().values
            self.assertEqual(len(values), 6)
            self.assertEqual([value.type for value in values], [
                ParameterType.PARAMETER_STRING, ParameterType.PARAMETER_STRING,
                ParameterType.PARAMETER_INTEGER, ParameterType.PARAMETER_DOUBLE,
                ParameterType.PARAMETER_DOUBLE, ParameterType.PARAMETER_BOOL,
            ])
            self.assertEqual(values[0].string_value, 'test_map')
            self.assertEqual(values[1].string_value, '/calibration/input')
            self.assertEqual(values[2].integer_value, 12)
            self.assertEqual(values[3].double_value, 210.0)
            self.assertEqual(values[4].double_value, 17.5)
            self.assertFalse(values[5].bool_value)
        finally:
            node.destroy_node()
            rclpy.shutdown()


@launch_testing.post_shutdown_test()
class TestCalibrationExit(unittest.TestCase):
    def test_clean_exit(self, proc_info):
        """确认被测节点正常退出。"""
        launch_testing.asserts.assertExitCodes(proc_info)
