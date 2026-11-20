"""真实节点的普通保存、写入失败恢复与先验地图重载。"""

import atexit
from contextlib import ExitStack
import hashlib
import os
from pathlib import Path
import signal
import subprocess
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
from localization_interfaces.msg import FrontendFrame, FrontendOdometry
from nav_msgs.msg import Odometry
import rclpy
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from std_srvs.srv import Trigger
import yaml

_resources = ExitStack()
atexit.register(_resources.close)


def generate_test_description():
    """在独立 ROS 域中通过生产 launch 启动被测节点。"""
    os.environ['ROS_DOMAIN_ID'] = str(_resources.enter_context(domain_id()))
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    root = Path(_resources.enter_context(tempfile.TemporaryDirectory(prefix='backend-ros-')))
    (root / 'logs').mkdir()
    os.environ['ROS_LOG_DIR'] = str(root / 'logs')
    parent = root / 'maps'
    parent.mkdir()
    parameters = {
        'common.frontend': 'zlio', 'common.odom_frame': 'odom',
        'common.input_cloud_in_local_frame': True,
        'prior_map.enabled': False, 'uwb.enabled': False, 'loop.enabled': False,
        'trajectory.save_directory': str(parent / 'saved'),
        'trajectory.save_on_shutdown': False, 'trajectory.map_voxel_resolution': 0.0,
        'trajectory.keyframe.min_interval': 0.0, 'trajectory.keyframe.max_interval': 0.1,
        'visualization.enabled': False,
    }
    config = root / 'parameters.yaml'
    launch_parameters = dict(parameters)
    launch_parameters.pop('prior_map.enabled')
    config.write_text(yaml.safe_dump(launch_parameters), encoding='utf-8')
    entry = Path(__file__).resolve().parents[2] / 'launch/backend.launch.py'
    backend = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(entry)), launch_arguments={
            'config_file': str(config), 'mode': 'mapping', 'rviz': 'false',
        }.items())
    return launch.LaunchDescription([backend, launch_testing.actions.ReadyToTest()]), {
        'root': root, 'parameters': parameters}


def fingerprint(directory):
    """按相对路径计算地图包文件摘要，供保存与重载检查。"""
    return {str(path.relative_to(directory)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in directory.rglob('*') if path.is_file()}


class TestMapService(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        """创建观察服务结果和话题输出的 ROS 测试节点。"""
        rclpy.init()
        cls.node = rclpy.create_node('map_service_observer')

    @classmethod
    def tearDownClass(cls):
        """释放测试节点并关闭 ROS 上下文。"""
        cls.node.destroy_node()
        rclpy.shutdown()

    def wait_until(self, predicate, timeout=10, drive=None):
        """在截止时间内处理 ROS 回调，并可持续驱动输入直到条件满足。"""
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            if drive is not None:
                drive()
            rclpy.spin_once(self.node, timeout_sec=.05)
        self.assertTrue(predicate(), 'condition did not become true before deadline')

    def save(self, client):
        """调用保存服务并检查响应在截止时间内返回。"""
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10)
        self.assertTrue(future.done(), 'save service timed out')
        return future.result()

    def publish_frame(self, publisher, predictor, received, sequence):
        """发布带已知位姿的同步帧和预测，等待后端输出对应时间戳。"""
        message = FrontendFrame(instance_id=901, epoch=0, sequence=sequence)
        header = Header(frame_id='odom')
        header.stamp.sec = 10 + sequence
        message.begin = header.stamp
        message.odometry.header = header
        message.odometry.child_frame_id = 'imu'
        message.odometry.pose.pose.orientation.w = 1.0
        message.odometry.pose.pose.position.x = .5 * sequence
        for axis in range(6):
            message.odometry.pose.covariance[7 * axis] = .01
        cloud_header = Header(frame_id='imu', stamp=header.stamp)
        message.cloud = point_cloud2.create_cloud_xyz32(
            cloud_header, [(1.0 + .1*x, .1*y, 1.0) for x in range(8) for y in range(8)])
        publisher.publish(message)
        prediction = FrontendOdometry(instance_id=901, epoch=0, odometry=message.odometry)
        self.wait_until(
            lambda: any(m.header.stamp.sec == header.stamp.sec for m in received),
            drive=lambda: predictor.publish(prediction))

    def test_save_failure_retry_and_reload(self, root, parameters, test_args):
        """覆盖实际保存、权限失败、重试与第二个节点重载，核对地图内容。"""
        executable = test_args['node']
        destination = root / 'maps/saved'
        client = self.node.create_client(Trigger, '/backend_node/save_map')
        publisher = self.node.create_publisher(FrontendFrame, '/frontend/frame', 10)
        predictor = self.node.create_publisher(FrontendOdometry, '/frontend/odometry', 10)
        received = []
        subscription = self.node.create_subscription(
            Odometry, '/localization/odometry', received.append, 20)
        try:
            self.assertTrue(client.wait_for_service(timeout_sec=10))
            self.assertFalse(self.save(client).success,
                             'empty trajectory must not report a saved map')
            self.wait_until(lambda: publisher.get_subscription_count() > 0
                            and predictor.get_subscription_count() > 0)
            for sequence in range(1, 4):
                self.publish_frame(publisher, predictor, received, sequence)
            self.assertTrue(self.save(client).success)
            before = fingerprint(destination)
            self.assertIn('map.pcd', before)
            self.assertIn('keyframes.txt', before)
            self.assertEqual(len(list((destination / 'clouds').glob('*.pcd'))), 3)
            self.assertNotEqual(
                os.geteuid(), 0, 'permission failure test must run as an unprivileged user')
            destination.parent.chmod(0o500)
            try:
                self.assertFalse(self.save(client).success)
                self.assertEqual(fingerprint(destination), before)
            finally:
                destination.parent.chmod(0o700)
            self.assertTrue(self.save(client).success,
                            'node must remain usable after a failed save')
            self.assertEqual(fingerprint(destination), before)
            self.assertFalse(list(destination.parent.glob('saved.staging.*')))
            self.publish_frame(publisher, predictor, received, 4)
            self.assertTrue(self.save(client).success)
            self.assertEqual(len(list((destination / 'clouds').glob('*.pcd'))), 4)
            self.assertNotEqual(fingerprint(destination), before)
            before = fingerprint(destination)
            # 第二个实际节点只读加载刚保存的包；服务可用意味着启动校验已完成。
            reload_parameters = dict(parameters, **{
                'prior_map.enabled': True, 'prior_map.directory': str(destination),
                'common.frame_topic': '/reload/unused_frames',
                'common.session_topic': '/reload/unused_sessions',
                'common.prediction_topic': '/reload/unused_predictions',
                'services.reset': '/reload/reset',
            })
            config = root / 'reload.yaml'
            config.write_text(
                yaml.safe_dump({'/**': {'ros__parameters': reload_parameters}}), encoding='utf-8')
            with (root / 'reload.log').open('w', encoding='utf-8') as log:
                child = subprocess.Popen(
                    [executable, '--ros-args', '-r', '__node:=reload_backend',
                     '--params-file', str(config)], stdout=log, stderr=subprocess.STDOUT)
                reload_client = self.node.create_client(Trigger, '/reload_backend/save_map')
                try:
                    self.assertTrue(reload_client.wait_for_service(timeout_sec=15))
                    self.assertIsNone(child.poll())
                    self.assertFalse(self.save(reload_client).success,
                                     'immutable prior cannot be saved as runtime map')
                finally:
                    self.node.destroy_client(reload_client)
                    child.send_signal(signal.SIGINT)
                    try:
                        child.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        child.kill()
                        child.wait(timeout=5)
                self.assertEqual(child.returncode, 0, (root / 'reload.log').read_text())
            self.assertEqual(fingerprint(destination), before)
        finally:
            self.node.destroy_subscription(subscription)
            self.node.destroy_publisher(publisher)
            self.node.destroy_publisher(predictor)
            self.node.destroy_client(client)


@launch_testing.post_shutdown_test()
class TestBackendExit(unittest.TestCase):
    def test_clean_exit(self, proc_info):
        """确认被测节点正常退出。"""
        launch_testing.asserts.assertExitCodes(proc_info)
