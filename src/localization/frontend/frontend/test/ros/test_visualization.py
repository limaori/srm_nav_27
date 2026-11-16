"""真实显示节点的坐标变换、有界历史和会话隔离；不启动 RViz。"""

import atexit
from contextlib import ExitStack
import math
import os
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
from localization_interfaces.msg import FrontendFrame, FrontendSession
from nav_msgs.msg import Path
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

_resources = ExitStack()
atexit.register(_resources.close)


def generate_test_description():
    os.environ['ROS_DOMAIN_ID'] = str(_resources.enter_context(domain_id()))
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ['ROS_LOG_DIR'] = _resources.enter_context(tempfile.TemporaryDirectory())
    process = ExecuteProcess(cmd=[
        LaunchConfiguration('node'), '--ros-args', '-p', 'path_max_poses:=2',
        '-p', 'map_max_voxels:=2', '-p', 'map_voxel_size:=0.1', '-p', 'map_publish_period:=0.0',
    ], output='screen')
    return launch.LaunchDescription([process, launch_testing.actions.ReadyToTest()]), {
        'process': process}


def frame(sequence, points, epoch=2, world='odom'):
    message = FrontendFrame(instance_id=11, epoch=epoch, sequence=sequence)
    header = Header(frame_id=world)
    header.stamp.sec = 10 + sequence
    message.odometry.header = header
    message.odometry.child_frame_id = 'imu'
    message.odometry.pose.pose.position.x = 10.0
    message.odometry.pose.pose.position.y = 20.0
    message.odometry.pose.pose.orientation.z = math.sqrt(.5)
    message.odometry.pose.pose.orientation.w = math.sqrt(.5)
    message.cloud = point_cloud2.create_cloud_xyz32(
        Header(frame_id='imu', stamp=header.stamp), points)
    return message


def xyz(cloud):
    return list(point_cloud2.read_points(cloud, field_names=('x', 'y', 'z'), skip_nans=False))


class TestVisualization(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.addCleanup(rclpy.shutdown)
        self.node = rclpy.create_node('visualization_observer')
        self.addCleanup(self.node.destroy_node)
        retained = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.sessions = self.node.create_publisher(FrontendSession, '/frontend/session', retained)
        self.frames = self.node.create_publisher(FrontendFrame, '/frontend/frame', 10)
        self.clouds, self.maps, self.paths = [], [], []
        self.node.create_subscription(
            PointCloud2, '/frontend/cloud_registered', self.clouds.append, 10)
        self.node.create_subscription(
            PointCloud2, '/frontend/cloud_map', self.maps.append, retained)
        self.node.create_subscription(Path, '/frontend/path', self.paths.append, retained)
        self.wait_until(lambda: self.frames.get_subscription_count() > 0
                        and self.sessions.get_subscription_count() > 0)

    def wait_until(self, predicate):
        deadline = time.monotonic() + 10
        while not predicate() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=.05)
        self.assertTrue(predicate(), 'visualization output did not arrive')

    def publish_frame(self, message):
        self.frames.publish(message)
        stamp = message.odometry.header.stamp.sec
        self.wait_until(lambda: self.maps and self.paths
                        and self.maps[-1].header.stamp.sec == stamp
                        and self.paths[-1].header.stamp.sec == stamp)

    def test_geometry_capacity_and_session_boundaries(self):
        self.sessions.publish(FrontendSession(instance_id=11, epoch=2))
        self.wait_until(lambda: self.maps and self.paths)
        self.publish_frame(frame(1, [(1.0, 0.0, 0.0), (1.05, 0.0, 0.0), (2.0, 0.0, 0.0)]))
        self.wait_until(lambda: any(message.header.stamp.sec == 11 for message in self.clouds))
        cloud = next(message for message in self.clouds if message.header.stamp.sec == 11)
        registered = xyz(cloud)
        self.assertEqual(len(registered), 3)
        for actual, expected in zip(registered, [(10, 21, 0), (10, 21.05, 0), (10, 22, 0)]):
            for coordinate, reference in zip(actual, expected):
                self.assertAlmostEqual(coordinate, reference, places=5)
        self.assertEqual(len(xyz(self.maps[-1])), 2)
        self.publish_frame(frame(2, [(3.0, 0.0, 0.0)]))
        self.publish_frame(frame(3, [(3.05, 0.0, 0.0)]))
        self.assertEqual([pose.header.stamp.sec for pose in self.paths[-1].poses], [12, 13])
        self.assertEqual(sorted(round(point[1], 3) for point in xyz(self.maps[-1])), [22, 23])
        # 无效输入不消费序号；随后的同序号有效帧仍必须被接纳。
        invalid = frame(4, [(4.0, 0.0, 0.0)])
        invalid.cloud.header.frame_id = 'wrong_sensor'
        self.frames.publish(invalid)
        invalid = frame(4, [(4.0, 0.0, 0.0)])
        invalid.odometry.pose.pose.orientation.z = 0.0
        invalid.odometry.pose.pose.orientation.w = 0.0
        self.frames.publish(invalid)
        self.frames.publish(frame(100, [(50.0, 0.0, 0.0)], epoch=1))
        self.publish_frame(frame(4, [(4.0, 0.0, 0.0)], world='new_odom'))
        self.assertEqual(self.paths[-1].header.frame_id, 'new_odom')
        self.assertEqual(len(self.paths[-1].poses), 1)
        self.assertEqual(len(xyz(self.maps[-1])), 1)
        self.sessions.publish(FrontendSession(instance_id=11, epoch=3))
        self.wait_until(lambda: self.maps[-1].width == 0 and not self.paths[-1].poses)
        self.sessions.publish(FrontendSession(instance_id=11, epoch=2))
        self.publish_frame(frame(1, [(1.0, 0.0, 0.0)], epoch=3))
        self.assertEqual(len(self.paths[-1].poses), 1)


@launch_testing.post_shutdown_test()
class TestVisualizationExit(unittest.TestCase):
    def test_clean_exit(self, proc_info, process):
        launch_testing.asserts.assertExitCodes(proc_info, process=process)
