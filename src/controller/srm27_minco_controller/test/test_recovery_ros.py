"""Real Nav2/MINCO/fakeyaw/mux recovery scenarios with an isolated external plant."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy
from action_msgs.msg import GoalStatus
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import PointCloud2, Imu
from rclpy.parameter import Parameter
from xml.etree import ElementTree as ET
from nav2_msgs.action import BackUp, FollowPath, NavigateToPose, NavigateThroughPoses
from lifecycle_msgs.srv import GetState
from nav2_msgs.srv import ManageLifecycleNodes
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
import yaml

REPO = Path(__file__).resolve().parents[4]
NAV = REPO / 'src/bringup/srm27_nav_bringup'
OUT = Path(os.environ.get('SRM_RECOVERY_OUT', '/out/results'))


class Rig:
    def __init__(self, name, yaw=0.0, wall=None, gazebo=False, map_service="global_costmap/get_costmap"):
        self.out = OUT / name
        self.out.mkdir(parents=True, exist_ok=True)
        self.node = rclpy.create_node('recovery_fixture')
        self.node.set_parameters([Parameter('use_sim_time', value=gazebo)])
        self.child_env = {}
        self.ground_truth = None
        self.wall = wall
        self.spin_target = None
        self.requested_yaw = yaw
        self.map_service = map_service
        self.sensor_counts = {'cloud': 0, 'imu': 0}
        self.processes, self.files, self.handles = [], [], []
        self.x = self.y = 0.0
        self.yaw = yaw
        self.gazebo = gazebo
        self.moving = True
        self.send_state = True
        self.last = time.monotonic()
        self.cmd = Twist()
        self.commands, self.samples, self.diag = [], [], []
        self.static = StaticTransformBroadcaster(self.node)
        static = TransformStamped()
        static.header.frame_id, static.child_frame_id = 'map', 'odom'
        static.transform.rotation.w = 1.0
        self.static.sendTransform(static)
        self.tf = TransformBroadcaster(self.node) if not gazebo else None
        self.odom_pub = self.node.create_publisher(Odometry, 'odometry', 10) if not gazebo else None
        self.gate_pub = self.node.create_publisher(Odometry, 'chassis_odometry_gt', 10) if gazebo else None
        self.spin_pub = self.node.create_publisher(Twist, 'rotation_cmd', 10)
        self.map_pub = self.node.create_publisher(
            OccupancyGrid, 'map', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.node.create_subscription(Twist, 'cmd_vel_sim', self.receive_command, 100)
        self.node.create_subscription(DiagnosticArray, 'FollowPath/diagnostics', self.receive_diag, 50)
        self.grid = OccupancyGrid()
        self.grid.header.frame_id = 'map'
        self.grid.info.resolution = .05
        self.grid.info.width = self.grid.info.height = 200
        self.grid.info.origin.position.x = self.grid.info.origin.position.y = -5.0
        self.grid.info.origin.orientation.w = 1.0
        cells = np.zeros((200, 200), dtype=np.int8)
        cells[[0, -1], :] = 100
        cells[:, [0, -1]] = 100
        if wall == 'near':
            cells[20:180, 91:93] = 100  # nearest edge -0.35 m; physical robot remains clear
        elif wall == 'near_y':
            cells[91:93, 20:180] = 100
        elif wall == 'blocked':
            cells[:, :] = 100
        elif wall == 'obstacle':
            cells[90:110, 120:130] = 100
        elif wall == 'corridor':
            cells[90, :] = cells[109, :] = 100  # 0.9m clear corridor
        self.grid.data = cells.reshape(-1).tolist()
        self.map_pub.publish(self.grid)
        self.timer = self.node.create_timer(.02, self.step)
        if gazebo:
            self.start_gazebo(wall)
        self.start_navigation()
        self.backup = ActionClient(self.node, BackUp, 'backup')
        self.follow = ActionClient(self.node, FollowPath, 'follow_path')
        self.navigate = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')
        self.through = ActionClient(self.node, NavigateThroughPoses, 'navigate_through_poses')
        assert self.wait(lambda: all(c.server_is_ready() for c in (self.backup, self.follow, self.navigate, self.through)), 30), 'Nav2 did not activate'
        for name in ('controller_server', 'planner_server', 'smoother_server', 'behavior_server', 'bt_navigator'):
            client = self.node.create_client(GetState, f'/{name}/get_state')
            assert self.wait(client.service_is_ready, 10)
            deadline = time.monotonic() + 15
            while self.result(client.call_async(GetState.Request())).current_state.id != 3:
                assert time.monotonic() < deadline, f'{name} did not activate'
                self.wait(lambda: False, .1)
            self.node.destroy_client(client)
        self.wait(lambda: False, .3)
        if gazebo and abs(self.requested_yaw) > .01:
            self.spin_target = self.requested_yaw
            assert self.wait(lambda: abs(self.yaw - self.spin_target) < .03 and abs(self.cmd.angular.z) < .05, 8), 'Gazebo rotation did not converge'
            self.spin_target = None
            self.wait(lambda: False, .3)

    def spawn(self, name, command, env=None):
        log = (self.out / f'{name}.log').open('w')
        self.files.append(log)
        child_env = os.environ.copy()
        child_env.update(self.child_env)
        if env:
            child_env.update(env)
        proc = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                start_new_session=True, env=child_env)
        self.processes.append(proc)
        return proc

    def receive_command(self, msg):
        self.cmd = msg
        self.commands.append((time.monotonic(), msg.linear.x, msg.linear.y))

    def receive_diag(self, msg):
        for status in msg.status:
            self.diag.append({v.key: v.value for v in status.values})

    def step(self):
        now = time.monotonic()
        dt, self.last = min(.05, now - self.last), now
        if self.moving and not self.gazebo:
            self.x += (math.cos(self.yaw) * self.cmd.linear.x - math.sin(self.yaw) * self.cmd.linear.y) * dt
            self.y += (math.sin(self.yaw) * self.cmd.linear.x + math.cos(self.yaw) * self.cmd.linear.y) * dt
        self.samples.append([self.x, self.y])
        spin = Twist()
        if self.spin_target is not None:
            error = math.atan2(math.sin(self.spin_target - self.yaw), math.cos(self.spin_target - self.yaw))
            spin.angular.z = max(-1.0, min(1.0, 2 * error)) if abs(error) > .01 else 0.0
        self.spin_pub.publish(spin)
        if self.gazebo:
            return  # Production ground-truth adapter owns odometry and TF.
        if not self.send_state:
            return
        stamp = self.node.get_clock().now().to_msg()
        transform = TransformStamped()
        transform.header.stamp, transform.header.frame_id = stamp, 'odom'
        transform.child_frame_id = 'gimbal_yaw'
        transform.transform.translation.x, transform.transform.translation.y = self.x, self.y
        transform.transform.rotation.z, transform.transform.rotation.w = math.sin(self.yaw / 2), math.cos(self.yaw / 2)
        chassis = TransformStamped()
        chassis.header = transform.header
        chassis.child_frame_id = 'base_link'
        chassis.transform = transform.transform
        self.tf.sendTransform([transform, chassis])
        msg = Odometry()
        msg.header.stamp, msg.header.frame_id = stamp, 'odom'
        msg.child_frame_id = 'gimbal_yaw'
        msg.pose.pose.position.x, msg.pose.pose.position.y = self.x, self.y
        msg.pose.pose.orientation = transform.transform.rotation
        msg.twist.twist = self.cmd
        self.odom_pub.publish(msg)

    def wait(self, predicate, timeout=10):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=.02)
            if predicate():
                return True
        return bool(predicate())

    def result(self, future, timeout=10):
        assert self.wait(future.done, timeout), 'ROS request timed out'
        return future.result()

    def send(self, client, goal):
        handle = self.result(client.send_goal_async(goal))
        assert handle.accepted
        self.handles.append(handle)
        return handle

    def pose(self, x, y):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.node.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = float(x), float(y)
        pose.pose.orientation.w = 1.0
        return pose

    def backup_goal(self, distance=.6, seconds=5.0):
        goal = BackUp.Goal()
        goal.target.x, goal.speed = -distance, .2
        goal.time_allowance.sec = int(seconds)
        goal.time_allowance.nanosec = int((seconds - int(seconds)) * 1e9)
        return self.send(self.backup, goal)

    def start_navigation(self):
        source = yaml.safe_load((NAV / 'config/simulation/nav2_params_srm_fakeyaw.yaml').read_text())
        names = ['controller_server', 'planner_server', 'smoother_server', 'behavior_server', 'bt_navigator', 'fake_vel_transform']
        config = {name: source[name] for name in names}
        for name in names:
            config[name]['ros__parameters']['use_sim_time'] = self.gazebo
        for name in ('local_costmap', 'global_costmap'):
            params = source[name][name]['ros__parameters']
            params.update(use_sim_time=self.gazebo, plugins=['static_layer', 'inflation_layer'],
                          update_frequency=20., publish_frequency=10., robot_radius=.30)
            params['static_layer'] = {'plugin': 'nav2_costmap_2d::StaticLayer', 'map_subscribe_transient_local': True}
            params['inflation_layer'] = {'plugin': 'nav2_costmap_2d::InflationLayer', 'inflation_radius': .6, 'cost_scaling_factor': 5.}
            config[name] = {name: {'ros__parameters': params}}
        ctrl = config['controller_server']['ros__parameters']
        follow = ctrl['FollowPath']
        follow['limits'] = dict(max_linear_speed=.5, max_linear_accel=1.0, max_angular_speed=1., max_angular_accel=2.)
        follow['state_timeout'], follow['build_grace_period'] = .3, 1.0
        follow['safety'].update(robot_radius=.28, clearance_margin=.02)
        follow['recovery'].update(fail_threshold=2, stuck_clearance=.4, escaped_clearance=.5, escape_speed=.2, window_sec=3.0)
        follow['yaw_policy']['mode'] = 'xy_only'
        ctrl['general_goal_checker'].update(xy_goal_tolerance=.2, trans_stopped_velocity=.10)
        b = config['behavior_server']['ros__parameters']
        b.update(costmap_topic='global_costmap/costmap_raw', footprint_topic='global_costmap/published_footprint', service_name=self.map_service)
        bt = config['bt_navigator']['ros__parameters']
        for key in ('default_nav_to_pose_bt_xml', 'default_nav_through_poses_bt_xml'):
            bt[key] = bt[key].replace('$(find-pkg-share srm27_nav_bringup)', str(NAV))
        file = self.out / 'params.yaml'
        file.write_text(yaml.safe_dump(config))
        for name, package in [('controller_server', 'nav2_controller'), ('planner_server', 'nav2_planner'),
                              ('smoother_server', 'nav2_smoother'), ('behavior_server', 'nav2_behaviors'),
                              ('bt_navigator', 'nav2_bt_navigator')]:
            cmd = ['ros2', 'run', package, name, '--ros-args', '--params-file', str(file)]
            if name in ('controller_server', 'behavior_server'):
                cmd += ['-r', 'cmd_vel:=cmd_vel_nav_fake']
            self.spawn(name, cmd)
        self.spawn('fakeyaw', ['ros2', 'run', 'fake_vel_transform', 'fake_vel_transform_node', '--ros-args', '--params-file', str(file), '-p', 'actuation_delay:=0.0'])
        self.spawn('mux', ['ros2', 'run', 'srm27_chassis_control', 'cmd_mux', '--ros-args',
                           '-p', 'nav_topic:=cmd_vel_nav_body', '-p', 'output_topic:=cmd_vel_sim',
                           '-p', 'spin_topic:=rotation_cmd', '-p', 'decision_required:=false',
                           '-p', f'use_sim_time:={str(self.gazebo).lower()}', '-p', 'v_max:=0.5', '-p', 'vx_max:=0.5', '-p', 'vy_max:=0.5'])
        self.spawn('lifecycle', ['ros2', 'run', 'nav2_lifecycle_manager', 'lifecycle_manager', '--ros-args',
                                '-p', f'use_sim_time:={str(self.gazebo).lower()}', '-p', 'autostart:=false', '-p', 'bond_timeout:=0.0', '-p',
                                'node_names:=[controller_server, planner_server, smoother_server, behavior_server, bt_navigator]'])
        manager = self.node.create_client(ManageLifecycleNodes, '/lifecycle_manager/manage_nodes')
        assert self.wait(manager.service_is_ready, 10)
        # Let the manager's lifecycle clients discover their response readers before startup.
        self.wait(lambda: False, 1.0)
        request = ManageLifecycleNodes.Request()
        request.command = ManageLifecycleNodes.Request.STARTUP
        assert self.result(manager.call_async(request), 30).success
        self.node.destroy_client(manager)


    def start_gazebo(self, wall):
        from srm27_robot_description.model_builder import build_sdf, build_urdf, load_geometry
        self.child_env.update(IGN_PARTITION=f'recovery_{os.getpid()}', LIBGL_ALWAYS_SOFTWARE='1')
        geometry = load_geometry()
        geometry['robot']['name'] = 'recovery_robot'
        model = ET.fromstring(build_sdf(geometry, command_in_gimbal_frame=True, odom_link='gimbal_yaw')).find('model')
        model.insert(0, ET.fromstring('<pose>0 0 0.28 0 0 0</pose>'))
        world = ET.parse(REPO / 'src/simulation/srm27_gazebo_simulator/worlds/srm_empty.sdf')
        root = world.getroot().find('world')
        root.append(model)
        boxes = []
        if wall == 'near':
            boxes = [(-.4, 0, .1, 8)]
        elif wall == 'obstacle':
            boxes = [(1.25, 0, .5, 1)]
        elif wall == 'corridor':
            boxes = [(0, -.475, 10, .05), (0, .475, 10, .05)]
        for i, (x, y, sx, sy) in enumerate(boxes):
            root.append(ET.fromstring(f'<model name="obstacle_{i}"><static>true</static><pose>{x} {y} .4 0 0 0</pose><link name="body"><collision name="collision"><geometry><box><size>{sx} {sy} .8</size></box></geometry></collision><visual name="visual"><geometry><box><size>{sx} {sy} .8</size></box></geometry></visual></link></model>'))
        world_file = self.out / 'world.sdf'
        world.write(world_file, encoding='unicode')
        self.spawn('gazebo', ['ign', 'gazebo', '-s', '-r', '--headless-rendering', '-v', '2', str(world_file)])
        bridge = yaml.safe_load((REPO / 'src/simulation/srm27_gazebo_simulator/config/ros_gz_bridge.yaml').read_text().replace('<robot_name>', 'recovery_robot'))
        bridge = [item for item in bridge if item['ros_topic_name'] == '/clock' or any(item['ros_topic_name'].endswith(t) for t in ('chassis_odometry_gt', 'joint_states', 'livox/lidar', 'livox/imu'))]
        for item in bridge:
            item['ros_topic_name'] = item['ros_topic_name'].replace('/recovery_robot/', '/')
            if item['ros_topic_name'] == '/chassis_odometry_gt':
                item['ros_topic_name'] = '/chassis_odometry_gt_raw'
        bridge_file = self.out / 'bridge.yaml'
        bridge_file.write_text(yaml.safe_dump(bridge))
        self.spawn('bridge', ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge', '--ros-args', '-p', f'config_file:={bridge_file}'])
        self.node.create_subscription(Odometry, 'chassis_odometry_gt_raw', self.receive_ground_truth, 20)
        self.node.create_subscription(PointCloud2, 'livox/lidar', lambda m: self.count_sensor('cloud', m.width * m.height), 5)
        self.node.create_subscription(Imu, 'livox/imu', lambda m: self.count_sensor('imu', 1), 5)
        self.spawn('ground_truth_adapter', ['ros2', 'run', 'srm27_nav_tools', 'simulation_ground_truth_odometry.py', '--ros-args', '-p', 'base_frame:=gimbal_yaw', '-p', 'use_sim_time:=false'])
        rsp_file = self.out / 'robot-description.yaml'
        rsp_file.write_text(yaml.safe_dump({'robot_state_publisher': {'ros__parameters': {
            'use_sim_time': True, 'robot_description': build_urdf(geometry, gimbal_is_root=True, gimbal_axis_sign=-1.0)}}}))
        self.spawn('robot_state_publisher', ['ros2', 'run', 'robot_state_publisher', 'robot_state_publisher', '--ros-args', '--params-file', str(rsp_file)])
        self.spawn('velocity_adapter', ['ros2', 'run', 'srm27_gazebo_simulator', 'srm_velocity_adapter', '--ros-args', '-p', 'robot_name:=recovery_robot', '-p', 'use_sim_time:=true'])
        assert self.wait(lambda: self.ground_truth is not None and all(self.sensor_counts.values()), 35), 'Gazebo physics/sensors did not produce data'

    def count_sensor(self, key, count):
        self.sensor_counts[key] += int(count > 0)

    def receive_ground_truth(self, msg):
        self.ground_truth = msg
        if self.send_state:
            self.gate_pub.publish(msg)
        self.x, self.y = msg.pose.pose.position.x, msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def minimum_physical_clearance(self):
        bounds = {'near': [(-.45, -.35, -4, 4)], 'near_y': [(-4, 4, -.45, -.35)],
                  'obstacle': [(1, 1.5, -.5, .5)],
                  'corridor': [(-5, 5, -.5, -.45), (-5, 5, .45, .5)]}.get(self.wall, [])
        if not bounds:
            return None
        return min(math.hypot(max(x0-x, 0, x-x1), max(y0-y, 0, y-y1))
                   for x, y in self.samples for x0, x1, y0, y1 in bounds)

    def close(self):
        for handle in self.handles:
            try:
                self.result(handle.cancel_goal_async(), 1)
            except Exception:
                pass
        self.wait(lambda: False, .2)
        for proc in reversed(self.processes):
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
        for proc in self.processes:
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait(timeout=2)
        for log in self.files:
            log.close()
        (self.out / 'trajectory.json').write_text(json.dumps(self.samples))
        (self.out / 'diagnostics.json').write_text(json.dumps(self.diag))
        (self.out / 'sensors.json').write_text(json.dumps(self.sensor_counts))
        (self.out / 'clearance.json').write_text(json.dumps({'minimum_center_clearance_m': self.minimum_physical_clearance()}))
        self.timer.cancel()
        for name in ('backup', 'follow', 'navigate', 'through'):
            client = getattr(self, name, None)
            if client:
                client.destroy()
        self.node.destroy_node()


@pytest.fixture
def rig(request):
    if not rclpy.ok():
        rclpy.init()
    options = getattr(request, 'param', {})
    instance = Rig.__new__(Rig)
    try:
        instance.__init__(request.node.name, **options)
        assert all(proc.poll() is None for proc in instance.processes), 'A navigation process exited'
        yield instance
    finally:
        instance.close()
        rclpy.shutdown()


@pytest.mark.parametrize('rig', [{'yaw': math.pi / 2, 'wall': 'near'}, {'yaw': -.7, 'wall': 'near'}], indirect=True)
def test_backup_direction_cancel_and_stop(rig):
    handle = rig.backup_goal()
    assert rig.wait(lambda: rig.x > .15, 4), 'backup did not move away from wall'
    assert abs(rig.y) < .05, 'global escape velocity rotated incorrectly'
    rig.result(handle.cancel_goal_async())
    result = rig.result(handle.get_result_async())
    assert result.status == GoalStatus.STATUS_CANCELED
    assert rig.wait(lambda: math.hypot(rig.cmd.linear.x, rig.cmd.linear.y) < 1e-5, 1)


@pytest.mark.parametrize('rig', [{'wall': 'blocked'}], indirect=True)
def test_no_direction_is_bounded_and_stationary(rig):
    before = time.monotonic()
    result = rig.result(rig.backup_goal().get_result_async(), 4)
    assert result.status == GoalStatus.STATUS_ABORTED
    assert time.monotonic() - before < 3
    assert all(math.hypot(x, y) < 1e-5 for _, x, y in rig.commands)


@pytest.mark.parametrize('rig', [{'wall': 'near_y'}], indirect=True)
def test_lateral_backup_rejects_a_new_obstacle(rig):
    handle = rig.backup_goal()
    assert rig.wait(lambda: rig.y > .05, 3)
    cells = np.array(rig.grid.data, dtype=np.int8).reshape(200, 200)
    cells[111:113, 96:104] = 100
    rig.grid.data = cells.reshape(-1).tolist()
    rig.map_pub.publish(rig.grid)
    result = rig.result(handle.get_result_async(), 3)
    assert result.status == GoalStatus.STATUS_ABORTED
    assert rig.y < .25, 'sideways footprint sweep missed the new obstacle'
    assert rig.wait(lambda: math.hypot(rig.cmd.linear.x, rig.cmd.linear.y) < 1e-5, 1)



def test_invalid_backup_goals_cannot_start_motion(rig):
    for speed, seconds in ((float('nan'), 5), (-.2, 5), (0.0, 5), (.2, 0)):
        goal = BackUp.Goal()
        goal.target.x, goal.speed, goal.time_allowance.sec = -.6, speed, seconds
        result = rig.result(rig.send(rig.backup, goal).get_result_async(), 2)
        assert result.status == GoalStatus.STATUS_ABORTED
    assert all(math.hypot(x, y) < 1e-5 for _, x, y in rig.commands)


@pytest.mark.parametrize('rig', [{'map_service': 'missing_costmap/get_costmap'}], indirect=True)
def test_missing_map_service_has_bounded_failure(rig):
    before = time.monotonic()
    result = rig.result(rig.backup_goal().get_result_async(), 3)
    assert result.status == GoalStatus.STATUS_ABORTED
    assert time.monotonic() - before < 2.5
    assert all(math.hypot(x, y) < 1e-5 for _, x, y in rig.commands)


def test_backup_timeout_stops_a_stationary_plant(rig):
    rig.moving = False
    result = rig.result(rig.backup_goal(seconds=.5).get_result_async(), 3)
    assert result.status == GoalStatus.STATUS_ABORTED
    assert rig.wait(lambda: math.hypot(rig.cmd.linear.x, rig.cmd.linear.y) < 1e-5, 1)


@pytest.mark.parametrize('rig', [{'yaw': math.pi / 2, 'wall': 'near'}], indirect=True)
def test_controller_recovery_state_loss_and_new_goal(rig):
    goal = FollowPath.Goal()
    goal.controller_id = 'FollowPath'
    goal.path.header.frame_id = 'map'
    goal.path.poses = [rig.pose(0, 0), rig.pose(-1, 0)]
    handle = rig.send(rig.follow, goal)
    assert rig.wait(lambda: any(d.get('recovery_active') == 'true' for d in rig.diag), 8)
    assert rig.wait(lambda: rig.x > .08, 3)
    assert abs(rig.y) < .05
    rig.send_state = False
    assert rig.wait(lambda: math.hypot(rig.cmd.linear.x, rig.cmd.linear.y) < 1e-5, 1)
    result = rig.result(handle.get_result_async(), 5)
    assert result.status == GoalStatus.STATUS_ABORTED, 'input loss kept recovery alive'
    rig.send_state = True
    rig.wait(lambda: False, .5)
    nav = NavigateToPose.Goal()
    nav.pose = rig.pose(1.5, .3)
    result = rig.result(rig.send(rig.navigate, nav).get_result_async(), 25)
    assert result.status == GoalStatus.STATUS_SUCCEEDED


@pytest.mark.parametrize('rig', [{'wall': 'corridor'}, {'wall': 'obstacle'}], indirect=True)
def test_navigation_through_poses_and_final_stop(rig):
    goal = NavigateThroughPoses.Goal()
    goal.poses = [rig.pose(.5, 0), rig.pose(2.3, 0)]
    result = rig.result(rig.send(rig.through, goal).get_result_async(), 40)
    assert result.status == GoalStatus.STATUS_SUCCEEDED
    assert math.hypot(rig.x - 2.3, rig.y) < .3
    assert rig.wait(lambda: math.hypot(rig.cmd.linear.x, rig.cmd.linear.y) < 1e-5, 1)


@pytest.mark.parametrize('rig', [{'wall': 'near'}], indirect=True)
def test_bt_recovery_cancel_and_replacement(rig):
    goal = NavigateToPose.Goal()
    goal.pose = rig.pose(20, 20)  # Outside the map: global planning must enter BT recovery.
    handle = rig.send(rig.navigate, goal)
    assert rig.wait(lambda: rig.x > .05, 15), 'Nav2 BT did not execute its backup fallback'
    rig.result(handle.cancel_goal_async())
    assert rig.result(handle.get_result_async()).status == GoalStatus.STATUS_CANCELED
    assert rig.wait(lambda: math.hypot(rig.cmd.linear.x, rig.cmd.linear.y) < 1e-5, 1)
    goal.pose = rig.pose(1.5, .3)
    assert rig.result(rig.send(rig.navigate, goal).get_result_async(), 25).status == GoalStatus.STATUS_SUCCEEDED


@pytest.mark.skipif(os.environ.get('SRM_RECOVERY_GAZEBO') != '1', reason='Enable the Gazebo validation stage')
@pytest.mark.parametrize('rig', [{'yaw': math.pi / 2, 'wall': 'near', 'gazebo': True}], indirect=True)
def test_gazebo_recovery_state_loss_and_new_goal(rig):
    test_controller_recovery_state_loss_and_new_goal(rig)
    assert all(rig.sensor_counts.values())
    from srm27_robot_description.model_builder import load_geometry
    assert rig.minimum_physical_clearance() >= load_geometry()['chassis']['radius']


@pytest.mark.skipif(os.environ.get('SRM_RECOVERY_GAZEBO') != '1', reason='Enable the Gazebo validation stage')
@pytest.mark.parametrize('rig', [{'wall': 'obstacle', 'gazebo': True}, {'wall': 'corridor', 'gazebo': True}], indirect=True)
def test_gazebo_navigation_and_final_stop(rig):
    test_navigation_through_poses_and_final_stop(rig)
    assert all(rig.sensor_counts.values())
    from srm27_robot_description.model_builder import load_geometry
    assert rig.minimum_physical_clearance() >= load_geometry()['chassis']['radius']
