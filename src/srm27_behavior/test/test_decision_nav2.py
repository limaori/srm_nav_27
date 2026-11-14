#!/usr/bin/env python3
"""Actual Nav2/MINCO + fakeyaw + mux, with a planar kinematic robot and static arena.

Only localization, referee and physical motion are simulated. No serial/Gazebo
hardware nodes are started. Production navigation parameters and BT XML are reused.
"""
import math
from pathlib import Path
import time

import pytest
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from lifecycle_msgs.srv import GetState
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import yaml

from test_decision_ros import Arena, wait_for, init_test_ros

REPO = Path(__file__).resolve().parents[3]
NAV = REPO / "src/srm27_navigation/srm27_nav_bringup"


class Nav2Arena(Arena):
    def __init__(self, league):
        self.yaw = 0.
        self.gimbal_yaw = .6
        self.last_step = time.monotonic()
        self.pose_history = []
        self.tf_pub = None
        super().__init__(league, "nav2", navigation="nav2")
        self.frame = "odom"
        self.tf_pub = TransformBroadcaster(self.node)
        self.static_tf = StaticTransformBroadcaster(self.node)
        transform = TransformStamped()
        transform.header.frame_id = "map"
        transform.child_frame_id = "odom"
        transform.transform.rotation.w = 1.
        self.static_tf.sendTransform(transform)
        self.map_pub = self.node.create_publisher(
            OccupancyGrid, "map", QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        grid = OccupancyGrid()
        grid.header.frame_id = "map"
        grid.info.resolution = .1
        grid.info.width = grid.info.height = 200
        grid.info.origin.position.x = grid.info.origin.position.y = -10.
        grid.info.origin.orientation.w = 1.
        grid.data = [0] * 40000
        for iy in range(200):
            for ix in range(200):
                x, y = ix * .1 - 10., iy * .1 - 10.
                if ix in (0, 199) or iy in (0, 199) or (1.6 <= x <= 2.4 and -2.6 <= y <= -.6):
                    grid.data[iy * 200 + ix] = 100
        self.map_pub.publish(grid)
        self.start_navigation()

    def publish(self):
        now = time.monotonic()
        dt = min(now - self.last_step, .1)
        self.last_step = now
        command = self.outputs[-1] if self.outputs else None
        vx, vy, wz = (command.linear.x, command.linear.y, command.angular.z) if command else (0., 0., 0.)
        self.x += (math.cos(self.gimbal_yaw) * vx - math.sin(self.gimbal_yaw) * vy) * dt
        self.y += (math.sin(self.gimbal_yaw) * vx + math.cos(self.gimbal_yaw) * vy) * dt
        self.yaw += wz * dt
        self.pose_history.append((self.x, self.y, self.yaw))
        if self.tf_pub:
            transform = TransformStamped()
            transform.header.stamp = self.node.get_clock().now().to_msg()
            transform.header.frame_id = "odom"
            transform.child_frame_id = "base_link"
            transform.transform.translation.x = self.x
            transform.transform.translation.y = self.y
            transform.transform.rotation.z = math.sin(self.yaw / 2.)
            transform.transform.rotation.w = math.cos(self.yaw / 2.)
            gimbal = TransformStamped()
            gimbal.header = transform.header
            gimbal.child_frame_id = "gimbal_yaw"
            gimbal.transform.translation = transform.transform.translation
            gimbal.transform.rotation.z = math.sin(self.gimbal_yaw / 2.)
            gimbal.transform.rotation.w = math.cos(self.gimbal_yaw / 2.)
            self.tf_pub.sendTransform([transform, gimbal])
        # Same gimbal odometry contract as the production launch; chassis yaw is separate.
        from nav_msgs.msg import Odometry
        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "gimbal_yaw"
        odom.header.stamp = self.node.get_clock().now().to_msg()
        odom.pose.pose.position.x, odom.pose.pose.position.y = self.x, self.y
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = math.sin(self.gimbal_yaw / 2.), math.cos(self.gimbal_yaw / 2.)
        odom.twist.twist.linear.x, odom.twist.twist.linear.y = vx, vy
        # The stabilized gimbal stays fixed while the chassis spins.
        odom.twist.twist.angular.z = 0.
        self.pubs["pose"].publish(odom)
        self.publish_referee()

    def close(self):
        super().close()
        import csv
        with (self.out / "trajectory.csv").open("w") as output:
            writer = csv.writer(output)
            writer.writerow(["x_m", "y_m", "yaw_rad"])
            writer.writerows((round(x, 4), round(y, 4), round(yaw, 4))
                            for x, y, yaw in self.pose_history)

    def start_navigation(self):
        source = yaml.safe_load((NAV / "config/simulation/nav2_params_srm_fakeyaw.yaml").read_text())
        names = ["controller_server", "planner_server", "smoother_server", "behavior_server", "bt_navigator", "fake_vel_transform"]
        config = {name: source[name] for name in names}
        for name in names:
            config[name]["ros__parameters"]["use_sim_time"] = False
        for name in ("local_costmap", "global_costmap"):
            params = source[name][name]["ros__parameters"]
            params.update(use_sim_time=False, plugins=["static_layer", "inflation_layer"],
                          update_frequency=20., publish_frequency=10.)
            params["static_layer"] = {"plugin": "nav2_costmap_2d::StaticLayer", "map_subscribe_transient_local": True}
            params["inflation_layer"] = {"plugin": "nav2_costmap_2d::InflationLayer", "inflation_radius": .6, "cost_scaling_factor": 5.}
            config[name] = {name: {"ros__parameters": params}}
        params = config["controller_server"]["ros__parameters"]
        params["general_goal_checker"]["plugin"] = "srm27_nav_plugins::OmniStoppedGoalChecker"
        params["FollowPath"]["yaw_policy"]["mode"] = "xy_only"
        params["FollowPath"]["spin_request"]["topic"] = "rotation_cmd"
        limits = yaml.safe_load((NAV / "config/simulation/speed_limits.yaml").read_text())
        params["FollowPath"]["limits"] = limits["controller_server@minco"]["ros__parameters"]["FollowPath"]["limits"]
        # The physical plant runs at 20 Hz; keep the production state freshness budget consistent.
        params["FollowPath"]["state_timeout"] = .2
        bt_params = config["bt_navigator"]["ros__parameters"]
        for key in ("default_nav_to_pose_bt_xml", "default_nav_through_poses_bt_xml"):
            bt_params[key] = bt_params[key].replace("$(find-pkg-share srm27_nav_bringup)", str(NAV))
        config_file = self.out / "nav2-params.yaml"
        config_file.write_text(yaml.safe_dump(config))
        for name, package in [("controller_server", "nav2_controller"), ("planner_server", "nav2_planner"),
                              ("smoother_server", "nav2_smoother"), ("behavior_server", "nav2_behaviors"),
                              ("bt_navigator", "nav2_bt_navigator")]:
            command = ["ros2", "run", package, name, "--ros-args", "--params-file", str(config_file)]
            if name in ("controller_server", "behavior_server"):
                command += ["-r", "cmd_vel:=cmd_vel_nav_fake"]
            self.spawn(name, command)
        self.spawn("fakeyaw", ["ros2", "run", "fake_vel_transform", "fake_vel_transform_node", "--ros-args",
                               "--params-file", str(config_file), "-p", "output_cmd_vel_topic:=cmd_vel_test_nav",
                               "-p", "chassis_cmd_vel_topic:=cmd_vel_test_output", "-p", "actuation_delay:=0.0"])
        self.spawn("lifecycle", ["ros2", "run", "nav2_lifecycle_manager", "lifecycle_manager", "--ros-args",
                                 "-r", "__node:=decision_test_lifecycle", "-p", "autostart:=true",
                                 "-p", "bond_timeout:=0.0", "-p",
                                 "node_names:=[controller_server, planner_server, smoother_server, behavior_server, bt_navigator]"])


@pytest.mark.parametrize("league", ["ul", "uc"])
def test_real_navigation_closed_loop(league):
    init_test_ros()
    a = Nav2Arena.__new__(Nav2Arena)
    try:
        a.__init__(league)
        client = a.node.create_client(GetState,
                                      "bt_navigator/get_state")
        assert client.wait_for_service(timeout_sec=15)
        wait_for(lambda: a.future(client.call_async(GetState.Request())).current_state.id == 3,
                 "real Nav2 failed to activate", timeout=30)
        if league == "uc":
            a.field.battlefield_valid = True
            a.field.outpost_control = [0, 2]
            a.field.base_control = [0, 2]
        a.game.game_progress = 4
        # Nav2 uses 0.5m XY tolerance around a grid-snapped endpoint (0.1m cells).
        arrival_bound = .6
        if league == "ul":
            target = (4.5, -3.2)
            wait_for(lambda: math.hypot(a.x-target[0], a.y-target[1]) < arrival_bound,
                     "MINCO did not reach control point", timeout=65)
            wait_for(lambda: a.outputs and abs(a.outputs[-1].angular.z - 7.) < .1,
                     "actual arrival spin did not reach mux", timeout=8)
            start = (a.x, a.y, a.yaw)
            time.sleep(.5)
            assert math.hypot(a.x-start[0], a.y-start[1]) < .08
            assert abs(a.yaw-start[2]) > 2.
            a.check("nav2_minco_arrival_and_stationary_spin")
        else:
            wait_for(lambda: math.hypot(a.x-5., a.y-1.2) < arrival_bound,
                     "MINCO did not reach enemy outpost control point", timeout=65)
            a.check("nav2_capture_enemy_outpost")
            a.field.outpost_control = [0, 0]
            wait_for(lambda: a.reports[-1]["mode"] == "hold_outposts" and
                     abs(a.outputs[-1].angular.z-7.) < .1, "real control hold failed", timeout=10)
            a.check("nav2_hold_two_outposts_from_real_feedback")
            a.field.outpost_control = [1, 0]
            wait_for(lambda: a.reports[-1]["mode"] == "restore_outpost" and
                     math.hypot(a.x-3.5, a.y) < arrival_bound,
                     "MINCO did not recover own outpost", timeout=35)
            a.check("nav2_restore_own_outpost_preempts_hold")
            a.ally.ally_base_hp = 1000
            wait_for(lambda: math.hypot(a.x-.8, a.y) < arrival_bound,
                     "real UC base defense failed", timeout=35)
            a.check("nav2_route_to_base_defense_preemption")
        a.status.current_hp = 100
        wait_for(lambda: math.hypot(a.x, a.y) < arrival_bound,
                 "real supply navigation failed", timeout=50)
        a.rfid.friendly_supply_zone_non_overlap = 1
        a.check("nav2_supply_return")
        a.game.game_progress = 5
        wait_for(lambda: a.outputs and abs(a.outputs[-1].linear.x) < 1e-6 and
                 abs(a.outputs[-1].angular.z) < 1e-6, "real match-end stop failed")
        a.check("nav2_match_end_stop")
        # Reject collisions rather than merely asserting that a goal was published.
        # Distance to occupied cell edges, not to cell centers.
        clearance = min(math.hypot(max(1.6-x, 0., x-2.5), max(-2.6-y, 0., y+.5))
                        for x, y, _ in a.pose_history)
        assert clearance >= .33, f"robot footprint intersects static obstacle: {clearance:.3f}m"
        a.check("static_obstacle_clearance")
        a.alive()
    finally:
        try:
            if hasattr(a, "executor"):
                a.close()
        finally:
            rclpy.shutdown()
