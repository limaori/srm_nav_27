#!/usr/bin/env python3
"""Production server/XML + real mux; only referee/localization/Nav2 are simulated.

Failure gaps: duplicate or overlapping goals, fast/rejected/aborted results,
RFID interruption, resurrection, missing/wrong/stale inputs, cancellation restart,
parameter rejection, UC outpost/base priority, and no UL posture/ammo resupply.
"""
import itertools
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import threading
import time

import pytest
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParametersAtomically
from ament_index_python.packages import get_package_share_directory
from btcpp_ros2_interfaces.action import ExecuteTree
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses
from nav_msgs.msg import Odometry
from rm_decision_interfaces.msg import (
    GameStatus, RobotStatus, RfidStatus, AllyRobotHP, SwitchPosition, Sefdefined, ProjectileAllowance, EventData,
)
from std_msgs.msg import String, UInt8, Bool
from std_srvs.srv import Trigger

DOMAINS = itertools.count(int(os.environ.get("ROS_DOMAIN_ID", "170")))


def init_test_ros():
    os.environ["ROS_DOMAIN_ID"] = str(100 + next(DOMAINS) % 100)
    rclpy.init()


OUT = Path(os.environ.get("DECISION_TEST_OUTPUT", ".cache/decision-validation/results"))


def wait_for(predicate, message, timeout=7):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.02)
    raise AssertionError(message)


class Arena:
    def __init__(self, league, scenario="match", navigation="fake", overrides=None):
        self.league = league
        self.out = OUT / f"{league}-{scenario}"
        self.events = []
        self.spins = []
        self.spin_requests = []
        self.outputs = []
        self.postures = []
        self.reports = []
        self.permits = []
        self.pose_stamp = None
        self.chassis_enabled = True
        self.processes = []
        self.logs = []
        self.sequence = 0
        self.reject = False
        self.rejections = 0
        self.accept_delay = 0.
        self.feedback_period = .03
        self.reject_cancel = False
        self.closing = False
        self.abort = False
        self.succeed = False
        self.suspended = set()
        self.game = GameStatus(game_type=4 if league == "ul" else 1,
                               game_progress=0, stage_remain_time=299 if league == "ul" else 419)
        self.status = RobotStatus(robot_id=7, status_valid=True, heat_valid=True, ammo_valid=True,
                                  current_hp=400, maximum_hp=400, projectile_allowance_17mm=750)
        self.rfid = RfidStatus()
        self.ally = AllyRobotHP(ally_outpost_hp=1500, ally_base_hp=5000)
        self.switch = SwitchPosition(switch_position=1)
        self.field = EventData()
        self.field.center_control_valid = True
        self.x, self.y, self.frame = -2., 0., "map"
        self.node = rclpy.create_node("decision_arena")
        self.group = ReentrantCallbackGroup()
        self.servers = []
        for kind, action, name in ([] if navigation != "fake" else [("point", NavigateToPose, "navigate_to_pose"),
                                   ("route", NavigateThroughPoses, "navigate_through_poses")]):
            self.servers.append(ActionServer(
                self.node, action, name,
                execute_callback=lambda h, k=kind, a=action: self.execute(h, k, a),
                goal_callback=self.accept_goal,
                cancel_callback=lambda h: CancelResponse.REJECT if self.reject_cancel else CancelResponse.ACCEPT,
                callback_group=self.group))
        self.pubs = {}
        for key, msg, topic in [
            ("game", GameStatus, "referee/game_status"),
            ("identity", Sefdefined, "test/referee_identity"),
            ("ammo", ProjectileAllowance, "referee/projectile_allowance"),
            ("field", EventData, "referee/event_data"),
            ("status", RobotStatus, "referee/robot_status"),
            ("rfid", RfidStatus, "referee/rfid_status"),
            ("ally", AllyRobotHP, "referee/ally_robot_hp"),
            ("switch", SwitchPosition, "referee/switch_position"),
            ("pose", Odometry, "odometry"),
        ]:
            self.pubs[key] = self.node.create_publisher(msg, topic, 10)
        self.subscriptions = [
            self.node.create_subscription(String, "decision/status", lambda m: self.reports.append(json.loads(m.data)), 10),
            self.node.create_subscription(Bool, "decision/motion_permit", lambda m: self.permits.append(m.data), 10),
            self.node.create_subscription(String, "behavior/spin_state", lambda m: self.spins.append(m.data), 10),
            self.node.create_subscription(Twist, "rotation_cmd", lambda m: self.spin_requests.append(m.angular.z), 10),
            self.node.create_subscription(Twist, "cmd_vel_test_output", lambda m: self.outputs.append(m), 10),
            self.node.create_subscription(UInt8, "sentry/desired_posture", lambda m: self.postures.append(m.data), 10),
        ]
        self.nav_pub = self.node.create_publisher(Twist, "cmd_vel_test_nav", 1)
        self.timer = self.node.create_timer(0.02, self.publish)
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        mux_limits = [] if navigation == "fake" else ["-p", "vx_max:=2.6", "-p", "vy_max:=2.6", "-p", "v_max:=2.6"]
        self.spawn("mux", ["ros2", "run", "srm27_chassis_control", "cmd_mux", "--ros-args",
                           "-p", "wz_max:=9.0", "-p", "nav_topic:=cmd_vel_test_nav", "-p", "output_topic:=cmd_vel_test_output"] + mux_limits)
        params_dir = Path(get_package_share_directory("srm27_behavior")) / "params"
        overrides = dict(overrides or {})
        if league == "ul":
            overrides.setdefault("waypoints_file", Path(__file__).resolve().with_name("ul_waypoints.yaml"))
        self.spawn("server", ["ros2", "run", "srm27_behavior", "srm27_behavior_server", "--ros-args",
                              "--params-file", str(params_dir / "sentry_behavior.yaml"), "-r", "__node:=srm27_behavior_server",
                              "-p", "use_sim_time:=false", "-p", "status_topic:=test/referee_identity",
                              "-p", "dry_run:=false", "-p", "waypoints_calibrated:=true"] +
                   [part for key, value in overrides.items() for part in ("-p", f"{key}:={value}")])
        self.client = ActionClient(self.node, ExecuteTree, "srm27_behavior")
        assert self.client.wait_for_server(timeout_sec=12), "decision server unavailable"
        self.tree = self.send_tree()
        self.checks = []

    def spawn(self, name, command):
        self.out.mkdir(parents=True, exist_ok=True)
        log = (self.out / f"{name}.log").open("w")
        self.logs.append(log)
        self.processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                                start_new_session=True))

    def publish_referee(self):
        for key, publisher in self.pubs.items():
            if key in self.suspended or key == "pose":
                continue
            if key == "identity":
                message = Sefdefined(robot_id=self.status.robot_id, current_hp=self.status.current_hp,
                                     maximum_hp=self.status.maximum_hp,
                                     power_management_chassis_output=int(self.chassis_enabled))
            elif key == "ammo":
                message = ProjectileAllowance(projectile_allowance_17mm=self.status.projectile_allowance_17mm)
            else:
                message = getattr(self, key)
            publisher.publish(message)

    def publish(self):
        self.publish_referee()
        if "pose" not in self.suspended:
            message = Odometry()
            message.header.frame_id = self.frame
            message.header.stamp = self.pose_stamp or self.node.get_clock().now().to_msg()
            message.pose.pose.position.x = self.x
            message.pose.pose.position.y = self.y
            message.pose.pose.orientation.w = 1.
            self.pubs["pose"].publish(message)

    def accept_goal(self, request):
        time.sleep(self.accept_delay)
        if self.reject:
            self.rejections += 1
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def execute(self, handle, kind, action):
        self.sequence += 1
        seq = self.sequence
        poses = [handle.request.pose] if kind == "point" else handle.request.poses
        self.events.append({"event": "accepted", "seq": seq, "kind": kind,
                            "xy": [[round(p.pose.position.x, 2), round(p.pose.position.y, 2)] for p in poses]})
        while rclpy.ok():
            if self.closing:
                handle.abort()
                return action.Result()
            if handle.is_cancel_requested:
                self.nav_pub.publish(Twist())
                handle.canceled()
                self.events.append({"event": "canceled", "seq": seq})
                return action.Result()
            if self.abort:
                self.nav_pub.publish(Twist())
                handle.abort()
                self.events.append({"event": "aborted", "seq": seq})
                return action.Result()
            if self.succeed:
                self.nav_pub.publish(Twist())
                handle.succeed()
                self.events.append({"event": "succeeded", "seq": seq})
                return action.Result()
            velocity = Twist()
            velocity.linear.x = 0.2
            velocity.angular.z = 0.4
            if "velocity" not in self.suspended:
                self.nav_pub.publish(velocity)
            feedback = action.Feedback()
            feedback.distance_remaining = 1.
            if kind == "route":
                feedback.number_of_poses_remaining = len(poses)
            handle.publish_feedback(feedback)
            time.sleep(self.feedback_period)
        return action.Result()

    def future(self, future):
        wait_for(future.done, "ROS request timed out")
        return future.result()

    def send_tree(self):
        return self.future(self.client.send_goal_async(ExecuteTree.Goal(target_tree=f"rm{self.league}_2027")))

    def accepted(self):
        return [e for e in self.events if e["event"] == "accepted"]

    def goal(self, x, y, after=0):
        wait_for(lambda: len(self.accepted()) > after and
                 self.accepted()[-1]["xy"][-1] == [x, y], f"missing destination {x, y}")
        self.alive()

    def alive(self):
        assert all(p.poll() is None for p in self.processes), "server/mux exited; inspect logs"

    def check(self, name):
        self.alive()
        self.checks.append(name)

    def stopped(self, after, stationary_spin=False):
        wait_for(lambda: any(e["event"] == "canceled" for e in self.events[after:]), "goal not canceled")
        wait_for(lambda: self.outputs and self.outputs[-1].linear.x == 0 and
                 (stationary_spin or self.outputs[-1].angular.z == 0), "mux did not stop")

    def set_parameters(self, values):
        client = self.node.create_client(SetParametersAtomically, "srm27_behavior_server/set_parameters_atomically")
        assert client.wait_for_service(timeout_sec=5)
        request = SetParametersAtomically.Request(parameters=[
            Parameter(name=k, value=ParameterValue(
                type=ParameterType.PARAMETER_INTEGER if isinstance(v, int) else ParameterType.PARAMETER_DOUBLE,
                integer_value=v if isinstance(v, int) else 0,
                double_value=v if isinstance(v, float) else 0.))
            for k, v in values.items()])
        result = self.future(client.call_async(request)).result
        self.node.destroy_client(client)
        return result.successful

    def manual_stop(self, stopped=True):
        client = self.node.create_client(Trigger, "srm_cmd_mux/stop_all" if stopped else "srm_cmd_mux/resume_all")
        assert client.wait_for_service(timeout_sec=3)
        assert self.future(client.call_async(Trigger.Request())).success
        self.node.destroy_client(client)

    def close(self):
        self.reject_cancel = False
        self.closing = True
        time.sleep(.1)
        if hasattr(self, "tree") and self.tree.accepted and self.processes[1].poll() is None:
            try:
                self.future(self.tree.cancel_goal_async())
                self.future(self.tree.get_result_async())
            except Exception:
                pass
        bad_exits = []
        for process in reversed(self.processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=6)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
                bad_exits.append("shutdown timed out")
            if process.returncode not in (0, -signal.SIGINT, 130):
                bad_exits.append(process.returncode)
        self.closing = True
        self.executor.shutdown(timeout_sec=3)
        self.thread.join(timeout=3)
        self.node.destroy_node()
        for log in self.logs:
            log.close()
        (self.out / "manifest.json").write_text(json.dumps(
            {"league": self.league, "checks": self.checks, "events": self.events},
            indent=2, sort_keys=True) + "\n")
        assert not bad_exits, f"process shutdown failed: {bad_exits}"


@pytest.fixture(params=["ul", "uc"])
def arena(request, tmp_path):
    init_test_ros()
    a = Arena.__new__(Arena)
    try:
        overrides = {}
        if request.param == "ul":
            # Isolated tunnel geometry exercises UL limits without field coordinates.
            semantic_map = tmp_path / "ul_tunnel.yaml"
            semantic_map.write_text(
                "frame_id: map\nzones:\n  - code: test_tunnel\n    type: tunnel\n"
                "    max_spin: 0.0\n    polygon: [[100, 100], [102, 100], [102, 102], [100, 102]]\n")
            overrides["ul_semantic_map_file"] = str(semantic_map)
        a.__init__(request.param, request.node.originalname, overrides=overrides)
        yield a
    finally:
        try:
            if hasattr(a, "executor"):
                a.close()
        finally:
            rclpy.shutdown()


def test_match_lifecycle_and_strategies(arena):
    a = arena
    time.sleep(0.6)
    assert not a.accepted(), "pregame sent navigation"
    a.check("pregame_stop")
    a.game.game_progress = 4
    if a.league == "ul":
        target = (4.5, -3.2)
        a.goal(*target)
        time.sleep(0.5)
        assert len(a.accepted()) == 1, "same target was sent repeatedly"
        a.check("single_goal_no_repeat")
        a.x, a.y = target
        a.succeed = True
        wait_for(lambda: any(e["event"] == "succeeded" for e in a.events), "fast result missing")
        wait_for(lambda: a.spins and "ARRIVED_FAST" in a.spins[-1], "arrival spin missing")
        wait_for(lambda: a.outputs and abs(a.outputs[-1].angular.z - 7.) < .01,
                 "arrival request did not reach real mux output")
        time.sleep(0.3)
        assert len(a.accepted()) == 1
        os.killpg(a.processes[1].pid, signal.SIGSTOP)
        try:
            wait_for(lambda: a.outputs and a.outputs[-1].angular.z == 0.,
                     "stale hold-spin lease survived decision pause", timeout=2)
        finally:
            os.killpg(a.processes[1].pid, signal.SIGCONT)
        wait_for(lambda: a.outputs and abs(a.outputs[-1].angular.z - 7.) < .01,
                 "hold-spin lease did not resume")
        a.check("stationary_spin_lease_timeout")
        a.succeed = False
        a.x -= 1.
        a.goal(*target, after=1)
        wait_for(lambda: "MOVING_SLOW" in a.spins[-1], "drift incorrectly remains arrived")
        a.check("success_hold_and_drift_recovery")
    else:
        target = (3.5, 0.)
        a.goal(*target)
        a.ally.ally_base_hp = 1500
        a.goal(0.8, 0., after=1)
        a.check("base_defense_preempts_patrol")
        a.ally.ally_base_hp = 5000
        a.goal(*target, after=2)
        a.ally.ally_outpost_hp = 0
        a.switch.switch_position = 0
        a.goal(2.4, -0.8, after=3)
        a.check("lost_outpost_blocks_push")
        a.ally.ally_outpost_hp = 1500
        a.goal(6., -1.2, after=4)
        a.check("explicit_outpost_push")
        a.switch.switch_position = 1
        a.goal(*target, after=5)
        count = len(a.accepted())
        a.status.projectile_allowance_17mm = 0
        a.goal(2.4, -0.8, after=count)
        a.check("uc_low_ammo_fortress_without_unavailable_purchase")
        count = len(a.accepted())
        a.status.projectile_allowance_17mm = 750
        a.goal(*target, after=count)
    wait_for(lambda: a.outputs and a.outputs[-1].linear.x > 0., "real mux did not relay movement")
    before = len(a.accepted())
    a.status.current_hp = 100
    a.goal(0., 0., after=before)
    a.status.current_hp = 250
    time.sleep(0.3)
    assert a.accepted()[-1]["xy"][-1] == [0., 0.]
    a.check("hp_hysteresis")
    index = len(a.events)
    a.rfid.friendly_supply_zone_non_overlap = 1
    a.stopped(index, stationary_spin=True)
    count = len(a.accepted())
    time.sleep(0.3)
    assert len(a.accepted()) == count, "supply RFID did not hold position"
    a.check("rfid_cancels_navigation")
    a.status.current_hp = 380
    a.goal(*target, after=count)
    a.rfid.friendly_supply_zone_non_overlap = 0
    if a.league == "ul":
        a.status.projectile_allowance_17mm = 0
        count = len(a.accepted())
        time.sleep(0.4)
        assert len(a.accepted()) == count
        assert not a.postures
        a.check("ul_no_ammo_resupply_or_posture")
    index = len(a.events)
    a.status.current_hp = 0
    a.stopped(index)
    a.check("death_stop")
    before = len(a.accepted())
    a.status.current_hp = 80
    a.goal(0., 0., after=before)
    a.status.current_hp = 380
    time.sleep(0.3)
    assert a.accepted()[-1]["xy"][-1] == [0., 0.]
    a.rfid.friendly_supply_zone_non_overlap = 1
    a.goal(*target, after=len(a.accepted()))
    a.rfid.friendly_supply_zone_non_overlap = 0
    a.check("resurrection_supply_until_rfid")
    index = len(a.events)
    a.suspended.add("game")
    a.stopped(index)
    count = len(a.accepted())
    time.sleep(0.3)
    assert len(a.accepted()) == count
    a.check("referee_timeout_stop")
    a.suspended.clear()
    a.goal(*target, after=count)
    index = len(a.events)
    a.frame = "odom"
    a.stopped(index)
    a.check("wrong_frame_localization_timeout")
    count = len(a.accepted())
    a.frame = "map"
    a.goal(*target, after=count)
    assert not a.set_parameters({"spin.moving_speed": 9., "spin.arrive_tolerance": 1., "spin.leave_tolerance": 0.1})
    assert not a.set_parameters({"spin.moving_speed": math.nan})
    assert not a.set_parameters({"spin.moving_speed": 9., "ros_plugins_timeout": -1})
    time.sleep(.15)
    assert "wz=2.50" in a.spins[-1], "rejected parameter batch changed running spin"
    assert a.set_parameters({"spin.moving_speed": 1.5, "spin.arrived_speed": 8.5})
    wait_for(lambda: a.spins and "wz=1.50" in a.spins[-1], "spin hot reload failed")
    a.check("atomic_parameter_validation_and_hot_reload")
    # The production execution Action must reject a second concurrent tree.
    duplicate = a.send_tree()
    assert not duplicate.accepted
    index = len(a.events)
    a.future(a.tree.cancel_goal_async())
    a.future(a.tree.get_result_async())
    a.stopped(index)
    count = len(a.accepted())
    time.sleep(0.4)
    assert len(a.accepted()) == count, "canceled tree restarted itself"
    a.check("cancel_stays_stopped")
    a.spawn("autostart-client", ["ros2", "run", "srm27_behavior", "srm27_behavior_client", "--ros-args",
                                  "-p", f"target_tree:=rm{a.league}_2027"])
    a.goal(*target, after=count)
    cancel_client = a.node.create_client(CancelGoal, "srm27_behavior/_action/cancel_goal")
    assert cancel_client.wait_for_service(timeout_sec=5)
    index = len(a.events)
    assert a.future(cancel_client.call_async(CancelGoal.Request())).goals_canceling
    a.stopped(index)
    count = len(a.accepted())
    time.sleep(.6)
    assert len(a.accepted()) == count, "production client automatically restarted canceled execution"
    a.check("production_autostart_client_does_not_restart_after_cancel")
    a.node.destroy_client(cancel_client)
    a.tree = a.send_tree()
    assert a.tree.accepted
    a.goal(*target, after=count)
    # Finish the match with an active low-HP latch, then begin another at mid HP.
    a.status.current_hp = 100
    a.goal(0., 0., after=len(a.accepted()))
    a.game.game_progress = 5
    time.sleep(0.4)
    a.status.current_hp = 200
    # HP and game status arrive on separate topics; establish next-match HP first.
    wait_for(lambda: a.reports and a.reports[-1]['hp'] == 200 and a.reports[-1]['mode'] == 'wait',
             'next-match HP not observed before match start')
    a.game.game_progress = 4
    a.game.stage_remain_time = 299 if a.league == "ul" else 419
    a.goal(*target, after=len(a.accepted()))
    a.check("new_match_resets_latches")
    # Every new goal must follow the previous goal's terminal result, across both actions.
    active = None
    for event in a.events:
        if event["event"] == "accepted":
            assert active is None, f"overlapping navigation: {a.events}"
            active = event["seq"]
        elif event["seq"] == active:
            active = None
    a.check("cancel_before_send_across_action_types")


def test_navigation_failure_paths(arena):
    a = arena
    target = (4.5, -3.2) if a.league == "ul" else (3.5, 0.)
    a.reject = True
    a.game.game_progress = 4
    wait_for(lambda: a.rejections >= 2, "goal rejection was not retried")
    assert not a.accepted()
    a.check("rejected_goal_no_crash_bounded_retry")
    a.reject = False
    a.abort = True
    wait_for(lambda: any(e["event"] == "aborted" for e in a.events), "abort result lost")
    a.abort = False
    a.goal(*target, after=len(a.accepted()))
    assert any(report["nav_fault"] == "navigation_failed" for report in a.reports)
    a.check("immediate_abort_and_recovery")
    # Interrupt while the next goal response is pending, rather than after acceptance.
    a.accept_delay = .35
    previous = len(a.accepted())
    a.status.current_hp = 100
    time.sleep(.1)
    a.game.game_progress = 5
    wait_for(lambda: len(a.accepted()) > previous, "delayed goal was not received")
    wait_for(lambda: a.events[-1]["event"] == "canceled", "pending goal survived stop")
    a.check("halt_while_goal_acceptance_pending")
    a.accept_delay = 0.
    a.status.current_hp = 400
    a.game.game_progress = 4
    a.goal(*target, after=len(a.accepted()))
    a.reject_cancel = True
    a.status.current_hp = 100
    result = a.future(a.tree.get_result_async())
    assert result.status == 6, "cancellation failure should abort decision execution"
    # stop_all overrides velocity still arriving from the uncancelable navigation.
    wait_for(lambda: a.outputs and a.outputs[-1].linear.x == 0. and
             a.outputs[-1].angular.z == 0., "cancel failure did not latch mux stop")
    assert a.accepted()[-1]["xy"][-1] == list(target), "new navigation sent despite cancel failure"
    a.check("cancel_failure_stops_mux_without_new_goal")


def test_goal_acceptance_timeout_stops_ambiguous_navigation(arena):
    a = arena
    a.accept_delay = 2.5  # Longer than both the request and cancellation acceptance budgets.
    a.game.game_progress = 4
    result = a.future(a.tree.get_result_async())
    assert result.status == 6
    wait_for(lambda: bool(a.accepted()), "late navigation acceptance was not exercised")
    time.sleep(.3)
    assert len(a.accepted()) == 1, "ambiguous acceptance caused duplicate navigation"
    assert a.outputs and a.outputs[-1].linear.x == 0. and a.outputs[-1].angular.z == 0.
    a.check("lost_or_delayed_acceptance_latches_stop_without_duplicate_goal")


@pytest.mark.parametrize('league', ['ul', 'uc'])
def test_full_hp_supply_exit_and_new_match_clear_hysteresis(league):
    init_test_ros()
    a = Arena.__new__(Arena)
    try:
        a.__init__(league, 'full-hp-and-new-match')
        a.game.game_progress = 4
        target = (4.5, -3.2) if league == 'ul' else (3.5, 0.)
        a.goal(*target)
        a.status.current_hp = 100
        a.goal(0., 0., after=len(a.accepted()))
        a.status.current_hp = 380
        time.sleep(.3)
        assert a.accepted()[-1]['xy'][-1] == [0., 0.]
        a.status.current_hp = a.status.maximum_hp
        a.goal(*target, after=len(a.accepted()))
        a.check('满血且无补给RFID时解除回补')
        a.status.current_hp = 100
        a.goal(0., 0., after=len(a.accepted()))
        index = len(a.events)
        a.game.game_progress = 0
        a.stopped(index)
        a.status.current_hp = 250
        time.sleep(.2)
        before = len(a.accepted())
        a.game.game_progress = 4
        a.goal(*target, after=before)
        a.check('新局不继承上一局的低血量滞回')
    finally:
        try:
            if hasattr(a, 'node'):
                a.close()
        finally:
            rclpy.shutdown()


def test_ul_neutral_contest_respects_hp_and_enemy_control():
    init_test_ros()
    a = Arena.__new__(Arena)
    try:
        a.__init__('ul', 'neutral-contest')
        a.field.center_control = 0
        a.field.center_gain_point_status = 2
        a.game.game_progress = 4
        a.goal(4.5, -3.2)
        a.status.current_hp = 240
        a.goal(3., -3.2, after=len(a.accepted()))
        a.status.current_hp = 300
        a.goal(4.5, -3.2, after=len(a.accepted()))
        a.field.center_control = 2
        a.goal(3., -3.2, after=len(a.accepted()))
        a.check('中立点争夺遵循血量门槛且敌控时回退')
    finally:
        try:
            if hasattr(a, 'node'):
                a.close()
        finally:
            rclpy.shutdown()


def test_ul_center_control_activation_freshness_and_supply_priority():
    init_test_ros()
    a = Arena.__new__(Arena)
    try:
        a.__init__('ul', 'center-fallback', overrides={'ul_contest_neutral': 'false'})
        a.game.game_progress = 4
        a.goal(4.5, -3.2)
        for control, activation, target in [
            (2, 2, (3., -3.2)),  # Enemy controls and exclusively activates the point.
            (2, 3, (4.5, -3.2)), # Contested activation pauses progress; control can still be enemy.
            (2, 0, (3., -3.2)),  # Empty point does not instantly lose its retained control.
            (0, 0, (4.5, -3.2)),
            (0, 2, (3., -3.2)),  # Enemy can activate before its 5-second capture completes.
            (1, 1, (4.5, -3.2)),
        ]:
            before = len(a.accepted())
            a.field.center_control = control
            a.field.center_gain_point_status = activation
            a.goal(*target, after=before)
        a.check('2027_control_and_activation_are_independent')
        a.suspended.add('field')
        a.goal(3., -3.2, after=len(a.accepted()))
        assert not a.reports[-1]['ul_center_valid']
        a.check('stale_center_feedback_retreats')
        a.suspended.remove('field')
        a.goal(4.5, -3.2, after=len(a.accepted()))
        a.field.center_control_valid = False
        a.goal(3., -3.2, after=len(a.accepted()))
        a.check('legacy_center_feedback_is_unknown')
        a.field.center_control_valid = True
        a.field.center_control = 3
        time.sleep(.25)
        assert not a.reports[-1]['ul_center_valid']
        a.field.center_control = 2
        a.field.center_gain_point_status = 2
        a.status.current_hp = 100
        a.goal(0., 0., after=len(a.accepted()))
        assert a.reports[-1]['mode'] == 'recover'
        a.check('hp_recovery_preempts_center_fallback')
        assert not a.postures
        active = None
        for event in a.events:
            if event['event'] == 'accepted':
                assert active is None, 'center switch overlapped navigation actions'
                active = event['seq']
            elif event['seq'] == active:
                active = None
        a.check('center_switch_cancels_before_send')
    finally:
        a.close()
        rclpy.shutdown()


@pytest.mark.parametrize('fallback', [False, True])
def test_ul_repositions_from_requested_point_instead_of_completion_anchor(fallback):
    init_test_ros()
    a = Arena.__new__(Arena)
    try:
        a.__init__('ul', f'reposition-{fallback}')
        if fallback:
            a.field.center_control = 2
            a.field.center_gain_point_status = 2
        target = (3., -3.2) if fallback else (4.5, -3.2)
        a.game.game_progress = 4
        a.goal(*target)
        a.x, a.y = target[0] - .30, target[1]
        a.succeed = True
        wait_for(lambda: any(e['event'] == 'succeeded' for e in a.events), 'Nav2 success missing')
        wait_for(lambda: a.outputs and abs(a.outputs[-1].angular.z - 7.) < .01, 'hold spin missing')
        a.succeed = False
        count = len(a.accepted())
        a.x = target[0] - .49
        time.sleep(.3)
        assert len(a.accepted()) == count, 'inside hold radius resent navigation'
        # Only 0.21m from the accepted stopping pose, but >0.5m from the configured point.
        a.x = target[0] - .51
        a.goal(*target, after=count)
        a.check('strict_half_meter_reposition_from_configured_point')
    finally:
        a.close()
        rclpy.shutdown()


def test_hp_drop_random_spin_preserves_navigation_semantic_and_stop_gates(arena):
    a = arena
    a.game.game_progress = 4
    a.goal(*((4.5, -3.2) if a.league == 'ul' else (3.5, 0.)))
    time.sleep(.3)  # Establish the real HP-difference condition's baseline.
    for name in ('spin.moving_speed', 'spin.arrived_speed', 'spin.supply_speed', 'spin.evasion_speed', 'spin.evasion_min_speed'):
        assert not a.set_parameters({name: -1.})
        assert not a.set_parameters({name: math.nan})
    assert not a.set_parameters({'spin.max_accel': 0.})
    assert not a.set_parameters({'spin.max_jerk': 20.})  # Read-only until restart.
    assert not a.set_parameters({'spin.evasion_min_speed': 9.0})
    assert not a.set_parameters({'spin.evasion_duration_ms': 0})
    assert a.set_parameters({'spin.evasion_min_speed': 3.0, 'spin.evasion_duration_ms': 5000})
    assert a.set_parameters({'spin.evasion_speed': 8.0})
    before = len(a.spin_requests)
    output_start = len(a.outputs)
    for _ in range(6):
        a.status.current_hp -= 10
        time.sleep(.65)  # Renew the real damage window while observing several random segments.
    requests = a.spin_requests[before:]
    assert len(requests) >= 40
    assert min(requests) > 0 and max(requests) <= 8.00001
    assert max(requests) - min(requests) > 1.
    assert 'HP_DROP_EVASION' in a.spins[-1]
    assert max(v.angular.z for v in a.outputs[output_start:]) > 4.
    assert min(v.angular.z for v in a.outputs[output_start:]) >= 0.
    if a.league == 'ul':
        assert a.accepted()[-1]['xy'][-1] == [4.5, -3.2]
        assert not a.postures
    else:
        a.goal(2.5, 1.5)
    (a.out / 'random-spin.json').write_text(json.dumps({
        'requests': requests,
        'mux_wz': [v.angular.z for v in a.outputs[output_start:]],
        'target_range': [3., 8.], 'max_accel': 8., 'max_jerk': 40.,
    }, indent=2) + '\n')
    a.check('hp_drop_positive_random_spin_reaches_real_mux_without_changing_navigation')
    a.x, a.y = 101., 101. if a.league == 'ul' else 100.5
    wait_for(lambda: 'SEMANTIC_ZONE' in a.spins[-1] and 'wz=0.00' in a.spins[-1],
             'semantic limit did not override attack spin')
    a.check('semantic_spin_limit_preempts_random_spin')
    a.x, a.y = -2., 0.
    a.status.current_hp -= 10
    wait_for(lambda: 'HP_DROP_EVASION' in a.spins[-1], 'attack spin did not resume')
    a.manual_stop()
    wait_for(lambda: a.outputs and a.outputs[-1].linear.x == 0. and a.outputs[-1].angular.z == 0.,
             'manual stop did not override attack spin')
    a.check('manual_stop_preempts_random_spin')
    wait_for(lambda: 'HP_DROP_EVASION' not in a.spins[-1], 'random spin survived HP-drop window', timeout=7)
    wait_for(lambda: 'wz=2.50' in a.spins[-1], 'constant moving spin did not resume', timeout=4)
    a.check('random_spin_expires_and_smoothly_returns_to_constant_speed')


def test_feedback_flood_does_not_starve_action_responses(arena):
    a = arena
    a.feedback_period = .005  # More feedback arrivals than BT ticks, as with real Nav2.
    target = (4.5, -3.2) if a.league == "ul" else (.8, 0.)
    if a.league == "uc":
        a.ally.ally_base_hp = 1000
        time.sleep(.15)
    a.game.game_progress = 4
    a.goal(*target)
    time.sleep(1.6)
    assert len(a.accepted()) == 1, "feedback starved acceptance and triggered duplicate goals"
    assert not any(e["event"] == "canceled" for e in a.events)
    a.x, a.y = target
    a.succeed = True
    wait_for(lambda: a.outputs and abs(a.outputs[-1].angular.z - 7.) < .01,
             "feedback starved the terminal result")
    a.check("high_rate_feedback_preserves_acceptance_and_terminal_result")


def test_process_shutdown_while_navigation_is_running(arena):
    a = arena
    a.game.game_progress = 4
    target = (4.5, -3.2) if a.league == "ul" else (3.5, 0.)
    a.goal(*target)
    server = a.processes[1]
    os.killpg(server.pid, signal.SIGINT)
    code = server.wait(timeout=6)
    assert code in (0, -signal.SIGINT, 130), f"active-tree shutdown crashed: {code}"
    a.checks.append("active_navigation_process_shutdown_without_crash")


def test_decision_lease_identity_and_manual_stop(arena):
    a = arena
    a.game.game_progress = 4
    wait_for(lambda: a.outputs and a.outputs[-1].linear.x > 0, 'initial motion missing')
    os.killpg(a.processes[1].pid, signal.SIGSTOP)
    try:
        wait_for(lambda: a.outputs[-1].linear.x == 0 and a.outputs[-1].angular.z == 0,
                 'frozen decision did not expire motion permission', timeout=2)
        time.sleep(.2)
        assert a.outputs[-1].linear.x == 0
    finally:
        os.killpg(a.processes[1].pid, signal.SIGCONT)
    wait_for(lambda: a.outputs[-1].linear.x > 0, 'decision reconnect did not resume')
    a.check('decision_frozen_stops_even_with_live_navigation')
    ready = subprocess.run(['python3', 'src/decision/backend/srm27_behavior/scripts/check_decision.py', '--seconds', '1',
                            '--output-topic', 'cmd_vel_test_output', '--output', str(a.out / 'readiness.json')],
                           capture_output=True, text=True, timeout=8)
    assert ready.returncode == 0, ready.stdout + ready.stderr
    a.check('read_only_commissioning_cli')
    a.suspended.add('identity')  # aggregate HP/heat and ammo continue arriving
    wait_for(lambda: not a.reports[-1]['inputs_ready'] and a.outputs[-1].linear.x == 0,
             'aggregate messages hid raw identity timeout')
    a.suspended.remove('identity')
    wait_for(lambda: a.outputs[-1].linear.x > 0, 'identity reconnect did not resume')
    a.check('aggregate_cannot_refresh_identity_hp')
    a.manual_stop()
    wait_for(lambda: a.outputs[-1].linear.x == 0, 'manual stop did not apply')
    a.game.game_progress = 0
    time.sleep(.3)
    a.game.game_progress = 4
    time.sleep(.8)
    assert a.outputs[-1].linear.x == 0 and a.outputs[-1].angular.z == 0
    a.check('manual_stop_survives_match_gate_reopening')
    a.manual_stop(False)
    wait_for(lambda: a.outputs[-1].linear.x > 0, 'explicit manual resume failed')
    a.chassis_enabled = False
    wait_for(lambda: a.outputs[-1].linear.x == 0 and not a.reports[-1]['motion_permit'],
             'chassis power gate failed')
    a.check('raw_chassis_power_gate')
    a.chassis_enabled = True
    wait_for(lambda: a.outputs[-1].linear.x > 0, 'power reconnect failed')
    a.pose_stamp = a.node.get_clock().now().to_msg()
    wait_for(lambda: a.outputs[-1].linear.x == 0 and not a.reports[-1]['inputs_ready'],
             'repeated frozen pose stamps kept localization alive')
    a.pose_stamp = None
    wait_for(lambda: a.outputs[-1].linear.x > 0, 'localization reconnect failed')
    a.check('repeated_pose_timestamp_does_not_refresh_localization')
    a.manual_stop()
    a.suspended.add('velocity')
    time.sleep(.15)
    a.manual_stop(False)
    time.sleep(.2)
    assert a.outputs[-1].linear.x == 0, 'manual resume replayed cached velocity'
    a.suspended.remove('velocity')
    wait_for(lambda: a.outputs[-1].linear.x > 0, 'new velocity did not resume')
    a.check('resume_waits_for_fresh_navigation_velocity')
    a.chassis_enabled = True
    a.game.stage_remain_time = 0
    time.sleep(.3)
    assert not a.reports[-1]['motion_permit']
    a.check('zero_remaining_time_stops_without_progress_change')


def test_uc_territory_priorities_feedback_and_free_ammo():
    init_test_ros()
    a = Arena.__new__(Arena)
    try:
        a.__init__('uc', 'territory', overrides={'uc_enemy_base_enabled': 'true'})
        a.field.battlefield_valid = True
        a.field.outpost_control = [0, 2]
        a.field.base_control = [0, 2]
        a.game.game_progress = 4
        a.goal(5., 1.2)
        a.check('capture_enemy_outpost_instead_of_blind_patrol')
        a.field.outpost_control = [0, 1]
        time.sleep(.3)
        assert a.reports[-1]['mode'] == 'capture_outpost'
        a.field.outpost_control = [0, 0]
        wait_for(lambda: a.reports[-1]['mode'] == 'hold_outposts', 'full-control feedback not used')
        a.check('temporary_control_does_not_count_as_full_control')
        a.field.outpost_control = [1, 0]
        a.goal(3.5, 0., after=len(a.accepted()))
        a.check('own_outpost_recovery_preempts_attack')
        a.field.base_control = [1, 2]
        a.goal(.8, 0., after=len(a.accepted()))
        a.check('base_territory_loss_preempts_outpost_recovery')
        a.field.base_control = [0, 2]
        a.field.outpost_control = [0, 0]
        a.ally.control_score_valid = True
        a.ally.red_outpost_hp = a.ally.blue_outpost_hp = 1500
        a.ally.ally_control_degree = 50
        a.ally.enemy_control_degree = 100
        a.game.stage_remain_time = 50
        a.goal(6., 0., after=len(a.accepted()))
        a.check('optional_endgame_base_pressure_uses_score_deficit')
        a.ally.ally_control_degree = 200
        a.goal(5., 1.2, after=len(a.accepted()))
        a.check('leading_team_keeps_outpost_income')
        a.suspended.add('field')
        wait_for(lambda: a.reports[-1]['mode'] == 'fortress', 'stale territory did not fall back')
        a.check('stale_arena_falls_back_without_claiming_control')
        a.suspended.remove('field')
        # The arrays are R/B, while 0/1/2 ownership is relative to the receiving team.
        a.status.robot_id = 107
        a.field.outpost_control = [0, 1]
        a.field.base_control = [2, 0]
        a.goal(3.5, 0., after=len(a.accepted()))
        wait_for(lambda: a.reports[-1]['mode'] == 'restore_outpost', 'blue-side mapping failed')
        a.check('blue_identity_maps_own_territory_to_B')
        a.field.outpost_control = [2, 0]
        a.status.projectile_allowance_17mm = 20
        a.game.stage_remain_time = 350
        a.goal(0., 0., after=len(a.accepted()))
        a.rfid.friendly_supply_zone_non_overlap = 1
        count = len(a.accepted())
        time.sleep(.3)
        assert a.reports[-1]['mode'] == 'recover'
        a.status.projectile_allowance_17mm = 160
        a.goal(5., 1.2, after=count)
        a.check('free_ammo_recovery_waits_for_real_allowance_and_rfid')
        a.rfid.friendly_supply_zone_non_overlap = 0
        a.field.base_activation = [0, 2]  # blue is the current own side
        a.status.projectile_allowance_17mm = 20
        a.goal(.8, 0., after=len(a.accepted()))
        a.check('base_intrusion_preempts_noncritical_ammo_visit')
    finally:
        a.close()
        rclpy.shutdown()


@pytest.mark.parametrize('dry_run,calibrated', [('true', 'true'), ('false', 'false')])
def test_observation_and_calibration_never_command_motion(dry_run, calibrated):
    init_test_ros()
    a = Arena.__new__(Arena)
    try:
        a.__init__('ul', f'observe-{dry_run}-{calibrated}',
                   overrides={'dry_run': dry_run, 'waypoints_calibrated': calibrated})
        a.game.game_progress = 4
        wait_for(lambda: bool(a.reports) and bool(a.outputs), 'decision or mux stream missing')
        time.sleep(.4)
        assert not a.accepted() and not a.postures
        assert all(not value for value in a.permits)
        assert a.outputs[-1].linear.x == 0 and a.outputs[-1].angular.z == 0
        if dry_run == 'true':
            assert not a.permits
            assert a.reports[-1]['mode'] == 'control'
        a.check('dry_run_or_uncalibrated_has_no_motion')
        ready = subprocess.run(['python3', 'src/decision/backend/srm27_behavior/scripts/check_decision.py', '--seconds', '1',
                                '--output-topic', 'cmd_vel_test_output', '--output', str(a.out / 'readiness.json')],
                               capture_output=True, text=True, timeout=8)
        assert ready.returncode == 1, 'readiness accepted a dry-run or uncalibrated configuration'
        a.check('readiness_rejects_observation_or_uncalibrated_configuration')
    finally:
        a.close()
        rclpy.shutdown()


def test_supply_attempt_has_timeout_shelter_and_retry():
    init_test_ros()
    a = Arena.__new__(Arena)
    try:
        a.__init__('ul', 'supply-window', overrides={'supply_timeout': 1.0, 'supply_retry': 1.0})
        a.status.current_hp = 100
        a.game.game_progress = 4
        a.goal(0., 0.)
        a.goal(1., 0., after=len(a.accepted()))
        a.check('unavailable_supply_enters_shelter')
        a.goal(0., 0., after=len(a.accepted()))
        a.check('bounded_supply_retries_after_cooldown')
    finally:
        a.close()
        rclpy.shutdown()


def test_serial_referee_2027_and_legacy_frames():
    """CRC-framed bytes through the actual serial node, isolated by an OS pseudo-terminal."""
    import struct
    import sys
    import tty
    sys.path.insert(0, str(Path(__file__).resolve().parents[5] / 'scripts/test'))
    from test_chassis_speed_feedback import build_frame, crc16
    init_test_ros()
    master, slave = os.openpty()
    tty.setraw(master)
    tty.setraw(slave)
    node = rclpy.create_node('referee_serial_test')
    received_event, received_hp = [], []
    speed_pub = node.create_publisher(Twist, 'cmd_vel_chassis', 10)
    speed_command = None
    downlink = []
    downlink_buffer = bytearray()

    subscriptions = [
        node.create_subscription(EventData, 'referee/event_data', received_event.append, 10),
        node.create_subscription(AllyRobotHP, 'referee/ally_robot_hp', received_hp.append, 10),
    ]
    from rclpy.executors import SingleThreadedExecutor
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    out = OUT / 'serial-referee'
    out.mkdir(parents=True, exist_ok=True)
    log = (out / 'protocol.log').open('w')
    process = subprocess.Popen([
        'ros2', 'run', 'srm27_nav_protocol', 'srm27_nav_protocol_node', '--ros-args',
        '-r', '__node:=srm27_nav_protocol', '--params-file',
        str(Path(get_package_share_directory('srm27_nav_protocol')) / 'config/srm27_nav_protocol.yaml'),
        '-p', f'device_name:={os.ttyname(slave)}', '-p', 'baud_rate:=115200',
        '-p', 'flow_control:=none', '-p', 'parity:=none', "-p", "stop_bits:='1'",
        '-r', '/srm/sefdefined:=test/serial_identity',
    ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    frames = [build_frame(0x0101, struct.pack('<II', 2 << 30, (2 << 2) | (3 << 24) | (2 << 26))) +
              build_frame(0x0003, struct.pack('<4Hh7H', 100, 0, 180, 190, -5, 380,
                                             1200, 4000, 1400, 3000, 123, 456))]
    stopping = threading.Event()
    def feed():
        while not stopping.wait(.05):
            os.write(master, frames[0])
            if speed_command is not None:
                speed_pub.publish(speed_command)
            import select
            if select.select([master], [], [], 0)[0]:
                downlink_buffer.extend(os.read(master, 4096))
                while len(downlink_buffer) >= 19:
                    frame = bytes(downlink_buffer[:19])
                    if frame[:3] != b'\xa5\x0c\x00' or crc16(frame[:-2]) != int.from_bytes(frame[-2:], 'little'):
                        del downlink_buffer[0]
                        continue
                    downlink.append(struct.unpack_from('<fff', frame, 5))
                    del downlink_buffer[:19]
    feeder = threading.Thread(target=feed, daemon=True)
    feeder.start()
    try:
        wait_for(lambda: received_event and received_hp, '2027 serial frames not decoded', timeout=15)
        assert process.poll() is None
        assert received_event[-1].battlefield_valid
        assert list(received_event[-1].outpost_control) == [0, 2]
        assert received_event[-1].center_control_valid
        assert received_event[-1].center_control == 2 and received_event[-1].center_gain_point_status == 3
        assert received_hp[-1].control_score_valid
        assert received_hp[-1].ally_control_degree == 123 and received_hp[-1].enemy_control_degree == 456
        assert received_hp[-1].blue_outpost_hp == 1400
        frames[0] = build_frame(0x0101, struct.pack('<I', 0)) + build_frame(0x0003, struct.pack('<8H', 1, 2, 3, 4, 0, 380, 1500, 5000))
        wait_for(lambda: not received_event[-1].battlefield_valid and not received_event[-1].center_control_valid
                 and not received_hp[-1].control_score_valid,
                 'legacy serial frames not preserved')
        assert received_hp[-1].ally_outpost_hp == 1500
        frames[0] = build_frame(0x0101, bytes(6)) + build_frame(0x0003, bytes(20))
        time.sleep(.15)
        counts = len(received_event), len(received_hp)
        time.sleep(.2)
        assert counts == (len(received_event), len(received_hp)), 'invalid payload length published state'
        assert process.poll() is None
        command = Twist()
        command.linear.x = command.linear.y = 10.
        command.angular.z = 20.
        start = len(downlink)
        speed_command = command
        wait_for(lambda: len(downlink) > start and downlink[-1][2] > 8.49 and
                 math.hypot(*downlink[-1][:2]) > 2.59, 'serial output did not reach the configured caps')
        applied = downlink[start:]
        assert all(math.hypot(vx, vy) <= 2.601 and abs(wz) <= 8.501 for vx, vy, wz in applied)
        assert math.isclose(applied[-1][0], applied[-1][1], abs_tol=.001)
        assert any(0. < wz < 8. for _, _, wz in applied), 'serial output skipped the ramp'
        speed_command = None
        wait_for(lambda: downlink[-1] == (0., 0., 0.), 'serial watchdog did not stop output')
        (out / 'manifest.json').write_text(json.dumps({'checks': [
            '2027_crc_serial_to_ros', 'legacy_frames_keep_control_invalid', 'wrong_lengths_rejected',
            'serial_exit_caps_and_watchdog']}, indent=2))
    finally:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=5)
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
            stopping.set()
            feeder.join(timeout=2)
            executor.shutdown(timeout_sec=2)
            spin.join(timeout=2)
            node.destroy_node()
            rclpy.shutdown()
            os.close(master)
            os.close(slave)
            log.close()
        assert process.returncode in (0, -signal.SIGINT, 130), 'serial node shutdown failed'


@pytest.mark.parametrize('fault', ['false_success', 'running_timeout'])
def test_bad_navigation_result_and_runtime_budget(fault):
    init_test_ros()
    a = Arena.__new__(Arena)
    try:
        a.__init__('ul', fault, overrides={'navigation_timeout': 1.0 if fault == 'running_timeout' else 45.0})
        a.game.game_progress = 4
        a.goal(4.5, -3.2)
        if fault == 'false_success':
            a.succeed = True  # The robot's localization is still far from the destination.
        expected = 'success_outside_tolerance' if fault == 'false_success' else 'navigation_timeout'
        wait_for(lambda: any(r.get('nav_fault') == expected for r in a.reports), 'navigation fault not exposed')
        wait_for(lambda: a.outputs[-1].linear.x == 0 and a.outputs[-1].angular.z == 0,
                 'navigation fault did not revoke motion')
        a.succeed = False
        count = len(a.accepted())
        wait_for(lambda: len(a.accepted()) > count, 'navigation did not retry after backoff')
        a.check(expected + '_stops_and_retries')
    finally:
        a.close()
        rclpy.shutdown()


def test_supply_rfid_does_not_bypass_pending_navigation():
    init_test_ros()
    a = Arena.__new__(Arena)
    try:
        a.__init__('ul', 'rfid-pending')
        a.rfid.friendly_supply_zone_non_overlap = 1
        a.accept_delay = .65
        a.game.game_progress = 4
        wait_for(lambda: a.reports and a.reports[-1]['mode'] == 'control', 'control decision missing')
        velocity = Twist()
        velocity.linear.x = .2
        for _ in range(8):
            a.nav_pub.publish(velocity)
            time.sleep(.02)
            assert not a.accepted(), 'fixture exceeded the pending-acceptance window'
            assert a.outputs[-1].linear.x == 0 and not a.reports[-1]['motion_permit']
        a.goal(4.5, -3.2)
        wait_for(lambda: a.outputs[-1].linear.x > 0, 'accepted navigation stayed blocked')
        a.check('supply_rfid_does_not_grant_permission_to_unaccepted_goal')
    finally:
        a.close()
        rclpy.shutdown()


@pytest.mark.parametrize('spin_topic', ['', 'rotation_cmd'])
def test_mux_all_inputs_obey_decision_lease(spin_topic):
    """Both production dual input and single-input fallback must discard blocked inputs."""
    from std_msgs.msg import Float64
    init_test_ros()
    out = OUT / ('mux-dual-lease' if spin_topic else 'mux-single-lease')
    out.mkdir(parents=True, exist_ok=True)
    node = rclpy.create_node('mux_lease_arena')
    outputs = []
    nav = node.create_publisher(Twist, 'lease/nav', 1)
    spin = node.create_publisher(Twist, 'rotation_cmd', 1)
    hold = node.create_publisher(Float64, 'behavior/hold_spin', 1)
    permit = node.create_publisher(Bool, 'decision/motion_permit', 1)
    subscription = node.create_subscription(Twist, 'lease/output', outputs.append, 10)
    with (out / 'mux.log').open('w') as log:
        process = subprocess.Popen([
            'ros2', 'run', 'srm27_chassis_control', 'cmd_mux', '--ros-args',
            '-p', 'nav_topic:=lease/nav', '-p', 'output_topic:=lease/output',
            '-p', "spin_topic:='rotation_cmd'" if spin_topic else "spin_topic:=''",
            '-p', 'wz_max:=9.0', '-p', 'wz_ramp:=25.0'],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        def pump(seconds, allowed=None, command=None, rotation=None, stationary=None):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                if allowed is not None:
                    permit.publish(Bool(data=allowed))
                if command is not None:
                    nav.publish(command)
                if rotation is not None:
                    request = Twist()
                    request.angular.z = rotation
                    spin.publish(request)
                if stationary is not None:
                    hold.publish(Float64(data=stationary))
                rclpy.spin_once(node, timeout_sec=.02)
            assert process.poll() is None, 'mux exited; inspect mux.log'
        def stopped():
            assert outputs and all(abs(v) < 1e-9 for v in (
                outputs[-1].linear.x, outputs[-1].linear.y, outputs[-1].angular.z))
        try:
            command = Twist()
            command.linear.x = .2
            command.angular.z = .4
            pump(2., False, command, 5., 7.)
            stopped()
            pump(.3, True)
            stopped()  # opening permission cannot replay inputs sent while blocked
            pump(.4, True, command, 5., 7.)
            assert outputs[-1].linear.x == .2
            assert abs(outputs[-1].angular.z - (5. if spin_topic else .4)) < .01
            pump(.4, True, Twist(), 5., 7.)
            assert outputs[-1].linear.x == 0.
            assert abs(outputs[-1].angular.z - 7.) < .01  # stationary lease wins
            pump(.1, False, command, 5., 7.)
            stopped()  # permission close bypasses the angular ramp
            pump(.4, True, command, 5., 0.)
            pump(.65, None, command, 5., 7.)
            stopped()  # decision freeze closes translation, spin and stationary hold
            pump(.3, True)
            stopped()
            pump(.3, True, command, 5., 0.)
            assert outputs[-1].linear.x == .2
            pump(.7, True)  # independent nav/spin/hold freshness budgets
            stopped()
            (out / 'manifest.json').write_text(json.dumps({
                'spin_topic': spin_topic, 'checks': ['discard_blocked_inputs', 'fresh_after_reopen',
                    'moving_hold_interlock', 'stationary_override', 'decision_freeze_stops_all',
                    'input_timeout_stops_all']}, sort_keys=True, indent=2) + '\n')
        finally:
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=5)
            node.destroy_subscription(subscription)
            node.destroy_node()
            rclpy.shutdown()
