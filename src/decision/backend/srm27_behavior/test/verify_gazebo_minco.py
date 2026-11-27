"""Exercise the running 219_1121 Gazebo stack with simulated referee input."""

import argparse
import json
import math
from pathlib import Path
import time

from action_msgs.msg import GoalStatus, GoalStatusArray
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path as NavPath
import rclpy
from rclpy.qos import qos_profile_action_status_default, qos_profile_sensor_data
from rm_decision_interfaces.msg import EventData, GameStatus, Sefdefined
from srm27_minco_interfaces.msg import MincoTrajectory, MincoPathRequest
from std_msgs.msg import String

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--output", type=Path, required=True)
parser.add_argument("--namespace", default="red_standard_robot1")
args = parser.parse_args()
rclpy.init()
node = rclpy.create_node("check_gazebo_decision", namespace=args.namespace)
messages = {}
counts = {}
statuses = {}
spins = []
moving_spins = []
requests = set()
received_trajectories = set()


def track(key, message):
    messages[key] = message
    counts[key] = counts.get(key, 0) + 1


def subscribe(typ, topic, key, qos=10):
    node.create_subscription(typ, topic, lambda m: track(key, m), qos)


def nav_status(message):
    for item in message.status_list:
        key = bytes(item.goal_info.goal_id.uuid).hex()
        statuses.setdefault(key, set()).add(item.status)


def trajectory(message):
    track("trajectory", message)
    received_trajectories.add((message.path_version, message.goal_epoch))


def request(message):
    track("request", message)
    requests.add((message.path_version, message.goal_epoch))

node.create_subscription(
    GoalStatusArray, "navigate_to_pose/_action/status", nav_status, qos_profile_action_status_default,
)
node.create_subscription(MincoTrajectory, "planning/minco_trajectory", trajectory, 10)
node.create_subscription(MincoPathRequest, "planning/follow_path", request, 10)
node.create_subscription(Twist, "rotation_cmd", lambda m: spins.append(m.angular.z), 10)


def command_received(message):
    track("cmd", message)
    if math.hypot(message.linear.x, message.linear.y) > .1:
        moving_spins.append(round(message.angular.z, 2))

node.create_subscription(Twist, "cmd_vel_sim", command_received, 10)
subscribe(Odometry, "odometry", "odom", qos_profile_sensor_data)
subscribe(NavPath, "plan", "plan")
subscribe(NavPath, "plan_smoothed", "smoothed")
subscribe(DiagnosticArray, "FollowPath/diagnostics", "mpc")
subscribe(String, "decision/status", "decision")

game_pub = node.create_publisher(GameStatus, "referee/game_status", 10)
identity_pub = node.create_publisher(Sefdefined, "mock/sefdefined", 10)
event_pub = node.create_publisher(EventData, "referee/event_data", 10)
game = GameStatus(game_type=4, game_progress=0, stage_remain_time=299)
identity = Sefdefined(robot_id=7, current_hp=400, maximum_hp=400, power_management_chassis_output=1)
event = EventData(center_control_valid=True)
next_referee = 0.0
summary = {}


def spin():
    global next_referee
    if time.monotonic() >= next_referee:
        next_referee = time.monotonic() + .05
        game_pub.publish(game)
        identity_pub.publish(identity)
        event_pub.publish(event)
    rclpy.spin_once(node, timeout_sec=.02)


def wait(predicate, name, timeout=55):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        spin()
        if predicate():
            print(name, xy(), report(), counts, flush=True)
            return
    raise RuntimeError(f"{name}: xy={xy()} decision={report()} counts={counts} status={statuses} mpc={mpc()}")


def report():
    return json.loads(messages["decision"].data) if "decision" in messages else {}


def xy():
    if "odom" in messages:
        pos = messages["odom"].pose.pose.position
        return (round(pos.x + .985, 4), round(pos.y + .885, 4))
    return ()


def mpc():
    return {item.key: item.value for item in messages["mpc"].status[0].values} if "mpc" in messages else {}


def reached_count():
    return sum(GoalStatus.STATUS_SUCCEEDED in seq for seq in statuses.values())


def dist(point):
    return math.dist(xy(), point) if xy() else math.inf


def speed():
    return math.hypot(messages["cmd"].linear.x, messages["cmd"].linear.y) if "cmd" in messages else math.inf

try:
    wait(lambda: xy() and "cmd" in messages and report().get("guard_ready"),
         "system ready and decision guard active")
    assert report()["mode"] == "wait" and not report()["motion_permit"] and speed() < .01
    summary["spawn"] = xy()
    game.game_progress = 4
    wait(lambda: report().get("mode") == "control" and bool(requests & received_trajectories) and
         "smoothed" in messages and speed() > .1 and
         mpc().get("planning_result") == "external_trajectory", "YAstar, MinimumSnap and external MINCO/MPC moving", 70)
    assert messages["trajectory"].header.frame_id == "odom"
    assert messages["trajectory"].piece_durations
    assert messages["request"].path == messages["smoothed"]
    wait(lambda: reached_count() >= 1 and report().get("arrived") and dist((.69, .04)) <= .5 and
         speed() < .03 and messages["cmd"].angular.z > 4, "center reached and stationary spin")
    summary["center"] = {"xy": xy(), "distance_m": round(dist((.69, .04)), 4),
                         "applied_wz": messages["cmd"].angular.z, "moving_wz": moving_spins[-1:]}
    before = len(spins)
    identity.current_hp = 350
    wait(lambda: len(spins[before:]) > 20 and max(spins[before:]) - min(spins[before:]) > .16,
         "HP drop produces changing spin")
    summary["hp_drop_wz_range"] = [round(min(spins[before:]), 4), round(max(spins[before:]), 4)]
    identity.current_hp = 110
    wait(lambda: report().get("mode") == "recover", "low HP requests supply")
    wait(lambda: reached_count() >= 2 and dist((.985, .885)) <= .5 and speed() < .03,
         "supply reached")
    summary["supply"] = {"xy": xy(), "distance_m": round(dist((.985, .885)), 4)}
    game.game_progress = 5
    wait(lambda: report().get("mode") == "wait" and not report()["motion_permit"] and speed() < .01 and
         abs(messages["cmd"].angular.z) < .01, "match end revokes motion")
    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline:
        spin()
    stopped_trajectories = counts["trajectory"]
    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline:
        spin()
    assert counts["trajectory"] == stopped_trajectories, counts
    summary["trajectory_message_count"] = counts["trajectory"]
    assert len({epoch for _, epoch in requests & received_trajectories}) == 2
    summary["path_count"] = counts["plan"]
    summary["smooth_count"] = counts["smoothed"]
    summary["mpc_versions_confirmed"] = len(requests & received_trajectories)
    summary["action_statuses"] = [sorted(values) for values in statuses.values()]
    args.output.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
finally:
    node.destroy_node()
    rclpy.try_shutdown()
