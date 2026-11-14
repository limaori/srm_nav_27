#!/usr/bin/env python3
"""Read-only commissioning check. Never publishes referee data or motion commands."""
import argparse
import json
from pathlib import Path
import time

import rclpy
from nav2_msgs.action import NavigateToPose
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--namespace', default='')
    parser.add_argument('--output-topic', default='cmd_vel_chassis')
    parser.add_argument('--seconds', type=float, default=5.0)
    parser.add_argument('--output', default='.cache/decision-validation/onboard.json')
    args = parser.parse_args()
    if not 0 < args.seconds <= 60:
        parser.error('--seconds must be in (0,60]')
    rclpy.init()
    node = Node('decision_readiness', namespace=args.namespace)
    received = []
    node.create_subscription(String, 'decision/status', lambda msg: received.append(
        (time.monotonic(), json.loads(msg.data))), 1)
    nav = ActionClient(node, NavigateToPose, 'navigate_to_pose')
    client = node.create_client(GetParameters, 'srm_cmd_mux/get_parameters')
    future = None
    deadline = time.monotonic() + args.seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if future is None and client.service_is_ready():
            future = client.call_async(GetParameters.Request(names=['decision_required']))
    publishers = node.get_publishers_info_by_topic(args.output_topic)
    status = received[-1][1] if received else {}
    guard = bool(future and future.done() and future.result().values and future.result().values[0].bool_value)
    checks = {
        'decision_status_fresh': bool(received and time.monotonic() - received[-1][0] < 0.5),
        'nav2_action_available': nav.server_is_ready(),
        'mux_guard_enabled': guard,
        'single_mux_motor_publisher': len(publishers) == 1 and publishers[0].node_name == 'srm_cmd_mux',
        'inputs_fresh': status.get('inputs_ready') is True,
        'waypoints_calibrated': status.get('waypoints_calibrated') is True,
        'execution_enabled': status.get('dry_run') is False,
        'decision_guard_ready': status.get('guard_ready') is True,
        'navigation_healthy': not status.get('nav_fault'),

    }
    result = {'passed': all(checks.values()), 'checks': checks, 'decision': status,
              'scope': 'read-only readiness; driving, RFID effects and stopping distance require field acceptance'}
    path = Path(args.output)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
    print(path.read_text())
    node.destroy_node()
    rclpy.shutdown()
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
