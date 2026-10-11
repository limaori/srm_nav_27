#!/usr/bin/env python3
"""简单速度脚本：给建图/回归试验驱动仿真里的 SRM 底盘。

订阅/发布都走 ``cmd_vel_sim``（srm_velocity_adapter 的唯一入口），
按时间表依次执行「直行 / 停 / 转」段。仅用于为建图或 S9 运动回归产生运动，
初始定位试验中机器人必须保持静止。

用法::

    ros2 run srm27_gicp_validation gicp_cmd_vel_script --ros-args \\
        -p namespace:=red_standard_robot1 -p route:=forward_turn_forward
"""

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


ROUTES = {
    # 命名路线：(vx, vy, wz, 持续秒数)
    "stationary": [(0.0, 0.0, 0.0, 5.0)],
    "forward": [(1.0, 0.0, 0.0, 15.0)],
    # 原地自转一整圈（约 40 s × 0.157 rad/s = 360°）：视点不变、覆盖全向，
    # 用于排除「运动中建图导致的重影」对配准残差的影响。
    "spin_360": [
        (0.0, 0.0, 0.0, 2.0),
        (0.0, 0.0, 0.15707963267948966, 40.0),
        (0.0, 0.0, 0.0, 2.0),
    ],
    # 建图用：慢速直线 + 一次 90° 转向 + 短直线，累计约 8 m，
    # 主要覆盖出生点周围的场地结构，避免长距离运动误差污染先验地图。
    "map_slow": [
        (0.6, 0.0, 0.0, 8.0),
        (0.0, 0.0, 0.0, 2.0),
        (0.0, 0.0, 0.6, 2.6),
        (0.0, 0.0, 0.0, 2.0),
        (0.6, 0.0, 0.0, 6.0),
        (0.0, 0.0, 0.0, 1.5),
    ],
    "forward_turn_forward": [
        (1.0, 0.0, 0.0, 12.0),
        (0.0, 0.0, 0.0, 2.0),
        (0.0, 0.0, 0.7, 2.3),
        (0.0, 0.0, 0.0, 1.5),
        (1.0, 0.0, 0.0, 8.0),
        (0.0, 0.0, 0.0, 1.5),
    ],
    "loop": [
        (1.0, 0.0, 0.0, 10.0),
        (0.0, 0.0, 0.0, 2.0),
        (0.0, 0.0, 0.7, 2.3),
        (0.0, 0.0, 0.0, 1.5),
        (1.0, 0.0, 0.0, 10.0),
        (0.0, 0.0, 0.0, 2.0),
        (0.0, 0.0, 0.7, 2.3),
        (0.0, 0.0, 0.0, 1.5),
        (1.0, 0.0, 0.0, 6.0),
        (0.0, 0.0, 0.0, 1.5),
    ],
    # S9：0.3 m/s 平移 + 独立自转短时回归
    "s9_motion": [
        (0.3, 0.0, 0.0, 4.0),
        (0.0, 0.0, 0.0, 1.0),
        (0.0, 0.0, 0.5, 4.0),
        (0.0, 0.0, 0.0, 1.0),
    ],
}


class CmdVelScript(Node):
    def __init__(self, topic: str, route: str, publish_rate: float) -> None:
        super().__init__("gicp_cmd_vel_script")
        self.publisher = self.create_publisher(Twist, topic, 10)
        self.route = ROUTES[route]
        self.period = 1.0 / publish_rate
        self.segment = 0
        self.segment_start = None
        self.done = False

    def step(self) -> None:
        if self.done:
            return
        now = time.monotonic()
        if self.segment_start is None:
            self.segment_start = now
            self.get_logger().info(
                "[cmd_vel_script] 段 %d/%d：vx=%.2f vy=%.2f wz=%.2f 持续 %.1f s"
                % ((self.segment + 1), len(self.route), *self.route[self.segment])
            )
        vx, vy, wz, duration = self.route[self.segment]
        if now - self.segment_start >= duration:
            self.segment += 1
            self.segment_start = now
            if self.segment >= len(self.route):
                self.publish_stop()
                self.done = True
                self.get_logger().info("[cmd_vel_script] 路线执行完成")
                return
            self.get_logger().info(
                "[cmd_vel_script] 段 %d/%d：vx=%.2f vy=%.2f wz=%.2f 持续 %.1f s"
                % ((self.segment + 1), len(self.route), *self.route[self.segment])
            )
            vx, vy, wz, duration = self.route[self.segment]
        del duration
        message = Twist()
        message.linear.x = float(vx)
        message.linear.y = float(vy)
        message.angular.z = float(wz)
        self.publisher.publish(message)

    def publish_stop(self) -> None:
        stop = Twist()
        for _ in range(20):
            self.publisher.publish(stop)
            time.sleep(0.01)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="仿真底盘速度脚本")
    parser.add_argument("--topic", default="red_standard_robot1/cmd_vel_sim")
    parser.add_argument("--route", default="forward", choices=sorted(ROUTES))
    parser.add_argument("--rate", type=float, default=20.0)
    known, remaining = parser.parse_known_args(argv)

    rclpy.init(args=remaining)
    node = CmdVelScript(known.topic, known.route, known.rate)
    try:
        while rclpy.ok() and not node.done:
            node.step()
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(node.period)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
