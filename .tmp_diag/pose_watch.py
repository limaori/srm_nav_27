#!/usr/bin/env python3
"""后台监控：把 yaw 链的三路位姿 + 云台关节角按时间戳落成 CSV。

只读，不发任何话题。用于"云台什么时候转了一下"之后的事后对账：
  * theta_deg : 关节角（gimbal_yaw_joint，rad→deg）
  * raw_*     : /zlio/odometry（雷达位姿，odom->livox_imu）
  * lid_*     : /lidar_odometry（桥接后的雷达位姿）
  * base_*    : /odometry（车=云台位姿，odom->gimbal_yaw）
"""
import math
import os
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

OUT = os.environ.get("POSE_WATCH_OUT", "/home/srm/srm_nav_27/.tmp_diag/pose_watch.csv")
DURATION = float(os.environ.get("POSE_WATCH_DURATION", "900"))
JOINT = "gimbal_yaw_joint"


def yaw(q):
    return math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y),
                                   1 - 2 * (q.y * q.y + q.z * q.z)))


class Watch(Node):
    def __init__(self):
        super().__init__("pose_watch")
        self.f = open(OUT, "w", buffering=1)
        self.f.write("t,theta_deg,raw_x,raw_y,raw_yaw,lid_x,lid_y,lid_yaw,"
                     "base_x,base_y,base_yaw\n")
        qos = QoSProfile(depth=50, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        self.theta = float("nan")
        self.p = {}
        for topic, key in (("/zlio/odometry", "raw"), ("/lidar_odometry", "lid"),
                           ("/odometry", "base")):
            self.create_subscription(
                Odometry, topic,
                lambda m, k=key: self._odom(m, k), qos)
        self.create_subscription(JointState, "/joint_states", self._joint, qos)

    def _joint(self, m):
        for name, pos in zip(m.name, m.position):
            if name == JOINT:
                self.theta = math.degrees(pos)

    def _odom(self, m, key):
        p = m.pose.pose.position
        self.p[key] = (p.x, p.y, yaw(m.pose.pose.orientation))
        if key == "base" and "raw" in self.p:
            r, b = self.p["raw"], self.p["base"]
            lid = self.p.get("lid", (float("nan"),) * 3)
            self.f.write("%.3f,%.2f,%.4f,%.4f,%.2f,%.4f,%.4f,%.2f,%.4f,%.4f,%.2f\n" % (
                time.time(), self.theta, r[0], r[1], r[2],
                lid[0], lid[1], lid[2], b[0], b[1], b[2]))


def main():
    rclpy.init()
    n = Watch()
    t0 = time.time()
    try:
        while rclpy.ok() and time.time() - t0 < DURATION:
            rclpy.spin_once(n, timeout_sec=0.2)
    finally:
        n.f.close()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
