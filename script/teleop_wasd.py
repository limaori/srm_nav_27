#!/usr/bin/env python3

# Copyright 2026 SRM
#
# 仿真/实车通用的键盘速度控制器（WASD 全向平移 + QE 匀速自转）。
#
# 为什么不直接 teleop_twist_keyboard 发 cmd_vel_sim:
#   仿真链路里 cmd_vel_sim 是 srm_cmd_mux **独占发布** 的话题，mux 以 200 Hz
#   持续发布合成结果；外部再往 cmd_vel_sim 发会被它的零速覆盖。而且 mux 会
#   **丢弃导航输入的 angular.z** —— 自转必须走独立的自转链路。
#
# 正确的两路入口（都是 geometry_msgs/msg/Twist）:
#   平移  -> <ns>/cmd_vel_nav        只用 linear.x / linear.y（车体系）
#   自转  -> <ns>/rotation_cmd       只用 angular.z
#          rotation_controller -> rotation_velocity -> cmd_mux -> cmd_vel_sim
#
# 超时（必须持续发布，否则会被看门狗清零）:
#   cmd_mux 的导航输入 0.3 s、自转输入 0.1 s；rotation_controller 请求 0.5 s。
#   本脚本默认 20 Hz 连续发布。
#
# 按键:
#   w / s      前进 / 后退（+x / -x），再按一次取消该分量
#   a / d      左移 / 右移（+y / -y），再按一次取消该分量
#               w+a 这类组合会自动归一化到 --speed，所以斜向也是 1 m/s
#   q / e      逆时针 / 顺时针匀速自转，再按一次停止
#   空格       全部停止（平移 + 自转）
#   h          打印帮助      Ctrl-C 退出（退出前会补发零速）
#
# 用法:
#   ./script/teleop_wasd.py                                  # 默认 1 m/s、1 rad/s
#   ./script/teleop_wasd.py --speed 0.5 --yaw-rate 0.5
#   ./script/teleop_wasd.py --namespace ""                   # 不带命名空间
#   ./script/teleop_wasd.py --duration 10                    # 10 秒后自动停并退出

import argparse
import math
import os
import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy

HELP = """
---------------- SRM27 键盘控制 ----------------
  w / s   前进 / 后退        再按一次取消该分量
  a / d   左移 / 右移        再按一次取消该分量
  q / e   逆时针 / 顺时针自转  再按一次停止
  空格    全部停止
  h       打印本帮助
  Ctrl-C  退出（自动补发零速）
  平移速度固定 {speed} m/s（组合方向自动归一化），自转固定 {yaw} rad/s
------------------------------------------------
"""

# 速度看门狗: cmd_mux 导航输入 0.3 s、自转输入 0.1 s，20 Hz 足够安全
DEFAULT_RATE = 20.0


class TeleopWASD(Node):
    def __init__(self, namespace, speed, yaw_rate, rate):
        super().__init__("srm_teleop_wasd")
        self.speed = float(speed)
        self.yaw_rate = float(yaw_rate)
        self.forward = 0.0   # +1 前进 / -1 后退 / 0 停
        self.left = 0.0      # +1 左移 / -1 右移 / 0 停
        self.yaw = 0.0       # +1 逆时针 / -1 顺时针 / 0 停

        qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        prefix = f"/{namespace.strip('/')}" if namespace.strip("/") else ""
        self.nav_pub = self.create_publisher(Twist, f"{prefix}/cmd_vel_nav", qos)
        self.rotation_pub = self.create_publisher(Twist, f"{prefix}/rotation_cmd", qos)
        self.timer = self.create_timer(1.0 / float(rate), self.publish)
        self.get_logger().info(
            f"平移 -> {prefix}/cmd_vel_nav (linear.x/y)  "
            f"自转 -> {prefix}/rotation_cmd (angular.z)  @ {rate} Hz"
        )

    def command(self):
        """当前指令: (vx, vy, wz)。斜向按模长归一化到 speed。"""
        vx = self.forward * self.speed
        vy = self.left * self.speed
        norm = math.hypot(vx, vy)
        if norm > self.speed and norm > 0.0:
            scale = self.speed / norm
            vx *= scale
            vy *= scale
        wz = self.yaw * self.yaw_rate
        return vx, vy, wz

    def publish(self):
        vx, vy, wz = self.command()
        nav = Twist()
        nav.linear.x = vx
        nav.linear.y = vy
        self.nav_pub.publish(nav)
        rot = Twist()
        rot.angular.z = wz
        self.rotation_pub.publish(rot)

    def publish_zero(self):
        self.forward = self.left = self.yaw = 0.0
        self.publish()
        self.publish()

    def status(self):
        vx, vy, wz = self.command()
        return f"vx={vx:+.2f} vy={vy:+.2f} wz={wz:+.2f}  (m/s, rad/s)"

    def handle_key(self, key):
        if key == "w":
            self.forward = 0.0 if self.forward > 0 else 1.0
        elif key == "s":
            self.forward = 0.0 if self.forward < 0 else -1.0
        elif key == "a":
            self.left = 0.0 if self.left > 0 else 1.0
        elif key == "d":
            self.left = 0.0 if self.left < 0 else -1.0
        elif key == "q":
            self.yaw = 0.0 if self.yaw > 0 else 1.0
        elif key == "e":
            self.yaw = 0.0 if self.yaw < 0 else -1.0
        elif key == " ":
            self.forward = self.left = self.yaw = 0.0
        elif key == "h":
            print(HELP.format(speed=self.speed, yaw=self.yaw_rate), flush=True)
            return
        else:
            return
        print(f"\r{self.status():48s}", end="", flush=True)


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="WASD 全向平移 + QE 匀速自转的键盘控制器",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=HELP.format(speed=1.0, yaw=1.0),
    )
    parser.add_argument(
        "--namespace", default=os.environ.get("ROBOT_NS", "red_standard_robot1"),
        help="机器人命名空间，默认 red_standard_robot1；传空字符串表示根命名空间",
    )
    parser.add_argument("--speed", type=float, default=1.0, help="平移速度 m/s，默认 1.0")
    parser.add_argument("--yaw-rate", type=float, default=1.0, help="自转速度 rad/s，默认 1.0")
    parser.add_argument("--rate", type=float, default=DEFAULT_RATE, help="发布频率 Hz，默认 20")
    parser.add_argument("--duration", type=float, default=0.0, help="运行多少秒后自动停并退出（0=不限）")
    args = parser.parse_args(argv)

    rclpy.init()
    node = TeleopWASD(args.namespace, args.speed, args.yaw_rate, args.rate)

    stdin_fd = sys.stdin.fileno()
    is_tty = sys.stdin.isatty()
    saved_attrs = None
    if is_tty:
        saved_attrs = termios.tcgetattr(stdin_fd)
        tty.setcbreak(stdin_fd)  # 逐键读取，不需要回车，保留 Ctrl-C

    print(HELP.format(speed=node.speed, yaw=node.yaw_rate), end="")
    print("当前指令: " + node.status(), flush=True)

    start = node.get_clock().now()
    rc = 0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            ready, _, _ = select.select([sys.stdin], [], [], 0.02)
            if ready:
                data = os.read(stdin_fd, 1)
                if not data:  # EOF: 管道/重定向输入结束
                    break
                try:
                    node.handle_key(data.decode(errors="ignore").lower())
                except UnicodeDecodeError:
                    pass
            if args.duration > 0.0:
                elapsed = (node.get_clock().now() - start).nanoseconds / 1e9
                if elapsed >= args.duration:
                    print("\n[提示] 到达 --duration，自动停止。")
                    break
    except KeyboardInterrupt:
        pass
    finally:
        if is_tty and saved_attrs is not None:
            termios.tcsetattr(stdin_fd, termios.TCSADRAIN, saved_attrs)
        try:
            node.publish_zero()
        except Exception:  # noqa: BLE001 - 退出路径尽力而为
            pass
        print("\n已停止（补发零速）。")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
