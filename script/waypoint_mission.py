#!/usr/bin/env python3
# =============================================================
# SRM 航点任务 (自己的"行为树", 替代 RViz Nav2 面板的途经点模式)
#
# 为什么不用面板的航点模式:
#   RViz Nav2 面板按 Start 发的是 navigate_through_poses —— 一条路径串起所有航点,
#   BT 里 RemovePassedGoals(radius) 决定"车离多近才把航点从待走列表删掉"。车切内弯
#   擦过航点时删不掉它, 下一个 3 Hz 重规划又把路径绕回该点, 车就来回折返(实车见过
#   "Failed to make progress")。本脚本改用**逐个航点独立导航**:
#     waypoint[i] --NavigateToPose--> 成功才发 waypoint[i+1]
#   每个点只受 goal checker 的 xy_goal_tolerance 约束, 不存在"路径绕回旧航点"。
#
# 用法:
#   1) 点选航点再执行 (推荐, 边看 RViz 边排点):
#        ./script/waypoint_mission.py
#      在 RViz 工具栏用 "Publish Point" (发 /clicked_point) 或 "2D Goal Pose"
#      (发 /goal_pose) 点选航点, 脚本会收集并在 RViz 里画出来;
#      点够了再触发开始:
#        ros2 service call /waypoint_mission/start std_srvs/srv/Trigger
#
#      ⚠ 两个必须知道的坑:
#      a) Nav2 面板里那个 "Nav2 Goal"(rviz 插件 GoalTool) **不发 /goal_pose** ——
#         它直接下发 navigate_to_pose action, 所以本脚本收不到任何东西(点了像没反应),
#         而且车会立刻自己动起来。要点击选就只用工具栏的 Publish Point / 2D Goal Pose。
#      b) 本仓库的 rviz/nav2_default_view.rviz 原本只有 GoalTool, 没有上面两个工具;
#         已补上, 改完**重启 RViz** 即可(改 .rviz 不需要 colcon build)。
#         不想动 RViz 就从终端加点:
#           ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
#             "{header: {frame_id: map}, pose: {position: {x: 1.0, y: -1.0}, orientation: {w: 1.0}}}"
#      脚本启动后若一直没收到点选, 会自动打印以上排查提示。
#      另外: srm27_behavior 的 PubNav2Goal 也往 goal_pose 发 PoseStamped, 同时跑行为树
#            时它的目标会被本脚本当成航点记下来。
#
#   2) 直接执行文件里的航点:
#        ./script/waypoint_mission.py --file mission.yaml
#
#   3) 只收集并保存, 不下发 (教点模式):
#        ./script/waypoint_mission.py --save-file mission.yaml
#        # 点选完执行 ~/save 存盘
#
# 服务 (std_srvs/srv/Trigger):
#   ~/start  开始执行已收集的航点
#   ~/stop   取消当前目标并暂停任务 (航点列表保留, 可再 start 继续)
#   ~/clear  清空航点列表 (执行中调用会先停)
#   ~/save   把航点列表存到 --save-file
#
# 话题:
#   订阅 /goal_pose (geometry_msgs/PoseStamped, RViz "2D Goal Pose")
#   订阅 /clicked_point (geometry_msgs/PointStamped, RViz "Publish Point")
#   发布 ~/waypoints (visualization_msgs/MarkerArray, 航点可视化, latched)
#   发布 ~/current_target (geometry_msgs/PoseStamped, 当前目标)
#
# 失败策略 (每个航点独立判定):
#   --retry N        单个航点失败后的重试次数 (默认 1; 0 = 失败即按策略处理)
#   --on-failure     重试用尽后: abort(默认, 停下并保留现场) / skip(跳过该点继续)
#   --timeout SEC    单个航点超时秒数 (默认 0 = 不限时)
#   --pause SEC      航点之间停顿
#   --loop           全部完成后从头再来
#
# 其它参数:
#   --frame <frame>  航点统一转到该坐标系 (默认 map); 点击时的固定坐标系不同也能用
#   --accept-timeout SEC  发出目标后等"已接受"应答的秒数 (默认 8)。超时按失败处理并重试。
#       ⚠ 这个参数是防"静默卡死"的: 导航栈/脚本刚起来时, 服务端可能还没发现本节点的
#         应答端点, 于是 bt_navigator 报 "Failed to send goal response (timeout)",
#         客户端永远收不到应答 —— 表现就是终端只打印一行目标、之后什么都不发生,
#         RViz 里也没有路径(BT 从没开始导航, /plan 无数据)。
#   --send-settle SEC     等到服务端就绪后、发第一个目标前再等几秒 (默认 1), 规避上面的竞态
#   --record-only    只记录不下发 (等同不按 start)
#   --no-markers     不发 RViz 标记
#   --use-sim-time
#
# 说明:
#   - 本节点只发导航目标, 不直接发速度: 速度仍然走 Nav2 控制器 -> velocity_smoother
#     -> fake_vel_transform -> /cmd_vel_chassis 这条链路, 所以底盘限幅/看门狗照旧生效。
#   - Ctrl+C 会先取消当前目标再退出; 车若仍在动, 用 ./script/start_real_nav.sh --stop。
#   - 执行中再点选航点会追加到列表末尾, 下一轮(--loop)或下次 start 生效。
# =============================================================

import argparse
import math
import os
import sys

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

try:
    import tf2_ros
except ImportError:  # tf2_ros 一定随 ROS 装, 这里只是别让缺包直接崩
    tf2_ros = None

NODE_NAME = "waypoint_mission"

STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
    GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
    GoalStatus.STATUS_EXECUTING: "EXECUTING",
    GoalStatus.STATUS_CANCELING: "CANCELING",
    GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
    GoalStatus.STATUS_CANCELED: "CANCELED",
    GoalStatus.STATUS_ABORTED: "ABORTED",
}

# 状态机的状态 (这个小状态机就是"行为树"的全部)
IDLE = "IDLE"        # 空闲: 收集航点 / 等待 start
SEND = "SEND"        # 已下发当前航点, 等目标被接受
RUNNING = "RUNNING"  # 目标执行中
PAUSE = "PAUSE"      # 航点之间的停顿
DONE = "DONE"        # 全部完成


def yaw_to_quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def quaternion_to_yaw(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class WaypointMission(Node):
    def __init__(self, args):
        super().__init__(NODE_NAME)
        self.args = args
        self.frame_id = args.frame
        self.use_sim_time = bool(args.use_sim_time)
        if self.use_sim_time:
            self.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])

        self.waypoints = []          # list[PoseStamped]
        self.state = IDLE
        self.index = 0
        self.attempts = 0
        self.pause_until = 0.0
        self.mission_running = False
        self.goal_future = None      # send_goal_async 的 future
        self.goal_handle = None
        self.result_future = None
        self.goal_started_at = 0.0
        self.last_feedback_log = 0.0
        self.last_distance = float("nan")
        self.recoveries = 0
        self.tf_ok = False

        # ---- 导航 action 客户端 ----
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # ---- TF: 把点选结果统一到 --frame (RViz 的固定坐标系未必是 map) ----
        self.tf_buffer = None
        self.tf_listener = None
        if tf2_ros is not None:
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---- 订阅: RViz 两种点选工具各发一个话题 ----
        self.create_subscription(PoseStamped, "/goal_pose", self.on_goal_pose, 10)
        self.create_subscription(PointStamped, "/clicked_point", self.on_clicked_point, 10)

        # ---- 发布: 航点标记 + 当前目标 ----
        marker_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_pub = self.create_publisher(MarkerArray, "~/waypoints", marker_qos)
        self.target_pub = self.create_publisher(PoseStamped, "~/current_target", 10)

        # ---- 服务 ----
        self.create_service(Trigger, "~/start", self.on_start)
        self.create_service(Trigger, "~/stop", self.on_stop)
        self.create_service(Trigger, "~/clear", self.on_clear)
        self.create_service(Trigger, "~/save", self.on_save)

        # ---- 状态机心跳 ----
        self.create_timer(0.1, self.tick)

        # ---- "点了没反应"自诊断 ----
        # 最常见的坑: RViz 里点的是 Nav2 面板的 "Nav2 Goal"(GoalTool), 它直接下发
        # navigate_to_pose action, 不经过 /goal_pose, 本脚本一个字都收不到。
        self.last_click_hint = 0.0
        self.create_timer(5.0, self.check_click_sources)

        self.get_logger().info(
            "航点任务节点就绪。RViz 用 '2D Goal Pose' / 'Publish Point' 点选航点; "
            f"开始: ros2 service call /{NODE_NAME}/start std_srvs/srv/Trigger"
        )

        if args.file:
            self.load_file(args.file)
            if self.waypoints and not args.record_only:
                self.get_logger().info(f"从 {args.file} 载入 {len(self.waypoints)} 个航点, 直接开始。")
                self.start_mission()
            elif self.waypoints:
                self.get_logger().info(
                    f"从 {args.file} 载入 {len(self.waypoints)} 个航点 (--record-only, 不下发)。"
                )

    # ---------------- "点了没反应"自诊断 ----------------
    def check_click_sources(self):
        """一个航点都没收到、而且两个点选话题上都没有发布者时, 给出可操作的提示。"""
        if self.waypoints or self.state in (SEND, RUNNING, PAUSE):
            return
        if self.count_publishers("/goal_pose") or self.count_publishers("/clicked_point"):
            return
        now = self.now_sec()
        if self.last_click_hint and now - self.last_click_hint < 60.0:
            return
        self.last_click_hint = now
        self.get_logger().warn(
            "还没收到任何点选, 且 /goal_pose、/clicked_point 上都没有发布者。常见原因:\n"
            "  1) 用的是 Nav2 面板里的 'Nav2 Goal' —— 它直接下发 navigate_to_pose action, "
            "不发 /goal_pose, 本脚本收不到; 请改用工具栏的 'Publish Point' 或 '2D Goal Pose'。\n"
            "  2) RViz 工具栏里没有这两个工具 —— 本仓库的 rviz/nav2_default_view.rviz 已补上, "
            "改完需要**重启 RViz**(改 .rviz 不用 colcon build)。\n"
            "  3) 懒得动 RViz 也可以直接从终端加点:\n"
            "     ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "
            "\"{header: {frame_id: map}, pose: {position: {x: 1.0, y: -1.0}, orientation: {w: 1.0}}}\"\n"
            "  4) 若同时跑着 srm27_behavior 的行为树, 它的 PubNav2Goal 也发 goal_pose, "
            "那些目标会被本脚本当成航点记下来。"
        )

    # ---------------- 点选 / 航点收集 ----------------
    def on_goal_pose(self, msg: PoseStamped):
        self.add_pose(msg)

    def on_clicked_point(self, msg: PointStamped):
        # "Publish Point" 只给位置, 朝向沿用上一次的(或 0), 主要用于先布点、后调朝向
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position = msg.point
        yaw = quaternion_to_yaw(self.waypoints[-1].pose.orientation) if self.waypoints else 0.0
        pose.pose.orientation.x, pose.pose.orientation.y, \
            pose.pose.orientation.z, pose.pose.orientation.w = yaw_to_quaternion(yaw)
        self.add_pose(pose)

    def add_pose(self, pose: PoseStamped):
        pose = self.to_frame(pose)
        if pose is None:
            return
        self.waypoints.append(pose)
        yaw = quaternion_to_yaw(pose.pose.orientation)
        self.get_logger().info(
            f"记下航点 #{len(self.waypoints)}: "
            f"({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f}, yaw {yaw:.2f}) "
            f"[{pose.header.frame_id}]  共 {len(self.waypoints)} 个"
        )
        self.publish_markers()

    def to_frame(self, pose: PoseStamped):
        """把航点转到 self.frame_id; 转不动就丢弃并告警(而不是默默用错坐标系)。"""
        src = pose.header.frame_id
        if src == self.frame_id or self.tf_buffer is None:
            pose.header.frame_id = self.frame_id
            return pose
        try:
            return self.tf_buffer.transform(pose, self.frame_id, timeout=Duration(seconds=0.5))
        except Exception as ex:  # tf2 各种异常的基类不统一, 这里统一兜住
            self.get_logger().warn(
                f"航点坐标系 {src} 无法转到 {self.frame_id} ({ex}); 已丢弃该点。"
                f"检查 RViz 的 Fixed Frame, 或用 --frame {src} 重跑。"
            )
            return None

    # ---------------- 服务 ----------------
    def on_start(self, request, response):
        if not self.waypoints:
            response.success = False
            response.message = "还没有航点: 先在 RViz 点选 (2D Goal Pose / Publish Point)。"
            return response
        if self.args.record_only:
            response.success = False
            response.message = "--record-only 模式下不下发目标。"
            return response
        self.start_mission()
        response.success = True
        response.message = f"开始执行 {len(self.waypoints)} 个航点。"
        return response

    def on_stop(self, request, response):
        was = self.state
        self.cancel_current("用户调用 ~/stop")
        self.mission_running = False
        self.state = IDLE
        response.success = True
        response.message = f"已停止 (原状态 {was}); 航点列表保留 {len(self.waypoints)} 个, 可再 start。"
        return response

    def on_clear(self, request, response):
        self.cancel_current("清空航点前先停")
        self.mission_running = False
        self.state = IDLE
        count = len(self.waypoints)
        self.waypoints = []
        self.index = 0
        self.publish_markers()
        response.success = True
        response.message = f"已清空 {count} 个航点。"
        return response

    def on_save(self, request, response):
        path = self.args.save_file
        if not path:
            response.success = False
            response.message = "没有指定保存路径: 用 --save-file <yaml> 启动。"
            return response
        if not self.waypoints:
            response.success = False
            response.message = "航点列表为空, 没什么可存。"
            return response
        try:
            self.save_file(path)
        except OSError as ex:
            response.success = False
            response.message = f"写文件失败: {ex}"
            return response
        response.success = True
        response.message = f"已保存 {len(self.waypoints)} 个航点到 {path}。"
        return response

    # ---------------- 航点文件 ----------------
    def load_file(self, path):
        try:
            with open(path, encoding="utf-8") as stream:
                data = yaml.safe_load(stream) or {}
        except OSError as ex:
            self.get_logger().error(f"读航点文件失败: {ex}")
            return
        frame = data.get("frame_id", self.frame_id)
        items = data.get("waypoints", [])
        if not isinstance(items, list):
            self.get_logger().error(f"{path} 里 waypoints 不是列表。")
            return
        for item in items:
            try:
                pose = PoseStamped()
                pose.header.frame_id = frame
                pose.pose.position.x = float(item["x"])
                pose.pose.position.y = float(item["y"])
                yaw = float(item.get("yaw", 0.0))
                pose.pose.orientation.x, pose.pose.orientation.y, \
                    pose.pose.orientation.z, pose.pose.orientation.w = yaw_to_quaternion(yaw)
            except (KeyError, TypeError, ValueError) as ex:
                self.get_logger().warn(f"跳过一条无效航点 {item!r}: {ex}")
                continue
            pose = self.to_frame(pose)
            if pose is not None:
                self.waypoints.append(pose)
        # 逐条打出来: 教点之后用 --record-only 载入核对时, 光看个数没法确认对不对
        for i, pose in enumerate(self.waypoints):
            self.get_logger().info(
                f"  #{i + 1}: ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f}, "
                f"yaw {quaternion_to_yaw(pose.pose.orientation):.2f})"
            )
        self.publish_markers()

    def save_file(self, path):
        data = {
            "frame_id": self.frame_id,
            "waypoints": [
                {
                    "x": round(pose.pose.position.x, 3),
                    "y": round(pose.pose.position.y, 3),
                    "yaw": round(quaternion_to_yaw(pose.pose.orientation), 3),
                }
                for pose in self.waypoints
            ],
        }
        directory = os.path.dirname(os.path.abspath(path))
        if directory:
            os.makedirs(directory, exist_ok=True)
        with open(path, "w", encoding="utf-8") as stream:
            yaml.safe_dump(data, stream, allow_unicode=True, sort_keys=False)
        self.get_logger().info(f"已保存 {len(self.waypoints)} 个航点到 {path}")
        self.get_logger().info(f"  下次执行: ./script/start_waypoints.sh --file {path}")
        self.get_logger().info(f"  只核对不下发: ./script/start_waypoints.sh --file {path} --record-only")

    # ---------------- 任务控制 ----------------
    def start_mission(self):
        if not self.nav_client.wait_for_server(timeout_sec=self.args.server_timeout):
            self.get_logger().error(
                "等不到 navigate_to_pose action 服务: 导航栈没起来?"
                "(bt_navigator 在跑吗: ros2 node list | grep bt_navigator)"
            )
            return
        self.index = 0
        self.attempts = 0
        self.mission_running = True
        self.get_logger().info(
            f"任务开始: {len(self.waypoints)} 个航点, retry={self.args.retry}, "
            f"on-failure={self.args.on_failure}, "
            f"timeout={self.args.timeout or '不限'}, loop={self.args.loop}"
        )
        # 不要立刻发: wait_for_server 只保证"本节点发现了服务端", 反方向
        # (服务端发现本节点的应答端点) 可能还没完成。这段窗口里发目标会出现
        # bt_navigator 报 "Failed to send goal response (timeout)", 客户端则永远
        # 收不到"已接受"应答 —— 老版本就在这里静默卡死。等一小会儿再发。
        if self.args.send_settle > 0:
            self.state = PAUSE
            self.pause_until = self.now_sec() + self.args.send_settle
            return
        self.send_current()

    def send_current(self):
        if self.index >= len(self.waypoints):
            self.finish_mission()
            return
        pose = self.waypoints[self.index]
        goal = NavigateToPose.Goal()
        goal.pose = pose
        yaw = quaternion_to_yaw(pose.pose.orientation)
        self.attempts += 1
        attempt_note = f" (第 {self.attempts} 次尝试)" if self.attempts > 1 else ""
        self.get_logger().info(
            f"[{self.index + 1}/{len(self.waypoints)}] 目标 "
            f"({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f}, yaw {yaw:.2f}){attempt_note}"
        )
        self.goal_started_at = self.now_sec()
        self.last_feedback_log = 0.0
        self.goal_future = self.nav_client.send_goal_async(
            goal, feedback_callback=self.on_feedback
        )
        self.goal_handle = None
        self.result_future = None
        self.state = SEND
        self.publish_markers()
        self.target_pub.publish(pose)

    def on_feedback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.last_distance = getattr(feedback, "distance_remaining", float("nan"))
        self.recoveries = getattr(feedback, "number_of_recoveries", 0)
        now = self.now_sec()
        # 刚下发时 nav2 会先发一帧 distance_remaining=0 的反馈, 直接打出来会误导,
        # 所以目标开始 1 秒后才记录, 之后每 2 秒一条。
        if now - self.goal_started_at >= 1.0 and now - self.last_feedback_log >= 2.0:
            self.last_feedback_log = now
            self.get_logger().info(
                f"  剩余 {self.last_distance:.2f} m, 已用 {now - self.goal_started_at:.1f} s, "
                f"恢复次数 {self.recoveries}"
            )

    def cancel_current(self, reason):
        handle = self.goal_handle
        future = self.goal_future
        if handle is not None:
            self.get_logger().warn(f"取消当前目标 ({reason})")
            handle.cancel_goal_async()
        elif future is not None:
            # 目标刚发出、还没被接受: 挂个回调, 一旦被接受立刻取消。
            # 否则"停止"之后车可能又跑起来(状态机已经不看这个目标了)。
            def _cancel_late(done_future, why=reason):
                try:
                    late_handle = done_future.result()
                except Exception:
                    return
                if late_handle is not None and late_handle.accepted:
                    self.get_logger().warn(f"取消刚被接受的目标 ({why})")
                    late_handle.cancel_goal_async()

            future.add_done_callback(_cancel_late)
        self.goal_handle = None
        self.goal_future = None
        self.result_future = None

    def finish_mission(self):
        self.state = DONE
        self.mission_running = False
        if self.args.loop:
            self.get_logger().info("一轮完成, --loop 开启, 重新开始。")
            self.index = 0
            self.attempts = 0
            self.mission_running = True
            self.state = IDLE
            self.pause_until = self.now_sec() + max(self.args.pause, 0.5)
            return
        self.get_logger().info(f"全部 {len(self.waypoints)} 个航点执行完毕。")

    def on_waypoint_failed(self, status_name):
        remaining_retries = self.args.retry - (self.attempts - 1)
        if remaining_retries > 0:
            self.get_logger().warn(
                f"航点 #{self.index + 1} {status_name}; 还剩 {remaining_retries} 次重试, 重新下发。"
            )
            self.pause_until = self.now_sec() + 1.0
            self.state = PAUSE
            return
        if self.args.on_failure == "skip":
            self.get_logger().warn(f"航点 #{self.index + 1} {status_name}; 按 --on-failure skip 跳过。")
            self.advance_waypoint()
            return
        self.get_logger().error(
            f"航点 #{self.index + 1} {status_name}; 按 --on-failure abort 停止任务。"
            f"现场保留: 可用 ~/start 重跑, 或用 ~/clear 清空。"
        )
        self.state = IDLE
        self.mission_running = False

    def advance_waypoint(self):
        self.index += 1
        self.attempts = 0
        if self.args.pause > 0 and self.index < len(self.waypoints):
            self.pause_until = self.now_sec() + self.args.pause
            self.state = PAUSE
        else:
            self.send_current()

    # ---------------- 状态机 ----------------
    def now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def tick(self):
        if self.state == PAUSE:
            if self.now_sec() >= self.pause_until:
                # index 由上一处逻辑决定: 重试时 index 未变(重发同一点),
                # 成功后 index 已 +1(发下一个点)。两种情况都走 send_current。
                self.send_current()
            return

        if self.state == SEND and self.goal_future is not None:
            if not self.goal_future.done():
                # 目标发出去却一直没等到"已接受/被拒绝"的应答: 通常是 DDS 端点匹配竞态
                # (nav2 日志里那句 "Failed to send goal response (timeout)")。
                # 这里必须自己超时, 否则会永远卡在 SEND, 表现就是"终端只打印了一行目标,
                # 之后什么都不发生, RViz 里也没有路径"。
                if self.args.accept_timeout > 0 and \
                        self.now_sec() - self.goal_started_at > self.args.accept_timeout:
                    self.get_logger().error(
                        f"目标发出 {self.args.accept_timeout:.1f} s 没收到接受应答, 放弃本次尝试。"
                        "常见原因: 导航栈刚起/脚本刚起时的 DDS 端点匹配竞态 —— "
                        "nav2 日志里会有 'Failed to send goal response (timeout)'。"
                        "重试一般就能成功(下面按失败策略处理)。"
                    )
                    self.cancel_current("目标应答超时")
                    self.on_waypoint_failed("NO_RESPONSE")
                return
            goal_handle = self.goal_future.result()
            self.goal_future = None
            if goal_handle is None or not goal_handle.accepted:
                self.get_logger().error("目标被拒绝 (action server 没接受, 常见于 BT 加载失败)。")
                self.on_waypoint_failed("REJECTED")
                return
            self.goal_handle = goal_handle
            self.result_future = goal_handle.get_result_async()
            self.state = RUNNING
            return

        if self.state == RUNNING:
            if self.args.timeout > 0 and self.now_sec() - self.goal_started_at > self.args.timeout:
                self.get_logger().warn(f"航点 #{self.index + 1} 超时 {self.args.timeout} s。")
                self.cancel_current("超时")
                self.on_waypoint_failed("TIMEOUT")
                return
            if self.result_future is None or not self.result_future.done():
                return
            result = self.result_future.result()
            self.result_future = None
            self.goal_handle = None
            status = result.status
            status_name = STATUS_NAMES.get(status, str(status))
            elapsed = self.now_sec() - self.goal_started_at
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info(
                    f"[{self.index + 1}/{len(self.waypoints)}] 到达 "
                    f"(用时 {elapsed:.1f} s, 恢复 {self.recoveries} 次)"
                )
                self.advance_waypoint()
            else:
                self.on_waypoint_failed(status_name)
            return

        # IDLE / DONE: --loop 且任务在跑时, 停顿结束就重发第一个点
        if self.state == IDLE and self.mission_running and self.waypoints:
            if self.now_sec() >= self.pause_until:
                self.send_current()

    # ---------------- RViz 可视化 ----------------
    def publish_markers(self):
        if self.args.no_markers:
            return
        array = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for i, pose in enumerate(self.waypoints):
            is_current = (self.state in (SEND, RUNNING, PAUSE)) and i == self.index
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = stamp
            marker.ns = "waypoints"
            marker.id = i
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose = pose.pose
            marker.scale.x = 0.35
            marker.scale.y = 0.06
            marker.scale.z = 0.06
            marker.color.r = 1.0 if is_current else 0.2
            marker.color.g = 0.2
            marker.color.b = 1.0 if not is_current else 0.2
            marker.color.a = 1.0 if is_current else 0.7
            array.markers.append(marker)

            label = Marker()
            label.header.frame_id = self.frame_id
            label.header.stamp = stamp
            label.ns = "labels"
            label.id = i
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose = pose.pose
            label.pose.position.z += 0.4
            label.scale.z = 0.25
            label.color.r = label.color.g = label.color.b = 1.0
            label.color.a = 1.0
            label.text = f"#{i + 1}"
            array.markers.append(label)

        # 用 DELETEALL 清掉已删除的航点残留
        clear = Marker()
        clear.action = Marker.DELETEALL
        array.markers.insert(0, clear)
        self.marker_pub.publish(array)


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="SRM 航点任务: RViz 点选航点, 逐个下发 NavigateToPose (替代面板途经点模式)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--file", "-f", help="航点 YAML 文件; 给了就直接执行里面的航点")
    parser.add_argument("--save-file", help="~/save 服务的保存路径")
    parser.add_argument("--frame", default="map", help="航点统一坐标系 (默认 map)")
    parser.add_argument("--retry", type=int, default=1, help="单个航点失败后的重试次数 (默认 1)")
    parser.add_argument("--on-failure", choices=("abort", "skip"), default="abort",
                        help="重试用尽后: abort 停止(默认) / skip 跳过")
    parser.add_argument("--timeout", type=float, default=0.0, help="单个航点超时秒数 (默认 0 = 不限)")
    parser.add_argument("--pause", type=float, default=0.0, help="航点之间停顿秒数")
    parser.add_argument("--loop", action="store_true", help="全部完成后从头再来")
    parser.add_argument("--record-only", action="store_true", help="只记录航点, 不下发")
    parser.add_argument("--no-markers", action="store_true", help="不发 RViz 航点标记")
    parser.add_argument("--server-timeout", type=float, default=10.0,
                        help="等 navigate_to_pose 服务的秒数 (默认 10)")
    parser.add_argument("--accept-timeout", type=float, default=8.0,
                        help="发出目标后等'已接受'应答的秒数, 超时按失败重试 (默认 8; 0 = 不限, 会静默卡死)")
    parser.add_argument("--send-settle", type=float, default=1.0,
                        help="等到服务后就绪后、发第一个目标前的等待秒数 (默认 1; 规避 DDS 端点匹配竞态)")
    parser.add_argument("--use-sim-time", action="store_true", help="使用 /clock (回放 rosbag)")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    if args.retry < 0:
        print("[错误] --retry 不能是负数", file=sys.stderr)
        return 2
    rclpy.init()
    node = WaypointMission(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl+C: rclpy 收到 SIGINT 后会让 spin 抛 ExternalShutdownException,
        # 这里一起接住, 免得打印一堆堆栈。
        pass
    finally:
        # Ctrl+C 时先撤销当前目标, 避免车继续跑最后一段
        try:
            node.cancel_current("退出")
        except Exception:
            pass
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
