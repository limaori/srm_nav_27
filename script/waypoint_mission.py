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
#   ⚠ 代价: 逐点导航 = 每个航点都要停车 —— goal checker 必须等车在 0.4 m 内停稳
#     才算"到达", 到了再发下一个, 车在每个点都会刹一下。想要"除了最后一个点, 其它
#     都是途径点、途径点不刹车"就用 --pass-through (见下面"途径点模式")。
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
#   4) 途径点模式 (除最后一个点外都当途径点, 途径点不刹车):
#        ./script/waypoint_mission.py --file mission.yaml --pass-through
#
# 途径点模式 (--pass-through) 怎么实现的、要注意什么:
#   - 整个列表一次交给 nav2 的 navigate_through_poses: 前 N-1 个点是途径点, 第 N 个
#     是终点。全局路径一次规划穿过所有点, 控制器只在**路径末端(终点)**做到达判定和
#     减速(OmniPidPursuitController 的 approach_velocity_scaling_dist 也是按路径末点
#     算的), 所以途径点不会停车, 到终点才停。终点精度与逐点模式相同(都用
#     general_goal_checker: xy_goal_tolerance 0.4 m)。
#   - 用的是 bt_navigator 的 default_nav_through_poses_bt_xml:
#       behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml
#     里面 RemovePassedGoals(radius="0.35") 负责"开过哪个途径点就把它从目标列表里删掉",
#     不删的话 3 Hz 重规划会把路径绕回已开过的点 → 车折返 / "Failed to make progress"。
#     这个 radius 和 general_goal_checker.xy_goal_tolerance(0.4)、控制器前瞻
#     (min/max_lookahead_dist 0.3/0.6)是一组参数, 要调一起调 —— 详见 xml 里的注释。
#     本脚本不发速度, 所以"不刹车"完全是这条链路(控制器→velocity_smoother)决定的。
#   - 失败处理: 整段失败后按 --retry 重发"还没开过的剩下的点"(已经开过的点不会再被
#     当成目标), retry 用完再按 --on-failure abort(停下保留现场)/skip(丢掉当前途径点
#     继续)。只剩终点一个点时自动退回 NavigateToPose, 保证终点精确到达。
#   - "已经开过哪些点"由本脚本自己用 TF 跟踪(默认 map <- base_link), 只用于失败后
#     重发剩余点和 RViz 高亮, **不参与到达判定**; 判定半径是 --passed-radius(默认 0.8 m)。
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
# 失败策略 (每个航点 / 每段途径点航段独立判定):
#   --retry N        失败后的重试次数 (默认 1; 0 = 失败即按策略处理)。
#                    --pass-through 时重发的是"还没开过的剩余点", 不是从头再来。
#   --on-failure     重试用尽后: abort(默认, 停下并保留现场) / skip(跳过该点继续;
#                    --pass-through 时丢掉车当前正要去的那一个途径点)
#   --timeout SEC    单个航点/整段途径点超时秒数 (默认 0 = 不限时)
#   --pause SEC      航点之间停顿
#   --loop           全部完成后从头再来
#
# 其它参数:
#   --pass-through   途径点模式: 除最后一个点(终点)外都当途径点, 途径点不刹车 (见上)
#   --passed-radius  途径点模式下"这个点已经开过"的判定半径 (默认 0.8 m)
#   --base-frame     判定"已开过"时用的机器人本体坐标系 (默认 base_link)
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
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
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
        self.index = 0               # 逐点模式: 当前目标下标; 途径点模式: 还没开过的第一个点
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
        # 当前在跑的这一段是逐点(NavigateToPose)还是途径点(NavigateThroughPoses)
        self.active_mode = "single"

        # ---- 导航 action 客户端 ----
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        # 途径点模式用: 一次穿过一串点, 只有最后一个点(终点)会停车
        self.through_client = ActionClient(self, NavigateThroughPoses, "navigate_through_poses")

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
        if args.pass_through:
            self.get_logger().info(
                "途径点模式 (--pass-through): 除最后一个点(终点)外都是途径点, 途径点不刹车; "
                "终点精度与逐点模式相同 (xy_goal_tolerance)。"
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
        use_through = self.args.pass_through and len(self.waypoints) >= 2
        if use_through and not self.through_client.wait_for_server(
                timeout_sec=self.args.server_timeout):
            self.get_logger().error(
                "等不到 navigate_through_poses action 服务。--pass-through 需要 bt_navigator "
                "带 navigate_through_poses 这个 navigator(默认就有)。检查:\n"
                "  ros2 action list | grep through_poses\n"
                "  以及 params 里的 navigator_plugins / "
                "default_nav_through_poses_bt_xml (用仓库默认的 config/real/nav2_params_srm.yaml)"
            )
            return
        self.index = 0
        self.attempts = 0
        self.mission_running = True
        mode_note = "途径点(只有终点停车)" if use_through else "逐点停车"
        self.get_logger().info(
            f"任务开始: {len(self.waypoints)} 个航点, 模式={mode_note}, retry={self.args.retry}, "
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
        """下发"当前该走的目标"。
        --pass-through: 只要还剩 >=2 个点, 就把剩下的整条当途径点一次下发(途径点不刹车);
        只剩终点(或没开 --pass-through)时就退回逐点 NavigateToPose。"""
        if self.index >= len(self.waypoints):
            self.finish_mission()
            return
        remaining = self.waypoints[self.index:]
        if self.args.pass_through and len(remaining) >= 2:
            self.send_through_poses(remaining)
            return
        self.send_to_pose(remaining[0])

    def send_through_poses(self, poses):
        """一次穿过 poses[0..n-2](途径点)到 poses[-1](终点): 只有终点会停车。"""
        goal = NavigateThroughPoses.Goal()
        goal.poses = poses
        last = poses[-1]
        last_yaw = quaternion_to_yaw(last.pose.orientation)
        self.attempts += 1
        attempt_note = f" (第 {self.attempts} 次尝试)" if self.attempts > 1 else ""
        self.get_logger().info(
            f"[{self.index + 1}..{len(self.waypoints)}/{len(self.waypoints)}] 途径点模式: "
            f"{len(poses) - 1} 个途径点(不刹车) → 终点 "
            f"({last.pose.position.x:.2f}, {last.pose.position.y:.2f}, yaw {last_yaw:.2f})"
            f"{attempt_note}"
        )
        self.active_mode = "through"
        self.goal_started_at = self.now_sec()
        self.last_feedback_log = 0.0
        self.goal_future = self.through_client.send_goal_async(
            goal, feedback_callback=self.on_feedback
        )
        self.goal_handle = None
        self.result_future = None
        self.state = SEND
        self.publish_markers()
        self.target_pub.publish(last)

    def send_to_pose(self, pose):
        goal = NavigateToPose.Goal()
        goal.pose = pose
        yaw = quaternion_to_yaw(pose.pose.orientation)
        self.attempts += 1
        attempt_note = f" (第 {self.attempts} 次尝试)" if self.attempts > 1 else ""
        self.get_logger().info(
            f"[{self.index + 1}/{len(self.waypoints)}] 目标 "
            f"({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f}, yaw {yaw:.2f}){attempt_note}"
        )
        self.active_mode = "single"
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
            # 途径点模式的 distance_remaining 是"到终点还剩多少"; 反馈里的
            # number_of_poses_remaining 各家版本算法不一致, 这里只信自己 TF 跟踪的进度。
            progress = (
                f"已过 {self.index}/{len(self.waypoints)} 个点, "
                if self.active_mode == "through" else ""
            )
            self.get_logger().info(
                f"  剩余 {self.last_distance:.2f} m, {progress}"
                f"已用 {now - self.goal_started_at:.1f} s, 恢复次数 {self.recoveries}"
            )

    def robot_xy(self):
        """TF 里机器人当前位置; 拿不到返回 None。只给途径点模式的进度跟踪用。"""
        if self.tf_buffer is None:
            return None
        try:
            tf = self.tf_buffer.lookup_transform(self.frame_id, self.args.base_frame, Time())
        except Exception:  # tf2 各种异常基类不统一, 统一兜住: 拿不到就当没进度
            return None
        self.tf_ok = True
        return tf.transform.translation.x, tf.transform.translation.y

    def update_passed_index(self):
        """--pass-through 专用: 车离当前途径点 < --passed-radius 就认为这个点已开过, index 前移。

        只影响两件事: 失败后重发哪些剩余点、RViz 高亮哪个点。**到达判定/是否停车完全由
        nav2 决定**(控制器只在路径末点做判定), 这里不参与, 所以判早了也不会让车少走。"""
        if not self.args.pass_through or self.index >= len(self.waypoints) - 1:
            return
        xy = self.robot_xy()
        if xy is None:
            return
        moved = False
        while self.index < len(self.waypoints) - 1:
            wp = self.waypoints[self.index].pose.position
            if math.hypot(xy[0] - wp.x, xy[1] - wp.y) > self.args.passed_radius:
                break
            self.get_logger().info(
                f"  途径点 #{self.index + 1} 已开过(距 {self.args.passed_radius:.2f} m 以内), "
                f"下一个是 #{self.index + 2}"
            )
            self.index += 1
            moved = True
        if moved:
            self.publish_markers()

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
        target = (
            f"途径点航段 (从 #{self.index + 1} 到终点)" if self.active_mode == "through"
            else f"航点 #{self.index + 1}"
        )
        if remaining_retries > 0:
            if self.active_mode == "through" and not self.tf_ok:
                self.get_logger().warn(
                    "拿不到机器人位姿 TF, 没法确认哪些途径点已经开过 —— 这次重试会从 "
                    f"#{self.index + 1} 重发, 已经开过的点可能又被当成目标。检查 "
                    "--frame/--base-frame 是否和实际一致。"
                )
            self.get_logger().warn(
                f"{target} {status_name}; 还剩 {remaining_retries} 次重试, 下发剩余航点。"
            )
            self.pause_until = self.now_sec() + 1.0
            self.state = PAUSE
            return
        if self.args.on_failure == "skip":
            if self.active_mode == "through":
                self.get_logger().warn(
                    f"{target} {status_name}; 按 --on-failure skip 丢掉途经点 "
                    f"#{self.index + 1}, 从下一个点继续。"
                )
            else:
                self.get_logger().warn(f"航点 #{self.index + 1} {status_name}; 按 --on-failure skip 跳过。")
            if self.index >= len(self.waypoints) - 1:
                self.get_logger().warn(
                    "  注意: 丢掉的已经是最后一个点(终点), 跳过它 = 终点不会精确到达。"
                )
            self.advance_waypoint()
            return
        self.get_logger().error(
            f"{target} {status_name}; 按 --on-failure abort 停止任务。"
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
        # 途径点模式: 边跑边用 TF 记下"哪些途径点已经开过了"(失败重发/高亮用, 见函数注释)
        if self.state in (SEND, RUNNING):
            self.update_passed_index()

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
                what = "途径点航段" if self.active_mode == "through" else f"航点 #{self.index + 1}"
                self.get_logger().warn(f"{what} 超时 {self.args.timeout} s。")
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
                if self.active_mode == "through":
                    # through_poses 只在最后一个点(路径末点)做到达判定, 所以它成功 =
                    # 车已经在终点停稳, 整条航点跑完, 不需要再逐点走。
                    self.get_logger().info(
                        f"[终点 {len(self.waypoints)}/{len(self.waypoints)}] 到达, "
                        f"途径点全程未停车 (用时 {elapsed:.1f} s, 恢复 {self.recoveries} 次)"
                    )
                    self.index = len(self.waypoints)
                    self.finish_mission()
                else:
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
            # 用 ASCII "GOAL": RViz 的字体没有中文字形, 中文会变成方块/乱码
            label.text = f"#{i + 1}"
            if self.args.pass_through and i == len(self.waypoints) - 1:
                label.text = f"#{i + 1} GOAL"
            array.markers.append(label)

        # 用 DELETEALL 清掉已删除的航点残留
        clear = Marker()
        clear.action = Marker.DELETEALL
        array.markers.insert(0, clear)
        self.marker_pub.publish(array)


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="SRM 航点任务: RViz 点选航点, 逐个下发 NavigateToPose "
                    "(--pass-through: 除终点外都当途径点, 途径点不刹车)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--file", "-f", help="航点 YAML 文件; 给了就直接执行里面的航点")
    parser.add_argument("--save-file", help="~/save 服务的保存路径")
    parser.add_argument("--frame", default="map", help="航点统一坐标系 (默认 map)")
    parser.add_argument("--pass-through", action="store_true",
                        help="途径点模式: 除最后一个点(终点)外都当途径点, 一次 "
                             "navigate_through_poses 下发, 途径点不刹车 (默认关闭: 每个点都停车)")
    parser.add_argument("--passed-radius", type=float, default=0.8,
                        help="途径点模式下判定'这个点已经开过'的距离 (默认 0.8 m); "
                             "只用于失败后重发剩余点与 RViz 高亮, 不参与到达判定")
    parser.add_argument("--base-frame", default="base_link",
                        help="判定'已开过'时用的机器人本体坐标系 (默认 base_link)")
    parser.add_argument("--retry", type=int, default=1,
                        help="失败后的重试次数 (默认 1); --pass-through 时重发的是剩余点")
    parser.add_argument("--on-failure", choices=("abort", "skip"), default="abort",
                        help="重试用尽后: abort 停止(默认) / skip 跳过(途径点模式丢掉当前途径点)")
    parser.add_argument("--timeout", type=float, default=0.0,
                        help="单个航点/整段途径点超时秒数 (默认 0 = 不限)")
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
    if args.pass_through and args.passed_radius <= 0:
        # radius <= 0 会让"已开过"的进度永远不前进, 失败重试就变成从头再来(会把车引回去)
        print("[错误] --pass-through 需要 --passed-radius > 0", file=sys.stderr)
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
