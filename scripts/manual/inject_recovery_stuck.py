#!/usr/bin/env python3
# Copyright 2026 SRM
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""确定性复现 MINCO 脱困：把障碍"注入"到机器人身上，逼出卡死条件。

为什么需要这个脚本
==================
脱困要**两个条件同时成立**（见 docs/minco脱困机制迁移实施记录(ai).md §3.4）：

  条件 1  重规划连续失败 >= recovery.fail_threshold 次
  条件 2  机器人**自身**净空 < recovery.stuck_clearance（默认 0.25 m）

条件 2 的净空是相对**原始致命栅格**算的，膨胀层不计入。而代价地图的膨胀层
（`inflation_radius`）本来就不让车靠近到 0.25 m 以内，所以"正常开车"几乎永远
碰不到条件 2 —— 手开车等它自己卡死会白等很久。本脚本直接把致命的障碍栅格
放到车身上，一步到位造出"前方被挡 + 自身净空 ~0"的真实场景。

它做什么
========
1. 订阅当前 `/map`（map_server / SLAM 发的那份，transient_local），拿到栅格图；
2. 在**机器人当前位置**周围画一小块致命障碍（形状由 --shape 决定）；
3. 用 transient_local 重新发布 `/map`，代价地图的 static_layer 下一个更新周期
   （update_frequency，仿真 10 Hz）就会把它并进来：
       * 全局代价地图  -> 规划失败（条件 1）
       * 机器人自身净空 -> 约 0（条件 2）
4. 用同一份栅格算一次精确 EDT，打印**注入后**机器人处的净空 + 各方向净空，
   直接告诉你"该往哪个方向逃"，用来和 RViz 里的绿线对照。

三种形状（--shape）
===================
  wall   机器人一侧一堵短墙：净空掉到 ~0，但**有明确的逃逸方向**。
         用来验证"脱困判定 + 脱困路径规划"能不能正常触发与给出方向。
  pocket 三面围住、只留一个开口：车真的退无可退。
         用来跑完整个 recovery.window_sec，看它超时退出（--shape pocket 时
         开口方向由 --open-at 指定，默认 +x）。
  restore  从**注入前自动保存的快照**恢复原图并重发（快照在首次注入时写到 TMPDIR）。
         不这么做不行：注入后代价地图里就是注入过的图，"把当前 /map 原样重发"等于
         没恢复 —— 这是本脚本踩过的坑，现在是快照实现。
         恢复成功后快照自动删除。本脚本从不写磁盘上的地图文件，改的只是内存里正在发的
         那一份，所以**重启导航栈**也能回到原状（快照丢了也不致命）。

默认参数为什么是这些值（本次实测标定，改前先读）
================================================
  --wall-gap 0.20   墙内边缘到车心 0.20 m -> 净空 ≈ 0.115~0.165 m，稳稳低于
                    默认 stuck_clearance 0.25 m。要卡死就得让它 < 0.25，
                    gap 取 0.28 时净空只有 0.265 m —— **刚好够不到**，会白等。
  --arena-radius 3.0 先把车周围 3 m 清空。本项目默认地图上，出生点最近的障碍在
                    0.65~0.73 m，不清的话注入的墙不是"控制净空的那一个"，
                    注入多少都看不出预期效果。
  --wall-length 1.2  短墙。注意一堵 1.2 m 的墙在 --search-radius 0.4 m 的环形采样
                    上是**可以绕过去的**，所以预测出来的逃逸方向会偏向墙的端头而不是
                    正对墙的法向反方向。想看到"正对背面"，用 --wall-length 3.0。

坐标约定
========
机器人出生时 map 系坐标就是 (0, 0)：仿真走 `slam:=False` + 真值里程计，odom 原点
被设成出生位姿，`map_to_odom` 是单位阵。所以 `--robot-x 0 --robot-y 0` 就能在
"车还没动"时跳过 TF 直接指定位置。车动过之后**不要**再用这个默认值，
让脚本自己查 TF（不给 --robot-x/--robot-y 即可）。

典型用法
========
  # 终端 A：起仿真
  ./script/start_sim_nav.sh --controller minco

  # 终端 B：先抓数据（只订阅，不控制车），事后可复盘
  python3 script/capture_minco_run.py

  # 终端 C：注入障碍
  python3 script/inject_recovery_stuck.py --shape wall --wall-dir +y
  #   等 3~5 s，看 RViz 里的绿线 + 终端 C 打印的方向是否一致

  # 验证完恢复现场
  python3 script/inject_recovery_stuck.py --shape restore

⚠ **逃逸方向不一定"正对墙的背面"，这通常是正常的。**
  环形搜索只在半径 `search_radius`（默认 0.4 m）的圆周上采样，所以一堵 1.2 m 长的短墙
  在这个半径上是可以"从侧面绕过去"的 —— 脚本预测出来的方向会偏向墙的端头，而不是
  墙的法向反方向。想让方向严格等于"背对墙"，把墙加长：
      --wall-length 3.0
  这是参考实现 `calculateEscapeVelocity` 的固有性质（本迁移保持语义一致），
  不是本脚本或脱困逻辑的 bug。详见 docs/minco脱困机制迁移实施记录(ai).md §7.1。

⚠ 建议先把门限调低，否则要等 5 次失败才进脱困：
     config/simulation/nav2_params_srm_minco.yaml
       FollowPath.recovery.fail_threshold: 2
       FollowPath.recovery.stuck_clearance: 0.60
   改完**必须重启导航栈**（本插件没有 onSetParameters 回调，ros2 param set 无效）。
"""

import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np

try:
    import rclpy
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.time import Time
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import OccupancyGrid
    from tf2_ros import Buffer, TransformListener
    from visualization_msgs.msg import Marker, MarkerArray
except ImportError as exc:  # pragma: no cover - 环境缺依赖时给出人话提示
    sys.exit(f"缺少 ROS 2 Python 依赖: {exc}\n请先 source 本工作区的 install/setup.bash")

# 与 srm27_minco_core::CellState / Esdf2D 的口径保持一致：
# 只有"原始占用"和"未知"进距离场，膨胀代价值不计。OccupancyGrid 里
# 100 = 占用、-1 = 未知、0..99 = 自由（含膨胀后的代价值）。
OCCUPIED = 100
UNKNOWN = -1

# 供 EDT 用的种子阈值：>= 65 当作占用（Nav2 的 INSCRIBED_INFLATED_OBSTACLE=253 也会
# 落到这里，但静态地图发布的是 0/100，正常不会出现中间值）。
SEED_THRESHOLD = 65


def snapshot_path(map_topic):
    """注入前快照的落盘位置（按话题名区分，避免多机/多话题互相覆盖）。

    放在 TMPDIR 下：这是**运行期**状态，不是仓库产物；重启导航栈也能回到原图，
    所以丢失快照并不致命。
    """
    import os
    import tempfile

    safe = map_topic.strip("/").replace("/", "_") or "map"
    return Path(tempfile.gettempdir()) / f"srm27_stuck_snapshot_{safe}.npy"


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


class MapInjector(Node):
    def __init__(self, args):
        super().__init__("srm27_inject_recovery_stuck")
        self.args = args
        self.map_msg = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.map_pub = self.create_publisher(OccupancyGrid, args.map_topic, latched)
        self.marker_pub = self.create_publisher(MarkerArray, args.marker_topic, 10)
        self.create_subscription(OccupancyGrid, args.map_topic, self._on_map, latched)

    def _on_map(self, msg):
        self.map_msg = msg

    # ---------------------------------------------------------------- 取图
    def wait_for_map(self, timeout):
        deadline = time.time() + timeout
        while time.time() < deadline and self.map_msg is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.map_msg is not None

    # ------------------------------------------------------------ 取车位姿
    def robot_pose(self, frame):
        """返回 (x, y, yaw)（map 系）。优先用 TF，失败再退回 --robot-x/y/yaw。"""
        if self.args.robot_x is not None and self.args.robot_y is not None:
            yaw = math.radians(self.args.robot_yaw_deg)
            self.get_logger().info("使用命令行给定的机器人位置 (跳过 TF)")
            return self.args.robot_x, self.args.robot_y, yaw

        deadline = time.time() + self.args.tf_timeout
        parent = self.args.base_frame
        while time.time() < deadline:
            try:
                tf = self.tf_buffer.lookup_transform(
                    frame, parent, Time(), timeout=Duration(seconds=0.3)
                )
                t = tf.transform.translation
                q = tf.transform.rotation
                # yaw from quaternion (只取平面分量)
                siny = 2.0 * (q.w * q.z + q.x * q.y)
                cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                yaw = math.atan2(siny, cosy)
                return t.x, t.y, yaw
            except Exception:  # TF 还没准备好
                rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().error(
            f"等不到 TF {frame} <- {parent}；用 --robot-x/--robot-y 显式给位置。"
        )
        return None


def rect_from_world(info, x0, y0, x1, y1):
    """世界坐标矩形 -> 栅格索引矩形 (col0, col1, row0, row1)，含端点，已夹到界内。

    **所有几何都走这个函数**，不要手写行列算术。原因：OccupancyGrid / pgm 的
    data 行序是"行 0 = 最高 y"，于是"世界 y 增大"对应"行号减小"。
    本脚本第一版在 build_shape 里直接写 `cy = my + dy * offset`，把 `+y` 算成了
    行号 +1，也就是**地图上的 -y 方向**，墙被砌到了车身后头（实测偏移 10 格）。
    换成世界坐标后就只有这一处需要关心行序，写不错。
    """
    res = info.resolution
    ox, oy = info.origin.position.x, info.origin.position.y
    h, w = info.height, info.width
    c0 = int(math.floor((min(x0, x1) - ox) / res))
    c1 = int(math.ceil((max(x0, x1) - ox) / res))
    # row = (oy + h*res - y)/res ；y 越大 row 越小
    r0 = int(math.floor((oy + h * res - max(y0, y1)) / res))
    r1 = int(math.ceil((oy + h * res - min(y0, y1)) / res))
    c0, c1 = max(0, c0), min(w - 1, c1)
    r0, r1 = max(0, r0), min(h - 1, r1)
    return c0, c1, r0, r1


def stamp_rect(cells2d, info, x0, y0, x1, y1):
    """把世界坐标矩形标成占用；返回改动格数。"""
    c0, c1, r0, r1 = rect_from_world(info, x0, y0, x1, y1)
    changed = 0
    for row in range(r0, r1 + 1):
        for col in range(c0, c1 + 1):
            if not cells2d[row, col]:
                cells2d[row, col] = True
                changed += 1
    return changed


def clear_arena(cells2d, info, mx, my, radius_m):
    """把机器人周围 radius_m 内的所有栅格清成自由。

    为什么必须先清而不是直接砌墙：真实地图上机器人附近可能本来就有障碍，
    那样"注入的墙"就不是控制净空的那一个，注入多少都看不出预期效果。
    先清出一块空地，注入的墙才唯一决定净空与逃逸方向 —— 这也是本脚本
    "确定性"的来源。
    """
    res = info.resolution
    h, w = cells2d.shape
    r = int(math.ceil(radius_m / res))
    y0, y1 = max(0, my - r), min(h, my + r + 1)
    x0, x1 = max(0, mx - r), min(w, mx + r + 1)
    changed = 0
    for yy in range(y0, y1):
        for xx in range(x0, x1):
            if cells2d[yy, xx]:
                cells2d[yy, xx] = False
                changed += 1
    return changed


def build_shape(cells2d, info, mx, my, shape, args):
    """往 `cells2d`（二维 bool，True=占用/种子）注入障碍。

    返回 (改动的格数, 说明字符串)。`restore` 直接返回 0，不做任何改动。
    形态一律用**世界坐标**描述（见 rect_from_world 的说明）。
    """
    res = info.resolution
    if shape == "restore":
        return 0, "restore（不注入任何障碍）"

    cleared = clear_arena(cells2d, info, mx, my, args.arena_radius)

    # 车心的世界坐标（由栅格反算，和 rect_from_world 用同一套换算）
    cx_w = info.origin.position.x + (mx + 0.5) * res
    cy_w = info.origin.position.y + (info.height - my - 0.5) * res

    directions = {"+x": (1.0, 0.0), "-x": (-1.0, 0.0), "+y": (0.0, 1.0), "-y": (0.0, -1.0)}
    half_len = args.wall_length / 2.0
    half_thick = 1.0 * res  # 2 格厚

    if shape == "wall":
        if args.wall_dir not in directions:
            sys.exit(f"--wall-dir 只支持 {list(directions)}，收到 {args.wall_dir}")
        ux, uy = directions[args.wall_dir]
        # 墙心沿法向外移 gap + half_thick -> 内边缘正好落在 gap 处
        d = args.wall_gap + half_thick
        wx, wy = cx_w + ux * d, cy_w + uy * d
        # 墙身沿**垂直于法向**的方向展开
        tx, ty = -uy, ux
        changed = stamp_rect(
            cells2d, info,
            wx - abs(tx) * half_len - abs(ux) * half_thick,
            wy - abs(ty) * half_len - abs(uy) * half_thick,
            wx + abs(tx) * half_len + abs(ux) * half_thick,
            wy + abs(ty) * half_len + abs(uy) * half_thick,
        )
        opposite = {"+x": "-x", "-x": "+x", "+y": "-y", "-y": "+y"}[args.wall_dir]
        return cleared + changed, (
            f"wall：清出 {args.arena_radius:.1f} m 空地后在 {args.wall_dir} 侧 "
            f"{args.wall_gap:.2f} m 处砌墙（长 {args.wall_length:.1f} m），"
            f"逃逸方向应指向 {opposite}"
        )

    if shape == "pocket":
        if args.open_at not in directions:
            sys.exit(f"--open-at 只支持 {list(directions)}，收到 {args.open_at}")
        changed = 0
        for name, (ux, uy) in directions.items():
            if name == args.open_at:
                continue
            d = args.wall_gap + half_thick
            wx, wy = cx_w + ux * d, cy_w + uy * d
            tx, ty = -uy, ux  # 墙身方向（垂直于法向）
            changed += stamp_rect(
                cells2d, info,
                wx - abs(tx) * half_len - abs(ux) * half_thick,
                wy - abs(ty) * half_len - abs(uy) * half_thick,
                wx + abs(tx) * half_len + abs(ux) * half_thick,
                wy + abs(ty) * half_len + abs(uy) * half_thick,
            )
        return cleared + changed, (
            f"pocket：清出 {args.arena_radius:.1f} m 空地后三面砌墙、只留 {args.open_at} 开口"
            f"（壁到车心约 {args.wall_gap:.2f} m），车应朝 {args.open_at} 逃；"
            f"跑满 window 后超时退出属正常"
        )

    sys.exit(f"未知形状: {shape}")


def clearance_field(cells2d, info):
    """二维 bool 种子图 -> 米制净空场（精确 EDT）。

    与 `srm27_minco_core::Esdf2D` 同口径：种子 = 占用/未知，距离按格中心算，
    再保守化减掉半格对角线（与 core 的 `conservative` 一致）。
    """
    from scipy import ndimage

    seed = cells2d
    if not seed.any():
        return None, np.full((h, w), 1.0e3)
    edt_cells = ndimage.distance_transform_edt(~seed)
    half_diag = 0.5 * math.hypot(info.resolution, info.resolution)
    edt_m = edt_cells * info.resolution - half_diag
    return seed, edt_m


def report_directions(edt_m, info, mx, my, radius_m, samples):
    """打印机器人处净空 + 环形采样里最开阔的方向（与脱困的环形搜索同口径）。"""
    res = info.resolution
    clr = edt_m[my, mx]
    print(f"  机器人处净空（EDT，米）: {clr:.3f}")
    best = (-1.0e9, None)
    for i in range(samples):
        ang = 2.0 * math.pi * i / samples
        sx = int(round(mx + radius_m * math.cos(ang) / res))
        sy = int(round(my - radius_m * math.sin(ang) / res))  # 栅格 y 向下
        if not (0 <= sx < info.width and 0 <= sy < info.height):
            continue
        v = edt_m[sy, sx]
        if v > best[0]:
            best = (v, ang)
    if best[1] is not None:
        deg = math.degrees(best[1])
        print(f"  环形搜索半径 {radius_m:.2f} m, {samples} 方向:")
        print(f"    最开阔方向 = {deg:7.1f} deg (净空 {best[0]:.3f} m)  <-- 绿线应指向这里")
    return clr, best


def main():
    ap = argparse.ArgumentParser(
        description="往机器人身上注入障碍，确定性复现 MINCO 脱困",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--shape", default="wall", choices=["wall", "pocket", "restore"])
    ap.add_argument("--ns", default="red_standard_robot1", help="机器人命名空间")
    ap.add_argument("--map-topic", default="/map", help="地图话题（注意带前导 /）")
    ap.add_argument("--marker-topic", default="/recovery_stuck_markers")
    ap.add_argument("--base-frame", default="base_link")
    ap.add_argument("--map-frame", default="map")
    ap.add_argument("--wall-dir", default="+y", choices=["+x", "-x", "+y", "-y"])
    ap.add_argument("--wall-gap", type=float, default=0.20,
                    help="墙内边缘到车心的距离（米）。净空 ≈ 该值 - 0.035（半格对角线）"
                         "。默认 0.20 -> 净空约 0.165，稳稳低于默认 stuck_clearance(0.25)")
    ap.add_argument("--wall-length", type=float, default=1.20, help="墙长（米）")
    ap.add_argument("--arena-radius", type=float, default=3.0,
                    help="先清空的空地半径（米）。清掉真实地图的干扰障碍，"
                         "让注入的墙唯一决定净空与方向")
    ap.add_argument("--open-at", default="+x", choices=["+x", "-x", "+y", "-y"])
    ap.add_argument("--pocket-size", type=float, default=0.60, help="口袋边长（米）")
    ap.add_argument("--search-radius", type=float, default=0.4, help="环形搜索半径（米）")
    ap.add_argument("--search-samples", type=int, default=36)
    ap.add_argument("--robot-x", type=float, default=None, help="跳过 TF，直接给 x（map 系）")
    ap.add_argument("--robot-y", type=float, default=None, help="跳过 TF，直接给 y（map 系）")
    ap.add_argument("--robot-yaw-deg", type=float, default=0.0)
    ap.add_argument("--tf-timeout", type=float, default=10.0)
    ap.add_argument("--map-timeout", type=float, default=15.0)
    ap.add_argument("--publish-repeat", type=int, default=3, help="重复发布次数（latched 保险）")
    args = ap.parse_args()

    rclpy.init()
    node = MapInjector(args)
    try:
        print("=" * 72)
        print("SRM27 脱困复现：往机器人身上注入障碍")
        print("=" * 72)

        if not node.wait_for_map(args.map_timeout):
            print(f"[错误] {args.map_timeout:.0f} s 内没收到 {args.map_topic}。")
            print("       确认导航栈已经起来（./script/start_sim_nav.sh --controller minco）。")
            return 1

        info = node.map_msg.info
        print(f"地图: {info.width} x {info.height} @ {info.resolution} m, "
              f"origin=({info.origin.position.x:.2f}, {info.origin.position.y:.2f})")

        # 全程用二维视图（行序与 OccupancyGrid 一致：行 0 = 最高 y）。
        # 一开始的版本一半用一维下标、一半用二维，直接 IndexError —— 统一成二维。
        before_flat = np.array(node.map_msg.data, dtype=np.int16)
        before = before_flat.reshape(info.height, info.width)

        snapshot = snapshot_path(args.map_topic)

        # ---------------- restore 快速路径 ----------------
        #
        # restore **不需要知道机器人在哪**，所以必须放在取位姿之前。
        # 踩过的坑：原先 restore 也走"先取 TF 位姿"的路，仿真没起/没有 TF 时
        # 它会在恢复之前就 return，用户以为恢复了其实没有。
        if args.shape == "restore":
            if not snapshot.exists():
                print(f"[错误] 找不到快照 {snapshot}，无法恢复。")
                print("       快照在执行注入时自动生成；若已丢失，重启导航栈即可回到原图。")
                return 1
            saved = np.load(snapshot)
            if saved.shape[0] != info.height * info.width:
                print(f"[错误] 快照尺寸与当前地图不符（{saved.shape[0]} vs "
                      f"{info.height * info.width}）—— 换过地图？删掉快照或重启导航栈。")
                return 1
            out = OccupancyGrid()
            out.header = node.map_msg.header
            out.header.stamp = node.get_clock().now().to_msg()
            out.info = info
            out.data = [int(v) for v in saved.astype(np.int16)]
            for _ in range(args.publish_repeat):
                node.map_pub.publish(out)
                rclpy.spin_once(node, timeout_sec=0.05)
            snapshot.unlink(missing_ok=True)
            print(f"已从快照恢复原图并发布到 {args.map_topic}: {snapshot}")
            print("快照已删除（地图已回到原状）。")
            print(f"  static_layer 下一个更新周期（update_frequency）就会生效")
            return 0

        # ---------------- 注入路径：需要机器人位姿 ----------------
        pose = node.robot_pose(args.map_frame)
        if pose is None:
            return 1
        rx, ry, ryaw = pose
        print(f"机器人位姿（{args.map_frame}）: x={rx:.3f} y={ry:.3f} yaw={math.degrees(ryaw):.1f} deg")

        # world -> cell（OccupancyGrid 的 data 自上而下：行 0 = 最高 y）
        mx = int(math.floor((rx - info.origin.position.x) / info.resolution))
        my = int(
            math.floor(
                (info.origin.position.y + info.height * info.resolution - ry)
                / info.resolution
            )
        )
        if not (0 <= mx < info.width and 0 <= my < info.height):
            print(f"[错误] 机器人 ({rx:.2f},{ry:.2f}) 在地图外；"
                  f"给它在地图内的坐标（--robot-x/--robot-y）。")
            return 1
        print(f"对应栅格: ({mx}, {my})")

        cells = (before >= SEED_THRESHOLD) | (before == UNKNOWN)
        changed, desc = build_shape(cells, info, mx, my, args.shape, args)
        print(f"注入: {desc}（改动 {changed} 格）")

        # 注入后的净空与逃逸方向预测
        _, edt_m = clearance_field(cells, info)
        print("注入后:")
        report_directions(edt_m, info, mx, my, args.search_radius, args.search_samples)

        # ---- 快照：注入前存一份"干净"的栅格，restore 才有东西可恢复 ----
        #
        # 踩过的坑：restore 若只"把当前 /map 原样重发"，拿到的是**已经被污染的**那一份
        # （注入后代价地图里就是注入过的图），等于没恢复。必须是注入前存下来的。
        if not snapshot.exists():
            np.save(snapshot, before_flat)
            print(f"已保存注入前快照: {snapshot}")

        # 只改被注入的格：把二维 cells 写回原来的一维 data，其余栅格原样保留
        # （包括膨胀后的中间代价值，不整张图重算，避免破坏 static_layer 已合并的内容）。
        out_flat = before_flat.copy()
        occupied_now = cells.reshape(-1)
        occupied_was = ((before_flat >= SEED_THRESHOLD) | (before_flat == UNKNOWN))
        out_flat[occupied_now & ~occupied_was] = OCCUPIED

        out = OccupancyGrid()
        out.header = node.map_msg.header
        out.header.stamp = node.get_clock().now().to_msg()
        out.info = info
        out.data = [int(v) for v in out_flat]
        for _ in range(args.publish_repeat):
            node.map_pub.publish(out)
            rclpy.spin_once(node, timeout_sec=0.05)
        print(f"已发布 {args.publish_repeat} 次到 {args.map_topic}（transient_local）")
        print(f"  static_layer 下一个更新周期（update_frequency）就会生效")

        # RViz 标记：把注入的障碍画出来（红色方块），便于确认注入位置
        if args.shape != "restore":
            ma = MarkerArray()
            m = Marker()
            m.header.frame_id = args.map_frame
            m.header.stamp = node.get_clock().now().to_msg()
            m.ns = "injected_stuck"
            m.id = 0
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = rx
            m.pose.position.y = ry
            m.pose.position.z = 0.02
            m.pose.orientation.w = 1.0
            # 标记只表示"注入中心"和影响范围，不是精确障碍形状；
            # 精确形状看代码里 build_shape 的输出即可，这里给个便于肉眼定位的方块。
            size = 2.0 * (args.wall_gap + 0.3)
            m.scale.x = max(size, 0.6)
            m.scale.y = max(size, 0.6)
            m.scale.z = 0.01
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.0, 0.0, 0.30
            ma.markers.append(m)
            node.marker_pub.publish(ma)
            print(f"已在 {args.marker_topic} 发布注入位置标记（RViz 加 MarkerArray 即可看到）")

        print()
        print("接下来该看到什么 (按时间顺序):")
        print("  1) 全局代价地图把注入区并进来 -> 规划开始失败")
        print("     FollowPath/diagnostics 的 planning_result 不再是 success")
        print(f"  2) 连续失败达 recovery.fail_threshold 次 -> 进入脱困")
        print("     日志: '进入脱困（触发=gradient|circular_search，进入净空 ...）'")
        print("     diagnostics: recovery_active=true")
        print("  3) RViz 里出现绿线（脱困路径）+ 蓝箭头 + 净空球")
        print(f"     绿线应指向上面算出的最开阔方向")
        print("  4) 脱离卡死区 -> '脱困成功'；跑满 window_sec -> '脱困超时' 回重规划")
        print()
        print("观察命令:")
        print(f"  ros2 topic echo /{args.ns}/FollowPath/diagnostics \\")
        print("    | grep -E 'recovery_(active|trigger|exit|clearance)'")
        print()
        print(f"恢复现场: python3 {Path(__file__).name} --shape restore")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
