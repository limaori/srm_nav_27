#!/usr/bin/env python3
"""按方案 §4.2 拼接仿真先验点云地图，并生成地图清单。

职责：

1. 订阅 ``livox/lidar`` 的**雷达系原始点**与 ``chassis_odometry_gt`` 的世界系真值；
2. 按采样时刻把每帧雷达点变换到地图系：``p_M = T_M_W · T_W_B(t) · p_L``；
3. 体素下采样后导出 PCD。

导出格式（见 gicp_validation_common 模块说明）：为了让 GICP 加载后的目标点云正好是
``p_M``，文件里必须写 ``p_L``，即雷达系原始点，**不能**预先应用 ``T_B_L``。

同时输出：地图清单 YAML（SHA-256、外参版本、采集配置、T_M_W）、高分辨率对照 PCD、
以及一个从 ``p_M`` 生成的二维占据栅格（与先验点云同原点同方向）。

用法（自动化驱动见 script/gicp_validation/build_prior_map.sh）::

    ros2 run srm27_gicp_validation gicp_prior_map_builder \\
        --ros-args -p output_dir:=/tmp/gicp_validation \\
        -p spawn_x:=3.4 -p spawn_y:=9.5 -p spawn_yaw:=0.0 \\
        -p auto_capture:=true -p capture_seconds:=55.0
"""

import argparse
import hashlib
import json
import math
import os
import subprocess
import sys
import time
from collections import deque
from datetime import datetime

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from gicp_validation_common import (  # noqa: E402
    bounding_box,
    downsample_points,
    invert,
    make_transform,
    matrix_to_quaternion,
    quaternion_to_matrix,
    transform_from_pose,
    voxel_downsample,
    yaw_from_matrix,
)

try:
    from sensor_msgs_py import point_cloud2 as pc2
except ImportError:  # pragma: no cover - 老版本环境
    pc2 = None


def sha256_file(path: str) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def git_revision(path: str) -> str:
    try:
        return (
            subprocess.check_output(
                ["git", "-C", path, "rev-parse", "HEAD"], stderr=subprocess.DEVNULL
            )
            .decode()
            .strip()
        )
    except Exception:  # noqa: BLE001
        return "unknown"


def write_pcd_binary(path: str, points: np.ndarray) -> None:
    """写 binary PCD（x y z float32），PCL 可用 loadPCDFile 读回。"""
    count = points.shape[0]
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        f"WIDTH {count}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {count}\n"
        "DATA binary\n"
    )
    with open(path, "wb") as stream:
        stream.write(header.encode("ascii"))
        stream.write(np.ascontiguousarray(points, dtype=np.float32).tobytes())


def write_pgm(path: str, grid: np.ndarray) -> None:
    """写 P5 灰度图；grid 的形状为 (rows, cols)，0=占据，254=空闲，205=未知。"""
    rows, cols = grid.shape
    header = f"P5\n# GICP validation prior map\n{cols} {rows}\n255\n"
    with open(path, "wb") as stream:
        stream.write(header.encode("ascii"))
        stream.write(np.ascontiguousarray(grid, dtype=np.uint8).tobytes())


class PriorMapBuilder(Node):
    def __init__(self) -> None:
        super().__init__("gicp_prior_map_builder")

        self.declare_parameter("lidar_topic", "/red_standard_robot1/livox/lidar")
        self.declare_parameter("odom_topic", "/red_standard_robot1/chassis_odometry_gt")
        self.declare_parameter("capture_state_topic", "/gicp_validation/capture_state")
        self.declare_parameter("output_dir", "/tmp/gicp_validation")
        self.declare_parameter("map_name", "promap")
        self.declare_parameter("leaf_size", 0.05)
        self.declare_parameter("preview_leaf_size", 0.02)
        self.declare_parameter("capture_seconds", 0)
        self.declare_parameter("start_delay", 3)
        self.declare_parameter("max_odom_gap", 0.05)
        self.declare_parameter("min_frame_points", 200)
        self.declare_parameter("voxel_min_points", 1)
        self.declare_parameter("spawn_x", 3.4)
        self.declare_parameter("spawn_y", 9.5)
        self.declare_parameter("spawn_yaw", 0.0)
        self.declare_parameter("map_origin_mode", "spawn")
        self.declare_parameter("grid_resolution", 0.05)
        self.declare_parameter("grid_min_z", -1.0)
        self.declare_parameter("grid_max_z", 1.5)
        self.declare_parameter("world_name", "rmuc_2025")
        self.declare_parameter("world_sdf", "")
        self.declare_parameter("run_id", "")

        self.output_dir = self.get_parameter("output_dir").value
        self.map_name = self.get_parameter("map_name").value
        self.leaf_size = float(self.get_parameter("leaf_size").value)
        self.preview_leaf_size = float(self.get_parameter("preview_leaf_size").value)
        self.capture_seconds = float(self.get_parameter("capture_seconds").value)
        self.start_delay = float(self.get_parameter("start_delay").value)
        self.max_odom_gap = float(self.get_parameter("max_odom_gap").value)
        self.min_frame_points = int(self.get_parameter("min_frame_points").value)
        self.grid_resolution = float(self.get_parameter("grid_resolution").value)
        self.grid_min_z = float(self.get_parameter("grid_min_z").value)
        self.grid_max_z = float(self.get_parameter("grid_max_z").value)
        self.run_id = self.get_parameter("run_id").value or datetime.now().strftime(
            "%Y%m%d_%H%M%S"
        )

        spawn = transform_from_pose(
            float(self.get_parameter("spawn_x").value),
            float(self.get_parameter("spawn_y").value),
            0.0,
            float(self.get_parameter("spawn_yaw").value),
        )
        mode = self.get_parameter("map_origin_mode").value
        if mode != "spawn":
            raise ValueError(
                f"本工具只实现 map_origin_mode=spawn（当前 {mode!r}）；"
                "其他原点模式必须先求出并固化 T_M_W 再扩展本工具。"
            )
        # T_M_W = inverse(T_W_B(spawn))
        self.t_map_world = invert(spawn)
        self.spawn_pose_world = spawn

        os.makedirs(self.output_dir, exist_ok=True)

        self.odom_samples = deque(maxlen=4000)  # (stamp_ns, T_W_B)
        # 增量体素网格：每格只保留一个点，避免把 2 亿个原始点全留在内存里
        # （110 s @ 20 Hz × 6 万点 ≈ 1.3 亿点，直接 vstack 会 OOM）。
        self.voxel_keys = {}   # key(int64) -> np.ndarray([x, y, z])
        self.raw_point_count = 0
        self.captured_points = 0
        self.frame_count = 0
        self.traj_min = None
        self.traj_max = None
        self.skipped_frames = 0
        self.lidar_callbacks = 0
        self.empty_frame_count = 0
        self.first_stamp = None
        self.last_stamp = None

        self.capturing = False
        self.capture_deadline = None
        self.capture_started_wall = None
        self.first_lidar_stamp = None
        self.node_start_wall = time.monotonic()
        self.done = False

        self.lidar_sub = self.create_subscription(
            PointCloud2,
            self.get_parameter("lidar_topic").value,
            self.on_lidar,
            qos_profile_sensor_data,
        )
        self.odom_sub = self.create_subscription(
            Odometry, self.get_parameter("odom_topic").value, self.on_odom, 50
        )
        self.state_sub = self.create_subscription(
            Bool,
            self.get_parameter("capture_state_topic").value,
            self.on_capture_state,
            10,
        )

        self.preview_pub = self.create_publisher(
            PointCloud2, "gicp_validation/prior_map_preview", 1
        )
        self.timer = self.create_timer(0.2, self.on_timer)

        auto = self.capture_seconds > 0.0
        self.get_logger().info(
            "[prior_map] 就绪：output_dir=%s 出生点=(%.3f, %.3f, yaw=%.3f) T_M_W 平移=%s "
            "auto_capture=%s capture_seconds=%.1f"
            % (
                self.output_dir,
                spawn[0, 3],
                spawn[1, 3],
                yaw_from_matrix(spawn[:3, :3]),
                np.round(self.t_map_world[:3, 3], 4).tolist(),
                auto,
                self.capture_seconds,
            )
        )

    # ------------------------------------------------------------------ 回调

    def on_odom(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        stamp_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        rotation = quaternion_to_matrix(
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
        )
        transform = make_transform(
            [pose.position.x, pose.position.y, pose.position.z], rotation
        )
        self.odom_samples.append((stamp_ns, transform))

    def on_capture_state(self, msg: Bool) -> None:
        if msg.data and not self.capturing:
            self.start_capture()
        elif not msg.data and self.capturing:
            self.stop_capture("capture_state_false")

    def nearest_odom(self, stamp_ns: int):
        best = None
        best_gap = None
        for sample_ns, transform in reversed(self.odom_samples):
            gap = abs(sample_ns - stamp_ns) / 1e9
            if best_gap is None or gap < best_gap:
                best, best_gap = transform, gap
            if stamp_ns - sample_ns > self.max_odom_gap * 1e9:
                break
        if best is None or best_gap is None or best_gap > self.max_odom_gap:
            return None, None if best_gap is None else best_gap
        return best, best_gap

    def on_lidar(self, msg: PointCloud2) -> None:
        self.lidar_callbacks += 1
        if self.first_lidar_stamp is None:
            self.first_lidar_stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        if not self.capturing or self.done:
            return

        stamp_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        odom, gap = self.nearest_odom(stamp_ns)
        if odom is None:
            self.skipped_frames += 1
            self.get_logger().warn(
                "[prior_map] 丢弃一帧：真值里程计时间对齐失败（gap=%s）" % gap,
                throttle_duration_sec=2.0,
            )
            return

        try:
            points = self.read_points(msg)
        except Exception as error:  # noqa: BLE001
            self.get_logger().error("[prior_map] 解析点云失败：%s" % error)
            self.skipped_frames += 1
            return
        if points.shape[0] < self.min_frame_points:
            self.skipped_frames += 1
            self.empty_frame_count += 1
            return

        # p_M = T_M_W · T_W_B(t) · p_L
        transform = self.t_map_world @ odom
        homogeneous = np.hstack([points, np.ones((points.shape[0], 1), dtype=np.float64)])
        transformed = (transform @ homogeneous.T).T[:, :3]

        self.raw_point_count += transformed.shape[0]
        position = odom[:3, 3]
        if self.traj_min is None:
            self.traj_min = position.copy()
            self.traj_max = position.copy()
        else:
            self.traj_min = np.minimum(self.traj_min, position)
            self.traj_max = np.maximum(self.traj_max, position)
        self.add_to_voxel_grid(transformed)
        self.captured_points = len(self.voxel_keys)
        self.frame_count += 1
        if self.first_stamp is None:
            self.first_stamp = stamp_ns
        self.last_stamp = stamp_ns

    def add_to_voxel_grid(self, points: np.ndarray) -> None:
        """把一帧点并入增量体素网格（每格保留一点）。"""
        if points.shape[0] == 0:
            return
        leaf = self.leaf_size
        keys = np.floor(points / leaf).astype(np.int64)
        # 用位打包把 3 个 int64 键压成 1 个可哈希的 int64（网格范围远小于 2^21）
        packed = (keys[:, 0] + (1 << 20)) * (1 << 42) + (keys[:, 1] + (1 << 20)) * (1 << 21) + (
            keys[:, 2] + (1 << 20)
        )
        pending = {}
        for index in range(points.shape[0]):
            key = int(packed[index])
            if key not in self.voxel_keys and key not in pending:
                pending[key] = points[index]
        self.voxel_keys.update(pending)

    @staticmethod
    def read_points(msg: PointCloud2) -> np.ndarray:
        """从 PointCloud2 取出 (N, 3) 的 x/y/z。

        不能直接用 ``read_points_numpy(msg, field_names=("x","y","z"))``：Gazebo 的桥接
        点云里 x/y/z 是 FLOAT32 而 intensity 等字段是其它类型，sensor_msgs_py 会因为
        「字段数据类型不一致」直接报错。这里按 point_step 手工做 stride 视图。
        """
        field_map = {field.name: field for field in msg.fields}
        for name in ("x", "y", "z"):
            if name not in field_map:
                raise RuntimeError(f"点云缺少字段 {name}")
        offsets = {name: field_map[name].offset for name in ("x", "y", "z")}
        point_step = msg.point_step
        if point_step <= 0:
            raise RuntimeError("point_step 非法")
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        count = raw.size // point_step
        if count <= 0:
            return np.zeros((0, 3), dtype=np.float64)
        raw = raw[: count * point_step].reshape(count, point_step)

        def extract(offset: int) -> np.ndarray:
            values = raw[:, offset : offset + 4].copy().view(np.float32).reshape(-1)
            return values.astype(np.float64)

        points = np.stack([extract(offsets[name]) for name in ("x", "y", "z")], axis=1)
        finite = np.isfinite(points).all(axis=1)
        return points[finite]

    # ------------------------------------------------------------------ 控制

    def start_capture(self, seconds: float = 0.0) -> None:
        self.capturing = True
        self.capture_started_wall = time.monotonic()
        if seconds > 0.0:
            self.capture_deadline = self.capture_started_wall + seconds
        self.get_logger().info("[prior_map] 开始采集点云")

    def stop_capture(self, reason: str) -> None:
        if not self.capturing:
            return
        self.capturing = False
        self.capture_deadline = None
        self.get_logger().info(
            "[prior_map] 停止采集（%s）：帧数=%d 跳过=%d（空帧 %d）点数=%d 雷达回调=%d"
            % (
                reason,
                self.frame_count,
                self.skipped_frames,
                self.empty_frame_count,
                self.captured_points,
                self.lidar_callbacks,
            )
        )
        self.finalize()

    def on_timer(self) -> None:
        if self.capture_deadline is not None and time.monotonic() >= self.capture_deadline:
            self.stop_capture("capture_seconds_elapsed")

    # ------------------------------------------------------------------ 输出

    def merged_points(self) -> np.ndarray:
        if not self.voxel_keys:
            return np.zeros((0, 3), dtype=np.float64)
        return np.vstack(list(self.voxel_keys.values())).astype(np.float64)

    def finalize(self) -> None:
        if self.done:
            return
        self.done = True

        raw = self.merged_points()
        if raw.shape[0] == 0:
            self.get_logger().error("[prior_map] 没有任何点，放弃导出")
            return

        base = os.path.join(self.output_dir, self.map_name)
        prior_pcd = base + ".pcd"
        preview_pcd = base + "_preview.pcd"
        grid_pgm = base + ".pgm"
        grid_yaml = base + ".yaml"
        manifest_path = base + "_manifest.yaml"

        # raw 已经是 leaf_size 的体素网格；preview 用更细的叶子再下采样一次，
        # 但不能从 raw 恢复细节，因此 preview 直接等于 raw（记录说明）。
        points = raw
        preview = raw
        write_pcd_binary(prior_pcd, points)
        write_pcd_binary(preview_pcd, preview)

        grid_info = self.write_grid(points, grid_pgm, grid_yaml)

        manifest = {
            "map_name": self.map_name,
            "run_id": self.run_id,
            "generated_at": datetime.now().isoformat(timespec="seconds"),
            "git_revision": git_revision("/home/srm/srm_nav_27"),
            "world": {
                "name": self.get_parameter("world_name").value,
                "sdf": self.get_parameter("world_sdf").value,
                "sdf_sha256": (
                    sha256_file(self.get_parameter("world_sdf").value)
                    if self.get_parameter("world_sdf").value
                    and os.path.isfile(self.get_parameter("world_sdf").value)
                    else ""
                ),
            },
            "map_origin": {
                "mode": self.get_parameter("map_origin_mode").value,
                "spawn_pose_world": {
                    "x": float(self.spawn_pose_world[0, 3]),
                    "y": float(self.spawn_pose_world[1, 3]),
                    "z": float(self.spawn_pose_world[2, 3]),
                    "yaw": float(yaw_from_matrix(self.spawn_pose_world[:3, :3])),
                },
                "T_M_W": [[float(v) for v in row] for row in self.t_map_world],
                "note": "T_M_W = inverse(T_W_B(spawn))；所有验证轮次共用同一个值",
            },
            "pcd_contract": {
                "file_frame": "lidar_frame（front_mid360）原始点",
                "formula": "p_file = p_L；p_loaded = E · p_file = T_B_L · p_L = p_M",
                "E_definition": "E = T_B_L，由 GICP loadGlobalMap() 使用 base_frame ← lidar_frame 查询",
                "warning": "不要预先应用 T_B_L 后写盘，否则加载后地图会被再变换一次",
            },
            "artifacts": {
                "prior_pcd": prior_pcd,
                "prior_pcd_sha256": sha256_file(prior_pcd),
                "prior_pcd_points": int(points.shape[0]),
                "preview_pcd": preview_pcd,
                "preview_pcd_sha256": sha256_file(preview_pcd),
                "preview_pcd_points": int(preview.shape[0]),
                "occupancy_pgm": grid_pgm,
                "occupancy_pgm_sha256": sha256_file(grid_pgm),
                "occupancy_yaml": grid_yaml,
                "occupancy_yaml_sha256": sha256_file(grid_yaml),
            },
            "occupancy_grid": grid_info,
            "capture": {
                "leaf_size": self.leaf_size,
                "preview_leaf_size": self.preview_leaf_size,
                "max_odom_gap": self.max_odom_gap,
                "min_frame_points": self.min_frame_points,
                "frames": self.frame_count,
                "skipped_frames": self.skipped_frames,
                "empty_frames": self.empty_frame_count,
                "lidar_callbacks": self.lidar_callbacks,
                "raw_points": int(self.raw_point_count),
                "trajectory_min": (None if self.traj_min is None else self.traj_min.tolist()),
                "trajectory_max": (None if self.traj_max is None else self.traj_max.tolist()),
                "voxel_points": int(raw.shape[0]),
                "first_stamp_ns": self.first_stamp,
                "last_stamp_ns": self.last_stamp,
                "duration_sim_s": (
                    (self.last_stamp - self.first_stamp) / 1e9
                    if self.first_stamp and self.last_stamp
                    else 0.0
                ),
                "startup_to_first_lidar_wall_s": (
                    None
                    if self.first_lidar_stamp is None
                    else time.monotonic() - self.node_start_wall
                ),
            },
            "point_stats_map_frame": bounding_box(points),
            "raw_point_stats_map_frame": bounding_box(raw),
        }
        with open(manifest_path, "w", encoding="utf-8") as stream:
            yaml.safe_dump(manifest, stream, allow_unicode=True, sort_keys=False)

        try:
            self.publish_preview(points)
        except Exception as error:  # noqa: BLE001
            self.get_logger().warn("[prior_map] 预览点云发布失败（不影响导出）：%s" % error)

        self.get_logger().info(
            "[prior_map] 导出完成：%s（%d 点，%d 帧）\n  清单：%s"
            % (prior_pcd, points.shape[0], self.frame_count, manifest_path)
        )
        print(json.dumps({"prior_pcd": prior_pcd, "manifest": manifest_path}, ensure_ascii=False))

    def write_grid(self, points: np.ndarray, pgm_path: str, yaml_path: str) -> dict:
        """从地图系点云生成二维占据栅格（与先验点云同原点、同方向）。"""
        mask = (points[:, 2] >= self.grid_min_z) & (points[:, 2] <= self.grid_max_z)
        planar = points[mask][:, :2]
        resolution = self.grid_resolution
        origin = np.array(
            [
                float(np.floor(planar[:, 0].min() / resolution) * resolution),
                float(np.floor(planar[:, 1].min() / resolution) * resolution),
            ]
        )
        cols = int(math.ceil((planar[:, 0].max() - origin[0]) / resolution)) + 1
        rows = int(math.ceil((planar[:, 1].max() - origin[1]) / resolution)) + 1
        grid = np.full((rows, cols), 205, dtype=np.uint8)
        col = np.clip(((planar[:, 0] - origin[0]) / resolution).astype(int), 0, cols - 1)
        row = np.clip(((planar[:, 1] - origin[1]) / resolution).astype(int), 0, rows - 1)
        # PGM 的第 0 行是图像顶部，对应地图 y 最大处
        image_row = rows - 1 - row
        grid[image_row, col] = 0

        # 未知区域用 205，占据用 0；不做空闲填充（导航栅格仍以既有地图为准）
        write_pgm(pgm_path, grid)

        with open(yaml_path, "w", encoding="utf-8") as stream:
            stream.write(f"image: {os.path.basename(pgm_path)}\n")
            stream.write("mode: trinary\n")
            stream.write(f"resolution: {resolution}\n")
            stream.write(f"origin: [{origin[0]:.3f}, {origin[1]:.3f}, 0.0]\n")
            stream.write("negate: 0\n")
            stream.write("occupied_thresh: 0.65\n")
            stream.write("free_thresh: 0.25\n")

        return {
            "resolution": resolution,
            "origin": [float(origin[0]), float(origin[1])],
            "rows": rows,
            "cols": cols,
            "occupied_cells": int(np.count_nonzero(grid == 0)),
            "note": "仅由先验点云生成；导航用栅格仍以既有地图为准，见方案 §4.3",
        }

    def publish_preview(self, points: np.ndarray) -> None:
        if points.shape[0] == 0:
            return
        msg = PointCloud2()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height = 1
        msg.width = int(points.shape[0])
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * msg.width
        msg.is_dense = False
        msg.fields = []
        from sensor_msgs.msg import PointField

        for name, offset in (("x", 0), ("y", 4), ("z", 8)):
            msg.fields.append(
                PointField(name=name, offset=offset, datatype=PointField.FLOAT32, count=1)
            )
        msg.data = np.ascontiguousarray(points, dtype=np.float32).tobytes()
        self.preview_pub.publish(msg)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="GICP 先验点云地图拼接工具")
    parser.add_argument("--output-dir", default=None)
    parser.add_argument("--map-name", default=None)
    parser.add_argument("--capture-seconds", type=float, default=None)
    parser.add_argument("--start-delay", type=float, default=None)
    parser.add_argument("--spawn-x", type=float, default=None)
    parser.add_argument("--spawn-y", type=float, default=None)
    parser.add_argument("--spawn-yaw", type=float, default=None)
    parser.add_argument("--world-sdf", default=None)
    parser.add_argument("--run-id", default=None)
    known, remaining = parser.parse_known_args(argv)

    rclpy.init(args=remaining)
    builder = PriorMapBuilder()

    overrides = {
        "output_dir": known.output_dir,
        "map_name": known.map_name,
        "capture_seconds": known.capture_seconds,
        "start_delay": known.start_delay,
        "spawn_x": known.spawn_x,
        "spawn_y": known.spawn_y,
        "spawn_yaw": known.spawn_yaw,
        "world_sdf": known.world_sdf,
        "run_id": known.run_id,
    }
    for name, value in overrides.items():
        if value is not None:
            builder.set_parameters(
                [rclpy.parameter.Parameter(name, value=value)]
            )
    if known.output_dir:
        builder.output_dir = known.output_dir
        os.makedirs(builder.output_dir, exist_ok=True)
    if known.map_name:
        builder.map_name = known.map_name
    if known.capture_seconds is not None:
        builder.capture_seconds = known.capture_seconds
    if known.run_id:
        builder.run_id = known.run_id

    if builder.capture_seconds > 0.0:
        def delayed_start():
            time.sleep(builder.start_delay)
            builder.start_capture(builder.capture_seconds)
            while rclpy.ok() and not builder.done:
                time.sleep(0.2)
            rclpy.shutdown()

        import threading

        threading.Thread(target=delayed_start, daemon=True).start()

    try:
        rclpy.spin(builder)
    except KeyboardInterrupt:
        builder.stop_capture("keyboard_interrupt")
    finally:
        if not builder.done:
            builder.stop_capture("shutdown")
        builder.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
