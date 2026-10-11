#!/usr/bin/env python3
"""先验点云地图离线重建工具（在地图采集时不在 ROS 回调里做重活）。

背景：在 ``gicp_prior_map_builder`` 的 ROS 回调里直接做体素合并，在 20 Hz × 6 万点/
帧的数据量下会把执行器阻塞到几十毫秒~秒级，导致后续帧被丢弃、点云话题速率塌到 1 Hz
以下。这里改成两段式：

1. 采集阶段（``gicp_map_recorder``）只做「按时间戳配对 + 落盘 npy」；
2. 本工具离线重建：读 npy → 按真值位姿变换到地图系 → 体素下采样 → 导出 PCD/栅格/清单。

用法::

    python3 rebuild_prior_map.py --session tmp/gicp_validation/map_session \\
        --output-dir tmp/gicp_validation --map-name promap \\
        --spawn-x 3.4 --spawn-y 9.5 --spawn-yaw 0.0
"""

import argparse
import glob
import hashlib
import json
import math
import os
import subprocess
from datetime import datetime

import numpy as np
import yaml

import sys

# 允许直接从源码树运行：把「包的父目录」加进 sys.path，才能 import 包本身
sys.path.insert(
    0, os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
)

from srm27_gicp_validation.gicp_validation_common import (  # noqa: E402
    bounding_box,
    invert,
    transform_from_pose,
    voxel_downsample,
    yaw_from_matrix,
)


def sha256_file(path: str) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_pcd_binary(path: str, points: np.ndarray) -> None:
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
    rows, cols = grid.shape
    header = f"P5\n# GICP validation prior map\n{cols} {rows}\n255\n"
    with open(path, "wb") as stream:
        stream.write(header.encode("ascii"))
        stream.write(np.ascontiguousarray(grid, dtype=np.uint8).tobytes())


def write_grid(points: np.ndarray, resolution: float, min_z: float, max_z: float, pgm: str, yaml_path: str):
    mask = (points[:, 2] >= min_z) & (points[:, 2] <= max_z)
    planar = points[mask][:, :2]
    if planar.shape[0] == 0:
        raise RuntimeError("栅格区间内没有点")
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
    grid[rows - 1 - row, col] = 0
    write_pgm(pgm, grid)
    with open(yaml_path, "w", encoding="utf-8") as stream:
        stream.write(f"image: {os.path.basename(pgm)}\n")
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
    }


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="离线重建先验点云地图与栅格")
    parser.add_argument("--session", required=True, help="gicp_map_recorder 的会话目录")
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--map-name", default="promap")
    parser.add_argument("--leaf-size", type=float, default=0.10)
    parser.add_argument("--grid-resolution", type=float, default=0.05)
    parser.add_argument("--grid-min-z", type=float, default=0.15)
    parser.add_argument("--grid-max-z", type=float, default=1.6)
    parser.add_argument("--spawn-x", type=float, default=3.4)
    parser.add_argument("--spawn-y", type=float, default=9.5)
    parser.add_argument("--spawn-yaw", type=float, default=0.0)
    parser.add_argument("--world-name", default="rmuc_2025")
    parser.add_argument("--world-sdf", default="")
    parser.add_argument("--run-id", default="")
    known = parser.parse_args(argv)

    spawn = transform_from_pose(known.spawn_x, known.spawn_y, 0.0, known.spawn_yaw)
    t_map_world = invert(spawn)

    frames = sorted(glob.glob(os.path.join(known.session, "frames", "*.npz")))
    if not frames:
        raise SystemExit(f"会话目录里没有帧：{known.session}/frames/*.npz")
    print(f"[rebuild] 读取 {len(frames)} 帧")

    chunks = []
    raw_points = 0
    trajectory = []
    for index, path in enumerate(frames):
        data = np.load(path)
        points = data["points"]
        odom = data["odom"]
        raw_points += points.shape[0]
        trajectory.append(odom[:3, 3].copy())
        transform = t_map_world @ odom
        mapped = points @ transform[:3, :3].T + transform[:3, 3]
        chunks.append(mapped.astype(np.float32))
        if (index + 1) % 200 == 0:
            print(f"[rebuild]   已处理 {index + 1}/{len(frames)} 帧")

    raw = np.vstack(chunks)
    print(f"[rebuild] 原始点 {raw.shape[0]}")
    points = voxel_downsample(raw, known.leaf_size)
    print(f"[rebuild] 体素下采样({known.leaf_size} m) 后 {points.shape[0]} 点")

    os.makedirs(known.output_dir, exist_ok=True)
    base = os.path.join(known.output_dir, known.map_name)
    prior_pcd = base + ".pcd"
    grid_pgm = base + ".pgm"
    grid_yaml = base + ".yaml"
    manifest_path = base + "_manifest.yaml"

    write_pcd_binary(prior_pcd, points)
    grid_info = write_grid(
        points, known.grid_resolution, known.grid_min_z, known.grid_max_z, grid_pgm, grid_yaml
    )

    trajectory = np.asarray(trajectory)
    manifest = {
        "map_name": known.map_name,
        "run_id": known.run_id or datetime.now().strftime("%Y%m%d_%H%M%S"),
        "generated_at": datetime.now().isoformat(timespec="seconds"),
        "source_session": known.session,
        "world": {
            "name": known.world_name,
            "sdf": known.world_sdf,
            "sdf_sha256": (
                sha256_file(known.world_sdf)
                if known.world_sdf and os.path.isfile(known.world_sdf)
                else ""
            ),
        },
        "map_origin": {
            "mode": "spawn",
            "spawn_pose_world": {
                "x": known.spawn_x,
                "y": known.spawn_y,
                "z": 0.0,
                "yaw": known.spawn_yaw,
            },
            "T_M_W": [[float(v) for v in row] for row in t_map_world],
            "note": "T_M_W = inverse(T_W_B(spawn))；所有验证轮次共用同一个值",
        },
        "pcd_contract": {
            "file_frame": "lidar_frame（front_mid360）原始点",
            "formula": "p_file = p_L；p_loaded = E · p_file = T_B_L · p_L = p_M",
            "E_definition": "E = T_B_L，由 GICP loadGlobalMap() 用 base_frame ← lidar_frame 查询",
            "warning": "不要预先应用 T_B_L 后写盘，否则加载后地图会被再变换一次",
        },
        "artifacts": {
            "prior_pcd": prior_pcd,
            "prior_pcd_sha256": sha256_file(prior_pcd),
            "prior_pcd_points": int(points.shape[0]),
            "occupancy_pgm": grid_pgm,
            "occupancy_pgm_sha256": sha256_file(grid_pgm),
            "occupancy_yaml": grid_yaml,
            "occupancy_yaml_sha256": sha256_file(grid_yaml),
        },
        "occupancy_grid": grid_info,
        "capture": {
            "frames": len(frames),
            "raw_points": int(raw_points),
            "leaf_size": known.leaf_size,
            "trajectory_min": trajectory.min(axis=0).tolist(),
            "trajectory_max": trajectory.max(axis=0).tolist(),
            "trajectory_span": (trajectory.max(axis=0) - trajectory.min(axis=0)).tolist(),
        },
        "point_stats_map_frame": bounding_box(points),
    }
    with open(manifest_path, "w", encoding="utf-8") as stream:
        yaml.safe_dump(manifest, stream, allow_unicode=True, sort_keys=False)

    print(
        json.dumps(
            {
                "prior_pcd": prior_pcd,
                "points": int(points.shape[0]),
                "manifest": manifest_path,
                "trajectory_span": manifest["capture"]["trajectory_span"],
                "xyz_min": manifest["point_stats_map_frame"]["min"],
                "xyz_max": manifest["point_stats_map_frame"]["max"],
            },
            ensure_ascii=False,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
