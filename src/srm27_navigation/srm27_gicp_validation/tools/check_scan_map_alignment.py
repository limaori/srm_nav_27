#!/usr/bin/env python3
"""离线检查：``registered_scan`` 与先验地图是否真的对齐（方案 §4.2 的地标核对）。

在 (dx, dy, dyaw) 上做粗搜索，用「扫描点到地图的最近邻距离中位数」评分，
最优解的残差直接告诉我们两张图是否同源、同原点、同朝向。

用法（需要链路在同一个 bash 调用里运行）::

    python3 check_scan_map_alignment.py --pcd tmp/gicp_validation/promap.pcd
"""

import argparse
import os
import sys
import time

import numpy as np
import rclpy
from scipy.spatial import cKDTree
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

sys.path.insert(
    0,
    os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "srm27_gicp_validation"
    ),
)


def read_pcd(path: str) -> np.ndarray:
    data = open(path, "rb").read()
    header_end = data.index(b"DATA binary\n") + len(b"DATA binary\n")
    return np.frombuffer(data[header_end:], dtype=np.float32).reshape(-1, 3).astype(np.float64)


def read_xyz(msg: PointCloud2) -> np.ndarray:
    field_map = {field.name: field for field in msg.fields}
    offsets = [field_map[name].offset for name in ("x", "y", "z")]
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    count = raw.size // msg.point_step
    raw = raw[: count * msg.point_step].reshape(count, msg.point_step)
    columns = [
        raw[:, offset : offset + 4].copy().view(np.float32).reshape(-1).astype(np.float64)
        for offset in offsets
    ]
    points = np.stack(columns, axis=1)
    return points[np.isfinite(points).all(axis=1)]


class Grabber(Node):
    def __init__(self) -> None:
        super().__init__("alignment_grabber")
        self.cloud = None
        self.create_subscription(
            PointCloud2,
            "/red_standard_robot1/registered_scan",
            self.on_cloud,
            qos_profile_sensor_data,
        )

    def on_cloud(self, msg: PointCloud2) -> None:
        self.cloud = read_xyz(msg)


def median_nn_distance(source: np.ndarray, tree: cKDTree, sample: int = 2000) -> float:
    """source 每个点到地图的最近邻距离中位数（用 KD 树，避免 O(N·M) 内存爆炸）。"""
    if source.shape[0] > sample:
        step = source.shape[0] // sample
        source = source[::step]
    distance, _ = tree.query(source, k=1, workers=4)
    return float(np.median(distance))


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="registered_scan 与先验地图对齐检查")
    parser.add_argument("--pcd", required=True)
    parser.add_argument("--map-leaf", type=float, default=0.5, help="地图抽稀步长（米），加速搜索")
    parser.add_argument("--range", type=float, default=25.0, help="只用地图上距原点该范围内的点")
    known, remaining = parser.parse_known_args(argv)

    rclpy.init(args=remaining)
    node = Grabber()
    deadline = time.monotonic() + 20
    while time.monotonic() < deadline and node.cloud is None:
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.cloud is None:
        print("没有收到 registered_scan")
        return 1
    scan = node.cloud
    print(f"扫描点数 {scan.shape[0]}；范围 x[{scan[:,0].min():.2f},{scan[:,0].max():.2f}] "
          f"y[{scan[:,1].min():.2f},{scan[:,1].max():.2f}] z[{scan[:,2].min():.2f},{scan[:,2].max():.2f}]")

    cloud = read_pcd(known.pcd)
    print(f"地图点数 {cloud.shape[0]}")
    # 抽稀地图以加速
    keys = np.floor(cloud / known.map_leaf).astype(np.int64)
    _, index = np.unique(keys, axis=0, return_index=True)
    target = cloud[np.sort(index)]
    target = target[np.linalg.norm(target[:, :2], axis=1) < known.range]
    print(f"搜索用地图点数（leaf={known.map_leaf}，|xy|<{known.range}）{target.shape[0]}")

    # 只保留扫描里距离原点 <20 m 的点，避免远端稀疏区噪声
    scan_used = scan[np.linalg.norm(scan[:, :2], axis=1) < 20.0]
    print(f"参与评分的扫描点数 {scan_used.shape[0]}")
    # 预抽稀一份更粗的地图用于快速粗搜索
    coarse_keys = np.floor(cloud / 1.0).astype(np.int64)
    _, coarse_index = np.unique(coarse_keys, axis=0, return_index=True)
    coarse = cloud[np.sort(coarse_index)]
    coarse = coarse[np.linalg.norm(coarse[:, :2], axis=1) < known.range]
    tree = cKDTree(coarse)
    print(f"粗搜索地图点数 {coarse.shape[0]}")

    def score(dx: float, dy: float, dyaw_deg: float) -> float:
        theta = np.radians(dyaw_deg)
        c, s = np.cos(theta), np.sin(theta)
        rotation = np.array([[c, -s], [s, c]])
        rotated = scan_used[:, :2] @ rotation.T + np.array([dx, dy])
        moved = np.hstack([rotated, scan_used[:, 2:3]])
        return median_nn_distance(moved, tree)

    print("\n== (0,0,0) 附近的粗搜索 ==")
    best = None
    for dyaw in range(-10, 11, 2):
        for dx in (-1.0, -0.5, 0.0, 0.5, 1.0):
            for dy in (-1.0, -0.5, 0.0, 0.5, 1.0):
                value = score(dx, dy, dyaw)
                if best is None or value < best[0]:
                    best = (value, dx, dy, dyaw)
    print(f"最优：中位最近邻 {best[0]:.4f} m @ dx={best[1]} dy={best[2]} dyaw={best[3]}°")

    print("\n== 关键假设检验：GICP 加载地图时施加的外参 E 到底是哪一个 ==")
    print("（若某个候选让中位最近邻接近 0，说明它才是加载器真正使用的变换）")
    sys.path.insert(
        0,
        os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "srm27_gicp_validation",
        ),
    )
    from gicp_static_tf_republisher import rpy_to_matrix  # noqa: E402
    from gicp_validation_common import invert, make_transform  # noqa: E402

    xyz = [0.15, -0.15, 0.22]
    rpy = [-0.06981317007977318, 0.0, -1.5707963267948966]
    e_base_lidar = make_transform(xyz, rpy_to_matrix(*rpy))
    e_lidar_base = invert(e_base_lidar)
    candidates = {
        "I": np.eye(4),
        "R_off only (=T_B_L 旋转)": make_transform([0, 0, 0], rpy_to_matrix(*rpy)),
        "t_off only": make_transform(xyz, np.eye(3)),
        "T_L_B = T_B_L^-1": e_lidar_base,
        "T_B_L": e_base_lidar,
        "R_off^-1": make_transform([0, 0, 0], rpy_to_matrix(*rpy).T),
    }
    for name, extrinsic in candidates.items():
        moved = cloud @ extrinsic[:3, :3].T + extrinsic[:3, 3]
        keys = np.floor(moved / 1.0).astype(np.int64)
        _, index = np.unique(keys, axis=0, return_index=True)
        slimmer = moved[np.sort(index)]
        slimmer = slimmer[np.linalg.norm(slimmer[:, :2], axis=1) < known.range]
        candidate_tree = cKDTree(slimmer)
        value = median_nn_distance(
            np.hstack([scan_used[:, :2], scan_used[:, 2:3]]), candidate_tree
        )
        print(f"  {name:28s} → 中位最近邻 {value:.4f} m（地图点数 {slimmer.shape[0]}）")

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
