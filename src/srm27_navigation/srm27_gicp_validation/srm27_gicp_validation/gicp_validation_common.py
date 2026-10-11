"""GICP 初始重定位验证的公共数学与几何工具。

坐标约定（与 docs/GICP初始重定位仿真验证方案.md §4 一致）：

* ``T_A_B`` 表示把 B 系点变换到 A 系的刚体变换，即 ``p_A = T_A_B · p_B``。
* 真值里程计 ``chassis_odometry_gt`` 给出世界系位姿 ``T_W_B``。
* 地图原点由 ``T_M_W`` 固定。方案 §4.1 要求所有轮次使用同一个 ``T_M_W``；
  本工程的验证配置把它固定为「出生点即地图原点」，因此默认
  ``T_M_W = inverse(T_W_B(spawn))``。
* **PCD 坐标契约**：GICP 的 ``loadGlobalMap()`` 读入 PCD 后会再左乘
  ``E = T_B_L``（``base_frame`` ← ``lidar_frame``）。因此文件里的点必须满足
  ``p_file = E⁻¹ · p_M``，加载后才是方案要求的 ``p_M``。
  对单帧采样点 ``T_B_L · p_L``（p_L 是**雷达系**的原始点）而言，
  ``p_file = p_L``：即直接保存雷达系原始点云即可，**不要**预先应用外参。
"""

import math
from typing import Iterable, Sequence, Tuple

import numpy as np


def rotation_z(yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)


def make_transform(
    translation: Sequence[float], rotation: np.ndarray
) -> np.ndarray:
    """由平移与 3x3 旋转构造 4x4 齐次变换。"""
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = np.asarray(rotation, dtype=np.float64)
    transform[:3, 3] = np.asarray(translation, dtype=np.float64)
    return transform


def transform_from_pose(
    x: float, y: float, z: float, yaw: float = 0.0
) -> np.ndarray:
    """平面位姿（x, y, z, yaw）对应的 4x4 变换。"""
    return make_transform([x, y, z], rotation_z(yaw))


def quaternion_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """四元数（xyzw）转 3x3 旋转矩阵。"""
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        raise ValueError("degenerate quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def matrix_to_quaternion(rotation: np.ndarray) -> Tuple[float, float, float, float]:
    """3x3 旋转矩阵转四元数（xyzw，w >= 0）。"""
    m = np.asarray(rotation, dtype=np.float64)
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    if w < 0.0:
        x, y, z, w = -x, -y, -z, -w
    return x, y, z, w


def yaw_from_matrix(rotation: np.ndarray) -> float:
    """3x3 旋转矩阵的 yaw。"""
    return math.atan2(rotation[1, 0], rotation[0, 0])


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap_to_pi(angle: float) -> float:
    """把角度折算到 (-pi, pi]。"""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def angle_diff(a: float, b: float) -> float:
    """最短角距离 |a - b|，结果落在 [0, pi]。"""
    return abs(wrap_to_pi(a - b))


def planar_error(est: np.ndarray, gt: np.ndarray) -> Tuple[float, float, float]:
    """返回 (e_xy, e_yaw, e_z)。"""
    e_xy = float(np.linalg.norm(est[:2, 3] - gt[:2, 3]))
    e_yaw = angle_diff(yaw_from_matrix(est[:3, :3]), yaw_from_matrix(gt[:3, :3]))
    e_z = float(abs(est[2, 3] - gt[2, 3]))
    return e_xy, e_yaw, e_z


def invert(transform: np.ndarray) -> np.ndarray:
    rotation = transform[:3, :3]
    translation = transform[:3, 3]
    inverse = np.eye(4, dtype=np.float64)
    inverse[:3, :3] = rotation.T
    inverse[:3, 3] = -rotation.T @ translation
    return inverse


def voxel_downsample(points: np.ndarray, leaf: float) -> np.ndarray:
    """体素下采样（每格保留第一点），返回 (N, 3)。"""
    if points.size == 0 or leaf <= 0.0:
        return points
    keys = np.floor(points / leaf).astype(np.int64)
    _, index = np.unique(keys, axis=0, return_index=True)
    return points[np.sort(index)]


def downsample_points(points: np.ndarray, keep_ratio: float, seed: int) -> np.ndarray:
    """按比例随机保留点（方案 §7 S3）。"""
    if keep_ratio >= 1.0 or points.shape[0] == 0:
        return points
    count = max(1, int(round(points.shape[0] * keep_ratio)))
    rng = np.random.default_rng(seed)
    index = rng.choice(points.shape[0], size=count, replace=False)
    index.sort()
    return points[index]


def add_noise(points: np.ndarray, sigma: float, seed: int) -> np.ndarray:
    """给点云加高斯噪声（方案 §7 S3）。"""
    if sigma <= 0.0 or points.shape[0] == 0:
        return points
    rng = np.random.default_rng(seed)
    return points + rng.normal(0.0, sigma, size=points.shape)


def mask_horizontal_sector(
    points: np.ndarray, center_angle_deg: float, half_width_deg: float
) -> np.ndarray:
    """按水平扇区遮挡点云（方案 §7 S4）。角度按雷达/底盘 x 轴正方向量测。"""
    if points.shape[0] == 0:
        return points
    angles = np.degrees(np.arctan2(points[:, 1], points[:, 0]))
    delta = np.abs((angles - center_angle_deg + 180.0) % 360.0 - 180.0)
    return points[delta > half_width_deg]


def bounding_box(points: np.ndarray) -> dict:
    if points.shape[0] == 0:
        return {"min": [0.0, 0.0, 0.0], "max": [0.0, 0.0, 0.0], "count": 0}
    return {
        "min": [float(v) for v in points.min(axis=0)],
        "max": [float(v) for v in points.max(axis=0)],
        "count": int(points.shape[0]),
    }


def percentile(values: Iterable[float], q: float) -> float:
    data = np.asarray(list(values), dtype=np.float64)
    if data.size == 0:
        return float("nan")
    return float(np.percentile(data, q))
