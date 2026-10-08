# Copyright 2026 Shiyu
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

"""srm_nav_pose_math 的几何回归测试（坡道修复方案第一批验收项 5）。

验收项：单位变换、非零外参、90° yaw、±10° pitch、roll/pitch/yaw 组合、
非零初始 yaw、高度变化；以及"静态场景的点云世界坐标在上/下坡过程中保持一致"
和"同一个雷达位姿经 TF 与直接组合得到相同结果"。

参照实现刻意使用**旋转矩阵**（不经过四元数），这样参照与被测代码不是同一套公式，
否则历史缺陷（把四元数分量当平移代入叉乘）会在两边同时出错而测不出来。
"""

import math
import random
import sys
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from srm_nav_pose_math import (  # noqa: E402 - 依赖上面的 sys.path
    compose_pose,
    identity_pose,
    inverse_pose,
    normalize_angle,
    rotate_vector,
    rpy_to_quaternion,
    transform_point,
    yaw_from_quaternion,
)

TOLERANCE = 1.0e-9

# SRM 雷达外参（srm27_robot_description 几何 YAML 的 base_link -> front_mid360）。
LIDAR_XYZ = (0.15, -0.15, 0.22)
LIDAR_RPY = (-0.06981317007977318, 0.0, -1.5707963267948966)

# 坡面上实测到的姿态：横滚约 4.36°、俯仰约 -9.01°。
RAMP_ROLL = math.radians(4.36)
RAMP_PITCH = math.radians(-9.01)


def _rotation_matrix(roll, pitch, yaw):
    """独立的 ZYX 旋转矩阵实现（测试参照，不经过四元数）。"""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def _matrix_apply(matrix, vector):
    """矩阵左乘列向量。"""
    return tuple(
        sum(matrix[row][column] * vector[column] for column in range(3))
        for row in range(3)
    )


def _assert_close(actual, expected, tolerance, message):
    """逐分量比较两个序列。"""
    assert len(actual) == len(expected), f"{message}: 长度不一致"
    for index, (got, want) in enumerate(zip(actual, expected)):
        assert abs(got - want) <= tolerance, (
            f"{message}: 分量 {index} 期望 {want!r}，实际 {got!r}（容差 {tolerance}）"
        )


def _pose_close(actual, expected, tolerance, message):
    """比较两个位姿（位置与四元数都允许多一个整体符号）。"""
    _assert_close(actual[0], expected[0], tolerance, f"{message}: 位置")
    same = all(abs(a - b) <= tolerance for a, b in zip(actual[1], expected[1]))
    flipped = all(abs(a + b) <= tolerance for a, b in zip(actual[1], expected[1]))
    assert same or flipped, f"{message}: 姿态期望 {expected[1]!r}，实际 {actual[1]!r}"


def _lidar_pose_from_chassis(chassis_pose):
    """底盘位姿 ∘ 雷达外参；这是 TF 链路与直接组合必须给出同一结果的量。"""
    extrinsic = (LIDAR_XYZ, rpy_to_quaternion(*LIDAR_RPY))
    return compose_pose(chassis_pose, extrinsic)


def test_identity_composition_is_identity():
    pose = identity_pose()
    assert compose_pose(pose, pose) == pose
    assert compose_pose(inverse_pose(pose), pose) == pose


def test_lidar_pose_matches_matrix_reference():
    """雷达位姿必须等于"底盘旋转矩阵 × 外参平移 + 底盘平移"的独立矩阵结果。"""
    cases = [
        (0.0, 0.0, 0.0),
        (0.0, 0.0, math.pi / 2.0),
        (RAMP_ROLL, 0.0, 0.3),
        (0.0, RAMP_PITCH, -1.2),
        (RAMP_ROLL, RAMP_PITCH, 2.4),
        (-0.2, 0.35, -2.9),
    ]
    for roll, pitch, yaw in cases:
        chassis_position = (3.5, -2.25, 0.4)
        chassis_pose = (chassis_position, rpy_to_quaternion(roll, pitch, yaw))
        lidar_pose = _lidar_pose_from_chassis(chassis_pose)

        # 独立的矩阵链路：p_lidar = p_chassis + R_chassis · t_extrinsic
        rotated = _matrix_apply(_rotation_matrix(roll, pitch, yaw), LIDAR_XYZ)
        expected_position = tuple(
            chassis_position[index] + rotated[index] for index in range(3)
        )
        _assert_close(
            lidar_pose[0],
            expected_position,
            1.0e-9,
            f"雷达位姿平移 (rpy={roll:.4f},{pitch:.4f},{yaw:.4f})",
        )

        # 朝向：q_lidar = q_chassis ⊗ q_extrinsic。用矩阵比较旋转作用效果。
        matrix_chassis = _rotation_matrix(roll, pitch, yaw)
        matrix_extrinsic = _rotation_matrix(*LIDAR_RPY)
        composed_by_matrix = tuple(
            tuple(
                sum(
                    matrix_chassis[row][k] * matrix_extrinsic[k][column]
                    for k in range(3)
                )
                for column in range(3)
            )
            for row in range(3)
        )
        probe = (0.31, -0.47, 0.83)
        expected_rotated = _matrix_apply(composed_by_matrix, probe)
        _assert_close(
            rotate_vector(lidar_pose[1], probe),
            expected_rotated,
            1.0e-9,
            f"雷达位姿朝向 (rpy={roll:.4f},{pitch:.4f},{yaw:.4f})",
        )


def test_ninety_degree_yaw_reproduces_the_documented_defect():
    """方案里记录的最小复现：底盘转 90° 时外参平移必须跟着转。"""
    chassis_pose = ((0.0, 0.0, 0.0), rpy_to_quaternion(0.0, 0.0, math.pi / 2.0))
    offset_only = ((0.15, -0.15, 0.22), (0.0, 0.0, 0.0, 1.0))
    composed = compose_pose(chassis_pose, offset_only)

    _assert_close(composed[0], (0.15, 0.15, 0.22), TOLERANCE, "90° yaw 外参平移")

    # 缺陷实现返回未旋转的平移，两者相差 0.30 m；这里把它写成显式断言，
    # 防止将来又退回"用第二个变换的四元数分量当平移"的写法。
    defective = (0.15, -0.15, 0.22)
    distance = math.dist(composed[0], defective)
    assert abs(distance - 0.30) < 1.0e-9, f"缺陷差值应约 0.30 m，实际 {distance}"


def test_ten_degree_pitch_propagation():
    """±10° 俯仰必须作用到外参平移上（上/下坡对应正负号）。"""
    chassis_position = (1.0, 2.0, 0.25)
    for pitch_degrees in (10.0, -10.0):
        pitch = math.radians(pitch_degrees)
        chassis_pose = (chassis_position, rpy_to_quaternion(0.0, pitch, 0.0))
        lidar_pose = _lidar_pose_from_chassis(chassis_pose)
        rotated = _matrix_apply(_rotation_matrix(0.0, pitch, 0.0), LIDAR_XYZ)
        expected = tuple(
            chassis_position[index] + rotated[index] for index in range(3)
        )
        _assert_close(lidar_pose[0], expected, 1.0e-9, f"pitch={pitch_degrees}° 雷达平移")
        # 俯仰会改变雷达高度：外参 z 分量必须随俯仰倾斜，而不是恒定 0.22 m。
        assert abs(lidar_pose[0][2] - (chassis_position[2] + LIDAR_XYZ[2])) > 1.0e-3, (
            f"pitch={pitch_degrees}° 时雷达高度未被俯仰影响：{lidar_pose[0][2]}"
        )


def test_composition_matches_matrix_reference_over_random_poses():
    """组合结果必须与独立的矩阵链路一致（含平移与旋转）。"""
    random.seed(20261008)
    matrices = []
    for _ in range(500):
        first_rpy = [random.uniform(-1.4, 1.4) for _ in range(3)]
        second_rpy = [random.uniform(-1.4, 1.4) for _ in range(3)]
        first_position = tuple(random.uniform(-5.0, 5.0) for _ in range(3))
        second_position = tuple(random.uniform(-3.0, 3.0) for _ in range(3))
        first = (first_position, rpy_to_quaternion(*first_rpy))
        second = (second_position, rpy_to_quaternion(*second_rpy))
        composed = compose_pose(first, second)

        rotated = _matrix_apply(_rotation_matrix(*first_rpy), second_position)
        expected_position = tuple(
            first_position[index] + rotated[index] for index in range(3)
        )
        _assert_close(composed[0], expected_position, 1.0e-9, "随机组合平移")

        expected_matrix = tuple(
            tuple(
                sum(
                    _rotation_matrix(*first_rpy)[row][k]
                    * _rotation_matrix(*second_rpy)[k][column]
                    for k in range(3)
                )
                for column in range(3)
            )
            for row in range(3)
        )
        probe = (0.17, -0.29, 0.41)
        _assert_close(
            rotate_vector(composed[1], probe),
            _matrix_apply(expected_matrix, probe),
            1.0e-9,
            "随机组合旋转",
        )
        matrices.append(composed)
    assert len(matrices) == 500


def test_nonzero_initial_yaw_keeps_height_and_tilt():
    """非零初始 yaw：odom 系仍重力对齐，高度与俯仰原样保留。"""
    initial_yaw = 1.1
    initial_world = (12.0, -4.5, 0.2)
    odom_frame_pose = (initial_world, rpy_to_quaternion(0.0, 0.0, initial_yaw))

    # 底盘在坡上：相对出生点前进 1.6 m、升高 0.28 m、俯仰 -9.01°、横滚 4.36°。
    world_pose = (
        (
            initial_world[0] + 1.6 * math.cos(initial_yaw),
            initial_world[1] + 1.6 * math.sin(initial_yaw),
            initial_world[2] + 0.28,
        ),
        rpy_to_quaternion(RAMP_ROLL, RAMP_PITCH, initial_yaw),
    )
    base_pose = compose_pose(inverse_pose(odom_frame_pose), world_pose)

    _assert_close(base_pose[0], (1.6, 0.0, 0.28), 1.0e-9, "非零初始 yaw 相对位姿")
    _assert_close(
        (yaw_from_quaternion(base_pose[1]),),
        (0.0,),
        1.0e-9,
        "非零初始 yaw 相对航向应回到 0",
    )
    # 俯仰/横滚不随初始航向被抹掉：用旋转作用效果核对。
    probe = (0.0, 0.0, 1.0)
    _assert_close(
        rotate_vector(base_pose[1], probe),
        _matrix_apply(_rotation_matrix(RAMP_ROLL, RAMP_PITCH, 0.0), probe),
        1.0e-9,
        "非零初始 yaw 相对姿态",
    )


def test_static_world_point_is_invariant_across_attitudes():
    """验收项：静态场景的点云世界坐标在上/下坡过程中保持一致。

    构造：世界系里固定一点 P；用真实位姿把它换算到雷达系（等价于 Gazebo 给出的
    点云），再用"已发布的 odom -> 雷达位姿"换算回 odom，最后用 odom -> 世界变换
    还原。只要整条链路的几何一致，还原结果必须等于 P；车姿态怎么变都不影响。
    """
    initial_world = (5.45, -3.39, 0.0)
    initial_yaw = -0.6
    odom_frame_pose = (initial_world, rpy_to_quaternion(0.0, 0.0, initial_yaw))
    world_point = (7.2, -1.4, 0.35)  # 世界系里的固定静态点

    attitudes = [
        (0.0, 0.0, 0.0, 0.0),
        (0.0, 0.0, math.radians(10.0), 0.15),
        (0.0, 0.0, math.radians(-10.0), 0.15),
        (RAMP_ROLL, RAMP_PITCH, 0.0, 0.40),
        (RAMP_ROLL, RAMP_PITCH, math.radians(90.0), 0.55),
        (-0.1, math.radians(6.0), math.radians(-140.0), 0.30),
    ]
    for roll, pitch, delta_yaw, height in attitudes:
        yaw = initial_yaw + delta_yaw
        world_pose = (
            (initial_world[0], initial_world[1], initial_world[2] + height),
            rpy_to_quaternion(roll, pitch, yaw),
        )
        # 真实的世界 -> 雷达位姿（点云就是这样产生的）。
        world_to_lidar = compose_pose(
            world_pose, (LIDAR_XYZ, rpy_to_quaternion(*LIDAR_RPY))
        )
        point_in_lidar = transform_point(inverse_pose(world_to_lidar), world_point)

        # 链路输出：odom -> 底盘（本适配器发布的 TF）与 odom -> 雷达（lidar_odometry）。
        odom_to_base = compose_pose(inverse_pose(odom_frame_pose), world_pose)
        odom_to_lidar = _lidar_pose_from_chassis(odom_to_base)

        point_in_odom = transform_point(odom_to_lidar, point_in_lidar)
        point_back_in_world = transform_point(odom_frame_pose, point_in_odom)
        _assert_close(
            point_back_in_world,
            world_point,
            1.0e-9,
            f"静态点回环 (roll={roll:.3f}, pitch={pitch:.3f}, dyaw={delta_yaw:.3f})",
        )


def test_yaw_extraction_and_angle_normalization():
    for yaw in (-3.0, -1.0, 0.0, 0.7, 2.9):
        quaternion = rpy_to_quaternion(0.12, -0.2, yaw)
        assert abs(yaw_from_quaternion(quaternion) - yaw) < 1.0e-9
    assert abs(normalize_angle(3.0 * math.pi) - math.pi) < 1.0e-9
    assert abs(normalize_angle(-3.0 * math.pi) + math.pi) < 1.0e-9


def _run_all():
    """直接以脚本方式运行全部用例，避免依赖 pytest 插件环境。"""
    failures = 0
    for name, function in sorted(globals().items()):
        if not name.startswith("test_") or not callable(function):
            continue
        try:
            function()
        except Exception as error:  # noqa: BLE001 - 测试入口需要报告任意失败
            failures += 1
            print(f"FAIL {name}: {error!r}")
        else:
            print(f"PASS {name}")
    return failures


if __name__ == "__main__":
    sys.exit(1 if _run_all() else 0)
