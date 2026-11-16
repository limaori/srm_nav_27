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

"""导航位姿几何：四元数与刚体变换的最小实现。

位姿统一表示为 ``(position, orientation)`` 二元组：

* ``position``    = ``(x, y, z)``
* ``orientation`` = ``(qx, qy, qz, qw)``，单位四元数

``compose_pose(first, second)`` 计算 ``T_first ∘ T_second``，即把第二个变换表达的
坐标写到第一个变换的父坐标系下：

* 位置：``p = p_first + R(q_first) · p_second``
* 朝向：``q = q_first ⊗ q_second``

**平移只允许使用 second 的平移参与 first 的旋转**，旋转用四元数乘法，两者必须是
互相独立的变量。真值里程计适配器历史上把 second 的四元数分量当成平移代入叉乘，
底盘转 90° 时雷达外参平移完全不跟着转（最小复现差 0.30 m）。

本模块只依赖标准库，因此可以在没有 ROS 的环境里做几何回归测试。
"""

import math


def identity_pose():
    """返回单位位姿（原点 + 单位四元数）。

    Returns:
        tuple: ``((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))``
    """
    return ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))


def normalize_quaternion(quaternion):
    """归一化四元数；模长退化为 0 时返回单位四元数。

    Args:
        quaternion: ``(qx, qy, qz, qw)``

    Returns:
        tuple: 归一化后的 ``(qx, qy, qz, qw)``
    """
    qx, qy, qz, qw = quaternion
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm < 1.0e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return (qx / norm, qy / norm, qz / norm, qw / norm)


def quaternion_multiply(first, second):
    """四元数乘法（Hamilton 积）：``q_first ⊗ q_second``。

    Args:
        first: ``(qx, qy, qz, qw)``
        second: ``(qx, qy, qz, qw)``

    Returns:
        tuple: ``(qx, qy, qz, qw)``
    """
    ax, ay, az, aw = first
    bx, by, bz, bw = second
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quaternion_inverse(quaternion):
    """单位四元数的逆（等于共轭）。

    Args:
        quaternion: ``(qx, qy, qz, qw)``

    Returns:
        tuple: ``(qx, qy, qz, qw)``
    """
    qx, qy, qz, qw = normalize_quaternion(quaternion)
    return (-qx, -qy, -qz, qw)


def rotate_vector(quaternion, vector):
    """用单位四元数旋转三维向量：``v' = q ⊗ (0, v) ⊗ q⁻¹``。

    这里刻意走四元数三明治乘积而不是手写展开式：展开式里的叉乘项正是历史上
    ``_compose`` 出错的地方，复用 ``quaternion_multiply`` 只有一个实现需要验证。

    Args:
        quaternion: ``(qx, qy, qz, qw)``
        vector: ``(x, y, z)``

    Returns:
        tuple: 旋转后的 ``(x, y, z)``
    """
    unit = normalize_quaternion(quaternion)
    pure = (vector[0], vector[1], vector[2], 0.0)
    # 结果仍是纯四元数，取向量部分 (x, y, z)，标量部分理论为 0。
    rx, ry, rz, _ = quaternion_multiply(
        quaternion_multiply(unit, pure), quaternion_inverse(unit)
    )
    return (rx, ry, rz)


def compose_pose(first, second):
    """组合刚体变换：``T_out = T_first ∘ T_second``。

    平移用 ``second`` 的平移参与 ``first`` 的旋转，旋转用四元数乘法；两者使用互相
    独立的变量（历史缺陷就是把 second 的四元数分量当成了平移）。

    Args:
        first: ``(position, orientation)``
        second: ``(position, orientation)``

    Returns:
        tuple: 组合后的 ``(position, orientation)``
    """
    first_position, first_orientation = first
    second_position, second_orientation = second
    rotated = rotate_vector(first_orientation, second_position)
    position = (
        first_position[0] + rotated[0],
        first_position[1] + rotated[1],
        first_position[2] + rotated[2],
    )
    orientation = normalize_quaternion(
        quaternion_multiply(first_orientation, second_orientation)
    )
    return (position, orientation)


def transform_point(pose, point):
    """把一个点从子坐标系变换到父坐标系：``p' = t + R·p``。

    Args:
        pose: ``(position, orientation)``
        point: ``(x, y, z)``

    Returns:
        tuple: ``(x, y, z)``
    """
    position, orientation = pose
    rotated = rotate_vector(orientation, point)
    return (
        position[0] + rotated[0],
        position[1] + rotated[1],
        position[2] + rotated[2],
    )


def inverse_pose(pose):
    """求刚体变换的逆：``T⁻¹ = (Rᵀ, -Rᵀ·t)``。

    Args:
        pose: ``(position, orientation)``

    Returns:
        tuple: 逆位姿 ``(position, orientation)``
    """
    position, orientation = pose
    inverse_orientation = quaternion_inverse(orientation)
    rotated = rotate_vector(inverse_orientation, position)
    return ((-rotated[0], -rotated[1], -rotated[2]), inverse_orientation)


def rpy_to_quaternion(roll, pitch, yaw):
    """固定轴 rpy（等价 ZYX 欧拉角）转四元数，与 URDF 外参约定一致。

    Args:
        roll: 绕 x 轴弧度
        pitch: 绕 y 轴弧度
        yaw: 绕 z 轴弧度

    Returns:
        tuple: ``(qx, qy, qz, qw)``
    """
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def yaw_from_quaternion(quaternion):
    """提取绕 z 轴的航向角（弧度，``(-π, π]``）。

    Args:
        quaternion: ``(qx, qy, qz, qw)``

    Returns:
        float: 航向角
    """
    qx, qy, qz, qw = normalize_quaternion(quaternion)
    return math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )


def normalize_angle(angle):
    """把角度归一化到 ``(-π, π]``。

    Args:
        angle: 弧度

    Returns:
        float: 归一化后的弧度
    """
    return math.atan2(math.sin(angle), math.cos(angle))


def pose_from_transform(transform):
    """从带 ``translation`` / ``rotation`` 字段的对象读取位姿。

    Args:
        transform: 任意具有 ``translation.x/y/z`` 与 ``rotation.x/y/z/w`` 的对象，
            例如 ``geometry_msgs.msg.TransformStamped.transform``

    Returns:
        tuple: ``(position, orientation)``
    """
    return _pose_from_fields(transform.translation, transform.rotation)


def pose_from_ros_pose(pose):
    """从带 ``position`` / ``orientation`` 字段的对象读取位姿。

    Args:
        pose: 任意具有 ``position.x/y/z`` 与 ``orientation.x/y/z/w`` 的对象，
            例如 ``geometry_msgs.msg.Pose`` 或 ``nav_msgs.msg.Odometry`` 的
            ``pose.pose``

    Returns:
        tuple: ``(position, orientation)``
    """
    return _pose_from_fields(pose.position, pose.orientation)


def _pose_from_fields(position, orientation):
    """把 ROS 风格的位置/姿态字段读成 ``(position, orientation)`` 位姿。"""
    return (
        (position.x, position.y, position.z),
        normalize_quaternion(
            (orientation.x, orientation.y, orientation.z, orientation.w)
        ),
    )
