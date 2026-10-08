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

"""真值里程计适配器的三维位姿集成测试（坡道修复方案第一批验收项 1/2/5）。

不启动 Gazebo：直接把合成好的 Gazebo 真值消息喂给适配器，替换 TF/话题发布器为
记录器，然后核对：

* ``odom -> base_link`` 的 TF 保留完整三维姿态（高度、roll/pitch 不被压平），
  并与 ``odometry`` 消息里的位姿完全一致；
* ``lidar_odometry`` 等于"已发布的底盘 TF 位姿 ∘ 几何 YAML 外参"，即 TF 与直接
  组合得到同一个雷达位姿；
* 静态场景里固定世界坐标的点，经"点云在雷达系 -> 雷达位姿 -> odom -> 世界"
  回环后与姿态、高度无关；
* 平地时行为与修复前一致（z=0、姿态只有 yaw），避免影响二维导航调参。
"""

import math
import os
import sys
import tempfile
from pathlib import Path


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

# rclpy 默认把日志写进 ~/.ros；受限环境里该目录可能不可写，这里显式指到临时目录
# （只在本文件未被外部指定时生效），否则 rclpy.init 会直接失败。
_LOG_DIR = os.path.join(tempfile.gettempdir(), "srm27_ground_truth_odometry_test")
os.makedirs(_LOG_DIR, exist_ok=True)
os.environ.setdefault("ROS_HOME", _LOG_DIR)
os.environ.setdefault("ROS_LOG_DIR", _LOG_DIR)

import rclpy  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402

import simulation_ground_truth_odometry as gt_odometry  # noqa: E402
from srm_nav_pose_math import (  # noqa: E402
    compose_pose,
    inverse_pose,
    pose_from_transform,
    rpy_to_quaternion,
    transform_point,
    yaw_from_quaternion,
)

TOLERANCE = 1.0e-9
RAMP_ROLL = math.radians(4.36)
RAMP_PITCH = math.radians(-9.01)

_NODE = None


class _Recorder:
    """替换发布器/广播器的最小记录器。"""

    def __init__(self):
        self.messages = []

    def sendTransform(self, message):  # noqa: N802 - 与 tf2_ros 接口同名
        self.messages.append(message)

    def publish(self, message):
        self.messages.append(message)

    def reset(self):
        self.messages.clear()

    def latest(self):
        assert self.messages, "适配器没有发布任何消息"
        return self.messages[-1]


def _node():
    """懒初始化共享节点：rclpy 只能初始化一次。"""
    global _NODE
    if _NODE is None:
        rclpy.init(args=None)
        _NODE = gt_odometry.SimulationGroundTruthOdometry()
        _NODE.tf_broadcaster = _Recorder()
        _NODE.odom_publisher = _Recorder()
        _NODE.lidar_odom_publisher = _Recorder()
    return _NODE


def _reset(initial_world, initial_yaw):
    """把节点重置到指定的出生位姿。"""
    node = _node()
    node.initial_pose = (
        initial_world,
        rpy_to_quaternion(0.0, 0.0, initial_yaw),
    )
    node.filtered_pose = None
    node.last_publish_time = None
    node.zero_pose_since = None
    node.tf_broadcaster.reset()
    node.odom_publisher.reset()
    node.lidar_odom_publisher.reset()
    return node


def _feed(world_position, world_orientation, stamp_nanoseconds=123_000_000_000):
    """喂一帧 Gazebo 真值并返回本次发布的 TF 与 lidar_odometry。"""
    node = _node()
    message = Odometry()
    message.header.stamp.sec = int(stamp_nanoseconds // 1_000_000_000)
    message.header.stamp.nanosec = int(stamp_nanoseconds % 1_000_000_000)
    message.pose.pose.position.x = world_position[0]
    message.pose.pose.position.y = world_position[1]
    message.pose.pose.position.z = world_position[2]
    message.pose.pose.orientation.x = world_orientation[0]
    message.pose.pose.orientation.y = world_orientation[1]
    message.pose.pose.orientation.z = world_orientation[2]
    message.pose.pose.orientation.w = world_orientation[3]
    message.twist.twist.linear.x = 0.4
    node.latest_message = message
    # 绕过 50 Hz 节流：每条用例都要拿到当前帧的输出。
    node.last_publish_time = None
    node._process_latest()
    return (
        node.tf_broadcaster.latest(),
        node.odom_publisher.latest(),
        node.lidar_odom_publisher.latest(),
    )


def _assert_close(actual, expected, tolerance, message):
    for index, (got, want) in enumerate(zip(actual, expected)):
        assert abs(got - want) <= tolerance, (
            f"{message}: 分量 {index} 期望 {want!r}，实际 {got!r}（容差 {tolerance}）"
        )


def _quaternion_close(actual, expected, tolerance):
    same = all(abs(a - b) <= tolerance for a, b in zip(actual, expected))
    flipped = all(abs(a + b) <= tolerance for a, b in zip(actual, expected))
    return same or flipped


def test_extrinsic_comes_from_the_geometry_yaml():
    """外参只有一个来源：srm27_robot_description 的几何 YAML。"""
    from srm27_robot_description import load_geometry

    geometry = load_geometry()
    node = _node()
    expected_xyz = tuple(float(value) for value in geometry["lidar"]["xyz"])
    expected_quaternion = rpy_to_quaternion(*geometry["lidar"]["rpy"])
    _assert_close(node.base_to_lidar[0], expected_xyz, TOLERANCE, "外参平移")
    assert _quaternion_close(node.base_to_lidar[1], expected_quaternion, TOLERANCE), (
        f"外参姿态期望 {expected_quaternion!r}，实际 {node.base_to_lidar[1]!r}"
    )


def test_flat_ground_output_matches_the_previous_planar_behaviour():
    """平地上 z 仍为 0、姿态仍只有 yaw：二维导航可见行为与修复前一致。"""
    _reset((5.0, -3.0, 0.0), 0.4)
    transform, odometry, _ = _feed(
        (6.2, -2.4, 0.0), rpy_to_quaternion(0.0, 0.0, 0.9)
    )
    assert abs(transform.transform.translation.z) < 1.0e-12
    _assert_close(
        (
            transform.transform.rotation.x,
            transform.transform.rotation.y,
        ),
        (0.0, 0.0),
        1.0e-12,
        "平地姿态应为纯 yaw",
    )
    assert transform.transform.rotation.z == odometry.pose.pose.orientation.z
    assert (
        abs(transform.transform.rotation.w - odometry.pose.pose.orientation.w) < 1.0e-15
    )


def test_tf_and_odometry_carry_the_same_full_3d_pose():
    """高度与 roll/pitch 必须同时出现在 TF 和 odometry 消息里。"""
    initial_world = (12.0, -4.5, 0.2)
    initial_yaw = 1.1
    _reset(initial_world, initial_yaw)
    roll, pitch = RAMP_ROLL, RAMP_PITCH
    world_pose = (
        (
            initial_world[0] + 1.6 * math.cos(initial_yaw),
            initial_world[1] + 1.6 * math.sin(initial_yaw),
            initial_world[2] + 0.28,
        ),
        rpy_to_quaternion(roll, pitch, initial_yaw),
    )
    transform, odometry, _ = _feed(world_pose[0], world_pose[1])

    _assert_close(
        (
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        ),
        (1.6, 0.0, 0.28),
        1.0e-9,
        "TF 相对位姿",
    )
    assert transform.header.frame_id == "odom"
    assert transform.child_frame_id == "base_link"
    # 消息位姿与 TF 必须表示同一件事。
    _assert_close(
        (
            odometry.pose.pose.position.x,
            odometry.pose.pose.position.y,
            odometry.pose.pose.position.z,
        ),
        (1.6, 0.0, 0.28),
        1.0e-9,
        "odometry 相对位姿",
    )
    assert _quaternion_close(
        (
            odometry.pose.pose.orientation.x,
            odometry.pose.pose.orientation.y,
            odometry.pose.pose.orientation.z,
            odometry.pose.pose.orientation.w,
        ),
        (
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        ),
        1.0e-12,
    ), "odometry 与 TF 的姿态不一致"
    # 俯仰/横滚真的进了姿态，没有被压成纯 yaw。
    tilt = math.sqrt(
        transform.transform.rotation.x ** 2 + transform.transform.rotation.y ** 2
    )
    assert tilt > 1.0e-3, f"TF 姿态里看不到 roll/pitch（四元数 xyz 模长 {tilt}）"


def test_lidar_odometry_equals_tf_pose_composed_with_extrinsic():
    """验收项：同一个雷达位姿经 TF 与直接组合得到相同结果。"""
    initial_world = (5.45, -3.39, 0.0)
    attitudes = [
        (0.0, 0.0, 0.0, 0.0),
        (0.0, 0.0, math.pi / 2.0, 0.15),
        (0.0, math.radians(10.0), 0.3, 0.17),
        (0.0, math.radians(-10.0), -0.7, -0.05),
        (RAMP_ROLL, RAMP_PITCH, math.radians(90.0), 0.40),
        (-0.12, 0.21, -2.6, 0.33),
    ]
    node = _reset(initial_world, -0.6)
    for roll, pitch, yaw, height in attitudes:
        _reset(initial_world, -0.6)
        world_pose = (
            (initial_world[0], initial_world[1], initial_world[2] + height),
            rpy_to_quaternion(roll, pitch, yaw),
        )
        transform, _, lidar_odometry = _feed(world_pose[0], world_pose[1])

        # 直接组合：已发布的底盘 TF 位姿 ∘ 外参。
        expected = compose_pose(
            pose_from_transform(transform.transform), node.base_to_lidar
        )
        actual = (
            (
                lidar_odometry.pose.pose.position.x,
                lidar_odometry.pose.pose.position.y,
                lidar_odometry.pose.pose.position.z,
            ),
            (
                lidar_odometry.pose.pose.orientation.x,
                lidar_odometry.pose.pose.orientation.y,
                lidar_odometry.pose.pose.orientation.z,
                lidar_odometry.pose.pose.orientation.w,
            ),
        )
        label = f"(rpy={roll:.4f},{pitch:.4f},{yaw:.4f}, z={height:.2f})"
        _assert_close(actual[0], expected[0], 1.0e-9, f"雷达位姿平移 {label}")
        assert _quaternion_close(actual[1], expected[1], 1.0e-12), (
            f"雷达位姿姿态 {label}: 期望 {expected[1]!r}，实际 {actual[1]!r}"
        )
        assert lidar_odometry.child_frame_id == node.lidar_frame
        # 外参平移必须随底盘姿态旋转：与"平移直接相加"的结果明显不同。
        if abs(yaw) > 1.0e-3 or abs(pitch) > 1.0e-3:
            unrotated = (
                transform.transform.translation.x + node.base_to_lidar[0][0],
                transform.transform.translation.y + node.base_to_lidar[0][1],
                transform.transform.translation.z + node.base_to_lidar[0][2],
            )
            assert math.dist(actual[0], unrotated) > 1.0e-3, (
                f"雷达外参没有随底盘姿态旋转 {label}"
            )


def test_static_world_point_round_trip_through_published_messages():
    """验收项：静态点的世界坐标在上/下坡过程中保持一致（用真实发布的消息回环）。"""
    initial_world = (5.45, -3.39, 0.0)
    initial_yaw = -0.6
    world_point = (7.2, -1.4, 0.35)
    odom_frame_pose = (initial_world, rpy_to_quaternion(0.0, 0.0, initial_yaw))

    attitudes = [
        (0.0, 0.0, 0.0, 0.0),
        (0.0, 0.0, math.radians(10.0), 0.15),
        (0.0, 0.0, math.radians(-10.0), 0.15),
        (RAMP_ROLL, RAMP_PITCH, 0.0, 0.40),
        (RAMP_ROLL, RAMP_PITCH, math.radians(90.0), 0.55),
    ]
    for roll, pitch, delta_yaw, height in attitudes:
        node = _reset(initial_world, initial_yaw)
        yaw = initial_yaw + delta_yaw
        world_pose = (
            (initial_world[0], initial_world[1], initial_world[2] + height),
            rpy_to_quaternion(roll, pitch, yaw),
        )
        transform, _, lidar_odometry = _feed(world_pose[0], world_pose[1])

        # 点云的生成过程：世界 -> 雷达，再取点。
        world_to_lidar = compose_pose(world_pose, node.base_to_lidar)
        point_in_lidar = transform_point(inverse_pose(world_to_lidar), world_point)

        # 链路输出：TF 里的底盘位姿与 lidar_odometry 里的雷达位姿。
        odom_to_base = pose_from_transform(transform.transform)
        odom_to_lidar = (
            (
                lidar_odometry.pose.pose.position.x,
                lidar_odometry.pose.pose.position.y,
                lidar_odometry.pose.pose.position.z,
            ),
            (
                lidar_odometry.pose.pose.orientation.x,
                lidar_odometry.pose.pose.orientation.y,
                lidar_odometry.pose.pose.orientation.z,
                lidar_odometry.pose.pose.orientation.w,
            ),
        )
        # 两条链路必须给出同一个雷达位姿（TF ∘ 外参 == lidar_odometry）。
        _assert_close(
            odom_to_lidar[0],
            compose_pose(odom_to_base, node.base_to_lidar)[0],
            1.0e-9,
            "TF 链路与 lidar_odometry 平移不一致",
        )

        point_back_in_world = transform_point(
            odom_frame_pose, transform_point(odom_to_lidar, point_in_lidar)
        )
        _assert_close(
            point_back_in_world,
            world_point,
            1.0e-9,
            f"静态点回环 (roll={roll:.3f}, pitch={pitch:.3f}, dyaw={delta_yaw:.3f})",
        )


def test_jitter_hold_keeps_the_whole_3d_pose_and_releases_on_motion():
    """静止抖动被整体保持（含高度/俯仰），一旦真实运动就整体更新。"""
    initial_world = (1.0, 2.0, 0.0)
    node = _reset(initial_world, 0.2)
    base_world = (1.4, 2.0, 0.10)
    base_orientation = rpy_to_quaternion(RAMP_ROLL, RAMP_PITCH, 0.2)
    first, _, _ = _feed(base_world, base_orientation)
    # 亚阈值抖动：平面位移 1 mm、航向 0.0005 rad，但姿态与高度都有微小变化。
    jittered = _feed(
        (base_world[0] + 0.001, base_world[1], base_world[2] + 0.01),
        rpy_to_quaternion(RAMP_ROLL + 0.02, RAMP_PITCH, 0.2005),
    )
    assert jittered[0].transform.translation.x == first.transform.translation.x
    assert jittered[0].transform.translation.z == first.transform.translation.z
    assert jittered[0].transform.rotation.x == first.transform.rotation.x

    # 超过阈值：位姿整体更新，高度与俯仰跟着走。
    moved = _feed(
        (base_world[0] + 0.05, base_world[1], base_world[2] + 0.10),
        rpy_to_quaternion(RAMP_ROLL + 0.02, RAMP_PITCH + 0.02, 0.21),
    )
    assert moved[0].transform.translation.x != first.transform.translation.x
    assert moved[0].transform.translation.z != first.transform.translation.z
    assert moved[0].transform.rotation.x != first.transform.rotation.x
    expected_height = first.transform.translation.z + 0.10
    assert abs(moved[0].transform.translation.z - expected_height) < 1.0e-9
    del node


def test_yaw_of_published_transform_tracks_the_truth():
    """发布位姿的航向必须等于真值航向（相对出生航向）。"""
    initial_yaw = 0.35
    _reset((0.0, 0.0, 0.0), initial_yaw)
    for delta_yaw in (-2.0, -0.4, 0.0, 0.9, 2.4):
        _reset((0.0, 0.0, 0.0), initial_yaw)
        transform, _, _ = _feed(
            (0.6 * math.cos(initial_yaw), 0.6 * math.sin(initial_yaw), 0.05),
            rpy_to_quaternion(0.05, -0.03, initial_yaw + delta_yaw),
        )
        actual = yaw_from_quaternion(
            (
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            )
        )
        assert abs(actual - delta_yaw) < 1.0e-9, (
            f"航向期望 {delta_yaw}，实际 {actual}"
        )


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
    try:
        sys.exit(1 if _run_all() else 0)
    finally:
        if _NODE is not None:
            _NODE.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
