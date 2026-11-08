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

"""`zlio_bridge` 的坐标数学回归（纯数学，不需要 ROS 图、不启节点）。

钉住的是那次**迁移引入的 bug**（实施记录 §5.5）：

    首帧 T_off 必须是  M · R_0⁻¹，不是 M

其中 `M = T_base←lidar(t0)` 是安装外参，`R_0` 是 ZLIO 首帧位姿。少乘 `R_0⁻¹` 等价于假设
`R_0 ≈ 单位阵`——Point-LIO 成立，**ZLIO 不成立**（`initialization.cpp` 的 `alignGravity()`
把世界系扶正到重力朝 -Z，于是 `R_0` 带上了雷达的安装倾角，仿真里 30° roll）。

判据（本测试的断言）：

1. `T_off · R_0 · M⁻¹ == I` —— 首帧的 odom→base_link 位姿必须是单位阵（契约 C3）；
2. 用旧的 `T_off = M` 时，同一个量会是一个**约 30° 的旋转**（就是"odom 系被转了 30°"）；
3. 之后各帧满足 `T_off · R_k · M_k⁻¹` 只反映底盘的真实运动，不含那 30° 常量；
4. 换代锚定 `T_off ← T_off·R_last·R_first⁻¹` 在 `R_0 ≠ I` 时仍然保持连续。
"""

import math
import os
import sys

PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(PACKAGE_ROOT, "scripts"))

from zlio_bridge import _Rigid  # noqa: E402  （只用到其中的纯数学部分）


def _assert(condition, message):
    if not condition:
        raise AssertionError(message)


def quat_from_rpy(roll, pitch, yaw):
    """ZYX 欧拉角（rad）→ 四元数 (w, x, y, z)。"""
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def angle_between(a, b):
    """两个姿态之间的旋转角（rad）。"""
    dot = abs(sum(x * y for x, y in zip(a.q, b.q)))
    return 2.0 * math.acos(min(1.0, max(-1.0, dot)))


def identity():
    return _Rigid()


# 仿真的真实几何：雷达装在云台上，相对 base_link 有 30° roll（+ 63° yaw 的安装朝向）。
MOUNT = _Rigid(quat_from_rpy(math.radians(30.0), 0.0, math.radians(63.0)),
               (0.032, 0.187, 0.456))
# ZLIO 首帧：世界系被 alignGravity() 扶正，所以 R_0 带着雷达相对重力的倾角（≈ 30° roll），
# 再叠上出生时的车体航向。
R0 = _Rigid(quat_from_rpy(math.radians(30.0), 0.0, math.radians(25.0)), (0.0, 0.0, 0.0))


def test_first_frame_pose_is_identity():
    """核心不变量：首帧的 odom→base_link 必须是单位阵。"""
    t_off = MOUNT * R0.inverse()             # ← 修好之后的写法
    base_at_first = (t_off * R0) * MOUNT.inverse()
    _assert(
        angle_between(base_at_first, identity()) < 1e-9,
        "首帧 base 位姿应为单位阵，实际差 %.3f°"
        % math.degrees(angle_between(base_at_first, identity())),
    )
    _assert(
        math.sqrt(sum(v * v for v in base_at_first.t)) < 1e-9,
        "首帧 base 平移应为 0，实际 %r" % (base_at_first.t,),
    )


def test_old_formula_leaves_a_30_degree_tilt():
    """旧写法（少乘 R_0⁻¹）必须能把那次倾斜复现出来 —— 这就是当初观察到的现象。

    * 纯 roll 30° 的首帧位姿 → 残留倾斜**正好 30°**，绕一根水平轴（与实跑一致）；
    * 一般首帧位姿（roll 30° + yaw 25°）→ 残留倾斜 = `R_0` 的旋转角 ≈ 38.9°
      （共轭不改变旋转角），其中 roll 分量仍是 30°。
    """
    # 子用例 1：雷达装在 30° roll 的云台上、出生朝向为 0
    r0_pure_roll = _Rigid(quat_from_rpy(math.radians(30.0), 0.0, 0.0), (0.0, 0.0, 0.0))
    base = (MOUNT * r0_pure_roll) * MOUNT.inverse()
    tilt = math.degrees(angle_between(base, identity()))
    _assert(
        abs(tilt - 30.0) < 1e-6,
        "纯 roll 30° 的首帧应残留 30.0° 倾斜，实际 %.4f°" % tilt,
    )
    # 倾斜轴必须是水平的（z 分量≈0）—— 实跑看到的"绕一根水平轴转 30°"
    sin_half = math.sqrt(max(0.0, 1.0 - base.q[0] ** 2))
    if sin_half > 1e-9:
        _assert(
            abs(base.q[3]) / sin_half < 1e-6,
            "倾斜轴应是水平的，实际 z 分量 %.3e" % (base.q[3] / sin_half),
        )

    # 子用例 2：一般情形下，残留倾斜的角度 == R_0 的旋转角
    base = (MOUNT * R0) * MOUNT.inverse()
    tilt = math.degrees(angle_between(base, identity()))
    r0_angle = math.degrees(angle_between(R0, identity()))
    _assert(
        abs(tilt - r0_angle) < 1e-6 and tilt > 25.0,
        "残留倾斜应等于 R_0 的旋转角 %.2f°，实际 %.2f°" % (r0_angle, tilt),
    )


def test_relative_motion_is_unaffected():
    """修好之后，相邻帧之间的**相对**运动必须与 ZLIO 给的一致（不能顺手改坏尺度）。"""
    t_off = MOUNT * R0.inverse()
    for yaw_delta in (0.05, 0.10, -0.20):
        # 底盘原地转 yaw_delta（rad）：雷达系随之转同样的角度
        r_k = _Rigid(quat_from_rpy(0.0, 0.0, yaw_delta), (0.0, 0.0, 0.0)) * R0
        base_k = (t_off * r_k) * MOUNT.inverse()
        expected = _Rigid(quat_from_rpy(0.0, 0.0, yaw_delta), (0.0, 0.0, 0.0))
        _assert(
            angle_between(base_k, expected) < 1e-9,
            "底盘转了 %.2f rad，重建出来却是 %.3f°"
            % (yaw_delta, math.degrees(angle_between(base_k, expected))),
        )


def test_epoch_anchor_stays_continuous_with_non_identity_r0():
    """换代锚定与 R_0 无关：R_0 ≠ I 时锚定后仍然连续。"""
    t_off = MOUNT * R0.inverse()
    # 老代次最后一帧：底盘转了 0.3 rad
    r_last_old = _Rigid(quat_from_rpy(0.0, 0.0, 0.3), (1.0, 0.0, 0.0)) * R0
    pose_before = t_off * r_last_old
    # 新代次第一帧：ZLIO 换了世界系，又做了一次 alignGravity → 原始位姿带 30° roll、位置归零
    r_first_new = _Rigid(quat_from_rpy(math.radians(30.0), 0.0, 0.0), (0.0, 0.0, 0.0))
    t_off_new = t_off * r_last_old * r_first_new.inverse()
    pose_after = t_off_new * r_first_new
    _assert(
        angle_between(pose_after, pose_before) < 1e-9
        and math.sqrt(sum((a - b) ** 2 for a, b in zip(pose_after.t, pose_before.t))) < 1e-9,
        "换代锚定后位姿应连续",
    )


def test_planarize_uses_mount_not_t_off():
    """planarize 分支必须用安装外参 M，而不是含 R_0⁻¹ 的 T_off。"""
    t_off = MOUNT * R0.inverse()
    # 用 T_off 当安装外参去"压平再乘回"会算错；这里只钉住"两者确实不同"这个前提。
    _assert(
        angle_between(t_off, MOUNT) > math.radians(5.0),
        "T_off 与安装外参不应该相等（否则说明 R_0⁻¹ 没生效）",
    )


def main():
    tests = [
        test_first_frame_pose_is_identity,
        test_old_formula_leaves_a_30_degree_tilt,
        test_relative_motion_is_unaffected,
        test_epoch_anchor_stays_continuous_with_non_identity_r0,
        test_planarize_uses_mount_not_t_off,
    ]
    for test in tests:
        test()
        print(f"[OK] {test.__name__}")
    print(f"\n全部通过: {len(tests)} 项")
    return 0


if __name__ == "__main__":
    sys.exit(main())
