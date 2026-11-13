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

"""URDF 的雷达安装角 ↔ 实车 nav2 参数里的 Point-LIO gravity 必须一致。

为什么值得单测（2026-11-13 实车踩到，两边差了 60°）：
模型里的 `lidar_roll` 是 +30°，而车静止时 `/livox/imu` 的比力是
(0.028, -0.500, 0.866) g —— 反推真实 roll 是 **-30°**。两边符号相反，
后果是**两种前端各坏一半，而且都不报错**：

* Point-LIO：`rot_init` 用 gravity 把世界系扶正，60° 误差被它吸收
  （`R_0 = Rx(-60°)`）。`cloud_registered` 经过 `loam_interface` 的
  `T_base←lidar` 后看着完全正常，但 `/odometry` 的航向一直带 ~17.8° 的常数偏置
  （实测 `map->gimbal_yaw` 在出生点就有 17.8°，而云台关节角只有 -2.2°）。
* ZLIO：`zlio_bridge` 把首帧位姿显式抵消（`T_off = M·R_0⁻¹`），模型误差**原样保留**，
  整片 `registered_scan`/`terrain_map_ext` 歪 60° —— 就是两张 RViz 对比图里
  "第一张（zlio）明显不对、第二张（point_lio）对"的直接原因。

这不是能靠"跑起来看着还行"发现的问题，所以用测试把两个数字钉在一起：
本测试**不启动任何节点、不需要 ROS**，只读 xacro 的默认值和三份实车参数文件。

判定：`gravity == 9.81·[sin(pitch), -cos(pitch)sin(roll), -cos(pitch)cos(roll)]`，
与 `real_mapping_launch.py` / `srm_robot_description.gravity_from_mounting()` 同一个公式。
两边任何一个改了而另一个没改，这里直接失败。
"""

import math
import re
from pathlib import Path

import yaml

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = PACKAGE_ROOT.parents[2]
URDF = WORKSPACE_ROOT / "src" / "srm27_robot_description" / "urdf" / "srm27_sentry.urdf.xacro"
REAL_CONFIG_DIR = PACKAGE_ROOT / "config" / "real"

#: 实车在跑的三种入口（minco 迁移链 / minco 旧链 / omni 旧链）。
#: nav2_params_upstream.yaml 是上游基线（gravity 语义不同），不参与。
REAL_PARAM_FILES = (
    "nav2_params_srm_minco_fakeyaw.yaml",
    "nav2_params_srm_minco.yaml",
    "nav2_params_srm.yaml",
)

#: 与 real_mapping_launch.py 的 gravity_from_mounting() 完全一致。
def gravity_from_mounting(roll, pitch):
    return [
        9.81 * math.sin(pitch),
        -9.81 * math.cos(pitch) * math.sin(roll),
        -9.81 * math.cos(pitch) * math.cos(roll),
    ]


def xacro_default(name):
    """从 xacro 里取 `<xacro:arg name="..." default="..."/>` 的默认值。"""
    text = URDF.read_text(encoding="utf-8")
    match = re.search(
        r'<xacro:arg\s+name="%s"\s+default="([^"]+)"' % re.escape(name), text
    )
    assert match is not None, f"{URDF} 里找不到 {name} 的默认值"
    return float(match.group(1))


def param_gravity(params_file):
    with open(params_file, encoding="utf-8") as stream:
        params = yaml.safe_load(stream)
    mapping = params["point_lio"]["ros__parameters"]["mapping"]
    return [float(v) for v in mapping["gravity"]], [float(v) for v in mapping["gravity_init"]]


def test_point_lio_gravity_matches_urdf_mount():
    roll = xacro_default("lidar_roll")
    pitch = xacro_default("lidar_pitch")
    expected = gravity_from_mounting(roll, pitch)

    # 模型本身得是"斜装"：roll=0 时这层保护就退化了（重力必然是 -z）。
    assert abs(roll) > 0.1, (
        f"URDF lidar_roll={roll:.6f} 几乎是平装。实车实测是 -30°（静止 IMU 比力 "
        f"(0.028,-0.500,0.866) g）；平装的话 gravity 也必须是 [0,0,-9.81]。"
    )

    for name in REAL_PARAM_FILES:
        path = REAL_CONFIG_DIR / name
        gravity, gravity_init = param_gravity(path)
        for key, value in (("gravity", gravity), ("gravity_init", gravity_init)):
            assert len(value) == 3, f"{name}: {key} 必须是 3 维"
            for axis, (got, want) in enumerate(zip(value, expected)):
                assert abs(got - want) < 1e-6, (
                    f"{name}: point_lio.mapping.{key}[{axis}] = {got!r}，但 URDF "
                    f"lidar_roll={roll!r} / lidar_pitch={pitch!r} 算出来应该是 {want!r}。\n"
                    f"  两边必须同号同值：改 URDF 的 lidar_roll 就要同步改这三份参数里的 "
                    f"gravity/gravity_init（反之亦然）。\n"
                    f"  公式: 9.81*[sin(pitch), -cos(pitch)sin(roll), -cos(pitch)cos(roll)]"
                )

    # 反向检查：符号写反（+30° vs -30°）必须能被上面拦下。
    flipped = gravity_from_mounting(-roll, pitch)
    assert any(abs(a - b) > 0.1 for a, b in zip(expected, flipped)), (
        "roll 的符号对 gravity 的 y 分量没有影响 —— 公式或取值有问题"
    )

    print(
        "OK: URDF lidar_roll=%r -> gravity=%s；%d 份实车参数一致"
        % (roll, ["%.6f" % v for v in expected], len(REAL_PARAM_FILES))
    )


if __name__ == "__main__":
    test_point_lio_gravity_matches_urdf_mount()
