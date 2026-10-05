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

"""srm27_robot_description 生成结果的自检。"""

import xml.etree.ElementTree as ET

from srm27_robot_description import load_geometry
from srm27_robot_description.model_builder import build_sdf, build_urdf

LEGACY_TOKENS = ("gimbal", "armor", "light_bar", "shooter", "MecanumDrive", "pb_rm_simulation")


def test_generated_models_are_well_formed():
    urdf = ET.fromstring(build_urdf())
    sdf = ET.fromstring(build_sdf())

    assert urdf.tag == "robot"
    assert sdf.tag == "sdf"
    assert sdf.find("model") is not None


def test_geometry_comes_from_single_source():
    geometry = load_geometry()
    urdf = build_urdf(geometry)
    sdf = build_sdf(geometry)

    assert f'name="{geometry["robot"]["base_link"]}"' in urdf
    assert f'name="{geometry["robot"]["base_link"]}"' in sdf
    # 底盘尺寸与轮组位置必须来自 YAML，避免模型两份各自漂移。
    assert f'<radius>{geometry["chassis"]["radius"]}</radius>' in sdf
    assert f'<length>{geometry["chassis"]["length"]}</length>' in sdf
    for position in geometry["wheel"]["positions"].values():
        assert " ".join(repr(float(value)) for value in position) in sdf


def test_legacy_sentry_components_are_gone():
    for text in (build_urdf(), build_sdf()):
        lowered = text.lower()
        for token in LEGACY_TOKENS:
            assert token.lower() not in lowered, f"生成结果仍包含旧组件: {token}"


def test_navigation_base_frame_and_lidar_names_are_stable():
    sdf = build_sdf()
    # 导航速度参考系与真值 TF 使用 base_link，不再有 base_footprint。
    assert "<link name=\"base_link\">" in sdf
    assert "base_footprint" not in sdf
    # 现有桥接与点云链路依赖这些帧名/传感器名。
    assert "front_mid360_lidar" in sdf
    assert "front_mid360_imu" in sdf
    assert "<link name=\"front_mid360\">" in sdf


def test_srm_velocity_system_plugin_is_configured():
    sdf = build_sdf(velocity_topic="cmd_vel", odometry_topic="odometry", command_timeout=0.1)
    assert "srm_velocity_system" in sdf
    assert "<command_topic>cmd_vel</command_topic>" in sdf
    assert "<odom_topic>odometry</odom_topic>" in sdf
    assert "<command_timeout>0.1</command_timeout>" in sdf


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
    import sys

    sys.exit(1 if _run_all() else 0)
