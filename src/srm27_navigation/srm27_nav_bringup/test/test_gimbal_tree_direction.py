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

"""云台系 TF 树的**方向**回归（2026-11-08，方案 §4.1 的补漏）。

为什么必须管方向：代价地图 / 行为树 / 假车头查的是 TF（`odom -> gimbal_yaw_fake`）。
只要 `odom -> gimbal_yaw` 这一段还经过 `base_link`（即还经过关节角），云台快转时那
±56° 的误差就**照样进代价地图** —— 只把 `/odometry` 改成云台位姿是不够的。

所以链路要满足三件事，这里逐条钉住：
1. URDF 能把根反过来：`gimbal_is_root:=true` 时 `gimbal_yaw` 是根、`base_link` 挂在它下面
   （关节 origin 的 z 变成 -chassis_top），默认 false 时保持原方向；
2. 两种方向下"底盘原点相对雷达"的安装位姿必须**完全一致**（导航的切片高度、重力方向
   都从它来），且 `livox_scan` 跟着雷达那一支走；
3. `sensor_scan_generation` 直接发 `odom -> gimbal_yaw`（`publish_robot_base_tf`），
   关节角语义因此变成"底盘相对云台"，`srm27_nav_protocol.gimbal_joint_sign` 必须是 +1.0，
   实车脚本必须把 `gimbal_is_root:=true` 传给 robot_state_publisher。

不启动任何节点：只展开 xacro、读 YAML、读脚本。
"""

import sys
import xml.etree.ElementTree as ET
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
# <ws>/src/srm27_navigation/srm27_nav_bringup -> parents[2] = <ws>
WORKSPACE_ROOT = PACKAGE_ROOT.parents[2]
CONFIG_DIR = PACKAGE_ROOT / "config" / "real"
SCRIPTS_DIR = PACKAGE_ROOT / "scripts"
REAL_PARAMS = CONFIG_DIR / "nav2_params_srm_minco_fakeyaw.yaml"
PROTOCOL_CONFIG = (
    WORKSPACE_ROOT / "src" / "srm27_nav_protocol" / "config" / "srm27_nav_protocol.yaml"
)
REAL_SCRIPT = WORKSPACE_ROOT / "script" / "start_real_nav.sh"

#: 云台基座面相对地面（= URDF 的 chassis_top = 0.1504 + 0.2353/2）。
CHASSIS_TOP = 0.26805

sys.path.insert(0, str(SCRIPTS_DIR))


def _assert(condition, message):
    if not condition:
        raise AssertionError(message)


def _description(inverted):
    from srm_robot_description import build_description

    return build_description(
        str(REAL_PARAMS), "", "", gimbal_joint_type="revolute", gimbal_is_root=inverted
    )


def _tree(xml_text):
    robot = ET.fromstring(xml_text)
    links = {link.get("name") for link in robot.findall("link")}
    joints = {}
    for joint in robot.findall("joint"):
        joints[joint.get("name")] = (
            joint.get("type"),
            joint.find("parent").get("link"),
            joint.find("child").get("link"),
            joint.find("origin").get("xyz"),
        )
    return links, joints


def test_default_tree_keeps_base_link_as_root():
    """默认方向必须与改造前一致：base_link 是根，关节是 base_link -> gimbal_yaw。"""
    _, joints = _tree(_description(False)[0])
    joint = joints["gimbal_yaw_joint"]
    _assert(joint[1] == "base_link" and joint[2] == "gimbal_yaw", f"默认方向错了: {joint}")
    # 云台在底盘上方 chassis_top。
    z = float(joint[3].split()[2])
    _assert(abs(z - CHASSIS_TOP) < 1e-6, f"默认方向的关节 z 应为 {CHASSIS_TOP}，实际 {z}")


def test_inverted_tree_puts_gimbal_yaw_at_the_root():
    """`gimbal_is_root` 打开后：gimbal_yaw 是根，base_link 挂在它下面（z 取反）。"""
    xml_text, _ = _description(True)
    links, joints = _tree(xml_text)
    children = {joint[2] for joint in joints.values()}
    roots = links - children
    _assert(roots == {"gimbal_yaw"}, f"反转后根必须只有 gimbal_yaw，实际 {sorted(roots)}")

    joint = joints["gimbal_yaw_joint"]
    _assert(joint[0] == "revolute", f"云台关节必须是 revolute（要跟 /joint_states）: {joint}")
    _assert(joint[1] == "gimbal_yaw" and joint[2] == "base_link",
            f"反转后关节必须是 gimbal_yaw -> base_link: {joint}")
    z = float(joint[3].split()[2])
    _assert(abs(z + CHASSIS_TOP) < 1e-6, f"反转后底盘应在云台基座面下方 -{CHASSIS_TOP}，实际 {z}")

    # 反转树里关节角是"底盘相对云台"，与下位机的 chassis_to_gimbal_yaw 同向。
    # 这也正是 srm27_nav_protocol 里 gimbal_joint_sign 必须 === +1.0 的原因。
    scan = joints["livox_scan_joint"]
    _assert(scan[1] == "gimbal_yaw",
            f"反转树里 livox_scan 必须挂在 gimbal_yaw（雷达就在云台上）: {scan}")


def test_lidar_mounting_is_identical_in_both_directions():
    """安装位姿（base_link <- 雷达）与切片系高度不能因为换方向而变。"""
    _, mounting_default = _description(False)
    _, mounting_inverted = _description(True)
    for key in ("xyz", "rpy"):
        for left, right in zip(mounting_default[key], mounting_inverted[key]):
            _assert(abs(left - right) < 1e-9,
                    f"{key} 换方向后变了: {mounting_default[key]} vs {mounting_inverted[key]}")
    # 雷达在 base_link 上方 0.346 m（地面=0 口径）。
    _assert(abs(mounting_inverted["xyz"][2] - 0.346388) < 1e-5,
            f"雷达高度解析错了: {mounting_inverted['xyz']}")


def test_real_chain_publishes_the_robot_base_tf():
    """实车必须由 sensor_scan_generation 直接发 odom->gimbal_yaw，且不再发 odom->base_link。"""
    import yaml

    cfg = yaml.safe_load(REAL_PARAMS.read_text())
    scan = cfg["sensor_scan_generation"]["ros__parameters"]
    _assert(scan["robot_base_frame"] == "gimbal_yaw", scan["robot_base_frame"])
    _assert(scan["publish_robot_base_tf"] is True,
            "实车必须打开 publish_robot_base_tf，否则 odom->gimbal_yaw 还要绕 base_link")
    _assert(scan["base_frame"] == "base_link",
            "base_frame 仍要写 base_link（关节角链路、文档与仿真对照都用它）")
    _assert(scan.get("cache_robot_base_transform") is True,
            "固定外参必须缓存（方案 §4.1.1）")


def test_real_script_passes_the_inverted_tree_flag():
    """start_real_nav.sh 必须把 gimbal_is_root:=true 传给 robot_state_publisher。"""
    text = REAL_SCRIPT.read_text()
    _assert("gimbal_is_root:=true" in text,
            "实车脚本没有传 gimbal_is_root:=true：TF 树仍是 base_link 为根，"
            "odom->gimbal_yaw 会继续经过关节角")
    _assert("gimbal_joint_type:=revolute" in text, "云台必须仍按真关节处理（/joint_states）")


def test_joint_sign_matches_the_inverted_tree():
    """反转树里关节角 = 底盘相对云台，符号必须 +1.0（原来的 -1.0 会让底盘反向转）。"""
    import yaml

    cfg = yaml.safe_load(PROTOCOL_CONFIG.read_text())
    # 顶层键是节点名（/srm27_nav_protocol），参数在 ros__parameters 下。
    node = next(iter(cfg.values()))["ros__parameters"]
    _assert(abs(float(node["gimbal_joint_sign"]) - 1.0) < 1e-9,
            f"树反转后 gimbal_joint_sign 必须是 +1.0，实际 {node['gimbal_joint_sign']}")
    _assert(node.get("gimbal_yaw_rate_topic") == "gimbal_yaw_rate",
            "协议里应有云台陀螺仪转速话题 gimbal_yaw_rate（yaw 外推的 gimbal_rate 源）")
    _assert("gimbal_yaw_rate_enc_topic" not in node,
            "编码器转速不该单独回传：它就是 chassis_to_gimbal_yaw 的变化率")


def _run_all():
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
