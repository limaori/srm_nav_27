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

"""实车迁移链（nav_srm_real_launch.py）的接线回归。

仿真那份链路由 test_fakeyaw_launch_wiring.py 守住；实车入口多了一层**执行端**差异，
而这层错了的表现同样是"链路全绿、车不动"，所以这里钉住四件事：

1. **唯一链路**：实车入口恒走 fakeyaw 迁移链（use_fakeyaw_transform=True、
   command_frame=gimbal_yaw_fake、cmd_vel_nav_topic=cmd_vel_nav_fake）。
2. **mux 出口是 cmd_vel_chassis**：串口节点只会订阅这个名字；出口写成 cmd_vel_sim
   （仿真默认值）时 launch 必须直接报错，而不是让车静默不动。
3. **参数文件与 launch 一致**：fake 的输入/输出 == Nav2 出口/mux 输入；导航参考系在
   bt / 两份 costmap / behavior 四处都是 gimbal_yaw_fake；odom_timeout 容得下实车
   10 Hz 的 /odometry。
4. **云台系（2026-11-08）**：/odometry 的 child = gimbal_yaw 且固定外参已缓存（不再依赖
   关节角 TF）；loam_interface 的开机快照也取 gimbal_yaw，并把 odom 的 z 拉回地面。
5. **两路 wz**：mux 的第二路是决策层 rotation_cmd（只取 angular.z）；导航默认 xy_only
   （只出平移），判停器是只看平移的 OmniStoppedGoalChecker。
6. **TF 树方向**：`odom -> gimbal_yaw` 由本节点直接发（`publish_robot_base_tf`），
   关节角只留在末端 `gimbal_yaw -> base_link`（细节见 test_gimbal_tree_direction.py）。

测试不启动任何节点：只构造 LaunchDescription、求值替换、跑一个 OpaqueFunction。
"""

import os
import sys
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_FILE = PACKAGE_ROOT / "launch" / "nav_srm_real_launch.py"
CONFIG_DIR = PACKAGE_ROOT / "config" / "real"

FAKEYAW_PARAMS = CONFIG_DIR / "nav2_params_srm_minco_fakeyaw.yaml"
MINCO_PARAMS = CONFIG_DIR / "nav2_params_srm_minco.yaml"
OMNI_PARAMS = CONFIG_DIR / "nav2_params_srm.yaml"


def _assert(condition, message):
    if not condition:
        raise AssertionError(message)


def _sub(context, value):
    """求值替换：同时接受 str、Substitution 与它们的列表。"""
    from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions

    return perform_substitutions(context, normalize_to_list_of_substitutions(value))


def _load_launch_module():
    """按路径加载 launch 模块（不通过 ament index，便于在源码树里直接跑）。"""
    import importlib.util

    spec = importlib.util.spec_from_file_location("srm_real_launch", str(LAUNCH_FILE))
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _make_context(module, overrides):
    """构造 LaunchDescription 并把声明参数的默认值灌进 LaunchContext。"""
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
    from launch_ros.actions import Node

    description = module.generate_launch_description()
    context = LaunchContext()
    context.launch_configurations["params_file"] = overrides["params_file"]

    arguments = [e for e in description.entities if isinstance(e, DeclareLaunchArgument)]
    # 默认值之间互相引用（mux 出口、yaw_mode 等），多轮求值直到稳定。
    pending = list(arguments)
    for _ in range(len(arguments) + 1):
        progressed = False
        for argument in list(pending):
            name = _sub(context, argument.name)
            if name in overrides:
                pending.remove(argument)
                continue
            try:
                value = _sub(context, argument.default_value)
            except Exception:  # noqa: BLE001 - 依赖尚未就绪，下一轮再来
                continue
            context.launch_configurations[name] = value
            pending.remove(argument)
            progressed = True
        if not progressed:
            break
    _assert(not pending, f"有参数默认值无法求值: {[_sub(context, a.name) for a in pending]}")

    for key, value in overrides.items():
        if key != "params_file":
            context.launch_configurations[key] = value

    includes = [e for e in description.entities if isinstance(e, IncludeLaunchDescription)]
    nodes = [e for e in description.entities if isinstance(e, Node)]
    functions = [e for e in description.entities if isinstance(e, OpaqueFunction)]
    return description, context, includes, nodes, functions


def _run_speed_chain_check(functions, context):
    """执行启动时的速度链核对（launch 里注册的那个 OpaqueFunction）。"""
    for entity in functions:
        result = entity.execute(context)
        if result:
            return [_sub(context, action.msg) for action in result]
    raise AssertionError("没找到速度链核对函数")


def _source_text(source):
    """取 launch 源的路径文本（`LaunchDescriptionSource` 的 location 替换）。"""
    location = getattr(source, "_LaunchDescriptionSource__location", None)
    if location is None:
        return str(source)
    parts = []
    for substitution in location:
        parts.append(str(getattr(substitution, "text", substitution)))
    return "".join(parts)


def _include_arguments(includes, context, target):
    """按目标 launch 文件名取某个 include 的 launch_arguments（已求值）。"""
    for include in includes:
        source = getattr(include, "launch_description_source", None)
        if source is None:
            continue
        if target in _source_text(source):
            arguments = {}
            for key, value in include.launch_arguments:
                arguments[_sub(context, key)] = _sub(context, value)
            return arguments
    raise AssertionError(f"没有找到 {target} 的 include")


# ---------------------------------------------------------------------------
# 用例
# ---------------------------------------------------------------------------
def test_defaults_select_the_migration_chain():
    module = _load_launch_module()
    _, context, includes, _, functions = _make_context(
        module, {"params_file": str(FAKEYAW_PARAMS)}
    )
    conf = context.launch_configurations

    _assert(conf["use_fakeyaw_transform"] == "True", f"实车入口必须默认启用 fake 转换器: {conf}")
    _assert(conf["command_frame"] == "gimbal_yaw_fake", "命令系必须是 gimbal_yaw_fake")
    _assert(conf["cmd_vel_nav_topic"] == "cmd_vel_nav_fake",
            "Nav2 出口必须是 cmd_vel_nav_fake（fake 的输入）")
    _assert(conf["use_chassis_mux"] == "True", "实车入口必须默认串 mux")
    _assert(conf["mux_spin_topic"] == "rotation_cmd",
            "mux 的第二路必须默认是决策层的 rotation_cmd（方案 §4.3）")
    _assert(conf["yaw_mode"] == "xy_only",
            "实车默认必须是 xy_only：导航只出平移，底盘 wz 由决策层给（方案 §4.2）")
    _assert(conf["mux_nav_topic"] == "cmd_vel_nav_body", "mux 输入必须是 fake 的输出")
    _assert(conf["mux_output_topic"] == "cmd_vel_chassis",
            "mux 输出必须是 cmd_vel_chassis（串口节点订阅它）")
    _assert(conf["use_sim_time"] == "False", "实车入口不能开 use_sim_time")
    _assert(conf["params_file"] == str(FAKEYAW_PARAMS) or
            conf["params_file"].startswith(str(CONFIG_DIR.parent)), conf["params_file"])

    nav2 = _include_arguments(includes, context, "nav2_stack_launch.py")
    _assert(nav2["use_fake_vel_transform"] == "True", f"Nav2 必须打开 fake 转换器: {nav2}")
    _assert(nav2["cmd_vel_nav_topic"] == "cmd_vel_nav_fake", nav2)
    _assert(nav2["use_sim_time"] == "False", nav2)
    _assert(nav2["use_lio_odometry"] == "True", f"实车必须有里程计来源: {nav2}")

    mux = _include_arguments(includes, context, "srm_chassis_control.launch.py")
    _assert(mux["nav_topic"] == "cmd_vel_nav_body", mux)
    _assert(mux["spin_topic"] == "rotation_cmd", f"mux 必须订阅决策层自转: {mux}")
    _assert(mux["output_topic"] == "cmd_vel_chassis", mux)

    messages = "\n".join(_run_speed_chain_check(functions, context))
    _assert("cmd_vel_chassis" in messages and "srm27_nav_protocol" in messages,
            f"启动日志必须打印到串口的完整链路: {messages}")


def test_params_file_matches_the_launch_wiring():
    """参数文件里的接线必须与 launch 的替换逐一对应（否则链路断开）。"""
    import yaml

    module = _load_launch_module()
    _, context, includes, _, _ = _make_context(module, {"params_file": str(FAKEYAW_PARAMS)})
    cfg = yaml.safe_load(FAKEYAW_PARAMS.read_text())
    fake = cfg["fake_vel_transform"]["ros__parameters"]
    nav2 = _include_arguments(includes, context, "nav2_stack_launch.py")
    mux = _include_arguments(includes, context, "srm_chassis_control.launch.py")

    _assert(fake["input_cmd_vel_topic"] == nav2["cmd_vel_nav_topic"],
            f"fake 输入 {fake['input_cmd_vel_topic']} != Nav2 出口 {nav2['cmd_vel_nav_topic']}")
    _assert(fake["output_cmd_vel_topic"] == mux["nav_topic"],
            f"fake 输出 {fake['output_cmd_vel_topic']} != mux 输入 {mux['nav_topic']}")
    _assert(fake["transform_mode"] == "nav_se2", "实车参数必须是 nav_se2（legacy 会叠加自转）")
    _assert(fake["robot_base_frame"] == "gimbal_yaw" and
            fake["fake_robot_base_frame"] == "gimbal_yaw_fake",
            "fake 节点 TF 父子必须与 TF 拓扑一致 (gimbal_yaw -> gimbal_yaw_fake)")
    _assert(float(fake["odom_timeout"]) >= 0.3,
            f"实车 /odometry 是 10 Hz，odom_timeout 至少 0.3 s（当前 {fake['odom_timeout']}）")
    _assert(fake.get("yaw_rate_source") == "imu",
            "云台系下 yaw 外推必须用云台转速（默认 imu；gimbal_rate 待 §8.1 验符号后切换）")
    _assert(fake.get("chassis_cmd_vel_topic", "") == "",
            "cmd_wz 源必须保持关闭（云台系下底盘 wz ≠ 云台转速）")

    scan = cfg["sensor_scan_generation"]["ros__parameters"]
    _assert(scan["robot_base_frame"] == "gimbal_yaw",
            "/odometry 的机器人系必须是 gimbal_yaw（雷达定位直接给出，不经过关节角）")
    _assert(scan.get("cache_robot_base_transform") is True,
            "实车必须打开固定外参缓存，否则关节角断流会拖垮 /odometry（方案 §4.1.1）")
    _assert(scan.get("publish_robot_base_tf") is True,
            "TF 树反转后必须直接发 odom->gimbal_yaw（不能再发 odom->base_link，"
            "否则 base_link 有两个父节点）")
    for node in ("loam_interface", "zlio_bridge"):
        section = cfg[node]["ros__parameters"]
        _assert(section["base_frame"] == "gimbal_yaw",
                f"{node}.base_frame 必须是 gimbal_yaw（开机快照不能含关节角，方案 §4.7）")
        _assert(section.get("ground_frame") == "base_link",
                f"{node}.ground_frame 必须补偿云台高度，否则代价地图的绝对 z 阈值整体上移")

    follow_path = cfg["controller_server"]["ros__parameters"]["FollowPath"]
    _assert(follow_path["base_frame"] == "gimbal_yaw",
            "MINCO 的车体系必须是 odometry 的 child（gimbal_yaw）")
    _assert(follow_path["command_frame"] == context.launch_configurations["command_frame"],
            "参数文件的 command_frame 与 launch 默认值必须一致")
    _assert(follow_path["fake_base_frame"] == fake["fake_robot_base_frame"],
            "FollowPath.fake_base_frame 必须与 fake 节点的 fake_robot_base_frame 一致")
    _assert(follow_path["yaw_policy"]["mode"] == "xy_only",
            "实车参数文件必须是 xy_only（云台系下导航只出平移，方案 §4.2）")
    spin_request = follow_path["spin_request"]
    _assert(spin_request["topic"] == "rotation_cmd",
            f"spin_request.topic 必须是 rotation_cmd（spin_control.sh 发的就是它）: {spin_request}")
    _assert(float(spin_request["timeout"]) > 0.0, "spin_request.timeout 必须为正")

    checker = cfg["controller_server"]["ros__parameters"]["general_goal_checker"]
    _assert("OmniStoppedGoalChecker" in checker["plugin"],
            f"xy_only 下必须用只看平移的判停器（自转时 StoppedGoalChecker 永不判到点）: "
            f"{checker['plugin']}")

    for node in ("bt_navigator", "behavior_server"):
        section = cfg[node]["ros__parameters"]
        _assert(section["robot_base_frame"] == "gimbal_yaw_fake",
                f"{node}.robot_base_frame 必须是 gimbal_yaw_fake")
    for node in ("local_costmap", "global_costmap"):
        section = cfg[node][node]["ros__parameters"]
        _assert(section["robot_base_frame"] == "gimbal_yaw_fake",
                f"{node}.robot_base_frame 必须是 gimbal_yaw_fake")


def test_zlio_real_wiring_uses_g_units_and_livox():
    """ZLIO 的实车输入接线必须是 g/9.81 + livox CustomMsg + livox_imu。"""
    module = _load_launch_module()
    _, context, includes, _, _ = _make_context(module, {"params_file": str(FAKEYAW_PARAMS)})
    nav2 = _include_arguments(includes, context, "nav2_stack_launch.py")

    _assert(nav2["zlio_input_type"] == "livox", nav2)
    _assert(nav2["zlio_lidar_topic"] == "livox/lidar", nav2)
    _assert(nav2["zlio_imu_acceleration_scale"] == "9.81",
            f"实车 MID-360 输出单位是 g，必须 9.81: {nav2}")
    _assert(nav2["zlio_body_frame"] == "livox_imu", nav2)
    _assert(nav2["zlio_config"].endswith("config/real/zlio_mid360.yaml"),
            f"实车必须用 config/real 那份 ZLIO 算法配置: {nav2['zlio_config']}")


def test_yaw_mode_is_injected_into_the_params():
    """yaw_mode 要真的改写参数文件，且判停器跟着换（方案 §4.2）。

    实车默认 xy_only：导航只出平移，底盘 wz 由决策层 rotation_cmd 给。
    se2 / auto 都必须被改写（se2 在云台系下不成立），spin 保留但同样用平移判停器。
    """
    import yaml

    module = _load_launch_module()
    for requested, expected_mode in (
        ("auto", "xy_only"),
        ("xy_only", "xy_only"),
        ("se2", "xy_only"),
        ("spin", "spin"),
    ):
        _, context, includes, _, _ = _make_context(
            module, {"params_file": str(FAKEYAW_PARAMS), "yaw_mode": requested}
        )
        nav2 = _include_arguments(includes, context, "nav2_stack_launch.py")
        path = _sub(context, nav2["params_file"])
        cfg = yaml.safe_load(open(path))
        mode = cfg["controller_server"]["ros__parameters"]["FollowPath"]["yaw_policy"]["mode"]
        plugin = cfg["controller_server"]["ros__parameters"]["general_goal_checker"]["plugin"]
        _assert(mode == expected_mode,
                f"yaw_mode={requested} 应注入成 {expected_mode}，实际 {mode}")
        _assert("OmniStoppedGoalChecker" in plugin,
                f"yaw_mode={requested} 的判停器必须只看平移（自转中停不下来）: {plugin}")


def test_mux_output_must_be_the_serial_topic():
    """mux 出口被改成别的名字时必须直接报错（否则串口收不到速度、车不动）。"""
    module = _load_launch_module()
    _, context, _, _, functions = _make_context(
        module, {"params_file": str(FAKEYAW_PARAMS), "mux_output_topic": "cmd_vel_sim"}
    )
    try:
        _run_speed_chain_check(functions, context)
    except RuntimeError as error:
        _assert("cmd_vel_chassis" in str(error),
                f"报错信息应指明串口节点订阅的话题: {error}")
    else:
        raise AssertionError("mux 出口写成 cmd_vel_sim 没有被拦下来")


def test_old_chain_and_bad_frame_are_rejected():
    """实车入口不接受"不迁移"的配置。"""
    module = _load_launch_module()
    _, context, _, _, functions = _make_context(
        module, {"params_file": str(MINCO_PARAMS), "use_fakeyaw_transform": "False"}
    )
    try:
        _run_speed_chain_check(functions, context)
    except RuntimeError as error:
        _assert("fakeyaw" in str(error), f"报错应说明必须走 fakeyaw 链: {error}")
    else:
        raise AssertionError("use_fakeyaw_transform=False 没有被拦下来")

    _, context, _, _, functions = _make_context(module, {"params_file": str(FAKEYAW_PARAMS)})
    context.launch_configurations["command_frame"] = "gimbal_yaw"
    try:
        _run_speed_chain_check(functions, context)
    except RuntimeError as error:
        _assert("command_frame" in str(error), f"报错应指明 command_frame: {error}")
    else:
        raise AssertionError("不一致的 command_frame 没有被拦下来")


def test_gimbal_relay_is_always_started_and_follows_the_topic_arg():
    """云台关节角中继必须常驻，输入类型随 gimbal_joint_topic 切换。

    revolute 关节的 `base_link -> gimbal_yaw` 这条 TF 只来自 `/joint_states`，
    没有中继时 `robot_state_publisher` 一个动态 TF 都不发，整棵树断开。所以中继
    不能"没话题就不起"：
      · 给了话题 -> float32，用真实回传角（0x0010 第 4 字段，2026-11-08 起可用）；
      · 没给     -> constant 暂位 0 度，TF 与 fixed 等价，结构保持不断。
    """
    module = _load_launch_module()
    _, context, _, nodes, _ = _make_context(
        module,
        {"params_file": str(FAKEYAW_PARAMS), "gimbal_joint_topic": "gimbal_joint_angle"},
    )
    relay = [n for n in nodes
             if getattr(n, "_Node__node_executable", "") == "gimbal_joint_state_relay.py"]
    _assert(len(relay) == 1, f"应该有一个云台关节角中继节点: {len(relay)}")

    def _param(node, context, key):
        # launch_ros 把参数字典归一化成 {(TextSubstitution,): value}，value 可能是
        # tuple[Substitution] / ParameterValue / 字面量，这里统一取出可求值的东西。
        for entry in getattr(node, "_Node__parameters"):
            if not isinstance(entry, dict):
                continue
            for raw_key, raw_value in entry.items():
                if _sub(context, raw_key) != key:
                    continue
                if hasattr(raw_value, "value"):  # ParameterValue
                    return raw_value.value
                return raw_value
        return None

    input_topic = _param(relay[0], context, "input_topic")
    _assert(input_topic is not None, "中继必须声明 input_topic")
    _assert(_sub(context, input_topic) == "gimbal_joint_angle",
            f"input_topic 应跟随 gimbal_joint_topic，实得 {_sub(context, input_topic)}")

    input_type = _param(relay[0], context, "input_type")
    _assert(input_type is not None, "中继必须声明 input_type")
    _assert(_sub(context, input_type) == "float32",
            f"给了话题就该走 float32，实得 {_sub(context, input_type)}")

    # 话题为空 -> constant 暂位（结构已就绪，TF 不断）
    _, empty_context, _, _, _ = _make_context(
        module, {"params_file": str(FAKEYAW_PARAMS), "gimbal_joint_topic": ""})
    _assert(_sub(empty_context, input_type) == "constant",
            f"话题为空时应退回 constant，实得 {_sub(empty_context, input_type)}")


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
