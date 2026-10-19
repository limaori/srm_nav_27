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

"""wz 控制迁移 / fakeyaw 的 launch 接线回归（方案 §4.4）。

这一层错了的表现是"车不动"或"重复旋转"，而且从日志上很难直接看出来，所以把三件事测住：

1. **默认值跟随参数文件**：用 fakeyaw 那份配置启动时，`use_fakeyaw_transform` /
   `wz_control_mode` / `command_frame` / `cmd_vel_nav_topic` 必须自动切到迁移链路；
   用旧基线配置启动时必须保持旧行为（否则旧链路会被静默改掉）。
2. **一致性核对真的会拒绝**：参数文件里的 `FollowPath.command_frame` 与 launch 的
   `command_frame` 不一致时必须直接报错，而不是让"转一次还是转两次"变成隐含约定。
3. **节点接线正确**：Nav2 的速度出口 == fake_vel_transform 的输入 == fakeyaw 配置里的
   `input_cmd_vel_topic`；mux 的 `nav_topic` == fake 的输出；fakeyaw 链上独立自转
   发送器必须是关的。

测试不启动任何节点：只构造 LaunchDescription、求值替换、跑一个 OpaqueFunction。
"""

import os
import sys
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_FILE = PACKAGE_ROOT / "launch" / "nav_srm_simulation_launch.py"
CONFIG_DIR = PACKAGE_ROOT / "config" / "simulation"

FAKEYAW_PARAMS = CONFIG_DIR / "nav2_params_srm_fakeyaw.yaml"
MINCO_PARAMS = CONFIG_DIR / "nav2_params_srm_minco.yaml"
BASELINE_PARAMS = CONFIG_DIR / "simulation" / "nav2_params_srm.yaml"
if not BASELINE_PARAMS.is_file():  # pragma: no cover - 正常路径
    BASELINE_PARAMS = CONFIG_DIR / "nav2_params_srm.yaml"


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

    spec = importlib.util.spec_from_file_location("srm_fakeyaw_launch", str(LAUNCH_FILE))
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _make_context(module, overrides):
    """构造 LaunchDescription 并把声明参数的默认值灌进 LaunchContext。"""
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction

    description = module.generate_launch_description()
    context = LaunchContext()
    context.launch_configurations["params_file"] = overrides["params_file"]

    arguments = [e for e in description.entities if isinstance(e, DeclareLaunchArgument)]
    # 默认值之间互相引用（command_frame 依赖 params_file 是否 fakeyaw），所以多轮求值直到稳定。
    pending = list(arguments)
    for _ in range(len(arguments) + 1):
        progressed = False
        for argument in list(pending):
            name = _sub(context, argument.name)
            # overrides 优先：params_file 由调用方给定，其余也允许显式覆盖默认值。
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
    functions = [e for e in description.entities if isinstance(e, OpaqueFunction)]
    return description, context, includes, functions


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


def _chassis_include_arguments(includes, context):
    return _include_arguments(includes, context, "srm_chassis_control.launch.py")


def _nav2_include_arguments(includes, context):
    return _include_arguments(includes, context, "nav2_stack_launch.py")


def test_fakeyaw_config_turns_the_whole_chain_on():
    module = _load_launch_module()
    _, context, includes, functions = _make_context(
        module, {"params_file": str(FAKEYAW_PARAMS)}
    )

    _assert(context.launch_configurations["use_fakeyaw_transform"] == "True",
            f"fakeyaw 配置应默认启用迁移链路: {context.launch_configurations}")
    _assert(context.launch_configurations["wz_control_mode"] == "nav",
            "fakeyaw 配置应默认 wz 归导航所有")
    _assert(context.launch_configurations["command_frame"] == "base_link_fake",
            "fakeyaw 配置的命令系必须是 base_link_fake")
    _assert(context.launch_configurations["cmd_vel_nav_topic"] == "cmd_vel_nav_fake",
            "fakeyaw 链上 Nav2 的输出必须是 cmd_vel_nav_fake")

    nav2 = _nav2_include_arguments(includes, context)
    _assert(nav2["use_fake_vel_transform"] == "True", f"Nav2 入口必须打开 fake 转换器: {nav2}")
    _assert(nav2["cmd_vel_nav_topic"] == "cmd_vel_nav_fake", f"Nav2 输出话题不对: {nav2}")

    chassis = _chassis_include_arguments(includes, context)
    _assert(chassis["nav_topic"] == "cmd_vel_nav_body",
            f"mux 的 nav_topic 必须是 fake 转换器的输出: {chassis}")
    _assert(chassis["wz_control_mode"] == "nav_se2", f"mux 必须是 nav_se2: {chassis}")
    _assert(chassis["start_rotation_sender"] == "False",
            f"迁移链上必须关掉独立自转发送器: {chassis}")

    messages = "\n".join(_run_speed_chain_check(functions, context))
    _assert("base_link_fake" in messages and "cmd_vel_nav_body" in messages,
            f"启动日志必须打印最终速度链: {messages}")


def test_params_file_topics_match_the_launch_wiring():
    """参数文件里 fake 节点的话题必须与 launch 的接线逐一对应（否则链路断开）。"""
    import yaml

    module = _load_launch_module()
    _, context, includes, _ = _make_context(module, {"params_file": str(FAKEYAW_PARAMS)})
    cfg = yaml.safe_load(FAKEYAW_PARAMS.read_text())
    fake = cfg["fake_vel_transform"]["ros__parameters"]
    nav2 = _nav2_include_arguments(includes, context)
    chassis = _chassis_include_arguments(includes, context)

    _assert(fake["input_cmd_vel_topic"] == nav2["cmd_vel_nav_topic"],
            f"fake 输入 {fake['input_cmd_vel_topic']} != Nav2 输出 {nav2['cmd_vel_nav_topic']}")
    _assert(fake["output_cmd_vel_topic"] == chassis["nav_topic"],
            f"fake 输出 {fake['output_cmd_vel_topic']} != mux 输入 {chassis['nav_topic']}")
    _assert(fake["transform_mode"] == "nav_se2", "fakeyaw 配置的 transform_mode 必须是 nav_se2")
    _assert(fake["robot_base_frame"] == "base_link" and
            fake["fake_robot_base_frame"] == "base_link_fake",
            "fake 节点的 TF 父子关系必须与 TF 拓扑一致")

    follow_path = cfg["controller_server"]["ros__parameters"]["FollowPath"]
    _assert(follow_path["command_frame"] == context.launch_configurations["command_frame"],
            "参数文件的 command_frame 与 launch 默认值必须一致")
    _assert(follow_path["fake_base_frame"] == fake["fake_robot_base_frame"],
            "FollowPath.fake_base_frame 必须与 fake 节点的 fake_robot_base_frame 一致")
    _assert(follow_path["yaw_policy"]["mode"] == "se2",
            "fakeyaw 配置必须让 MPC 拥有 wz（yaw_policy.mode=se2）")
    goal_checker = cfg["controller_server"]["ros__parameters"]["general_goal_checker"]
    _assert("StoppedGoalChecker" in goal_checker["plugin"],
            f"迁移配置必须用带角速度停稳判定的检查器: {goal_checker['plugin']}")
    _assert(float(goal_checker["rot_stopped_velocity"]) > 0.0,
            "rot_stopped_velocity 必须显式给出正数")
    for node, _ in (("bt_navigator", None), ("local_costmap", None), ("global_costmap", None)):
        section = cfg[node]
        section = section.get(node, section)["ros__parameters"]
        _assert(section["robot_base_frame"] == "base_link_fake",
                f"{node}.robot_base_frame 必须是 base_link_fake")


def test_baseline_config_keeps_the_old_chain():
    module = _load_launch_module()
    for params in (BASELINE_PARAMS, MINCO_PARAMS):
        _, context, includes, functions = _make_context(module, {"params_file": str(params)})
        _assert(context.launch_configurations["use_fakeyaw_transform"] == "False",
                f"{params.name} 不该默认打开迁移链路")
        _assert(context.launch_configurations["wz_control_mode"] == "rotation",
                f"{params.name} 的 wz 应仍来自独立自转链路")
        _assert(context.launch_configurations["command_frame"] == "base_link",
                f"{params.name} 的命令系应是真实车体系")
        _assert(context.launch_configurations["cmd_vel_nav_topic"] == "cmd_vel_nav",
                f"{params.name} 的输出话题应保持 cmd_vel_nav")
        chassis = _chassis_include_arguments(includes, context)
        _assert(chassis["nav_topic"] == "cmd_vel_nav", f"旧链路 mux 输入不对: {chassis}")
        _assert(chassis["wz_control_mode"] == "legacy_independent_rotation",
                f"旧链路 mux 模式不对: {chassis}")
        messages = "\n".join(_run_speed_chain_check(functions, context))
        _assert("legacy" in messages, f"旧链路日志应说明 wz 来源: {messages}")


def test_inconsistent_command_frame_is_rejected():
    """参数文件说 base_link、launch 说 base_link_fake（或反过来）必须直接失败。"""
    module = _load_launch_module()
    _, context, _, functions = _make_context(module, {"params_file": str(FAKEYAW_PARAMS)})
    # 伪造：参数文件是 fakeyaw 的（command_frame=base_link_fake），但 launch 传 base_link。
    context.launch_configurations["command_frame"] = "base_link"
    try:
        _run_speed_chain_check(functions, context)
    except RuntimeError as error:
        _assert("command_frame" in str(error), f"报错信息应指明 command_frame: {error}")
    else:
        raise AssertionError("不一致的 command_frame 没有被拦下来")

    # 反向：没有 fake 转换节点却要求 base_link_fake 命令。
    _, context, _, functions = _make_context(module, {"params_file": str(BASELINE_PARAMS)})
    context.launch_configurations["command_frame"] = "base_link_fake"
    try:
        _run_speed_chain_check(functions, context)
    except RuntimeError:
        pass
    else:
        raise AssertionError("没有 fake 转换节点时的 base_link_fake 没有被拦下来")


def test_bad_wz_control_mode_is_rejected():
    module = _load_launch_module()
    _, context, _, functions = _make_context(module, {"params_file": str(FAKEYAW_PARAMS)})
    context.launch_configurations["wz_control_mode"] = "guess_from_sign"
    try:
        _run_speed_chain_check(functions, context)
    except RuntimeError as error:
        _assert("wz_control_mode" in str(error), str(error))
    else:
        raise AssertionError("非法的 wz_control_mode 没有被拦下来")


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
