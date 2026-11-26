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

"""Z-Localization backend 模式的 launch 接线回归（方案 §3.2 第 5、6 条）。

为什么值得单测：这条链接错的表现**全部是静默的**（问题分析 §B1/B5/B6）。

* 静态 `map→odom` 没关：backend 与 `static_transform_publisher` 同时发同一条 TF，
  RViz/代价地图看到两个全局位姿源，地图来回跳，没有任何节点报错。
* `coordinated_reset` 没翻成 `true`：前端自己换代 + backend 再来一次事务，
  一次断流 = 两次换代 + 一次被 bridge 锚定的全局跳变。
* `backend_sensor_frame` 没跟上 `zlio_body_frame`：backend 逐帧校验
  `child_frame_id == common.sensor_frame`，不符就**静默丢帧**（永远 INITALIZING）。
* 冲突没在启动前失败：两个 `map→odom` 所有者同时起来，现场表现为"车定位到错的位置"。

本测试**不启动任何节点**：只构造 LaunchDescription、执行无副作用的
`DeclareLaunchArgument` / `OpaqueFunction`，再对实体求值 condition / 参数。
"""

import importlib.util
import sys
import tempfile
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_DIR = PACKAGE_ROOT / "launch"
SIM_PARAMS = PACKAGE_ROOT / "config" / "simulation" / "current" / "nav2_params_srm_fakeyaw.yaml"
SIM_MAP = PACKAGE_ROOT / "map" / "simulation" / "rmuc_2025_tunnel.yaml"


def _assert(condition, message):
    if not condition:
        raise AssertionError(message)


def _render(value, context=None):
    """把 substitution / 字符串 / 列表统一渲染成字符串。"""
    if context is not None and not isinstance(value, (list, tuple)):
        try:
            from launch.utilities import (
                normalize_to_list_of_substitutions,
                perform_substitutions,
            )

            return perform_substitutions(
                context, normalize_to_list_of_substitutions(value)
            )
        except Exception:  # noqa: BLE001 - 求值失败退回静态渲染
            pass
    if isinstance(value, (list, tuple)):
        return "".join(_render(item, context) for item in value)
    text = getattr(value, "text", None)
    if isinstance(text, str):
        return text
    return str(value)


def _label(entity):
    from launch_ros.actions import LoadComposableNodes, Node

    if isinstance(entity, Node):
        name = getattr(entity, "_Node__node_name", "")
        executable = getattr(entity, "_Node__node_executable", "")
        return f"node:{_render(name) or _render(executable)}"
    if isinstance(entity, LoadComposableNodes):
        descriptions = getattr(
            entity, "_LoadComposableNodes__composable_node_descriptions", []
        ) or []
        return ",".join(
            f"composable:{_render(getattr(d, 'node_name', ''))}" for d in descriptions
        )
    return f"other:{type(entity).__name__}"


def _load_launch_module(module_name, filename):
    path = LAUNCH_DIR / filename
    spec = importlib.util.spec_from_file_location(module_name, str(path))
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _prepare(module, overrides):
    """构造 LaunchDescription，灌好覆盖值，执行无副作用的声明/校验动作。"""
    from launch import LaunchContext
    from launch.actions import (
        DeclareLaunchArgument,
        OpaqueFunction,
        SetLaunchConfiguration,
    )

    description = module.generate_launch_description()
    context = LaunchContext()
    for key, value in overrides.items():
        context.launch_configurations[key] = value
    for entity in description.entities:
        if isinstance(entity, (DeclareLaunchArgument, SetLaunchConfiguration)):
            entity.execute(context)
        elif isinstance(entity, OpaqueFunction):
            entity.execute(context)
    return description, context


def _active_labels(description, context):
    """返回"会被启动"的实体标签集合（不展开 IncludeLaunchDescription）。"""
    from launch.actions import GroupAction
    from launch_ros.actions import LoadComposableNodes, Node

    active = []

    def walk(entities):
        for entity in entities:
            condition = getattr(entity, "condition", None)
            enabled = True if condition is None else bool(condition.evaluate(context))
            if isinstance(entity, GroupAction):
                if enabled:
                    walk(entity.get_sub_entities())
                continue
            if isinstance(entity, (Node, LoadComposableNodes)) and enabled:
                active.extend(_label(entity).split(","))

    walk(description.entities)
    return set(active)


def _include_for(description, needle):
    """按"launch 文件路径里包含 needle"找 IncludeLaunchDescription。

    humble 的 `PythonLaunchDescriptionSource` 把路径存在 `__location`（substitution
    列表）里，不是一个可以直接读的字符串属性，所以这里渲染它再匹配。
    """
    from launch.actions import GroupAction, IncludeLaunchDescription

    found = []

    def walk(entities):
        for entity in entities:
            if isinstance(entity, GroupAction):
                walk(entity.get_sub_entities())
                continue
            if not isinstance(entity, IncludeLaunchDescription):
                continue
            source = entity.launch_description_source
            location = getattr(source, "_LaunchDescriptionSource__location", None)
            text = _render(location) if location is not None else ""
            if needle in text:
                found.append(entity)

    walk(description.entities)
    return found[0] if found else None


def _include_enabled(include, context):
    condition = getattr(include, "condition", None)
    return True if condition is None else bool(condition.evaluate(context))


def _node_parameter(module, overrides, node_name, parameter):
    """取某个 Node 的参数标量（兼容 ParameterValue 与普通 substitution）。"""
    from launch_ros.actions import Node

    description, context = _prepare(module, overrides)
    for entity in description.entities:
        if not isinstance(entity, Node):
            continue
        if _render(getattr(entity, "_Node__node_name", "")) != node_name:
            continue
        for parameters in getattr(entity, "_Node__parameters", []):
            if not isinstance(parameters, dict):
                continue
            for key, value in parameters.items():
                if _render(key, context) != parameter:
                    continue
                evaluate = getattr(value, "evaluate", None)
                if callable(evaluate):
                    evaluated = evaluate(context)
                    return evaluated if isinstance(evaluated, bool) else _render(evaluated, context)
                return _render(value, context).strip("'\"")
    return None


# nav2_stack_launch 把 prior_pcd_file 声明成必填（无默认值），任何"执行声明"的
# 测试都必须显式给一个值（内容不会被读取，只有 backend 模式才会用到它）。
PRIOR_PCD = "/tmp/srm27-test-prior.pcd"


def _backend_overrides(**extra):
    """backend 模式的一组合法覆盖值（先验地图用一个绝对临时目录）。"""
    prior_map = tempfile.mkdtemp(prefix="zloc-prior-")
    overrides = {
        "namespace": "",
        "params_file": str(SIM_PARAMS),
        "map": str(SIM_MAP),
        "prior_pcd_file": PRIOR_PCD,
        "use_sim_time": "False",
        "use_composition": "False",
        "use_zlocalization_backend": "True",
        "use_lio_odometry": "True",
        "use_pcd_localization": "False",
        "odometry_frontend": "zlio",
        "zlocalization_prior_enabled": "True",
        "zlocalization_prior_initialization_only": "False",
        "zlocalization_prior_map": prior_map,
    }
    overrides.update(extra)
    return overrides


def _base_overrides(**extra):
    overrides = {
        "namespace": "",
        "params_file": str(SIM_PARAMS),
        "map": str(SIM_MAP),
        "prior_pcd_file": PRIOR_PCD,
        "use_sim_time": "False",
        "use_composition": "False",
        "use_lio_odometry": "True",
        "use_pcd_localization": "False",
        "odometry_frontend": "zlio",
    }
    overrides.update(extra)
    return overrides


# ---------------------------------------------------------------------------
# localization_launch.py
# ---------------------------------------------------------------------------
def test_default_mode_keeps_static_tf_and_never_starts_backend():
    """默认（use_zlocalization_backend 缺省 False）：静态 map->odom 照旧，backend 不启动。"""
    module = _load_launch_module("srm_zloc_default", "localization_launch.py")
    description, context = _prepare(module, _base_overrides())
    active = _active_labels(description, context)
    _assert(
        "node:static_transform_publisher_map2odom" in active,
        f"默认模式静态 map->odom 被误关: {sorted(active)}",
    )
    include = _include_for(description, "backend.launch.py")
    _assert(include is not None, "localization_launch 里没有 backend.launch.py 的 include")
    _assert(not _include_enabled(include, context), "backend include 默认必须是关闭的")
    _assert(
        _node_parameter(module, _base_overrides(), "zlio", "coordinated_reset") is False,
        "非 backend 模式 coordinated_reset 必须保持 false（前端自己换代）",
    )
    _assert(
        _node_parameter(module, _base_overrides(), "zlio_bridge", "backend_enabled") is False,
        "非 backend 模式 zlio_bridge 不应打开 backend 输出",
    )


def test_backend_mode_owns_map_to_odom_and_disables_static_tf():
    """backend 模式：静态 TF 关掉、backend include 打开，且参数按本仓库接线传入。"""
    module = _load_launch_module("srm_zloc_on", "localization_launch.py")
    overrides = _backend_overrides()
    description, context = _prepare(module, overrides)
    active = _active_labels(description, context)
    _assert(
        "node:static_transform_publisher_map2odom" not in active,
        f"backend 模式必须关掉静态 map->odom（否则两个发布者抢同一条 TF）: {sorted(active)}",
    )
    _assert("node:zlio" in active, "backend 模式仍需要 ZLIO 前端")
    _assert(
        "node:point_lio" not in active,
        "backend 只吃 FrontendFrame，不能同时起 point_lio",
    )
    include = _include_for(description, "backend.launch.py")
    _assert(include is not None and _include_enabled(include, context), "backend include 未启用")
    arguments = {key: _render(value, context) for key, value in include.launch_arguments}
    _assert(arguments.get("prior_enabled") == "True", "先验开关必须显式透传")
    _assert("mode" not in arguments, "不可再用 localization 模式强制开启先验")
    _assert(
        arguments.get("prior_map") == overrides["zlocalization_prior_map"],
        f"先验地图包未透传: {arguments.get('prior_map')!r}",
    )
    _assert(arguments.get("odom_frame") == "odom", "backend 的 odom_frame 必须是本仓库 odom")
    _assert(
        arguments.get("sensor_frame") == "front_mid360",
        f"backend sensor_frame 必须等于 zlio_body_frame，实际 {arguments.get('sensor_frame')!r}",
    )
    _assert(arguments.get("rviz") == "false", "导航链里不应顺带起 backend 的 RViz")
    _assert(
        _node_parameter(module, overrides, "zlio", "coordinated_reset") is True,
        "backend 模式 coordinated_reset 必须为 true（等 backend 协调换代）",
    )
    _assert(
        _node_parameter(module, overrides, "zlio_bridge", "backend_enabled") is True,
        "backend 模式 zlio_bridge 必须打开 backend 输出",
    )
    _assert(
        _node_parameter(module, overrides, "zlio_bridge", "backend_sensor_frame") == "front_mid360",
        "zlio_bridge 的 backend_sensor_frame 必须跟随 zlio_body_frame",
    )


def test_backend_sensor_frame_follows_zlio_body_frame():
    """实车把 zlio_body_frame 换成 livox_imu 时，两处 sensor_frame 一起变。"""
    module = _load_launch_module("srm_zloc_real_frame", "localization_launch.py")
    overrides = _backend_overrides(zlio_body_frame="livox_imu")
    description, context = _prepare(module, overrides)
    include = _include_for(description, "backend.launch.py")
    arguments = {key: _render(value, context) for key, value in include.launch_arguments}
    _assert(arguments.get("sensor_frame") == "livox_imu", "backend sensor_frame 未跟随实车")
    _assert(
        _node_parameter(module, overrides, "zlio_bridge", "backend_sensor_frame") == "livox_imu",
        "zlio_bridge 的 backend_sensor_frame 未跟随实车",
    )


def test_backend_mode_conflicts_fail_before_start():
    """每一种"两个 map->odom 所有者"或"输入不成立"的组合都必须在启动前失败。"""
    module = _load_launch_module("srm_zloc_conflicts", "localization_launch.py")
    cases = {
        "GICP 也发 map->odom": {"use_pcd_localization": "True"},
        "point_lio 没有 FrontendFrame": {"odometry_frontend": "point_lio"},
        "缺里程计链路": {"use_lio_odometry": "False"},
        "缺先验地图包": {"zlocalization_prior_map": ""},
        "先验地图包是相对路径": {"zlocalization_prior_map": "maps/pcd/x"},
        "backend 配置不存在": {"zlocalization_backend_config": "/tmp/definitely-missing.yaml"},
    }
    for name, overrides in cases.items():
        try:
            _prepare(module, _backend_overrides(**overrides))
        except RuntimeError as error:
            _assert("use_zlocalization_backend" in str(error), f"{name}: 报错信息不指向 backend")
        else:
            raise AssertionError(f"{name}: backend 模式应在启动前失败，实际通过了校验")


def test_backend_mode_requires_the_backend_package():
    """backend 包没构建时，默认链路照常，只有显式打开 backend 才报错。"""
    module = _load_launch_module("srm_zloc_missing_pkg", "localization_launch.py")
    original = module._backend_package_share
    try:
        module._backend_package_share = lambda: ""
        description, context = _prepare(module, _base_overrides())
        active = _active_labels(description, context)
        _assert(
            "node:static_transform_publisher_map2odom" in active,
            "backend 包缺失时默认链路必须照常（静态 map->odom 不能丢）",
        )
        _assert(_include_for(description, "backend.launch.py") is None,
                "backend 包缺失时不应生成 include 动作")
        try:
            _prepare(module, _backend_overrides())
        except RuntimeError as error:
            _assert("srm27_zlocalization_backend" in str(error), "缺包报错应点名包名")
        else:
            raise AssertionError("backend 包缺失时打开 backend 模式应失败")
    finally:
        module._backend_package_share = original


# ---------------------------------------------------------------------------
# nav2_stack_launch.py / 入口 launch
# ---------------------------------------------------------------------------
def test_nav2_stack_rejects_backend_with_slam():
    """slam 链路由 slam_toolbox 发 map->odom，与 backend 互斥。"""
    module = _load_launch_module("srm_zloc_stack_slam", "nav2_stack_launch.py")
    try:
        _prepare(module, _backend_overrides(slam="True"))
    except RuntimeError as error:
        _assert("slam" in str(error), f"报错信息应点名 slam: {error}")
    else:
        raise AssertionError("backend + slam 应在启动前失败")


def test_nav2_stack_forwards_backend_arguments():
    """nav2_stack 必须把路径和三项独立开关透传给 localization_launch。"""
    module = _load_launch_module("srm_zloc_stack_fwd", "nav2_stack_launch.py")
    description, context = _prepare(module, _backend_overrides())
    include = _include_for(description, "localization_launch.py")
    _assert(include is not None, "nav2_stack 里没有 localization_launch 的 include")
    arguments = {key: _render(value, context) for key, value in include.launch_arguments}
    _assert(arguments.get("use_zlocalization_backend") == "True", "backend 开关未透传")
    for key in ("zlocalization_backend_config", "zlocalization_prior_map",
                "zlocalization_trajectory_directory", "zlocalization_prior_enabled",
                "zlocalization_prior_initialization_only",
                "zlocalization_loop_enabled", "zlocalization_uwb_enabled"):
        _assert(key in arguments, f"nav2_stack 未透传 {key}")


def test_entry_launches_forward_backend_arguments():
    """两个正式入口都要透传 4 个 backend 参数，且默认关闭。"""
    for module_name, filename, extra in (
        ("srm_zloc_entry_sim", "nav_srm_simulation_launch.py", {"slam": "False"}),
        ("srm_zloc_entry_real", "nav_srm_real_launch.py", {"slam": "False"}),
    ):
        module = _load_launch_module(module_name, filename)
        description, context = _prepare(module, {
            **extra,
            "slam": "False",
            "use_sim_time": "False",
            "map": str(SIM_MAP),
            "prior_pcd_file": PRIOR_PCD,
        })
        include = _include_for(description, "nav2_stack_launch.py")
        _assert(include is not None, f"{filename} 里没有 nav2_stack 的 include")
        arguments = {key: _render(value, context) for key, value in include.launch_arguments}
        for key in ("use_zlocalization_backend", "zlocalization_backend_config",
                    "zlocalization_prior_map", "zlocalization_trajectory_directory",
                    "zlocalization_prior_initialization_only"):
            _assert(key in arguments, f"{filename} 未透传 {key}")
        _assert(
            arguments["use_zlocalization_backend"] == "False",
            f"{filename} 的 backend 默认值必须是 False（默认行为不变）",
        )
        for feature in ("prior", "loop", "uwb"):
            _assert(arguments[f"zlocalization_{feature}_enabled"] == "False",
                    f"{filename} 的 {feature} 默认必须关闭")
        _assert(
            arguments["zlocalization_prior_initialization_only"] == "False",
            f"{filename} 的 prior-once 默认必须关闭（默认行为不变）",
        )


def test_prior_initialization_only_flows_through_every_layer():
    """--zloc-prior-once 必须是独立开关：四层都透传，且不隐式改 prior_enabled。"""
    cases = (
        ("nav_srm_real_launch.py", "nav2_stack_launch.py",
         "zlocalization_prior_initialization_only"),
        ("nav_srm_simulation_launch.py", "nav2_stack_launch.py",
         "zlocalization_prior_initialization_only"),
        ("nav2_stack_launch.py", "localization_launch.py",
         "zlocalization_prior_initialization_only"),
        ("localization_launch.py", "backend.launch.py", "prior_initialization_only"),
    )
    for filename, child, target in cases:
        module = _load_launch_module("prior_once_" + filename, filename)
        for value in ("True", "False"):
            description, context = _prepare(
                module,
                _backend_overrides(slam="False",
                                   zlocalization_prior_initialization_only=value),
            )
            include = _include_for(description, child)
            _assert(include is not None, f"{filename} 里没有 {child} 的 include")
            arguments = {key: _render(v, context) for key, v in include.launch_arguments}
            _assert(arguments.get(target) == value,
                    f"{filename} 未按 {value} 透传 {target}: {arguments.get(target)}")


def test_all_eight_feature_combinations_survive_each_launch_layer():
    """三开关在实车/仿真/stack/localization 四层均独立，不要求关闭的先验有地图。"""
    for filename, child in (
        ("nav_srm_real_launch.py", "nav2_stack_launch.py"),
        ("nav_srm_simulation_launch.py", "nav2_stack_launch.py"),
        ("nav2_stack_launch.py", "localization_launch.py"),
        ("localization_launch.py", "backend.launch.py"),
    ):
        module = _load_launch_module("matrix_" + filename, filename)
        for mask in range(8):
            flags = {f"zlocalization_{key}_enabled": str(bool(mask & bit))
                     for key, bit in (("prior", 1), ("loop", 2), ("uwb", 4))}
            overrides = _backend_overrides(slam="False", **flags)
            if not mask & 1:
                overrides["zlocalization_prior_map"] = ""
            description, context = _prepare(module, overrides)
            include = _include_for(description, child)
            arguments = {key: _render(value, context) for key, value in include.launch_arguments}
            for name, value in flags.items():
                target = name.removeprefix("zlocalization_") if child == "backend.launch.py" else name
                _assert(arguments[target] == value, f"{filename} mask={mask} 丢失 {target}")


def main():
    tests = [
        test_default_mode_keeps_static_tf_and_never_starts_backend,
        test_backend_mode_owns_map_to_odom_and_disables_static_tf,
        test_backend_sensor_frame_follows_zlio_body_frame,
        test_backend_mode_conflicts_fail_before_start,
        test_backend_mode_requires_the_backend_package,
        test_nav2_stack_rejects_backend_with_slam,
        test_nav2_stack_forwards_backend_arguments,
        test_entry_launches_forward_backend_arguments,
        test_prior_initialization_only_flows_through_every_layer,
        test_all_eight_feature_combinations_survive_each_launch_layer,
    ]
    for test in tests:
        test()
        print(f"[OK] {test.__name__}")
    print(f"\n全部通过: {len(tests)} 项")
    return 0


if __name__ == "__main__":
    sys.exit(main())
