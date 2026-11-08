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

"""`--lio` 里程计前端（point_lio / zlio）的 launch 接线回归（方案 §6.2）。

为什么值得单测：这条链配错的表现**全部是静默的**。

* 换成 zlio 时如果 `loam_interface` 没关：它与 `zlio_bridge` 抢同一个
  `registered_scan` / `lidar_odometry`，`terrain_analysis` 吃到跳变的点云，不报错。
* 如果两条启动路径只关了一条（`Node` 与 `LoadComposableNodes` 是两条独立路径，
  且 `use_composition` 的默认值在两个入口里不同）：组合模式下照样会起 `loam_interface`。
* 如果 `odometry_frontend` 拼错：两条条件同时不成立，`odom -> base_link` 直接消失，
  车不动但没有任何报错。

本测试**不启动任何节点**：只构造 LaunchDescription、执行无副作用的
`DeclareLaunchArgument` / `OpaqueFunction`，然后对每个 `Node` / `LoadComposableNodes`
求值它的 `condition`，看它到底会不会被启动。
"""

import importlib.util
import sys
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_DIR = PACKAGE_ROOT / "launch"


def _assert(condition, message):
    if not condition:
        raise AssertionError(message)


def _render(value, context=None):
    """把 substitution / 字符串 / 列表统一渲染成字符串（只用于断言与报错信息）。

    给了 `context` 时**先尝试真正求值**（`LaunchConfiguration` 只有在上下文里才有值）：
    ZLIO 的输入接线已经从"写死的字符串"改成 launch 参数了（仿真/实车不同），
    断言必须看求值结果，而不是 substitution 的 repr。
    """
    if context is not None and not isinstance(value, (list, tuple)):
        try:
            from launch.utilities import (
                normalize_to_list_of_substitutions,
                perform_substitutions,
            )

            return perform_substitutions(
                context, normalize_to_list_of_substitutions(value)
            )
        except Exception:  # noqa: BLE001 - 求值失败就退回下面的静态渲染
            pass
    if isinstance(value, (list, tuple)):
        return "".join(_render(item, context) for item in value)
    text = getattr(value, "text", None)
    if isinstance(text, str):
        return text
    return str(value)


def _label(entity):
    """给实体一个可断言的短标签（用节点名，避免依赖包名/可执行名的内部属性）。"""
    from launch_ros.actions import LoadComposableNodes, Node

    if isinstance(entity, Node):
        name = getattr(entity, "_Node__node_name", "")
        executable = getattr(entity, "_Node__node_executable", "")
        return f"node:{_render(name) or _render(executable)}"
    if isinstance(entity, LoadComposableNodes):
        descriptions = getattr(
            entity, "_LoadComposableNodes__composable_node_descriptions", []
        ) or []
        parts = []
        for description in descriptions:
            plugin = _render(getattr(description, "node_plugin", ""))
            name = _render(getattr(description, "node_name", ""))
            parts.append(f"composable:{name or plugin}")
        return ",".join(parts)
    return f"other:{type(entity).__name__}"


def _node_parameter(module, overrides, node_name, parameter):
    """从某个 Node 的 parameters 里取一个标量参数（用于断言"谁订谁发"这类接线）。"""
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument, OpaqueFunction
    from launch_ros.actions import Node

    description = module.generate_launch_description()
    context = LaunchContext()
    for key, value in overrides.items():
        context.launch_configurations[key] = value
    for entity in description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
        elif isinstance(entity, OpaqueFunction):
            entity.execute(context)
    for entity in description.entities:
        if not isinstance(entity, Node):
            continue
        if _render(getattr(entity, "_Node__node_name", "")) != node_name:
            continue
        # launch_ros 会把 python 值统一成 substitution / substitution 列表，
        # 所以这里一律用 _render 渲染回字符串。
        for parameters in getattr(entity, "_Node__parameters", []):
            if not isinstance(parameters, dict):
                continue
            # 注意：键也可能是 substitution，不能直接 `parameter in dict`
            for key, value in parameters.items():
                if _render(key, context) == parameter:
                    return _render(value, context).strip("'\"")
    return None


def _load_launch_module(module_name, filename):
    path = LAUNCH_DIR / filename
    spec = importlib.util.spec_from_file_location(module_name, str(path))
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _active_labels(module, overrides):
    """构造 LaunchDescription → 灌好 launch 配置 → 返回"会被启动"的实体标签集合。"""
    from launch import LaunchContext
    from launch.actions import (
        DeclareLaunchArgument,
        GroupAction,
        OpaqueFunction,
        SetLaunchConfiguration,
    )
    from launch_ros.actions import LoadComposableNodes, Node

    description = module.generate_launch_description()
    context = LaunchContext()
    # 覆盖值先写进去：DeclareLaunchArgument 对已存在的键不再覆盖（即"用户显式给了值"）。
    for key, value in overrides.items():
        context.launch_configurations[key] = value

    active = []

    def walk(entities):
        for entity in entities:
            if isinstance(entity, SetLaunchConfiguration):
                entity.execute(context)
                continue
            if isinstance(entity, DeclareLaunchArgument):
                entity.execute(context)
                continue
            if isinstance(entity, OpaqueFunction):
                result = entity.execute(context)
                if result:
                    walk(result)
                continue
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


def test_localization_launch_frontends():
    """localization_launch：前端选择必须成对起停 point_lio / zlio + zlio_bridge。"""
    module = _load_launch_module("srm_lio_loc", "localization_launch.py")
    base = {
        "namespace": "red_standard_robot1",
        "params_file": str(PACKAGE_ROOT / "config" / "simulation" / "nav2_params_srm_fakeyaw.yaml"),
        "map": str(PACKAGE_ROOT / "map" / "simulation" / "rmuc_2025_tunnel.yaml"),
        "use_sim_time": "True",
        "use_composition": "False",
    }

    # --lio + zlio：起 zlio 与 zlio_bridge，不起 point_lio
    active = _active_labels(module, {
        **base,
        "use_lio_odometry": "True",
        "use_pcd_localization": "False",
        "odometry_frontend": "zlio",
    })
    _assert("node:zlio" in active, f"--lio-frontend zlio 时未启动 zlio: {sorted(active)}")
    _assert("node:zlio_bridge" in active, f"--lio-frontend zlio 时未启动 zlio_bridge: {sorted(active)}")
    _assert("node:point_lio" not in active, f"--lio-frontend zlio 时 point_lio 仍在启动: {sorted(active)}")
    # IMU 饱和保护：与 zlio 同条件起停，而且 zlio 必须订它的输出（否则 zlio 收不到 IMU）
    _assert(
        "node:imu_saturation_guard" in active,
        f"--lio-frontend zlio 时未启动 IMU 饱和保护: {sorted(active)}",
    )
    imu_topics = _node_parameter(module, {
        **base,
        "use_lio_odometry": "True",
        "use_pcd_localization": "False",
        "odometry_frontend": "zlio",
    }, "zlio", "imu_topic")
    # 用 substring 而不是全等：launch_ros 把参数字符串包成 substitution，不同版本的
    # repr/render 行为会有差异（本机实测：真实节点的参数是干净的 livox/imu_saturated，
    # 已用 `ros2 param get /ns/zlio imu_topic` 复核过）。这里要钉住的是
    # "zlio 订的是守卫的输出，不是原始 livox/imu" 这件事。
    _assert(
        imu_topics is not None and "imu_saturated" in imu_topics,
        f"zlio 的 imu_topic 应指向饱和保护的输出, 实际 {imu_topics!r}",
    )
    # map->odom 静态 TF 不受前端切换影响（--lio 的语义就是 odom 原点 = 出生点）
    _assert(
        "node:static_transform_publisher_map2odom" in active,
        f"--lio 时静态 map->odom 被误关: {sorted(active)}",
    )

    # 默认（point_lio）：行为与迁移前完全一致
    active = _active_labels(module, {
        **base,
        "use_lio_odometry": "True",
        "use_pcd_localization": "False",
        "odometry_frontend": "point_lio",
    })
    _assert("node:point_lio" in active, f"默认前端未启动 point_lio: {sorted(active)}")
    _assert("node:zlio" not in active, f"默认前端不该启动 zlio: {sorted(active)}")
    _assert("node:zlio_bridge" not in active, f"默认前端不该启动 zlio_bridge: {sorted(active)}")
    _assert(
        "node:imu_saturation_guard" not in active,
        f"point_lio 前端不该启动 IMU 饱和保护（它自己有 check_satu）: {sorted(active)}",
    )

    # --gicp（use_pcd_localization）：仍然是 point_lio + 没有静态 map->odom
    active = _active_labels(module, {
        **base,
        "use_lio_odometry": "False",
        "use_pcd_localization": "True",
        "prior_pcd_file": "/tmp/prior.pcd",
        "odometry_frontend": "point_lio",
    })
    _assert("node:point_lio" in active, f"--gicp 时 point_lio 未启动: {sorted(active)}")
    _assert(
        "node:static_transform_publisher_map2odom" not in active,
        f"--gicp 时不该再有静态 map->odom: {sorted(active)}",
    )


def test_navigation_launch_loam_interface_paths():
    """navigation_launch：loam_interface 的两条启动路径都要跟着前端走。

    `Node(...)`（非组合，第 167 行附近）与 `LoadComposableNodes(...)`（组合，
    第 432 行附近）是两条独立路径，漏改一条就会在 zlio 模式下继续与 zlio_bridge 抢话题。
    `use_composition` 在两个入口的默认值不同（start_sim_nav.sh 给 False，
    nav2_stack_launch.py 的声明缺省是 True），所以两种都要覆盖。
    """
    module = _load_launch_module("srm_lio_nav", "navigation_launch.py")
    params = str(PACKAGE_ROOT / "config" / "simulation" / "nav2_params_srm_fakeyaw.yaml")
    for composition in ("False", "True"):
        base = {
            "namespace": "red_standard_robot1",
            "params_file": params,
            "slam": "False",
            "use_pcd_localization": "False",
            "use_lio_odometry": "True",
            "use_composition": composition,
            "use_sim_time": "True",
        }
        active = _active_labels(module, {**base, "odometry_frontend": "point_lio"})
        _assert(
            "node:loam_interface" in active or any(
                label.startswith("composable:loam_interface") for label in active
            ),
            f"point_lio 前端下 loam_interface 应启动 (use_composition={composition}): "
            f"{sorted(active)}",
        )
        _assert(
            "node:sensor_scan_generation" in active
            or any(label.startswith("composable:sensor_scan_generation") for label in active),
            f"sensor_scan_generation 恒启动 (use_composition={composition}): {sorted(active)}",
        )

        active = _active_labels(module, {**base, "odometry_frontend": "zlio"})
        _assert(
            "node:loam_interface" not in active
            and not any(label.startswith("composable:loam_interface") for label in active),
            f"zlio 前端下 loam_interface 必须关闭 (use_composition={composition}): "
            f"{sorted(active)}",
        )
        _assert(
            "node:terrain_analysis" in active,
            f"zlio 前端下 terrain_analysis 仍要启动: {sorted(active)}",
        )


def test_nav2_stack_frontend_validation():
    """nav2_stack_launch：拼错前端名 / zlio 缺 --lio 必须直接失败，而不是静默不发 TF。"""
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument, OpaqueFunction

    module = _load_launch_module("srm_lio_stack", "nav2_stack_launch.py")
    description = module.generate_launch_description()
    functions = [e for e in description.entities if isinstance(e, OpaqueFunction)]
    _assert(functions, "nav2_stack_launch 里没找到前端校验用的 OpaqueFunction")

    declarations = [
        e for e in description.entities if isinstance(e, DeclareLaunchArgument)
    ]

    def run_with(frontend, lio, slam="False"):
        context = LaunchContext()
        # map / prior_pcd_file 是没有默认值的必填参数，launch 在 execute 时会直接报错，
        # 这里先按"用户给了"填上（本测试只关心前端校验，不真的去加载地图）。
        context.launch_configurations.update(
            {
                "map": str(PACKAGE_ROOT / "map" / "simulation" / "rmuc_2025_tunnel.yaml"),
                "prior_pcd_file": "/tmp/prior.pcd",
                "params_file": str(
                    PACKAGE_ROOT / "config" / "simulation" / "nav2_params_srm_fakeyaw.yaml"
                ),
            }
        )
        for argument in declarations:
            argument.execute(context)
        context.launch_configurations["odometry_frontend"] = frontend
        context.launch_configurations["use_lio_odometry"] = lio
        context.launch_configurations["slam"] = slam
        for function in functions:
            function.execute(context)

    def expect_failure(frontend, lio, expected_word, slam="False"):
        try:
            run_with(frontend, lio, slam)
        except RuntimeError as error:
            _assert(
                expected_word in str(error),
                f"错误信息里应提到 {expected_word!r}, 实际: {error}",
            )
            return
        raise AssertionError(
            f"odometry_frontend={frontend} use_lio_odometry={lio} slam={slam} 应该直接报错"
        )

    expect_failure("zlioo", "True", "只支持")
    expect_failure("zlio", "False", "需要 use_lio_odometry")
    expect_failure("zlio", "True", "slam", slam="True")

    # 合法组合不得抛异常
    run_with("zlio", "True")
    run_with("point_lio", "False")


def test_frontend_defaults():
    """默认值契约（2026-11-05 切换）：仿真入口默认 zlio，其余入口保持 point_lio。

    为什么只切仿真入口：`frontend` 的算法参数是**仿真专用**的（`imu_acceleration_scale: 1.0`、
    `body_frame: front_mid360`、`lidar_topic: velodyne_points`），实车不一样（方案 §10）。
    所以导航栈/实车入口的缺省仍是 point_lio，只由仿真入口显式下发 zlio。
    """
    from launch.actions import DeclareLaunchArgument
    from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
    from launch import LaunchContext

    expected = {
        "nav_srm_simulation_launch.py": "zlio",
        "localization_launch.py": "point_lio",
        "navigation_launch.py": "point_lio",
        "nav2_stack_launch.py": "point_lio",
        "nav_simulation_launch.py": "zlio",
    }
    for filename, want in expected.items():
        module = _load_launch_module("srm_defaults_" + filename.replace(".", "_"), filename)
        description = module.generate_launch_description()
        found = {}
        for entity in description.entities:
            if not isinstance(entity, DeclareLaunchArgument):
                continue
            context = LaunchContext()
            name = perform_substitutions(
                context, normalize_to_list_of_substitutions(entity.name)
            )
            if name == "odometry_frontend":
                found[name] = perform_substitutions(
                    context, normalize_to_list_of_substitutions(entity.default_value)
                )
        _assert(
            found.get("odometry_frontend") == want,
            f"{filename} 的 odometry_frontend 默认值应是 {want}, 实际 {found}",
        )

    # 建图入口必须显式钉回 point_lio，否则会撞上 nav2_stack 的 slam+zlio 校验
    slam_module = _load_launch_module("srm_defaults_sim_slam", "sim_slam_launch.py")
    from launch.actions import IncludeLaunchDescription

    includes = [
        entity
        for entity in slam_module.generate_launch_description().entities
        if isinstance(entity, IncludeLaunchDescription)
    ]
    _assert(includes, "sim_slam_launch.py 里没找到 include")
    forwarded = dict(includes[0].launch_arguments)
    _assert(
        forwarded.get("odometry_frontend") == "point_lio",
        f"sim_slam_launch 必须透传 odometry_frontend:=point_lio, 实际 {forwarded.get('odometry_frontend')}",
    )


def main():
    tests = [
        test_localization_launch_frontends,
        test_navigation_launch_loam_interface_paths,
        test_nav2_stack_frontend_validation,
        test_frontend_defaults,
    ]
    for test in tests:
        test()
        print(f"[OK] {test.__name__}")
    print(f"\n全部通过: {len(tests)} 项")
    return 0


if __name__ == "__main__":
    sys.exit(main())
