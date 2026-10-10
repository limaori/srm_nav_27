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

"""速度限幅单一来源（speed_limits.yaml + srm_speed_limits 合并）的回归测试。

这一层错了的表现是"改了限速没效果"或"启动直接起不来"，都很贵，所以把三件事测住：

1. **唯一来源真的唯一**：四份 SRM 参数文件里不能再出现被迁移的限幅键，
   否则合并会因"两处定义"报错（值不同）或留下会误导人的重复值（值相同）。
2. **变体选择正确**：`<节点>@minco` / `@omni` 只对对应变体生效，
   MINCO 文件拿到 limits.*、Omni 文件拿到 v_linear_max，且互不串味。
3. **launch 走的是同一条路**：`SpeedLimitParams`（launch 替换）产出的临时文件
   里确实含着覆盖层的值——这是启动时各节点真正读的那份。
"""

import os
import sys
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT / "scripts"))

import srm_speed_limits as speed  # noqa: E402

CONFIG_DIR = PACKAGE_ROOT / "config"
SIM_PARAMS = CONFIG_DIR / "simulation" / "nav2_params_srm.yaml"
SIM_MINCO_PARAMS = CONFIG_DIR / "simulation" / "nav2_params_srm_minco.yaml"
REAL_PARAMS = CONFIG_DIR / "real" / "nav2_params_srm.yaml"
REAL_MINCO_PARAMS = CONFIG_DIR / "real" / "nav2_params_srm_minco.yaml"
UPSTREAM_PARAMS = CONFIG_DIR / "real" / "nav2_params_upstream.yaml"
CHASSIS_PARAMS = (
    PACKAGE_ROOT.parents[1] / "srm27_chassis_control" / "config" / "srm_chassis_control.yaml"
)

PARAMS_CASES = [
    (SIM_PARAMS, speed.VARIANT_OMNI),
    (SIM_MINCO_PARAMS, speed.VARIANT_MINCO),
    (REAL_PARAMS, speed.VARIANT_OMNI),
    (REAL_MINCO_PARAMS, speed.VARIANT_MINCO),
]

#: 迁移到覆盖层后, 参数文件里不应再出现的键（相对节点段）。
MIGRATED_KEYS = [
    "controller_server.FollowPath.v_linear_max",
    "controller_server.FollowPath.v_linear_min",
    "controller_server.FollowPath.v_angular_max",
    "controller_server.FollowPath.v_angular_min",
    "controller_server.FollowPath.limits.max_linear_speed",
    "controller_server.FollowPath.limits.max_linear_accel",
    "controller_server.FollowPath.limits.max_angular_speed",
    "controller_server.FollowPath.limits.max_angular_accel",
    "velocity_smoother.max_velocity",
    "velocity_smoother.min_velocity",
    "velocity_smoother.max_accel",
    "velocity_smoother.max_decel",
    "behavior_server.max_rotational_vel",
    "behavior_server.min_rotational_vel",
    "behavior_server.rotational_acc_lim",
]

CHASSIS_MIGRATED_KEYS = [
    "srm_cmd_mux.vx_max",
    "srm_cmd_mux.vy_max",
    "srm_cmd_mux.v_max",
    "srm_cmd_mux.wz_max",
    "rotation_controller.wz_max",
    "rotation_controller.max_angular_accel",
]


def _assert(condition, message):
    if not condition:
        raise AssertionError(message)


def test_overlay_files_exist_and_parse():
    for mode in ("simulation", "real"):
        path = CONFIG_DIR / mode / speed.OVERLAY_FILENAME
        _assert(path.is_file(), f"缺少速度限幅覆盖层: {path}")
        data = speed.load_yaml(str(path))
        _assert(data, f"{path} 解析结果为空")
        controller_keys = [key for key in data if key.startswith("controller_server@")]
        _assert(
            sorted(controller_keys)
            == ["controller_server@minco", "controller_server@omni"],
            f"{path} 必须分别给出 omni/minco 两个变体的控制器段, 实际: {controller_keys}",
        )


def test_merge_is_clean_for_every_shipped_params_file():
    """四份 SRM 参数文件都必须能无冲突合并（唯一来源的核心约束）。"""
    for params, expected_variant in PARAMS_CASES:
        result = speed.merge_params_files(str(params))
        _assert(result.overlay_path is not None, f"{params.name} 没有找到覆盖层")
        _assert(
            result.variant == expected_variant,
            f"{params.name} 变体判定为 {result.variant!r}, 期望 {expected_variant!r}",
        )
        _assert(
            not result.conflicts,
            f"{params.name} 与覆盖层冲突: {result.conflicts}",
        )
        _assert(
            not result.duplicates,
            f"{params.name} 里还留着重复定义的限幅键: {result.duplicates}",
        )
        _assert(
            "controller_server" in result.sections and "velocity_smoother" in result.sections,
            f"{params.name} 合并后缺少控制器或平滑器段: {sorted(result.sections)}",
        )


def test_overlay_values_reach_the_merged_config():
    """合并后的值必须等于覆盖层里写的值（不是参数文件里的旧值）。"""
    for params, variant in PARAMS_CASES:
        result = speed.merge_params_files(str(params))
        overlay = speed.load_yaml(result.overlay_path)
        _, section = next(
            (key, value)
            for key, value in overlay.items()
            if key == f"controller_server@{variant}"
        )
        follow = section["ros__parameters"]["FollowPath"]
        if variant == speed.VARIANT_MINCO:
            expected = follow["limits"]["max_linear_speed"]
            actual = speed.lookup(
                result.merged, "controller_server.FollowPath.limits.max_linear_speed"
            )
        else:
            expected = follow["v_linear_max"]
            actual = speed.lookup(result.merged, "controller_server.FollowPath.v_linear_max")
        _assert(
            abs(float(actual) - float(expected)) < 1e-9,
            f"{params.name} 合并后控制器上限 {actual} != 覆盖层 {expected}",
        )

        smoother = overlay[f"velocity_smoother@{variant}"]["ros__parameters"]["max_velocity"]
        merged_smoother = speed.lookup(result.merged, "velocity_smoother.max_velocity")
        _assert(
            [float(v) for v in merged_smoother] == [float(v) for v in smoother],
            f"{params.name} 合并后平滑器 {merged_smoother} != 覆盖层 {smoother}",
        )


def test_variant_sections_do_not_leak_between_controllers():
    """MINCO 文件不该被塞进 v_linear_max，Omni 文件不该被塞进 limits.*。"""
    minco = speed.merge_params_files(str(SIM_MINCO_PARAMS))
    _assert(
        speed.lookup(minco.merged, "controller_server.FollowPath.v_linear_max") is None,
        "MINCO 参数文件被写入了 Omni 的 v_linear_max（变体串味）",
    )
    _assert(
        "controller_server@omni" in minco.skipped_sections,
        f"MINCO 合并不该应用 @omni 段, skipped={minco.skipped_sections}",
    )

    omni = speed.merge_params_files(str(SIM_PARAMS))
    _assert(
        speed.lookup(omni.merged, "controller_server.FollowPath.limits.max_linear_speed") is None,
        "Omni 参数文件被写入了 MINCO 的 limits.max_linear_speed（变体串味）",
    )
    _assert(
        "controller_server@minco" in omni.skipped_sections,
        f"Omni 合并不该应用 @minco 段, skipped={omni.skipped_sections}",
    )


def test_migrated_keys_are_gone_from_params_files():
    """参数文件里不能再有这些键：既保证"只此一处"，也防止误导后来的人。"""
    for params, _ in PARAMS_CASES:
        cfg = speed.load_yaml(str(params))
        leftovers = [key for key in MIGRATED_KEYS if speed.lookup(cfg, key) is not None]
        _assert(not leftovers, f"{params.name} 仍写着已迁移的限幅键: {leftovers}")


def test_chassis_config_gets_mux_and_rotation_limits_from_overlay():
    overlay = CONFIG_DIR / "simulation" / speed.OVERLAY_FILENAME
    result = speed.merge_params_files(str(CHASSIS_PARAMS), str(overlay))
    base = speed.load_yaml(str(CHASSIS_PARAMS))
    _assert(result.variant == speed.VARIANT_NONE, "底盘配置不该判定出控制器变体")
    for key in CHASSIS_MIGRATED_KEYS:
        _assert(
            speed.lookup(base, key) is None,
            f"底盘配置里仍写着已迁移的 {key}（应只在 speed_limits.yaml 里）",
        )
        _assert(
            speed.lookup(result.merged, key) is not None,
            f"底盘配置合并后缺少 {key}",
        )
    _assert(
        "srm_cmd_mux" in result.sections and "rotation_controller" in result.sections,
        f"底盘合并缺少 mux/自转段: {sorted(result.sections)}",
    )


def test_conflicting_duplicate_is_an_error():
    """两处都写且值不同 -> 报错（这正是"改了没效果"的根源）。"""
    base = {"velocity_smoother": {"ros__parameters": {"max_velocity": [1.0, 1.0, 0.0]}}}
    overlay = {"velocity_smoother": {"ros__parameters": {"max_velocity": [2.0, 2.0, 0.0]}}}
    try:
        speed.merge_params(base, overlay)
    except speed.SpeedLimitError:
        pass
    else:
        raise AssertionError("值冲突没有被拦下来")


def test_same_value_duplicate_is_reported_not_fatal():
    base = {"velocity_smoother": {"ros__parameters": {"max_velocity": [2.0, 2.0, 0.0]}}}
    overlay = {"velocity_smoother": {"ros__parameters": {"max_velocity": [2.0, 2.0, 0.0]}}}
    result = speed.merge_params(base, overlay)
    _assert(
        result.duplicates == ["velocity_smoother.max_velocity"],
        f"同值重复没有被记为 duplicate: {result.duplicates}",
    )
    _assert(not result.conflicts, "同值重复不该算冲突")


def test_section_without_ros_parameters_is_rejected():
    """覆盖层少了 ros__parameters 会静默失效，必须直接拦下。"""
    base = {"velocity_smoother": {"ros__parameters": {"feedback": "OPEN_LOOP"}}}
    overlay = {"velocity_smoother": {"max_velocity": [2.0, 2.0, 0.0]}}
    try:
        speed.merge_params(base, overlay)
    except speed.SpeedLimitError:
        pass
    else:
        raise AssertionError("缺少 ros__parameters 的覆盖层没有被拦下来")


def test_auto_discovery_only_applies_to_srm_params():
    for params, _ in PARAMS_CASES:
        _assert(
            speed.resolve_overlay_path(str(params)) is not None,
            f"{params.name} 应自动找到同目录的覆盖层",
        )
    _assert(
        speed.resolve_overlay_path(str(UPSTREAM_PARAMS)) is None,
        "上游参考文件不该被自动套上 SRM 速度限幅",
    )
    for disabled in ("", "none", "off"):
        _assert(
            speed.resolve_overlay_path(str(SIM_PARAMS), disabled) is None,
            f"speed_limits_file={disabled!r} 时应关闭合并",
        )
    try:
        speed.resolve_overlay_path(str(SIM_PARAMS), "/nonexistent/speed_limits.yaml")
    except speed.SpeedLimitError:
        pass
    else:
        raise AssertionError("显式指定不存在的覆盖层没有被报错")


def test_chain_summary_reports_the_binding_layer():
    """自检脚本靠它回答"到底哪一层卡住了速度"。"""
    result = speed.merge_params_files(str(SIM_MINCO_PARAMS))
    summary = speed.chain_summary(result.merged)
    layers = [
        summary["controller"],
        summary["smoother"],
        summary["mux_linear"],
    ]
    _assert(
        abs(summary["binding"] - min(layers)) < 1e-9,
        f"binding={summary['binding']} 不是各层最小值 {layers}",
    )
    text = speed.format_chain(result.merged, indent="")
    _assert("控制器" in text and "速度平滑器" in text and "上限" in text, text)


def test_launch_substitution_writes_merged_file():
    """launch 用的替换必须产出"含覆盖层值"的临时文件（各节点真正读的那份）。"""
    try:
        from launch import LaunchContext

        SpeedLimitParams = speed.SpeedLimitParams
    except ImportError:  # pragma: no cover - 纯 CLI 环境
        return

    context = LaunchContext()
    params = SpeedLimitParams(str(SIM_MINCO_PARAMS), speed.AUTO, quiet=True)
    merged_path = params.perform(context)
    _assert(merged_path != str(SIM_MINCO_PARAMS), "覆盖层没有被合并（返回了原文件路径）")
    _assert(os.path.isfile(merged_path), f"合并文件不存在: {merged_path}")
    merged = speed.load_yaml(merged_path)
    overlay = speed.load_yaml(str(CONFIG_DIR / "simulation" / speed.OVERLAY_FILENAME))
    expected = overlay["controller_server@minco"]["ros__parameters"]["FollowPath"]["limits"][
        "max_linear_speed"
    ]
    actual = speed.lookup(merged, "controller_server.FollowPath.limits.max_linear_speed")
    _assert(
        abs(float(actual) - float(expected)) < 1e-9,
        f"launch 合并文件里的控制器上限 {actual} != 覆盖层 {expected}",
    )
    # 同一份替换重复求值必须复用同一个文件（各节点读到一致的参数）。
    _assert(params.perform(context) == merged_path, "重复求值产生了不同的临时文件")

    disabled = SpeedLimitParams(str(SIM_MINCO_PARAMS), "", quiet=True)
    _assert(
        disabled.perform(context) == str(SIM_MINCO_PARAMS),
        "speed_limits_file 为空时应原样返回参数文件",
    )


def _run_all():
    """直接以脚本方式运行全部用例, 避免依赖 pytest 插件环境。"""
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
