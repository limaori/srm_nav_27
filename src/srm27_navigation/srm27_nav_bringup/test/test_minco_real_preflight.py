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

"""srm_minco_real_preflight 的回归测试。

这个脚本本身是"上电前的最后一道闸门", 所以它自己的判定必须被测到 ——
否则一个误判就会让实车带着已知问题启动, 或者反过来把正确的配置拦住。

覆盖两类:
1. **合成配置**: 逐条制造"应该被拦"的情形 (use_sim_time 为真、state_timeout 不大于
   里程计周期、制动走廊不可能满足、坐标系不一致、到点几何矛盾、限速分层被钳),
   断言对应等级确实出现。用的都是能从代码读出来的因果关系, 不依赖现场数据。
2. **随仓库发布的真实配置**: 实车 MINCO 配置必须**零阻塞**; 三份配置的插件识别必须正确;
   键名查询要能穿透 `ros__parameters` 与 `/节点名` 这类容器键。
"""

import sys
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT / "scripts"))

import srm_minco_real_preflight as preflight  # noqa: E402 - 依赖上面的 sys.path

REAL_MINCO = PACKAGE_ROOT / "config" / "real" / "nav2_params_srm_minco.yaml"
SIM_MINCO = PACKAGE_ROOT / "config" / "simulation" / "nav2_params_srm_minco.yaml"
REAL_OMNI = PACKAGE_ROOT / "config" / "real" / "nav2_params_srm.yaml"


def _follow_path(**overrides):
    """构造一份最小可用的 FollowPath 参数块, 逐项可覆盖。

    基线是一份"应当零阻塞"的配置, 每个用例只改它关心的那一项,
    这样失败原因就唯一。
    """
    follow = {
        "plugin": preflight.MINCO_PLUGIN,
        "planning_frame": "odom",
        "base_frame": "base_link",
        "odom_topic": "odometry",
        "planning_horizon": 2.0,
        "state_timeout": 0.25,
        "map_timeout": 0.30,
        "trajectory_max_age": 0.30,
        "build_grace_period": 1.0,
        "terminal_reached_radius": 0.20,
        "goal_change_tolerance": 0.10,
        "terminal": {"enabled": True, "tolerance": 0.0, "exit_margin": 0.15},
        "state": {"max_sample_gap": 0.30},
        "minco": {"terminal_speed": 0.0, "optimization_clearance_margin": 0.05},
        "limits": {
            "max_linear_speed": 1.5,
            "max_linear_accel": 3.0,
            "max_angular_speed": 1.0,
            "max_angular_accel": 0.5,
        },
        "mpc": {"prediction_dt": 0.02, "prediction_steps": 30, "command_lookahead": 0.2},
        "yaw_policy": {"mode": "xy_only"},
        "safety": {
            "robot_radius": 0.33,
            "clearance_margin": 0.05,
            "unknown_is_obstacle": True,
            "braking_deceleration": 3.0,
            "reaction_latency": 0.10,
        },
    }
    follow.update(overrides)
    return {
        "livox_ros_driver2": {"ros__parameters": {"publish_freq": 10.0}},
        "point_lio": {
            "ros__parameters": {"mapping": {"lidar_time_inte": 0.1}},
        },
        "controller_server": {
            "ros__parameters": {
                "use_sim_time": False,
                "controller_frequency": 50.0,
                "general_goal_checker": {
                    "plugin": "srm27_nav_plugins::OmniStoppedGoalChecker",
                    "xy_goal_tolerance": 0.50,
                    "trans_stopped_velocity": 0.03,
                },
                "FollowPath": follow,
            }
        },
        "local_costmap": {
            "ros__parameters": {
                "local_costmap": {
                    "ros__parameters": {
                        "use_sim_time": False,
                        "global_frame": "odom",
                        "robot_radius": 0.33,
                        "inflation_layer": {"inflation_radius": 0.5},
                    }
                }
            }
        },
        "velocity_smoother": {
            "ros__parameters": {
                "use_sim_time": False,
                "max_velocity": [1.5, 1.5, 0.0],
                "max_accel": [3.0, 3.0, 0.0],
            }
        },
    }


def _run_checks(config, protocol=None, sim=None, expect_minco=True):
    """执行与 main() 相同的一组检查, 返回 Report。"""
    report = preflight.Report()
    preflight.check_plugin(config, expect_minco, report)
    sim_like = preflight.check_sim_time(config, report)
    preflight.check_frames(config, report)
    period = preflight.expected_odom_period(config, report)
    state_timeout = preflight.check_state_timeout(config, period, report, sim_like=sim_like)
    reaction_latency = preflight.resolve_reaction_latency(config, state_timeout, report)
    preflight.check_braking_corridor(
        config, state_timeout, report, reaction_latency=reaction_latency)
    preflight.check_terminal_stop(config, report)
    preflight.check_goal_change_tolerance(config, report)
    preflight.check_goal_checker(config, report)
    preflight.check_velocity_layers(config, protocol, report)
    preflight.check_goal_geometry(config, report)
    preflight.check_timing(config, report)
    preflight.check_safety_geometry(config, report)
    preflight.check_yaw_ownership(config, report)
    preflight.check_real_sim_divergence(config, sim, report)
    return report


def _titles(report, level):
    return [title for item_level, title, _ in report.items if item_level == level]


def _assert(condition, message):
    if not condition:
        raise AssertionError(message)


def test_baseline_fixture_has_no_blockers():
    """基线夹具本身必须零阻塞, 否则后面每个用例的"只多了一条"就不成立。"""
    report = _run_checks(_follow_path())
    _assert(
        report.count(preflight.BLOCK) == 0,
        f"基线夹具出现阻塞: {_titles(report, preflight.BLOCK)}",
    )


def test_flags_use_sim_time_true():
    """实车参数文件里出现 use_sim_time: True 必须阻塞。

    这是最隐蔽的失败: now() 停在约 0 -> 过期检查全部失效 ->
    控制器永远返回零速、build_grace_period 永不超时、BT 也不报错。
    """
    config = _follow_path()
    config["controller_server"]["ros__parameters"]["use_sim_time"] = True
    report = _run_checks(config)
    _assert(
        any("use_sim_time" in title for title in _titles(report, preflight.BLOCK)),
        "use_sim_time: True 没有被判为阻塞",
    )


def test_flags_state_timeout_not_greater_than_odometry_period():
    """state_timeout 必须严格大于实车里程计到达周期 (10 Hz -> 0.1 s)。"""
    report = _run_checks(_follow_path(state_timeout=0.10))
    _assert(
        any("state_timeout" in title for title in _titles(report, preflight.BLOCK)),
        "state_timeout 等于里程计周期时没有被判为阻塞",
    )
    # 0.25 s = 2.5 倍周期, 既不是阻塞也不该是"余量偏小"警告。
    report_ok = _run_checks(_follow_path(state_timeout=0.25))
    _assert(
        not any("state_timeout" in title for title in _titles(report_ok, preflight.WARN)),
        "0.25 s (2.5 倍周期) 不应被判为余量偏小",
    )
    # 1.5 倍周期应给出"余量偏小"警告, 但不阻塞。
    report_tight = _run_checks(_follow_path(state_timeout=0.15))
    _assert(
        any("state_timeout" in title for title in _titles(report_tight, preflight.WARN)),
        "0.15 s (1.5 倍周期) 应给出余量不足的警告",
    )


def test_flags_impossible_braking_corridor():
    """a_brake 取小了会让每一条候选轨迹都被拒 —— 必须阻塞。

    数值取自迁移记录 §6.5 的现场: a_brake=0.3 时要求前缀 5.1 s,
    而 2 m 局部视野的轨迹永远到不了。
    """
    report = _run_checks(_follow_path(safety={
        "robot_radius": 0.33,
        "clearance_margin": 0.05,
        "unknown_is_obstacle": True,
        "braking_deceleration": 0.3,
    }))
    titles = _titles(report, preflight.BLOCK)
    _assert(any("制动" in title for title in titles), f"制动走廊不可能满足时未阻塞: {titles}")


def test_flags_frame_mismatch():
    """规划系与局部代价地图系不一致必须阻塞 (ESDF 与状态会索引到不同坐标系)。"""
    config = _follow_path()
    config["local_costmap"]["ros__parameters"]["local_costmap"]["ros__parameters"]["global_frame"] = "map"
    report = _run_checks(config)
    _assert(
        any("坐标系" in title for title in _titles(report, preflight.BLOCK)),
        "坐标系不一致没有被判为阻塞",
    )


def test_flags_goal_radius_larger_than_tolerance():
    """terminal_reached_radius > xy_goal_tolerance 必须阻塞 (会"到点但判失败")。

    用仿真配置里实际存在的组合作夹具: 容差 0.15 / 半径 0.20。
    """
    config = _follow_path()
    config["controller_server"]["ros__parameters"]["general_goal_checker"]["xy_goal_tolerance"] = 0.15
    report = _run_checks(config)
    _assert(
        any("terminal_reached_radius" in title for title in _titles(report, preflight.BLOCK)),
        "到点几何矛盾没有被判为阻塞",
    )
    # 半径反过来小于容差 (实车当前的 0.20 / 0.40) 必须通过。
    report_ok = _run_checks(_follow_path())
    _assert(
        not any("terminal_reached_radius" in title for title in _titles(report_ok, preflight.BLOCK)),
        "0.20 <= 0.40 的正确组合被误判为阻塞",
    )


def test_detects_fake_vel_transform_free_chain_assumption():
    """Yaw 策略放开但 smoother 把 yaw 钳成 0 时必须阻塞 (角速度会被静默清零)。"""
    config = _follow_path(yaw_policy={"mode": "follow_tangent"})
    report = _run_checks(config)
    _assert(
        any("yaw" in title for title in _titles(report, preflight.BLOCK)),
        "yaw 权限与 smoother 限幅冲突没有被判为阻塞",
    )


def test_warns_when_smoother_clips_controller_speed():
    """上游要 1.5、smoother 只给 0.5 时必须警告 (现场"改了参数却没生效"的来源)。"""
    config = _follow_path()
    config["velocity_smoother"]["ros__parameters"]["max_velocity"] = [0.5, 0.5, 0.0]
    report = _run_checks(config)
    _assert(
        any("velocity_smoother" in title for title in _titles(report, preflight.WARN)),
        "smoother 钳速没有被警告",
    )


def test_expect_minco_rejects_omni_config():
    """--expect-minco 必须拦住"改了控制器却没换参数文件"。"""
    config = _follow_path(plugin="srm27_omni_pid_controller::OmniPidPursuitController")
    report = _run_checks(config, expect_minco=True)
    _assert(report.count(preflight.BLOCK) == 1, "Omni 插件未被 --expect-minco 拦下")
    # 不带该开关时不应阻塞 (纯粹是 A/B 对照场景)。
    report_plain = _run_checks(config, expect_minco=False)
    _assert(report_plain.count(preflight.BLOCK) == 0, "不带 --expect-minco 时不应因插件名阻塞")


def test_lookup_descends_transparent_container_keys():
    """键名查询要能穿透 ros__parameters 与 '/节点名' 容器键。

    srm27_nav_protocol 的配置顶层键是 '/srm27_nav_protocol', 里面再套一层
    ros__parameters —— 少穿透一层就会静默取到 None, 让"串口限速对照"变成空检查。
    """
    protocol = {"/srm27_nav_protocol": {"ros__parameters": {"max_vx": 2.5, "max_wz": 1.0}}}
    _assert(preflight._first_scalar(protocol, "max_vx") == 2.5, "未穿透容器键取到 max_vx")
    config = _follow_path()
    _assert(
        preflight._first_scalar(config, "livox_ros_driver2.publish_freq") == 10.0,
        "未穿透 ros__parameters 取到 livox_ros_driver2.publish_freq",
    )
    _assert(
        preflight._first_scalar(config, "point_lio.mapping.lidar_time_inte") == 0.1,
        "未穿透两级容器取到 point_lio.mapping.lidar_time_inte",
    )


def test_flags_disabled_terminal_stop():
    """关掉终点急停必须阻塞 —— 那就是实车终点振荡的原始缺口。"""
    config = _follow_path()
    config["controller_server"]["ros__parameters"]["FollowPath"]["terminal"][
        "enabled"] = False
    report = _run_checks(config)
    _assert(
        any("terminal.enabled" in title for title in _titles(report, preflight.BLOCK)),
        "terminal.enabled = false 没有被判为阻塞",
    )


def test_terminal_stop_reports_required_braking():
    """必须报出"底盘至少需要多少减速"这个可执行数字。

    车在距末点 tol 处被给零, 滑行 v²/(2·a_chassis) 后停下; 容差对末点对称,
    所以"仍停在 ±tol 内"要求 a_chassis >= v_entry²/(4·tol)。
    实车 (v_max 1.5 / tol 0.50) 应得到 1.12 m/s²。
    """
    report = _run_checks(_follow_path())
    text = "\n".join(d for _, _, d in report.items)
    _assert("1.12 m/s^2" in text, f"没有报出所需的最低减速能力: {text[:400]}")
    _assert("1.500 m/s" in text, f"没有报出进入急停的速度: {text[:400]}")


def test_warns_when_braking_capability_is_unidentified():
    """braking_deceleration 未填时必须警告: 急停的停车几何全靠底盘自己。

    终点急停不再限制减速度, 滑多远完全由底盘决定; 而当前的能力估计是占位值(加速度上限),
    所以必须提醒按实测辨识, 并给出那条边界。
    """
    config_unset = _follow_path()
    config_unset["controller_server"]["ros__parameters"]["FollowPath"]["safety"][
        "braking_deceleration"] = 0.0
    report = _run_checks(config_unset)
    titles = _titles(report, preflight.WARN)
    _assert(any("未辨识的制动能力" in t for t in titles), f"未警告: {titles}")

    # 声明了具体值之后, 退化为提示, 并把所需能力与声明值一起列出。
    config = _follow_path()
    config["controller_server"]["ros__parameters"]["FollowPath"]["safety"][
        "braking_deceleration"] = 2.0
    report_ok = _run_checks(config)
    titles_ok = _titles(report_ok, preflight.WARN)
    _assert(not any("未辨识的制动能力" in t for t in titles_ok), f"声明后仍在警告: {titles_ok}")
    detail = next(d for level, t, d in report_ok.items if "停车几何自洽" in t)
    _assert("2.00 m/s^2" in detail, f"应列出声明的能力值: {detail}")


def test_warns_when_translation_stop_check_is_absent():
    """去掉平动停稳检查要警告: 否则车可能高速掠过目标点即判成功。"""
    config = _follow_path()
    config["controller_server"]["ros__parameters"]["general_goal_checker"][
        "trans_stopped_velocity"] = 0.0
    report = _run_checks(config)
    _assert(
        any("trans_stopped_velocity" in title for title in _titles(report, preflight.WARN)),
        "trans_stopped_velocity = 0 没有被警告",
    )


def test_shipped_real_minco_config_has_no_blockers():
    """随仓库发布的实车 MINCO 配置必须零阻塞。

    这条用例是"配置被改坏"的守门员: 任何人把 state_timeout 调回仿真值、
    或把 limits 提到 smoother 之上, 都会在这里失败。
    """
    _assert(REAL_MINCO.is_file(), f"找不到实车 MINCO 配置: {REAL_MINCO}")
    exit_code = preflight.main([
        "--params", str(REAL_MINCO),
        "--expect-minco",
        "--sim-params", str(SIM_MINCO),
    ])
    _assert(exit_code == 0, "实车 MINCO 配置出现阻塞问题 (见上面的预检输出)")


def test_shipped_configs_are_classified_correctly():
    """三份配置的插件识别: 两份 MINCO 都是 MINCO, Omni 那份不是。"""
    for path in (REAL_MINCO, SIM_MINCO):
        config = preflight.load_yaml(str(path))
        report = preflight.Report()
        plugin = preflight.check_plugin(config, True, report)
        _assert(plugin == preflight.MINCO_PLUGIN, f"{path.name} 未被识别为 MINCO: {plugin}")
    omni = preflight.load_yaml(str(REAL_OMNI))
    report = preflight.Report()
    plugin = preflight.check_plugin(omni, False, report)
    _assert(plugin == preflight.OMNI_PLUGIN, f"{REAL_OMNI.name} 未被识别为 Omni: {plugin}")


def test_sim_config_is_treated_as_simulation():
    """仿真配置 (use_sim_time: True) 下不应再按实车里程计频率去判 state_timeout。

    仿真的 /odometry 是真值话题 (50 Hz), 与雷达帧率无关;
    否则仿真配置会被自己的启发式误判。
    """
    config = preflight.load_yaml(str(SIM_MINCO))
    report = preflight.Report()
    sim_like = preflight.check_sim_time(config, report)
    _assert(sim_like, "仿真配置未被识别为 use_sim_time 为真")
    period = preflight.expected_odom_period(config, report)
    preflight.check_state_timeout(config, period, report, sim_like=True)
    _assert(
        not any("state_timeout" in title for title in _titles(report, preflight.BLOCK)),
        "仿真配置的 state_timeout 被误判为阻塞",
    )


def test_reports_the_documented_real_vs_sim_divergence():
    """实车与仿真的差异项必须被列出来, 否则"仿真跑通"会被误当成"实车也跑通"。"""
    config = preflight.load_yaml(str(REAL_MINCO))
    report = preflight.Report()
    preflight.check_real_sim_divergence(config, SIM_MINCO, report)
    warnings = [title for title in _titles(report, preflight.WARN) if "差异" in title]
    _assert(warnings, "实车/仿真参数差异没有被报告")
    detail = next(detail for level, title, detail in report.items if "差异" in title)
    for key in ("state_timeout", "limits.max_linear_speed", "minco.terminal_speed"):
        _assert(key in detail, f"差异报告里缺少 {key}")


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
