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

"""MINCO + MPC 实车参数预检。

为什么需要这个脚本
------------------
仿真里跑通的 MINCO 参数抄到实车会踩到几类**只能靠读配置发现**的坑, 它们的共同点是
"启动不报错、车不动或走错, 现场很难归因":

* 参数文件里 `FollowPath.plugin` 其实还是 Omni —— 改了控制器却没换文件;
* `use_sim_time: True` —— 换真机后 `now()` 恒为约 0, 状态/地图/轨迹的过期检查全部失效,
  控制器会**永远返回零速且不抛异常**, BT 认为一切正常, 车就是不动;
* `state_timeout` 按仿真的 50 Hz 里程计标定 (0.10 s), 而实车 `/odometry` 由
  `sensor_scan_generation` 按雷达帧输出, 只有 10 Hz —— 每个控制周期都有概率被判"状态过期";
* 制动走廊: 验证器要求有效前缀覆盖 `reaction_latency + v/a_brake`, 而
  `reaction_latency` 就是 `state_timeout` (代码里两者共用同一个参数)。a_brake 取小了
  会让**每一条候选轨迹都被拒绝** (迁移记录 §6.5 现场就是这个现象);
* 限速分层不一致: `FollowPath.limits` > `velocity_smoother.max_velocity` > 串口 `max_vx`,
  任何一层留低, 最终执行速度都会被它静默钳住。

本脚本只读 YAML、不依赖 ROS, 因此可以在**上电之前**跑, 也可以进 CI。
退出码 0 = 没有阻塞问题; 1 = 有阻塞问题; 2 = 用法/读取错误。

用法
----
    python3 srm_minco_real_preflight.py \\
        --params   src/srm27_navigation/srm27_nav_bringup/config/real/nav2_params_srm_minco.yaml \\
        --protocol src/srm27_nav_protocol/config/srm27_nav_protocol.yaml

`--protocol` 不给时, 会尝试在工作空间里按默认路径找 srm27_nav_protocol 的配置;
找不到就跳过与串口限速的对照 (只提示, 不算问题)。
"""

import argparse
import os
import sys

try:
    import yaml
except ImportError:  # pragma: no cover - 正常 ROS 环境一定有 PyYAML
    print("错误: 需要 PyYAML (python3-yaml)。", file=sys.stderr)
    sys.exit(2)

MINCO_PLUGIN = "srm27_minco_controller::MincoMpcController"
OMNI_PLUGIN = "srm27_omni_pid_controller::OmniPidPursuitController"

# 判定等级: 阻塞 (车不能安全/正确运动) / 警告 (会与预期不符, 需人工确认) / 提示 (待辨识项)
BLOCK, WARN, NOTE = "阻塞", "警告", "提示"


class Report:
    """收集并分级打印检查结果。"""

    def __init__(self):
        self.items = []

    def add(self, level, title, detail):
        self.items.append((level, title, detail))

    def count(self, level):
        return sum(1 for item in self.items if item[0] == level)

    def dump(self, stream):
        order = {BLOCK: 0, WARN: 1, NOTE: 2}
        print("=" * 72, file=stream)
        print("MINCO + MPC 实车参数预检", file=stream)
        print("=" * 72, file=stream)
        for level, title, detail in sorted(self.items, key=lambda i: order[i[0]]):
            mark = {BLOCK: "[阻塞]", WARN: "[警告]", NOTE: "[提示]"}[level]
            print("%s %s" % (mark, title), file=stream)
            for line in detail.splitlines():
                if line.strip():
                    print("        %s" % line, file=stream)
        print("-" * 72, file=stream)
        print(
            "结论: %d 个阻塞问题, %d 个警告, %d 条提示。"
            % (self.count(BLOCK), self.count(WARN), self.count(NOTE)),
            file=stream,
        )
        if self.count(BLOCK) == 0:
            print("没有阻塞问题 —— 仍需按验收清单逐级上电, 预检通过不等于实车通过。", file=stream)


def _is_transparent(key):
    """判断某个键是否只是"容器", 可以带着剩余路径穿过去。

    Nav2 的 YAML 里每个节点都套一层 `ros__parameters`, 而 srm27_nav_protocol 的顶层键
    是带斜杠的节点名 (`/srm27_nav_protocol`)。这两类键不参与语义, 查询时直接下沉,
    这样才能用统一的 `a.b.c` 写法取到值。
    """
    return key == "ros__parameters" or key.startswith("/")


def _lookup(node, parts):
    """在 node 里按 parts 逐级查找; 只在透明容器键上下沉, 不会跨兄弟节点乱匹配。"""
    if not parts:
        return node
    if not isinstance(node, dict):
        return None
    key = parts[0]
    if key in node:
        found = _lookup(node[key], parts[1:])
        if found is not None:
            return found
    for name, value in node.items():
        if _is_transparent(str(name)) and isinstance(value, dict):
            found = _lookup(value, parts)
            if found is not None:
                return found
    return None


def _first_scalar(node, path):
    """取 'a.b.c' 的值; 找不到返回 None。"""
    return _lookup(node, path.split("."))


def _as_float(value, default=None):
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _find_protocol_config(params_path):
    """按工作空间布局猜 srm27_nav_protocol 的配置文件位置。"""
    here = os.path.dirname(os.path.abspath(params_path))
    # .../src/srm27_navigation/srm27_nav_bringup/config/real/<file>.yaml
    src_root = os.path.abspath(os.path.join(here, "..", "..", "..", ".."))
    guesses = [
        os.path.join(src_root, "srm27_nav_protocol", "config", "srm27_nav_protocol.yaml"),
        os.path.join(here, "..", "..", "..", "..", "install", "srm27_nav_protocol",
                     "share", "srm27_nav_protocol", "config", "srm27_nav_protocol.yaml"),
    ]
    for guess in guesses:
        if os.path.isfile(guess):
            return os.path.normpath(guess)
    return None


def load_yaml(path):
    with open(path, encoding="utf-8") as stream:
        return yaml.safe_load(stream)


# --------------------------------------------------------------------------
# 各项检查
# --------------------------------------------------------------------------

def check_plugin(cfg, expect_minco, report):
    plugin = _first_scalar(cfg, "controller_server.FollowPath.plugin")
    if plugin is None:
        report.add(BLOCK, "controller_server.FollowPath.plugin 缺失",
                   "参数文件里找不到 FollowPath 插件声明, 无法确认加载的是哪个控制器。")
        return None
    plugin = str(plugin)
    if expect_minco and plugin != MINCO_PLUGIN:
        report.add(BLOCK, "FollowPath 插件不是 MINCO",
                   "参数文件里是: %s\n"
                   "期望: %s\n"
                   "说明: 这是最常见的假迁移 —— 改了控制器却没换参数文件。"
                   % (plugin, MINCO_PLUGIN))
    elif plugin == MINCO_PLUGIN:
        report.add(NOTE, "FollowPath 插件 = MINCO", plugin)
    elif plugin == OMNI_PLUGIN:
        report.add(NOTE, "FollowPath 插件 = Omni", plugin)
    else:
        report.add(WARN, "FollowPath 插件是未知实现", plugin)
    return plugin


def find_true_sim_time(node, path=""):
    """递归找出所有 use_sim_time 为真的位置。"""
    hits = []
    if isinstance(node, dict):
        for key, value in node.items():
            child = "%s.%s" % (path, key) if path else str(key)
            if key == "use_sim_time" and value is True:
                hits.append(child)
            elif isinstance(value, (dict, list)):
                hits.extend(find_true_sim_time(value, child))
    elif isinstance(node, list):
        for index, value in enumerate(node):
            hits.extend(find_true_sim_time(value, "%s[%d]" % (path, index)))
    return hits


def check_sim_time(cfg, report):
    hits = find_true_sim_time(cfg)
    if hits:
        report.add(
            BLOCK,
            "use_sim_time: True 出现在实车参数文件里",
            "\n".join(hits) + "\n"
            "说明: 实车没有 /clock, 节点内 now() 会停在约 0; 状态/地图/轨迹的过期检查会全部失效, "
            "控制器持续返回零速且 build_grace_period 永不超时 —— 现象是\"插件正常、车不动、"
            "BT 也不报错\"。实车必须全部为 False。\n"
            "(如果这本来就是仿真配置, 那这条不是问题 —— 但预检的里程计频率假设不适用于仿真, "
            "相关结论会同步降级为提示。)",
        )
    else:
        report.add(NOTE, "use_sim_time 全为 False", "与实车系统时钟一致。")
    return bool(hits)


def check_frames(cfg, report):
    planning_frame = _first_scalar(cfg, "controller_server.FollowPath.planning_frame")
    costmap_frame = _first_scalar(cfg, "local_costmap.local_costmap.global_frame")
    if planning_frame is None or costmap_frame is None:
        report.add(WARN, "无法核对坐标系", "缺少 FollowPath.planning_frame 或 local_costmap.global_frame。")
        return
    if str(planning_frame) != str(costmap_frame):
        report.add(
            BLOCK,
            "planning_frame 与局部代价地图坐标系不一致",
            "FollowPath.planning_frame = %s\nlocal_costmap.global_frame = %s\n"
            "说明: 插件的 GridSnapshot 不带坐标系, ESDF 会按一个系的索引去查另一个系的位姿, "
            "不会报任何错。两者必须相同。" % (planning_frame, costmap_frame),
        )
    else:
        report.add(NOTE, "规划系与代价地图系一致", "planning_frame = local_costmap.global_frame = %s"
                   % planning_frame)
    odom_topic = _first_scalar(cfg, "controller_server.FollowPath.odom_topic")
    if odom_topic is not None:
        report.add(NOTE, "状态输入话题", "FollowPath.odom_topic = %s "
                   "(实车该话题由 sensor_scan_generation 发布)" % odom_topic)


def expected_odom_period(cfg, report):
    """估算实车 /odometry 的到达周期。

    实车链路是 Point-LIO -> loam_interface(lidar_odometry) -> sensor_scan_generation
    (与 registered_scan 做 ApproximateTime 同步后发布 /odometry), 所以 /odometry 的
    频率等于**雷达帧率**, 不是控制器频率, 也不是 IMU 频率。
    """
    freq = _as_float(_first_scalar(cfg, "livox_ros_driver2.publish_freq"))
    source = "livox_ros_driver2.publish_freq"
    if not freq or freq <= 0.0:
        inte = _as_float(_first_scalar(cfg, "point_lio.mapping.lidar_time_inte"))
        source = "point_lio.mapping.lidar_time_inte"
        if inte and inte > 0.0:
            freq = 1.0 / inte
    if not freq or freq <= 0.0:
        report.add(WARN, "无法估算里程计频率",
                   "参数文件里没有可用的 livox_ros_driver2.publish_freq 或 "
                   "point_lio.mapping.lidar_time_inte, 跳过 state_timeout 相关检查。")
        return None
    report.add(NOTE, "里程计频率估算",
               "/odometry 频率 ≈ %.1f Hz (周期 %.3f s, 来源 %s)" % (freq, 1.0 / freq, source))
    return 1.0 / freq


def check_state_timeout(cfg, period, report, sim_like=False):
    state_timeout = _as_float(_first_scalar(cfg, "controller_server.FollowPath.state_timeout"))
    if state_timeout is None:
        report.add(WARN, "缺少 state_timeout", "无法核对状态新鲜度门限。")
        return None
    if sim_like:
        # 仿真的 /odometry 是真值话题 (50 Hz), 与雷达帧率无关, 上面的周期估算不适用。
        report.add(NOTE, "跳过 state_timeout 与里程计周期的对照 (仿真配置)",
                   "state_timeout = %.3f s; 仿真的 /odometry 由真值适配器以固定 50 Hz 发布, "
                   "这里的周期估算 (按雷达帧率) 不适用。" % state_timeout)
    elif period is not None:
        if state_timeout <= period:
            report.add(
                BLOCK,
                "state_timeout 不大于里程计周期",
                "state_timeout = %.3f s, 里程计周期 = %.3f s\n"
                "说明: 仿真的真值里程计是 50 Hz (周期 0.02 s), 0.10 s 的门限有 5 倍余量; "
                "实车 /odometry 跟随雷达帧率, 门限必须严格大于到达周期, 否则每个控制周期都有"
                "概率被判\"状态过期\", 表现是走走停停、超时抛异常后 BT 反复 recovery。" 
                % (state_timeout, period),
            )
        elif state_timeout < 2.0 * period:
            report.add(
                WARN,
                "state_timeout 余量偏小",
                "state_timeout = %.3f s, 里程计周期 = %.3f s (不足 2 倍)\n"
                "建议取 2~3 倍周期, 以吸收 LIO 解算延迟与传输抖动。" % (state_timeout, period),
            )
        else:
            report.add(NOTE, "state_timeout 与里程计周期匹配",
                       "state_timeout = %.3f s, 里程计周期 = %.3f s (%.1f 倍)"
                       % (state_timeout, period, state_timeout / period))
    # state_timeout 曾经同时被当作 reaction_latency 使用; 现在只有 safety.reaction_latency
    # 没填 (或填 0) 时才会退回这个耦合, 由 resolve_reaction_latency 单独核对。
    gap = _as_float(_first_scalar(cfg, "controller_server.FollowPath.state.max_sample_gap"))
    if gap is not None and period is not None and gap < 2.0 * period:
        report.add(WARN, "state.max_sample_gap 相对里程计周期偏小",
                   "max_sample_gap = %.3f s, 里程计周期 = %.3f s; 差分求速度会被频繁拒绝。"
                   % (gap, period))
    return state_timeout


def resolve_reaction_latency(cfg, state_timeout, report):
    """解析制动模型实际使用的反应延迟。

    代码里 `config.reaction_latency = (safety.reaction_latency > 0) ? 那个值 : state_timeout`。
    实车必须把两者拆开：`state_timeout` 不得小于里程计到达周期（10 Hz -> 0.25 s），
    而"从下发命令到车真正动起来"只有 0.1 s 量级。混用会让终点短停车轨迹被一律拒绝。
    """
    explicit = _as_float(
        _first_scalar(cfg, "controller_server.FollowPath.safety.reaction_latency"))
    shown_timeout = state_timeout if state_timeout is not None else float("nan")
    if explicit is None:
        report.add(WARN, "缺少 safety.reaction_latency",
                   "该参数在 2026-10-09 之后新增; 缺失时退化为使用 state_timeout。")
        return state_timeout
    if explicit > 0.0:
        report.add(NOTE, "reaction_latency 已与 state_timeout 拆开",
                   "safety.reaction_latency = %.3f s (state_timeout = %.3f s)\n"
                   "说明: 短停车轨迹能否通过取决于 轨迹总时长 >= reaction_latency + v/a_brake。"
                   % (explicit, shown_timeout))
        return explicit
    report.add(
        WARN,
        "safety.reaction_latency = 0 (退化为 state_timeout)",
        "当前 state_timeout = %.3f s, 两者会共用同一个值。\n"
        "实车 /odometry 只有约 10 Hz, state_timeout 必须 >= 0.25 s; 而反应延迟只有 0.1 s 量级。\n"
        "共用时终点短停车轨迹会被一律拒绝 —— 2026-10-09 实车日志的\n"
        "  terminal=global_goal | 有效前缀=0.2611s 需要=0.6s\n"
        "就是这么来的 (0.26 s 的终点轨迹在 reaction_latency=0.25 时只允许 v <= 0.033 m/s)。\n"
        "请按实测填写: 命令阶跃到 /odometry 速度起变化的时延中位数。"
        % shown_timeout,
    )
    return state_timeout


def check_terminal_stop(cfg, report):
    """终点急停必须启用, 且"急刹后能停在容差内"必须成立。

    2026-10-09 实车终点振荡: 目标检查器允许在 xy 容差内判成功, 而 MPC 仍继续追精确末点,
    冲过末点再反向修正。终点急停 (`terminal_stop.hpp`) 就是消掉这个矛盾:
    **一进容差就立即给零速**, 之后控制器不再输出任何朝末点或背向末点的速度,
    所以不可能再主动把车带出容差 —— 振荡在逻辑上不会出现。

    但急刹的**滑行距离**由底盘自己决定 (控制器不再限速), 所以要检查:

        v_entry² / (2·a_chassis) <= 2·tol

    容差对末点是对称的, 所以这是"停车点仍落在末点 ±tol 内"的充要条件;
    不满足就意味着车必然滑出容差 —— 那才是要拦的情况。
    """
    enabled = _first_scalar(cfg, "controller_server.FollowPath.terminal.enabled")
    tolerance = _as_float(_first_scalar(cfg, "controller_server.FollowPath.terminal.tolerance"))
    goal_tol = _as_float(
        _first_scalar(cfg, "controller_server.general_goal_checker.xy_goal_tolerance"))
    radius = _as_float(_first_scalar(cfg, "controller_server.FollowPath.terminal_reached_radius"))
    v_max = _as_float(_first_scalar(cfg, "controller_server.FollowPath.limits.max_linear_speed"))
    a_max = _as_float(_first_scalar(cfg, "controller_server.FollowPath.limits.max_linear_accel"))
    braking = _as_float(
        _first_scalar(cfg, "controller_server.FollowPath.safety.braking_deceleration"))
    shown_goal = goal_tol if goal_tol is not None else float("nan")
    shown_radius = radius if radius is not None else float("nan")

    if enabled is None:
        report.add(WARN, "缺少 terminal.enabled",
                   "该参数在 2026-10-09 之后新增; 缺失时插件用代码默认值 true。")
        return
    if enabled is not True:
        report.add(
            BLOCK,
            "terminal.enabled = false",
            "关掉终点急停就回到了 2026-10-09 实车终点振荡的那个缺口:\n"
            "  目标检查器允许在 %.2f m 内判成功, 而 MPC 仍会继续把车往精确末点修正;\n"
            "  terminal_reached_radius (%.2f m) 只在局部路径不足两点时才生效, "
            "正常接近终点时不会触发。" % (shown_goal, shown_radius),
        )
        return

    if tolerance is None or tolerance <= 0.0:
        report.add(NOTE, "终点急停进入阈值跟随目标检查器",
                   "terminal.tolerance = 0 -> 使用 xy_goal_tolerance = %.2f m;\n"
                   "读不到目标检查器容差时退回 terminal_reached_radius = %.2f m。\n"
                   "提示: 这个阈值同时决定急刹有多猛 —— 进入速度 = sqrt(2·a_plan·tol), "
                   "tol=0.40 时约满速、tol=0.15 时约 0.95 m/s。"
                   % (shown_goal, shown_radius))
        effective_tolerance = goal_tol if goal_tol is not None else radius
    else:
        effective_tolerance = tolerance
        if goal_tol is not None and tolerance > goal_tol:
            report.add(WARN, "终点急停阈值大于目标检查器容差",
                       "terminal.tolerance = %.2f m > xy_goal_tolerance = %.2f m;\n"
                       "车会在目标检查器判成功之前就急停, 可能停在容差外导致目标判失败。"
                       % (tolerance, goal_tol))
        else:
            report.add(NOTE, "终点急停阈值已显式指定",
                       "terminal.tolerance = %.2f m" % tolerance)

    if None in (effective_tolerance, v_max) or v_max <= 0.0 or effective_tolerance is None:
        report.add(WARN, "无法核对终点急停的停车几何",
                   "缺少 limits.max_linear_speed 或有效容差。")
        return
    a_plan = a_max if a_max is not None and a_max > 0.0 else None
    if a_plan is None:
        report.add(WARN, "缺少 limits.max_linear_accel", "无法估算进入急停时的速度。")
        return
    if braking is not None and braking > 0.0:
        a_plan = min(a_plan, braking)
    # 急刹的滑行完全由底盘自己决定, 能力估计就用 safety.braking_deceleration;
    # 它没填时本来就会退化为 limits.max_linear_accel。
    a_chassis = braking if (braking is not None and braking > 0.0) else a_max
    # 进入急停时的速度: 受 v_max 限制, 也受"轨迹在 tol 内能到多快"限制。
    v_entry = min(v_max, (2.0 * a_plan * effective_tolerance) ** 0.5)
    # 要让滑行后仍落在末点 ±tol 内 (容差对称), 底盘至少需要:
    #   v_entry² / (2·a) <= 2·tol   =>   a >= v_entry² / (4·tol)
    a_required = v_entry * v_entry / (4.0 * effective_tolerance)
    slide = v_entry * v_entry / (2.0 * a_chassis)
    detail = ("容差 tol                        = %.3f m\n"
              "进入急停速度 v_entry            = min(v_max %.2f, sqrt(2·a_plan·tol)) = %.3f m/s\n"
              "滑行后仍停在末点 ±tol 内, 需要    a_chassis >= v_entry²/(4·tol) = %.2f m/s^2\n"
              "当前用作能力估计的值            = %.2f m/s^2 -> 滑行 %.3f m (允许 %.3f m)"
              % (effective_tolerance, v_max, v_entry, a_required, a_chassis, slide,
                 2.0 * effective_tolerance))
    # 注意: 这里**没有**阻塞分支, 而且这是刻意的。
    # 轨迹的停车剖面用的是 a_plan = min(max_linear_accel, braking_deceleration),
    # 急刹后底盘的能力估计也取 braking_deceleration —— 两者同源, 于是
    # "滑行量 <= 2·tol" 在结构上恒成立 (滑行量 <= tol)。真正会出事的是
    # **声明的值本身不准**(它是待辨识的占位值), 那只能用实测解决, 检查器拦不出来。
    # 所以这里只报出"底盘至少需要多少减速"这个可执行的数字。
    if braking is None or braking <= 0.0:
        report.add(
            WARN,
            "终点急停的停车几何依赖未辨识的制动能力",
            detail + "\n说明: safety.braking_deceleration 未填, 上面那个能力估计用的是加速度上限 "
            "(占位值)。\n终点急停不再限制减速度, 车在距末点 tol 处被给零之后**滑多远完全由底盘决定**;\n"
            "请按 S7.2 辨识制动, 并确认实测值不低于 %.2f m/s² —— 低于它就会滑出容差、再回来。\n"
            "想减小急刹冲击可以调小 terminal.tolerance (进入越晚速度越低)。" % a_required,
        )
    else:
        report.add(NOTE, "终点急停的停车几何自洽",
                   detail + "\n余量 %.2f m/s^2 (滑行 %.3f m / 允许 %.3f m)"
                   % (a_chassis - a_required, slide, 2.0 * effective_tolerance))


def check_goal_checker(cfg, report):
    """到点判定必须由"角速度不参与"的检查器负责。

    2026-10-09 之后 SRM 改为自建插件 `srm27_nav_plugins::OmniStoppedGoalChecker`：
    与 `nav2_controller::StoppedGoalChecker` 的唯一差别是去掉了
    `|wz| <= rot_stopped_velocity` 这一条。导航不拥有自转（xy_only 下插件输出 wz≡0，
    smoother 把 yaw 上界钳成 0），拿 wz 否掉到达结论等于把下位机的状态当成导航的失败。
    """
    plugin = _first_scalar(cfg, "controller_server.general_goal_checker.plugin")
    xy = _as_float(_first_scalar(cfg, "controller_server.general_goal_checker.xy_goal_tolerance"))
    trans = _as_float(
        _first_scalar(cfg, "controller_server.general_goal_checker.trans_stopped_velocity"))
    rot = _as_float(
        _first_scalar(cfg, "controller_server.general_goal_checker.rot_stopped_velocity"))
    shown = plugin if plugin is not None else "?"

    if plugin is None:
        report.add(BLOCK, "缺少 general_goal_checker.plugin", "无法确认到点判定由谁负责。")
        return
    if "OmniStoppedGoalChecker" not in str(plugin):
        report.add(
            BLOCK,
            "到点判定仍依赖角速度 (plugin = %s)" % shown,
            "期望 srm27_nav_plugins::OmniStoppedGoalChecker。\n"
            "nav2_controller::StoppedGoalChecker 在位置/航向满足后还要求 |wz| <= "
            "rot_stopped_velocity，\n也就是**自转会让它永远判不了到点**；而 SRM 的自转不归导航管。",
        )
    else:
        report.add(NOTE, "到点判定不依赖角速度", "plugin = %s" % shown)

    if rot is not None:
        report.add(
            BLOCK,
            "参数文件里残留 rot_stopped_velocity = %s" % rot,
            "OmniStoppedGoalChecker 没有这个参数：未声明的键不会被声明、也就不会被读取，"
            "留在配置里只会让人误以为它生效。请删掉这一行。",
        )
    if trans is None:
        report.add(WARN, "缺少 trans_stopped_velocity",
                   "没有它就无法保证\"到点\"时车真的停住，可能高速掠过目标点即判成功。")
    elif trans <= 0.0:
        report.add(
            WARN,
            "trans_stopped_velocity = %s (等于不检查平动停稳)" % trans,
            "点检查只判位置，车可能在掠过目标点的瞬间被判到达。除非确定只需要位置判定。",
        )
    else:
        report.add(NOTE, "平动停稳仍在判定内",
                   "trans_stopped_velocity = %.3f m/s (位置 + 平动停稳, 角速度不参与)" % trans)
    if xy is not None and xy > 0.0:
        radius = _as_float(
            _first_scalar(cfg, "controller_server.FollowPath.terminal_reached_radius"))
        if radius is not None and radius > xy:
            report.add(WARN, "到点几何: terminal_reached_radius > xy_goal_tolerance",
                       "%.2f > %.2f (见 check_goal_geometry)" % (radius, xy))


def check_goal_change_tolerance(cfg, report):
    """会话重置阈值必须明显大于全局规划器的末点抖动。"""
    value = _as_float(
        _first_scalar(cfg, "controller_server.FollowPath.goal_change_tolerance"))
    goal_tol = _as_float(
        _first_scalar(cfg, "controller_server.general_goal_checker.xy_goal_tolerance"))
    if value is None:
        report.add(WARN, "缺少 goal_change_tolerance",
                   "该参数在 2026-10-09 之后新增; 缺失时插件用代码默认值 0.10 m。")
        return
    if value <= 0.01:
        report.add(
            BLOCK,
            "goal_change_tolerance 过小 (%.3f m)" % value,
            "2026-10-09 实车日志: 两次 \"Reached the goal\" 之前各多出一次会话重置\n"
            "  (14:01:01.600 新会话 #2 -> 14:01:01.720 到点)\n"
            "原因是原实现用 1 mm 判定末点变化, 比规划器每次刷新的末点抖动还小。\n"
            "新会话会清空轨迹/热启动/重规划状态, 让车在终点附近重新加速或反向修正。",
        )
    else:
        report.add(NOTE, "会话重置阈值合理",
                   "goal_change_tolerance = %.3f m (> 规划器末点抖动, < 航点间距)" % value)
    if goal_tol is not None and value > goal_tol:
        report.add(WARN, "goal_change_tolerance 大于目标点容差",
                   "%.3f m > xy_goal_tolerance %.3f m: 换一个很近的目标可能不被识别为新会话。"
                   % (value, goal_tol))


def check_braking_corridor(cfg, state_timeout, report, reaction_latency=None):
    """核对"有效前缀时长"与"局部轨迹可能的最短时长"。

    两个量必须分清, 它们用的是**不同的加速度**:

    * 验证器要求的有效前缀 (`TrajectoryValidator`):
          required_prefix = reaction_latency + v_max / a_brake
      其中 reaction_latency 就是 state_timeout, a_brake = safety.braking_deceleration
      (填 0 时退化为 limits.max_linear_accel)。它表达的是"要能在前面这段路上停住"。
    * 局部轨迹自身的时长: 由前端按 limits.max_linear_accel 生成起停剖面。
      planning_horizon 这么长的起停剖面的**最短**时长 (速度刚好用到 v_max) 是:
          梯形 (v_max^2 / a_accel <= horizon):  horizon / v_max + v_max / a_accel
          三角形 (v_max 到不了):                2 * sqrt(horizon / a_accel)

    若 required_prefix >= 最短轨迹时长, 说明连"最快的合法轨迹"都覆盖不了制动要求,
    于是**每一条候选轨迹都会被拒绝**。迁移记录 §6.5 的现场就是这个: a_brake 误取 0.3,
    required 变成 0.1 + 1.5/0.3 = 5.1 s, 而 2 m 的轨迹永远到不了 5.1 s。
    """
    limits_speed = _as_float(_first_scalar(cfg, "controller_server.FollowPath.limits.max_linear_speed"))
    limits_accel = _as_float(_first_scalar(cfg, "controller_server.FollowPath.limits.max_linear_accel"))
    horizon = _as_float(_first_scalar(cfg, "controller_server.FollowPath.planning_horizon"))
    braking = _as_float(_first_scalar(cfg, "controller_server.FollowPath.safety.braking_deceleration"))
    latency = reaction_latency if reaction_latency is not None else state_timeout
    if None in (limits_speed, horizon) or limits_speed <= 0.0 or latency is None:
        report.add(WARN, "无法核对制动走廊",
                   "缺少 limits.max_linear_speed / planning_horizon / reaction_latency。")
        return
    if braking is None or braking <= 0.0:
        if limits_accel is None or limits_accel <= 0.0:
            report.add(BLOCK, "没有可用的制动减速度",
                       "safety.braking_deceleration = 0 且 limits.max_linear_accel 不可用。")
            return
        braking = limits_accel
        report.add(
            WARN,
            "safety.braking_deceleration 未填 (退化为加速度上限 %.2f m/s^2)" % braking,
            "该值必须是**实车辨识**得到的、可保证的制动能力。填 0 等于假设车能按加速上限刹车; "
            "取小值又会让候选轨迹被一律拒绝 (实测 0.3 m/s^2 时要求前缀 5.1 s, 必然失败)。\n"
            "上实车前请按验收清单里的制动辨识步骤填实测值; 未辨识前不要提速。",
        )
    if limits_accel is None or limits_accel <= 0.0:
        report.add(WARN, "缺少 limits.max_linear_accel", "无法估算局部轨迹的最短时长。")
        return

    if limits_speed * limits_speed / limits_accel <= horizon:
        t_min = horizon / limits_speed + limits_speed / limits_accel
        profile = "梯形剖面 (能跑到 v_max)"
    else:
        t_min = 2.0 * (horizon / limits_accel) ** 0.5
        profile = "三角形剖面 (受 horizon 限制跑不到 v_max)"
    required_prefix = latency + limits_speed / braking
    detail = ("reaction_latency                 = %.3f s\n"
              "v_max / a_brake                    = %.2f / %.2f = %.3f s\n"
              "  => 要求有效前缀 >= %.3f s\n"
              "最短局部轨迹时长 (horizon=%.2f, a_accel=%.2f, %s)\n"
              "                                   = %.3f s"
              % (latency, limits_speed, braking, limits_speed / braking,
                 required_prefix, horizon, limits_accel, profile, t_min))
    if required_prefix >= t_min:
        report.add(
            BLOCK,
            "制动要求超过局部轨迹可能的最短时长",
            detail + "\n说明: 这样连最快的合法轨迹都覆盖不了制动要求, 于是每一条候选轨迹都会被 "
            "TrajectoryValidator 拒绝 —— 现象是规划一直失败、车不动、BT 反复 recovery。\n"
            "处理顺序: 先按实测把 a_brake 填成\"确实能保证\"的值, 再考虑加大 planning_horizon。",
        )
    elif required_prefix > 0.7 * t_min:
        report.add(WARN, "制动走廊余量不足 30%", detail + "\n说明: 能通过但很勉强, 建议按实测重填 a_brake。")
    else:
        report.add(NOTE, "制动走廊余量正常",
                   detail + "\n余量 %.3f s (%.0f%%)"
                   % (t_min - required_prefix, 100.0 * (1.0 - required_prefix / t_min)))


def check_velocity_layers(cfg, protocol_cfg, report):
    """三层限速必须自洽: FollowPath.limits <= velocity_smoother <= 串口。"""
    limits_speed = _as_float(_first_scalar(cfg, "controller_server.FollowPath.limits.max_linear_speed"))
    limits_accel = _as_float(_first_scalar(cfg, "controller_server.FollowPath.limits.max_linear_accel"))
    smoother_vel = _first_scalar(cfg, "velocity_smoother.max_velocity")
    smoother_acc = _first_scalar(cfg, "velocity_smoother.max_accel")
    yaw_mode = _first_scalar(cfg, "controller_server.FollowPath.yaw_policy.mode")

    smoother_x = None
    if isinstance(smoother_vel, (list, tuple)) and smoother_vel:
        smoother_x = _as_float(smoother_vel[0])
        smoother_wz = _as_float(smoother_vel[2]) if len(smoother_vel) > 2 else None
    else:
        smoother_wz = None

    if limits_speed is not None and smoother_x is not None and smoother_x < limits_speed:
        report.add(
            WARN,
            "velocity_smoother 会把控制器速度钳住",
            "FollowPath.limits.max_linear_speed = %.2f m/s\n"
            "velocity_smoother.max_velocity[0] = %.2f m/s\n"
            "说明: 上游想跑 %.2f、执行端只给 %.2f, 上层参数等于没生效。"
            % (limits_speed, smoother_x, limits_speed, smoother_x),
        )
    if limits_accel is not None and isinstance(smoother_acc, (list, tuple)) and smoother_acc:
        smoother_ax = _as_float(smoother_acc[0])
        if smoother_ax is not None and smoother_ax < limits_accel:
            report.add(WARN, "velocity_smoother 加速度上限低于控制器",
                       "FollowPath.limits.max_linear_accel = %.2f, velocity_smoother.max_accel[0] = %.2f"
                       % (limits_accel, smoother_ax))
    if yaw_mode == "xy_only" and smoother_wz is not None and smoother_wz > 0.0:
        report.add(NOTE, "xy_only 模式下 smoother 仍留有 yaw 上限",
                   "smoother.max_velocity[2] = %.2f, 但阶段一控制器输出 wz 恒为 0, 不构成冲突; "
                   "将来切 follow_tangent/SE(2) 时必须同步放开这一项。" % smoother_wz)
    elif yaw_mode and yaw_mode != "xy_only" and smoother_wz == 0.0:
        report.add(BLOCK, "yaw 策略已放开但 smoother 仍把 yaw 钳成 0",
                   "yaw_policy.mode = %s, 而 velocity_smoother.max_velocity[2] = 0.0; "
                   "导航指令的角速度会被静默清零。" % yaw_mode)

    if protocol_cfg is not None:
        max_vx = _as_float(_first_scalar(protocol_cfg, "max_vx"))
        max_wz = _as_float(_first_scalar(protocol_cfg, "max_wz"))
        if limits_speed is not None and max_vx is not None and max_vx < limits_speed:
            report.add(WARN, "串口层限速低于控制器",
                       "srm27_nav_protocol.max_vx = %.2f < FollowPath.limits.max_linear_speed = %.2f"
                       % (max_vx, limits_speed))
        else:
            report.add(NOTE, "串口层限速与控制器自洽",
                       "srm27_nav_protocol max_vx = %s, max_wz = %s (仅用于对照; 实车最终出口不是它)"
                       % (max_vx, max_wz))
    else:
        report.add(NOTE, "未核对串口层限速",
                   "没找到 srm27_nav_protocol 的配置, 用 --protocol 指定后可一起对照。")


def check_goal_geometry(cfg, report):
    radius = _as_float(_first_scalar(cfg, "controller_server.FollowPath.terminal_reached_radius"))
    tol = _as_float(_first_scalar(cfg, "controller_server.general_goal_checker.xy_goal_tolerance"))
    if radius is None or tol is None:
        report.add(WARN, "无法核对到点几何",
                   "缺少 terminal_reached_radius 或 general_goal_checker.xy_goal_tolerance。")
        return
    if radius > tol:
        report.add(
            BLOCK,
            "terminal_reached_radius 大于目标点容差",
            "terminal_reached_radius = %.3f m > xy_goal_tolerance = %.3f m\n"
            "说明: 车在离目标 %.3f~%.3f m 处就会被插件判成\"已到点\"并输出零速(不算失败), "
            "但 StoppedGoalChecker 不接受, 进度检查器随后判定目标失败 -> BT 进入 recovery。"
            % (radius, tol, tol, radius),
        )
    else:
        report.add(NOTE, "到点几何自洽",
                   "terminal_reached_radius = %.3f m <= xy_goal_tolerance = %.3f m" % (radius, tol))


def check_timing(cfg, report):
    freq = _as_float(_first_scalar(cfg, "controller_server.controller_frequency"))
    dt = _as_float(_first_scalar(cfg, "controller_server.FollowPath.mpc.prediction_dt"))
    steps = _as_float(_first_scalar(cfg, "controller_server.FollowPath.mpc.prediction_steps"))
    if freq and dt and abs(dt - 1.0 / freq) > 1e-6:
        report.add(WARN, "prediction_dt 与控制器频率不一致",
                   "controller_frequency = %.1f Hz (周期 %.4f s), prediction_dt = %.4f s"
                   % (freq, 1.0 / freq, dt))
    if dt and steps:
        report.add(NOTE, "MPC 预测窗口",
                   "N * dt = %.0f * %.3f = %.3f s, 该值同时是验证器的 required_prefix_duration。"
                   % (steps, dt, steps * dt))
    lookahead = _as_float(_first_scalar(cfg, "controller_server.FollowPath.mpc.command_lookahead"))
    if lookahead:
        report.add(
            WARN,
            "mpc.command_lookahead 是仿真标定值 (%.2f s)" % lookahead,
            "它应与\"从下发命令到车真正动起来\"的有效执行延迟同量级。仿真没有内环延迟, "
            "取 0.20 s; 实车有串口 100 Hz 重发、下位机解算和电机响应。\n"
            "取太小会复现速度层自锁 (起步时 QP 最优解把加速后置, 第 0 拍命令≈0, 车不动); "
            "取太大则会过冲。上实车后按验收清单里的延迟辨识步骤重标。",
        )


def check_safety_geometry(cfg, report):
    radius = _as_float(_first_scalar(cfg, "controller_server.FollowPath.safety.robot_radius"))
    margin = _as_float(_first_scalar(cfg, "controller_server.FollowPath.safety.clearance_margin"))
    opt_margin = _as_float(_first_scalar(cfg, "controller_server.FollowPath.minco.optimization_clearance_margin"))
    costmap_radius = _as_float(_first_scalar(cfg, "local_costmap.local_costmap.robot_radius"))
    if radius is not None and margin is not None:
        hard = radius + margin
        soft = hard + (opt_margin or 0.0)
        report.add(
            NOTE,
            "碰撞几何只来自 safety.*, 与代价地图无关",
            "硬阈值 robot_radius + clearance_margin = %.3f m\n"
            "优化目标再加 optimization_clearance_margin = %.3f m\n"
            "说明: 控制器只看 LETHAL_OBSTACLE(254), 膨胀层代价(1~253)全部丢弃, "
            "因此 local_costmap.inflation_radius=%s / robot_radius=%s 对避障判定没有影响。"
            % (hard, soft, _first_scalar(cfg, "local_costmap.local_costmap.inflation_layer.inflation_radius"),
               costmap_radius),
        )
        report.add(WARN, "safety.robot_radius 是模型包络而非实测外接圆",
                   "robot_radius = %.3f m。窄通道前必须实测车身外接圆(含轮子外廓)。" % radius)


def check_yaw_ownership(cfg, report):
    mode = _first_scalar(cfg, "controller_server.FollowPath.yaw_policy.mode")
    if mode == "xy_only":
        report.add(
            NOTE,
            "阶段一 yaw 所有权: 导航不产生自转",
            "yaw_policy.mode = xy_only -> MPC 的角速度上下界与参考都为 0, "
            "输出 angular.z 恒为 0。\n"
            "哨兵自转必须由独立链路提供; 实车当前没有 rotation_controller/cmd_spin 发布者, "
            "自转只能来自下位机或行为树 (srm27_behavior 的 PublishSpinSpeed)。",
        )


def check_real_sim_divergence(cfg, sim_path, report):
    """把实车配置与仿真配置逐项对照, 列出会让实车行为不同于已验证仿真的差异。"""
    if not sim_path or not os.path.isfile(sim_path):
        report.add(NOTE, "未对照仿真配置", "找不到仿真 MINCO 参数文件, 跳过差异对照。")
        return
    try:
        sim = load_yaml(sim_path)
    except Exception as exc:  # pragma: no cover
        report.add(WARN, "读取仿真配置失败", str(exc))
        return
    keys = [
        "planning_horizon",
        "minco.terminal_speed",
        "limits.max_linear_speed",
        "limits.max_linear_accel",
        "mpc.command_lookahead",
        "safety.braking_deceleration",
        "safety.reaction_latency",
        "state_timeout",
        "goal_change_tolerance",
        "terminal.hold_enabled",
        "terminal.hold_tolerance",
        "terminal.hold_speed",
    ]
    diffs = []
    for key in keys:
        real_value = _first_scalar(cfg, "controller_server.FollowPath." + key)
        sim_value = _first_scalar(sim, "controller_server.FollowPath." + key)
        if real_value != sim_value:
            diffs.append("  %-30s 实车 %-8s 仿真 %s" % (key, real_value, sim_value))
    if diffs:
        report.add(
            WARN,
            "实车与仿真 MINCO 参数存在差异",
            "仿真跑通的行为**不自动**代表实车行为。差异项:\n" + "\n".join(diffs) +
            "\n建议: 首次实车联调时把差异项一并记录, 避免把\"实车配置不同\"误判成\"MINCO 在实车上有问题\"。",
        )
    else:
        report.add(NOTE, "实车与仿真关键参数一致", "上述 %d 项关键参数完全相同。" % len(keys))


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="MINCO + MPC 实车参数预检 (只读, 不依赖 ROS)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--params", required=True, help="实车 Nav2 参数 YAML 路径")
    parser.add_argument("--protocol", default=None,
                        help="srm27_nav_protocol 的配置 YAML; 缺省按工作空间布局自动查找")
    parser.add_argument("--sim-params", default=None,
                        help="仿真 MINCO 参数 YAML, 用于差异对照; 缺省按工作空间布局自动查找")
    parser.add_argument("--expect-minco", action="store_true",
                        help="要求 FollowPath.plugin 必须是 MincoMpcController, 否则算阻塞问题")
    args = parser.parse_args(argv)

    if not os.path.isfile(args.params):
        print("错误: 参数文件不存在: %s" % args.params, file=sys.stderr)
        return 2
    try:
        cfg = load_yaml(args.params)
    except Exception as exc:
        print("错误: 解析参数文件失败: %s" % exc, file=sys.stderr)
        return 2

    protocol_path = args.protocol or _find_protocol_config(args.params)
    protocol_cfg = None
    if protocol_path and os.path.isfile(protocol_path):
        try:
            protocol_cfg = load_yaml(protocol_path)
        except Exception:
            protocol_cfg = None

    sim_path = args.sim_params
    if sim_path is None:
        here = os.path.dirname(os.path.abspath(args.params))
        candidate = os.path.normpath(os.path.join(here, "..", "simulation", "nav2_params_srm_minco.yaml"))
        sim_path = candidate if os.path.isfile(candidate) else None

    report = Report()
    print("参数文件: %s" % os.path.abspath(args.params))
    if protocol_path:
        print("串口配置: %s" % protocol_path)
    if sim_path:
        print("仿真对照: %s" % sim_path)
    print()

    check_plugin(cfg, args.expect_minco, report)
    sim_like = check_sim_time(cfg, report)
    check_frames(cfg, report)
    period = expected_odom_period(cfg, report)
    state_timeout = check_state_timeout(cfg, period, report, sim_like=sim_like)
    reaction_latency = resolve_reaction_latency(cfg, state_timeout, report)
    check_braking_corridor(cfg, state_timeout, report, reaction_latency=reaction_latency)
    check_terminal_stop(cfg, report)
    check_goal_change_tolerance(cfg, report)
    check_goal_checker(cfg, report)
    check_velocity_layers(cfg, protocol_cfg, report)
    check_goal_geometry(cfg, report)
    check_timing(cfg, report)
    check_safety_geometry(cfg, report)
    check_yaw_ownership(cfg, report)
    check_real_sim_divergence(cfg, sim_path, report)

    report.dump(sys.stdout)
    return 1 if report.count(BLOCK) > 0 else 0


if __name__ == "__main__":
    sys.exit(main())
