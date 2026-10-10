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

"""速度限幅自检：把 speed_limits.yaml 合并进参数文件，然后看"到底哪一层卡住了速度"。

回答的就是每次提速都会问的那个问题：**现在速度上限被哪一个限幅参数限制**。
它会按 launch 完全相同的方式（`srm_speed_limits.merge_params_files`）合并，再逐层列出
控制器 / 速度平滑器 / srm_cmd_mux（仿真）/ 串口（实车）的生效值，指出最小值来自哪一层，
并校验分层关系是否自洽。

用法
----
    # 检查仓库里全部 SRM 参数文件（仿真 + 实车，两种控制器变体）
    python3 src/srm27_navigation/srm27_nav_bringup/scripts/srm_speed_limits_check.py

    # 只查一份
    python3 .../srm_speed_limits_check.py --params <绝对路径>

    # 指定覆盖层（默认按 srm_speed_limits.resolve_overlay_path 的约定自动查找）
    python3 .../srm_speed_limits_check.py --params <参数文件> --speed-limits <speed_limits.yaml>

    # 把合并结果写出来看（launch 用的就是这份内容）
    python3 .../srm_speed_limits_check.py --params <参数文件> --dump /tmp/merged.yaml

退出码：0 = 没有阻塞问题；1 = 有阻塞问题（分层不自洽 / 两处重复定义）；2 = 用法或读取错误。
"""

import argparse
import os
import sys

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import srm_speed_limits as speed  # noqa: E402

#: 阻塞问题的前缀，便于一眼区分。
BLOCK = "阻塞"
WARN = "警告"
INFO = "提示"


def _num(value):
    return float(value) if isinstance(value, (int, float)) else None


def _fmt(value, unit=""):
    number = _num(value)
    if number is None:
        return "—"
    return ("%.2f" % number) + ((" " + unit) if unit else "")


def check_invariants(merged, variant):
    """分层自洽性检查，返回 [(等级, 说明)]。"""
    summary = speed.chain_summary(merged)
    controller = _num(summary["controller"])
    smoother = _num(summary["smoother"])
    mux = _num(summary["mux_linear"])
    ctrl_accel = _num(summary["controller_accel"])
    smooth_accel = _num(summary["smoother_accel"])
    horizon = _num(speed.lookup(merged, "controller_server.FollowPath.planning_horizon"))
    brake = _num(speed.lookup(merged, "controller_server.FollowPath.safety.braking_deceleration"))
    latency = _num(speed.lookup(merged, "controller_server.FollowPath.safety.reaction_latency"))

    problems = []
    if controller is None:
        problems.append((BLOCK, "控制器上限读不到（v_linear_max / limits.max_linear_speed 都没定义）"))
        return problems

    if smoother is not None and smoother < controller:
        problems.append(
            (
                BLOCK,
                "velocity_smoother 的 %.2f m/s 低于控制器上限 %.2f m/s："
                "上游提速会被它整个吃掉" % (smoother, controller),
            )
        )
    if mux is not None and mux < controller:
        problems.append(
            (
                BLOCK,
                "srm_cmd_mux.v_max 的 %.2f m/s 低于控制器上限 %.2f m/s："
                "cmd_vel_sim 被钳住（仿真的常见坑）" % (mux, controller),
            )
        )
    if ctrl_accel is not None and smooth_accel is not None and smooth_accel < ctrl_accel:
        problems.append(
            (
                BLOCK,
                "平滑器 max_accel %.2f < 控制器 max_linear_accel %.2f："
                "加速度被平滑器限制，短航段更跑不满" % (smooth_accel, ctrl_accel),
            )
        )

    # 视野决定"物理上能不能跑到"：加速-减速三角剖面的峰值 ≈ √(a·d)。
    if horizon is not None and ctrl_accel is not None:
        peak = (ctrl_accel * horizon) ** 0.5
        if peak < controller * 0.95:
            problems.append(
                (
                    WARN,
                    "planning_horizon %.1f m × a_max %.1f 的三角剖面峰值只有 %.2f m/s，"
                    "直道上也到不了 %.2f m/s（要真跑到需要 horizon ≈ v²/a = %.1f m）"
                    % (horizon, ctrl_accel, peak, controller, controller * controller / ctrl_accel),
                )
            )
        # 制动距离 = 反应延迟×v + v²/(2a) + 车体包络。
        accel = brake if brake and brake > 0.0 else ctrl_accel
        latency = latency if latency is not None else 0.1
        stop_distance = latency * controller + controller * controller / (2.0 * accel) + 0.33
        if stop_distance > horizon:
            problems.append(
                (
                    BLOCK,
                    "按 %.2f m/s、a=%.2f、延迟 %.2f s 估的制动距离 %.2f m 超过 view horizon %.1f m："
                    "轨迹校验会拒绝候选轨迹（车不动/反复 recovery），"
                    "要提速必须先加大 horizon 或做制动辨识"
                    % (controller, accel, latency, stop_distance, horizon),
                )
            )
        elif stop_distance > 0.7 * horizon:
            problems.append(
                (
                    WARN,
                    "制动距离 %.2f m 已占 horizon %.1f m 的 %.0f%%（含车体包络 0.33 m），余量偏小"
                    % (stop_distance, horizon, 100.0 * stop_distance / horizon),
                )
            )
    if brake == 0.0 or brake is None:
        problems.append(
            (
                INFO,
                "safety.braking_deceleration = 0：校验器退化用 limits.max_linear_accel 当制动能力"
                "（未经辨识的乐观值）",
            )
        )
    return problems


def check_protocol(merged, protocol_cfg):
    """实车：把串口那一层也算进来（它不在 speed_limits.yaml 里）。"""
    problems = []
    limits = speed.lookup(merged, "controller_server.FollowPath.limits.max_linear_speed")
    controller = _num(limits if limits is not None else speed.lookup(
        merged, "controller_server.FollowPath.v_linear_max"))
    if protocol_cfg is None or controller is None:
        return problems
    max_vx = _num(speed.lookup(protocol_cfg, "max_vx"))
    scale = _num(speed.lookup(protocol_cfg, "linear_velocity_scale")) or 1.0
    if max_vx is None:
        return problems
    # 串口限幅作用在缩放之后：出口 = min(scale × 指令, max_vx)，指令 ≤ 控制器上限。
    serial_ceiling = min(controller * scale if scale > 0 else controller, max_vx)
    problems.append(
        (
            INFO,
            "串口层（不在 speed_limits.yaml 里）：max_vx %.2f × 对齐系数 %.2f → "
            "出口上限 %.2f m/s%s"
            % (
                max_vx,
                scale,
                serial_ceiling,
                "；这一层比控制器低，实车真正的瓶颈是它" if serial_ceiling < controller else "",
            ),
        )
    )
    if controller * scale > max_vx:
        problems.append(
            (
                WARN,
                "控制器上限 %.2f × 对齐系数 %.2f = %.2f 超过串口 max_vx %.2f："
                "串口限幅作用在缩放之后，超过的部分会被静默砍掉"
                % (controller, scale, controller * scale, max_vx),
            )
        )
    return problems


def resolve_protocol_config(params_path):
    """按工作空间布局猜 srm27_nav_protocol 配置的位置（与实车预检同一套约定）。"""
    here = os.path.dirname(os.path.abspath(params_path))
    src_root = os.path.normpath(os.path.join(here, "..", "..", ".."))
    candidates = [
        os.path.join(src_root, "srm27_nav_protocol", "config", "srm27_nav_protocol.yaml"),
        os.path.normpath(
            os.path.join(here, "..", "..", "..", "..", "install", "srm27_nav_protocol",
                         "share", "srm27_nav_protocol", "config", "srm27_nav_protocol.yaml")
        ),
    ]
    for candidate in candidates:
        if os.path.isfile(candidate):
            return candidate
    return None


def default_params_files():
    """仓库里全部 SRM 参数文件：config/{simulation,real}/nav2_params_srm*.yaml。"""
    here = os.path.dirname(os.path.abspath(__file__))
    config_dir = os.path.normpath(os.path.join(here, "..", "config"))
    found = []
    for mode in ("simulation", "real"):
        mode_dir = os.path.join(config_dir, mode)
        if not os.path.isdir(mode_dir):
            continue
        for name in sorted(os.listdir(mode_dir)):
            if name.startswith(speed.SRM_PARAMS_PREFIX) and name.endswith(".yaml"):
                found.append(os.path.join(mode_dir, name))
    return found


def check_one(params_path, overlay, protocol_cfg, dump=None):
    """检查一份参数文件，打印生效链；返回阻塞问题数。"""
    print("=" * 78)
    print("参数文件: %s" % params_path)
    try:
        result = speed.merge_params_files(params_path, overlay)
    except speed.SpeedLimitError as exc:
        print("  [%s] %s" % (BLOCK, exc))
        return 1
    if result.overlay_path:
        print("覆盖层:   %s" % result.overlay_path)
    else:
        print("覆盖层:   （无——只检查参数文件里已写的值）")
    print("控制器变体: %s" % (result.variant or "未识别（本文件没有控制器插件段）"))

    blocking = 0
    for entry in result.conflicts:
        print("  [%s] 两处定义且值不同: %s" % (BLOCK, entry))
        blocking += 1
    for entry in result.duplicates:
        print(
            "  [%s] 参数文件里仍有重复定义（应删除，速度限幅只写在 %s）: %s"
            % (BLOCK, speed.OVERLAY_FILENAME, entry)
        )
        blocking += 1

    if result.applied and "controller_server" not in result.sections:
        print("  [%s] 覆盖层里没有适用于变体 %r 的 controller_server 段" % (BLOCK, result.variant))
        blocking += 1

    print(speed.format_chain(result.merged, indent="  "))

    for level, message in check_invariants(result.merged, result.variant):
        print("  [%s] %s" % (level, message))
        if level == BLOCK:
            blocking += 1
    if protocol_cfg is not None:
        for level, message in check_protocol(result.merged, protocol_cfg):
            print("  [%s] %s" % (level, message))

    if dump:
        with open(dump, "w") as stream:
            yaml.safe_dump(result.merged, stream, default_flow_style=False,
                           sort_keys=False, allow_unicode=True)
        print("合并结果已写出: %s" % dump)
    return blocking


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="速度限幅自检：合并 speed_limits.yaml 并指出卡住速度的是哪一层。",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("--params", action="append", default=None,
                        help="Nav2 参数文件；可重复。不给时检查仓库里全部 nav2_params_srm*.yaml")
    parser.add_argument("--speed-limits", default=speed.AUTO,
                        help="速度限幅覆盖层；默认 auto（同目录的 speed_limits.yaml）")
    parser.add_argument("--protocol", default=None,
                        help="（可选）srm27_nav_protocol 配置，用于把串口那一层一起算出来")
    parser.add_argument("--dump", default=None,
                        help="把合并后的参数写出到这个路径，便于人工核对")
    args = parser.parse_args(argv)

    params_files = args.params or default_params_files()
    if not params_files:
        print("错误: 没找到任何参数文件", file=sys.stderr)
        return 2

    protocol_path = args.protocol
    if protocol_path is None and any(os.sep + "real" + os.sep in path for path in params_files):
        protocol_path = resolve_protocol_config(params_files[0])
    protocol_cfg = speed.load_yaml(protocol_path) if protocol_path else None
    if protocol_cfg is not None:
        print("串口配置: %s" % protocol_path)

    blocking = 0
    for params_path in params_files:
        if not os.path.isfile(params_path):
            print("错误: 参数文件不存在: %s" % params_path, file=sys.stderr)
            return 2
        blocking += check_one(params_path, args.speed_limits, protocol_cfg, args.dump)

    print("=" * 78)
    if blocking:
        print("结论: 有 %d 个阻塞问题需要修（见上面 [%s] 行）。" % (blocking, BLOCK))
        return 1
    print("结论: 分层自洽，没有阻塞问题。")
    print(
        "提速时记住：改档位只改 speed_limits.yaml；"
        "上述 '这条链的上限' 就是当前被限住的那一层。"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
