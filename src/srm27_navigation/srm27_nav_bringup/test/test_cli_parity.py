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

"""`start_real_nav.sh` 与 `start_sim_nav.sh` 的命令行参数面回归。

为什么值得单测：这两份脚本是"同一套导航栈的两个入口"。参数面一旦漂移，
现场就会遇到"仿真里这么写能起，实车报未知参数"这种纯摩擦——而且往往发生在
最不该分心的时候（上电、车在场地里）。

本测试钉住三件事：

1. **仿真脚本的每个参数，实车脚本都接受**（实车可以是"接受但提示/拒绝执行"，
   例如 `--gui` / `--verify-spin` / `--gimbal-spin`，但不能是"未知参数"）；
2. **两边同名的参数语义一致**：`--gicp` 在仿真里是"用先验点云重定位"，
   实车必须落到 `--reloc` 那条路，不能被当成别的东西；
3. **实车独有的参数不被误删**（`--mux` / `--gimbal-joint-topic` / `--speed-limits` 等）。

不启动任何进程：只解析两份脚本的文本，并用 `bash -n` 做语法自检。
"""

import os
import re
import subprocess
import sys
from pathlib import Path

#: 本文件在 <ws>/src/srm27_navigation/srm27_nav_bringup/test/ 下。
PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = PACKAGE_ROOT.parents[2]
SCRIPT_DIR = WORKSPACE_ROOT / "script"
SIM_SCRIPT = SCRIPT_DIR / "start_sim_nav.sh"
REAL_SCRIPT = SCRIPT_DIR / "start_real_nav.sh"

#: 允许"接受但不执行"的参数（实车侧只提示/拒绝，不能报未知参数）。
REAL_ACCEPTS_ONLY = {
    "--gui": "Gazebo GUI 开关，实车忽略",
    "--no-gui": "Gazebo GUI 开关，实车忽略",
    "--run": "Gazebo 立即运行开关，实车忽略",
    "--no-run": "Gazebo 立即运行开关，实车忽略",
    "--world": "--world 是仿真世界名，实车只提示忽略（地图用 -m）",
    "--verify-spin": "自主验证会驱动车辆，实车拒绝执行并打印人工步骤",
    "--gimbal-spin": "云台由下位机驱动，上位机无指令通道：只告警",
    "--no-gimbal-spin": "同上",
    "--prior-pcd-manifest": "用建图出生点推导初值，实车由 --map-to-odom/--init-pose 承担",
}

_FLAG_RE = re.compile(r"(?<![\w-])(--[a-z][a-z0-9-]*)")


def _assert(condition, message):
    if not condition:
        raise AssertionError(message)


def _flags(path):
    """抓取脚本里出现的所有 `--flag`（含帮助文本与 case 分支，两者都要一致）。"""
    text = path.read_text(encoding="utf-8")
    return set(_FLAG_RE.findall(text))


def _case_labels(path):
    """从 case 分支的标签里取参数（`--a|--b)` 这种写法也要能拆开）。"""
    text = path.read_text(encoding="utf-8")
    labels = set()
    for line in text.splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        match = re.match(r"^([-\w|]+)\)", stripped)
        if not match:
            continue
        for token in match.group(1).split("|"):
            token = token.strip()
            if token.startswith("--") or (len(token) == 2 and token.startswith("-")):
                labels.add(token)
    return labels


def test_scripts_exist_and_parse():
    for script in (SIM_SCRIPT, REAL_SCRIPT):
        _assert(script.is_file(), f"找不到脚本: {script}")
        result = subprocess.run(
            ["bash", "-n", str(script)], capture_output=True, text=True
        )
        _assert(result.returncode == 0, f"{script.name} 语法错误: {result.stderr}")


def test_every_sim_flag_is_accepted_by_the_real_script():
    """仿真能用的每个参数，实车都必须能解析（可以是"接受但提示"）。

    只比较**真正实现的 case 分支**，不比较注释：仿真脚本的注释里还留着
    已删除的旧参数（`--wz-source` / `--fakeyaw` / `--rotation-*`）的说明，
    那些不该逼着实车也实现一遍。
    """
    sim_flags = _case_labels(SIM_SCRIPT)
    real_flags = _case_labels(REAL_SCRIPT)
    _assert(sim_flags, "没能从 start_sim_nav.sh 里解析出任何 case 分支")
    missing = sorted(sim_flags - real_flags)
    _assert(
        not missing,
        "实车脚本缺少这些仿真脚本的参数（现场会出现「未知参数」）:\n  "
        + "\n  ".join(missing),
    )


def test_accept_only_flags_are_implemented_not_just_documented():
    """"接受但不执行"的参数必须在 case 分支里真的有入口，而不只是写在注释里。

    注释里写一句 "已支持" 而代码没接，是最难发现的漂移。
    """
    real_labels = _case_labels(REAL_SCRIPT)
    missing = sorted(flag for flag in REAL_ACCEPTS_ONLY if flag not in real_labels)
    _assert(
        not missing,
        "这些参数只在注释里出现、case 分支没有实现:\n  "
        + "\n  ".join(f"{f}（{REAL_ACCEPTS_ONLY[f]}）" for f in missing),
    )


def test_same_named_flags_keep_their_semantics():
    """同名参数必须落到同一套语义上（这里是几个最容易走味的）。"""
    real_text = REAL_SCRIPT.read_text(encoding="utf-8")

    # --gicp 必须是 --reloc 的别名：实车默认走 args 解析里写 START_RELOC=1
    gicp_case = re.search(r"^\s*--gicp\)(.*)$", real_text, re.MULTILINE)
    _assert(gicp_case is not None, "实车脚本没有 --gicp 分支")
    _assert("START_RELOC=1" in gicp_case.group(1),
            f"--gicp 必须等价于 --reloc: {gicp_case.group(1).strip()}")
    no_gicp_case = re.search(r"^\s*--no-gicp\)(.*)$", real_text, re.MULTILINE)
    _assert(no_gicp_case is not None, "实车脚本没有 --no-gicp 分支")
    _assert("NO_RELOC=1" in no_gicp_case.group(1),
            f"--no-gicp 必须落到 LIO + 静态 map->odom: {no_gicp_case.group(1).strip()}")

    # --init-pose 必须转成 map->odom（实车是二维静态 TF）
    _assert("apply_init_pose" in real_text and "MAP_TO_ODOM_X" in real_text,
            "--init-pose 必须被转成 map->odom 的 x/y/yaw")

    # 2026-11-08（云台系）：--chassis-spin 不再切 MINCO 的航向策略 —— 请求由 srm_cmd_mux
    # 的 rotation_cmd 直接消费（方案 §4.3）。它必须仍然在，且不能再往 spin 模式跳。
    _assert('CHASSIS_SPIN_CLI_GIVEN=1' in real_text,
            "--chassis-spin 仍然必须能下发决策层自转请求（spin_control.sh）")
    _assert('YAW_MODE="spin"' not in real_text,
            "--chassis-spin 不能再把航向策略切成 spin：云台系下 wz 由 mux 消费，"
            "MINCO 的 spin 模式只会输出没人用的 wz（方案 §4.2/§4.3）")
    # --gimbal-spin 必须真的提示"不执行"，而不是静默吞掉
    _assert("实车不执行 --gimbal-spin" in real_text,
            "--gimbal-spin 必须有明确的「不执行」告警")


def test_real_only_flags_survive():
    """实车专有参数不能被"对齐"顺手删掉。"""
    real_labels = _case_labels(REAL_SCRIPT)
    for flag in ("--mux", "--no-mux", "--gimbal-joint-topic", "--speed-limits",
                 "--lio-frontend", "--zlio-config", "--map-to-odom", "--stop",
                 "--list-maps"):
        _assert(flag in real_labels, f"实车专用参数丢了: {flag}")


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
