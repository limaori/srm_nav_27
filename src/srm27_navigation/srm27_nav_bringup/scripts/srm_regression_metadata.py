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

"""写出一次 SRM 仿真回归的元数据（实施方案 §8）。

记录中必须能复现「世界 / 模型参数 / 软件版本」，本脚本把这些信息落到
<输出目录>/metadata.yaml，与 rosbag 放在一起。
"""

import argparse
import datetime
import os
import platform
import re
import socket
import subprocess
import sys

import yaml

SRM_PACKAGES = (
    "srm27_robot_description",
    "srm27_chassis_control",
    "srm27_gazebo_simulator",
    "srm27_nav_bringup",
)


def run(cmd, cwd=None):
    try:
        result = subprocess.run(
            cmd, cwd=cwd, capture_output=True, text=True, check=False
        )
    except OSError:
        return None
    if result.returncode != 0:
        return None
    return result.stdout.strip()


def workspace_root():
    """从本文件位置推导工作空间根目录（src/<pkg>/scripts/xxx.py）。

    安装后本文件是 install/<pkg>/lib/<pkg>/ 下的 symlink，必须用 realpath 解析
    回源码树，否则会算成 install 目录，git / 参数内联全都会取不到。
    """
    # src/<pkg>/scripts/xxx.py -> 上溯 4 层才是工作空间根
    return os.path.abspath(
        os.path.join(
            os.path.dirname(os.path.realpath(__file__)),
            "..", "..", "..", "..",
        )
    )


def git_info(root):
    if not os.path.isdir(os.path.join(root, ".git")):
        return {}
    return {
        "branch": run(["git", "rev-parse", "--abbrev-ref", "HEAD"], cwd=root),
        "commit": run(["git", "rev-parse", "HEAD"], cwd=root),
        "describe": run(["git", "describe", "--always", "--dirty", "--tags"], cwd=root),
        "dirty": bool(run(["git", "status", "--porcelain"], cwd=root)),
    }


def package_versions(root):
    versions = {}
    src_dir = os.path.join(root, "src")
    for dirpath, _dirnames, filenames in os.walk(src_dir):
        if "package.xml" not in filenames:
            continue
        path = os.path.join(dirpath, "package.xml")
        try:
            with open(path, "r", encoding="utf-8") as handle:
                text = handle.read()
        except OSError:
            continue
        name = re.search(r"<name>([^<]+)</name>", text)
        version = re.search(r"<version>([^<]+)</version>", text)
        if name and version and name.group(1) in SRM_PACKAGES:
            versions[name.group(1)] = version.group(1)
    return versions


def read_yaml(path):
    if not path or not os.path.isfile(path):
        return None
    try:
        with open(path, "r", encoding="utf-8") as handle:
            return yaml.safe_load(handle)
    except (OSError, yaml.YAMLError):
        return None


def physics_from_world(world_sdf):
    """从世界 SDF 里取出物理步长，便于复现仿真节奏。"""
    if not world_sdf or not os.path.isfile(world_sdf):
        return {}
    try:
        with open(world_sdf, "r", encoding="utf-8") as handle:
            text = handle.read()
    except OSError:
        return {}
    step = re.search(r"<max_step_size>([^<]+)</max_step_size>", text)
    rtf = re.search(r"<real_time_factor>([^<]+)</real_time_factor>", text)
    return {
        "world_sdf": world_sdf,
        "max_step_size": step.group(1) if step else None,
        "real_time_factor": rtf.group(1) if rtf else None,
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description="写出 SRM 仿真回归元数据")
    parser.add_argument("--output-dir", required=True, help="与 rosbag 同目录")
    parser.add_argument("--world", default="")
    parser.add_argument("--map", default="")
    parser.add_argument("--params-file", default="")
    parser.add_argument("--namespace", default="")
    parser.add_argument("--note", default="")
    parser.add_argument("--case", default="")
    args = parser.parse_args(argv)

    root = workspace_root()
    geometry_file = os.path.join(
        root, "src", "srm27_robot_description", "config", "srm27_sentry_geometry.yaml"
    )
    sim_config_file = os.path.join(
        root, "src", "srm27_gazebo_simulator", "config", "srm_sim.yaml"
    )
    chassis_config_file = os.path.join(
        root, "src", "srm27_chassis_control", "config", "srm_chassis_control.yaml"
    )

    metadata = {
        "recorded_at": datetime.datetime.now().astimezone().isoformat(),
        "case": args.case,
        "note": args.note,
        "robot_namespace": args.namespace,
        "world": args.world,
        "map": args.map,
        "params_file": args.params_file,
        "workspace": root,
        "git": git_info(root),
        "packages": package_versions(root),
        "ros": {
            "distro": os.environ.get("ROS_DISTRO", ""),
            "domain_id": os.environ.get("ROS_DOMAIN_ID", "0"),
            "rmw": os.environ.get("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp"),
        },
        "host": {
            "hostname": socket.gethostname(),
            "platform": platform.platform(),
            "kernel": platform.release(),
            "cpu": platform.processor(),
        },
        "physics": physics_from_world(
            os.path.join(
                root,
                "src",
                "srm27_gazebo_simulator",
                "worlds",
                f"{args.world}.sdf",
            )
            if args.world
            else ""
        ),
        # 世界/模型/控制器参数直接内联，避免以后改了配置就无法复现本次记录。
        "srm_sim_config": read_yaml(sim_config_file),
        "srm_sentry_geometry": read_yaml(geometry_file),
        "srm_chassis_control_config": read_yaml(chassis_config_file),
        "random_seed": {
            "gazebo": "未设置（Fortress 默认物理为确定性积分，噪声来自 SDF 中的高斯参数）",
            "sensor_noise": "见 srm27_sentry_geometry.yaml 的 lidar.noise_stddev",
        },
    }

    os.makedirs(args.output_dir, exist_ok=True)
    output = os.path.join(args.output_dir, "metadata.yaml")
    with open(output, "w", encoding="utf-8") as handle:
        yaml.safe_dump(metadata, handle, allow_unicode=True, sort_keys=False)
    print(f"[metadata] 已写入 {output}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
