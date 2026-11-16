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

"""命令行入口：从几何 YAML 生成 SRM 的 URDF 或 SDF。

用法：
  generate_robot_model.py --format urdf
  generate_robot_model.py --format sdf --output /tmp/srm_sentry.sdf
  generate_robot_model.py --format sdf --velocity-topic cmd_vel --command-timeout 0.1
"""

import argparse
import sys

from srm27_robot_description.model_builder import build_sdf, build_urdf


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="生成 SRM 机器人 URDF/SDF")
    parser.add_argument(
        "--format",
        choices=("urdf", "sdf"),
        required=True,
        help="生成格式",
    )
    parser.add_argument(
        "--geometry",
        default=None,
        help="几何参数 YAML 路径，默认使用包内 config/srm27_sentry_geometry.yaml",
    )
    parser.add_argument("--output", default=None, help="输出文件，缺省打印到标准输出")
    parser.add_argument(
        "--velocity-topic",
        default="cmd_vel",
        help="SDF 中速度执行插件订阅的 Gazebo Transport 相对话题名",
    )
    parser.add_argument(
        "--odometry-topic",
        default="odometry",
        help="SDF 中速度执行插件发布的 Gazebo Transport 真值里程计话题名",
    )
    parser.add_argument(
        "--command-timeout",
        type=float,
        default=0.1,
        help="SDF 中速度执行插件的命令超时（秒）",
    )
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)

    from srm27_robot_description import load_geometry

    geometry = load_geometry(args.geometry)
    if args.format == "urdf":
        text = build_urdf(geometry)
    else:
        text = build_sdf(
            geometry,
            velocity_topic=args.velocity_topic,
            odometry_topic=args.odometry_topic,
            command_timeout=args.command_timeout,
        )

    if args.output:
        with open(args.output, "w", encoding="utf-8") as handle:
            handle.write(text)
    else:
        sys.stdout.write(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
