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

"""SRM 仿真回归记录（实施方案 §8）。

按 §8 的要求记录一次回归所需的全部话题，并把世界/模型参数、软件版本写到同一个
目录的 metadata.yaml。记录内容必须能区分三种速度：

  * 导航请求的速度        -> cmd_vel_nav
  * 合成后的执行命令      -> cmd_vel_sim
  * 实际运动速度          -> odometry（真值里程计）

用法：
  ros2 launch srm27_nav_bringup srm_simulation_record_launch.py \\
    namespace:=red_standard_robot1 world:=rmuc_2025 case:=nav_plain

  # 只记录速度链路相关话题（点云以原始话题记录，体积较小的情况）
  ros2 launch srm27_nav_bringup srm_simulation_record_launch.py record_clouds:=False
"""

import datetime
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_prefix


def _topic_list(namespace, record_clouds):
    prefix = f"/{namespace}" if namespace else ""
    topics = [
        "/clock",
        f"{prefix}/cmd_vel_nav",
        f"{prefix}/cmd_vel_sim",
        f"{prefix}/rotation_cmd",
        f"{prefix}/rotation_velocity",
        f"{prefix}/odometry",
        f"{prefix}/chassis_odometry_gt",
        f"{prefix}/tf",
        f"{prefix}/tf_static",
        f"{prefix}/diagnostics",
        f"{prefix}/livox/imu",
        f"{prefix}/terrain_map",
        f"{prefix}/terrain_map_ext",
        f"{prefix}/local_costmap/costmap_raw",
        f"{prefix}/global_costmap/costmap_raw",
        f"{prefix}/navigate_to_pose/_action/feedback",
        f"{prefix}/navigate_to_pose/_action/status",
    ]
    if record_clouds:
        topics.extend(
            [
                f"{prefix}/livox/lidar",
                f"{prefix}/velodyne_points",
                f"{prefix}/registered_scan",
            ]
        )
    return topics


def _launch_setup(context):
    namespace = LaunchConfiguration("namespace").perform(context)
    world = LaunchConfiguration("world").perform(context)
    map_file = LaunchConfiguration("map").perform(context)
    params_file = LaunchConfiguration("params_file").perform(context)
    case = LaunchConfiguration("case").perform(context)
    note = LaunchConfiguration("note").perform(context)
    output_dir = LaunchConfiguration("output_dir").perform(context).strip()
    record_clouds = LaunchConfiguration("record_clouds").perform(
        context
    ).strip().lower() in ("1", "true", "yes")
    compression = LaunchConfiguration("compression").perform(context).strip().lower()

    if not output_dir:
        stamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        suffix = f"_{case}" if case else ""
        output_dir = os.path.join("rosbags", f"srm_regression_{stamp}{suffix}")

    metadata_script = os.path.join(
        get_package_prefix("srm27_nav_bringup"),
        "lib",
        "srm27_nav_bringup",
        "srm_regression_metadata.py",
    )

    write_metadata = ExecuteProcess(
        cmd=[
            "python3",
            metadata_script,
            "--output-dir",
            output_dir,
            "--world",
            world,
            "--map",
            map_file,
            "--params-file",
            params_file,
            "--namespace",
            namespace,
            "--case",
            case,
            "--note",
            note,
        ],
        output="screen",
    )

    record_cmd = ["ros2", "bag", "record", "-o", output_dir]
    if compression and compression != "none":
        record_cmd.extend(
            ["--compression-mode", "file", "--compression-format", compression]
        )
    record_cmd.extend(_topic_list(namespace, record_clouds))

    record = ExecuteProcess(cmd=record_cmd, output="screen")

    return [write_metadata, record]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="red_standard_robot1",
                description="机器人命名空间",
            ),
            DeclareLaunchArgument("world", default_value="rmuc_2025"),
            DeclareLaunchArgument("map", default_value=""),
            DeclareLaunchArgument("params_file", default_value=""),
            DeclareLaunchArgument(
                "case",
                default_value="",
                description="回归场景标识，写进 metadata.yaml（如 nav_plain / nav_constant_rotation）",
            ),
            DeclareLaunchArgument("note", default_value=""),
            DeclareLaunchArgument(
                "output_dir",
                default_value="",
                description="记录目录，留空则用 rosbags/srm_regression_<时间戳>_<case>",
            ),
            DeclareLaunchArgument(
                "record_clouds",
                default_value="True",
                description="是否记录原始点云（体积较大）",
            ),
            DeclareLaunchArgument(
                "compression",
                default_value="zstd",
                description="rosbag 压缩格式：zstd / none",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
