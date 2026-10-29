# Copyright 2025 Lihan Chen
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

# NOTE: This startup file is only used when the navigation module is standalone.
# 独立调试用：只发车体 TF（robot_state_publisher），不碰雷达/LIO/导航节点。
#
# 车体模型与实车链路**同一份**：srm27_robot_description/urdf/srm27_sentry.urdf.xacro，
# 由 scripts/srm_robot_description.build_description() 展开（base_link 原点在地面、
# 轮子与云台 fixed、雷达帧 livox_frame），并追加 livox_imu / livox_scan。
# 也就是说这里发布出来的 TF 与 real_mapping_launch.py / 导航栈完全一致，
# 不会再出现"调试时的模型和跑车时的模型不是同一台车"。

import sys
from pathlib import Path

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetRemap

sys.path.insert(
    0, str(Path(get_package_prefix("srm27_nav_bringup")) / "lib" / "srm27_nav_bringup")
)

from srm_robot_description import build_description  # noqa: E402


def launch_setup(context):
    bringup_dir = Path(get_package_share_directory("srm27_nav_bringup"))
    params_file = LaunchConfiguration("params_file").perform(context)
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")

    robot_description, _mounting = build_description(
        params_file,
        LaunchConfiguration("lidar_xyz").perform(context),
        LaunchConfiguration("lidar_rpy").perform(context),
    )

    bringup_cmd_group = GroupAction(
        [
            PushRosNamespace(namespace=namespace),
            SetRemap("/tf", "tf"),
            SetRemap("/tf_static", "tf_static"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "robot_description": robot_description,
                    }
                ],
            ),
        ]
    )
    return [bringup_cmd_group]


def generate_launch_description():
    bringup_dir = Path(get_package_share_directory("srm27_nav_bringup"))
    arguments = [
        (
            "namespace",
            "",
            "Top-level namespace",
        ),
        (
            "use_sim_time",
            "false",
            "Use simulation (Gazebo) clock if true",
        ),
        (
            "params_file",
            str(bringup_dir / "config/real/nav2_params_srm.yaml"),
            "Params file providing point_lio mapping extrinsic_T",
        ),
        (
            "lidar_xyz",
            "",
            "LiDAR origin in base_link, in metres (CAD measurement, ground = 0)",
        ),
        (
            "lidar_rpy",
            "",
            "LiDAR mounting roll pitch yaw, in radians (CAD: roll 30 deg, yaw 63 deg)",
        ),
    ]
    return LaunchDescription(
        [
            *[
                DeclareLaunchArgument(
                    name, default_value=default, description=description
                )
                for name, default, description in arguments
            ],
            OpaqueFunction(function=launch_setup),
        ]
    )
