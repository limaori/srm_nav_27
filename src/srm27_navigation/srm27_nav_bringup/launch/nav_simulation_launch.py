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

"""已弃用的仿真入口，转发到 nav_srm_simulation_launch.py。

旧步兵模型使用的 gimbal_yaw_fake / fake_vel_transform 链路和 MecanumDrive2 底盘
控制器已经删除，SRM 仿真只有一条速度执行链路。保留本文件只是为了让旧命令还能
跑起来；请改用：

  ros2 launch srm27_nav_bringup nav_srm_simulation_launch.py ...

新旧底盘执行器互斥：不要同时启动本入口与 srm27_gazebo_simulator 的旧入口。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution

_FORWARDED_ARGUMENTS = (
    "namespace",
    "slam",
    "world",
    "map",
    "prior_pcd_file",
    "use_sim_time",
    "params_file",
    "autostart",
    "use_composition",
    "use_respawn",
    "rviz_config_file",
    "use_rviz",
    "use_pcd_localization",
    "use_lio_odometry",
    "map_to_odom_x",
    "map_to_odom_y",
    "map_to_odom_yaw",
)


def generate_launch_description():
    bringup_dir = get_package_share_directory("srm27_nav_bringup")

    ld = LaunchDescription()
    ld.add_action(
        LogInfo(
            msg=(
                "[已弃用] nav_simulation_launch.py 已转发到 nav_srm_simulation_launch.py。"
                " 请直接使用 SRM 入口；参数文件默认改为 config/simulation/nav2_params_srm.yaml。"
            )
        )
    )

    # 沿用旧参数的默认值，转发时逐个传下去。
    ld.add_action(DeclareLaunchArgument("namespace", default_value="red_standard_robot1"))
    ld.add_action(DeclareLaunchArgument("slam", default_value="False"))
    ld.add_action(DeclareLaunchArgument("world", default_value="rmuc_2025"))
    ld.add_action(
        DeclareLaunchArgument(
            "map",
            default_value=[
                TextSubstitution(
                    text=os.path.join(
                        bringup_dir, "map", "simulation", "rmuc_2025_tunnel.yaml"
                    )
                )
            ],
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "prior_pcd_file",
            default_value=[
                TextSubstitution(text=os.path.join(bringup_dir, "pcd", "simulation", "")),
                LaunchConfiguration("world"),
                TextSubstitution(text=".pcd"),
            ],
        )
    )
    ld.add_action(DeclareLaunchArgument("use_sim_time", default_value="True"))
    ld.add_action(
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(
                bringup_dir, "config", "simulation", "nav2_params_srm.yaml"
            ),
        )
    )
    ld.add_action(DeclareLaunchArgument("autostart", default_value="true"))
    ld.add_action(DeclareLaunchArgument("use_composition", default_value="True"))
    ld.add_action(DeclareLaunchArgument("use_respawn", default_value="False"))
    ld.add_action(
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=os.path.join(bringup_dir, "rviz", "nav2_default_view.rviz"),
        )
    )
    ld.add_action(DeclareLaunchArgument("use_rviz", default_value="True"))
    ld.add_action(DeclareLaunchArgument("use_pcd_localization", default_value="False"))
    ld.add_action(DeclareLaunchArgument("use_lio_odometry", default_value="False"))
    ld.add_action(DeclareLaunchArgument("map_to_odom_x", default_value="0.0"))
    ld.add_action(DeclareLaunchArgument("map_to_odom_y", default_value="0.0"))
    ld.add_action(DeclareLaunchArgument("map_to_odom_yaw", default_value="0.0"))

    forwarded = {
        name: LaunchConfiguration(name) for name in _FORWARDED_ARGUMENTS
    }

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, "launch", "nav_srm_simulation_launch.py")
            ),
            launch_arguments=forwarded.items(),
        )
    )

    return ld
