# Copyright 2026 SRM27 Navigation Team
# Licensed under the Apache License, Version 2.0.
#
import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def _include_simulation(context, bringup_launch):
    config_file = LaunchConfiguration("sim_config_file").perform(context)
    with open(config_file, encoding="utf-8") as stream:
        spawn = yaml.safe_load(stream)["robot"]

    # 真值里程计从出生位姿开始，场地地图使用 Gazebo 世界坐标。
    arguments = {
        f"map_to_odom_{axis}": LaunchConfiguration(f"map_to_odom_{axis}").perform(context)
        or str(spawn[axis])
        for axis in ("x", "y", "yaw")
    }
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_launch),
        launch_arguments=arguments.items(),
    )]


def generate_launch_description():
    share = get_package_share_directory("tdt_nav_kit")
    bringup_share = get_package_share_directory("srm27_nav_bringup")

    bringup_launch = os.path.join(
        bringup_share,
        "launch",
        "nav_srm_simulation_launch.py",
    )

    default_params = os.path.join(share, "config", "nav2_params_srm_tdt_smooth.yaml")
    default_sim_config = os.path.join(
        get_package_share_directory("srm27_gazebo_simulator"), "config", "srm_sim.yaml"
    )

    # 不传 speed_limits_file：原 launch 的默认值是显式绝对路径
    # config/simulation/speed_limits.yaml，与 params_file 所在目录无关。
    # 若显式改成 auto，参数文件经 ReplaceString 落到临时目录后 basename 变化，
    # srm_speed_limits.py 会静默不合并限幅，退回插件默认高值。
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="rmuc_2025"),
        DeclareLaunchArgument(
            "map", default_value=PathJoinSubstitution([
                bringup_share, "map", "simulation", [LaunchConfiguration("world"), ".yaml"],
            ])
        ),
        DeclareLaunchArgument("sim_config_file", default_value=default_sim_config),
        DeclareLaunchArgument("odometry_frontend", default_value="point_lio"),
        DeclareLaunchArgument("start_simulation", default_value="True"),
        DeclareLaunchArgument("run_immediately", default_value="True"),
        DeclareLaunchArgument("gui", default_value="False"),
        DeclareLaunchArgument("map_to_odom_x", default_value=""),
        DeclareLaunchArgument("map_to_odom_y", default_value=""),
        DeclareLaunchArgument("map_to_odom_yaw", default_value=""),
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description=(
                "默认启用 YAstar 和 MinimumSnap。传 nav2_params_srm_tdt.yaml 使用 SimpleSmoother。"
            ),
        ),
        OpaqueFunction(function=_include_simulation, args=[bringup_launch]),
    ])
