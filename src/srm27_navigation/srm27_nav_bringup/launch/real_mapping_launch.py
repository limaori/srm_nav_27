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

# 实车 MID360 建图入口。
#
# 车体模型只有一份：srm27_robot_description/urdf/srm27_sentry.urdf.xacro
# （几何取自 SRM全向轮哨兵.STEP 的 CAD 实测，实车/仿真共用）。本文件用
# scripts/srm_robot_description.build_description() 展开它，追加 livox_imu
# （偏移取 -mapping.extrinsic_T）与 livox_scan（偏移取 lidar_xyz）。
#
# 注意：lidar_rpy 同时决定两件事 ——
#   1) URDF 里 base_link -> livox_frame 的安装姿态；
#   2) Point-LIO 的 mapping.gravity / gravity_init（见 gravity_from_mounting）。
# 模型是 CAD 的 roll=30°，装进 Point-LIO 的重力向量就是"相对雷达 -z 偏 30°"的值。
# 若实测雷达其实是平装的，这里必须把 lidar_rpy 的 roll 改回小角度（TF 与重力
# 会一起跟着变，这正是它们共用同一个来源的意义）。
#
# 用法:
#   ros2 launch srm27_nav_bringup real_mapping_launch.py

import json
import sys
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

# 描述构建模块与预检脚本装在同一个目录（见 CMakeLists 的 install(FILES ...)）。
sys.path.insert(
    0, str(Path(get_package_prefix("srm27_nav_bringup")) / "lib" / "srm27_nav_bringup")
)

from srm_robot_description import (  # noqa: E402
    build_description,
    gravity_from_mounting,
    parse_triplet,
)


def launch_setup(context):
    params_file = LaunchConfiguration("params_file").perform(context)
    lidar_config = LaunchConfiguration("lidar_config").perform(context)
    with open(lidar_config, encoding="utf-8") as config_stream:
        driver_config = json.load(config_stream)
    for lidar in driver_config["lidar_configs"]:
        if any(lidar["extrinsic_parameter"].values()):
            raise ValueError(
                "Set driver extrinsics to zero; use lidar_xyz/lidar_rpy for mounting"
            )

    lidar_xyz = LaunchConfiguration("lidar_xyz").perform(context)
    lidar_rpy = LaunchConfiguration("lidar_rpy").perform(context)
    position = parse_triplet(lidar_xyz, "lidar_xyz")
    parse_triplet(lidar_rpy, "lidar_rpy")

    robot_xml = build_description(params_file, lidar_xyz, lidar_rpy)
    gravity = gravity_from_mounting(lidar_rpy)

    use_sim_time = ParameterValue(
        LaunchConfiguration("use_sim_time"), value_type=bool
    )
    clock_params = {"use_sim_time": use_sim_time}
    point_lio_defaults = str(
        Path(get_package_share_directory("point_lio")) / "config/mid360.yaml"
    )

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[
                clock_params,
                {"robot_description": robot_xml},
            ],
        ),
        Node(
            package="livox_ros_driver2",
            executable="livox_ros_driver2_node",
            name="livox_ros_driver2",
            condition=IfCondition(LaunchConfiguration("start_lidar")),
            output="screen",
            parameters=[
                params_file,
                clock_params,
                {"user_config_path": lidar_config},
            ],
        ),
        Node(
            package="point_lio",
            executable="pointlio_mapping",
            name="point_lio",
            output="screen",
            parameters=[
                point_lio_defaults,
                params_file,
                clock_params,
                {
                    "mapping.gravity": gravity,
                    "mapping.gravity_init": gravity,
                    "pcd_save.pcd_save_en": ParameterValue(
                        LaunchConfiguration("save_pcd"), value_type=bool
                    ),
                },
            ],
        ),
        Node(
            package="loam_interface",
            executable="loam_interface_node",
            name="loam_interface",
            output="screen",
            parameters=[params_file, clock_params],
        ),
        Node(
            package="sensor_scan_generation",
            executable="sensor_scan_generation_node",
            name="sensor_scan_generation",
            output="screen",
            parameters=[params_file, clock_params],
        ),
        Node(
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            name="pointcloud_to_laserscan",
            output="screen",
            parameters=[
                params_file,
                clock_params,
                {
                    # 切片带相对 livox_scan 系（在雷达高度上、水平），换算成
                    # "离地 0.10~1.00 m" 的绝对高度带；雷达装机高度变了也不用改这里。
                    "min_height": 0.10 - position[2],
                    "max_height": 1.00 - position[2],
                },
            ],
            remappings=[("cloud_in", "registered_scan")],
        ),
        Node(
            package="slam_toolbox",
            executable="sync_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[params_file, clock_params],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            condition=IfCondition(LaunchConfiguration("use_rviz")),
            arguments=["-d", str(bringup_dir_rviz())],
            parameters=[clock_params],
            output="screen",
        ),
    ]


def bringup_dir_rviz():
    bringup_dir = Path(get_package_share_directory("srm27_nav_bringup"))
    return bringup_dir / "rviz/mapping.rviz"


def generate_launch_description():
    bringup_dir = Path(get_package_share_directory("srm27_nav_bringup"))
    arguments = [
        (
            "params_file",
            str(bringup_dir / "config/real/mapping_params.yaml"),
            "Mapping node parameters",
        ),
        (
            "lidar_config",
            str(bringup_dir / "config/real/mid360_user_config.json"),
            "MID360 network config with zero driver extrinsics",
        ),
        (
            "lidar_xyz",
            "0.12081 -0.056721 0.346388",
            "LiDAR origin in base_link, in metres (CAD measurement, ground = 0)",
        ),
        (
            "lidar_rpy",
            "0.523598776 0.0 1.099557429",
            "LiDAR mounting roll pitch yaw, in radians (CAD: roll 30 deg, yaw 63 deg)",
        ),
        (
            "start_lidar",
            "True",
            "Start the hardware driver; disable for rosbag playback",
        ),
        ("use_sim_time", "False", "Use the bag clock when replaying with --clock"),
        ("use_rviz", "True", "Start the mapping RViz view"),
        (
            "save_pcd",
            "False",
            "Accumulate a Point-LIO PCD and save it on clean shutdown",
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
