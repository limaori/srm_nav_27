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

# 实车车体 TF 发布入口（SRM 新模型）。
#
# 车体模型只有一份：srm27_robot_description/urdf/srm27_sentry.urdf.xacro
# （几何取自 SRM全向轮哨兵.STEP 的 CAD 实测，实车/仿真共用）。本文件用
# scripts/srm_robot_description.build_description() 现场展开它，得到
# base_link / livox_frame / livox_imu / livox_scan / wheel_* / gimbal_yaw 的静态 TF。
#
# 与 real_mapping_launch.py 的关系:
#   描述构建用的是**同一个函数**（同一份模型、同一组外参），区别只在于本文件
#   只启动 robot_state_publisher，不启动雷达驱动 / Point-LIO / loam_interface /
#   sensor_scan_generation / pointcloud_to_laserscan / slam_toolbox —— 那些由
#   srm27 导航栈（nav2_stack_launch.py）负责，避免与导航入口重复启动同一批节点。
#
# 用法:
#   ros2 launch srm27_nav_bringup real_robot_state_publisher_launch.py

import json
import sys
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

# 描述构建模块与预检脚本装在同一个目录（见 CMakeLists 的 install(FILES ...)）。
sys.path.insert(
    0, str(Path(get_package_prefix("srm27_nav_bringup")) / "lib" / "srm27_nav_bringup")
)

from srm_robot_description import build_description  # noqa: E402


def launch_setup(context):
    params_file = LaunchConfiguration("params_file").perform(context)
    lidar_config = LaunchConfiguration("lidar_config").perform(context)

    # 雷达驱动里的外参必须为零，安装位姿统一由 lidar_xyz / lidar_rpy 表达，
    # 否则点云会被旋转两次。
    with open(lidar_config, encoding="utf-8") as config_stream:
        driver_config = json.load(config_stream)
    for lidar in driver_config["lidar_configs"]:
        if any(lidar["extrinsic_parameter"].values()):
            raise ValueError(
                "Set driver extrinsics to zero; use lidar_xyz/lidar_rpy for mounting"
            )

    robot_xml = build_description(
        params_file,
        LaunchConfiguration("lidar_xyz").perform(context),
        LaunchConfiguration("lidar_rpy").perform(context),
    )

    use_sim_time = ParameterValue(
        LaunchConfiguration("use_sim_time"), value_type=bool
    )
    clock_params = {"use_sim_time": use_sim_time}

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
    ]


def generate_launch_description():
    bringup_dir = Path(get_package_share_directory("srm27_nav_bringup"))
    arguments = [
        (
            "params_file",
            str(bringup_dir / "config/real/nav2_params_srm.yaml"),
            "Params file providing point_lio mapping extrinsic_T",
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
        ("use_sim_time", "False", "Use the simulation clock if True"),
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
