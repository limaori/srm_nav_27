# 实车 MID360 建图入口。
#
# 车体模型用本包内的 urdf/sentry_robot_cylinder.xacro（SRM 实车模型的本地副本，
# 不依赖外部工作区）: lidar_xyz / lidar_rpy 作为 xacro 外参映射注入。mesh 指向
# srm27_robot_description 包内的副本；历史遗留的 package://pb_rm_simulation/meshes/...
# 前缀加载时改写为描述包内的同路径资源，
# 并追加 livox_imu（偏移取 -mapping.extrinsic_T）与 livox_scan（偏移取 lidar_xyz）。
#
# 用法:
#   ros2 launch srm27_nav_bringup real_mapping_launch.py

import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def launch_setup(context):
    bringup_dir = Path(get_package_share_directory("srm27_nav_bringup"))
    robot_file = bringup_dir / "urdf/sentry_robot_cylinder.xacro"
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
    position = [float(value) for value in lidar_xyz.split()]
    rotation = [float(value) for value in lidar_rpy.split()]
    if len(position) != 3 or len(rotation) != 3:
        raise ValueError("lidar_xyz and lidar_rpy must each contain three numbers")
    if not all(math.isfinite(value) for value in position + rotation):
        raise ValueError("LiDAR mounting parameters must be finite")

    with open(params_file, encoding="utf-8") as params_stream:
        mapping_params = yaml.safe_load(params_stream)
    lio_params = mapping_params["point_lio"]["ros__parameters"]
    imu_translation = lio_params["mapping"]["extrinsic_T"]
    if len(imu_translation) != 3 or not all(
        math.isfinite(value) for value in imu_translation
    ):
        raise ValueError("extrinsic_T must contain three finite numbers")
    if lio_params["mapping"]["extrinsic_R"] != [
        1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0
    ]:
        raise ValueError("This MID360 launch requires identity LiDAR-to-IMU rotation")

    robot_xml = xacro.process_file(
        str(robot_file), mappings={"xyz": lidar_xyz, "rpy": lidar_rpy}
    ).toxml()
    robot = ET.fromstring(robot_xml)
    # mesh 解析: xacro 已指向 srm27_robot_description 包内的副本;
    # 若还有历史遗留的 pb_rm_simulation 前缀, 一并改写为描述包内的同路径资源。
    description_dir = Path(get_package_share_directory("srm27_robot_description"))
    for mesh in robot.iter("mesh"):
        filename = mesh.attrib["filename"]
        prefix = "package://pb_rm_simulation/"
        if filename.startswith(prefix):
            relative = filename.removeprefix(prefix)
            if not (description_dir / relative).is_file():
                raise FileNotFoundError(description_dir / relative)
            mesh.set("filename", f"package://srm27_robot_description/{relative}")
    for child, parent, translation in [
        ("livox_imu", "livox_frame", [-value for value in imu_translation]),
        ("livox_scan", "base_link", position),
    ]:
        ET.SubElement(robot, "link", name=child)
        joint = ET.SubElement(robot, "joint", name=child + "_joint", type="fixed")
        ET.SubElement(joint, "parent", link=parent)
        ET.SubElement(joint, "child", link=child)
        ET.SubElement(
            joint, "origin", xyz=" ".join(map(str, translation)), rpy="0 0 0"
        )

    roll, pitch, _ = rotation
    gravity = [
        9.81 * math.sin(pitch),
        -9.81 * math.cos(pitch) * math.sin(roll),
        -9.81 * math.cos(pitch) * math.cos(roll),
    ]
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
                {"robot_description": ET.tostring(robot, encoding="unicode")},
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
            arguments=["-d", str(bringup_dir / "rviz/mapping.rviz")],
            parameters=[clock_params],
            output="screen",
        ),
    ]


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
        ("lidar_xyz", "0.15 -0.15 0.22", "LiDAR origin in base_link, in metres"),
        (
            "lidar_rpy",
            "-0.06981317007977318 0.0 -1.5707963267948966",
            "LiDAR mounting roll pitch yaw, in radians",
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
