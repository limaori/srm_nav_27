# 把 SRM 自建栈的实车模型搬进 srm27_nav_bringup。
#
# 用途:
#   在 srm27 导航栈里发布 SRM 实车模型（base_link / livox_frame / livox_imu /
#   livox_scan），与当前 SRM 仿真模型的坐标系保持一致。
#
# 模型构建逻辑与 real_mapping_launch.py 完全一致（那是实车建图验证过的版本）:
#   1) 用本包内的 urdf/sentry_robot_cylinder.xacro（SRM 实车模型的本地副本，不再
#      依赖任何外部工作区），并把 lidar_xyz / lidar_rpy 作为 xacro 的外参映射注入
#      （雷达安装位姿只在这里定义一次）；
#   2) 把 xacro 里的 package://pb_rm_simulation/meshes/... mesh 改写成本包内的
#      package://srm27_nav_bringup/meshes/...（pb_rm_simulation 不在本工作空间）；
#   3) 追加两个 link 与固定关节:
#        livox_imu  : 挂在 livox_frame 下，偏移取 -mapping.extrinsic_T
#        livox_scan : 挂在 base_link 下，偏移取 lidar_xyz（二维投影用的水平系）
#
# 与 real_mapping_launch.py 的区别:
#   这里只启动 robot_state_publisher，不启动雷达驱动 / Point-LIO / loam_interface /
#   sensor_scan_generation / pointcloud_to_laserscan / slam_toolbox —— 这些由 srm27
#   导航栈（nav2_stack_launch.py）负责，避免与导航入口重复启动同一批节点。
#
# 用法:
#   ros2 launch srm27_nav_bringup real_robot_state_publisher_launch.py

import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def launch_setup(context):
    bringup_dir = Path(get_package_share_directory("srm27_nav_bringup"))
    robot_file = bringup_dir / "urdf/sentry_robot_cylinder.xacro"
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

    lidar_xyz = LaunchConfiguration("lidar_xyz").perform(context)
    lidar_rpy = LaunchConfiguration("lidar_rpy").perform(context)
    position = [float(value) for value in lidar_xyz.split()]
    rotation = [float(value) for value in lidar_rpy.split()]
    if len(position) != 3 or len(rotation) != 3:
        raise ValueError("lidar_xyz and lidar_rpy must each contain three numbers")
    if not all(math.isfinite(value) for value in position + rotation):
        raise ValueError("LiDAR mounting parameters must be finite")

    # livox_imu 相对 livox_frame 的偏移由 Point-LIO 的 extrinsic_T 决定，
    # 从参数文件读取，保证与里程计使用的外参是同一个来源。
    with open(params_file, encoding="utf-8") as params_stream:
        nav_params = yaml.safe_load(params_stream)
    imu_translation = nav_params["point_lio"]["ros__parameters"]["mapping"][
        "extrinsic_T"
    ]
    if len(imu_translation) != 3 or not all(
        math.isfinite(value) for value in imu_translation
    ):
        raise ValueError("extrinsic_T must contain three finite numbers")
    if nav_params["point_lio"]["ros__parameters"]["mapping"]["extrinsic_R"] != [
        1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0
    ]:
        raise ValueError("This MID360 model requires identity LiDAR-to-IMU rotation")

    robot_xml = xacro.process_file(
        str(robot_file), mappings={"xyz": lidar_xyz, "rpy": lidar_rpy}
    ).toxml()
    robot = ET.fromstring(robot_xml)

    # mesh 改写: pb_rm_simulation 不在本工作空间, 指向本包内的同路径副本
    for mesh in robot.iter("mesh"):
        filename = mesh.attrib["filename"]
        prefix = "package://pb_rm_simulation/"
        if filename.startswith(prefix):
            relative = filename.removeprefix(prefix)
            if not (bringup_dir / relative).is_file():
                raise FileNotFoundError(bringup_dir / relative)
            mesh.set("filename", f"package://srm27_nav_bringup/{relative}")

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
                {"robot_description": ET.tostring(robot, encoding="unicode")},
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
        ("lidar_xyz", "0.15 -0.15 0.22", "LiDAR origin in base_link, in metres"),
        (
            "lidar_rpy",
            "-0.06981317007977318 0.0 -1.5707963267948966",
            "LiDAR mounting roll pitch yaw, in radians",
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
