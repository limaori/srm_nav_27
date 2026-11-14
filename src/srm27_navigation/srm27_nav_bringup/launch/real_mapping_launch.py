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
from launch.substitutions import PythonExpression
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

    robot_xml, mounting = build_description(
        params_file,
        LaunchConfiguration("lidar_xyz").perform(context),
        LaunchConfiguration("lidar_rpy").perform(context),
        # 云台按**真关节**处理（默认 revolute，与仿真一致）。
        # 关键前提：revolute 时 base_link -> gimbal_yaw 这条 TF **只来自 /joint_states**，
        # 没有发布者的话 robot_state_publisher 一个动态 TF 都不发，livox_frame / livox_imu
        # 整棵子树从 base_link 断开 —— 实测后果是 loam_interface 永远初始化不了、
        # 没有 /registered_scan -> 没有 /scan -> 没有 map。所以下面一定会起
        # gimbal_joint_state_relay：有回传话题就用真实角，没有就 constant 暂位 0 度。
        gimbal_joint_type=LaunchConfiguration("gimbal_joint_type").perform(context),
        gimbal_is_root=LaunchConfiguration("gimbal_is_root").perform(context),
    )
    # 安装位姿与 gravity 都来自同一个来源（唯一模型），不会再出现"模型改了、重力没改"。
    position = mounting["xyz"]
    gravity = gravity_from_mounting(mounting["rpy"])

    use_sim_time = ParameterValue(
        LaunchConfiguration("use_sim_time"), value_type=bool
    )
    # 累积点云默认落到与 Point-LIO 同一个文件：下游 pcd_to_mapframe.py /
    # build_pcd_from_bag.sh 都按这个路径找，换前端不该换路径。
    data_root = Path(
        os.environ.get("SRM27_DATA_DIR", str(Path.home() / "srm_nav_27_data"))
    ).expanduser()
    default_pcd_path = str(data_root / "generated_pcd" / "point_lio" / "scans.pcd")
    pcd_save_path = LaunchConfiguration("pcd_save_path").perform(context) or default_pcd_path
    lio_frontend = LaunchConfiguration("lio_frontend").perform(context)
    zlio_config = LaunchConfiguration("zlio_config").perform(context)
    # 建图只换"谁产生里程计与配准点云"这一层：point_lio + loam_interface
    # 换成 frontend/zlio_node + zlio_bridge。下游 sensor_scan_generation /
    # pointcloud_to_laserscan / slam_toolbox 全部照旧（bridge 的输出沿用
    # registered_scan / lidar_odometry 原名）。
    use_zlio = lio_frontend == "zlio"
    # 2D 扫描的输入点云：默认 registered_scan（LIO 去畸变后的稀疏云）；想抓矮/薄障碍
    # 可传 scan_cloud_topic:=/livox/lidar/pointcloud（原始 ~20k 点/帧）。
    scan_cloud_topic = LaunchConfiguration("scan_cloud_topic").perform(context)
    gimbal_joint_topic = LaunchConfiguration("gimbal_joint_topic").perform(context)
    # 云台按真关节(与仿真一致)。没有回传角时用 constant 暂位 0 度, 见下方 relay 节点。
    gimbal_input_type = "float32" if gimbal_joint_topic else "constant"
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
            package="srm27_nav_bringup",
            executable="gimbal_joint_state_relay.py",
            name="gimbal_joint_state_relay",
            output="screen",
            parameters=[
                clock_params,
                {"input_topic": gimbal_joint_topic},
                {"input_type": gimbal_input_type},
                {"constant_angle": 0.0},
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
            condition=IfCondition(PythonExpression(["not ", str(use_zlio)])),
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
            condition=IfCondition(PythonExpression(["not ", str(use_zlio)])),
            output="screen",
            parameters=[params_file, clock_params],
        ),
        # ---- ZLIO 前端（lio_frontend:=zlio）----
        # 输入接线是实车口径：CustomMsg + livox/imu + IMU 单位 g。
        Node(
            package="frontend",
            executable="zlio_node",
            name="zlio",
            condition=IfCondition(PythonExpression([str(use_zlio)])),
            output="screen",
            parameters=[
                clock_params,
                {"config_file": zlio_config},
                {"input_type": "livox"},
                {"lidar_topic": "livox/lidar"},
                {"imu_topic": "livox/imu"},
                {"imu_acceleration_scale": 9.81},
                {"publish_tf": False},
                {"world_frame": "odom"},
                {"body_frame": "livox_imu"},
                {"reset_mode": "warm"},
                {"coordinated_reset": False},
            ],
            remappings=[
                ("odometry", "zlio/odometry"),
                ("lidar_odometry", "zlio/lidar_odometry"),
                ("frontend/frame", "zlio/frame"),
                ("frontend/odometry", "zlio/frontend_odometry"),
                ("frontend/session", "zlio/session"),
            ],
        ),
        Node(
            package="srm27_nav_bringup",
            executable="zlio_bridge.py",
            name="zlio_bridge",
            condition=IfCondition(PythonExpression([str(use_zlio)])),
            output="screen",
            parameters=[
                params_file,
                clock_params,
                # bridge 的默认 lidar_frame 是仿真的 front_mid360，实车必须覆盖。
                {"odom_frame": "odom", "base_frame": "base_link", "lidar_frame": "livox_imu"},
                # 累积点云落盘（等价 Point-LIO 的 pcd_save）：退出时按体素降采样写 PCD。
                {
                    "pcd_save_en": ParameterValue(
                        LaunchConfiguration("save_pcd"), value_type=bool
                    ),
                    "pcd_save_path": pcd_save_path,
                    "pcd_voxel_leaf": 0.10,
                },
            ],
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
            remappings=[("cloud_in", scan_cloud_topic)],
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
            "",
            "LiDAR origin in base_link, in metres (CAD measurement, ground = 0)",
        ),
        (
            "lidar_rpy",
            "",
            "LiDAR mounting roll pitch yaw, in radians (CAD: roll 30 deg, yaw 63 deg)",
        ),
        (
            "gimbal_is_root",
            "false",
            "TF 树反转（gimbal_yaw 为根、base_link 挂末端）。云台系导航要传 true，"
            "见 real_robot_state_publisher_launch.py 的同名参数。",
        ),
        (
            "gimbal_joint_type",
            "revolute",
            "云台关节类型（与仿真一致，默认 revolute）。"
            "前提是 /joint_states 有发布者：本文件一定会起 gimbal_joint_state_relay，"
            "有回传话题就用真实角，没有就用 constant 暂位 0 度。",
        ),
        (
            "lio_frontend",
            "zlio",
            "里程计/配准点云前端: zlio（默认，与导航链一致）或 point_lio（回退）。"
            "zlio 会起 frontend/zlio_node + zlio_bridge，不再起 point_lio/loam_interface；"
            "注意 ZLIO 没有 --save-pcd 那样的 PCD 落盘。",
        ),
        (
            "zlio_config",
            str(bringup_dir / "config/real/zlio_mid360.yaml"),
            "ZLIO 算法参数（只在 lio_frontend:=zlio 时生效）",
        ),
        (
            "scan_cloud_topic",
            "registered_scan",
            "喂给 pointcloud_to_laserscan 的点云。默认 registered_scan（LIO 输出、已去畸变，"
            "但降采样到 ~1.7k 点/帧：720 个方位格只覆盖 ~264 个，矮台阶/薄栏杆会被相邻射线"
            "当空闲刷掉）。改成 /livox/lidar/pointcloud 可覆盖 ~598 格，代价是没做运动去畸变。",
        ),
        (
            "gimbal_joint_topic",
            "",
            "云台关节角回传话题; 留空 = constant 暂位 0 度 (TF 与 fixed 等价)",
        ),
        (
            "start_lidar",
            "True",
            "Start the hardware driver; disable for rosbag playback",
        ),
        ("use_sim_time", "False", "Use the bag clock when replaying with --clock"),
        ("use_rviz", "True", "Start the mapping RViz view"),
        (
            "pcd_save_path",
            "",
            "累积点云落盘路径（默认 ~/srm_nav_27_data/generated_pcd/point_lio/scans.pcd，"
            "与 Point-LIO 同路径；只在 save_pcd 为真时生效）",
        ),
        (
            "save_pcd",
            "False",
            "在正常退出时落盘累积点云: point_lio 由它自己写, zlio 由 zlio_bridge 写",
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
