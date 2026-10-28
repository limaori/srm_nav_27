# 仿真 SLAM 建图入口（Gazebo 由 script/start_sim_slam.sh 单独拉起）。

"""仿真 SLAM 建图的 launch 入口（Gazebo 由 script/start_sim_slam.sh 单独拉起）。

本文件本身很薄：把 ``nav_srm_simulation_launch.py`` 固定成 ``slam:=True``，
并把 RViz 配置和世界名透传下去。真正的建图节点都在下面这条链路里：

  * ``nav2_slam_launch.py`` 起 **Point-LIO**（里程计 + 3D 建图）、slam_toolbox
    （2D 栅格）、map_saver_server、pointcloud_to_laserscan；
  * ``navigation_launch.py`` 起 loam_interface / sensor_scan_generation
    （在 has_odometry_source 为真时把 publish_tf 置 True，发布 odom -> base_link）。

Point-LIO 退出时把累积点云落盘成 ``PCD/scans.pcd``，路径是编译期常量 ROOT_DIR：
``src/srm27_navigation/point_lio/PCD/scans.pcd``
（见 point_lio/CMakeLists.txt 的 ``-DROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/"``）。
用 ``./script/start_sim_slam.sh --collect`` 把它收走。

完整链路：

    Gazebo gpu_lidar ──bridge──> livox/lidar ──ign_sim_pointcloud_tool──> velodyne_points
                                                  livox/imu ─────────────────┤
                                                                             ▼
                                                                        Point-LIO
                                        ┌──────────────────────────────────┤
                                        ▼                                  ▼
                          cloud_registered / aft_mapped_to_init        PCD/scans.pcd
                                        │
                                 loam_interface ──> registered_scan
                                        │
                            sensor_scan_generation ──> odom->base_link / sensor_scan
                                        │
                    terrain_analysis* ──> terrain_map_ext ──> pointcloud_to_laserscan
                                        │                              │
                                        ▼                              ▼
                                   代价地图                        slam_toolbox ──> map (2D)

用法：
    ros2 launch srm27_nav_bringup sim_slam_launch.py
    ros2 launch srm27_nav_bringup sim_slam_launch.py save_pcd:=false
    ros2 launch srm27_nav_bringup sim_slam_launch.py rviz:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_dir = get_package_share_directory("srm27_nav_bringup")
    launch_dir = os.path.join(bringup_dir, "launch")

    namespace = LaunchConfiguration("namespace")
    world = LaunchConfiguration("world")
    params_file = LaunchConfiguration("params_file")
    rviz = LaunchConfiguration("rviz")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    chassis_control = LaunchConfiguration("start_chassis_control")

    # 1) Nav2 + 建图栈（slam:=True），Gazebo 不在这里起
    slam_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "nav_srm_simulation_launch.py")
        ),
        launch_arguments={
            "namespace": namespace,
            "world": world,
            "params_file": params_file,
            "slam": "True",
            "use_pcd_localization": "False",
            "use_lio_odometry": "False",
            "start_simulation": "False",
            "start_chassis_control": chassis_control,
            # 建图时不做自转测试，避免把旋转揉进地图
            "start_rotation_sender": "False",
            "use_sim_time": "True",
            "use_rviz": rviz,
            "rviz_config_file": rviz_config_file,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace", default_value="red_standard_robot1", description="机器人命名空间"
            ),
            DeclareLaunchArgument(
                "world", default_value="rmuc_2025", description="Gazebo 世界名（仅用于透传）"
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(
                    bringup_dir, "config", "simulation", "nav2_params_srm.yaml"
                ),
                description="仿真参数文件（含 point_lio / slam_toolbox 段）",
            ),
            DeclareLaunchArgument("rviz", default_value="true", description="是否启动 RViz"),
            DeclareLaunchArgument(
                "rviz_config_file",
                default_value=os.path.join(bringup_dir, "rviz", "nav2_default_view.rviz"),
                description="RViz 配置路径",
            ),
            DeclareLaunchArgument(
                "start_chassis_control",
                default_value="True",
                description="是否一并启动速度合成（cmd_mux 等）",
            ),
            slam_stack,
        ]
    )
