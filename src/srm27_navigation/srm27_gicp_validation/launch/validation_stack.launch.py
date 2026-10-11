# GICP 初始重定位验证专用的「长期存活」链路（方案 §3.2）。
#
# 只包含验证真正需要、且在整场试验中保持不变的节点：
#
#   ign_sim_pointcloud_tool      → velodyne_points（雷达系点云转换）
#   point_lio                    → aft_mapped_to_init（Point-LIO 里程计）
#   gicp_registered_scan_provider → registered_scan（odom 系）+ odom → base_link
#   gicp_static_tf_republisher   → base_link → front_mid360（周期重发外参）
#
# 不包含：Nav2 规划/控制/代价地图、terrain_analysis、RViz、自转发送器。
# 被测量本身（small_gicp_relocalization）刻意**不在这里**启动——它由轮次驱动
# 逐轮重启，见 gicp_reload.launch.py。

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    bringup_dir = get_package_share_directory("srm27_nav_bringup")
    default_params = os.path.join(bringup_dir, "config", "simulation", "nav2_params_srm.yaml")

    namespace = LaunchConfiguration("namespace")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_pcd_localization = LaunchConfiguration("use_pcd_localization")
    use_lio_odometry = LaunchConfiguration("use_lio_odometry")
    slam = LaunchConfiguration("slam")
    log_level = LaunchConfiguration("log_level")

    has_odometry_source = PythonExpression(
        [slam, " or ", use_pcd_localization, " or ", use_lio_odometry]
    )

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={"use_sim_time": use_sim_time},
            convert_types=True,
        ),
        allow_substs=True,
    )

    actions = [
        DeclareLaunchArgument("namespace", default_value="red_standard_robot1"),
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("use_sim_time", default_value="True"),
        DeclareLaunchArgument("use_pcd_localization", default_value="True"),
        DeclareLaunchArgument("use_lio_odometry", default_value="False"),
        DeclareLaunchArgument("slam", default_value="False"),
        DeclareLaunchArgument("log_level", default_value="info"),
        GroupAction(
            actions=[
                PushRosNamespace(namespace),
                Node(
                    package="ign_sim_pointcloud_tool",
                    executable="ign_sim_pointcloud_tool_node",
                    name="ign_sim_pointcloud_tool",
                    output="screen",
                    parameters=[configured_params],
                ),
                Node(
                    condition=IfCondition(has_odometry_source),
                    package="point_lio",
                    executable="pointlio_mapping",
                    name="point_lio",
                    output="screen",
                    parameters=[
                        configured_params,
                        {"use_sim_time": use_sim_time},
                        # 与 nav_srm_simulation_launch.py 一致：Point-LIO 的先验地图配准
                        # 在本工程会发散，先验地图求 map->odom 由 GICP 完成。
                        {"prior_pcd.enable": False},
                    ],
                    arguments=["--ros-args", "--log-level", log_level],
                ),
                # registered_scan + odom → base_link 的唯一提供者（详见该节点文件头注释：
                # 上游 loam_interface / sensor_scan_generation 在本环境不可用）。
                ExecuteProcess(
                    cmd=[
                        os.path.join(
                            get_package_prefix("srm27_gicp_validation"),
                            "bin",
                            "gicp_registered_scan_provider",
                        ),
                        "--ros-args",
                        "-p",
                        "use_sim_time:=true",
                        "-p",
                        "input_cloud_topic:=/red_standard_robot1/velodyne_points",
                        "-p",
                        "input_odometry_topic:=/red_standard_robot1/aft_mapped_to_init",
                        "-p",
                        "output_cloud_topic:=/red_standard_robot1/registered_scan",
                        # 与 point_lio / robot_state_publisher 保持同一个 TF 话题
                        "-p",
                        "tf_topic:=/tf",
                    ],
                    name="gicp_registered_scan_provider",
                    output="screen",
                ),
                # 周期重发底盘 -> 雷达外参：本环境下 /tf_static 的 latch 消息送不到
                # 晚加入的 tf2 监听者，缺了它整条链路起不来（见该节点文件头注释）。
                ExecuteProcess(
                    cmd=[
                        os.path.join(
                            get_package_prefix("srm27_gicp_validation"),
                            "bin",
                            "gicp_static_tf_republisher",
                        ),
                        "--ros-args",
                        "-p",
                        "use_sim_time:=true",
                        "-p",
                        "tf_topic:=/tf",
                    ],
                    name="gicp_static_tf_republisher",
                    output="screen",
                ),
            ]
        ),
    ]

    return LaunchDescription(actions)
