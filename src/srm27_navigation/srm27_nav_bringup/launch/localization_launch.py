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


import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory("srm27_nav_bringup")

    namespace = LaunchConfiguration("namespace")
    map_yaml_file = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    prior_pcd_file = LaunchConfiguration("prior_pcd_file")
    params_file = LaunchConfiguration("params_file")
    use_composition = LaunchConfiguration("use_composition")
    container_name = LaunchConfiguration("container_name")
    container_name_full = (namespace, "/", container_name)
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")
    use_pcd_localization = LaunchConfiguration("use_pcd_localization")
    # 只跑 Point-LIO 里程计、不做 GICP 重定位（"纯 LIO"模式）。
    # 与 use_pcd_localization 分开是为了让 "谁来提供里程计" 和 "要不要 GICP 修正"
    # 可以独立选择：纯 LIO 模式下 map->odom 由静态 TF 给出（见下文的
    # start_static_transform_node，它的条件是 use_pcd_localization 为假）。
    use_lio_odometry = LaunchConfiguration("use_lio_odometry")
    map_to_odom_x = LaunchConfiguration("map_to_odom_x")
    map_to_odom_y = LaunchConfiguration("map_to_odom_y")
    map_to_odom_yaw = LaunchConfiguration("map_to_odom_yaw")

    lifecycle_nodes = ["map_server"]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {"use_sim_time": use_sim_time, "yaml_filename": map_yaml_file}

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    colorized_output_envvar = SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1")

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map", description="Full path to map yaml file to load"
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_prior_pcd_file_cmd = DeclareLaunchArgument(
        "prior_pcd_file",
        default_value="",
        description="Full path to prior PCD file to load",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(bringup_dir, "config", "simulation", "nav2_params.yaml"),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the nav2 stack",
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        "use_composition",
        default_value="False",
        description="Use composed bringup if True",
    )

    declare_container_name_cmd = DeclareLaunchArgument(
        "container_name",
        default_value="nav2_container",
        description="the name of container that nodes will load in if use composition",
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn if a node crashes. Applied when composition is disabled.",
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level", default_value="info", description="log level"
    )

    declare_use_pcd_localization_cmd = DeclareLaunchArgument(
        "use_pcd_localization", default_value="False",
        description="Use small_gicp localization against a prior PCD",
    )
    declare_use_lio_odometry_cmd = DeclareLaunchArgument(
        "use_lio_odometry", default_value="False",
        description=(
            "Use Point-LIO odometry without GICP relocalization; "
            "map->odom stays a static transform"
        ),
    )
    declare_map_to_odom_x_cmd = DeclareLaunchArgument("map_to_odom_x", default_value="0.0")
    declare_map_to_odom_y_cmd = DeclareLaunchArgument("map_to_odom_y", default_value="0.0")
    declare_map_to_odom_yaw_cmd = DeclareLaunchArgument("map_to_odom_yaw", default_value="0.0")

    start_point_lio_node = Node(
        condition=IfCondition(
            PythonExpression([use_pcd_localization, " or ", use_lio_odometry])
        ),
        package="point_lio",
        executable="pointlio_mapping",
        name="point_lio",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[
            configured_params,
            {"use_sim_time": use_sim_time},
            # 注意: Point-LIO 的 prior_pcd(先验地图配准)在本工程里会导致状态发散
            # (实测开启后 odom 会漂到几十万米)。这里的 "先验地图求 map->odom"
            # 实际由 small_gicp_relocalization 完成, 因此 Point-LIO 只需跑纯
            # 里程计(prior_pcd 关闭), 保持稳定即可。故强制设为 False,
            # 不再跟随 use_pcd_localization。
            {"prior_pcd.enable": False},
            {"prior_pcd.prior_pcd_map_path": prior_pcd_file},
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    start_static_transform_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher_map2odom",
        output="screen",
        condition=IfCondition(PythonExpression(["not ", use_pcd_localization])),
        arguments=[
            "--x", map_to_odom_x, "--y", map_to_odom_y, "--z", "0.0",
            "--roll", "0.0", "--pitch", "0.0", "--yaw", map_to_odom_yaw,
            "--frame-id", "map", "--child-frame-id", "odom",
        ],
    )

    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(["not ", use_composition])),
        actions=[
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                condition=IfCondition(use_pcd_localization),
                package="small_gicp_relocalization",
                executable="small_gicp_relocalization_node",
                name="small_gicp_relocalization",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params, {"prior_pcd_file": prior_pcd_file}],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_localization",
                output="screen",
                arguments=["--ros-args", "--log-level", log_level],
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"autostart": autostart},
                    {"node_names": lifecycle_nodes},
                ],
            ),
        ],
    )

    load_composable_nodes = LoadComposableNodes(
        condition=IfCondition(use_composition),
        target_container=container_name_full,
        composable_node_descriptions=[
            ComposableNode(
                package="nav2_map_server",
                plugin="nav2_map_server::MapServer",
                name="map_server",
                parameters=[configured_params],
            ),
            ComposableNode(
                package="nav2_lifecycle_manager",
                plugin="nav2_lifecycle_manager::LifecycleManager",
                name="lifecycle_manager_localization",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": autostart,
                        "node_names": lifecycle_nodes,
                    }
                ],
            ),
        ],
    )

    # ComposableNode conditions are not consistently honored across ROS 2
    # distributions. Keep the optional GICP component behind an action-level
    # condition so it is never loaded for static-map localization.
    load_pcd_localization_node = LoadComposableNodes(
        condition=IfCondition(use_pcd_localization),
        target_container=container_name_full,
        composable_node_descriptions=[
            ComposableNode(
                package="small_gicp_relocalization",
                plugin="small_gicp_relocalization::SmallGicpRelocalizationNode",
                name="small_gicp_relocalization",
                parameters=[configured_params, {"prior_pcd_file": prior_pcd_file}],
            )
        ],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(colorized_output_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_prior_pcd_file_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_use_pcd_localization_cmd)
    ld.add_action(declare_use_lio_odometry_cmd)
    ld.add_action(declare_map_to_odom_x_cmd)
    ld.add_action(declare_map_to_odom_y_cmd)
    ld.add_action(declare_map_to_odom_yaw_cmd)

    # Add the actions to launch all of the localiztion nodes
    ld.add_action(start_point_lio_node)
    ld.add_action(start_static_transform_node)
    ld.add_action(load_nodes)
    ld.add_action(load_composable_nodes)
    ld.add_action(load_pcd_localization_node)

    return ld
