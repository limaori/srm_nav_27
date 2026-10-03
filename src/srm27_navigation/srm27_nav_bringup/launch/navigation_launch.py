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
    slam = LaunchConfiguration("slam")
    use_pcd_localization = LaunchConfiguration("use_pcd_localization")
    # "纯 LIO"模式: Point-LIO 提供里程计, 但没有 GICP 重定位。
    # 这套条件里的每一项在纯 LIO 下都必须按"有里程计"处理, 否则会出现
    # TF 不发(车在图上不动)或地面分割订阅到没人发布的话题(避障失效)。
    use_lio_odometry = LaunchConfiguration("use_lio_odometry")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    params_file = LaunchConfiguration("params_file")
    use_composition = LaunchConfiguration("use_composition")
    container_name = LaunchConfiguration("container_name")
    container_name_full = (namespace, "/", container_name)
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")

    lifecycle_nodes = [
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
        "velocity_smoother",
    ]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {"use_sim_time": use_sim_time, "autostart": autostart}

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

    declare_slam_cmd = DeclareLaunchArgument(
        "slam", default_value="False", description="Whether Point-LIO provides navigation odometry"
    )
    declare_use_pcd_localization_cmd = DeclareLaunchArgument(
        "use_pcd_localization", default_value="False",
        description="Whether prior-PCD Point-LIO localization provides navigation odometry",
    )
    declare_use_lio_odometry_cmd = DeclareLaunchArgument(
        "use_lio_odometry", default_value="False",
        description="Whether Point-LIO odometry (without GICP relocalization) provides navigation odometry",
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(
            bringup_dir, "config", "simulation", "nav2_params.yaml"
        ),
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

    start_terrain_analysis_cmd = Node(
        package="terrain_analysis",
        executable="terrainAnalysis",
        name="terrain_analysis",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[configured_params],
        remappings=[(
            "registered_scan",
            PythonExpression([
                "'velodyne_points' if not (", slam, " or ", use_pcd_localization, " or ", use_lio_odometry,
                ") else 'registered_scan'",
            ]),
        )],
    )

    start_terrain_analysis_ext_cmd = Node(
        package="terrain_analysis_ext",
        executable="terrainAnalysisExt",
        name="terrain_analysis_ext",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[configured_params],
        remappings=[(
            "registered_scan",
            PythonExpression([
                "'velodyne_points' if not (", slam, " or ", use_pcd_localization, " or ", use_lio_odometry,
                ") else 'registered_scan'",
            ]),
        )],
    )

    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(["not ", use_composition])),
        actions=[
            Node(
                condition=IfCondition(PythonExpression([slam, " or ", use_pcd_localization, " or ", use_lio_odometry])),
                package="loam_interface",
                executable="loam_interface_node",
                name="loam_interface",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="sensor_scan_generation",
                executable="sensor_scan_generation_node",
                name="sensor_scan_generation",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                arguments=["--ros-args", "--log-level", log_level],
                # In pure simulation odometry mode, Gazebo ground truth owns
                # odom->base_footprint.  Avoid a second broadcaster racing it;
                # retain this publisher for Point-LIO/PCD localization modes.
                parameters=[configured_params, {
                    "publish_tf": PythonExpression([
                        "'true' if (", slam, " or ", use_pcd_localization, " or ", use_lio_odometry,
                        ") else 'false'",
                    ])
                }],
                # Use the Point-LIO odometry topic when SLAM/PCD localization is active;
                # otherwise keep the derived scan odometry separate from ground truth.
                remappings=[(
                    "odometry",
                    PythonExpression([
                        "'odometry' if (", slam, " or ", use_pcd_localization, " or ", use_lio_odometry,
                        ") else 'sensor_odometry'",
                    ]),
                ), ("registered_scan", PythonExpression([
                    "'velodyne_points' if not (", slam, " or ", use_pcd_localization, " or ", use_lio_odometry,
                    ") else 'registered_scan'",
                ]))],
            ),
            Node(
                package="fake_vel_transform",
                executable="fake_vel_transform_node",
                name="fake_vel_transform",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=[("cmd_vel", "cmd_vel_controller")],
            ),
            Node(
                package="nav2_smoother",
                executable="smoother_server",
                name="smoother_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=[
                    ("cmd_vel", "cmd_vel_nav2_result"),  # remap output
                ],
            ),
            Node(
                package="nav2_waypoint_follower",
                executable="waypoint_follower",
                name="waypoint_follower",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="nav2_velocity_smoother",
                executable="velocity_smoother",
                name="velocity_smoother",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=[
                    ("cmd_vel", "cmd_vel_controller"),  # remap input
                    ("cmd_vel_smoothed", "cmd_vel_nav2_result"),  # remap output
                ],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
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
                package="sensor_scan_generation",
                plugin="sensor_scan_generation::SensorScanGenerationNode",
                name="sensor_scan_generation",
                parameters=[configured_params, {
                    "publish_tf": PythonExpression([
                        "'true' if (", slam, " or ", use_pcd_localization, " or ", use_lio_odometry,
                        ") else 'false'",
                    ])
                }],
                # Keep the composed path consistent with the non-composed path:
                # Point-LIO owns odometry for SLAM/PCD localization.
                remappings=[(
                    "odometry",
                    PythonExpression([
                        "'odometry' if (", slam, " or ", use_pcd_localization, " or ", use_lio_odometry,
                        ") else 'sensor_odometry'",
                    ]),
                ), ("registered_scan", PythonExpression([
                    "'velodyne_points' if not (", slam, " or ", use_pcd_localization, " or ", use_lio_odometry,
                    ") else 'registered_scan'",
                ]))],
            ),
            ComposableNode(
                package="fake_vel_transform",
                plugin="fake_vel_transform::FakeVelTransform",
                name="fake_vel_transform",
                parameters=[configured_params],
            ),
            ComposableNode(
                package="nav2_controller",
                plugin="nav2_controller::ControllerServer",
                name="controller_server",
                parameters=[configured_params],
                remappings=[("cmd_vel", "cmd_vel_controller")],
            ),
            ComposableNode(
                package="nav2_smoother",
                plugin="nav2_smoother::SmootherServer",
                name="smoother_server",
                parameters=[configured_params],
            ),
            ComposableNode(
                package="nav2_planner",
                plugin="nav2_planner::PlannerServer",
                name="planner_server",
                parameters=[configured_params],
            ),
            ComposableNode(
                package="nav2_behaviors",
                plugin="behavior_server::BehaviorServer",
                name="behavior_server",
                parameters=[configured_params],
                remappings=[
                    ("cmd_vel", "cmd_vel_nav2_result"),  # remap output
                ],
            ),
            ComposableNode(
                package="nav2_bt_navigator",
                plugin="nav2_bt_navigator::BtNavigator",
                name="bt_navigator",
                parameters=[configured_params],
            ),
            ComposableNode(
                package="nav2_waypoint_follower",
                plugin="nav2_waypoint_follower::WaypointFollower",
                name="waypoint_follower",
                parameters=[configured_params],
            ),
            ComposableNode(
                package="nav2_velocity_smoother",
                plugin="nav2_velocity_smoother::VelocitySmoother",
                name="velocity_smoother",
                parameters=[configured_params],
                remappings=[
                    ("cmd_vel", "cmd_vel_controller"),  # remap input
                    ("cmd_vel_smoothed", "cmd_vel_nav2_result"),  # remap output
                ],
            ),
            ComposableNode(
                package="nav2_lifecycle_manager",
                plugin="nav2_lifecycle_manager::LifecycleManager",
                name="lifecycle_manager_navigation",
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

    load_loam_interface_component = LoadComposableNodes(
        condition=IfCondition(PythonExpression([
            use_composition, " and (", slam, " or ", use_pcd_localization, " or ", use_lio_odometry, ")",
        ])),
        target_container=container_name_full,
        composable_node_descriptions=[
            ComposableNode(
                package="loam_interface",
                plugin="loam_interface::LoamInterfaceNode",
                name="loam_interface",
                parameters=[configured_params],
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
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_use_pcd_localization_cmd)
    ld.add_action(declare_use_lio_odometry_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    # Add the actions to launch all of the navigation nodes
    ld.add_action(start_terrain_analysis_cmd)
    ld.add_action(start_terrain_analysis_ext_cmd)
    ld.add_action(load_nodes)
    ld.add_action(load_composable_nodes)
    ld.add_action(load_loam_interface_component)

    return ld
