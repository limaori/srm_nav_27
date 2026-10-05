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

"""Nav2 导航栈（含点云、地形分析、定位适配）。

速度出口说明（实施方案 6.3）：

* 保留平滑器（use_velocity_smoother:=True）：
  controller → cmd_vel_controller → velocity_smoother → <cmd_vel_nav_topic>
* 跳过平滑器（use_velocity_smoother:=False）：
  controller 直接发布 <cmd_vel_nav_topic>

两种配置互斥。cmd_vel_nav_topic 是导航链路的最终平移速度出口，SRM 仿真入口把它
接到 srm_cmd_mux 的导航输入，由 mux 丢弃角速度并合成 cmd_vel_sim。

fake_vel_transform 已变为可选（use_fake_vel_transform，默认 True 以兼容旧配置）。
SRM 仿真使用真实 base_link，不再需要 gimbal_yaw_fake，因此显式关闭它。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


def _as_bool(value):
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def launch_setup(context):
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
    use_fake_vel_transform = LaunchConfiguration("use_fake_vel_transform")
    use_velocity_smoother = LaunchConfiguration("use_velocity_smoother")
    cmd_vel_nav_topic = LaunchConfiguration("cmd_vel_nav_topic")

    # 平滑器与"跳过平滑器"两条分支互斥：跳过时控制器直接写导航速度出口。
    controller_output_topic = PythonExpression([
        "'cmd_vel_controller' if ",
        use_velocity_smoother,
        " else '",
        cmd_vel_nav_topic,
        "'",
    ])

    lifecycle_nodes = [
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
    ]
    if _as_bool(use_velocity_smoother.perform(context)):
        lifecycle_nodes.append("velocity_smoother")

    # 使用 Point-LIO / GICP / SLAM 时由这些节点提供里程计。
    has_odometry_source = PythonExpression([
        slam, " or ", use_pcd_localization, " or ", use_lio_odometry,
    ])

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
                "'velodyne_points' if not (", has_odometry_source,
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
                "'velodyne_points' if not (", has_odometry_source,
                ") else 'registered_scan'",
            ]),
        )],
    )

    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(["not ", use_composition])),
        actions=[
            Node(
                condition=IfCondition(has_odometry_source),
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
                # odom->base_link.  Avoid a second broadcaster racing it;
                # retain this publisher for Point-LIO/PCD localization modes.
                parameters=[configured_params, {
                    "publish_tf": PythonExpression([
                        "'true' if (", has_odometry_source, ") else 'false'",
                    ])
                }],
                # Use the Point-LIO odometry topic when SLAM/PCD localization is active;
                # otherwise keep the derived scan odometry separate from ground truth.
                remappings=[(
                    "odometry",
                    PythonExpression([
                        "'odometry' if (", has_odometry_source,
                        ") else 'sensor_odometry'",
                    ]),
                ), ("registered_scan", PythonExpression([
                    "'velodyne_points' if not (", has_odometry_source,
                    ") else 'registered_scan'",
                ]))],
            ),
            # 旧步兵链路用 fake_vel_transform 把 gimbal_yaw 伪装成 gimbal_yaw_fake。
            # SRM 使用真实 base_link，此节点默认关闭，仅为兼容旧配置保留。
            Node(
                condition=IfCondition(use_fake_vel_transform),
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
                remappings=[("cmd_vel", controller_output_topic)],
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
                # 恢复行为同样输出到导航速度出口；其中的角速度分量由 mux 丢弃。
                remappings=[("cmd_vel", cmd_vel_nav_topic)],
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
                remappings=[("cmd_vel", cmd_vel_nav_topic)],
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
                condition=IfCondition(use_velocity_smoother),
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
                    ("cmd_vel_smoothed", cmd_vel_nav_topic),  # remap output
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

    composable_node_descriptions = [
        ComposableNode(
            package="sensor_scan_generation",
            plugin="sensor_scan_generation::SensorScanGenerationNode",
            name="sensor_scan_generation",
            parameters=[configured_params, {
                "publish_tf": PythonExpression([
                    "'true' if (", has_odometry_source, ") else 'false'",
                ])
            }],
            # Keep the composed path consistent with the non-composed path:
            # Point-LIO owns odometry for SLAM/PCD localization.
            remappings=[(
                "odometry",
                PythonExpression([
                    "'odometry' if (", has_odometry_source,
                    ") else 'sensor_odometry'",
                ]),
            ), ("registered_scan", PythonExpression([
                "'velodyne_points' if not (", has_odometry_source,
                ") else 'registered_scan'",
            ]))],
        ),
        ComposableNode(
            package="nav2_controller",
            plugin="nav2_controller::ControllerServer",
            name="controller_server",
            parameters=[configured_params],
            remappings=[("cmd_vel", controller_output_topic)],
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
            remappings=[("cmd_vel", cmd_vel_nav_topic)],
        ),
        ComposableNode(
            package="nav2_bt_navigator",
            plugin="nav2_bt_navigator::BtNavigator",
            name="bt_navigator",
            parameters=[configured_params],
            remappings=[("cmd_vel", cmd_vel_nav_topic)],
        ),
        ComposableNode(
            package="nav2_waypoint_follower",
            plugin="nav2_waypoint_follower::WaypointFollower",
            name="waypoint_follower",
            parameters=[configured_params],
        ),
    ]
    # 平滑器与"跳过平滑器"两条分支互斥，只在启用时才加载组件。
    if _as_bool(use_velocity_smoother.perform(context)):
        composable_node_descriptions.append(
            ComposableNode(
                package="nav2_velocity_smoother",
                plugin="nav2_velocity_smoother::VelocitySmoother",
                name="velocity_smoother",
                parameters=[configured_params],
                remappings=[
                    ("cmd_vel", "cmd_vel_controller"),  # remap input
                    ("cmd_vel_smoothed", cmd_vel_nav_topic),  # remap output
                ],
            )
        )
    composable_node_descriptions.append(
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
        )
    )

    load_composable_nodes = LoadComposableNodes(
        condition=IfCondition(use_composition),
        target_container=container_name_full,
        composable_node_descriptions=composable_node_descriptions,
    )

    load_fake_vel_transform_component = LoadComposableNodes(
        condition=IfCondition(PythonExpression([
            use_composition, " and ", use_fake_vel_transform,
        ])),
        target_container=container_name_full,
        composable_node_descriptions=[
            ComposableNode(
                package="fake_vel_transform",
                plugin="fake_vel_transform::FakeVelTransform",
                name="fake_vel_transform",
                parameters=[configured_params],
            )
        ],
    )

    load_loam_interface_component = LoadComposableNodes(
        condition=IfCondition(PythonExpression([
            use_composition, " and (", has_odometry_source, ")",
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

    return [
        start_terrain_analysis_cmd,
        start_terrain_analysis_ext_cmd,
        load_nodes,
        load_composable_nodes,
        load_fake_vel_transform_component,
        load_loam_interface_component,
    ]


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory("srm27_nav_bringup")

    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(
        SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1")
    )
    ld.add_action(SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"))

    # Declare the launch options
    ld.add_action(
        DeclareLaunchArgument(
            "namespace", default_value="", description="Top-level namespace"
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "slam",
            default_value="False",
            description="Whether Point-LIO provides navigation odometry",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "use_pcd_localization",
            default_value="False",
            description="Whether prior-PCD Point-LIO localization provides navigation odometry",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "use_lio_odometry",
            default_value="False",
            description="Whether Point-LIO odometry (without GICP relocalization) provides navigation odometry",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation (Gazebo) clock if true",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(
                bringup_dir, "config", "simulation", "nav2_params.yaml"
            ),
            description="Full path to the ROS2 parameters file to use for all launched nodes",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "autostart",
            default_value="true",
            description="Automatically startup the nav2 stack",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "use_composition",
            default_value="False",
            description="Use composed bringup if True",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "container_name",
            default_value="nav2_container",
            description="the name of container that nodes will load in if use composition",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "use_respawn",
            default_value="False",
            description="Whether to respawn if a node crashes. Applied when composition is disabled.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "log_level", default_value="info", description="log level"
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "use_fake_vel_transform",
            default_value="True",
            description="是否启动 fake_vel_transform（旧步兵的 gimbal_yaw_fake 链路）",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "use_velocity_smoother",
            default_value="True",
            description="True: controller→velocity_smoother→cmd_vel_nav；False: controller 直接输出",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "cmd_vel_nav_topic",
            default_value="cmd_vel_nav2_result",
            description="导航链路最终平移速度出口话题（相对命名空间）",
        )
    )

    # Add the actions to launch all of the navigation nodes
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
