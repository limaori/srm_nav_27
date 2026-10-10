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
import sys

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import (
    IfCondition,
    LaunchConfigurationEquals,
    LaunchConfigurationNotEquals,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import ReplaceString, RewrittenYaml

# 速度限幅的合并模块装在 lib/<包名> 下（scripts/srm_speed_limits.py），
# 预检脚本与本 launch 共用同一份实现。
_SCRIPTS_DIR = os.path.join(
    get_package_prefix("srm27_nav_bringup"), "lib", "srm27_nav_bringup"
)
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from srm_speed_limits import AUTO, SpeedLimitParams  # noqa: E402


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory("srm27_nav_bringup")
    launch_dir = os.path.join(bringup_dir, "launch")

    # Create the launch configuration variables
    namespace = LaunchConfiguration("namespace")
    slam = LaunchConfiguration("slam")
    map_yaml_file = LaunchConfiguration("map")
    prior_pcd_file = LaunchConfiguration("prior_pcd_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    speed_limits_file = LaunchConfiguration("speed_limits_file")
    autostart = LaunchConfiguration("autostart")
    use_composition = LaunchConfiguration("use_composition")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")
    use_pcd_localization = LaunchConfiguration("use_pcd_localization")
    use_lio_odometry = LaunchConfiguration("use_lio_odometry")
    map_to_odom_x = LaunchConfiguration("map_to_odom_x")
    map_to_odom_y = LaunchConfiguration("map_to_odom_y")
    map_to_odom_yaw = LaunchConfiguration("map_to_odom_yaw")
    use_fake_vel_transform = LaunchConfiguration("use_fake_vel_transform")
    use_velocity_smoother = LaunchConfiguration("use_velocity_smoother")
    cmd_vel_nav_topic = LaunchConfiguration("cmd_vel_nav_topic")

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {"use_sim_time": use_sim_time, "yaml_filename": map_yaml_file}

    # 速度限幅（控制器 / 平滑器 / mux / 恢复行为）只写在 config/<mode>/speed_limits.yaml 里，
    # 这里在启动时把它合并进参数文件：合并后的临时文件才是各节点真正读的那份。
    # speed_limits_file 默认 "auto" = 参数文件同目录的 speed_limits.yaml（仅对
    # nav2_params_srm*.yaml 生效）；传 "" 可关闭合并（见 scripts/srm_speed_limits.py）。
    # ⚠ 必须放在 ReplaceString 之前：ReplaceString 会把参数文件先落到 /tmp 的临时文件上，
    #   那时"按文件名认 mode"的自动查找就失效了（文件名不再是 nav2_params_srm*.yaml）。
    params_file = SpeedLimitParams(params_file, speed_limits_file)

    # Only it applies when `namespace` is not empty.
    # '<robot_namespace>' keyword shall be replaced by 'namespace' launch argument
    # in config file 'nav2_multirobot_params.yaml' as a default & example.
    # User defined config file should contain '<robot_namespace>' keyword for the replacements.
    params_file = ReplaceString(
        source_file=params_file,
        replacements={"<robot_namespace>": ("")},
        condition=LaunchConfigurationEquals("namespace", ""),
    )

    params_file = ReplaceString(
        source_file=params_file,
        replacements={"<robot_namespace>": ("/", namespace)},
        condition=LaunchConfigurationNotEquals("namespace", ""),
    )

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
        "slam", default_value="False", description="Whether run a SLAM"
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map", description="Full path to map yaml file to load"
    )

    declare_prior_pcd_file_cmd = DeclareLaunchArgument(
        "prior_pcd_file", description="Full path to prior PCD file to load"
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(bringup_dir, "config", "simulation", "nav2_params.yaml"),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    declare_speed_limits_file_cmd = DeclareLaunchArgument(
        "speed_limits_file",
        default_value=AUTO,
        description=(
            "速度限幅覆盖层（config/<mode>/speed_limits.yaml）。auto = 与参数文件同目录下"
            "自动查找（只对 nav2_params_srm*.yaml 生效）；给绝对路径可显式指定。"
            "覆盖层的值优先，且参数文件里不得重复定义同一键。"
            "给空串可关闭合并，但那会退回插件默认值（Omni v_linear_max 默认 3.0 m/s，"
            "比档位高），只适合排查问题。"
        ),
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the nav2 stack",
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        "use_composition",
        default_value="True",
        description="Whether to use composed bringup",
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn if a node crashes. Applied when composition is disabled.",
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level", default_value="info", description="log level"
    )
    declare_use_pcd_localization_cmd = DeclareLaunchArgument("use_pcd_localization", default_value="False")
    declare_use_lio_odometry_cmd = DeclareLaunchArgument(
        "use_lio_odometry",
        default_value="False",
        description=(
            "Point-LIO provides odometry without GICP relocalization; "
            "map->odom stays static (requires the odometry chain, not the prior PCD)"
        ),
    )
    declare_map_to_odom_x_cmd = DeclareLaunchArgument("map_to_odom_x", default_value="0.0")
    declare_map_to_odom_y_cmd = DeclareLaunchArgument("map_to_odom_y", default_value="0.0")
    declare_map_to_odom_yaw_cmd = DeclareLaunchArgument("map_to_odom_yaw", default_value="0.0")
    declare_use_fake_vel_transform_cmd = DeclareLaunchArgument(
        "use_fake_vel_transform",
        default_value="True",
        description="是否启动 fake_vel_transform（旧步兵 gimbal_yaw_fake 链路）",
    )
    declare_use_velocity_smoother_cmd = DeclareLaunchArgument(
        "use_velocity_smoother",
        default_value="True",
        description="True: controller→velocity_smoother→cmd_vel_nav；False: controller 直接输出",
    )
    declare_cmd_vel_nav_topic_cmd = DeclareLaunchArgument(
        "cmd_vel_nav_topic",
        default_value="cmd_vel_nav2_result",
        description="导航链路最终平移速度出口话题（相对命名空间）",
    )

    # Specify the actions
    bringup_cmd_group = GroupAction(
        [
            PushRosNamespace(namespace=namespace),
            SetRemap("/tf", "tf"),
            SetRemap("/tf_static", "tf_static"),
            Node(
                condition=IfCondition(use_composition),
                name="nav2_container",
                package="rclcpp_components",
                executable="component_container_isolated",
                parameters=[configured_params, {"autostart": autostart}],
                arguments=["--ros-args", "--log-level", log_level],
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "nav2_slam_launch.py")
                ),
                condition=IfCondition(slam),
                launch_arguments={
                    "namespace": namespace,
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "use_respawn": use_respawn,
                    "params_file": params_file,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "localization_launch.py")
                ),
                condition=IfCondition(PythonExpression(["not ", slam])),
                launch_arguments={
                    "namespace": namespace,
                    "map": map_yaml_file,
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "params_file": params_file,
                    "prior_pcd_file": prior_pcd_file,
                    "use_composition": use_composition,
                    "use_respawn": use_respawn,
                    "container_name": "nav2_container",
                    "use_pcd_localization": use_pcd_localization,
                    "use_lio_odometry": use_lio_odometry,
                    "map_to_odom_x": map_to_odom_x,
                    "map_to_odom_y": map_to_odom_y,
                    "map_to_odom_yaw": map_to_odom_yaw,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "navigation_launch.py")
                ),
                launch_arguments={
                    "namespace": namespace,
                    "slam": slam,
                    "use_pcd_localization": use_pcd_localization,
                    "use_lio_odometry": use_lio_odometry,
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "params_file": params_file,
                    "use_composition": use_composition,
                    "use_respawn": use_respawn,
                    "container_name": "nav2_container",
                    "use_fake_vel_transform": use_fake_vel_transform,
                    "use_velocity_smoother": use_velocity_smoother,
                    "cmd_vel_nav_topic": cmd_vel_nav_topic,
                }.items(),
            ),
        ]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(colorized_output_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_prior_pcd_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_speed_limits_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_use_pcd_localization_cmd)
    ld.add_action(declare_use_lio_odometry_cmd)
    ld.add_action(declare_map_to_odom_x_cmd)
    ld.add_action(declare_map_to_odom_y_cmd)
    ld.add_action(declare_map_to_odom_yaw_cmd)
    ld.add_action(declare_use_fake_vel_transform_cmd)
    ld.add_action(declare_use_velocity_smoother_cmd)
    ld.add_action(declare_cmd_vel_nav_topic_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(bringup_cmd_group)

    return ld
