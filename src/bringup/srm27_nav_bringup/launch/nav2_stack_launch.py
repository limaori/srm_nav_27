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

import copy
import os
import sys

import yaml

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
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
    get_package_prefix("srm27_nav_tools"), "lib", "srm27_nav_tools"
)
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from srm_speed_limits import AUTO, SpeedLimitParams  # noqa: E402


class MincoPlannerParams(RewrittenYaml):
    """Bind the planner and controller to the same active path in topic mode."""

    def perform(self, context):
        path = super().perform(context)
        with open(path, encoding="utf-8") as stream:
            config = yaml.safe_load(stream)
        follow = config.get("controller_server", {}).get("ros__parameters", {}).get("FollowPath", {})
        if follow.get("trajectory_source") != "topic":
            return path

        planner = config.get("planner_server", {}).get("ros__parameters", {})
        grid = planner.get("GridBased", {})
        yastar_plugin = "srm27_nav_planner::YAstarGlobalPlanner"
        minco_plugin = "srm27_minco_planner::MincoPlanner"
        if (planner.get("planner_plugins") != ["GridBased"]
                or grid.get("plugin") not in (yastar_plugin, minco_plugin)
                or follow.get("plugin") != "srm27_minco_controller::MincoMpcController"):
            raise RuntimeError("MINCO topic mode requires GridBased YAstar/MINCO and FollowPath MINCO/MPC")

        if grid["plugin"] == yastar_plugin:
            planner["yastar"] = grid
            grid = dict(planner.pop("MincoPlanner", {}))
            planner["GridBased"] = grid
            grid["plugin"] = minco_plugin
            grid["yastar_plugin_name"] = "yastar"

        # Copy the effective settings. Shared YAML objects would emit aliases that rcl rejects.
        for key in (
            "base_frame", "odom_topic", "planning_horizon", "path_centering_max_displacement",
            "terminal_reached_radius", "goal_change_tolerance", "state_timeout",
            "limits", "safety", "minco", "validator",
        ):
            if key in follow:
                grid[key] = copy.deepcopy(follow[key])
        grid["state_timeout"] = max(0.3, grid.get("state_timeout", 0.5))
        grid["opt_frequency"] = follow.get("replan_frequency", 10.0)
        grid["output_frame"] = follow.get("planning_frame", "odom")
        grid["follow_path_topic"] = follow.get("path_request_topic", "planning/follow_path")
        grid["trajectory_topic"] = follow.get("trajectory_topic", "planning/minco_trajectory")
        if not grid["follow_path_topic"] or not grid["trajectory_topic"]:
            raise RuntimeError("MINCO topic mode requires nonempty path and trajectory topics")

        with open(path, "w", encoding="utf-8") as stream:
            yaml.safe_dump(config, stream)
        return path


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
    # 里程计前端（方案 §6.2）：point_lio（默认 = 旧行为）或 zlio。
    # 这里只透传，真正的分叉在 localization_launch（起哪个前端 + 要不要 zlio_bridge）
    # 与 navigation_launch（要不要 loam_interface）里。
    odometry_frontend = LaunchConfiguration("odometry_frontend")
    # ZLIO 的算法配置文件：仿真与实车**不是同一份**（IMU 单位 m/s² vs g、点云点格式
    # PointCloud2 vs LiDAR CustomMsg、雷达-IMU 外参也不同）。这里只透传，默认值保持
    # 仿真那份以免改变旧行为；实车入口（nav_srm_real_launch.py）显式传 config/real/ 那份。
    zlio_config = LaunchConfiguration("zlio_config")
    # ZLIO 输入接线（话题/单位/帧名）：仿真与实车不同，入口按平台给值，这里只透传。
    zlio_input_type = LaunchConfiguration("zlio_input_type")
    zlio_lidar_topic = LaunchConfiguration("zlio_lidar_topic")
    zlio_imu_topic = LaunchConfiguration("zlio_imu_topic")
    zlio_point_time_field = LaunchConfiguration("zlio_point_time_field")
    zlio_point_time_scale = LaunchConfiguration("zlio_point_time_scale")
    zlio_point_time_absolute = LaunchConfiguration("zlio_point_time_absolute")
    zlio_imu_acceleration_scale = LaunchConfiguration("zlio_imu_acceleration_scale")
    zlio_body_frame = LaunchConfiguration("zlio_body_frame")
    map_to_odom_x = LaunchConfiguration("map_to_odom_x")
    map_to_odom_y = LaunchConfiguration("map_to_odom_y")
    map_to_odom_yaw = LaunchConfiguration("map_to_odom_yaw")
    # Z-Localization backend 实验模式（默认关闭），只透传；真正接线在 localization_launch。
    use_zlocalization_backend = LaunchConfiguration("use_zlocalization_backend")
    zlocalization_backend_config = LaunchConfiguration("zlocalization_backend_config")
    zlocalization_prior_enabled = LaunchConfiguration("zlocalization_prior_enabled")
    zlocalization_prior_initialization_only = LaunchConfiguration(
        "zlocalization_prior_initialization_only")
    zlocalization_loop_enabled = LaunchConfiguration("zlocalization_loop_enabled")
    zlocalization_uwb_enabled = LaunchConfiguration("zlocalization_uwb_enabled")
    zlocalization_prior_map = LaunchConfiguration("zlocalization_prior_map")
    zlocalization_trajectory_directory = LaunchConfiguration("zlocalization_trajectory_directory")
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
    params_file = MincoPlannerParams(
        source_file=SpeedLimitParams(params_file, speed_limits_file), param_rewrites={}
    )

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
    declare_odometry_frontend_cmd = DeclareLaunchArgument(
        "odometry_frontend",
        default_value="point_lio",
        description=(
            "里程计前端: point_lio (默认, 旧行为) 或 zlio。"
            "zlio 需要 use_lio_odometry:=True，且会同时起 zlio_bridge；"
            "回退方式就是改回 point_lio（方案 §9 的三级回退之一）"
        ),
    )
    declare_zlio_config_cmd = DeclareLaunchArgument(
        "zlio_config",
        default_value=os.path.join(
            bringup_dir, "config", "simulation", "zlio_mid360.yaml"
        ),
        description=(
            "ZLIO 前端算法配置（IMU 单位/点云格式/雷达-IMU 外参随车不同）："
            "仿真用 config/simulation，实车用 config/real。只在 odometry_frontend:=zlio 时生效"
        ),
    )
    # ZLIO 输入接线：默认值 = 仿真。实车入口（nav_srm_real_launch.py）会覆盖。
    declare_zlio_input_type_cmd = DeclareLaunchArgument(
        "zlio_input_type", default_value="pointcloud2",
        description="LiDAR 消息格式: pointcloud2（仿真）或 livox（实车 CustomMsg）",
    )
    declare_zlio_lidar_topic_cmd = DeclareLaunchArgument(
        "zlio_lidar_topic", default_value="velodyne_points",
        description="LiDAR 输入话题（仿真 velodyne_points / 实车 livox/lidar）",
    )
    declare_zlio_imu_topic_cmd = DeclareLaunchArgument(
        "zlio_imu_topic", default_value="livox/imu_saturated",
        description="IMU 输入话题（默认取饱和保护的输出）",
    )
    declare_zlio_point_time_field_cmd = DeclareLaunchArgument(
        "zlio_point_time_field", default_value="time",
        description="PointCloud2 逐点时间字段名（livox 输入不使用）",
    )
    declare_zlio_point_time_scale_cmd = DeclareLaunchArgument(
        "zlio_point_time_scale", default_value="1.0",
        description="逐点时间换算成秒的倍率（纳秒 1e-9 / 秒 1.0）",
    )
    declare_zlio_point_time_absolute_cmd = DeclareLaunchArgument(
        "zlio_point_time_absolute", default_value="False",
        description="逐点时间是否为绝对时间",
    )
    declare_zlio_imu_acceleration_scale_cmd = DeclareLaunchArgument(
        "zlio_imu_acceleration_scale", default_value="1.0",
        description="IMU 加速度单位换算：g -> 9.81（实车），m/s² -> 1.0（仿真）",
    )
    declare_zlio_body_frame_cmd = DeclareLaunchArgument(
        "zlio_body_frame", default_value="front_mid360",
        description="里程计 child frame（仿真 front_mid360 / 实车 livox_imu）",
    )
    declare_map_to_odom_x_cmd = DeclareLaunchArgument("map_to_odom_x", default_value="0.0")
    declare_map_to_odom_y_cmd = DeclareLaunchArgument("map_to_odom_y", default_value="0.0")
    declare_map_to_odom_yaw_cmd = DeclareLaunchArgument("map_to_odom_yaw", default_value="0.0")
    declare_use_zlocalization_backend_cmd = DeclareLaunchArgument(
        "use_zlocalization_backend",
        default_value="False",
        description="用 srm27_zlocalization_backend 发布 map->odom（实验模式，默认关闭）",
    )
    declare_zlocalization_backend_config_cmd = DeclareLaunchArgument(
        "zlocalization_backend_config", default_value="",
        description="backend 算法配置绝对路径；空 = 由 localization_launch 用包内默认",
    )
    declare_zlocalization_prior_enabled_cmd = DeclareLaunchArgument(
        "zlocalization_prior_enabled", default_value="False",
        description="启用先验配准，独立开关，默认关闭",
    )
    declare_zlocalization_prior_initialization_only_cmd = DeclareLaunchArgument(
        "zlocalization_prior_initialization_only", default_value="False",
        description="先验只在初始化时匹配一次，之后定位全走前端里程计",
    )
    declare_zlocalization_loop_enabled_cmd = DeclareLaunchArgument(
        "zlocalization_loop_enabled", default_value="False",
        description="启用运行轨迹回环，独立开关，默认关闭",
    )
    declare_zlocalization_uwb_enabled_cmd = DeclareLaunchArgument(
        "zlocalization_uwb_enabled", default_value="False",
        description="启用UWB 融合，独立开关，默认关闭",
    )
    declare_zlocalization_prior_map_cmd = DeclareLaunchArgument(
        "zlocalization_prior_map", default_value="",
        description="先验地图包绝对目录（keyframes.txt + clouds/ + 描述子）",
    )
    declare_zlocalization_trajectory_directory_cmd = DeclareLaunchArgument(
        "zlocalization_trajectory_directory", default_value="",
        description="backend 轨迹地图包绝对目录",
    )
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
                    "odometry_frontend": odometry_frontend,
                    "zlio_config": zlio_config,
                    "zlio_input_type": zlio_input_type,
                    "zlio_lidar_topic": zlio_lidar_topic,
                    "zlio_imu_topic": zlio_imu_topic,
                    "zlio_point_time_field": zlio_point_time_field,
                    "zlio_point_time_scale": zlio_point_time_scale,
                    "zlio_point_time_absolute": zlio_point_time_absolute,
                    "zlio_imu_acceleration_scale": zlio_imu_acceleration_scale,
                    "zlio_body_frame": zlio_body_frame,
                    "map_to_odom_x": map_to_odom_x,
                    "map_to_odom_y": map_to_odom_y,
                    "map_to_odom_yaw": map_to_odom_yaw,
                    "use_zlocalization_backend": use_zlocalization_backend,
                    "zlocalization_backend_config": zlocalization_backend_config,
                    "zlocalization_prior_enabled": zlocalization_prior_enabled,
                    "zlocalization_prior_initialization_only":
                        zlocalization_prior_initialization_only,
                    "zlocalization_loop_enabled": zlocalization_loop_enabled,
                    "zlocalization_uwb_enabled": zlocalization_uwb_enabled,
                    "zlocalization_prior_map": zlocalization_prior_map,
                    "zlocalization_trajectory_directory": zlocalization_trajectory_directory,
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
                    "odometry_frontend": odometry_frontend,
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
    ld.add_action(declare_odometry_frontend_cmd)
    ld.add_action(declare_zlio_config_cmd)
    ld.add_action(declare_zlio_input_type_cmd)
    ld.add_action(declare_zlio_lidar_topic_cmd)
    ld.add_action(declare_zlio_imu_topic_cmd)
    ld.add_action(declare_zlio_point_time_field_cmd)
    ld.add_action(declare_zlio_point_time_scale_cmd)
    ld.add_action(declare_zlio_point_time_absolute_cmd)
    ld.add_action(declare_zlio_imu_acceleration_scale_cmd)
    ld.add_action(declare_zlio_body_frame_cmd)
    ld.add_action(declare_map_to_odom_x_cmd)
    ld.add_action(declare_map_to_odom_y_cmd)
    ld.add_action(declare_map_to_odom_yaw_cmd)
    ld.add_action(declare_use_zlocalization_backend_cmd)
    ld.add_action(declare_zlocalization_backend_config_cmd)
    ld.add_action(declare_zlocalization_prior_enabled_cmd)
    ld.add_action(declare_zlocalization_prior_initialization_only_cmd)
    ld.add_action(declare_zlocalization_loop_enabled_cmd)
    ld.add_action(declare_zlocalization_uwb_enabled_cmd)
    ld.add_action(declare_zlocalization_prior_map_cmd)
    ld.add_action(declare_zlocalization_trajectory_directory_cmd)
    ld.add_action(declare_use_fake_vel_transform_cmd)
    ld.add_action(declare_use_velocity_smoother_cmd)
    ld.add_action(declare_cmd_vel_nav_topic_cmd)

    # 里程计前端的取值校验：拼错一个字（比如 zlio_frontend:=zlIO）会让两条条件同时不成立，
    # 结果既没有 loam_interface 也没有 zlio_bridge，odom->base_link 静默消失。
    # 这里直接让 launch 失败，比"车不会动"好排查得多。
    def _validate_odometry_frontend(context):
        frontend = str(
            context.launch_configurations.get("odometry_frontend", "point_lio")
        ).strip()
        if frontend not in ("point_lio", "zlio"):
            raise RuntimeError(
                f"odometry_frontend 只支持 point_lio / zlio，收到 '{frontend}'"
            )
        if frontend != "zlio":
            return []
        truthy = ("1", "true", "yes", "on")
        lio = str(
            context.launch_configurations.get("use_lio_odometry", "False")
        ).strip().lower()
        if lio not in truthy:
            raise RuntimeError(
                "odometry_frontend:=zlio 需要 use_lio_odometry:=True："
                "ZLIO 只在纯 LIO 链路（--lio）启用；--gicp 支持见方案 §10（尚未接入）"
            )
        slam = str(
            context.launch_configurations.get("slam", "False")
        ).strip().lower()
        backend = str(
            context.launch_configurations.get("use_zlocalization_backend", "False")
        ).strip().lower()
        if backend in truthy and slam in truthy:
            raise RuntimeError(
                "use_zlocalization_backend:=True 与 slam:=True 互斥："
                "slam 链路走 nav2_slam_launch.py，由 slam_toolbox 发 map->odom，"
                "backend 模式不经过 localization_launch（同上：不能有两个 map->odom 所有者）"
            )
        if slam in truthy:
            raise RuntimeError(
                "odometry_frontend:=zlio 与 slam:=True 暂不兼容："
                "slam 链路走 nav2_slam_launch.py，不经过 localization_launch，"
                "zlio/zlio_bridge 不会被启动（方案 §10）。"
                "建图请显式加 odometry_frontend:=point_lio（sim_slam_launch.py 已经这么做）"
            )
        return []

    # Add the actions to launch all of the navigation nodes
    ld.add_action(OpaqueFunction(function=_validate_odometry_frontend))
    ld.add_action(bringup_cmd_group)

    return ld
